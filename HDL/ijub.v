// ============================================================
//  IJUB Architecture — 12-bit Accumulator Processor
//  16 opcodes | 8-bit accumulator | 16-slot I/D SRAM
//  Target: Intel Cyclone IV EP4CE6
// ============================================================

module ijub (
    input  [11:0] input_block,   // DIP switches [11:0]  — instruction input
    output reg [15:0] output_block,
    input  mode,                 // DIP switch [12]      — 0=program, 1=run
    input  speed,                // DIP switch [13]      — 0=50MHz,   1=1Hz
    input  save,                 // DIP switch [14]      — write to I_SRAM
    input  rst,                  // DIP switch [15]      — reset processor
    input  browse_next,          // FPGA button          — scroll forward
    input  browse_prev,          // FPGA button          — scroll backward
    input  clk                   // 50MHz board clock
);

    // ── Clock Enable: generates a 1-cycle pulse at 1Hz ───────────────
    // Instead of dividing the clock (which causes multiple-driver errors),
    // we generate a single-cycle enable pulse every 50,000,000 cycles.
    // Run mode only executes when speed=0 (every cycle) or run_enable=1 (1Hz).
    reg [25:0] div_counter;
    reg        run_enable;

    always @(posedge clk) begin
        if (div_counter >= 26'd49_999_999) begin
            div_counter <= 26'd0;
            run_enable  <= 1'b1;   // pulse HIGH for exactly one cycle
        end else begin
            div_counter <= div_counter + 26'd1;
            run_enable  <= 1'b0;
        end
    end

    // ── Switch/Button Conditioning ───────────────────────────────────
    // DIP switches are mechanical and bounce rapidly when flipped.
    // Each switch goes through: debouncer → edge_detector → single pulse.
    wire clean_save, pulse_save;
    wire clean_rst,  pulse_rst;
    wire clean_next, pulse_next;
    wire clean_prev, pulse_prev;

    debouncer #(.STABLE_COUNT(500000)) u_deb_save (
        .clk(clk), .noisy(save),        .clean(clean_save)
    );
    edge_detector u_edg_save (
        .clk(clk), .sw(clean_save),     .btn_pulse(pulse_save)
    );

    debouncer #(.STABLE_COUNT(500000)) u_deb_rst (
        .clk(clk), .noisy(rst),         .clean(clean_rst)
    );
    edge_detector u_edg_rst (
        .clk(clk), .sw(clean_rst),      .btn_pulse(pulse_rst)
    );

    debouncer #(.STABLE_COUNT(500000)) u_deb_next (
        .clk(clk), .noisy(browse_next), .clean(clean_next)
    );
    edge_detector u_edg_next (
        .clk(clk), .sw(clean_next),     .btn_pulse(pulse_next)
    );

    debouncer #(.STABLE_COUNT(500000)) u_deb_prev (
        .clk(clk), .noisy(browse_prev), .clean(clean_prev)
    );
    edge_detector u_edg_prev (
        .clk(clk), .sw(clean_prev),     .btn_pulse(pulse_prev)
    );

    // ── Memory ───────────────────────────────────────────────────────
    reg [11:0] I_SRAM [15:0];   // instruction memory: 16 x 12-bit
    reg [7:0]  D_SRAM [15:0];   // data memory:        16 x 8-bit

    // ── Registers ────────────────────────────────────────────────────
    reg [7:0] w_reg;            // accumulator — holds current working value
    reg [3:0] pc;               // program counter — points to current instruction
    reg [3:0] prog_address;     // write pointer used in program mode
    reg [3:0] browse_address;   // read pointer for browsing I_SRAM slots
    reg       halted;           // set by HLT instruction; cleared by reset

    // ── Flags ────────────────────────────────────────────────────────
    reg zf;   // zero flag      — set by CMP when w_reg == operand
    reg lt;   // less-than flag — set by CMP when w_reg  < operand
    reg gt;   // greater-than   — set by CMP when w_reg  > operand
    reg cf;   // carry flag     — set by ADD when result exceeds 255
    reg bf;   // borrow flag    — set by SUB when result goes below 0

    // ── Instruction Fetch (combinational) ────────────────────────────
    // These are wires, not registers — they update instantly based on pc.
    wire [11:0] current_instr   = I_SRAM[pc];
    wire [3:0]  current_op      = current_instr[11:8];
    wire [7:0]  current_operand = current_instr[7:0];

    // ── ALU: 9-bit to capture overflow/underflow in bit [8] ──────────
    wire [8:0] add_result = {1'b0, w_reg} + {1'b0, current_operand};
    wire [8:0] sub_result = {1'b0, w_reg} - {1'b0, current_operand};

    // ── Unified Core Logic ───────────────────────────────────────────
    // Everything lives in ONE always block on the base clock.
    // This avoids the multiple-driver error that occurs when two always
    // blocks on different clocks both write to the same register.
    always @(posedge clk) begin

        // ── RESET (highest priority, edge-triggered) ─────────────────
        // pulse_rst fires for exactly one cycle when SW[15] is flipped ON.
        // The processor resets instantly and resumes operation immediately after.
        if (pulse_rst) begin
            prog_address   <= 4'd0;
            browse_address <= 4'd0;
            pc             <= 4'd0;
            w_reg          <= 8'd0;
            halted         <= 1'b0;
            zf <= 1'b0; lt <= 1'b0; gt <= 1'b0;
            cf <= 1'b0; bf <= 1'b0;
            output_block   <= 16'd0;

        // ── PROGRAM MODE ─────────────────────────────────────────────
        end else if (!mode) begin
            pc <= 4'd0; // hold PC at 0 — nothing executes while programming

            // Save: write input_block into I_SRAM at current write pointer
            if (pulse_save) begin
                I_SRAM[prog_address] <= input_block;
                prog_address         <= prog_address + 4'd1;
                browse_address       <= prog_address + 4'd1; // auto-follow write pointer
            end

            // Browse: scroll through I_SRAM slots to verify stored instructions
            if (pulse_next) browse_address <= browse_address + 4'd1;
            if (pulse_prev) browse_address <= browse_address - 4'd1;

            // LEDs always show current browse slot: [15:4]=instruction, [3:0]=address
            output_block <= {I_SRAM[browse_address], browse_address};

        // ── RUN MODE ─────────────────────────────────────────────────
        // Executes every cycle at 50MHz, or only on run_enable ticks at 1Hz.
        end else if ((speed == 1'b0 || run_enable) && !halted) begin

            case (current_op)

                4'd0: begin
                    // HLT — halt the processor
                    // Only a reset (SW[15]) can resume execution after this.
                    halted <= 1'b1;
                    pc     <= pc;       // freeze program counter
                end

                4'd1: begin
                    // LDI — load immediate value into accumulator
                    w_reg <= current_operand;
                    pc    <= pc + 4'd1;
                end

                4'd2: begin
                    // LD — load from data memory into accumulator
                    w_reg <= D_SRAM[current_operand[3:0]];
                    pc    <= pc + 4'd1;
                end

                4'd3: begin
                    // ST — store accumulator into data memory
                    D_SRAM[current_operand[3:0]] <= w_reg;
                    pc    <= pc + 4'd1;
                end

                4'd4: begin
                    // CMP — compare accumulator with operand, set flags
                    // Exactly one of zf/lt/gt will be set; the others cleared.
                    if (w_reg == current_operand) begin
                        zf <= 1'b1; lt <= 1'b0; gt <= 1'b0;
                    end else if (w_reg < current_operand) begin
                        zf <= 1'b0; lt <= 1'b1; gt <= 1'b0;
                    end else begin
                        zf <= 1'b0; lt <= 1'b0; gt <= 1'b1;
                    end
                    pc <= pc + 4'd1;
                end

                4'd5: begin
                    // JMP — unconditional jump to target address
                    pc <= current_operand[3:0];
                end

                4'd6: begin
                    // SKEQ — skip next instruction if zero flag is set
                    // Use after CMP to skip when values were equal.
                    pc <= zf ? pc + 4'd2 : pc + 4'd1;
                end

                4'd7: begin
                    // SKLT — skip next instruction if less-than flag is set
                    // Use after CMP to skip when accumulator was less than operand.
                    pc <= lt ? pc + 4'd2 : pc + 4'd1;
                end

                4'd8: begin
                    // SKGT — skip next instruction if greater-than flag is set
                    // Use after CMP to skip when accumulator was greater than operand.
                    pc <= gt ? pc + 4'd2 : pc + 4'd1;
                end

                4'd9: begin
                    // ADD — add operand to accumulator
                    // Bit [8] of add_result captures overflow into carry flag.
                    w_reg <= add_result[7:0];
                    cf    <= add_result[8];
                    pc    <= pc + 4'd1;
                end

                4'd10: begin
                    // SUB — subtract operand from accumulator
                    // Bit [8] of sub_result captures underflow into borrow flag.
                    w_reg <= sub_result[7:0];
                    bf    <= sub_result[8];
                    pc    <= pc + 4'd1;
                end

                4'd11: begin
                    // AND — bitwise AND
                    w_reg <= w_reg & current_operand;
                    pc    <= pc + 4'd1;
                end

                4'd12: begin
                    // OR — bitwise OR
                    w_reg <= w_reg | current_operand;
                    pc    <= pc + 4'd1;
                end

                4'd13: begin
                    // NOT — bitwise NOT (operand is ignored)
                    w_reg <= ~w_reg;
                    pc    <= pc + 4'd1;
                end

                4'd14: begin
                    // XOR — bitwise XOR
                    w_reg <= w_reg ^ current_operand;
                    pc    <= pc + 4'd1;
                end

                4'd15: begin
                    // OUT — send flags and accumulator to output LEDs
                    // Format: [15]=cf [14]=bf [13]=gt [12]=lt [11]=zf [10:8]=000 [7:0]=w_reg
                    output_block <= {cf, bf, gt, lt, zf, 3'b000, w_reg};
                    pc           <= pc + 4'd1;
                end

                default: begin
                    // Unknown opcode — safely advance PC to avoid lockup
                    pc <= pc + 4'd1;
                end

            endcase
        end
    end

endmodule
