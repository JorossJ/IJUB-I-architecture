// ============================================================
//  IJUB Architecture — 12-bit Accumulator Processor
//  16 opcodes | 8-bit accumulator | 16-slot I/D SRAM
//  Target: Intel Cyclone IV EP4CE6
// ============================================================
//
//  OUTPUT_BLOCK LED FORMAT (run mode):
//  [15:12] = current_op  — opcode of the instruction executing
//  [11:8]  = pc          — address of that instruction in I_SRAM
//  [7:0]   = w_reg       — accumulator value (updated by OUT)
//
//  OUTPUT_BLOCK LED FORMAT (program mode):
//  [15:4]  = instruction at browse slot
//  [3:0]   = browse slot address
// ============================================================

module ijub(
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

    // ── Clock Enable: 50MHz → 1Hz pulse ─────────────────────────────
    // We generate a single-cycle enable pulse every 50,000,000 cycles
    // rather than dividing the clock. Using a divided clock as an actual
    // clock signal causes multiple-driver errors in synthesis because two
    // always blocks would end up driving the same registers.
    reg [25:0] div_counter;
    reg        run_enable;

    always @(posedge clk) begin
        if (div_counter >= 26'd49_999_999) begin
            div_counter <= 26'd0;
            run_enable  <= 1'b1;   // HIGH for exactly one clock cycle
        end else begin
            div_counter <= div_counter + 26'd1;
            run_enable  <= 1'b0;
        end
    end

    // ── Switch/Button Conditioning ───────────────────────────────────
    // Every DIP switch goes through:
    //   debouncer   — waits 10ms for signal to settle (eliminates bounce)
    //   edge_detector — converts the stable level into a 1-cycle pulse
    // This means each flip of a switch fires exactly one action.
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
    reg [7:0] w_reg;            // accumulator — the main working register
    reg [3:0] pc;               // program counter — points to current instruction
    reg [3:0] prog_address;     // write pointer used in program mode
    reg [3:0] browse_address;   // read pointer for browsing I_SRAM in program mode
    reg       halted;           // set by HLT; cleared only by reset

    // ── Flags ────────────────────────────────────────────────────────
    reg zf;   // zero flag      — set by CMP when w_reg == operand
    reg lt;   // less-than flag — set by CMP when w_reg  < operand
    reg gt;   // greater-than   — set by CMP when w_reg  > operand
    reg cf;   // carry flag     — set by ADD when result exceeds 255
    reg bf;   // borrow flag    — set by SUB when result goes below 0

    // ── Instruction Fetch (combinational) ────────────────────────────
    // These wires update instantly whenever pc changes — no clock needed.
    wire [11:0] current_instr   = I_SRAM[pc];
    wire [3:0]  current_op      = current_instr[11:8];
    wire [7:0]  current_operand = current_instr[7:0];

    // ── ALU: 9-bit to detect overflow and underflow ───────────────────
    // The extra bit [8] acts as a carry/borrow detector.
    // If add_result[8] == 1, the sum exceeded 255 (overflow).
    // If sub_result[8] == 1, the result went negative (underflow).
    wire [8:0] add_result = {1'b0, w_reg} + {1'b0, current_operand};
    wire [8:0] sub_result = {1'b0, w_reg} - {1'b0, current_operand};

    // ── Unified Core Logic (single always block on base clock) ────────
    // Everything lives in one always block to avoid multiple-driver errors.
    // Speed selection is handled via the run_enable gate rather than
    // a separate clock domain.
    always @(posedge clk or posedge rst) begin

        // ── RESET (highest priority) ──────────────────────────────────
        // pulse_rst fires for ONE cycle on the 0→1 edge of SW[15].
        // The processor clears all state and immediately resumes operation.
        // I_SRAM and D_SRAM are NOT cleared — your program stays intact.
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
            pc <= 4'd0; // freeze PC at 0 — nothing executes while programming

            // Each 0→1 flip of SW[14] writes one instruction and advances
            // the write pointer. browse_address auto-follows so the LEDs
            // always show the slot that was just written.
            if (pulse_save) begin
                I_SRAM[prog_address] <= input_block;
                prog_address         <= prog_address + 4'd1;
                browse_address       <= prog_address + 4'd1;
            end

            // BTN0/BTN1 scroll the browse pointer independently of the
            // write pointer, so you can inspect any slot at any time.
            if (pulse_next) browse_address <= browse_address + 4'd1;
            if (pulse_prev) browse_address <= browse_address - 4'd1;

            // LEDs: upper 12 bits = instruction, lower 4 bits = slot address
            output_block <= {I_SRAM[browse_address], browse_address};

        // ── RUN MODE ─────────────────────────────────────────────────
        // Executes every cycle at 50MHz (speed=0),
        // or only on the run_enable tick at 1Hz (speed=1).
        end else if ((speed == 1'b0 || run_enable) && !halted) begin

            // Top 8 LEDs always show the live opcode and PC.
            // This updates every cycle so you can watch execution in real time.
            // Bottom 8 LEDs (w_reg) are only updated by the OUT instruction.
            output_block[15:12] <= current_op;
            output_block[11:8]  <= pc;

            case (current_op)

                4'd0: begin
                    // HLT — halt execution
                    // The processor freezes here until reset.
                    // Useful as a deliberate end-of-program marker.
                    halted <= 1'b1;
                    pc     <= pc;
                end

                4'd1: begin
                    // LDI — load immediate
                    // Puts the operand value directly into the accumulator.
                    w_reg <= current_operand;
                    pc    <= pc + 4'd1;
                end

                4'd2: begin
                    // LD — load from data memory
                    // Reads D_SRAM at address operand[3:0] into the accumulator.
                    w_reg <= D_SRAM[current_operand[3:0]];
                    pc    <= pc + 4'd1;
                end

                4'd3: begin
                    // ST — store to data memory
                    // Writes the accumulator to D_SRAM at address operand[3:0].
                    D_SRAM[current_operand[3:0]] <= w_reg;
                    pc    <= pc + 4'd1;
                end

                4'd4: begin
                    // CMP — compare accumulator with operand
                    // Sets exactly one of zf/lt/gt; clears the other two.
                    // Always use CMP before a skip instruction.
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
                    // (i.e. last CMP found w_reg == operand)
                    pc <= zf ? pc + 4'd2 : pc + 4'd1;
                end

                4'd7: begin
                    // SKLT — skip next instruction if less-than flag is set
                    // (i.e. last CMP found w_reg < operand)
                    pc <= lt ? pc + 4'd2 : pc + 4'd1;
                end

                4'd8: begin
                    // SKGT — skip next instruction if greater-than flag is set
                    // (i.e. last CMP found w_reg > operand)
                    pc <= gt ? pc + 4'd2 : pc + 4'd1;
                end

                4'd9: begin
                    // ADD — add operand to accumulator
                    // add_result[8] captures overflow into carry flag cf.
                    w_reg <= add_result[7:0];
                    cf    <= add_result[8];
                    pc    <= pc + 4'd1;
                end

                4'd10: begin
                    // SUB — subtract operand from accumulator
                    // sub_result[8] captures underflow into borrow flag bf.
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
                    // OUT — latch accumulator value to bottom 8 LEDs
                    // Top 8 LEDs (opcode + PC) continue updating as normal.
                    // Bottom 8 LEDs hold this value until the next OUT.
                    output_block[7:0] <= w_reg;
                    pc                <= pc + 4'd1;
                end

                default: begin
                    // Unknown opcode — advance PC to avoid permanent lockup
                    pc <= pc + 4'd1;
                end

            endcase
        end
    end

endmodule
