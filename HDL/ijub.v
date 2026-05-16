// ============================================================
//  IJUB Architecture — 12-bit Accumulator Processor
//  16 opcodes | 8-bit accumulator | 16-slot I/D SRAM
// ============================================================

module ijub (
    input  [11:0] input_block,   // DIP switches [11:0]  — instruction input
    output reg [15:0] output_block,
    input  mode,                 // DIP switch [12]      — 0=program, 1=run
    input  speed,                // DIP switch [13]      — 0=50MHz,   1=1Hz
    input  save,                 // DIP switch [14]      — write to I_SRAM
    input  rst,                  // DIP switch [15]      — Reset Processor
    input  browse_next,          // FPGA button          — scroll forward
    input  browse_prev,          // FPGA button          — scroll backward
    input  clk                   // 50MHz board clock
);

    // ── Clock Enable: 50MHz → 1Hz Pulse ──────────────────────────────
    reg [25:0] div_counter;
    reg        run_enable;

    always @(posedge clk) begin
        // Generate a 1-cycle enable pulse every 50,000,000 ticks (1Hz)
        if (div_counter >= 26'd49_999_999) begin
            div_counter <= 26'd0;
            run_enable  <= 1'b1;
        end else begin
            div_counter <= div_counter + 26'd1;
            run_enable  <= 1'b0;
        end
    end

    // ── Switch/Button Conditioning ───────────────────────────────────
    wire clean_save, pulse_save;
    wire clean_rst,  pulse_rst;
    wire clean_next, pulse_next;
    wire clean_prev, pulse_prev;

    debouncer #(.STABLE_COUNT(500000)) u_deb_save (
        .clk(clk), .noisy(save), .clean(clean_save)
    );
    edge_detector u_edg_save (.clk(clk), .sw(clean_save),  .btn_pulse(pulse_save));

    debouncer #(.STABLE_COUNT(500000)) u_deb_rst (
        .clk(clk), .noisy(rst), .clean(clean_rst)
    );
    edge_detector u_edg_rst (.clk(clk), .sw(clean_rst),  .btn_pulse(pulse_rst));

    debouncer #(.STABLE_COUNT(500000)) u_deb_next (
        .clk(clk), .noisy(browse_next), .clean(clean_next)
    );
    edge_detector u_edg_next (.clk(clk), .sw(clean_next), .btn_pulse(pulse_next));

    debouncer #(.STABLE_COUNT(500000)) u_deb_prev (
        .clk(clk), .noisy(browse_prev), .clean(clean_prev)
    );
    edge_detector u_edg_prev (.clk(clk), .sw(clean_prev), .btn_pulse(pulse_prev));

    // ── Memory ───────────────────────────────────────────────────────
    reg [11:0] I_SRAM [15:0];   // instruction memory: 16 x 12-bit
    reg [7:0]  D_SRAM [15:0];   // data memory:        16 x 8-bit

    // ── Registers ────────────────────────────────────────────────────
    reg [7:0] w_reg;            // accumulator
    reg [3:0] pc;               // program counter (run mode)
    reg [3:0] prog_address;     // write pointer  (program mode)
    reg [3:0] browse_address;   // read pointer   (program mode browse)

    // ── Flags ────────────────────────────────────────────────────────
    reg zf;   // zero flag      — set by CMP when w_reg == operand
    reg lt;   // less-than flag — set by CMP when w_reg  < operand
    reg gt;   // greater-than   — set by CMP when w_reg  > operand
    reg cf;   // carry flag     — set by ADD on overflow
    reg bf;   // borrow flag    — set by SUB on underflow

    // ── Instruction Fetch (combinational) ────────────────────────────
    wire [11:0] current_instr   = I_SRAM[pc];
    wire [3:0]  current_op      = current_instr[11:8];
    wire [7:0]  current_operand = current_instr[7:0];

    // ── ALU wires (9-bit to capture carry/borrow) ────────────────────
    wire [8:0] add_result = {1'b0, w_reg} + {1'b0, current_operand};
    wire [8:0] sub_result = {1'b0, w_reg} - {1'b0, current_operand};

    // ── Unified Core Logic (Single Always Block) ─────────────────────
    always @(posedge clk) begin
        if (pulse_rst) begin
            // Highest Priority: Reset everything
            prog_address   <= 4'd0;
            browse_address <= 4'd0;
            pc             <= 4'd0;
            w_reg          <= 8'd0;
            zf <= 0; lt <= 0; gt <= 0; cf <= 0; bf <= 0;
            output_block   <= 16'd0;
            
        end else if (!mode) begin
            // ── PROGRAM MODE ──
            pc <= 4'd0; // hold PC at zero while programming

            // Write current input_block into I_SRAM at prog_address
            if (pulse_save) begin
                I_SRAM[prog_address] <= input_block;
                prog_address         <= prog_address + 4'd1;
            end

            // Scroll browse pointer forward/backward
            if (pulse_next) browse_address <= browse_address + 4'd1;
            if (pulse_prev) browse_address <= browse_address - 4'd1;

            // Display: instruction at browse slot + its address
            output_block <= {I_SRAM[browse_address], browse_address};

        end else if (speed == 0 || run_enable) begin
            // ── RUN MODE ──
            // Executes every cycle if speed=0 (50MHz), or once per pulse if speed=1 (1Hz)
            
            pc <= pc + 4'd1; // default: advance to next instruction

            case (current_op)
                4'd0: begin
                    // NOP
                end
                4'd1: begin
                    w_reg <= current_operand;
                end
                4'd2: begin
                    w_reg <= D_SRAM[current_operand[3:0]];
                end
                4'd3: begin
                    D_SRAM[current_operand[3:0]] <= w_reg;
                end
                4'd4: begin
                    if (w_reg == current_operand) begin
                        zf <= 1; lt <= 0; gt <= 0;
                    end else if (w_reg < current_operand) begin
                        zf <= 0; lt <= 1; gt <= 0;
                    end else begin
                        zf <= 0; lt <= 0; gt <= 1;
                    end
                end
                4'd5: begin
                    pc <= current_operand[3:0];
                end
                4'd6: begin
                    if (zf) pc <= pc + 4'd2;
                end
                4'd7: begin
                    if (lt) pc <= pc + 4'd2;
                end
                4'd8: begin
                    if (gt) pc <= pc + 4'd2;
                end
                4'd9: begin
                    w_reg <= add_result[7:0];
                    cf    <= add_result[8];
                end
                4'd10: begin
                    w_reg <= sub_result[7:0];
                    bf    <= sub_result[8];
                end
                4'd11: begin
                    w_reg <= w_reg & current_operand;
                end
                4'd12: begin
                    w_reg <= w_reg | current_operand;
                end
                4'd13: begin
                    w_reg <= ~w_reg;
                end
                4'd14: begin
                    w_reg <= w_reg ^ current_operand;
                end
                4'd15: begin
                    output_block <= {cf, bf, gt, lt, zf, 3'b000, w_reg};
                end
                default: begin
                    // unrecognized opcode — do nothing
                end
            endcase
        end
    end

endmodule
