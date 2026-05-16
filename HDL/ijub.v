//  IJUB Architecture — 12-bit Accumulator Processor
//  16 opcodes | 8-bit accumulator | 16-slot I/D SRAM
// ============================================================

module IJUB_architecture (
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

    // ── Clock Divider: 50MHz → 1Hz ───────────────────────────────────
    reg [25:0] div_counter;
    reg        clk_1hz;

    always @(posedge clk) begin
        if (div_counter >= 26'd24_999_999) begin
            div_counter <= 0;
            clk_1hz     <= ~clk_1hz;
        end else begin
            div_counter <= div_counter + 1;
        end
    end

    wire clk_selected = speed ? clk_1hz : clk;

    // ── Switch/Button Conditioning ───────────────────────────────────
    wire clean_save, pulse_save;
    wire clean_rst,  pulse_rst;
    
    wire clean_next, pulse_next;
    wire clean_prev, pulse_prev;

    // save: debounce + edge detect
    debouncer #(.STABLE_COUNT(500000)) u_deb_save (
        .clk(clk), .noisy(save), .clean(clean_save)
    );
    edge_detector u_edg_save (.clk(clk), .sw(clean_save),  .btn_pulse(pulse_save));

    // rst: debounce + edge detect
    debouncer #(.STABLE_COUNT(500000)) u_deb_rst (
        .clk(clk), .noisy(rst), .clean(clean_rst)
    );
    edge_detector u_edg_rst (.clk(clk), .sw(clean_rst),  .btn_pulse(pulse_rst));

    // browse_next: debounced + edge detect
    debouncer #(.STABLE_COUNT(500000)) u_deb_next (
        .clk(clk), .noisy(browse_next), .clean(clean_next)
    );
    edge_detector u_edg_next (.clk(clk), .sw(clean_next), .btn_pulse(pulse_next));

    // browse_prev: debounced + edge detect
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

    // ── Program Mode (runs on base 50MHz clock) ───────────────────────
    always @(posedge clk) begin
        if (pulse_rst) begin
            prog_address   <= 0;
            browse_address <= 0;
        end else if (!mode) begin
            pc <= 0; // hold PC at reset while programming

            // Write current input_block into I_SRAM at prog_address
            if (pulse_save) begin
                I_SRAM[prog_address] <= input_block;
                prog_address         <= prog_address + 1;
            end

            // Scroll browse pointer forward/backward
            if (pulse_next) browse_address <= browse_address + 1;
            if (pulse_prev) browse_address <= browse_address - 1;

            // Always display: instruction at browse slot + its address
            output_block <= {I_SRAM[browse_address], browse_address};
        end
    end

    // ── Run Mode (runs on clk_selected — 50MHz or 1Hz) ───────────────
    always @(posedge clk_selected) begin
        if (clean_rst) begin
            pc    <= 0;
            w_reg <= 0;
            zf    <= 0; 
            lt    <= 0; 
            gt    <= 0; 
            cf    <= 0; 
            bf    <= 0;
        end else if (mode) begin
            case (current_op)
                4'd0: begin
                    // NOP — do nothing, advance PC
                    pc <= pc + 1;
                end
                4'd1: begin
                    w_reg <= current_operand;
                    pc    <= pc + 1;
                end
                4'd2: begin
                    w_reg <= D_SRAM[current_operand[3:0]];
                    pc    <= pc + 1;
                end
                4'd3: begin
                    D_SRAM[current_operand[3:0]] <= w_reg;
                    pc    <= pc + 1;
                end
                4'd4: begin
                    if (w_reg == current_operand) begin
                        zf <= 1; lt <= 0; gt <= 0;
                    end else if (w_reg < current_operand) begin
                        zf <= 0; lt <= 1; gt <= 0;
                    end else begin
                        zf <= 0; lt <= 0; gt <= 1;
                    end
                    pc <= pc + 1;
                end
                4'd5: begin
                    // JMP: Unconditional branch to target address
                    pc <= current_operand[3:0];
                end
                4'd6: begin
                    // JZ: Jump to address if Zero, else step forward
                    pc <= zf ? current_operand[3:0] : pc + 1;
                end
                4'd7: begin
                    // JLT: Jump to address if Less Than, else step forward
                    pc <= lt ? current_operand[3:0] : pc + 1;
                end
                4'd8: begin
                    // JGT: Jump to address if Greater Than, else step forward
                    pc <= gt ? current_operand[3:0] : pc + 1;
                end
                4'd9: begin
                    w_reg <= add_result[7:0];
                    cf    <= add_result[8];
                    pc    <= pc + 1;
                end
                4'd10: begin
                    w_reg <= sub_result[7:0];
                    bf    <= sub_result[8];
                    pc    <= pc + 1;
                end
                4'd11: begin
                    w_reg <= w_reg & current_operand;
                    pc    <= pc + 1;
                end
                4'd12: begin
                    w_reg <= w_reg | current_operand;
                    pc    <= pc + 1;
                end
                4'd13: begin
                    w_reg <= ~w_reg;
                    pc    <= pc + 1;
                end
                4'd14: begin
                    w_reg <= w_reg ^ current_operand;
                    pc    <= pc + 1;
                end
                4'd15: begin
                    output_block <= {cf, bf, gt, lt, zf, 3'b000, w_reg};
                    pc           <= pc + 1;
                end
                default: begin
                    // Unrecognized opcode safely advances to avoid permanent lockup
                    pc <= pc + 1;
                end
            endcase
        end
    end

endmodule
