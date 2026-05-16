// ============================================================
//  Debouncer
//  Waits STABLE_COUNT cycles of steady signal before passing.
//  Use for DIP switches to eliminate mechanical bounce.
// ============================================================
module debouncer #(
    parameter STABLE_COUNT = 500000  // ~10ms at 50MHz
)(
    input      clk,
    input      noisy,
    output reg clean
);
    reg [19:0] count;
    reg        last;

    always @(posedge clk) begin
        if (noisy != last) begin
            last  <= noisy;
            count <= 0;
        end else if (count < STABLE_COUNT) begin
            count <= count + 1;
        end else begin
            clean <= last;
        end
    end
endmodule