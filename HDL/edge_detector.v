// ============================================================
//  Edge Detector
//  Produces a single 1-cycle pulse on the rising edge of sw.
//  Turns a level signal into a single button-press event.
// ============================================================
module edge_detector (
    input  clk,
    input  sw,
    output btn_pulse
);
    reg sw_curr, sw_prev;

    always @(posedge clk) begin
        sw_curr <= sw;
        sw_prev <= sw_curr;
    end

    assign btn_pulse = sw_curr & ~sw_prev;
endmodule