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

/*reg sync_0, sync_1;
    reg sw_prev;

    always @(posedge clk) begin
        sync_0  <= sw;     // Stage 1: Catch asynchronous input
        sync_1  <= sync_0; // Stage 2: Stabilize signal to clock domain
        sw_prev <= sync_1; // Stage 3: Delay for edge comparison
    end

    // High for exactly one clock cycle when transition completes cleanly
    assign btn_pulse = sync_1 & ~sw_prev;
endmodule 
*/
