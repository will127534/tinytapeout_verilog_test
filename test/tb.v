`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/
module tb ();

  // Dump the signals to a VCD file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;


  // Aliases
  wire [5:0] le    = uio_out[5:0];
  wire [6:0] seg7  = uo_out[6:0];   // {a..g}
  wire       colon = uo_out[7];
  wire       pm    = uio_out[6];
  wire       onehz = uio_out[7];

  // Latched digits (what the external latches would hold)
  reg [3:0] Ht,Ho,Mt,Mo,St,So;

  // -------- Globals / loop counters (unique per task!) --------
  integer i_wait;              // wait_ticks
  integer i_step;              // step_n
  integer i_main;              // main loops
  integer cnt, tol;            // measure_colon_ticks and PPS check
  reg     prev;                // edge detect helper
  integer npress;              // step_n copies n here
  integer cur, delta;          // setters
  integer pulses_seen, ticks_since_pulse;

  // AC "tick" period (sped up): one AC tick = 200us (5 kHz)
  parameter integer TICK_US = 200;

  // Clock gen
  initial begin
    clk = 1'b0;
    forever #(TICK_US/2) clk = ~clk;
  end

  // Defaults
  initial begin
    ena    = 1'b1;
    ui_in  = 8'h00;
    uio_in = 8'h00;
  end



`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  // Replace tt_um_example with your module name:
  tt_um_willwhang user_project (

      // Include power ports for the Gate Level test:
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

  // ------------ Helpers ------------
  task wait_ticks; input integer n;
    begin
      for (i_wait=0; i_wait<n; i_wait=i_wait+1) @(posedge clk);
    end
  endtask

  // press ui_in[bit_idx] for 'high_ticks' AC ticks (for debounce)
  task press; input integer bit_idx; input integer high_ticks;
    begin
      @(posedge clk);
      ui_in[bit_idx] = 1'b1;
      wait_ticks(high_ticks);
      ui_in[bit_idx] = 1'b0;
    end
  endtask

  // 7-seg decoder {a..g} -> 0..9 (F=invalid)
  function [3:0] dec7; input [6:0] abcd_efg; // pass {a..g}
    begin
      case (abcd_efg)
        7'b1111110: dec7 = 4'd0;
        7'b0110000: dec7 = 4'd1;
        7'b1101101: dec7 = 4'd2;
        7'b1111001: dec7 = 4'd3;
        7'b0110011: dec7 = 4'd4;
        7'b1011011: dec7 = 4'd5;
        7'b1011111: dec7 = 4'd6;
        7'b1110000: dec7 = 4'd7;
        7'b1111111: dec7 = 4'd8;
        7'b1111011: dec7 = 4'd9;
        default:     dec7 = 4'hF;
      endcase
    end
  endfunction

  // Capture digits exactly when LEs strobe; seg7 is already {a..g}
  always @(posedge clk) begin
    if (le[0]) Ht <= dec7(seg7);
    if (le[1]) Ho <= dec7(seg7);
    if (le[2]) Mt <= dec7(seg7);
    if (le[3]) Mo <= dec7(seg7);
    if (le[4]) St <= dec7(seg7);
    if (le[5]) So <= dec7(seg7);
  end

  task show_time; input [127:0] tag;
    begin
      $display("[%0t us] %0s  HH=%0d%0d (PM=%0d)  MM=%0d%0d  SS=%0d%0d  colon=%0d oneHz=%0d",
        $time, tag, Ht,Ho, pm, Mt,Mo, St,So, colon, onehz);
    end
  endtask

  // Measure AC ticks between two colon LED toggles.
  task measure_colon_ticks; input integer expect_ticks; input integer tol_i;
    begin
      tol = tol_i;
      // wait for next edge
      prev = colon; @(posedge clk);
      while (colon==prev) @(posedge clk);
      // count until next edge
      prev = colon; cnt=0;
      while (colon==prev) begin @(posedge clk); cnt=cnt+1; end
      $display("Colon period ticks = %0d (expected %0d±%0d)", cnt, expect_ticks, tol);
      if ( (cnt < (expect_ticks - tol)) || (cnt > (expect_ticks + tol)) )
        $display("**WARN** colon ticks out of expected range");
    end
  endtask

  // N debounced presses on a button
  task step_n; input integer bit_idx; input integer n;
    begin
      npress = n;
      for (i_step=0; i_step<npress; i_step=i_step+1) begin
        press(bit_idx, 3);   // exactly DEB_LEN ticks high
        wait_ticks(3);
      end
    end
  endtask

  // Bring minutes to specific BCD in set_mode
  task set_minutes_bcd; input integer tgt;
    begin
      cur   = Mt*10 + Mo;
      delta = tgt - cur; if (delta<0) delta = delta + 60;
      step_n(3, delta); // ui_in[3]=inc_minutes
    end
  endtask

  // Bring hours to specific 24h in set_mode
  task set_hours_24; input integer tgt24;
    begin
      cur   = Ht*10 + Ho;
      delta = tgt24 - cur; if (delta<0) delta = delta + 24;
      step_n(2, delta); // ui_in[2]=inc_hours
    end
  endtask

  // ---------------------- Main stimulus ----------------------
  reg [3:0] So_before;
  reg [7:0] hr_before;

  initial begin

    // Reset
    rst_n = 1'b0;
    ui_in = 8'h00;      // 60 Hz mode (ui_in[5]=0), PPS low
    wait_ticks(5);
    rst_n = 1'b1;

    // -------- Verify 60 Hz colon period (expect 60 ticks) -------
    ui_in[1] = 1'b0;     // run mode
    wait_ticks(10);      // settle
    measure_colon_ticks(60, 0);

    // -------- Switch to 50 Hz and verify (expect 50 ticks ±3) ---
    ui_in[5] = 1'b1;     // ac50_sel=1
    measure_colon_ticks(50, 3);

    // -------- Debounce check in set_mode ------------------------
    ui_in[1] = 1'b1;     // set_mode=1 (freeze)
    wait_ticks(3);

    // Snapshot seconds ones
    So_before = So;

    // 1-tick glitch on inc_seconds -> should NOT register
    press(4, 1);         // ui_in[4]=inc_seconds, < DEB_LEN
    wait_ticks(4);
    show_time("after 1-tick glitch on inc_seconds");
    if (So !== So_before) $display("**FAIL** debounce: 1-tick glitch changed seconds");

    // Proper press (==DEB_LEN ticks) -> should increment seconds
    press(4, 3);
    wait_ticks(4);
    show_time("after debounced inc_seconds");

    // -------- No-cascade in set_mode: minutes 59 -> 00, hours unchanged ----
    ui_in[6] = 1'b0;     // hour_12h=0 (24h display)
    set_minutes_bcd(59);
    wait_ticks(6); show_time("minutes set to 59");
    hr_before = Ht*10 + Ho;

    // one debounced press on inc_minutes -> expect minutes=00, hours same
    press(3, 3);
    wait_ticks(6); show_time("after inc_minutes at 59 (no cascade expected)");

    if (Mt!==4'd0 || Mo!==4'd0) $display("**FAIL** minutes didn't wrap to 00");
    if ((Ht*10+Ho)!==hr_before) $display("**FAIL** hours changed in set_mode minute wrap");

    // -------- 12h mode check: set to 13:xx -> show 01 and PM=1 -------
    ui_in[6] = 1'b0; set_hours_24(13); wait_ticks(6); // force 13h in 24h
    ui_in[6] = 1'b1; wait_ticks(20); show_time("12h display after setting 13h");

    if (!(Ht==4'd0 && Ho==4'd1)) $display("**FAIL** 12h display: expected 01 for 13h");
    if (pm!==1'b1)               $display("**FAIL** PM not asserted on uio_out[6]");

    // -------- 1 Hz pulse check (uio_out[7]) in run mode -------------
    ui_in[1] = 1'b0;   // leave set_mode
    pulses_seen = 0; ticks_since_pulse = 0;
    for (i_main=0; i_main<130; i_main=i_main+1) begin
      @(posedge clk);
      if (onehz) begin pulses_seen = pulses_seen + 1; ticks_since_pulse = 0; end
      else ticks_since_pulse = ticks_since_pulse + 1;
    end
    $display("1Hz pulses observed in ~130 ticks: %0d", pulses_seen);

    // -------- PPS alignment test -----------------------------------
    ui_in[5] = 1'b0;    // back to 60 Hz

    prev = colon; @(posedge clk);
    while (colon==prev) @(posedge clk);
    wait_ticks(10);
    // PPS one-tick pulse (must be >= one AC tick to be seen)
    ui_in[0] = 1'b1;
    @(posedge clk);
    ui_in[0] = 1'b0;

    // Count ticks until next colon edge; expect ~0..1 (immediate align)
    prev = colon; cnt = 0;
    while (colon==prev) begin @(posedge clk); cnt = cnt + 1; end
    $display("Ticks from PPS to colon toggle: %0d (expect ~0..1)", cnt);
    if (cnt > 2) $display("**FAIL** PPS did not align seconds");



    wait_ticks(38400*60);
    $display("TB done.");
    $finish;
  end


endmodule
