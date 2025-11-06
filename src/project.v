/*
 * 50/60 Hz mains clock -> static 6-digit 7-seg (HH:MM:SS)
 * - Template 'clk' is the AC tick (50/60 Hz selectable by ui_in[5])
 * - ui_in[0] = PPS (pulse-per-second). If a rising edge is seen, seconds align to it.
 * - set_mode: inc_* wrap field only (no cascade)
 * Outputs:
 *   uo_out[7]   = Colon LED (toggles each second)
 *   uo_out[6:0] = Shared segment bus {a,b,c,d,e,f,g} to all digits (no dp)
 *   uio_out[5:0]= 6 latch-enables (Ht,Ho,Mt,Mo,St,So)
 *   uio_out[6]  = PM (1 in 12h mode for 12..23)
 *   uio_out[7]  = 1 Hz pulse (1 AC tick wide each second)
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// ===================== TOP =======================
module tt_um_example (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // [7]=colon LED, [6:0]={a..g}
    input  wire [7:0] uio_in,   // IOs: Input path (unused)
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (1=drive)
    input  wire       ena,      // always 1 (unused)
    input  wire       clk,      // AC-derived logic clock: 50/60 Hz
    input  wire       rst_n     // reset_n - low to reset
);

  // --------- Inputs ---------
  wire pps_in      = ui_in[0];  // PPS (>=1 AC tick wide pulse)
  wire set_mode    = ui_in[1];
  wire inc_hours   = ui_in[2];
  wire inc_minutes = ui_in[3];
  wire inc_seconds = ui_in[4];
  wire ac50_sel    = ui_in[5];  // 0=60 Hz, 1=50 Hz
  wire hour_12h    = ui_in[6];
  // ui_in[7] spare

  wire rst    = ~rst_n;
  wire clk_ac = clk;

  // --------- Time core ---------
  wire [23:0] bcd24;
  wire        pm_led, colon_1hz;
  wire        sec_pulse_1hz;

  time_core_ac_bcd24 #(
    .DEB_LEN(3)
  ) u_time (
    .clk_ac       (clk_ac),
    .rst          (rst),
    .ac50_sel     (ac50_sel),
    .pps_in       (pps_in),
    .set_mode     (set_mode),
    .inc_hours    (inc_hours),
    .inc_minutes  (inc_minutes),
    .inc_seconds  (inc_seconds),
    .hour_12h     (hour_12h),
    .bcd24        (bcd24),
    .pm_led       (pm_led),
    .colon_1hz    (colon_1hz),
    .sec_pulse_1hz(sec_pulse_1hz)
  );

  // --------- 7-seg driver (no dp) ---------
  wire [6:0] seg7_bus; // {a,b,c,d,e,f,g}
  wire [5:0] le;       // one-hot pulse per digit: {Ht,Ho,Mt,Mo,St,So}

  bcd24_to_seg7_latched #(
    .SEG_ACTIVE_LOW(1'b0),   // 0=common-cathode (active-high), 1=common-anode (active-low)
    .LE_ACTIVE_HIGH(1'b1)    // set to 0 if your latches are active-low
  ) u_seg (
    .clk_ac  (clk_ac),
    .rst     (rst),
    .bcd24   (bcd24),
    .seg7_bus(seg7_bus),
    .le      (le)
  );

  // --------- Map to pins ---------
  assign uo_out[6:0] = seg7_bus;    // {a..g}
  assign uo_out[7]   = colon_1hz;   // toggles once per second

  assign uio_out[5:0] = le;
  assign uio_out[6]   = pm_led;         // PM indicator
  assign uio_out[7]   = sec_pulse_1hz;  // 1 AC-tick pulse each second
  assign uio_oe       = 8'b11111111;    // drive all uio_out pins

  // Tie off unused to avoid warnings
  wire _unused = &{ena, uio_in, ui_in[7], 1'b0};

endmodule

// =================== TIME CORE ===================
module time_core_ac_bcd24 #(
    parameter integer DEB_LEN = 3   // debounce depth in AC ticks (N>=2)
)(
    input  wire        clk_ac,      // 50/60 Hz logic clock
    input  wire        rst,
    input  wire        ac50_sel,    // 1=50Hz, 0=60Hz
    input  wire        pps_in,      // PPS (>=1 AC tick wide)
    input  wire        set_mode,
    input  wire        inc_hours,
    input  wire        inc_minutes,
    input  wire        inc_seconds,
    input  wire        hour_12h,
    output wire [23:0] bcd24,       // {Ht,Ho,Mt,Mo,St,So}
    output reg         pm_led,
    output reg         colon_1hz,   // toggles each second
    output reg         sec_pulse_1hz// 1 AC-tick pulse at each second boundary
);
    // Debounce (AC domain)
    wire set_d, ih_d, im_d, is_d, mode12_d;
    debounce_sr #(.N(DEB_LEN)) db_set (.clk(clk_ac), .din(set_mode),   .dout(set_d));
    debounce_sr #(.N(DEB_LEN)) db_ih  (.clk(clk_ac), .din(inc_hours),  .dout(ih_d));
    debounce_sr #(.N(DEB_LEN)) db_im  (.clk(clk_ac), .din(inc_minutes),.dout(im_d));
    debounce_sr #(.N(DEB_LEN)) db_is  (.clk(clk_ac), .din(inc_seconds),.dout(is_d));
    debounce_sr #(.N(DEB_LEN)) db_12  (.clk(clk_ac), .din(hour_12h),   .dout(mode12_d));

    // Rising-edge detectors for single-step
    reg ih_q, im_q, is_q;
    always @(posedge clk_ac) begin
        if (rst) begin ih_q<=1'b0; im_q<=1'b0; is_q<=1'b0; end
        else begin ih_q<=ih_d; im_q<=im_d; is_q<=is_d; end
    end
    wire inc_h_pulse = ih_d & ~ih_q;
    wire inc_m_pulse = im_d & ~im_q;
    wire inc_s_pulse = is_d & ~is_q;

    // PPS synchronizer & edge detect (sampled at AC rate)
    reg pps_q;
    always @(posedge clk_ac) begin
        if (rst) pps_q <= 1'b0;
        else     pps_q <= pps_in;
    end
    wire pps_edge = pps_in & ~pps_q;  // requires PPS high at an AC clock edge

    // Divider
    reg [5:0] ac_div;
    wire [5:0] ac_top   = ac50_sel ? 6'd49 : 6'd59;   // count 0..49 or 0..59
    wire       run_mode = ~set_d;

    // --- NEW: combinational second tick (no 1-cycle latency) ---
    // Based on *current* state: tick when PPS rises OR divider == top, but only in run mode.
    wire sec_tick = run_mode && (pps_edge || (ac_div == ac_top));

    // Divider + colon + 1Hz pulse (use sec_tick)
    always @(posedge clk_ac) begin
        if (rst) begin
            ac_div        <= 6'd0;
            colon_1hz     <= 1'b0;
            sec_pulse_1hz <= 1'b0;
        end else begin
            sec_pulse_1hz <= 1'b0; // default

            if (run_mode) begin
                if (sec_tick) begin
                    ac_div        <= 6'd0;
                    sec_pulse_1hz <= 1'b1;      // mirrors sec_tick for one AC tick
                    colon_1hz     <= ~colon_1hz;
                end else begin
                    ac_div <= ac_div + 6'd1;
                end
            end
            // set_mode: freeze divider & no pulses
        end
    end

    // Timekeeping in BCD (24h base)
    reg [3:0] ss_1, ss_10;  // 00..59
    reg [3:0] mm_1, mm_10;  // 00..59
    reg [3:0] hh_1, hh_10;  // 00..23

    // Run vs set behavior (no cascade in set mode)
    wire sec_roll     = (ss_1 == 4'd9) && (ss_10 == 4'd5);  // 59
    wire min_roll     = (mm_1 == 4'd9) && (mm_10 == 4'd5);  // 59

    // --- Use sec_tick directly so seconds/minutes/hours update in the *same* AC edge ---
    wire add_sec      = run_mode ? sec_tick : inc_s_pulse;
    wire run_add_min  = run_mode && sec_tick && sec_roll;
    wire add_min      = run_mode ? run_add_min  : inc_m_pulse;
    wire run_add_hour = run_mode && run_add_min && min_roll;
    wire add_hour     = run_mode ? run_add_hour : inc_h_pulse;

    // seconds
    always @(posedge clk_ac) begin
        if (rst) begin ss_1<=4'd0; ss_10<=4'd0; end
        else if (add_sec) begin
            if (ss_1 == 4'd9) begin
                ss_1  <= 4'd0;
                ss_10 <= (ss_10 == 4'd5) ? 4'd0 : (ss_10 + 4'd1);
            end else ss_1 <= ss_1 + 4'd1;
        end
    end
    // minutes
    always @(posedge clk_ac) begin
        if (rst) begin mm_1<=4'd0; mm_10<=4'd0; end
        else if (add_min) begin
            if (mm_1 == 4'd9) begin
                mm_1  <= 4'd0;
                mm_10 <= (mm_10 == 4'd5) ? 4'd0 : (mm_10 + 4'd1);
            end else mm_1 <= mm_1 + 4'd1;
        end
    end
    // hours (00..23)
    always @(posedge clk_ac) begin
        if (rst) begin hh_1<=4'd0; hh_10<=4'd0; end
        else if (add_hour) begin
            if ((hh_10 == 4'd2) && (hh_1 == 4'd3)) begin
                hh_10 <= 4'd0; hh_1 <= 4'd0; // 23 -> 00
            end else if (hh_1 == 4'd9) begin
                hh_1  <= 4'd0; hh_10 <= hh_10 + 4'd1;
            end else hh_1 <= hh_1 + 4'd1;
        end
    end

    // 24h -> 12h display + PM flag
    reg  [5:0] h24, h12;     // 0..23 / 1..12
    reg        t12;          // 0/1 for tens
    reg  [3:0] disp_h_10, disp_h_1, ones12;

    always @* begin
        h24    = (hh_10 * 6'd10) + {2'b00, hh_1};
        pm_led = (mode12_d && (h24 >= 6'd12)) ? 1'b1 : 1'b0;

        if (!mode12_d) begin
            disp_h_10 = hh_10;
            disp_h_1  = hh_1;
        end else begin
            if (h24 == 6'd0)       h12 = 6'd12;
            else if (h24 <= 6'd12) h12 = h24;
            else                   h12 = h24 - 6'd12;

            t12    = (h12 >= 6'd10);
            ones12 = h12 - (t12 ? 6'd10 : 6'd0);

            disp_h_10 = {3'b000, t12}; // 0 or 1
            disp_h_1  = ones12;        // 0..9
        end
    end

    assign bcd24 = {disp_h_10, disp_h_1, mm_10, mm_1, ss_10, ss_1};
endmodule


// =================== 7-SEG DRIVER (no dp) ===================
module bcd24_to_seg7_latched #(
    parameter SEG_ACTIVE_LOW = 1'b0,  // 0=active-high (common-cathode), 1=active-low (common-anode)
    parameter LE_ACTIVE_HIGH = 1'b1   // 1=LE high pulses, 0=LE low pulses
)(
    input  wire        clk_ac,
    input  wire        rst,
    input  wire [23:0] bcd24,        // {Ht,Ho,Mt,Mo,St,So}
    output reg  [6:0]  seg7_bus,     // {a,b,c,d,e,f,g} shared to all digits
    output reg  [5:0]  le            // one-hot latch enables {Ht,Ho,Mt,Mo,St,So}
);
    // Unpack nibbles
    wire [3:0] Ht = bcd24[23:20];
    wire [3:0] Ho = bcd24[19:16];
    wire [3:0] Mt = bcd24[15:12];
    wire [3:0] Mo = bcd24[11:8];
    wire [3:0] St = bcd24[7:4];
    wire [3:0] So = bcd24[3:0];

    // Phase: which digit to latch this AC tick
    reg [2:0] phase;

    // 7-seg encoder: returns {a,b,c,d,e,f,g} active-high
    function [6:0] enc7;
        input [3:0] d;
        begin
            case (d)
              4'd0: enc7 = 7'b1111110;
              4'd1: enc7 = 7'b0110000;
              4'd2: enc7 = 7'b1101101;
              4'd3: enc7 = 7'b1111001;
              4'd4: enc7 = 7'b0110011;
              4'd5: enc7 = 7'b1011011;
              4'd6: enc7 = 7'b1011111;
              4'd7: enc7 = 7'b1110000;
              4'd8: enc7 = 7'b1111111;
              4'd9: enc7 = 7'b1111011;
              default: enc7 = 7'b0000001; // '-'
            endcase
        end
    endfunction

    // Polarity adapt (keeps {a..g} order)
    function [6:0] adapt7;
        input [6:0] abcd_efg;
        begin
            adapt7 = SEG_ACTIVE_LOW ? ~abcd_efg : abcd_efg;
        end
    endfunction

    // active level for LE pin
    wire [5:0] LE_ON  = LE_ACTIVE_HIGH ? 6'b111111 : 6'b000000;
    wire [5:0] LE_OFF = LE_ACTIVE_HIGH ? 6'b000000 : 6'b111111;

    always @(posedge clk_ac) begin
        if (rst) begin
            phase    <= 3'd0;
            seg7_bus <= 7'b0000000;
            le       <= LE_OFF;
        end else begin
            le <= LE_OFF;

            case (phase)
                3'd0: begin seg7_bus <= adapt7(enc7(Ht)); le[0] <= LE_ON[0]; end // Hour tens
                3'd1: begin seg7_bus <= adapt7(enc7(Ho)); le[1] <= LE_ON[1]; end // Hour ones
                3'd2: begin seg7_bus <= adapt7(enc7(Mt)); le[2] <= LE_ON[2]; end // Minute tens
                3'd3: begin seg7_bus <= adapt7(enc7(Mo)); le[3] <= LE_ON[3]; end // Minute ones
                3'd4: begin seg7_bus <= adapt7(enc7(St)); le[4] <= LE_ON[4]; end // Second tens
                3'd5: begin seg7_bus <= adapt7(enc7(So)); le[5] <= LE_ON[5]; end // Second ones
                default: begin seg7_bus <= adapt7(enc7(4'd0)); end
            endcase

            // advance to next digit each AC tick
            phase <= (phase == 3'd5) ? 3'd0 : (phase + 3'd1);
        end
    end
endmodule

// =================== Debouncer (look-ahead, exact N ticks) ===================
module debounce_sr #(
    parameter integer N = 3   // N >= 2 recommended
)(
    input  wire clk,
    input  wire din,
    output reg  dout
);
    // compute next shift value to include the current sample
    generate
      if (N == 1) begin : gen_n1
        always @(posedge clk) begin
          if (din)      dout <= 1'b1;
          else          dout <= 1'b0;
        end
      end else begin : gen_nge2
        reg  [N-1:0] sh;
        wire [N-1:0] sh_next = {sh[N-2:0], din};
        wire         all1    = &sh_next;
        wire         all0    = ~|sh_next;
        always @(posedge clk) begin
          sh <= sh_next;
          if (all1)      dout <= 1'b1;
          else if (all0) dout <= 1'b0;
          // else hold last dout
        end
      end
    endgenerate
endmodule
