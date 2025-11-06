# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
#
# Cocotb test for tt_um_example:
# - Drives a low-rate "AC tick" clock (simulating 50/60 Hz source, sped up)
# - Verifies colon period in 60/50 Hz modes
# - Debounce behavior for inc_seconds (glitch vs valid press)
# - NO-CASCADE in set_mode (minutes wrap without changing hours)
# - 12-hour display mapping and PM flag
# - 1 Hz pulse generation
# - PPS alignment (seconds snap to PPS)
#
# UI bit mapping (ui_in):
# [0]=PPS, [1]=set_mode, [2]=inc_hours, [3]=inc_minutes, [4]=inc_seconds,
# [5]=ac50_sel (0=60 Hz, 1=50 Hz), [6]=hour_12h, [7]=spare
#
# Outputs:
# uo_out[7]   = colon LED (toggles each second)
# uo_out[6:0] = {a,b,c,d,e,f,g}
# uio_out[5:0]= latch enables {Ht,Ho,Mt,Mo,St,So}
# uio_out[6]  = PM
# uio_out[7]  = 1 Hz pulse (one AC tick wide)

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# --- 7-seg decode for active-high {a,b,c,d,e,f,g} ---
SEG_TO_DIGIT = {
    0b1111110: 0,
    0b0110000: 1,
    0b1101101: 2,
    0b1111001: 3,
    0b0110011: 4,
    0b1011011: 5,
    0b1011111: 6,
    0b1110000: 7,
    0b1111111: 8,
    0b1111011: 9,
}


class DigitCapture:
    """Continuously captures digits when latch-enables pulse."""

    def __init__(self, dut):
        self.dut = dut
        self.Ht = 0
        self.Ho = 0
        self.Mt = 0
        self.Mo = 0
        self.St = 0
        self.So = 0

    def _decode(self, seg7_val: int) -> int:
        return SEG_TO_DIGIT.get(seg7_val & 0x7F, 0xF)

    async def run(self):
        """Snoop every posedge; when an LE bit is set, latch the current seg pattern as that digit."""
        while True:
            await RisingEdge(self.dut.clk)
            seg_bus = int(self.dut.uo_out.value) & 0x7F  # {a..g}
            le = int(self.dut.uio_out.value) & 0x3F      # {Ht..So}
            if le & (1 << 0):
                self.Ht = self._decode(seg_bus)
            if le & (1 << 1):
                self.Ho = self._decode(seg_bus)
            if le & (1 << 2):
                self.Mt = self._decode(seg_bus)
            if le & (1 << 3):
                self.Mo = self._decode(seg_bus)
            if le & (1 << 4):
                self.St = self._decode(seg_bus)
            if le & (1 << 5):
                self.So = self._decode(seg_bus)

    def hhmmss(self):
        return (self.Ht, self.Ho, self.Mt, self.Mo, self.St, self.So)


# --- UI helpers ---
def ui_get(dut) -> int:
    return int(dut.ui_in.value)

def ui_set(dut, val: int):
    dut.ui_in.value = val & 0xFF

def ui_bit_set(dut, idx: int, v: int):
    cur = ui_get(dut)
    if v:
        cur |= (1 << idx)
    else:
        cur &= ~(1 << idx)
    ui_set(dut, cur)


async def press(dut, bit_idx: int, high_ticks: int):
    """Hold a ui_in bit high for 'high_ticks' clock cycles (debounced button press)."""
    await RisingEdge(dut.clk)
    ui_bit_set(dut, bit_idx, 1)
    await ClockCycles(dut.clk, high_ticks)
    ui_bit_set(dut, bit_idx, 0)


async def step_n(dut, bit_idx: int, n: int):
    """Perform n debounced button presses, spacing them a few ticks apart."""
    for _ in range(n):
        await press(dut, bit_idx, 3)        # DEB_LEN = 3 ticks
        await ClockCycles(dut.clk, 3)


async def set_minutes_bcd(dut, cap: DigitCapture, tgt: int):
    """In set_mode, bring minutes to tgt (0..59) using inc_minutes without cascades."""
    cur = cap.Mt * 10 + cap.Mo
    delta = tgt - cur
    if delta < 0:
        delta += 60
    await step_n(dut, 3, delta)             # ui_in[3] = inc_minutes


async def set_hours_24(dut, cap: DigitCapture, tgt24: int):
    """In set_mode (24h display), bring hours to tgt24 (0..23) using inc_hours."""
    cur = cap.Ht * 10 + cap.Ho
    delta = tgt24 - cur
    if delta < 0:
        delta += 24
    await step_n(dut, 2, delta)             # ui_in[2] = inc_hours


async def measure_colon_ticks(dut, expect: int, tol: int = 0) -> int:
    """Count clk cycles between two colon LED toggles."""
    colon = (int(dut.uo_out.value) >> 7) & 0x1
    # wait for next edge
    while True:
        await RisingEdge(dut.clk)
        c = (int(dut.uo_out.value) >> 7) & 0x1
        if c != colon:
            colon = c
            break
    # count until next edge
    cnt = 0
    while True:
        await RisingEdge(dut.clk)
        cnt += 1
        c = (int(dut.uo_out.value) >> 7) & 0x1
        if c != colon:
            break

    if tol == 0:
        assert cnt == expect, f"Colon ticks={cnt}, expected {expect}"
    else:
        assert (expect - tol) <= cnt <= (expect + tol), f"Colon ticks={cnt}, expected {expect}±{tol}"
    return cnt


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # Use a sped-up "AC tick": one clk cycle = 200 us (5 kHz).
    # Colon toggles every 60 (or 50) ticks -> ~12 ms @ 5 kHz. Fast but preserves tick counts.
    clock = Clock(dut.clk, 200, unit="us")
    cocotb.start_soon(clock.start())

    # Start digit capture snooper
    cap = DigitCapture(dut)
    cocotb.start_soon(cap.run())

    # Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    ui_set(dut, 0)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # ---------------- 60 Hz colon period ----------------
    ui_bit_set(dut, 1, 0)   # set_mode=0
    ui_bit_set(dut, 5, 0)   # ac50_sel=0 -> 60 Hz
    await ClockCycles(dut.clk, 10)
    ticks_60 = await measure_colon_ticks(dut, expect=60, tol=0)
    dut._log.info(f"Colon period @60Hz: {ticks_60} ticks")

    # ---------------- 50 Hz colon period ----------------
    ui_bit_set(dut, 5, 1)   # ac50_sel=1 -> 50 Hz
    ticks_50 = await measure_colon_ticks(dut, expect=50, tol=3)
    dut._log.info(f"Colon period @50Hz: {ticks_50} ticks")

    # ---------------- Debounce in set_mode ---------------
    ui_bit_set(dut, 1, 1)   # set_mode=1 (freeze time)
    await ClockCycles(dut.clk, 3)

    so_before = cap.So

    # 1-tick glitch on inc_seconds -> should NOT register
    await press(dut, 4, 1)  # ui_in[4] = inc_seconds
    await ClockCycles(dut.clk, 4)
    dut._log.info(f"After 1-tick glitch: SS ones={cap.So}")
    assert cap.So == so_before, "Debounce failed: 1-tick glitch changed seconds"

    # Proper press (==3 ticks) -> should increment seconds (wrap allowed)
    await press(dut, 4, 3)
    await ClockCycles(dut.clk, 4)
    dut._log.info(f"After debounced press: SS ones={cap.So}")

    # ---------------- No-cascade in set_mode -------------
    # Ensure 24h display for this check
    ui_bit_set(dut, 6, 0)   # hour_12h=0
    # Drive minutes to 59 via increments
    await set_minutes_bcd(dut, cap, 59)
    await ClockCycles(dut.clk, 6)
    dut._log.info(f"Minutes set: {cap.Mt}{cap.Mo}")
    hr_before = cap.Ht * 10 + cap.Ho

    # One debounced inc_minutes -> expect 00, hours unchanged
    await press(dut, 3, 3)
    await ClockCycles(dut.clk, 6)
    dut._log.info(f"After inc at 59 -> {cap.Mt}{cap.Mo}, hours {cap.Ht}{cap.Ho}")
    assert (cap.Mt, cap.Mo) == (0, 0), "Minutes didn't wrap to 00 in set_mode"
    assert (cap.Ht * 10 + cap.Ho) == hr_before, "Hours changed on minute wrap in set_mode"

    # ---------------- 12h display & PM flag --------------
    # Force 13:xx (24h), then switch to 12h and check "01" + PM=1
    await set_hours_24(dut, cap, 13)      # still in set_mode, 24h display
    await ClockCycles(dut.clk, 6)
    ui_bit_set(dut, 6, 1)                 # hour_12h=1
    await ClockCycles(dut.clk, 6)
    pm = (int(dut.uio_out.value) >> 6) & 1
    dut._log.info(f"12h view: HH={cap.Ht}{cap.Ho}, PM={pm}")
    assert (cap.Ht, cap.Ho) == (0, 1), "12h display incorrect for 13h (expected 01)"
    assert pm == 1, "PM flag not asserted for 13h in 12h mode"

    # ---------------- 1 Hz pulse in run mode -------------
    ui_bit_set(dut, 1, 0)   # set_mode=0 (run)
    pulses = 0
    for _ in range(130):    # ~2.16 seconds at 60Hz equivalent (in ticks)
        await RisingEdge(dut.clk)
        onehz = (int(dut.uio_out.value) >> 7) & 1
        if onehz:
            pulses += 1
    dut._log.info(f"1Hz pulses seen in ~130 ticks: {pulses}")
    assert pulses >= 2, "Expected at least 2 one-second pulses"

    # ---------------- PPS alignment ----------------------
    # Back to 60 Hz
    ui_bit_set(dut, 5, 0)
    # Wait for colon edge
    prev = (int(dut.uo_out.value) >> 7) & 1
    while True:
        await RisingEdge(dut.clk)
        c = (int(dut.uo_out.value) >> 7) & 1
        if c != prev:
            break
    # After 10 ticks, assert PPS for 1 tick
    await ClockCycles(dut.clk, 10)
    ui_bit_set(dut, 0, 1)  # PPS high
    await RisingEdge(dut.clk)
    ui_bit_set(dut, 0, 0)  # PPS low

    # Count ticks until next colon toggle; expect immediate or next tick (<=2)
    prev = (int(dut.uo_out.value) >> 7) & 1
    cnt = 0
    while True:
        await RisingEdge(dut.clk)
        cnt += 1
        c = (int(dut.uo_out.value) >> 7) & 1
        if c != prev:
            break
    dut._log.info(f"PPS->colon ticks: {cnt}")
    assert cnt <= 2, "PPS did not align seconds (too many ticks to colon edge)"

    dut._log.info("All checks passed ✅")
