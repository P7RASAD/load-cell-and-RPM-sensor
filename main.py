"""
Hub-motor dynamometer main loop.

LCD1 (mux ch0): applied load in kg + target setpoint.
LCD2 (mux ch1): rotor RPM + computed torque in Nm.

Loop runs at 1 kHz. The HX711 is sampled only when DOUT is ready
(10 SPS hardware rate); the value is smoothed with an EMA so the
integer kg readout stops jittering. RPM is derived from the
pulse-to-pulse period (1 RPM resolution at any speed). Torque is
the sum of the load-cell reaction torque and the rotational-
inertia torque computed from the 30 kg motor mass.
"""

from machine import Pin, I2C, disable_irq, enable_irq
import time

from i2c import LCD1602
from TCA9548A import TCA9548A
from hx711 import HX711


# --- Pin assignments (from project wiring table) ---
SDA_PIN   = 0
SCL_PIN   = 1
HX711_DT  = 2
HX711_SCK = 3
IR_PIN    = 4
MUX_A0    = 5
MUX_A1    = 6
MUX_A2    = 7
BTN_TARE  = 8
BTN_RESET = 9
BTN_INC   = 10
BTN_DEC   = 11
MUX_RST   = 12

# --- System constants ---
IR_SLOTS         = 18         # slots on the encoder wheel
MOTOR_MASS_KG    = 30.0
WHEEL_RADIUS_M   = 0.25       # adjust to actual hub motor radius
G                = 9.81
LOAD_CAL_FACTOR  = 1.0        # raw counts per kg (set after calibration)

LOAD_EMA_ALPHA   = 0.20       # load cell smoothing (0..1, larger = faster)
RPM_TIMEOUT_US   = 1_000_000  # treat as 0 RPM if no pulse for 1 s
IR_DEBOUNCE_US   = 5          # accept pulses spaced > 5 us
MIN_VALID_PERIOD_US = 50      # discard sub-50 us "periods" as noise (>20k RPM)
RPM_CALC_MS      = 20         # recompute RPM every 20 ms (50 Hz)
RPM_EMA_ALPHA    = 0.30       # smoothing applied to the computed RPM
PULSE_BUF_N      = 16         # ring-buffer depth for pulse periods
BTN_DEBOUNCE_MS  = 200
LCD_ADDR         = 0x3E
MUX_ADDR         = 0x70
LCD_CH_LOAD      = 0
LCD_CH_MOTOR     = 1
LOOP_PERIOD_US   = 1000       # 1 ms target


# --- ISR-shared state ---
# Ring buffer of raw pulse periods (us). Smoothing/trimming happens in main
# so a single noisy pulse cannot poison the IIR history.
_ir_last_us  = 0
_pulse_buf   = [0] * PULSE_BUF_N
_pulse_widx  = 0
_pulse_total = 0   # monotonic count of accepted pulses since boot/reset


def _ir_isr(pin):
    global _ir_last_us, _pulse_widx, _pulse_total
    now = time.ticks_us()
    last = _ir_last_us
    if last != 0:
        dt = time.ticks_diff(now, last)
        if dt > IR_DEBOUNCE_US:
            _pulse_buf[_pulse_widx] = dt
            _pulse_widx = (_pulse_widx + 1) % PULSE_BUF_N
            _pulse_total += 1
            _ir_last_us = now
    else:
        _ir_last_us = now


def main():
    global _ir_last_us, _pulse_widx, _pulse_total

    # ---- I2C bus ----
    i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400_000)

    # ---- TCA9548A address pins low, reset high ----
    Pin(MUX_A0, Pin.OUT, value=0)
    Pin(MUX_A1, Pin.OUT, value=0)
    Pin(MUX_A2, Pin.OUT, value=0)
    mux_rst = Pin(MUX_RST, Pin.OUT, value=1)
    mux = TCA9548A(i2c, MUX_ADDR, rst_pin=mux_rst)

    # ---- LCDs ----
    mux.select(LCD_CH_LOAD)
    lcd_load = LCD1602(i2c, LCD_ADDR)
    mux.select(LCD_CH_MOTOR)
    lcd_motor = LCD1602(i2c, LCD_ADDR)

    # ---- HX711 ----
    hx = HX711(Pin(HX711_DT), Pin(HX711_SCK), gain=128)
    hx.set_scale(LOAD_CAL_FACTOR)
    time.sleep_ms(500)            # let the bridge settle
    hx.tare(times=10)

    # ---- IR sensor ----
    ir_pin = Pin(IR_PIN, Pin.IN, Pin.PULL_UP)
    ir_pin.irq(trigger=Pin.IRQ_FALLING, handler=_ir_isr)

    # ---- Buttons (active low) ----
    btn_tare  = Pin(BTN_TARE,  Pin.IN, Pin.PULL_UP)
    btn_reset = Pin(BTN_RESET, Pin.IN, Pin.PULL_UP)
    btn_inc   = Pin(BTN_INC,   Pin.IN, Pin.PULL_UP)
    btn_dec   = Pin(BTN_DEC,   Pin.IN, Pin.PULL_UP)

    # ---- Run-time state ----
    load_ema       = 0.0
    have_load      = False
    load_kg        = 0
    target_kg      = 0
    last_load_disp = None
    last_tgt_disp  = None
    last_rpm_disp  = None
    last_torq_disp = None

    rpm              = 0
    rpm_smooth       = 0.0
    alpha            = 0.0
    last_omega       = 0.0
    last_alpha_us    = time.ticks_us()
    last_rpm_calc_ms = 0

    last_btn_ms = 0

    TWO_PI_OVER_60 = 2 * 3.14159265 / 60.0
    INERTIA       = MOTOR_MASS_KG * WHEEL_RADIUS_M * WHEEL_RADIUS_M  # hoop model

    while True:
        cycle_start = time.ticks_us()

        # ---- Load cell (only when fresh sample is ready) ----
        if hx.is_ready():
            raw = hx._read_raw()
            kg = (raw - hx.OFFSET) / hx.SCALE
            if not have_load:
                load_ema = kg
                have_load = True
            else:
                load_ema = load_ema * (1.0 - LOAD_EMA_ALPHA) + kg * LOAD_EMA_ALPHA
            v = int(round(load_ema))
            load_kg = v if v > 0 else 0

        # ---- RPM update (every RPM_CALC_MS, not every loop) ----
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_rpm_calc_ms) >= RPM_CALC_MS:
            last_rpm_calc_ms = now_ms

            # Snapshot the ring buffer atomically
            irq_state = disable_irq()
            buf_snap   = list(_pulse_buf)
            total      = _pulse_total
            last_pulse = _ir_last_us
            enable_irq(irq_state)

            now_us = time.ticks_us()
            if last_pulse == 0 or time.ticks_diff(now_us, last_pulse) > RPM_TIMEOUT_US:
                # No pulses recently -> motor stopped. Clear state.
                target_rpm = 0
                irq_state = disable_irq()
                _ir_last_us  = 0
                _pulse_total = 0
                enable_irq(irq_state)
            else:
                # Keep only periods that are physically plausible. With 5 us
                # debounce, ringing and double-edges can squeeze in -- those
                # land far below MIN_VALID_PERIOD_US and are dropped here.
                if total >= PULSE_BUF_N:
                    valid = [p for p in buf_snap if p >= MIN_VALID_PERIOD_US]
                else:
                    valid = [p for p in buf_snap[:total] if p >= MIN_VALID_PERIOD_US]

                n = len(valid)
                if n >= 4:
                    # Trimmed mean: drop the smallest and largest 25%. Robust
                    # to single-pulse glitches in either direction without
                    # the lag a long IIR would introduce.
                    valid.sort()
                    cut = n >> 2
                    trimmed = valid[cut:n - cut] if cut else valid
                    avg_period = sum(trimmed) / len(trimmed)
                elif n >= 1:
                    valid.sort()
                    avg_period = valid[n >> 1]   # median
                else:
                    avg_period = 0

                if avg_period > 0:
                    target_rpm = int(60_000_000 / (avg_period * IR_SLOTS))
                else:
                    target_rpm = 0

            # Second-stage smoothing on the RPM scalar itself. This is what
            # kills the visible jumping -- the period buffer handles glitches,
            # the EMA handles the residual ripple.
            if rpm_smooth == 0.0 and target_rpm > 0:
                rpm_smooth = float(target_rpm)
            else:
                rpm_smooth = rpm_smooth * (1.0 - RPM_EMA_ALPHA) + target_rpm * RPM_EMA_ALPHA
            rpm = int(round(rpm_smooth))
            if rpm < 0:
                rpm = 0

            # Angular acceleration (rad/s^2) -- recomputed on the same cadence
            omega = rpm * TWO_PI_OVER_60
            dt_s = time.ticks_diff(now_us, last_alpha_us) / 1_000_000
            alpha = (omega - last_omega) / dt_s if dt_s > 0 else 0.0
            last_omega = omega
            last_alpha_us = now_us

        # ---- Torque (Nm) ----
        # Reaction from load applied through screw:  T_load = m*g*r
        # Rotational inertia of the spinning rotor:  T_inertia = I*alpha
        # Force the output to zero if either the load OR the RPM is zero --
        # a static load with no rotation produces no mechanical torque, and
        # a spinning rotor with no applied load isn't being measured.
        if load_kg == 0 or rpm == 0:
            torque = 0
        else:
            torque_f = load_kg * G * WHEEL_RADIUS_M + INERTIA * alpha
            torque = int(round(torque_f))
            if torque < 0:
                torque = 0

        # ---- Buttons ----
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_btn_ms) > BTN_DEBOUNCE_MS:
            if not btn_tare.value():
                hx.tare(times=10)
                load_ema = 0.0
                have_load = False
                last_btn_ms = now_ms
            elif not btn_reset.value():
                target_kg = 0
                irq_state = disable_irq()
                _ir_last_us  = 0
                _pulse_total = 0
                enable_irq(irq_state)
                rpm = 0
                rpm_smooth = 0.0
                alpha = 0.0
                last_omega = 0.0
                last_btn_ms = now_ms
            elif not btn_inc.value():
                target_kg += 1
                last_btn_ms = now_ms
            elif not btn_dec.value():
                if target_kg > 0:
                    target_kg -= 1
                last_btn_ms = now_ms

        # ---- LCD updates (only on change to keep I2C traffic low) ----
        if load_kg != last_load_disp or target_kg != last_tgt_disp:
            mux.select(LCD_CH_LOAD)
            lcd_load.write_at(0, 0, "Load:   %4d kg" % load_kg)
            lcd_load.write_at(0, 1, "Target: %4d kg" % target_kg)
            last_load_disp = load_kg
            last_tgt_disp  = target_kg

        if rpm != last_rpm_disp or torque != last_torq_disp:
            mux.select(LCD_CH_MOTOR)
            lcd_motor.write_at(0, 0, "RPM:    %5d" % rpm)
            lcd_motor.write_at(0, 1, "Torque: %4d Nm" % torque)
            last_rpm_disp  = rpm
            last_torq_disp = torque

        # ---- Hold 1 kHz cadence ----
        elapsed = time.ticks_diff(time.ticks_us(), cycle_start)
        if elapsed < LOOP_PERIOD_US:
            time.sleep_us(LOOP_PERIOD_US - elapsed)


if __name__ == "__main__":
    main()
