"""
I2C bus scanner for the dyno wiring.

Scans the bus directly (devices wired to the Pico, e.g. the TCA9548A
itself) and then walks each mux channel to find devices behind it
(the two LCDs). Run this before main.py to confirm:
  - TCA9548A is reachable at 0x70
  - LCD1602 (AiP31068L) is reachable at 0x3E on channels 0 and 1

Expected output:
  Direct bus        : 0x70
  Mux channel 0     : 0x3E
  Mux channel 1     : 0x3E
  Mux channels 2..7 : (empty)
"""

from machine import Pin, I2C
import time

SDA_PIN = 0
SCL_PIN = 1
MUX_A0  = 5
MUX_A1  = 6
MUX_A2  = 7
MUX_RST = 12
MUX_ADDR = 0x70


def fmt(addrs):
    return ", ".join("0x%02X" % a for a in addrs) if addrs else "(none)"


def scan(i2c, exclude=()):
    found = [a for a in i2c.scan() if a not in exclude]
    return found


def main():
    # Tie mux address pins low so its address is 0x70
    Pin(MUX_A0, Pin.OUT, value=0)
    Pin(MUX_A1, Pin.OUT, value=0)
    Pin(MUX_A2, Pin.OUT, value=0)

    # Pulse reset
    rst = Pin(MUX_RST, Pin.OUT, value=0)
    time.sleep_ms(2)
    rst.value(1)
    time.sleep_ms(2)

    i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400_000)

    # 1) Direct scan (mux disabled, all channels off)
    try:
        i2c.writeto(MUX_ADDR, bytes([0x00]))
    except OSError as e:
        print("WARN: could not deselect mux channels:", e)

    direct = scan(i2c)
    print("Direct bus        :", fmt(direct))
    if MUX_ADDR not in direct:
        print("  ! TCA9548A not found at 0x%02X - check SDA/SCL/RST wiring." % MUX_ADDR)

    # 2) Per-channel scan (devices behind the mux)
    for ch in range(8):
        try:
            i2c.writeto(MUX_ADDR, bytes([1 << ch]))
        except OSError as e:
            print("  ! mux channel %d select failed: %s" % (ch, e))
            continue
        time.sleep_ms(2)
        # Exclude the mux's own address from per-channel results
        devs = scan(i2c, exclude=(MUX_ADDR,))
        print("Mux channel %d     : %s" % (ch, fmt(devs)))

    # Leave mux in a known state
    try:
        i2c.writeto(MUX_ADDR, bytes([0x00]))
    except OSError:
        pass

    print()
    print("Expected: 0x3E on channels 0 and 1 (Waveshare LCD1602, AiP31068L).")


if __name__ == "__main__":
    main()
