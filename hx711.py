"""
HX711 24-bit ADC bit-bang driver for the SparkFun load cell amp.

Gain selection (extra SCK pulses after the 24 data bits):
  128 -> 1 pulse (channel A)
   32 -> 2 pulses (channel B)
   64 -> 3 pulses (channel A)

IRQs are disabled during the 24-bit shift so the IR sensor edge
interrupt cannot stretch SCK high beyond ~60us (which would put the
HX711 to sleep).
"""

from machine import Pin, disable_irq, enable_irq
import time


class HX711:
    def __init__(self, dt_pin, sck_pin, gain=128):
        self.dout = dt_pin
        self.sck = sck_pin
        self.sck.init(Pin.OUT, value=0)
        self.dout.init(Pin.IN)
        self.OFFSET = 0
        self.SCALE = 1.0
        self.gain_bits = 1
        self.set_gain(gain)

    def set_gain(self, gain):
        if gain == 128:
            self.gain_bits = 1
        elif gain == 64:
            self.gain_bits = 3
        elif gain == 32:
            self.gain_bits = 2

    def is_ready(self):
        return self.dout.value() == 0

    def power_down(self):
        self.sck.value(0)
        self.sck.value(1)
        time.sleep_us(70)  # >60us holds the chip in power-down

    def power_up(self):
        self.sck.value(0)
        time.sleep_ms(1)

    def _read_raw(self):
        irq_state = disable_irq()
        count = 0
        for _ in range(24):
            self.sck.value(1)
            count <<= 1
            self.sck.value(0)
            if self.dout.value():
                count |= 1
        for _ in range(self.gain_bits):
            self.sck.value(1)
            self.sck.value(0)
        enable_irq(irq_state)
        # 24-bit two's complement -> signed
        if count & 0x800000:
            count -= 0x1000000
        return count

    def read(self):
        while not self.is_ready():
            pass
        return self._read_raw()

    def read_average(self, times=10):
        total = 0
        for _ in range(times):
            total += self.read()
        return total // times

    def tare(self, times=15):
        self.OFFSET = self.read_average(times)

    def set_scale(self, scale):
        # scale = raw counts per 1 kg (from calibration)
        self.SCALE = scale if scale else 1.0

    def get_units(self, times=1):
        if times <= 1:
            raw = self._read_raw() if self.is_ready() else self.read()
            return (raw - self.OFFSET) / self.SCALE
        return (self.read_average(times) - self.OFFSET) / self.SCALE
