"""
TCA9548A I2C multiplexer driver.

A0/A1/A2 set the slave address. With all three tied low (default in
this project) the address is 0x70. Hold RST low for >5us to reset.
Writing a single byte selects channels via a bitmask: bit n => CHn.
"""

import time


class TCA9548A:
    def __init__(self, i2c, addr=0x70, rst_pin=None):
        self.i2c = i2c
        self.addr = addr
        if rst_pin is not None:
            rst_pin.value(0)
            time.sleep_ms(1)
            rst_pin.value(1)
            time.sleep_ms(1)
        self._current = -1
        self.deselect_all()

    def select(self, channel):
        """Activate exactly one downstream channel (0..7)."""
        if 0 <= channel <= 7:
            mask = 1 << channel
        else:
            mask = 0
        if mask != self._current:
            self.i2c.writeto(self.addr, bytes([mask]))
            self._current = mask

    def select_mask(self, mask):
        """Activate multiple channels via raw 8-bit mask."""
        mask &= 0xFF
        if mask != self._current:
            self.i2c.writeto(self.addr, bytes([mask]))
            self._current = mask

    def deselect_all(self):
        self.i2c.writeto(self.addr, bytes([0x00]))
        self._current = 0
