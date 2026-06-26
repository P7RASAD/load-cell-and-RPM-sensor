"""
LCD1602 I2C driver (Waveshare LCD1602 I2C / AiP31068L compatible).
Default I2C 7-bit address: 0x3E.

Control byte convention:
  0x80 -> single command byte follows
  0x40 -> single data byte follows
"""

import time


class LCD1602:
    CMD_CLEAR       = 0x01
    CMD_HOME        = 0x02
    CMD_ENTRY_MODE  = 0x06  # cursor moves right, no display shift
    CMD_DISPLAY_ON  = 0x0C  # display on, cursor off, blink off
    CMD_FUNC_SET    = 0x38  # 8-bit, 2-line, 5x8 font, IS=0
    CMD_FUNC_SET_IS = 0x39  # 8-bit, 2-line, 5x8 font, IS=1
    CMD_DDRAM       = 0x80
    LINE_OFFSET     = (0x00, 0x40)

    def __init__(self, i2c, addr=0x3E):
        self.i2c = i2c
        self.addr = addr
        self._init_display()

    def _cmd(self, c):
        self.i2c.writeto(self.addr, bytes([0x80, c]))

    def _data(self, d):
        self.i2c.writeto(self.addr, bytes([0x40, d]))

    def _init_display(self):
        time.sleep_ms(50)              # power-on settle
        self._cmd(self.CMD_FUNC_SET)
        time.sleep_us(150)
        self._cmd(self.CMD_FUNC_SET_IS)
        time.sleep_us(50)
        self._cmd(0x14)                # internal OSC frequency
        self._cmd(0x78)                # contrast (low nibble)
        self._cmd(0x5E)                # ICON on, booster on, contrast high
        self._cmd(0x6D)                # follower control
        time.sleep_ms(200)
        self._cmd(self.CMD_FUNC_SET)   # back to IS=0
        self._cmd(self.CMD_DISPLAY_ON)
        self._cmd(self.CMD_CLEAR)
        time.sleep_ms(2)
        self._cmd(self.CMD_ENTRY_MODE)

    def clear(self):
        self._cmd(self.CMD_CLEAR)
        time.sleep_ms(2)

    def home(self):
        self._cmd(self.CMD_HOME)
        time.sleep_ms(2)

    def set_cursor(self, col, row):
        if row > 1:
            row = 1
        self._cmd(self.CMD_DDRAM | (col + self.LINE_OFFSET[row]))

    def write(self, text):
        for ch in text:
            self._data(ord(ch))

    def write_at(self, col, row, text):
        # Pads to 16 chars so stale characters are overwritten.
        self.set_cursor(col, row)
        if len(text) < 16 - col:
            text = text + " " * (16 - col - len(text))
        else:
            text = text[: 16 - col]
        self.write(text)
