import time

from wpilib import I2C, DigitalInput, AnalogInput

_CHAR_TABLE = {
    "0": [0b00111111, 0b00000000],
    "1": [0b00000110, 0b00000000],
    "2": [0b11011011, 0b00000000],
    "3": [0b11001111, 0b00000000],
    "4": [0b11100110, 0b00000000],
    "5": [0b11101101, 0b00000000],
    "6": [0b11111101, 0b00000000],
    "7": [0b00000111, 0b00000000],
    "8": [0b11111111, 0b00000000],
    "9": [0b11101111, 0b00000000],
    " ": [0b00000000, 0b00000000],
    "-": [0b11000000, 0b00000000],
}


class RevDigit:
    def __init__(self):
        self.i2c = I2C(I2C.Port.kMXP, 0x70)
        self.button_a = DigitalInput(19)
        self.button_b = DigitalInput(20)
        self.a_down = False
        self.b_down = False
        self.pot = AnalogInput(3)

        self.routine_names = ["   2", "   3", "   5"]
        self.routine_idx = 0

        self.i2c.writeBulk(bytes([0x21]))
        time.sleep(0.01)
        self.i2c.writeBulk(bytes([0xE0 | 15]))
        time.sleep(0.01)
        self.i2c.writeBulk(bytes([0x81]))
        time.sleep(0.01)

        self._write_str(self.routine_names[self.routine_idx])

    def _write_str(self, s: str):
        data = [0x00]

        for i in [3, 2, 1, 0]:
            data += _CHAR_TABLE[s[i]]

        self.i2c.writeBulk(bytes(data))

    def update(self):
        if not self.button_a.get():
            if not self.a_down:
                self.a_down = True
                self.routine_idx += 1
                if self.routine_idx == len(self.routine_names):
                    self.routine_idx = 0
                self._write_str(self.routine_names[self.routine_idx])
        else:
            self.a_down = False
        if not self.button_b.get():
            if not self.b_down:
                self.b_down = True
                self.routine_idx -= 1
                if self.routine_idx == -1:
                    self.routine_idx = len(self.routine_names) - 1
                self._write_str(self.routine_names[self.routine_idx])
        else:
            self.b_down = False
