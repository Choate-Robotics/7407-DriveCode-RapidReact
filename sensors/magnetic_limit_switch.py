import wpilib

class Magnetic_Limit_Switch:
    def __init__(self, port):
        self.magnetic_limit_switch = wpilib.DigitalInput(port)
    def get_value(self):
        return self.magnetic_limit_switch.get()