import wpilib

class Limit_Switch:
    def __init__(self, port):
        self.limit_switch = wpilib.DigitalInput(port)
    def get_value(self):
        return self.limit_switch.get()