import wpilib

class Limit_Switch:
    def __init__(self, port, reverse=True):
        self.limit_switch = wpilib.DigitalInput(port)
        self.reverse = reverse
    def get_value(self):
        if self.reverse:
            return not self.limit_switch.get()
        return self.limit_switch.get()