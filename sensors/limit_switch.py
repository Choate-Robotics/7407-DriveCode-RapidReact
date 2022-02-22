import wpilib


class LimitSwitch:
    def __init__(self, port: int, reverse: bool = True):
        self.limit_switch = wpilib.DigitalInput(port)
        self.reverse = reverse
    
    def get_value(self):
        if self.reverse:
            return not self.limit_switch.get()
        return self.limit_switch.get()
