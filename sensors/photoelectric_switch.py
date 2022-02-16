import wpilib

class Photoelectric_Switch:
    def __init__(self, port):
        self.Photoelectric_Switch = wpilib.DigitalInput(port)
    def get_value(self):
        return self.Photoelectric_Switch.get()