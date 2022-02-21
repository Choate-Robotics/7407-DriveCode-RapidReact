from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig
import wpilib

from utils.can_optimizations import optimize_normal_talon, optimize_leader_talon

_MOTOR_CFG = TalonConfig(neutral_brake=False)


class Intake(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(13, inverted=True), TalonFX(14, inverted=False), config=_MOTOR_CFG)
    s_left = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 0, 1)
    s_right = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 2, 3)

    def init(self):
        self.motors.init()
        optimize_leader_talon(self.motors.motors[0])
        optimize_normal_talon(self.motors.motors[1])

    def set(self, motor_speed: float):
        # TODO Velocity control
        self.motors.set_raw_output(motor_speed)

    def toggle_left_intake(self):
        if self.s_left.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.s_left.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.s_left.toggle()

    def toggle_right_intake(self):
        if self.s_right.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.s_right.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.s_right.toggle()
