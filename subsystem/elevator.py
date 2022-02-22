from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig
import wpilib
from robotpy_toolkit_7407.unum import Unum

import constants
from utils.can_optimizations import optimize_leader_talon, optimize_normal_talon


_MOTOR_CFG = TalonConfig(neutral_brake=True)  # TODO add pid constants to this


class Elevator(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(17, inverted=True), TalonFX(18, inverted=False), config=_MOTOR_CFG)
    solenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
    l_climb_1 = wpilib.DigitalInput(0)  # TODO find ids
    l_climb_2 = wpilib.DigitalInput(1)
    l_grab_1 = wpilib.DigitalInput(2)
    l_grab_2 = wpilib.DigitalInput(3)

    def init(self):
        self.motors.init()
        optimize_leader_talon(self.motors.motors[0])
        optimize_normal_talon(self.motors.motors[1])
        self.retract_solenoid()

    def set_height(self, h: Unum):
        self.motors.set_target_position(h * constants.elevator_gear_ratio)

    def get_height(self):
        return self.motors.get_sensor_position() / constants.elevator_gear_ratio

    def extend_solenoid(self):  # TODO Verify solenoid direction
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def retract_solenoid(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def bar_on_climb_hooks(self) -> bool:
        return self.l_climb_1.get() and self.l_climb_2.get()

    def bar_on_grab_hooks(self) -> bool:
        return self.l_grab_1.get() and self.l_grab_2.get()
