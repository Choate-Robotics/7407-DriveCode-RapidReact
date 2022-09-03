import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig

from utils.can_optimizations import optimize_normal_talon_no_sensor

_MOTOR_CFG = TalonConfig(neutral_brake=False)


class Intake(Subsystem):
    left_intake_motor: TalonFX = TalonFX(13, inverted=False, config=_MOTOR_CFG)
    right_intake_motor: TalonFX = TalonFX(14, inverted=True, config=_MOTOR_CFG)
    left_dinglebob: TalonFX = TalonFX(22, inverted=False, config=_MOTOR_CFG)
    right_dinglebob: TalonFX = TalonFX(19, inverted=False, config=_MOTOR_CFG)
    s_left: wpilib.DoubleSolenoid
    s_right: wpilib.DoubleSolenoid

    left_intake_down: bool
    left_intake_speed: float
    right_intake_down: bool
    right_intake_speed: float
    intake_speed: float
    dinglebob_speed: float
    left_dinglebob_in: bool
    right_dinglebob_in: bool

    intake_camera_left_found: list[list[float]]
    intake_camera_right_found: list[list[float]]

    def init(self):
        self.left_intake_motor.init()
        self.right_intake_motor.init()
        self.left_dinglebob.init()
        self.right_dinglebob.init()
        self.s_left = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 2, 3)
        self.s_right = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
        optimize_normal_talon_no_sensor(self.left_intake_motor)
        optimize_normal_talon_no_sensor(self.right_intake_motor)
        optimize_normal_talon_no_sensor(self.left_dinglebob)
        optimize_normal_talon_no_sensor(self.right_dinglebob)

        self.left_intake_down = False
        self.right_intake_down = False
        self.intake_speed = .7
        self.dinglebob_speed = .7
        self.dinglebob_eject_speed = 1
        self.left_dinglebob_in = True
        self.right_dinglebob_in = True

        self.dinglebobs_extra = False

        self.dinglebob_run_extend = False
        self.DISABLE_INTAKES = False
        self.AUTO_INTAKE = False

        self.intake_camera_left_found = []
        self.intake_camera_right_found = []

    def left_intake_enable(self):
        if not self.DISABLE_INTAKES:
            self.s_left.set(wpilib.DoubleSolenoid.Value.kForward)
            self.left_intake_down = True
            self.left_intake_motor.set_raw_output(self.intake_speed)
        
    def left_intake_disable(self):
        self.dinglebobs_extra = True
        self.s_left.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.left_intake_down = False
        self.left_intake_motor.set_raw_output(0)


    def right_intake_enable(self):
        if not self.DISABLE_INTAKES:
            self.s_right.set(wpilib.DoubleSolenoid.Value.kForward)
            self.right_intake_down = True
            self.right_intake_motor.set_raw_output(self.intake_speed)

    def right_intake_disable(self):
        self.dinglebobs_extra = True
        self.s_right.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.right_intake_down = False
        self.right_intake_motor.set_raw_output(0)

    def toggle_left_intake(self):
        if not self.DISABLE_INTAKES:
            if self.left_intake_down:
                self.dinglebobs_extra = True
                self.s_left.set(wpilib.DoubleSolenoid.Value.kReverse)
                self.left_intake_down = False
                self.left_intake_motor.set_raw_output(0)
            else:
                self.s_left.set(wpilib.DoubleSolenoid.Value.kForward)
                self.left_intake_down = True
                self.left_intake_motor.set_raw_output(self.intake_speed)

        # self.s_right.set(wpilib.DoubleSolenoid.Value.kReverse)
        # self.right_intake_down = False
        # self.right_intake_motor.set_raw_output(0)
        # self.right_intake_on = False
            

    def toggle_right_intake(self):
        if not self.DISABLE_INTAKES:
            if self.right_intake_down:
                self.dinglebobs_extra = True
                self.s_right.set(wpilib.DoubleSolenoid.Value.kReverse)
                self.right_intake_down = False
                self.right_intake_motor.set_raw_output(0)
            else:
                self.s_right.set(wpilib.DoubleSolenoid.Value.kForward)
                self.right_intake_down = True
                self.right_intake_motor.set_raw_output(self.intake_speed)

        # self.s_left.set(wpilib.DoubleSolenoid.Value.kReverse)
        # self.left_intake_down = False
        # self.left_intake_motor.set_raw_output(0)
        # self.left_intake_on = False

    def dinglebobs_in(self):
        self.left_dinglebob.set_raw_output(-self.dinglebob_speed)
        self.right_dinglebob.set_raw_output(self.dinglebob_speed)
        self.left_dinglebob_in = True
        self.right_dinglebob_in = True
        if self.left_intake_down:
            self.left_intake_motor.set_raw_output(self.intake_speed)
        if self.right_intake_down:
            self.right_intake_motor.set_raw_output(self.intake_speed)
        if not self.left_intake_down:
            self.left_intake_motor.set_raw_output(0)
        if not self.right_intake_down:
            self.right_intake_motor.set_raw_output(0)

    def dinglebobs_out(self):
        self.left_dinglebob.set_raw_output(self.dinglebob_speed-.3)
        self.right_dinglebob.set_raw_output(-self.dinglebob_speed-.3)
        self.left_dinglebob_in = True
        self.right_dinglebob_in = True

    def dinglebob_eject_left(self):
        self.left_dinglebob.set_raw_output(self.dinglebob_eject_speed)
        self.right_dinglebob.set_raw_output(self.dinglebob_eject_speed)
        self.left_dinglebob_in = False
        self.right_dinglebob_in = True
        self.left_intake_motor.set_raw_output(-self.intake_speed)

    def dinglebob_eject_right(self):
        self.left_dinglebob.set_raw_output(-self.dinglebob_eject_speed)
        self.right_dinglebob.set_raw_output(-self.dinglebob_eject_speed)
        self.left_dinglebob_in = True
        self.right_dinglebob_in = False
        self.right_intake_motor.set_raw_output(-self.intake_speed)
    
    def dinglebobs_off(self):
        self.right_dinglebob.set_raw_output(0)
        self.left_dinglebob.set_raw_output(0)
        self.right_dinglebob_in = False
        self.left_dinglebob_in = False