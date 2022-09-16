from global_config import Dinglebobs
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig

from sensors import LimitSwitch
from utils.can_optimizations import optimize_normal_talon_no_sensor

import time

_MOTOR_CFG = TalonConfig(neutral_brake=True)


class Index(Subsystem):
    # motor: TalonFX = TalonFX(16, inverted=False, config=_MOTOR_CFG)
    left_dinglebob: TalonFX = TalonFX(22, inverted=False, config=_MOTOR_CFG)
    right_dinglebob: TalonFX = TalonFX(19, inverted=False, config=_MOTOR_CFG)
    photo_electric = LimitSwitch(0)
    left_limit = LimitSwitch(3)
    right_limit = LimitSwitch(5)

    dinglebob_speed: float
    left_dinglebob_in: bool
    right_dinglebob_in: bool

    left_oc: bool
    right_oc: bool
    staged_oc: bool

    

    def init(self):
        #self.motor.init()
        # optimize_normal_talon_no_sensor(self.motor)
        self.left_dinglebob.init()
        self.right_dinglebob.init()
        optimize_normal_talon_no_sensor(self.left_dinglebob)
        optimize_normal_talon_no_sensor(self.right_dinglebob)
        self.ball_queue = 0
        self.running = False
        self.refresh = False

        self.left_oc = False
        self.right_oc = False
        self.staged_oc = False

        self.dinglebob_speed = .5
        self.dinglebob_eject_speed = 1
        self.left_dinglebob_in = True
        self.right_dinglebob_in = True

        self.dinglebobs_extra = False

        self.dinglebob_run_extend = False
        

    def set(self, motor_speed: float):
        # TODO Velocity control
        self.motor.set_raw_output(motor_speed)
        if motor_speed > 0:
            self.running = True
        else:
            self.running = False

    def dinglebobs_in(self):
        self.left_dinglebob.set_raw_output(-self.dinglebob_speed)
        self.right_dinglebob.set_raw_output(self.dinglebob_speed)
        self.left_dinglebob_in = True
        self.right_dinglebob_in = True
        # if self.left_intake_down:
        #     self.left_intake_motor.set_raw_output(self.intake_speed)
        # if self.right_intake_down:
        #     self.right_intake_motor.set_raw_output(self.intake_speed)
        # if not self.left_intake_down:
        #     self.left_intake_motor.set_raw_output(0)
        # if not self.right_intake_down:
        #     self.right_intake_motor.set_raw_output(0)

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
        # self.left_intake_motor.set_raw_output(-self.intake_speed)

    def dinglebob_travel(self, Dir):
        self.left_dinglebob.set_raw_output(self.dinglebob_eject_speed)
        self.right_dinglebob.set_raw_output(self.dinglebob_eject_speed)
        if Dir == "Left":
            self.left_dinglebob_in = False
            self.right_dinglebob_in = True
        elif Dir == "Right":
            self.left_dinglebob_in = True
            self.right_dinglebob_in = False

    def dinglebob_eject_right(self):
        self.left_dinglebob.set_raw_output(-self.dinglebob_eject_speed)
        self.right_dinglebob.set_raw_output(-self.dinglebob_eject_speed)
        self.left_dinglebob_in = True
        self.right_dinglebob_in = False
        # self.right_intake_motor.set_raw_output(-self.intake_speed)

    def dinglebobs_off(self):
        self.right_dinglebob.set_raw_output(0)
        self.left_dinglebob.set_raw_output(0)
        self.right_dinglebob_in = False
        self.left_dinglebob_in = False

    def single_dinglebob_in(self, Dir):
        if Dir == "Right":
            self.right_dinglebob.set_raw_output(self.dinglebob_speed)
        elif Dir == "Left":
            self.left_dinglebob.set_raw_output(-self.dinglebob_speed)

    def single_dinglebob_out(self, Dir):
        if Dir == "Right":
            self.right_dinglebob.set_raw_output(-self.dinglebob_speed)
        elif Dir == "Left":
            self.left_dinglebob.set_raw_output(self.dinglebob_speed)
    
    def dinglebobs_control(self, Dir, Pos = "None"):
        '''
        Control Dinglebobs based on ball location

        :Param str Pos: The current Position of the ball (only needed for staging)

        :Param str Dir: The New position of the ball that it moves too

        :Raises TypeError: if Pos or Dir is not a string

        :Raises ValueError: if Pos str is not: Left, Right, Staged

        :Raises ValueError: if Dir str is not: Left, Right, Stage
        '''
        if Dir == "In" or Dir == "Out":
            if Dir == "In":
                self.dinglebobs_in()
            elif Dir == "Out":
                self.dinglebobs_out()
        elif Dir == "Left" or Dir == "Right":
            if not self.staged_oc:
                self.dinglebob_travel(Dir)
            elif Pos == "Staged":
                self.single_dinglebob_out(Dir)
        elif Dir == "Stage":
            self.single_dinglebob_in(Pos)


