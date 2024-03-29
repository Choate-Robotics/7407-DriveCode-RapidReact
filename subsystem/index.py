from global_config import Dinglebobs
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig

from sensors import LimitSwitch
from utils.can_optimizations import optimize_normal_talon_no_sensor
import time
import constants

_MOTOR_CFG = TalonConfig(neutral_brake=True)


class Index(Subsystem):
    # motor: TalonFX = TalonFX(16, inverted=False, config=_MOTOR_CFG)
    left_dinglebob: TalonFX = TalonFX(22, inverted=False, config=_MOTOR_CFG)
    right_dinglebob: TalonFX = TalonFX(19, inverted=False, config=_MOTOR_CFG)
    photo_electric = LimitSwitch(0)
    left_limit = LimitSwitch(3)
    right_limit = LimitSwitch(5)
    #ctre.BaseTalon.getSupplyCurrent #input
    #ctre.BaseTalon.getStatorCurrent #output
    left_dinglebob_in: bool
    right_dinglebob_in: bool

    left_oc = False
    right_oc = False 
    staged_oc = False
    traffic_oc = False

    RDB = 0

    LDB = 0

    ball_count = 0

    shooting = False

    autoShotToggle = False
    autoShoot = False

    stage = False
    resetBall = False
    destageBall = False
    
    aiming = False

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

        
        self.left_dinglebob_speed = constants.default_index_speed
        self.right_dinglebob_speed = constants.default_index_speed
        self.dinglebob_eject_speed = .8
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
        self.left_dinglebob.set_raw_output(-self.left_dinglebob_speed)
        self.right_dinglebob.set_raw_output(self.right_dinglebob_speed)
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
        self.left_dinglebob.set_raw_output(self.left_dinglebob_speed-.3)
        self.right_dinglebob.set_raw_output(-self.right_dinglebob_speed-.3)
        self.left_dinglebob_in = True
        self.right_dinglebob_in = True

    def isLeftLimit(self):
        if self.left_limit.get_value():
            return True
        return False

    def isRightLimit(self):
        if self.right_limit.get_value():
            return True
        return False

    def dinglebob_eject_left(self):
        self.left_dinglebob.set_raw_output(self.dinglebob_eject_speed)
        self.right_dinglebob.set_raw_output(self.dinglebob_eject_speed)
        self.left_dinglebob_in = False
        self.right_dinglebob_in = True
        # self.left_intake_motor.set_raw_output(-self.intake_speed)

    def dinglebob_travel(self, Dir):
        l: int
        r: int
        if Dir == "Left":
            self.left_dinglebob_in = False
            l = 1
            self.right_dinglebob_in = True
            r = 1
        elif Dir == "Right":
            self.left_dinglebob_in = True
            l = -1
            self.right_dinglebob_in = False
            r = -1
        self.left_dinglebob.set_raw_output(l * self.left_dinglebob_speed)
        self.right_dinglebob.set_raw_output(r * self.right_dinglebob_speed)

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
            self.right_dinglebob.set_raw_output(self.right_dinglebob_speed)
            self.right_dinglebob_in = True
        elif Dir == "Left":
            self.left_dinglebob.set_raw_output(-self.left_dinglebob_speed)
            self.left_dinglebob_in = True
                

    def single_dinglebob_out(self, Dir):
        if Dir == "Right":
            self.right_dinglebob.set_raw_output(-self.right_dinglebob_speed)
            self.right_dinglebob_in = False
        elif Dir == "Left":
            self.left_dinglebob.set_raw_output(self.left_dinglebob_speed)
            self.left_dinglebob_in = False
        elif Dir == "Shoot":
            if not self.left_oc:
                self.left_dinglebob.set_raw_output(self.left_dinglebob_speed)
                self.left_dinglebob_in = False
            elif not self.right_oc:
                self.right_dinglebob.set_raw_output(-self.right_dinglebob_speed)
                self.right_dinglebob_in = False

    def single_dinglebob_off(self, Dir):
        if Dir == "Right":
            self.right_dinglebob_speed = constants.default_index_speed
            self.right_dinglebob.set_raw_output(0)
            self.left_dinglebob_in = False
        elif Dir == "Left":
            self.left_dinglebob_speed = constants.default_index_speed
            self.left_dinglebob.set_raw_output(0)
            self.left_dinglebob_in = False

    def single_dinglebob(self, pos, Dir):
        if Dir == "In":
            self.single_dinglebob_in(pos)
        elif Dir == "Out":
            self.single_dinglebob_out(pos)
        elif Dir == "Off":
            self.single_dinglebob_off(pos)

    def Intake_Staged(self, pos):
        if pos == "Left":
            pos = "Right"
        else:
            pos = "Left"
        self.dinglebob_travel(pos)

    def opp(self, pos):
        if pos == "Left":
            return "Right"
        else:
            return "Left"

    def shoot(self):
        if not self.left_oc:
            self.left_dinglebob_speed = constants.index_shooting_speed
            self.shooting = "Left"
            self.single_dinglebob_in("Left")
        elif not self.right_oc:
            self.right_dinglebob_speed = constants.index_shooting_speed
            self.shooting = "Right"
            self.single_dinglebob_in("Right")

    def dinglebobs_control(self, Dir: str, Pos: str):
        '''
        Control Dinglebobs based on ball location

        :Param str Pos: The current Position of the ball (only needed for staging)

        :Param str Dir: The New position of the ball that it moves too

        :Raises TypeError: if Pos or Dir is not a string

        :Raises ValueError: if Pos str is not: Left, Right, Staged

        :Raises ValueError: if Dir str is not: Left, Right, Stage
        '''
        self.left_dinglebob_speed = constants.default_index_speed
        self.right_dinglebob_speed = constants.default_index_speed
        if Dir == "In" or Dir == "Out":
            if Dir == "In":
                self.dinglebobs_in()
            elif Dir == "Out":
                self.dinglebobs_out()
        elif Dir == "Left" or Dir == "Right":
            if not self.staged_oc:
                print("Turning on motor")
                self.dinglebob_travel(Dir)
            elif Pos == "Stage": # or Pos ==  "Shoot":
                self.single_dinglebob_out(Dir)
        elif Dir == "Stage":
            if Pos == "Shoot":
                self.single_dinglebob_out(Pos)
            else:
                self.single_dinglebob_in(Pos)
        elif Dir == "Shoot":
            self.shoot()

    def moveBall(self, Dir:str, pos:str = "none"):
        self.dinglebobs_control(Dir, pos)

    def intakeBall(self, pos:str, Dir:str):
        '''
        Intaking index ball logic system

        @Param: str pos: Dinglebob Position ["left", "Right"]

        @Param: str Dir: Dinglebob Direction ["In", "Out", "Off"]
        '''
        if self.staged_oc:
            self.left_dinglebob_speed = constants.index_intaking_speed
            self.right_dinglebob_speed = constants.index_intaking_speed
            if self.traffic_oc and Dir == "Out":
                if pos == "Left":
                    self.left_dinglebob_speed = .4
                else:
                    self.right_dinglebob_speed = .4
                self.single_dinglebob(pos, "Out")
            else:
                if not self.left_oc and not self.right_oc:
                    if Dir == "In":
                        self.Intake_Staged(pos)
                    if Dir == "Out":
                        self.dinglebob_travel(pos)
                    if Dir == "Off":
                        self.dinglebobs_off()
        else:
            #self.left_dinglebob_speed = self.dinglebob_speed
            #self.right_dinglebob_speed = self.dinglebob_speed
            self.single_dinglebob(pos, Dir)
        # self.single_dinglebob(pos, Dir)
            


        
            

        

