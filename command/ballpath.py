import commands2
import wpilib
from commands2 import WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand
from subsystem import Index, Intake
from oi.keymap import Keymap, Controllers
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit

import config
from oi.keymap import Keymap
from robot_systems import Robot
from robot_systems import Sensors
from subsystem import Index


class BallPath(SubsystemCommand[Index]):
    def __init__(self, subsystem):
        super().__init__(subsystem)
        self.index_speed = 0
        self.dinglebob_direction = "off"
        self.left_intake_direction = "off"
        self.right_intake_direction = "off"

        self.desired_distance = None

        self.intake_active_check = True # Set to false when trying to run something for a specified amount of time to avoid cancelling ejection

        self.intake_eject_left = False
        self.intake_eject_right = False

        self.intake_force_stop = False

        self.index_shoot = False
        self.index_index = False
        self.index_normal = True
        self.dinglebob_dinglebob = False

        self.normal = True

        self.operator_index = False


    def initialize(self) -> None:
        pass

    def reactivate_intake_check(self):
        self.intake_active_check = True

    def execute(self) -> None:

        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100]))
        left_color = Sensors.color_sensors.color()
        left_val = Sensors.color_sensors.get_val()
        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b1000]))
        right_color = Sensors.color_sensors.color()
        right_val = Sensors.color_sensors.get_val()

        if Robot.intake.dinglebob_run_extend:
            self.dinglebob_direction = "in"

        if (Robot.index.ball_queue == 1 and (left_color != 'none' or right_color != 'none')) or Robot.index.ball_queue == 2: #(Robot.index.ball_queue == 1 and (left_color != 'none' or right_color != 'none')) or
            self.intake_force_stop = True
            self.dinglebob_dinglebob = True
        else:
            self.intake_force_stop = False

        if self.intake_active_check:

            self.index_speed = 0
            self.dinglebob_direction = "off"
            self.left_intake_direction = "off"
            self.right_intake_direction = "off"

            if Robot.intake.left_intake_down or Robot.intake.right_intake_down:
                #print(f"Left: {Robot.intake.left_intake_down} Right: {Robot.intake.right_intake_down}")
                self.dinglebob_direction = "in"
                #print(f"Left Color: {left_val}, Right Color: {right_val}, Left: {left_color}, Right: {right_color}")
                if Robot.intake.left_intake_down and left_color != config.TEAM and left_color != "none":
                    #print("EJECT LEFT")
                    self.dinglebob_direction = "eject_right"
                    if Robot.intake.right_intake_down:
                        self.right_intake_direction = "out"
                    self.intake_active_check = False
                    commands2.CommandScheduler.getInstance().schedule(WaitCommand(.5).andThen(self.reactivate_intake_check))
                if Robot.intake.right_intake_down and right_color != config.TEAM and right_color != "none":
                    #print("EJECT RIGHT")
                    self.dinglebob_direction = "eject_left"
                    if Robot.intake.left_intake_down:
                        self.left_intake_direction = "out"
                    self.intake_active_check = False
                    commands2.CommandScheduler.getInstance().schedule(WaitCommand(.5).andThen(self.reactivate_intake_check))

        if Robot.index.ball_queue == 0 and Robot.index.photo_electric.get_value():
            # self.desired_distance = Robot.index.motor.get_sensor_position() + self.ball_distance
            self.index_index = True
            self.index_normal = False
        elif Robot.index.ball_queue == 1 and Robot.index.photo_electric.get_value():
            Robot.index.ball_queue += 1
        elif Robot.index.ball_queue == 2:
            if Robot.index.photo_electric.get_value():
                wpilib.XboxController(Controllers.DRIVER).setRumble(wpilib.XboxController.RumbleType.kLeftRumble, .4)
                self.dinglebob_direction = "off"
            else:
                Robot.index.ball_queue -= 1

        if Robot.index.ball_queue < 2:
            wpilib.XboxController(Controllers.DRIVER).setRumble(wpilib.XboxController.RumbleType.kLeftRumble, 0)
        elif (left_color != config.TEAM and left_color != 'none') or (right_color != config.TEAM and right_color != 'none'):
            wpilib.XboxController(Controllers.OPERATOR).setRumble(wpilib.XboxController.RumbleType.kLeftRumble, 1)
        else:
            wpilib.XboxController(Controllers.OPERATOR).setRumble(wpilib.XboxController.RumbleType.kLeftRumble, 0)
        
        if self.index_index:
            if not Robot.index.photo_electric.get_value():
                self.index_index = False
                self.index_normal = True
                self.index_speed = 0
                self.desired_distance = None
                Robot.index.ball_queue += 1
            else:
                self.index_speed = .3
        else:
            self.desired_distance = None

        if self.dinglebob_dinglebob:
            if Robot.index.photo_electric.get_value():
                self.dinglebob_dinglebob = False
                self.dinglebob_direction = 'off'
            else:
                self.dinglebob_direction = 'in'

        if Robot.shooter.drive_ready and Robot.shooter.shooter_ready: # and Robot.limelight.get_x_offset()!=0 and Robot.limelight.get_x_offset() and abs(Robot.drivetrain.chassis_speeds) < .1:
            self.index_shoot = True
            self.index_index = False
            self.index_normal = False
            self.index_speed = .5
            self.dinglebob_direction = "in"

        if self.intake_force_stop:
            #print("FORCED OUT")
            Robot.intake.DISABLE_INTAKES = True
            Robot.intake.left_intake_disable()
            Robot.intake.right_intake_disable()
        else:
            Robot.intake.DISABLE_INTAKES = False

        left_joy = Keymap.Index.INDEX_JOY.value

        if abs(left_joy) >= .1:
            if left_joy < 0:
                self.index_normal = False
                self.index_speed = .5
                self.dinglebob_direction = "in"
            else:
                self.index_normal = False
                self.index_speed = -.5
                self.dinglebob_direction = "out"
            self.subsystem.ball_queue = 0

        Robot.index.set(self.index_speed)
        if self.dinglebob_direction == "in":
            Robot.intake.dinglebobs_in()
        elif self.dinglebob_direction == "out":
            Robot.intake.dinglebobs_out()
        elif self.dinglebob_direction == "eject_left":
            #print("Eject Left")
            Robot.intake.dinglebob_eject_left()
        elif self.dinglebob_direction == "eject_right":
            #print("Eject Right")
            Robot.intake.dinglebob_eject_right()
        elif self.dinglebob_direction == "off":
            Robot.intake.dinglebobs_off()
        #print(self.dinglebob_direction)

        #print(Robot.index.ball_queue)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass
