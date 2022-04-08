from commands2 import InstantCommand, WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand
from subsystem import Index, Intake
from oi.keymap import Keymap
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit
from robotpy_toolkit_7407.unum import Unum
from robot_systems import Robot, Sensors
import commands2
import wpilib
from robot_systems import Robot
from subsystem import Intake
import constants

class BallPath(SubsystemCommand[Index]):
    def __init__(self, subsystem):
        super().__init__(subsystem)
        self.index_speed = 0
        self.dinglebob_direction = "off"
        self.left_intake_direction = "off"
        self.right_intake_direction = "off"

        self.ball_distance = 17600*talon_sensor_unit
        self.desired_distance = None

        self.intake_active_check = True # Set to false when trying to run something for a specified amount of time to avoid cancelling ejection

        self.intake_eject_left = False
        self.intake_eject_right = False

        self.index_shoot = False
        self.index_index = False
        self.index_normal = True

        self.normal = True

        self.operator_index = False

    def initialize(self) -> None:
        pass

    def reactivate_intake_check(self):
        self.intake_active_check = True

    def execute(self) -> None:
        

        if self.intake_active_check:

            self.index_speed = 0
            self.dinglebob_direction = "off"
            self.left_intake_direction = "off"
            self.right_intake_direction = "off"

            if Robot.intake.left_intake_down or Robot.intake.right_intake_down:
                print(f"Left: {Robot.intake.left_intake_down} Right: {Robot.intake.right_intake_down}")
                self.dinglebob_direction = "in"
                Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100]))
                left_color = Sensors.color_sensors.color()
                left_val = Sensors.color_sensors.get_val()
                Sensors.color_sensors.multiplexer.writeBulk(bytes([0b1000]))
                right_color = Sensors.color_sensors.color()
                right_val = Sensors.color_sensors.get_val()
                print(f"Left Color: {left_val}, Right Color: {right_val}, Left: {left_color}, Right: {right_color}")
                if Robot.intake.left_intake_down and left_color != constants.TEAM and left_color != "none":
                    print("EJECT LEFT")
                    self.dinglebob_direction = "eject_right"
                    if Robot.intake.right_intake_down:
                        self.right_intake_direction = "out"
                    self.intake_active_check = False
                    commands2.CommandScheduler.getInstance().schedule(WaitCommand(.5).andThen(self.reactivate_intake_check))
                if Robot.intake.right_intake_down and right_color != constants.TEAM and right_color != "none":
                    print("EJECT RIGHT")
                    self.dinglebob_direction = "eject_left"
                    if Robot.intake.left_intake_down:
                        self.left_intake_direction = "out"
                    self.intake_active_check = False
                    commands2.CommandScheduler.getInstance().schedule(WaitCommand(.5).andThen(self.reactivate_intake_check))

        if Robot.index.ball_queue == 0 and Robot.index.photo_electric.get_value():
            self.desired_distance = Robot.index.motor.get_sensor_position() + self.ball_distance
            self.index_index = True
            self.index_normal = False
            Robot.index.ball_queue += 1
        elif Robot.index.ball_queue == 1 and Robot.index.photo_electric.get_value():
            Robot.index.ball_queue += 1
        elif Robot.index.ball_queue == 2:
            pass
        
        if self.index_index:
            if self.desired_distance <= Robot.index.motor.get_sensor_position():
                self.index_index = False
                self.index_normal = True
                self.index_speed = 0
                self.desired_distance = None
            else:
                self.index_speed = .3
        else:
            self.desired_distance = None
        
        if Robot.shooter.drive_ready and Robot.shooter.shooter_ready: # and Robot.limelight.get_x_offset()!=0 and Robot.limelight.get_x_offset() and abs(Robot.drivetrain.chassis_speeds) < .1:
            self.index_shoot = True
            self.index_index = False
            self.index_normal = False
            self.index_speed = .5
            self.dinglebob_direction = "in"

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
            print("Eject Left")
            Robot.intake.dinglebob_eject_left()
        elif self.dinglebob_direction == "eject_right":
            print("Eject Right")
            Robot.intake.dinglebob_eject_right()
        elif self.dinglebob_direction == "off":
            Robot.intake.dinglebobs_off()

        # print(f"Index Shooting: {self.index_shoot}, Index Indexing: {self.index_index}")
        # print(f"INTAKE ACTIVE CHECK: {self.intake_active_check}")
        # print(f"Dinglebob Direction: {self.dinglebob_direction}")
        # print(f"Index Speed: ", self.index_speed)
        

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass
