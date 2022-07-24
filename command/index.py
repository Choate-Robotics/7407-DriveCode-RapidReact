from commands2 import InstantCommand
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors import ctre_motors

from oi.keymap import Keymap
from robot_systems import Robot
from subsystem import Index

IndexOn = lambda: InstantCommand(lambda: Robot.index.set(.5), Robot.index)
IndexOff = lambda: InstantCommand(lambda: Robot.index.set(0), Robot.index)


class IndexAutoDrive(SubsystemCommand):
    def __init__(self, subsystem: Index):
        super().__init__(subsystem)
        self.done = True
        self.subsystem = subsystem
        self.ball_distance = 17600 * ctre_motors.k_sensor_pos_to_radians
        self.desired_distance = None

    def initialize(self):
        pass

    def execute(self):
        speed = 0

        if self.subsystem.photo_electric.get_value():
            match self.subsystem.ball_queue:
                case 0:
                    if self.done:
                        self.done = False
                        self.desired_distance = self.subsystem.motor.get_sensor_position() + self.ball_distance
                        # speed = 0
                        # self.subsystem.motor.set_target_position(self.desired_distance)

                case 1:
                    speed = 0
                    self.subsystem.ball_queue += 1
                case 2:
                    speed = 0

        left_joy = Keymap.Index.INDEX_JOY.value
        if abs(left_joy) < .1:
            pass
        else:
            if left_joy < 0:
                speed = .5
            else:
                speed = -.5
            self.subsystem.ball_queue = 0

        if speed == 0 and not self.done:
            if self.subsystem.motor.get_sensor_position() >= self.desired_distance:
                speed = 0
                self.subsystem.ball_queue += 1
                self.done = True
                self.desired_distance = None
                self.subsystem.set(0)
            else:
                # print(self.subsystem.motor.get_sensor_position(), self.desired_distance)
                # speed = .3
                self.subsystem.set(.3)

        elif Robot.shooter.ready:
            # print("READY")
            # speed = .5
            self.subsystem.set(.5)
            Robot.intake.dinglebobs_in()

        else:
            if speed > 0:
                Robot.intake.dinglebobs_in()
            # else:
            #     Robot.intake.dinglebobs_off()
            self.subsystem.set(speed)

        """
        LOGIC

        If intake running:
            if the photo sensor senser a ball and the ballcount is 0:
                set the ball count to 1
                run the index until it goes a certain amount (need to measure)
                set the index speed to zero
            elif the photo sensor senses a ball and the ballcount is 1:
                set the ball count to 2
                set the index speed to zero
                stop the dinglebobs
            elif the ballcount count is 2 and the photo sensor senses a ball:
                set the index speed to zero
                stop the dinglebobs
            else
                set the index speed to zero
        if shooting
            set the index speed to .5 when the robot is ready to shoot
            if the ball is shot reduce ball count
        if was shooting and now not
            run the index down until the photo senses a ball and then continue
        override by operator controls
                
        """

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted=False):
        pass


class IndexDrive(SubsystemCommand[Index]):
    def __init__(self, subsystem: Index):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.was_on = False

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        left_joy = Keymap.Index.INDEX_JOY.value
        if abs(left_joy) < .1:
            self.subsystem.set(0)
            if self.was_on:
                self.was_on = False
        else:
            if left_joy < 0:
                self.subsystem.set(.5)
            else:
                self.subsystem.set(-.5)
            self.was_on = True

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
