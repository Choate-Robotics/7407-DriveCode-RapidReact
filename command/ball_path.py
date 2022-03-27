from robotpy_toolkit_7407 import Command

from robot_systems import Robot, Sensors
from subsystem import Intake, Index, Shooter
import wpilib
import constants
import commands2

from robotpy_toolkit_7407.utils.units import rad, m, s, deg

class BallPathStatus:
    standard = 0
    filled_ejecting = 1
    empty_ejecting = 2
    filled = 3
    waiting = 4


class BallPathControlCommand(Command):
    def __init__(self, intake: Intake, index: Index, shooter: Shooter):
        super().__init__()
        self.addRequirements(intake, index, shooter)
        self.ball_path_state = BallPathStatus.empty
        self.intake = intake
        self.index = index
        self.shooter = shooter

    def initialize(self) -> None:
        self.ball_path_state = BallPathStatus.empty

    def execute(self) -> None:
        match Robot.ball_queue:
            case []:
                self.ball_path_state = BallPathStatus.standard
            case [Robot.team]:
                self.ball_path_state = BallPathStatus.standard
            case [Robot.team, Robot.not_team]:
                self.ball_path_state = BallPathStatus.filled_ejecting
            case [Robot.not_team]:
                self.ball_path_state = BallPathStatus.empty_ejecting
            case [Robot.not_team, Robot.not_team]:
                self.ball_path_state = BallPathStatus.empty_ejecting
            case [Robot.not_team, Robot.team]:
                self.ball_path_state = BallPathStatus.empty_ejecting
            case [Robot.team, Robot.team]:
                self.ball_path_state = BallPathStatus.fille                

        match self.ball_path_state:
            case BallPathStatus.standard:
                self.intake.dinglebobs_in()

            case BallPathStatus.filled_ejecting:
                Sensors.color_sensors.towrite = bytes([0b1000])
                left_color = Sensors.color_sensors.color()

                if left_color == Robot.not_team:
                    self.intake.dinglebob_eject_left()
                    self.ball_path_state = BallPathStatus.waiting
                    def pop_ball():
                        Robot.ball_queue.pop(0)
                        self.ball_path_state = BallPathStatus.standard
                    commands2.CommandScheduler.schedule(commands2.WaitCommand(1.5)
                    .andThen(commands2.InstantCommand(pop_ball)))
                    print("EJECTING LEFT")

                Sensors.color_sensors.towrite = bytes([0b100])
                right_color = Sensors.color_sensors.color()

                if right_color == Robot.not_team:
                    self.intake.dinglebob_eject_right()
                    self.ball_path_state = BallPathStatus.waiting
                    commands2.CommandScheduler.add_command(commands2.WaitCommand(1.5))
                    Robot.ball_queue.pop(0)
                    print("EJECTING RIGHT")

            case BallPathStatus.empty_ejecting:
                self.intake.dinglebobs_in()
                self.shooter.set_launch_angle(75 * deg)
                self.shooter.set_flywheels(.5 * m/s)
                if not self.index.photo_electric.get_value():
                    self.index.set(.5)
                else:
                    self.index.set(0)
                    self.ball_path_state = BallPathStatus.waiting
                    commands2.CommandScheduler.add_command(commands2.WaitCommand(.2))
                    Robot.ball_queue.pop(0)


            case BallPathStatus.filled:
                self.intake.dinglebobs_off()

    def isFinished(self) -> bool:
        return False
