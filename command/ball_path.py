from robotpy_toolkit_7407 import Command

from subsystem import Intake, Index


class BallPathStatus:
    empty = 0
    filled_ejecting = 1
    empty_ejecting = 2
    filled = 3


class BallPathControlCommand(Command):
    def __init__(self, intake: Intake, index: Index):
        super().__init__()
        self.addRequirements(intake, index)
        self.ball_path_state = BallPathStatus.empty

    def initialize(self) -> None:
        self.ball_path_state = BallPathStatus.empty

    def execute(self) -> None:
        match self.ball_path_state:
            case BallPathStatus.empty:
                pass
            case BallPathStatus.filled_ejecting:
                pass
            case BallPathStatus.empty_ejecting:
                pass
            case BallPathStatus.filled:
                pass

    def isFinished(self) -> bool:
        return False
