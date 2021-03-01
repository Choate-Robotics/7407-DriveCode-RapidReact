import ctre


class Gyro:
    def __init__(self, can_id: int = 0) -> None:
        self._pigeon = ctre.PigeonIMU(can_id)
        self._initial_heading = 0

    def reset(self, degrees: float = 0):
        self._initial_heading = self._pigeon.getFusedHeading() - degrees

    @property
    def heading(self) -> float:
        return self._pigeon.getFusedHeading() - self._initial_heading
