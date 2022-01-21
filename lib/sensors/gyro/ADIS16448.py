from wpilib import ADIS16448_IMU


class GyroADIS16448:
    def __init__(self) -> None:
        self._gyro = ADIS16448_IMU()
        self.__offset = 0

    @property
    def angle(self) -> float:
        return self._gyro.getGyroAngleZ() + self.__offset

    def reset(self):
        self.__offset = self.angle
