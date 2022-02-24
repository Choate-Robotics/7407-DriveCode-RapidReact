from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import deg, s, m

import constants
from subsystem import Shooter


class ShooterDataCollectCommand(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)

        self.angle = 45 * deg
        self.start_vel = 5 * m/s
        self.step_vel = 5 * m/s
        self.end_vel = 25 * m/s
        self.num_trials = 5

        self.ready = False
        self.ready_count = 0
        self.dropped_off = False
        self.data = []
        self.c_data = []
        self.target_vel = self.start_vel
        self.c_min_vel = self.start_vel

    def initialize(self) -> None:
        self.ready = False
        self.ready_count = 0
        self.dropped_off = False
        self.data = []
        self.c_data = []
        self.target_vel = self.start_vel
        self.c_min_vel = self.start_vel

    def execute(self) -> None:
        self.subsystem.set_launch_angle(self.angle)
        self.subsystem.set_flywheels(self.target_vel, self.target_vel)
        v = self.get_motor_vel()

        if not self.ready:
            if abs(v - self.target_vel) < 0.1 * m/s:
                self.ready_count += 1
            else:
                self.ready_count = 0
            if self.ready_count == 10:
                self.ready = True
                logger.info(f"READY {self.target_vel}")
            return

        if self.target_vel - v > 0.3 * m/s:
            self.dropped_off = True
            if self.c_min_vel > v:
                logger.info(f"new min: {v}")
                self.c_min_vel = v
        elif self.dropped_off:
            logger.info(f"GLOBAL MIN: {self.c_min_vel}")
            self.c_data.append(self.c_min_vel)
            self.ready = False
            self.ready_count = 0
            self.dropped_off = False
            if len(self.c_data) == self.num_trials:
                self.data.append(self.c_data)
                self.c_data = []
                self.target_vel += self.step_vel
            self.c_min_vel = self.target_vel

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        if self.target_vel > self.end_vel:
            logger.info(f"FINAL DATA: {self.data}")
            return True
        return False

    def get_motor_vel(self) -> Unum:
        return 0.5 * (
                self.subsystem.m_top.get_sensor_velocity() / constants.shooter_top_gear_ratio +
                self.subsystem.m_bottom.get_sensor_velocity() / constants.shooter_bottom_gear_ratio
        ).asUnit(m/s)


data = {
    5: [4.559135257261849, 4.545272592145895, 4.531409927029942, 4.5517117590784535, 4.525298870514329],
    10: [8.806278521810508, 8.786386827783177, 8.778676232984955, 8.77375457673077, 8.787084062419188],
    15: [12.866891014325565, 12.817797493190074, 12.824728825748052, 12.966390498264335, 14.187658483938142],
    20: [17.040578559280522, 17.21808629484812, 17.212631459166396, 17.233917622465743, 17.49300181044644],
    25: [21.565016139950362, 21.49315995863927, 21.57469539725026, 21.675917460877997, 21.635723934802158]
}
