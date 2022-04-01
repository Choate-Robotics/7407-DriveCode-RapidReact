from commands2 import InstantCommand
from robot_systems import Robot, Sensors
import commands2
from robotpy_toolkit_7407.command import SubsystemCommand
import wpilib
from robot_systems import Robot
from subsystem import Intake
from oi.keymap import Keymap
import constants

class IntakeAutoEject(SubsystemCommand):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.ejecting = False

    def initialize(self):
        pass
    def execute(self):

        if Robot.index.ball_queue == 2:
            self.subsystem.dinglebobs_off()
            self.subsystem.toggle_left_intake()
        else:
            print("RUNNNING/n"*3)
            #print("INTAKE: ", self.subsystem.left_intake_down)
            # if len(Robot.shooter.ball_queue) == 2:
            #     self.subsystem.intake_speed = 0
            # else:
            #     self.subsystem.intake_speed = .7
            Sensors.color_sensors.multiplexer.writeBulk(bytes([0b100]))
            left_color = Sensors.color_sensors.color()
            print("Lefty: ", Sensors.color_sensors.get_val())
            print("Left Color:", left_color)

            if left_color != constants.TEAM and left_color != "none":
                self.ejecting = True
                self.subsystem.dinglebob_eject_right()
                wpilib.wait(1.5)
                print("EJECTING")

            Sensors.color_sensors.multiplexer.writeBulk(bytes([0b1000]))
            right_color = Sensors.color_sensors.color()
            print("Righty: ", Sensors.color_sensors.get_val())
            print("Right Color:", right_color)

            if right_color != constants.TEAM and right_color != "none":
                self.ejecting = True
                self.subsystem.dinglebob_eject_left()
                wpilib.wait(1.5)
                print("EJECTING")
                
            elif self.subsystem.left_intake_down and self.subsystem.right_intake_down:
                print("OFF")
                self.subsystem.dinglebobs_off()

            else:
                print("IN")
                self.subsystem.dinglebobs_in()
                #self.subsystem.dinglebob_eject_left()
            
            print("EJECTING: ", self.ejecting)


    def isFinished(self) -> bool:
        return False
    def end(self, interrupted=False):
        pass

class IntakeToggleLeft(SubsystemCommand):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
    def initialize(self):
        pass
    def execute(self):
        self.subsystem.toggle_left_intake()
    def isFinished(self) -> bool:
        return True
    def end(self, interrupted=False):
        commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))

class IntakeToggleRight(SubsystemCommand):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
    def initialize(self):
        pass
    def execute(self):
        self.subsystem.toggle_right_intake()
    def isFinished(self) -> bool:
        return True
    def end(self, interrupted=False):
        commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))

class IntakeDinglebobOn(SubsystemCommand):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
    def initialize(self):
        pass
    def execute(self):
        self.subsystem.dinglebobs_in()
    def isFinished(self) -> bool:
        return True
    def end(self, interrupted=False):
        commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))

class IntakeDinglebobOff(SubsystemCommand):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
    def initialize(self):
        pass
    def execute(self):
        self.subsystem.dinglebobs_off()
    def isFinished(self) -> bool:
        return True
    def end(self, interrupted=False):
        commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))

class IntakeToggleReverse(SubsystemCommand):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
    def initialize(self):
        pass
    def execute(self):
        self.subsystem.dinglebob_eject_right()
    def isFinished(self) -> bool:
        return True
    def end(self, interrupted=False):
        commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))