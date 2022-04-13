import commands2
import wpilib
from robotpy_toolkit_7407.command import SubsystemCommand

import config
import constants
from robot_systems import Robot
from robot_systems import Sensors
from subsystem import Intake


class IntakeAutoEject(SubsystemCommand):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.ejecting = False
        self.left_check = False

        

    def initialize(self):
        pass
    def execute(self):


        if Robot.index.ball_queue == 2:
            self.subsystem.dinglebobs_off()
        elif self.subsystem.left_intake_down:
            left_color = Sensors.color_sensors.color()
            #print("Lefty: ", Sensors.color_sensors.get_val())
            #print("Left Color:", left_color)
            Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100])) #0100 = Left

            if left_color != config.TEAM and left_color != "none":
                self.ejecting = True
                self.subsystem.dinglebob_eject_right()
                reset = False
                if self.subsystem.left_intake_down:
                    reset = True
                #self.subsystem.left_intake_motor.set_raw_output(0)
                wpilib.wait(.5) #.45
                #if reset:
                #    self.subsystem.left_intake_motor.set_raw_output(self.subsystem.intake_speed)
                ###print("EJECTING")
                active = True
            else:
                self.subsystem.dinglebobs_in()

        elif self.subsystem.right_intake_down:
            right_color = Sensors.color_sensors.color()
            #print("Righty: ", Sensors.color_sensors.get_val())
            #print("Right Color:", right_color)
            Sensors.color_sensors.multiplexer.writeBulk(bytes([0b1000]))

            if right_color != config.TEAM and right_color != "none":
                self.ejecting = True
                self.subsystem.dinglebob_eject_left()
                reset = False
                if self.subsystem.right_intake_down:
                    reset = True
                #self.subsystem.left_intake_motor.set_raw_output(0)
                wpilib.wait(.5) #.45
                #if reset:
                #    self.subsystem.left_intake_motor.set_raw_output(self.subsystem.intake_speed)
                ###print("EJECTING")
                active = True
            else:
                self.subsystem.dinglebobs_in()
        elif not Robot.index.running:
            if not Robot.index.photo_electric.get_value():
                if Robot.intake.dinglebobs_extra:
                    commands2.CommandScheduler.getInstance().schedule(commands2.WaitCommand(.5).andThen(commands2.InstantCommand(self.subsystem.dinglebobs_off)))
                    Robot.intake.dinglebobs_extra = False
                #self.subsystem.dinglebobs_off()
            else:
                self.subsystem.dinglebobs_off()



        # self.left_check = not self.left_check

        # if Robot.index.ball_queue == 2:
        #     self.subsystem.dinglebobs_off()
        #     #self.subsystem.toggle_left_intake()
        # else:
        #     ###print("RUNNNING/n"*3)
        #     ####print("INTAKE: ", self.subsystem.left_intake_down)
        #     # if len(Robot.shooter.ball_queue) == 2:
        #     #     self.subsystem.intake_speed = 0
        #     # else:
        #     #     self.subsystem.intake_speed = .7

        #     active = False

        #     if self.left_check:
                
        #         left_color = Sensors.color_sensors.color()
        #         print("Lefty: ", Sensors.color_sensors.get_val())
        #         print("Left Color:", left_color)
        #         Sensors.color_sensors.multiplexer.writeBulk(bytes([0b1000]))

        #         if left_color != constants.TEAM and left_color != "none":
        #             self.ejecting = True
        #             self.subsystem.dinglebob_eject_right()
        #             reset = False
        #             if self.subsystem.left_intake_down:
        #                 reset = True
        #             #self.subsystem.left_intake_motor.set_raw_output(0)
        #             wpilib.wait(.5) #.45
        #             #if reset:
        #             #    self.subsystem.left_intake_motor.set_raw_output(self.subsystem.intake_speed)
        #             ###print("EJECTING")
        #             active = True
            
        #     elif not self.left_check:

                
        #         right_color = Sensors.color_sensors.color()
        #         print("Righty: ", Sensors.color_sensors.get_val())
        #         print("Right Color:", right_color)
        #         Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100]))

        #         if right_color != constants.TEAM and right_color != "none":
        #             self.ejecting = True
        #             self.subsystem.dinglebob_eject_left()
        #             reset = False
        #             if self.subsystem.right_intake_down:
        #                 reset = True
        #             #self.subsystem.right_intake_motor.set_raw_output(0)
        #             wpilib.wait(.5) #.45
        #             #if reset:
        #             #    self.subsystem.right_intake_motor.set_raw_output(self.subsystem.intake_speed)
        #             active = True

        #     if not active:

        #         if not self.subsystem.left_intake_down and not self.subsystem.right_intake_down:
        #             ###print("OFF")
        #             self.subsystem.dinglebobs_off()

        #         else:
        #             ###print("IN")
        #             self.subsystem.dinglebobs_in()
        #             #self.subsystem.dinglebob_eject_left()
            
        #     ###print("EJECTING: ", self.ejecting)


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
    def end(self, interrupted=False): ...
        #commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))

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
    def end(self, interrupted=False): ...
        #commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))

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
    def end(self, interrupted=False): ...
        #commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))

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
    def end(self, interrupted=False): ...
        #commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))

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