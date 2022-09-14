from asyncio.log import logger
import logging
import time

import commands2
import wpilib
from commands2 import InstantCommand, ParallelCommandGroup, ConditionalCommand, SequentialCommandGroup, WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit

import config
from oi.keymap import Controllers
from oi.keymap import Keymap
from robot_systems import Robot
from robot_systems import Sensors
from subsystem import Index

ball_count = 0
ball = 0

class Ball():
    ball_count = 0
    ball = 0

    def __init__(self, pos):
        self.position = pos

    team = True
    removed = False
    moving = False


    def isPos(self, pos):
        if pos != "Shot":
            x = pos
            i = 0
            while i < self.ball_count + 1:
                if ball[i].position == x:
                    return i
                else:
                   i += 1
            return False
        else:
            i = 0
            self.balls = []
            while i < self.ball_count + 1:
                if self.ball[i].position == "Shot":
                   self.balls.append(i) 
                   i += 1
            return self.balls

    def findBalls(self, nPos):
        path_clear = True
        if self.isPos("Left"):
            Index.left_oc = True
        else:
            Index.left_oc = False
        if self.isPos("Right"):
            Index.right_oc = True
        else:
            Index.right_oc = False
        if self.isPos("Staged"):
            Index.staged_oc = True
        else:
            Index.staged_oc = False
        
        match nPos:
            case "Left":
                if Index.left_oc != False:
                    path_clear = "Left"
                elif Index.staged_oc != False:
                    path_clear = "Stage Block"
            case "Right":
                if Index.right_oc != False:
                    path_clear = "Right"
                elif Index.staged_oc != False:
                    path_clear = "Stage Block"
            case "Staged":
                if Index.staged_oc != False:
                    path_clear = "Staged"
        if not path_clear:
            return path_clear

    def move(self, pos):
        cPos = self.position
        match pos:
            case "Left":
                Left = InstantCommand(lambda: Robot.index.dinglebobs_control("Left"), Robot.index)
                return Left
            case "Right":
                Right = InstantCommand(lambda: Robot.index.dinglebobs_control("Right"), Robot.index)
                return Right
            case "Stage":
                Stage = InstantCommand(lambda: Robot.index.dinglebobs_control(cPos, "Stage"), Robot.index)        
                return Stage

    def setPos(self, pos, timeout: 5):
        '''
        Moves the ball based on position

        :Param str pos: the new position for the ball

        :Param int timeout: time for timeout on movement (default 5)

        :Raises TypeError: if pos is not a str

        :Raises ValueError: if pos is not: Left, Right, Stage
        '''
        if pos == "Stage":
            x = Index.photo_electric.get_value()
        elif pos == "Left":
            x = Index.left_limit.get_value()
        elif pos == "Right":
            x = Index.right_limit.get_value()
        else:
            return "False Position"
        cPos = self.position
        nPos = pos
        if self.findBalls(nPos) != True:
            return self.findBalls(nPos)
        else:
            self.moving = nPos
            time = time.time() + timeout
            if not x:
                self.move(nPos)
                if time.time() > timeout:
                    return "Timeout"
            else:
                self.position = nPos
                return True

    def purge(self):
        if not Index.left_limit.get_value() and not Index.right_limit.get_value() and not Index.photo_electric.get_value():
            Index.dinglebobs_control("None", "Out")

    
    def remove(self):
        self.removed = True

class BallPath(SubsystemCommand[Index]):
    def __init__(self, subsystem):
        super().__init__(subsystem)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:

        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100]))
        left_color = Sensors.color_sensors.color()
        left_val = Sensors.color_sensors.get_val()
        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b1000]))
        right_color = Sensors.color_sensors.color()
        right_val = Sensors.color_sensors.get_val()

        if left_val[0] != 0 and right_val[0] != 0:
            Sensors.color_sensors.working = True
        else:
            Sensors.color_sensors.working = False
            
        if Ball.ball_count > 0:
            i = 0
            while i < Ball.ball_count + 1:
                if not Ball.ball[i].removed and Ball.ball[i].moving != False:
                    Ball.ball[i].setPos(ball[i].moving)
                i += 1

        #print(Robot.index.left_limit.get_value())
        print(Robot.index.right_limit.get_value())
        if Robot.intake.left_intake_down or Robot.intake.right_intake_down:
            if Robot.intake.left_intake_down:
                if Robot.index.left_limit.get_value():
                    Ball.ball_count += 1
                    c = Ball.ball_count
                    Ball.ball[c] = Ball("Left")
                    #print("Lefty: ", Sensors.color_sensors.get_val())
                    #print("Left Color:", left_color)
                    Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100])) #0100 = Left
                    #if ball is team ball then store right, if not then stage ball
                    if left_color != config.TEAM and left_color != "none":
                        Ball.ball[c].team = False
                        if Ball.isPos("Staged") != False:
                            self.subsystem.dinglebobs_off()
                    else:
                        Ball.ball[c].team = True
                        if not Index.right_limit.get_value():
                            Ball.ball[c].setPos("Right")
            #Robot.intake.dinglebobs_in()
            #Robot.index.left_dinglebob.set_raw_output(-.7)
        else:
            Robot.index.dinglebobs_off()

        # logging.info(f"Color Sensors {left_val, right_val}")

        

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass
