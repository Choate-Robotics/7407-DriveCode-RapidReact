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


class Ball():
    ball_count = 0
    ball = []

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
                if self.ball[i].position == x:
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
        #code break here
        if self.isPos("Left"):
            Index.left_oc = True
        else: # testing only remove all of these in production
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
                    print("Left Blocked")
                elif Index.staged_oc != False:
                    path_clear = "Stage Block"
                    print("Stage Block Left")
            case "Right":
                if Index.right_oc != False:
                    path_clear = "Right"
                    print("Right Blocked")
                elif Index.staged_oc != False:
                    path_clear = "Stage Block"
                    print("Stage Block Right")
            case "Staged":
                if Index.staged_oc != False:
                    path_clear = "Staged"
                    print("Stage Blocked")

        return path_clear

    def disableOC(self, pos):
        match pos:
            case "Left":
                Index.left_oc = False
            case "Right":
                Index.right_oc = False
            case "Staged":
                Index.staged_oc = False
    
    def enableOC(self, pos):
        match pos:
            case "Left":
                Index.left_oc = True
            case "Right":
                Index.right_oc = True
            case "Staged":
                Index.staged_oc = True
    
    def move(self, pos):
        cPos = self.position
        match pos:
            case "Left":
                print("Dinglebobs Left")
                InstantCommand(Robot.index.dinglebobs_control("Left"), Robot.index).withInterrupt(Robot.index.left_limit.get_value()).andThen(self.finMove("Left"))
                #return Left
            case "Right":
                print("Dinglebobs Right")
                InstantCommand(Robot.index.dinglebobs_control("Right"), Robot.index).withInterrupt(Robot.index.right_limit.get_value()).andThen(self.finMove("Right"))
            case "Stage":
                print("Dinglebobs Stage")
                InstantCommand(Robot.index.dinglebobs_control("Stage", cPos), Robot.index).withInterrupt(Robot.index.photo_electric.get_value()).andThen(self.finMove("Stage"))    
                #return Stage
    def finMove(self, nPos):
        self.disableOC(self.position)
        self.position = nPos
        self.enableOC(self.position)
        self.moving = False

    def setPos(self, pos, timeout = 5):
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
            print("False Position")
            return "False Position"
        cPos = self.position
        nPos = pos
        if self.findBalls(nPos) != True:
            print(self.findBalls(nPos))
            return self.findBalls(nPos)
        else:
            print("Moving Dinglebobs")
            self.moving = nPos
            #time = time.time() + timeout
            # if not x:
            self.move(nPos)
                #if time.time() > timeout:
                    #return "Timeout"
            # else:
            return True

    def purge(self):
        if not Index.left_limit.get_value() and not Index.right_limit.get_value() and not Index.photo_electric.get_value():
            Index.dinglebobs_control("Out")

    def remove(self):
        self.removed = True

class BallPath(SubsystemCommand[Index]):
    def __init__(self, subsystem):
        super().__init__(subsystem)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        Ball.ball_count = 0 #onlt for testing -- remove when production
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
            
        # if Ball.ball_count > 0:
        #     i = 0
        #     while i != Ball.ball_count:
        #         if not Ball.ball[i].removed and Ball.ball[i].moving != False:
        #             Ball.ball[i].setPos(Ball.ball[i].moving)
        #         i += 1

        #print(Robot.index.left_limit.get_value())
        if Robot.intake.left_intake_down or Robot.intake.right_intake_down:
            if Robot.intake.left_intake_down:
                if Robot.index.left_limit.get_value() and not Robot.index.left_oc:

                    c = Ball.ball_count
                    Ball.ball.append(Ball("Left"))
                    #print("Lefty: ", Sensors.color_sensors.get_val())
                    #print("Left Color:", left_color)
                    Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100])) #0100 = Left
                    #if ball is team ball then store right, if not then stage ball
                    if left_color != config.TEAM and left_color != "none":
                        print("OPP BALL")
                        Ball.ball[c].team = False
                        if Ball.ball[c].isPos("Staged") != False:
                            self.subsystem.dinglebobs_off()
                    else:
                        print("TEAM BALL")
                        Ball.ball[c].team = True
                        #if not Index.right_limit.get_value(): #not sure about this. will comment out for now
                        Ball.ball[c].setPos("Right")
                Ball.ball_count += 1
            #Robot.intake.dinglebobs_in()
            #Robot.index.left_dinglebob.set_raw_output(-.7)
        else: ...
            #Robot.index.dinglebobs_off()

        # logging.info(f"Color Sensors {left_val, right_val}")

        

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass
