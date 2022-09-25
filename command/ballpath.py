from asyncio.log import logger
from cgitb import enable
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
    position: str

    def __init__(self, pos):
        self.position = pos

        if pos == "Left":
            Robot.index.left_oc = True
        elif pos == "Right":
            Robot.index.right_oc = True
        elif pos == "Stage":
            Robot.index.staged_oc = True

    team = True
    removed = False
    moving = False

    def isPos(self, pos):
        if pos != "Shot":
            x = pos
            for i in range(len(self.ball)):
                if self.ball[i].position == x:
                    return i
                else:
                   i += 1
            return False
        else:
            i = 0
            self.balls = []
            for i in range(len(self.ball)):
                if self.ball[i].position == "Shot":
                   self.balls.append(i) 
                   i += 1
            return self.balls

    def posNum(oc: str):
        x = 0
        for i in range(len(Ball.ball)):
            if Ball.ball[i].position == oc:
                x = i
            i += 1
        return x
    
    def newPos(self, pos):
        oPos = self.position
        nPos = pos
        if oPos == "Left":
            Robot.index.left_oc = False
            print("Turning off Left OC")
        elif oPos == "Right":
            Robot.index.right_oc = False

        elif oPos == "Stage":
            Robot.index.staged_oc = False

        print("New Position")
        self.position = nPos
        print(self.position)
        if nPos == "Left":
            Robot.index.left_oc = True

        elif nPos == "Right":
            Robot.index.right_oc = True

        elif nPos == "Stage":
            Robot.index.staged_oc = True
    
    def findBalls(self, nPos):
        path_clear = True
        cPos = self.position
        #code break here
        # if self.isPos("Left"):
        #     Robot.index.left_oc = True
        # else: ...# testing only remove all of these in production
        #     #Robot.index.left_oc = False
        # if self.isPos("Right"):
        #     Robot.index.right_oc = True
        # else: ...
        #     #Robot.index.right_oc = False
        # if self.isPos("Stage"):
        #     Robot.index.staged_oc = True
        # else: ...
        #     #Robot.index.staged_oc = False
        
        match nPos:
            case "Left":
                if Robot.index.left_oc != False and cPos != "Left":
                    path_clear = "Left"
                    print("Left Blocked")
                elif Robot.index.staged_oc != False and cPos != "Stage":
                    path_clear = "Stage Block"
                    print("Stage Block Left")
            case "Right":
                if Robot.index.right_oc != False and cPos != "Right":
                    path_clear = "Right"
                    print("Right Blocked")
                elif Robot.index.staged_oc != False and cPos != "Stage":
                    path_clear = "Stage Block"
                    print("Stage Block Right")
            case "Stage":
                if Robot.index.staged_oc != False and cPos != "Stage":
                    path_clear = "Stage"
                    print("Stage Blocked")

        return path_clear

    def disableOC(self, pos):
        match pos:
            case "Left":
                Robot.index.left_oc = False
            case "Right":
                Robot.index.right_oc = False
            case "Stage":
                Robot.index.staged_oc = False
    
    def enableOC(self, pos):
        match pos:
            case "Left":
                Robot.index.left_oc = True
            case "Right":
                Robot.index.right_oc = True
            case "Stage":
                Robot.index.staged_oc = True
    
    def finMove(self, nPos):
        self.disableOC(self.position)
        self.position = nPos
        self.enableOC(self.position)
        self.moving = False

    def __move(self, pos):
        cPos = self.position
        #self.newPos(pos)
        x: object
        y: object
        match pos:
            case "Left":
                print("Dinglebobs Left")
                InstantCommand(Robot.index.moveBall("Left", cPos), Robot.index)
            case "Right":
                print("Dinglebobs Right")
                InstantCommand(Robot.index.moveBall("Right", cPos), Robot.index)
            case "Stage":
                print("Dinglebobs Stage")
                InstantCommand(Robot.index.moveBall("Stage", cPos), Robot.index)

    def setPos(self, pos, timeout = 5):
        '''
        Moves the ball based on position

        :Param str pos: the new position for the ball

        :Param int timeout: time for timeout on movement (default 5)

        :Raises TypeError: if pos is not a str

        :Raises ValueError: if pos is not: Left, Right, Stage

        LOGIC:

            If path is not Clear #findBalls uses occupation variables in index subsystem

                turn off dinglebobs
                
                return result
            else:
                set moving variable to pos parameter #too tell program where its going when it loops back in the code(juggling)

                sets raw power of dinglebobs #control of specific dinglebob movement in in index subsystem
        '''
        nPos = pos
        if self.findBalls(nPos) != True:
            print(self.findBalls(nPos))
            Robot.index.single_dinglebob_off(self.position)
            return self.findBalls(nPos)
        else:
            self.moving = nPos
            self.__move(nPos)

            return True

    def isDone(self, pos):
        '''
        Checks to see if ball movement is finished

        @Param: pos: the movement variable from the ball

            LOGIC:
            
            grabs movement variable output #where the ball is going originally (Match case) 
            and finds motion ending button/photoelectric sensor

            if that value is pressed:
                turn dinglebobs off #probably change after testing

                set position variable too new position

                set moving variable to False

        '''
        y: object
        match pos:
            case "Left":
                print("Dinglebobs not left yet")
                #y = Robot.index.left_limit
                if Robot.intake.left_intake_down:
                    z = self.position
                    print(z)
                    x = 30000
                    c: str
                    for i in range(len(Ball.ball)):
                        if Ball.ball[i].position == z:
                            x = i
                        i += 1
                    if not x == 30000:
                        if not Robot.index.right_oc:
                            c = "Right"
                            print("Right Not occupied, try there")
                            Ball.ball[x].setPos(c)
                            return
                        elif not Robot.index.staged_oc:
                            c = "Stage"
                            print("Stage not occupied, try there")
                            Ball.ball[x].setPos(c)
                            return
                else:
                    y = Robot.index.left_limit

            case "Right":
                print("Dinglebobs not right yet")
                if Robot.intake.right_intake_down:
                    z = self.position
                    print(z)
                    x = 30000
                    c: str
                    for i in range(len(Ball.ball)):
                        if Ball.ball[i].position == z:
                            x = i
                        i += 1
                    if not x == 30000:
                        if not Robot.index.left_oc:
                            c = "Left"
                            print("Left Not occupied, try there")
                            Ball.ball[x].setPos(c)
                            return
                        elif not Robot.index.staged_oc:
                            c = "Stage"
                            print("Stage not occupied, try there")
                            Ball.ball[x].setPos(c)
                            return
                else:
                    y = Robot.index.right_limit

            case "Stage":
                if Robot.index.aiming:
                    y = Robot.index.photo_electric
                else:
                    y: str
                    z = self.position
                    print(z)
                    x = 30000
                    c: str
                    for i in range(len(Ball.ball)):
                        if Ball.ball[i].position == z:
                            x = i
                        i += 1
                    if not Robot.index.left_oc:
                        c = "Left"
                        print("Left Not occupied, try there")
                        Ball.ball[x].setPos(c)
                        return
                    if not Robot.index.right_oc:
                        c = "Right"
                        print("right not occupied, try there")
                        Ball.ball[x].setPos(c)
                        return
                    else:
                        print("No free destage")
                        y = Robot.index.photo_electric
        if y.get_value():
            print("Limit Reached")
            Robot.index.dinglebobs_off()
            self.newPos(pos)
            self.moving = False
        else:
            print("Limit not reached")
   

    def purge(self):
        if not Robot.index.left_limit.get_value() and not Robot.index.right_limit.get_value() and not Robot.index.photo_electric.get_value():
            Robot.index.dinglebobs_control("Out")

    def remove(self):
        self.removed = True

    def shoot(self):
        x: str
        

class BallPath(SubsystemCommand[Index]):
    def __init__(self, subsystem):
        super().__init__(subsystem)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        #print("Ball Count")
        #print(Ball.ball_count)
        # print("Photoelectric")
        # print(Robot.index.photo_electric.get_value())
        # print("Left:")
        # print(Robot.index.left_oc)
        # print("Right:")
        # print(Robot.index.right_oc)
        #Ball.ball_count = 0 #onlt for testing -- remove when production
        left_joy = Keymap.Index.LEFT_JOY.value
        right_joy = Keymap.Index.RIGHT_JOY.value
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
        
        #Manual Control overide hopefuly it works
            
        if abs(left_joy) > .5:
            if left_joy < .5:
                print("Left Joystick Movement In")
                Robot.index.single_dinglebob_in("Left")
            elif left_joy > .5:
                print("Left Joystick Movement Out")
                Robot.index.single_dinglebob_out("Left")
            else:
                print("Left Joystick movement off")
                Robot.index.single_dinglebob_off("Left")

        elif abs(right_joy) > .5:
            if right_joy < .5:
                print("Right Joystick movement in")
                Robot.index.single_dinglebob_in("Right")
            elif right_joy > .5:
                print("Right Joystick movement out")
                Robot.index.single_dinglebob_out("Right")
            else:
                print("Right Joystick movement off")
                Robot.index.single_dinglebob_off("Right")
        else:

            if len(Ball.ball) > 0:
                '''
                    LOGIC:
                    if number of current balls excede 0:
                        search in each ball object:
                            if ball variable moving is not False:
                                check whether the ball is finished moving

                '''
                for i in range(len(Ball.ball)):
                    if not Ball.ball[i].removed and Ball.ball[i].moving != False:
                        Ball.ball[i].isDone(Ball.ball[i].moving)
                    i += 1
            #if Robot.intake.left_intake_down or Robot.intake.right_intake_down:
            
            if Robot.index.stage:
                print("Stage Balls")
                if  not len(Ball.ball) == 0:
                    if Robot.index.staged_oc:
                        y: str
                        x = Ball.posNum("Stage")
                        if Ball.ball[x].team == False:
                            if Robot.index.left_oc:
                                y = "Left"
                            if Robot.index.right_oc:
                                y = "Right"
                            Ball.ball[x].setPos(y)
                        else:
                            Ball.ball[x].shoot()
                    else:
                        x = 30000
                        if Robot.intake.left_intake_down:
                            x = Ball.posNum("Left")
                            if x:
                                Ball.ball[x].setPos("Stage")
                            else:
                                x = Ball.posNum("Right")
                                if x:
                                    Ball.ball[x].setPos("Stage")
                        elif Robot.intake.right_intake_down:
                            x = Ball.posNum("Right")
                            if x:
                                Ball.ball[x].setPos("Stage")
                            else:
                                x = Ball.posNum("Left")
                                if x:
                                    Ball.ball[x].setPos("Stage")
                        else:
                            x = False
                            if Robot.index.left_oc:
                                x = Ball.posNum("Left")
                            if Robot.index.right_oc:
                                x = Ball.posNum("Right")
                            if not x == 30000:
                                Ball.ball[x].setPos("Stage")
                Robot.index.stage = False
            
            if Robot.index.destageBall:
                if not len(Ball.ball) == 0:
                    print("Destaging Ball")
                    if Robot.index.staged_oc:
                        print("")
                        y: str
                        x = Ball.posNum("Stage")
                        if not Robot.index.left_oc:
                            y = "Left"
                            print("Left Not occupied, try there")
                        if not Robot.index.right_oc:
                            y = "Right"
                            print("right not occupied, try there")
                        else:
                            print("No free destage")
                        Ball.ball[x].setPos(y)
                    else:
                        print("No Ball in Stage")
                Robot.index.destageBall = False

            if Robot.intake.left_intake_down:
                '''
                Logic:
                    If left Intake Down:
                        if left side of index occupied by ball:
                            identify ball number
                            move ball position to right if path clear #no right ball occupation
                        if ball count is not greater than or equal to 2:
                            Run left Dinglebobs #still runs intakes
                        if Left button gets input and no ball occupying left side:
                            get the current ball count
                            create new ball object with left position
                            add 1 to ball count
                            if left color sensor detects not team ball:
                                set ball team to false
                                move ball to stage if path clear #no staged ball occupation
                            else:
                                set ball team to true
                                move ball to right if path clear #no staged ball/no right ball occupation
                    else if:
                        turn off left dinglebob

                    #Vice versa for right side

                '''
                if Robot.index.left_oc:
                    x = Ball.posNum("Left")
                    Ball.ball[x].setPos("Right")
                if not Ball.ball_count >= 2:
                    Robot.index.single_dinglebob_in("Left")
                if Robot.index.left_limit.get_value() and not Robot.index.left_oc:

                    c = Ball.ball_count
                    Ball.ball.append(Ball("Left"))
                    print("Ball count + 1")
                    Ball.ball_count += 1
                    #print("Lefty: ", Sensors.color_sensors.get_val())
                    #print("Left Color:", left_color)
                    Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100])) #0100 = Left
                    #if ball is team ball then store right, if not then stage ball
                    if left_color != config.TEAM and left_color != "none":
                        print("OPP BALL")
                        Ball.ball[c].team = False
                        Ball.ball[c].setPos("Stage")
                    else:
                        print("TEAM BALL")
                        Ball.ball[c].team = True
                        #if not Robot.index.right_limit.get_value(): #not sure about this. will comment out for now
                        if not Robot.index.stage and not Robot.index.staged_oc:
                            Ball.ball[c].setPos("Right")
                        else:
                            Ball.ball[c].setPos("Stage")
                    
            elif not Robot.intake.left_intake_down:
                if len(Ball.ball) > 0:
                    '''
                        LOGIC:
                        if number of current balls excede 0:
                            search in each ball object:
                                if ball variable moving is not False:
                                    check whether the ball is finished moving

                    '''
                    x = 0
                    for i in range(len(Ball.ball)):
                        if not Ball.ball[i].removed and Ball.ball[i].moving != False:
                            x +=1
                            Ball.ball[i].isDone(Ball.ball[i].moving)
                        i += 1
                    if x == 0:
                        Robot.index.single_dinglebob_off("Left")
                        
                else:
                    Robot.index.single_dinglebob_off("Left")

            if Robot.intake.right_intake_down:
                '''
                Logic:
                    If left Intake Down:
                        if left side of index occupied by ball:
                            identify ball number
                            move ball position to right if path clear #no right ball occupation
                        if ball count is not greater than or equal to 2:
                            Run left Dinglebobs #still runs intakes
                        if Left button gets input and no ball occupying left side:
                            get the current ball count
                            create new ball object with left position
                            add 1 to ball count
                            if left color sensor detects not team ball:
                                set ball team to false
                                move ball to stage if path clear #no staged ball occupation
                            else:
                                set ball team to true
                                move ball to right if path clear #no staged ball/no right ball occupation
                    else if:
                        turn off left dinglebob

                    #Vice versa for right side

                '''
                if Robot.index.right_oc:
                    x = Ball.posNum("Right")
                    Ball.ball[x].setPos("Left")
                if not Ball.ball_count >= 2:
                    Robot.index.single_dinglebob_in("Right")
                if Robot.index.right_limit.get_value() and not Robot.index.right_oc:

                    c = Ball.ball_count
                    Ball.ball.append(Ball("Right"))
                    print("Ball count + 1")
                    Ball.ball_count += 1
                    #print("Lefty: ", Sensors.color_sensors.get_val())
                    #print("Left Color:", left_color)
                    Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100])) #0100 = Left
                    #if ball is team ball then store right, if not then stage ball
                    if right_color != config.TEAM and right_color != "none":
                        print("OPP BALL")
                        Ball.ball[c].team = False
                        Ball.ball[c].setPos("Stage")
                    else:
                        print("TEAM BALL")
                        Ball.ball[c].team = True
                        #if not Robot.index.right_limit.get_value(): #not sure about this. will comment out for now
                        Ball.ball[c].setPos("Left")
                    
            elif not Robot.intake.right_intake_down:
                if len(Ball.ball) > 0:
                    '''
                        LOGIC:
                        if number of current balls excede 0:
                            search in each ball object:
                                if ball variable moving is not False:
                                    check whether the ball is finished moving

                    '''
                    x = 0
                    for i in range(len(Ball.ball)):
                        if not Ball.ball[i].removed and Ball.ball[i].moving != False:
                            x +=1
                            Ball.ball[i].isDone(Ball.ball[i].moving)
                        i=+1
                    if x == 0:
                        Robot.index.single_dinglebob_off("Right")
                        
                else:
                    Robot.index.single_dinglebob_off("Right")
            

            if Robot.index.resetBall:
                Ball.ball = []
                Ball.ball_count = 0
                Robot.index.resetBall = False

            

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass


