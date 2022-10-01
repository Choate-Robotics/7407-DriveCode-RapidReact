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

    leftPress = False
    ll = 0
    rightPress = False
    rl = 0
    stagePress = False
    sl = 0

    leftInvalid = False
    li = 0
    rightInvalid = False
    ri = 0
    stageInvalid = False
    sl = 0

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
            if not Ball.ball[i].removed:
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
        elif nPos == "Shoot":
            pass
    
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

    def intakeInvalid(self, y):
        match y:
            case "Left":
                self.leftInvalid = True
                print("Left Invalid")
            case "Right":
                self.rightInvalid = True
                print("Right Invalid")
            case "Stage":
                pass
    def __move(self, pos):
        cPos = self.position
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
            case "Shoot":
                print("Dinglebobs Shoot")
                y: str
                if not Robot.index.left_oc:
                    y = "Left"
                    print("Left Not occupied, try there")
                if not Robot.index.right_oc:
                    y = "Right"
                    print("right not occupied, try there")
                else:
                    y = "Left"
                InstantCommand(Robot.index.moveBall("Shoot", y), Robot.index)
        self.newPos(pos)

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
        if self.removed == False:
            nPos = pos
            if self.findBalls(nPos) != True:
                print(self.findBalls(nPos))
                Robot.index.single_dinglebob_off(self.position)
                return self.findBalls(nPos)
            else:
                if not self.moving:
                    y: object
                    match self.position:
                        case "Left":
                            y = Robot.index.left_limit
                        case "Right":
                            y = Robot.index.right_limit
                        case "Stage":
                            y = Robot.index.photo_electric
                        case "Shoot":
                            pass
                    if self.position != "Shoot":
                        if not y.get_value():
                            self.intakeInvalid(self.position)
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
                y = Robot.index.left_limit

            case "Right":
                print("Dinglebobs not right yet")
                y = Robot.index.right_limit

            case "Stage":
                # if Robot.index.aiming:
                print("Stage not there yet")
                y = Robot.index.photo_electric
            case "Shoot":
                y = Robot.index.photo_electric
        if pos != "Shoot":
            if y.get_value():
                print("Limit Reached")
                Robot.index.dinglebobs_off()
                # self.newPos(pos)
                self.moving = False
            else:
                print("Limit not reached")
        else:
            if not y.get_value():
                print("Limit Reached")
                Robot.index.dinglebobs_off()
                self.ball_count -= 1
                self.removed = True
                self.moving = False
            else:
                print("Limit not reached")
   

    def purge(self):
        if not Robot.index.left_limit.get_value() and not Robot.index.right_limit.get_value() and not Robot.index.photo_electric.get_value():
            Robot.index.dinglebobs_control("Out")

    def remove(self):
        self.removed = True

        

class BallPath(SubsystemCommand[Index]):
    def __init__(self, subsystem):
        super().__init__(subsystem)

    def moveBall(self):
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

    def checkLimit(self):
        #Checks if button is still being pressed or if it is switching states -> let intakes know if new ball or old ball
        if Ball.leftPress:
            if Ball.ll < 2:
                if not Robot.index.left_limit.get_value():
                    Ball.ll += 1
            else:
                Ball.ll = 0
                Ball.leftPress = False

        if Ball.rightPress:
            if Ball.rl < 2:
                if not Robot.index.right_limit.get_value():
                    Ball.rl += 1
            else:
                Ball.rl = 0
                Ball.rightPress = False

        if Ball.stagePress:
            if Ball.sl < 2:
                if not Robot.index.right_limit.get_value():
                    Ball.sl += 1
            else:
                Ball.sl = 0
                Ball.stagePress = False
        
        if Ball.leftInvalid:
            if Ball.li < 25:
                if Robot.index.left_limit.get_value():
                    Ball.li += 1
            else:
                Ball.li = 0
                Ball.leftInvalid = False

        if Ball.rightInvalid:
            if Ball.rl < 25:
                if Robot.index.right_limit.get_value():
                    Ball.rightInvalid = False
            else:
                Ball.li = 0
                Ball.rightInvalid = False

    def shooting(self, auto: bool):
        if Robot.index.stage:
            Robot.index.stage = False
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
                        y = 0
                        x = False
                        if Robot.index.left_oc:
                            y += 1
                            x = Ball.posNum("Left")
                        if Robot.index.right_oc:
                            y += 1
                            x = Ball.posNum("Right")
                        if not x == 30000:
                            Ball.ball[x].setPos("Stage")
                            if y < 1:
                                Robot.index.stage = True                          
        
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

        if Robot.shooter.ready:
            if Robot.index.staged_oc and Robot.index.photo_electric.get_value():
                x = Ball.posNum("Stage")
                Ball.ball[x].setPos("Shoot")
            else:
                if auto:
                    #Shooting automatically
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

    def leftIntake(self):
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
                Ball.leftPress = True
            else:
                if not Robot.index.left_oc and not Robot.index.right_oc:
                    Robot.index.single_dinglebob_in("Left")

                    if Robot.index.left_limit.get_value() and not Robot.index.left_oc and not Ball.leftPress and not Ball.leftInvalid:

                        c = Ball.ball_count
                        Ball.ball.append(Ball("Left"))
                        print("Ball count + 1")
                        Ball.ball_count += 1
                        Ball.leftPress = True
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

    def rightIntake(self):
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
                Ball.rightPress = True

            else:
                if not Robot.index.left_oc and not Robot.index.right_oc:
                    Robot.index.single_dinglebob_in("Right")
                    if Robot.index.right_limit.get_value() and not Robot.index.right_oc and not Ball.rightPress and not Ball.rightInvalid:

                        c = Ball.ball_count
                        Ball.ball.append(Ball("Right"))
                        print("Ball count + 1")
                        Ball.ball_count += 1
                        Ball.rightPress = True
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
    
    def resetBall(self):
        if Robot.index.resetBall:
            Ball.ball = []
            Ball.ball_count = 0
            Robot.index.resetBall = False
            Robot.index.left_oc = False
            Robot.index.right_oc = False
            Robot.index.staged_oc = False

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        #print("Ball Count")
        # print(Ball.ball_count)
        wpilib.SmartDashboard.putNumber("Ball Count", Ball.ball_count)
        wpilib.SmartDashboard.putBoolean("Left Side Occupied?", Robot.index.left_oc)
        wpilib.SmartDashboard.putBoolean("Right Side Occupied?", Robot.index.right_oc)
        wpilib.SmartDashboard.putBoolean("Stage Side Occupied?", Robot.index.staged_oc)
        wpilib.SmartDashboard.putBoolean("Left Invalid", Ball.leftInvalid)
        wpilib.SmartDashboard.putBoolean("Right Invalid", Ball.rightInvalid)
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

        if abs(left_joy) > .5 or abs(right_joy) > .5:
            Ball.ball = []
            Ball.ball_count = 0
            Robot.index.resetBall = False
            Robot.index.left_oc = False
            Robot.index.right_oc = False
            Robot.index.staged_oc = False  
            if abs(left_joy) > .5:
                if left_joy < .5:
                    print("Left Joystick Movement In")
                    Robot.index.single_dinglebob_in("Left")
                    Robot.intake.left_intake_motor.set_raw_output(-(left_joy))
                elif left_joy > .5:
                    print("Left Joystick Movement Out")
                    Robot.index.single_dinglebob_out("Left")
                    Robot.intake.left_intake_motor.set_raw_output(-(left_joy))
            else:
                print("Left Joystick movement off")
                Robot.index.single_dinglebob_off("Left")
                Robot.intake.left_intake_motor.set_raw_output(0)

            if abs(right_joy) > .5:
                if right_joy < .5:
                    print("Right Joystick movement in")
                    Robot.index.single_dinglebob_in("Right")
                    Robot.intake.right_intake_motor.set_raw_output(-(right_joy))
                elif right_joy > .5:
                    print("Right Joystick movement out")
                    Robot.index.single_dinglebob_out("Right")
                    Robot.intake.right_intake_motor.set_raw_output(-(right_joy))
            else:
                print("Right Joystick movement off")
                Robot.index.single_dinglebob_off("Right")
                Robot.intake.right_intake_motor.set_raw_output(0)
            
        else:
            self.checkBall() #checks if balls are moving
            
            self.checkLimit() #checks limits for exceptions
            
            self.shooting(False) #if shooting (False or True for automatic shooting)

            self.leftIntake() #if left intake is down

            self.rightIntake() #if right intake is down

            self.resetBall() #if reset ball button is pressed         


    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass


