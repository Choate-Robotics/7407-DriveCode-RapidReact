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
import constants
from oi.keymap import Controllers
from oi.keymap import Keymap
from robot_systems import Robot
from robot_systems import Sensors
from subsystem import Index

pdh = wpilib.PowerDistribution()

class Ball():
    #if debugging necessary, use variables if can be

    Lval = 2 #Validation # threshold for singlepress limit (Change for wider/shorter range of error)

    IVal = 25 # Validation # threshold for ball generation disability for moving ball (Change for wider/shorter range of error)

    #General Variables
    ball = [] #list of stored ball objects
    aimed = False
    #Ball Specific Variables
    position: str #position of ball (While moving, position is set to the future position, IE: Ball right moving left, Ball position is Left)
    lastPosition: str #Last Position
    lastSidePosition: str #Last Side Position [Left, Right]
    team = True #if ball is on team side
    removed = False #if ball is shot/pooped/etc
    moving = False #if ball is currently moving
    waiting = False
    longLeft = 0
    longRight = 0


    def __init__(self, pos:str):
        #Sets position and occupation\
        if pos != None:
            self.position = pos
            self.lastPosition = None
            if pos == "Left":
                Robot.index.left_oc = True
            elif pos == "Right":
                Robot.index.right_oc = True
            elif pos == "Stage":
                Robot.index.staged_oc = True


    #Variables used in Check balls

    #Lval validation # variable used below
    leftPress = False #if Left limit is pressed -> makes sure that new ball is not generated by same ball
    ll = 0 #varlidation index -> goes up every time Left Limit is not pressed -> resets to 0 after certain # reached
    rightPress = False # if Left limit is pressed -> makes sure that new ball is not generated by same ball
    rl = 0 #validation index -> goes up every time Right Limit is not pressed -> resets to 0 after certain # reached

    #IVal Validation # variable used below
    #if Ball is not currently pressing limit button and attempting to move -> makes sure that new ball is not generated by moving ball
    leftInvalid = False 
    li = 0 # Validation index
    #if Ball is not currently pressing limit button and attempting to move -> makes sure that new ball is not generated by moving ball
    rightInvalid = False
    ri = 0 # Validation index

    ES = 0

    PH = 0 #Photo electric int

    CPU = 0 #Controller Pulse Up int

    CPD = 0 #Controller Pulse Down int

    aimCoolDown = False # aim Cool down

    ACDT = 0 # aim cool down threshold

    def reset(self):
        self.ball = []
        Robot.index.ball_count = 0
        Robot.index.resetBall = False
        Robot.index.left_oc = False
        Robot.index.right_oc = False
        Robot.index.staged_oc = False
        self.waiting = False
        Robot.index.traffic_oc = False
        Robot.index.dinglebobs_off()
    
    def posNum(self, oc: str):
        '''
        Gets index number for ball in position -> used for finding index to move ball/change variable

        :Param oc str: position to select the ball ("Left","Right","Stage")
        '''
        x = 0
        for i in range(len(self.ball)):
            if not self.ball[i].removed:
                if self.ball[i].position == oc:
                    x = i
            i += 1
        return x

    def RemovedNum(self):
        '''
        Gets the total number of balls removed/Shot
        '''
        x = 0
        for i in range(len(self.ball)):
            if self.ball[i].removed:
                x += 1
            i += 1
        return x
    
    def CurrentNum(self):
        x = 0
        if Robot.index.left_oc:
            x += 1
        if Robot.index.right_oc:
            x += 1
        if Robot.index.staged_oc:
            x += 1
        return x
    
    def rumble(self):
        x = self.CurrentNum()
        if x > 1:
            wpilib.XboxController(Controllers.OPERATOR).setRumble(wpilib.XboxController.RumbleType.kRightRumble, 1)
        else:
            wpilib.XboxController(Controllers.OPERATOR).setRumble(wpilib.XboxController.RumbleType.kRightRumble, 0)
    
    def newPos(self, pos):
        '''
        Changes current ball position and occupation to new position and occupation

        :Param pos str: New position for the ball
        '''
        oPos = self.position
        self.lastPosition = oPos
        if oPos != "Stage":
            self.lastSidePosition = oPos
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
    
    def pathClear(self, nPos):
        '''
        Checks to see if ball can move to specified position

        :Param nPos str: position the ball is attempting to go

        Returns True if can move

        Returns "Left", "Right", or "Stage" if blocked by those positions

        '''
        path_clear = True
        cPos = self.position       
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
            case "Shoot":
                pass

        return path_clear

    def __intakeInvalid(self, y):
        '''
        Disables new ball generation for intake

        Private Variable

        :Param y str: position relative to intake
        '''
        match y:
            case "Left":
                self.leftInvalid = True
                print("Left Invalid")
            case "Right":
                self.rightInvalid = True
                print("Right Invalid")
            case "Stage":
                pass
    
    def __intakeGeneration(self):
        if not self.moving and self.position != "Shoot":    
            y: object
            match self.position:
                case "Left":
                    y = Robot.index.left_limit
                case "Right":
                    y = Robot.index.right_limit
                case "Stage":
                    y = Robot.index.photo_electric
            if not y.get_value():
                self.__intakeInvalid(self.position)

    def __move(self, pos):
        '''
        Starts dinglebob raw power through instant command

        Private Method

        :Param pos str: new position of ball
        '''
        cPos = self.position
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
                # Robot.index.shooting = True
                # y: str
                # if not Robot.index.left_oc:
                #     y = "Left"
                #     print("Left Not occupied, try there")
                # if not Robot.index.right_oc:
                #     y = "Right"
                #     print("right not occupied, try there")
                # else:
                #     y = "Left"
                InstantCommand(Robot.index.moveBall("Shoot", cPos), Robot.index)
        self.newPos(pos)

    def __traffic(self, pos):
        if self.moving != False and not self.waiting:
            Robot.index.traffic_oc = False
        if Robot.index.traffic_oc:
            self.waiting = True
            self.moving = pos
            return False
        elif pos == "Shoot":
            if not Robot.shooter.ready:
                self.waiting = True
                self.moving = pos
                return False
        else:
            self.waiting = False
            self.moving = False
            return True
    
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
        if pos == "No Num":
            return False
        else:
            if not self.removed:
                nPos = pos
                if self.__traffic(nPos) == False:
                    return self.__traffic(nPos)
                elif self.pathClear(nPos) != True:
                    Robot.index.single_dinglebob_off(self.position)
                    return self.pathClear(nPos)
                else:
                    self.__intakeGeneration()
                    Robot.index.traffic_oc = True
                    self.moving = nPos
                    self.__move(nPos)
                    return True

    def isDone(self, pos):
        '''
        Checks to see if ball movement is finished

        :Param str pos: the movement variable from the ball

            LOGIC:
            
            grabs movement variable output #where the ball is going originally (Match case) 
            and finds motion ending button/photoelectric sensor

            if that value is pressed:
                turn dinglebobs off #probably change after testing

                set position variable too new position

                set moving variable to False

        '''
        if pos == "Left" or pos == "Right":
            y: object
            match pos:
                case "Left":
                    print("Dinglebobs not left yet")
                    y = Robot.index.left_limit

                case "Right":
                    print("Dinglebobs not right yet")
                    y = Robot.index.right_limit
            if y.get_value():
                print("Limit Reached")
                Robot.index.dinglebobs_off()
                # self.newPos(pos)
                self.moving = False
                Robot.index.traffic_oc = False
            else:
                print("Limit not reached")
        elif pos == "Stage":
            print("Stage not there yet")
            y = Robot.index.photo_electric
            if y.get_value():
                if self.PH > 3:
                    Robot.index.dinglebobs_off()
                    self.moving = False
                    Robot.index.traffic_oc = False
                    self.PH = 0
                else:
                    self.PH += 1

        elif pos == "Shoot":
            # if Robot.shooter.ready:
            #     Scurrent = pdh.getCurrent(11)
            #     if Scurrent > 10:
            #         Robot.index.single_dinglebob_off(Robot.index.shooting)
            #         self.removed = True
            #         self.moving = False
            #         Robot.index.shooting = False
            # else:
            #     Robot.index.single_dinglebob_off(Robot.index.shooting)
            #     self.setPos(self.lastPosition)

            Scurrent = pdh.getCurrent(11)
            if Scurrent > 10:
                Robot.index.single_dinglebob_off(Robot.index.shooting)
                self.removed = True
                self.moving = False
                Robot.index.traffic_oc = False
                Robot.index.shooting = False
            

   
    def purge(self):
        '''
        Purges all balls down and out of system
        '''
        if not Robot.index.left_limit.get_value() and not Robot.index.right_limit.get_value() and not Robot.index.photo_electric.get_value():
            Robot.index.dinglebobs_control("Out")
        self.reset()

    def remove(self):
        '''
        Removes ball
        '''
        self.removed = True
      

class BallPath(SubsystemCommand[Index]):
    def __init__(self, subsystem):
        super().__init__(subsystem)
        self.BallController = Ball(None)
        self.pdh = pdh

    def sensorCheck(self, left_val, right_val):
        if left_val[0] != 0 and right_val[0] != 0:
            Sensors.color_sensors.working = True
        else:
            Sensors.color_sensors.working = False
    
    def operatorControl(self, left_joy, right_joy):
        '''
        Operator control Overide

        :Param object left_joy: joystick for left Dinglebob

        :Param object right_joy: Joystick for right Dinglebob
        '''
        self.BallController.reset()  
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
    
    def checkBall(self):
        '''
        Rumbles Controller for feedback
        checks if balls are moving within system -> checks if balls have reached its destination
        '''
        self.BallController.rumble()
        if len(self.BallController.ball) > 0:
            '''
                LOGIC:
                if number of current balls excede 0:
                    search in each ball object:
                        if ball variable moving is not False:
                            check whether the ball is finished moving

            '''
            for i in range(len(self.BallController.ball)):
                if not self.BallController.ball[i].removed and self.BallController.ball[i].moving != False:
                    if self.BallController.ball[i].waiting:
                        self.BallController.ball[i].setPos(self.BallController.ball[i].moving)
                    elif not self.BallController.ball[i].waiting:
                        self.BallController.ball[i].isDone(self.BallController.ball[i].moving)
                i += 1

    def checkLimit(self):
        '''
        checks limits for exceptions to intaking rules -> enables or disables booleans for Intakes to generate a new ball
        '''
        if self.BallController.leftPress:
            if self.BallController.ll < self.BallController.Lval:
                if not Robot.index.left_limit.get_value():
                    self.BallController.ll += 1
            else:
                self.BallController.ll = 0
                self.BallController.leftPress = False

        if self.BallController.rightPress:
            if self.BallController.rl < self.BallController.Lval:
                if not Robot.index.right_limit.get_value():
                    self.BallController.rl += 1
            else:
                self.BallController.rl = 0
                self.BallController.rightPress = False

        
        if self.BallController.leftInvalid:
            if self.BallController.li < self.BallController.IVal:
                if Robot.index.left_limit.get_value():
                    self.BallController.li += 1
            else:
                self.BallController.li = 0
                self.BallController.leftInvalid = False

        if self.BallController.rightInvalid:
            if self.BallController.ri < self.BallController.IVal:
                if Robot.index.right_limit.get_value():
                    self.BallController.rightInvalid = False
            else:
                self.BallController.li = 0
                self.BallController.rightInvalid = False
        

    def currentSensing(self, enabled):
        if enabled:
            if Robot.index.left_oc and Robot.intake.left_intake_down:
                lIcurrent = self.pdh.getCurrent(4)
                if lIcurrent > 10 and not Robot.intake.left_current:
                    Robot.intake.left_current = True
                    Robot.intake.left_intake_speed = 0
                elif lIcurrent < 10 and Robot.intake.left_current:
                    Robot.intake.left_current = False
                    Robot.intake.left_intake_speed = constants.default_intake_speed
            else:
                Robot.intake.left_current = False
                Robot.intake.left_intake_speed = constants.default_intake_speed

            if Robot.index.right_oc and Robot.intake.right_intake_down:
                lRcurrent = self.pdh.getCurrent(3)
                if lRcurrent > 10 and not Robot.intake.right_current:
                    Robot.intake.right_current = True
                    Robot.intake.right_intake_speed = 0
                elif lRcurrent < 10 and Robot.intake.right_current:
                    Robot.intake.right_current = False
                    Robot.intake.right_intake_speed = constants.default_intake_speed
            else:
                Robot.intake.right_current = False
                Robot.intake.right_intake_speed = constants.default_intake_speed
    
    def shooting(self, auto: bool):
        '''
        if shooting 
        
        :Param bool auto: auto shooting enable or disable

        uses shooter.ready to shoot balls that are staged
        '''
        ball = self.BallController
        def staging(ball):
            Robot.index.stage = False
            print("Stage Balls")
            if len(ball.ball) != 0:
                if Robot.index.staged_oc:
                    y: str
                    x = ball.posNum("Stage")
                    if ball.ball[x].team == False:
                        if not Robot.index.left_oc:
                            y = "Left"
                        if not Robot.index.right_oc:
                            y = "Right"
                        self.BallController.ball[x].setPos(y)
                else:
                    if Robot.index.left_oc:
                        x = ball.posNum("Left")
                        ball.ball[x].setPos("Stage")
                    elif Robot.index.right_oc:
                        x = ball.posNum("Right")
                        ball.ball[x].setPos("Stage") 

        def destaging(ball):
            if not len(ball.ball) == 0:
                print("Destaging Ball")
                if Robot.index.staged_oc:
                    y: str
                    x = ball.posNum("Stage")
                    if Robot.intake.left_intake_down:
                        y = "Right"
                    elif Robot.intake.right_intake_down:
                        y = "Left"
                    else:
                        if not Robot.index.left_oc:
                            y = "Left"
                            print("Left Not occupied, try there")
                        elif not Robot.index.right_oc:
                            y = "Right"
                            print("right not occupied, try there")
                        else:
                            print("No free destage")
                    ball.ball[x].setPos(y)
                else:
                    print("No Ball in Stage")
            Robot.index.destageBall = False

        if Robot.index.autoShotToggle:
            if  Robot.index.autoShoot == True:
                Robot.index.autoShoot = False
            else:
                 Robot.index.autoShoot = True
            Robot.index.autoShotToggle = False

        if Robot.index.stage:
            if Robot.index.staged_oc == False:
                print("Attempting to stage Ball")
                staging(ball)
            else:
                Robot.index.stage = False  

        if Robot.index.aiming:
            if self.BallController.CurrentNum() == 0:
                wpilib.XboxController(Controllers.DRIVER).setRumble(wpilib.XboxController.RumbleType.kRightRumble, 1)   
        else:
            wpilib.XboxController(Controllers.DRIVER).setRumble(wpilib.XboxController.RumbleType.kRightRumble, 0)
            

        if Robot.index.destageBall: 
            if Robot.index.staged_oc == True:
                destaging(ball)
            else:
                Robot.index.destageBall = False

        if Robot.shooter.ready:
            if Robot.index.staged_oc and Robot.index.photo_electric.get_value():
                x = ball.posNum("Stage")
                if ball.ball[x].team:
                    ball.ball[x].setPos("Shoot")
                else:
                    pass # set variable to shoot enemy ball
            else:
                if auto:
                    # if not Robot.intake.left_intake_down and not Robot.intake.right_intake_down:
                    #     staging(ball) 
                    staging(ball)

    def leftIntake(self, left_color):
        '''
        if left intake is down -> generates new ball and positions it within the system

        :Param object left_color: Left color sensor OR [True,False] for forced team/not team

        LOGIC inside function 
        '''
        if Robot.intake.left_intake_down:
            '''
            Logic:
                If left Intake Down:
                    if left side of index occupied by ball:
                        identify ball number
                        move ball position to right if path clear #no right ball occupation
                    else:
                        if both sides of the intakes are not occupied:
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
                elif Left Intake not Down:
                    if Balls are moving in system:
                        Keep running Left Dinglebobs
                    else:
                        Stop Left Dinglebobs

                #Vice versa for right side

            '''
            if Robot.index.left_oc:
                x = self.BallController.posNum("Left")
                self.BallController.ball[x].setPos("Right")
                self.BallController.leftPress = True
            else:
                if not Robot.index.left_oc:
                    Robot.index.intakeBall("Left", "In")
                    if Robot.index.left_limit.get_value() and not Robot.index.left_oc and not self.BallController.leftPress and not self.BallController.leftInvalid:
                        c = Robot.index.ball_count
                        self.BallController.ball.append(Ball("Left"))
                        print("Ball count + 1")
                        Robot.index.ball_count += 1
                        self.BallController.leftPress = True
                        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100])) #0100 = Left
                        if left_color != False and left_color != True:
                            if left_color != config.TEAM and left_color != "none":
                                print("OPP BALL")
                                self.BallController.ball[c].team = False
                                self.BallController.ball[c].setPos("Stage")
                            else:
                                print("TEAM BALL")
                                self.BallController.ball[c].team = True
                                self.BallController.ball[c].setPos("Right")
                        elif left_color == True:
                            self.BallController.ball[c].team = True
                            self.BallController.ball[c].setPos("Right")
                        elif left_color == False:
                            self.BallController.ball[c].team = False
                            self.BallController.ball[c].setPos("Stage")
                
        elif not Robot.intake.left_intake_down:
            if not Robot.index.traffic_oc and Robot.index.shooting == False:
                Robot.index.intakeBall("Left", "Off")
    
    def rightIntake(self, right_color):
        '''
        if Right intake is down -> generates new ball and positions it within the system

        :Param object right_color: Right color sensor OR [True,False] for forced team/not team 

        LOGIC inside Left Intake function 
        '''
        if Robot.intake.right_intake_down:
            if Robot.index.right_oc:
                x = self.BallController.posNum("Right")
                self.BallController.ball[x].setPos("Left")
                self.BallController.rightPress = True
            else:
                if not Robot.index.right_oc:
                    Robot.index.intakeBall("Right", "In")
                    if Robot.index.right_limit.get_value() and not Robot.index.right_oc and not self.BallController.rightPress and not self.BallController.rightInvalid:
                        c = Robot.index.ball_count
                        self.BallController.ball.append(Ball("Right"))
                        print("Ball count + 1")
                        Robot.index.ball_count += 1
                        self.BallController.rightPress = True
                        if right_color != False and right_color != True:
                            Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100])) #0100 = Left
                            if right_color != config.TEAM and right_color != "none":
                                print("OPP BALL")
                                self.BallController.ball[c].team = False
                                self.BallController.ball[c].setPos("Stage")
                            else:
                                print("TEAM BALL")
                                self.BallController.ball[c].team = True
                                self.BallController.ball[c].setPos("Left")
                        elif right_color == True:
                            self.BallController.ball[c].team = True
                            self.BallController.ball[c].setPos("Left")
                        elif right_color == False:
                            self.BallController.ball[c].team = False
                            self.BallController.ball[c].setPos("Stage")
                
        elif not Robot.intake.right_intake_down:
            if not Robot.index.traffic_oc and Robot.index.shooting == False:
                Robot.index.intakeBall("Right", "Off")

    def resetBall(self):
        '''
        if reset ball button is pressed -> Empties all occupation variables and delets all balls stored in list
        '''
        if Robot.index.resetBall:
            self.BallController.reset()

    def SmartDashboard(self):
        '''
        inputs variables to shuffleboard for debugging and general operation
        '''
        lIcurrent = self.pdh.getCurrent(4)
        lRcurrent = self.pdh.getCurrent(3)
        Scurrent = self.pdh.getCurrent(11)
        wpilib.SmartDashboard.putNumber("Shooter Flywheel Current", Scurrent)
        wpilib.SmartDashboard.putNumber("Left Intake Current", lIcurrent)
        wpilib.SmartDashboard.putNumber("Right Intake Current", lRcurrent)
        wpilib.SmartDashboard.putBoolean("Traffic Light?", Robot.index.traffic_oc)
        wpilib.SmartDashboard.putBoolean("Auto Shoot?", Robot.index.autoShoot)
        wpilib.SmartDashboard.putNumber("Balls in Index", self.BallController.CurrentNum())
        wpilib.SmartDashboard.putNumber("Total Balls Shot", self.BallController.RemovedNum())
        wpilib.SmartDashboard.putNumber("Total Ball Count", Robot.index.ball_count)
        wpilib.SmartDashboard.putBoolean("Left Side Occupied?", Robot.index.left_oc)
        wpilib.SmartDashboard.putBoolean("Right Side Occupied?", Robot.index.right_oc)
        wpilib.SmartDashboard.putBoolean("Stage Side Occupied?", Robot.index.staged_oc)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        
        self.SmartDashboard()
        #Operator Joystick
        left_joy = Keymap.Index.LEFT_JOY.value
        right_joy = Keymap.Index.RIGHT_JOY.value
        #Color sensors
        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100]))
        left_color = Sensors.color_sensors.color()
        left_val = Sensors.color_sensors.get_val()
        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b1000]))
        right_color = Sensors.color_sensors.color()
        right_val = Sensors.color_sensors.get_val()

        #sensor reading check
        self.sensorCheck(left_val, right_val)

        if abs(left_joy) > .5 or abs(right_joy) > .5:
            #Manual Control overide
            self.operatorControl(left_joy, right_joy)
        else:
            #threshold variables can be found at top of Ball Class
            self.checkBall() #checks if balls are moving within system -> checks if balls have reached its destination
            
            self.checkLimit() #checks limits for exceptions to intaking rules -> enables or disables booleans for Intakes to generate a new ball
            
            self.currentSensing(constants.intake_current_sensing) #Intake current sensing for dragging balls

            self.shooting(Robot.index.autoShoot) #if shooting (False or True for automatic shooting) -> uses shooter.ready to shoot balls that are staged

            self.leftIntake(left_color) #if left intake is down -> generates new ball and positions it within the system

            self.rightIntake(right_color) #if right intake is down -> generates new ball and positions it within the system

            self.resetBall() #if reset ball button is presse d -> Empties all occupation variables and delets all balls stored in list       


    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass


