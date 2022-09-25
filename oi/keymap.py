from robotpy_toolkit_7407.oi import XBoxController, JoystickAxis, DefaultButton

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController

class Controllers:
    DRIVER = 0
    OPERATOR = 1


class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[1])
        DRIVE_ROTATION_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.R_JOY[0])
        DRIVE_Y2_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.R_JOY[1])
        RESET_GYRO = DefaultButton(Controllers.DRIVER, controllerDRIVER.A)
        REZERO_MOTORS = DefaultButton(Controllers.DRIVER, controllerDRIVER.B)
        AIM_SWERVE = DefaultButton(Controllers.DRIVER, controllerDRIVER.RT)
        DRIVER_CENTRIC = DefaultButton(Controllers.DRIVER, controllerDRIVER.LB)
        DRIVER_CENTRIC_REVERSED = DefaultButton(Controllers.DRIVER, controllerDRIVER.RB)

    class Elevator:
        ELEVATOR_INIT = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.LB)
        ELEVATOR_CLIMB = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.RB)
        ELEVATOR_DOWN = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.SELECT)

    class Intake:
        LEFT_INTAKE_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.LT)
        RIGHT_INTAKE_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.RT)
        AUTO_INTAKE_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.B)

    class Index:
        #LEFT_JOY and RIGHT_JOY are used in ballpath command
        LEFT_JOY = JoystickAxis(Controllers.OPERATOR, controllerOPERATOR.L_JOY[1])
        RIGHT_JOY = JoystickAxis(Controllers.OPERATOR, controllerOPERATOR.R_JOY[1])
        RESET_BALL = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.Y)
        
    class BallPath:
        TOGGLE_AUTO_EJECT = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.START)
        TOGGLE_AUTO_EJECT_COLOR = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.X)

    class Shooter:
        #SHOOTER_ENABLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.RT)
        SHOOTER_EJECT = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.A)
        #FENDER_SHOT = DefaultButton(Controllers.DRIVER, controllerDRIVER.LT)
        #SHOOTER_SHORT_EJECT = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.LT)
        #SHOOTER_OFFSET_UP = DefaultButton(Controllers.DRIVER, controllerDRIVER.LB)
        #SHOOTER_OFFSET_DOWN = DefaultButton(Controllers.DRIVER, controllerDRIVER.RB)

