from robotpy_toolkit_7407.oi import XBoxController, LogitechController, JoystickAxis, DefaultButton

controllerDRIVER = XBoxController
controllerOPERATOR = LogitechController


class Controllers:
    DRIVER = 1
    OPERATOR = 0

class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[1])
        DRIVE_ROTATION_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.R_JOY[0])
        RESET_GYRO = DefaultButton(Controllers.DRIVER, controllerDRIVER.A)
        REZERO_MOTORS = DefaultButton(Controllers.DRIVER, controllerDRIVER.B)

    class Elevator:
        ELEVATOR_INIT = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.A)
        ELEVATOR_CLIMB = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.B)
        ELEVATOR_SOLENOID_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.START)

    class Intake:
        INTAKE_ON = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.A)
        INTAKE_OFF = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.B)
        INTAKE_LEFT_SOLENOID_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.LB)
        INTAKE_RIGHT_SOLENOID_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.RB)

    class Index:
        INDEX_ON = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.X)
        INDEX_OFF = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.Y)

    class Shooter:
        SHOOTER_ENABLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.RT)
