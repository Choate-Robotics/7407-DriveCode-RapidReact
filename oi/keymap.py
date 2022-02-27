from robotpy_toolkit_7407.oi import XBoxController, LogitechController, JoystickAxis, DefaultButton

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
        RESET_GYRO = DefaultButton(Controllers.DRIVER, controllerDRIVER.A)
        REZERO_MOTORS = DefaultButton(Controllers.DRIVER, controllerDRIVER.B)
        AIM_SWERVE = DefaultButton(Controllers.DRIVER, controllerDRIVER.RT)

    class Elevator:
        ELEVATOR_INIT = DefaultButton(Controllers.DRIVER, controllerOPERATOR.START)
        ELEVATOR_CLIMB = DefaultButton(Controllers.DRIVER, controllerOPERATOR.SELECT)
        ELEVATOR_SOLENOID_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.START)

    class Intake:
        LEFT_INTAKE_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.A)
        RIGHT_INTAKE_TOGGLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.B)

    class Index:
        INDEX_JOY = JoystickAxis(Controllers.OPERATOR, controllerOPERATOR.L_JOY[1])

    class Shooter:
        SHOOTER_ENABLE = DefaultButton(Controllers.OPERATOR, controllerOPERATOR.RT)
