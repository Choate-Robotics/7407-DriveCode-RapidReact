from robotpy_toolkit_7407.oi import XBoxController, LogitechController, JoystickAxis, DefaultButton

controller = LogitechController


class Controllers:
    DRIVER = 0
    OPERATOR = 1


class Keymap:
    class Drivetrain:
        #DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controller.L_JOY[0])
        #DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controller.L_JOY[1])
        #DRIVE_ROTATION_AXIS = JoystickAxis(Controllers.DRIVER, controller.R_JOY[0])
        #RESET_GYRO = DefaultButton(Controllers.DRIVER, controller.A)
        #REZERO_MOTORS = DefaultButton(Controllers.DRIVER, controller.B)
        pass
    class Elevator:
        ELEVATOR_UP = DefaultButton(Controllers.DRIVER, controller.RT)
        ELEVATOR_DOWN = DefaultButton(Controllers.DRIVER, controller.LT)
    class Intake:
        INTAKE_ON = DefaultButton(Controllers.DRIVER, controller.R_BUMPER)
        INTAKE_OFF = DefaultButton(Controllers.DRIVER, controller.L_BUMPER)
    class Index:
        INDEX_ON = DefaultButton(Controllers.DRIVER, controller.Y)
        INDEX_OFF = DefaultButton(Controllers.DRIVER, controller.X)
