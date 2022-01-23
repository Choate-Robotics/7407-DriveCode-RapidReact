from robotpy_toolkit_7407.oi import XBoxController, LogitechController, JoystickAxis, DefaultButton

controller = XBoxController


class Controllers:
    DRIVER = 0
    OPERATOR = 1


class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controller.L_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controller.L_JOY[1])
        DRIVE_ROTATION_AXIS = JoystickAxis(Controllers.DRIVER, controller.R_JOY[0])

    class Grabber:
        UP = DefaultButton(Controllers.DRIVER, controller.Y)
        DOWN = DefaultButton(Controllers.DRIVER, controller.A)
        OPEN = DefaultButton(Controllers.DRIVER, controller.B)
        CLOSE = DefaultButton(Controllers.DRIVER, controller.X)

    class Intake:
        IN = DefaultButton(Controllers.DRIVER, controller.LB)
        OUT = DefaultButton(Controllers.DRIVER, controller.RB)
