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
        RESET_GYRO = DefaultButton(Controllers.DRIVER, controller.A)
        REZERO_MOTORS = DefaultButton(Controllers.DRIVER, controller.B)
