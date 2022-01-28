from robotpy_toolkit_7407.oi import XBoxController, LogitechController, JoystickAxis, DefaultButton

controller = LogitechController


class Controllers:
    DRIVER = 0


class Keymap:
    class Shooter:
        DECREASE_INCREMENT = DefaultButton(Controllers.DRIVER, controller.LB)
        INCREASE_INCREMENT = DefaultButton(Controllers.DRIVER, controller.RB)
        INCREMENT_STICK = JoystickAxis(Controllers.DRIVER, controller.R_JOY[1])
        DECREMENT_STICK = JoystickAxis(Controllers.DRIVER, controller.L_JOY[1])
