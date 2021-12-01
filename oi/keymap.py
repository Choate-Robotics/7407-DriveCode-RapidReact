import lib.oi.controllermap as controllers

from lib.oi.buttons import DefaultButton
from lib.oi.joysticks import JoystickAxis

controller = controllers.XBoxController


class Controllers:
    DRIVER = 0
    OPERATOR = 1


class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controller.R_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.OPERATOR, controller.L_JOY[1])
