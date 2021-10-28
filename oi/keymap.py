from dataclasses import dataclass

import oi.controllermap as controllers

import commands2 as commands
import commands2.button

from oi.buttons import DefaultButton, AxisButton
from oi.joysticks import Joysticks


controller = controllers.XBoxController


class Controllers:
    DRIVER = 0
    OPERATOR = 1


class Keymap:
    class Drivetrain:
        DRIVE_CONTROLLER = Controllers.DRIVER
        DRIVE_X_AXIS = controller.R_JOY[0]
        DRIVE_Y_AXIS = controller.L_JOY[1]

    class Shifter:
        HIGH = DefaultButton(Controllers.DRIVER, controller.SELECT)
        LOW = DefaultButton(Controllers.DRIVER, controller.START)

    class Index:
        DOWN = DefaultButton(Controllers.DRIVER, controller.LB)
        UP = DefaultButton(Controllers.DRIVER, controller.RB)
        MANUAL_INDEX_CONTROLLER = Controllers.OPERATOR
        MANUAL_INDEX_TOP_AXIS = controller.L_JOY[1]
        MANUAL_INDEX_BOTTOM_AXIS = controller.R_JOY[1]

    class Intake:
        START = DefaultButton(Controllers.OPERATOR, controller.A)
        STOP = DefaultButton(Controllers.OPERATOR, controller.B)
        DOWN = DefaultButton(Controllers.OPERATOR, controller.X)
        UP = DefaultButton(Controllers.OPERATOR, controller.Y)

    class Hopper:
        RUN = DefaultButton(Controllers.OPERATOR, controller.RB)
        RUN_REVERSE = DefaultButton(Controllers.OPERATOR, controller.LB)

    class Shooter:
        LOW_RETRACTED = DefaultButton(Controllers.DRIVER, controller.A)
        LOW_EXTENDED = DefaultButton(Controllers.DRIVER, controller.B)
        HIGH_RETRACTED = DefaultButton(Controllers.DRIVER, controller.X)
        HIGH_EXTENDED = DefaultButton(Controllers.DRIVER, controller.Y)
        SHOOT = DefaultButton(Controllers.OPERATOR, controller.RT)
        AIM = DefaultButton(Controllers.OPERATOR, controller.LT)

    class Turret:
        TURRET_LEFT = DefaultButton(Controllers.OPERATOR, controller.LT)
        TURRET_RIGHT = DefaultButton(Controllers.OPERATOR, controller.RT)

    class Hanger:
        HANGER_INIT = DefaultButton(Controllers.OPERATOR, controller.START)
        HANGER_CLIMB = DefaultButton(Controllers.OPERATOR, controller.SELECT)
        HANGER_RESET = DefaultButton(Controllers.DRIVER, controller.SELECT)
