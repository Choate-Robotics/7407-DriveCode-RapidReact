import oi.controllermap as controller

DRIVER = 0
OPERATOR = 1

DRIVER_CONTROLLER_ID = 0
OPERATOR_CONTROLLER_ID = 1


class Keymap:
    class Drivetrain:
        DRIVE_CONTROLLER = DRIVER
        DRIVE_X_AXIS = 4
        DRIVE_Y_AXIS = 1

    class Shifter:
        HIGH = (controller.SELECT, DRIVER)
        LOW = (controller.START, DRIVER)

    class Index:
        DOWN = (controller.LB, DRIVER)
        UP = (controller.RB, DRIVER)

    class Intake:
        START = (controller.A, DRIVER)
        STOP = (controller.B, DRIVER)
        DOWN = (controller.X, DRIVER)
        UP = (controller.Y, DRIVER)

    class Hopper:
        ENABLE = (controller.RB, OPERATOR)

    class Shooter:
        LOW_RETRACTED = (controller.A, OPERATOR)
        LOW_EXTENDED = (controller.B, OPERATOR)
        HIGH_RETRACTED = (controller.X, OPERATOR)
        HIGH_EXTENDED = (controller.Y, OPERATOR)
        ENABLE = (controller.RT, DRIVER)
        AIM = (controller.LT, DRIVER)

    class Turret:
        TURRET_LEFT = (controller.LT, OPERATOR)
        TURRET_RIGHT = (controller.RT, OPERATOR)
