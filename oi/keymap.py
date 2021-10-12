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
        MANUAL_INDEX_CONTROLLER = OPERATOR
        MANUAL_INDEX_TOP_AXIS = 1
        MANUAL_INDEX_BOTTOM_AXIS = 5

    class Intake:
        START = (controller.A, OPERATOR)
        STOP = (controller.B, OPERATOR)
        DOWN = (controller.X, OPERATOR)
        UP = (controller.Y, OPERATOR)

    class Hopper:
        RUN = (controller.RB, OPERATOR)
        RUN_REVERSE = (controller.LB, OPERATOR)

    class Shooter:
        LOW_RETRACTED = (controller.A, DRIVER)
        LOW_EXTENDED = (controller.B, DRIVER)
        HIGH_RETRACTED = (controller.X, DRIVER)
        HIGH_EXTENDED = (controller.Y, DRIVER)
        SHOOT = (controller.RT, OPERATOR)
        AIM = (controller.LT, OPERATOR)

    class Turret:
        TURRET_LEFT = (controller.LT, OPERATOR)
        TURRET_RIGHT = (controller.RT, OPERATOR)
