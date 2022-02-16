from sensors import limit_switch
import subsystem


class Robot:
    #drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()
    index = subsystem.Index()
    elevator = subsystem.Elevator()
    color_sensor = subsystem.Color_Sensor(I2CPort = 1, deviceAddress = 0x29) # TODO: IMPORTANTE
    limit_switch = subsystem.Limit_Switch(port = 1) # TODO: IMPORTANTE
    magnetic_limit_switch = subsystem.Magnetic_Limit_Switch(port = 1) # TODO: IMPORTANTE
    photoelectric_switch = subsystem.Photoelectric_Switch(port = 1) # TODO: IMPORTANTE
