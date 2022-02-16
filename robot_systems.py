from sensors import analog_pressure_sensor, limit_switch
import sensors
import subsystem


class Robot:
    #drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()
    index = subsystem.Index()
    elevator = subsystem.Elevator()
    color_sensor = sensors.Color_Sensor(I2CPort=1, lane=0) # TODO: IMPORTANTE
    limit_switch = sensors.Limit_Switch(port = 1) # TODO: IMPORTANTE
    magnetic_limit_switch = sensors.Magnetic_Limit_Switch(port = 1) # TODO: IMPORTANTE
    photoelectric_switch = sensors.Photoelectric_Switch(port = 1) # TODO: IMPORTANTE
    analog_pressure_sensor = sensors.Analog_Pressure_Sensor(CanAddress=1, PressureChannel=1) # TODO: IMPORTANTE
