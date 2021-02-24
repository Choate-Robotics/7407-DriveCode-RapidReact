Team 7407 drive code for 2021 (python version)

# Structure
The code is divided up into 3 main components: Subsystems, Commands, and OI


## Subsystems (`subsystem/`)
Subsystems are parts of the robot that are operated independently of one another. We currently have 7 subsystems:

#### Drivetrain (`subsystem/drivetrain/`)
The drivetrain subsystem controls the wheels.

#### Hopper (`subsystem/hopper/`)
The hopper subsystem controls the hopper, which is used to unjam balls before they are indexed.

#### Index (`subsystem/index/`)
The index subsystem controls the two index motors and the solenoid that moves the index up and down.

#### Intake (`subsystem/intake/`)
The intake subsystem controls the intake motor and the intake solenoid.

#### Shifter (`subsystem/shifter/`)
The shifter subsystem controls the solenoid that shifts the gears on the drivetrain.

#### Shooter (`subsystem/shooter/`)
The shooter subsystem controls the two flywheel motors and the hood solenoid.

#### Turret (`subsystem/turret/`)
The turret subsystem controls the turret motor that rotates the turret during auto-aiming.


## Commands (`command/`)
Commands are associated with a particular subsystem and can be scheduled by controller button presses or autonomous
routines. These commands control their subsystem's components to perform various robot tasks


## OI (`oi/`)
OI, or Operator Interface, maps controller buttons and joysticks to robot commands. 