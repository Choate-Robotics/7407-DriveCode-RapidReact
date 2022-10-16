# TEAM 7407 DriveCode for the 2021 Rapid React Competition

## Getting Started:

### Clone the repository code onto your computer:

```

git clone https://github.com/Choate-Robotics/7407-DriveCode-2021-Python.git

```
If you prefer ssh:

```

git clone git@github.com:Choate-Robotics/7407-DriveCode-2021-Python.git

```

### If you don't have Poetry installed already:

#### Linux and Mac

```

curl -k https://install.python-poetry.org/ | python3 - 

```
You might have to replace "python" at the end with "python3" depending on how python is configured in your system.

#### Windows Powershell

```

(Invoke-WebRequest -Uri https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py -UseBasicParsing).Content | python -

```

Further information can be found here: https://python-poetry.org/docs/



Make sure to add Poetry to your path.



### With Poetry Installed:

```

poetry shell

poetry install

```

### Deploying Code:
Connect to the robot's wifi.
``python robot.py deploy``
If absolutely necessary, use ``python robot.py deploy --no-version-check`` to avoid WPILib version issues on the robot.


## Best Practices

### Commenting
Comment, comment, comment!
 - Use block quotes to start any function with parameters, and every class's "\_\_init\_\_" function. Block quotes should contain:
	 - Summary
	 - Arguments, with types and descriptions
	 - Return description
	 There are many extensions to help with docstrings. Examples include:
		 - autoDocstring on VsCode
		 - On PyCharm
			 - Place your cursor over a function or class name.
			 - Alt-Enter
			 - Generate documentation string stub
 - Use single line comments for any function without parameters with a description of the function.
 - Use single line comments before any complex function to describe how it works, and to the right of any line or variable that is very complicated.
 - Use TODO comments freely.

### Adding libraries
Always use ``poetry add {library}`` to add libraries. This ensures that libraries are compatible and allows everyone to work easier.
Never, ever, edit poetry.lock or pyproject.toml manually.

### Committing, Pushing, and Pulling
To commit:
```
git add .
git commit -m "Message"
```
To push:
```
git push
```
To pull:
```
git fetch
git pull
```

### Branching
To branch, first make sure that all your local changes are committed. If you would like to abandon the changes, run ``git reset --hard``. Be very careful with resetting.
To branch: ``git branch {branch name}

Branch names are as follows:
 - Subsystem Initialization branch format: init/{subsystem}
	 - Example: init/shooter
	 - Example: init/drivetrain
 - Feature branch format: feat/{subsystems}/{feature}
	 - Example: feat/shooter/optimized_shooting
	 - Example: feat/intake-index/ejection
 - Fix branch format: fix/{subsystems}/issue
	 - Example: fix/camera_server/wrong_ports
	 - Example: fix/robot/network_loop_time
	 - Example: fix/sensors/clean_up
 - Competition branch format: comp/{competition}/day/{day}
	 - Example: comp/battlecry/day/0 (load_in, initial setup, configurations)
	 - Example: comp/hartford/day/1

### Pull Requests
To integrate a branch with branch **Master**,  create a pull-request with the same title as your branch. Make sure pre-commits pass before pushing to ensure clean code. Do not approve the pull request manually; wait for the programming lead or mentor to approve the request.

### Debugging:
#### Logger
 - USE LOGGER! It makes it easier on everyone to debug. (logger is a python library made for logging)
#### Smart Dashboard/Shuffleboard
 - Shuffleboard is preferred over the Smart Dashboard and console for debugging. To use shuffleboard, just push a string, number, boolean, or similar value to the SmartDashboard using "wpilib.SmartDashboard.pushNumber ..." etc. The value is then accessible through ShuffleBoard.

## Resources
 - [RobotPy Documentation](https://robotpy.readthedocs.io/en/stable/) We love RobotPy!
 - [WPILib Documentation](https://docs.wpilib.org/en/stable/index.html) RobotPy is just a wrapper for the WPILib C++ Code. Most of the structure remains the same.
 - [Chief Delphi](https://www.chiefdelphi.com/) Many a sensor problem have been fixed by looking here.
 - [7407 DriveCode-2021-Python](https://github.com/Choate-Robotics/7407-DriveCode-2021-Python) Worlds level code!
