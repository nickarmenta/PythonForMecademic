# Python control for Mecademic

A python module designed for the Meca500 robots. The module offers tools that give access to most features of the Mecademic Robots as well as useful movement and settings tools.

#### Supported Robots

 * Meca500 R3

## Getting Started

These instructions will allow you to control the Robots from Mecademic through a python package and use the package for development and deployment purposes.

## Prerequisites

To be able to use the module without unexpected errors, the user must have a copy of python installed on their machine and it is recommended to use python version 3.6 or higher. [Python](https://www.python.org/) can be installed from its main website (a reboot will be require after the installation to complete the setup). 

## Installing the package

You can use pip to install the library:

```sh
user@host:~$ pip install git+https://github.com/nickarmenta/PythonForMecademic
``` 

And/or build off the provided Dockerfile

```sh
user@host:~$ docker build mecademic:latest .
``` 

## Using the library

### Startup

Start by importing the library and connecting to then initializing the robot.

```py
import mecademic
robot = mecademic.Robot("192.168.0.100")
robot.Startup()
```

You can now move your robot around in multiple ways

```py
# Positioning the joints in degrees [j0,j1,j2,j3,j4,j5]
robot.MoveJ([0,-60,60,0,0,0])
import mecademic
robot = mecademic.Robot("192.168.0.100")
robot.Startup()
```



This will open a python terminal with the module already imported. From there you need to connect to the Mecademic Robot before being able to perform actions. This is done by making an instance of the class RobotController by passing the IP Address of the Robot as an argument and using the function connect():
```py
>>> robot = MecademicRobot.RobotController('192.168.0.100')
>>> robot.connect()
```

Once succesfully connected to the Robot, you are able to start performing actions. To get it ready for operations, it must be activated and homed or it will fall in error. To do so, the following functions are run:

```py
>>> robot.ActivateRobot()
>>> robot.home()
```

The Robot is now ready to perform operations. [The user programming manual](https://mecademic.com/resources/documentation) or the documentation in the module is sufficiant to be able to make the Robot perform actions and control the robot. 

When done with the Robot and desire to power it off, it must first be deactivated and disconnected to avoid issues and problems. It is done by two functions:

```py
>>> robot.DeactivateRobot()
>>> robot.disconnect()
```

If during use the Robot encounters an error, it will go into error mode. In this mode, the module will block any command to the Robot unless its an error reset. To properly reset errors on the Robot, the following function must be run:

```py
>>> robot.ResetError()
```

#### Runnable Script

To make a runnable script, the above functions calls remain the same in the script. Actions must be placed between an activation and a deactivation to avoid errors. Writing the script is like regular programming in python. It is recommended to have an error catcher to get the Robot out of error mode and not have the Robot stop working in the middle of operation. This can be creatively done by using the isInError() and action functions to catch the Robot immediately as it falls in error and bringing it back to operating condition. A method can be made to it like the following:
```py
def AutoRepair(robot):
    if(robot.isInError()):
        robot.ResetError()
    elif(robot.GetStatus()['Paused']==1):
        robot.ResumeMotion()
```
Note: Deactivating and reactivating the Robot is not necessary but can be helpful in power cycle recovery.

An example of a script for a Mecademic Robot would be:
```py
import MecademicRobot
robot = MecademicRobot.RobotController('192.168.0.100')
robot.connect()
robot.ActivateRobot()
robot.home()
robot.SetBlending(0)
robot.SetJointVel(100)
while True:
	robot.MoveJoints(0,0,0,170,115,175)
	robot.MoveJoints(0,0,0,-170,-115,-175)
	robot.MoveJoints(0,-70,70,0,0,0)
	robot.MoveJoints(0,90,-135,0,0,0)
	robot.GripperClose()
	robot.MoveJoints(0,0,0,0,0,0)
	robot.MoveJoints(175,0,0,0,0,0)
	robot.MoveJoints(-175,0,0,0,0,0)
	robot.MoveJoints(175,0,0,0,0,0)
	robot.GripperOpen()
	robot.MoveJoints(0,0,0,0,0,0)
```
This will make the Robot perform a repetitive task until the program is terminated.

A more viable way to make long programs for the Mecademic Products is by using a string. In python, there are various ways to write strings and the string variable type is useful to make a program script by using the triple quotes string. This format can be spread across multiple lines and includes newlines into the string without placing them implicitly. This makes it easy to read and write the program you are trying to develop. The script can be written easily with the string format as follow:
```py
    	Program = """SetBlending(0)
                    SetJointVel(100)
                    MoveJoints(0,0,0,170,115,175)
                    MoveJoints(0,0,0,-170,-115,-175)
                    MoveJoints(0,0,0,170,115,175)
                    MoveJoints(0,0,0,-170,-115,-175)
                    MoveJoints(0,0,0,170,115,175)
                    MoveJoints(0,0,0,-170,-115,-175)
                    MoveJoints(0,0,0,170,115,175)
                    MoveJoints(0,0,0,-170,-115,-175)
                    MoveJoints(0,0,0,170,115,175)
                    MoveJoints(0,0,0,-170,-115,-175)
                    MoveJoints(0,-70,70,0,0,0)
                    MoveJoints(0,90,-135,0,0,0)
                    MoveJoints(0,-70,70,0,0,0)
                    MoveJoints(0,90,-135,0,0,0)
                    MoveJoints(0,-70,70,0,0,0)
                    MoveJoints(0,90,-135,0,0,0)
                    MoveJoints(0,-70,70,0,0,0)
                    MoveJoints(0,90,-135,0,0,0)
                    MoveJoints(0,-70,70,0,0,0)
                    MoveJoints(0,0,0,0,0,0)
                    MoveJoints(175,0,0,0,0,0)
                    MoveJoints(-175,0,0,0,0,0)
                    MoveJoints(175,0,0,0,0,0)
                    MoveJoints(-175,0,0,0,0,0)
                    MoveJoints(175,0,0,0,0,0)
                    MoveJoints(-175,0,0,0,0,0)
                    MoveJoints(175,0,0,0,0,0)
                    MoveJoints(-175,0,0,0,0,0)
                    MoveJoints(175,0,0,0,0,0)
                    MoveJoints(0,0,0,0,0,0)"""
```
Each line has a command with the arguments it requires. The commands are written in the same way as the functions in the module. Once the script is complete, it must be broken down into a list of actions. Python makes it easy by using the available string functions.
```py
Program = Program.replace(' ','')
movements = Program.split("\n")
```
The variable movements contains the list of actions to perform after getting rid of all the empty spaces and seperating the command by distinguishing commands by line. To go through the actions one by one, it is only required to make a loop that iterates through the actions. Using the __exchangeMsg__ function, it is easy to send the command to the Robot. The __exchangeMsg__ function is responsable for interpreting the commands and return expected messages back to the user. It is the backbone of most of the functions of the module.
```py
for action in movements:
	robot.exchangeMsg(action)
	if(robot.isInError()):
    		AutoRepair(robot)
```
If the script you wrote is one you wish the Robot to repeat until stopped by a user for whatever reason, the previous loop can be placed inside an infinite loop. Using all the information, building blocks and functions provided, you are fully equipped to control and program your Mecademic Robot for your project's requirements.

## Get Live Positional Feedback from the Robot

The robot is capable of giving it's position while in movement and the RobotFeedback module of the MecademicRobot package allows the user to have access to that data. If the module is run in interactive shell or in a script, the best way to get data as fast as possible to another file or to be printed to the user is by using the module in an infinite loop. 

The RobotFeedback constructor takes in two arguments, the IP address of the robot and the firmware version of the robot. Both are of the string variable type. The functions call looks as follows:
```py
feedback = MecademicRobot.RobotFeedback(IP, firmware_version)
```
An example for how to use the RobotFeedback module is as follows:
```py
import MecademicRobot
feedback = MecademicRobot.RobotFeedback('192.168.0.100', '7.0.6')
feedback.connect()
while(True):
	feedback.get_data()
	print(feedback.joints)
	print(feedback.cartesian)
```
By calling get_data(), the values of joints and cartesian get updated with the latest received data from the robot. The format of the data for joints is (joint_1, joint_2, ..., joint_n), where n is the number of joints on the Robot and the values are in degrees. For the format of the data in cartesian, the data is of the form (x, y, z, alpha, beta, gamma), where x, y and z are in mm and alpha, beta and gamma are in degrees. This module works at its best when it is run in parallel with RobotController, either as another runnable, threading, etc. When in parallel, the live data will be refreshed at a faster speed while controlling the robot. 

## Getting Help

To get support, you can start an issue on the Mecademic/python_driver issues section or send an email to support@mecademic.com.

## License

All packages in this repository are licensed under the MIT license.

## Authors 

* **Mecademic** - *Continuous work*
