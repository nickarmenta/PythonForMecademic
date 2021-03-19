import MecademicRobot

ROBOT_IP= "192.168.1.36"
FIRMWARE = '8.1.9'

robot = MecademicRobot.Robot(ROBOT_IP, FIRMWARE)
robot.Connect()
robot.Activate()
robot.Home()