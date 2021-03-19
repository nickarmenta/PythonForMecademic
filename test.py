import MecademicRobot
import threading
from time import sleep

def inMotion(robot):
    while not robot.GetStatusRobot()['EOM']:
        if robot.is_in_error():
            print("Error!")
            exit()
    
    print("Done moving!")

IP_ADDRESS = '192.168.1.36'
FIRMWARE = '8.1.9'

robot = MecademicRobot.RobotController(IP_ADDRESS, FIRMWARE)
if robot.connect():
    print("Connected to robot!")
else:
    exit()

if not robot.GetStatusRobot()['Activated']:
    print("Activating robot...")
    robot.ActivateRobot()

if not robot.GetStatusRobot()['Homing']:
    input("Press enter when ready to home...")
    robot.home()

print(f"Current position: {robot.GetPose()}")
newPose = list(robot.GetPose())
newPose[2] -= 20
print(f"New Position: {newPose}")
robot.SetCartLinVel(5)
robot.MoveLin(newPose)

x = threading.Thread(target=inMotion, args=(robot,))
x.start()

if False:
    print("Deactivating robot...")
    robot.DeactivateRobot()

if robot.GetStatusRobot()['EOM']:
    robot.disconnect()
    print("Disconnected from robot!")
