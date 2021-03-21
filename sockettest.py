import mecademic
import time
import threading

ROBOT_IP= "192.168.1.36"

robot = mecademic.Robot(ROBOT_IP)
if robot.Connect():
    print(f'Connected to {robot.product} FW version {robot.firmware}')

print(f'Starting robot position: {robot.GetPose()}')

joints = [0,5,35,0,30,-40]
robot.SetSpeed(30)
robot.MoveJ(robot.joints[robot.joint])

pushpullpose = [200,4,225,175,75,49]
robot.SetSpeed(50)
robot.MoveP(pushpullpose)

robot.Push(30)
robot.Pull(60)
robot.SetSpeed(20)

appdeptestpose = [180,0,80,180,0,40]
robot.Approach(appdeptestpose,80)
robot.MoveL(appdeptestpose)
robot.Depart(appdeptestpose,80)
