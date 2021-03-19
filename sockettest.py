import mecademic

ROBOT_IP= "192.168.1.36"
FIRMWARE = '8.1.9'

robot = mecademic.Robot(ROBOT_IP, FIRMWARE)
print(f'Connected: {robot.Connect()}')
print(f'Activated: {robot.Activate()}')
print(f'Homed: {robot.Home()}')
joints = [0,5,35,0,30,-40]
robot.MoveJ(joints)
