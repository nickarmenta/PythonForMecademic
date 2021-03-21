# Using the module

The module is designed using a "high-dive" approach where high-level users can quickly get started and low-level users can get more fine-tuned control if needed.

## Getting Started

At it's simplest level you can connect to the robot and have it fully activated in 3 lines of code.

```py
import mecademic
robot = mecademic.Robot("192.168.0.100")
robot.Startup()
print(f'Connected to {robot.product} FW version {robot.firmware}!')
```

## Controlling the robot

### Positioning

There are two types of positions: poses and joint positions.
* Poses are in the cartesian space and are saved in lists of millimeters and degrees
* Joint positions are saved in lists of degrees.

The robot can also store these positions for future reference.

```py
pose = [95,0,200,0,90,0] # [x,y,z,rx,ry,rz]
joints = [0,-60,60,0,0,0] # [j0,j1,j2,j3,j4,j5]
robot.AddPose('myPose', pose)
robot.AddPose('myJoints', joints)
```

### Moving

There are 3 types of moves:
* Joint (MoveJoints) moves the robot from one joint position to another in the shortest time possible.
* Linear (MoveLinear) moves the robot from one pose to another in a straight line.
* Pose (MovePose) moves the robot from one pose to another in the shortest time possible.

```py
robot.MoveJoints(joints) # same as robot.MoveJoints('myJoints')
robot.MoveLinear(pose) # same as robot.MoveLinear('myPose')
robot.MovePose(pose) # same as robot.MovePose('myPose')
```

### Tool offsets and work frames

Tool offsets (TCP/TRF) and work planes (WRF) can be set and stored just like positions.

```py
tool = [0,0,60,0,0,0] # [x,y,z,rx,ry,rz]
work = [200,0,0,60,0,0,0] # [x,y,z,rx,ry,rz]
robot.AddTool('myTool', tool)
robot.AddWork('myWork', work)
robot.SetTool(tool) # same as robot.SetTool('myTool')
robot.SetWork(work) # same as robot.SetWork('myWork')
```
