#!/usr/bin/env python3
import socket
import threading
import logging
logging.basicConfig(filename='meca.log', level=logging.DEBUG)

# Dictionary of status indexes in robot status message
statusDict = {'activated': 0,
            'homed': 1,
            'simulating': 2,
            'error': 3,
            'paused': 4,
            'EOB': 5,
            'EOM': 6}

# Ease of use cartesian index labeling
cartDict = {'x': 0,
            'y': 1,
            'z': 2,
            'rx': 3,
            'ry': 4,
            'rz': 5}

# Dictionary of command responses
responseDict = {'ActivateRobot': [2000, 2001],
                'DeactivateRobot': [2004],
                'BrakesOn': [2010],
                'BrakesOff': [2008],
                'Home': [2002, 2003],
                'GetJoints': [2026],
                'GetPose': [2027],
                'ClearMotion': [2044],
                'PauseMotion': [2042],
                'ResumeMotion': [2043],
                'ResetError': [2005],
                'GetStatusRobot': [2007],
                'GetFwVersion': [2081],
                'GetProductType': [2084]}

# Combined control and feedback class for Mecademic
class Robot:
    def __init__(self, ip):
        self.ip = ip   
        self.connected = False
        # Initialize tool and work reference frames
        self.pose = 0
        self.poses = [[0,0,0,0,0,0]]
        self.joint = 0
        self.joints = [[0,-60,60,0,0,0]]
        self.tool = 0
        self.toolFrames = [[0,0,0,0,0,0]]
        self.work = 0
        self.workPlanes = [[0,0,0,0,0,0]]

    # Connect to both control and feedback servers
    def Connect(self):
        self.connected = True
        self.controlClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.controlClient.settimeout(10)  # 100ms
        self.controlClient.connect((self.ip, 10000))
        code, response = self.controlClient.recv(1024).decode('ascii')[1:-2].split('][')
        if int(code) != 3000:
            if int(code) == 3001:
                print('Another user is already connected!')
                exit()

            logging.warning('Unable to connect to port 10000')
            self.connected = False

        # Clear initial errors
        if self.GetStatus('error'):
            logging.info('Error on initialization')
            self.ResetError()
            
        self.firmware = self.ReadResponse('GetFwVersion')    
        self.product = self.ReadResponse('GetProductType')    

        self.feedbackClient = socket.socket()
        self.feedbackClient.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY,1)
        self.feedbackClient.settimeout(10)  # 100ms
        self.feedbackClient.connect((self.ip, 10001))
        code = int(self.feedbackClient.recv(1024).decode('ascii')[1:-2].split('][')[0])
        if int(code) != 2079:
            logging.warning('Unable to connect to port 10001')
            self.connected = False

        return self.connected

    # Easy setup routine
    def Startup(self):
        if self.Activate(): return self.Home()
    
    # Ease of use 0-100% global speed adjustment
    def SetSpeed(self, percentage):
        # If speed is provided as fractional change to percentage
        if percentage < 1: percentage *= 100
        self.SetCartAcc(percentage)
        self.SetCartAngVel(3*percentage)
        self.SetCartLinVel(10*percentage)
        self.SetJointAcc(1.5*percentage)
        self.SetJointVel(percentage)
    
    # Move robot in +Z of tool frame
    def Push(self, mm): self.MoveToolRel([0,0,mm,0,0,0])

    # Move robot in -Z of tool frame
    def Pull(self, mm): self.MoveToolRel([0,0,-mm,0,0,0])

    # Move robot Z-offset of tool frame
    def Approach(self, pose, zOffset):
        approachPose = pose.copy()
        approachPose[2] += zOffset
        self.MoveP(approachPose)

    # Move robot Z-offset of tool frame
    def Depart(self, pose, zOffset):
        departPose = pose.copy()
        departPose[2] += zOffset
        self.MoveL(departPose)

    # Power-up robot motors
    def Activate(self):
        if self.GetStatus('activated'): return True
        else: return self.SendCommand('ActivateRobot')            

    # Power-down robot motors
    def Deactivate(self):
        if not self.GetStatus('activated'): return True
        else: return self.SendCommand('DeactivateRobot')            

    # De-activate robot and engage brakes
    def BrakesOn(self):
        if self.GetStatus('activated'): self.Deactivate()
        else: return self.SendCommand('BrakesOn')            

    # Activate robot and disengage brakes
    def BrakesOff(self):
        if not self.GetStatus('activated'): self.Activate()
        else: return self.SendCommand('BrakesOff')            

    # Home robot motors
    def Home(self):
        if self.GetStatus('homed'): return True
        else: return self.SendCommand('Home')            

    # Move robot to target "joints" list
    def MoveJ(self, joints):
        if not self._checkJointLimits(joints):
            logging.warning("Target position outside joint limits!")
            return False
        else: return self.SendCommand(f'MoveJoints{tuple(joints)}')            

    # Jog robot at target "joints" speed
    def MoveJV(self, joints):
        if not self._checkJointSpeedLimits(joints):
            logging.warning("Target speed outside joint limits!")
            return False
        else: return self.SendCommand(f'MoveJointsVel{tuple(joints)}')            

    # Move robot linearly to target "pose" list relative to work frame
    def MoveL(self, pose):
        return self.SendCommand(f'MoveLin{tuple(pose)}')            

    # Move robot in by "pose" list relative to tool frame
    def MoveToolRel(self, pose):
        return self.SendCommand(f'MoveLinRelTRF{tuple(pose)}')            

    # Move robot in by "pose" list relative to work frame
    def MoveWorkRel(self, pose):
        return self.SendCommand(f'MoveLinRelWRF{tuple(pose)}')
    
    # Jog at target "pose" speed relative to tool frame
    def MoveToolVel(self, pose):
        return self.SendCommand(f'MoveLinVelTRF{tuple(pose)}')

    # Jog tool at target "pose" speed relative to work plane
    def MoveWorkVel(self, pose):
        return self.SendCommand(f'MoveLinVelWRF{tuple(pose)}')

    # Move robot to target "pose" list relative to work plane
    def MoveP(self, pose):
        return self.SendCommand(f'MovePose{tuple(pose)}')

    # Set blend radius from 0-100%
    def SetBlending(self, percentage):
        assert percentage >= 0 and percentage <= 100
        return self.SendCommand(f'SetBlending({percentage})')

    # Set cartesian acceleration from 0.001-600%
    def SetCartAcc(self, percentage):
        assert percentage >= .001 and percentage <= 600
        return self.SendCommand(f'SetCartAcc({percentage})')

    # Set cartesian angular velocity from 0.001-300deg/s
    def SetCartAngVel(self, degrees):
        assert degrees >= 0.001 and degrees <= 300
        return self.SendCommand(f'SetCartAngVel({degrees})')

    # Set cartesian linear velocity from 0.001-1,000mm/s
    def SetCartLinVel(self, mms):
        assert mms >= 0.001 and mms <= 1000
        return self.SendCommand(f'SetCartLinVel({mms})')

    # Set joint acceleration from 0.001-150%
    def SetJointAcc(self, percentage):
        return self.SendCommand(f'SetJointAcc({percentage})')

    # Set joint velocity from 0.001-100%
    def SetJointVel(self, percentage):
        return self.SendCommand(f'SetJointVel({percentage})')

    # Set tool frame to existing tool or arbitrary offset
    def SetTool(self, toolOffset):
        if type(toolOffset) is int:
            self.tool = toolOffset
            self.SendCommand(f'SetTRF({self.toolFrames[toolOffset]})')
        elif type(toolOffset) is list:
            self.tool = None
            self.SendCommand(f'SetTRF({toolOffset})')

    # Add a new tool frame to robot tools
    def AddTool(self, toolOffset, index=-1):
        if index >= 0:
            self.toolFrames[index] = toolOffset
        else:
            if len(toolOffset) == 3:
                for vector in range(3):
                    toolOffset.append(0)
            self.toolFrames.append(toolOffset)

    # Set work plane to existing plane or arbitrary offset
    def SetWork(self, workPlane):
        if type(workPlane) is int:
            self.work = workPlane
            self.SendCommand(f'SetWRF({self.workPlanes[workPlane]})')
        elif type(workPlane) is list:
            self.work = workPlane
            self.SendCommand(f'SetWRF({workPlane})')

    # Add a new work plane frame to robot work planes
    def AddWork(self, workPlane, index=-1):
        if index >= 0:
            self.workPlanes[index] = workPlane
        else:
            if len(workPlane) == 3:
                for vector in range(3):
                    toolOffset.append(0)
            self.workPlanes.append(workPlane)

    # Get list of current joint positions in degrees
    def GetJoints(self):
        return self.ReadResponse('GetJoints')

    # Get list of current cartesian position in millimeters
    def GetPose(self):
        return self.ReadResponse('GetPose')

    # Delete current planned move
    def ClearMove(self):
        return self.SendCommand('ClearMotion')

    # Pause current move
    def PauseMove(self):
        return self.SendCommand('PauseMotion')

    # Resume current move
    def ResumeMove(self):
        return self.SendCommand('ResumeMotion')

    # Reset error
    def ResetError(self):
        return self.SendCommand('ResetError')

    def SetCheckpoint(self, step=1):
        self.controlClient.send(bytes(f'SetCheckpoint({step})\0','ascii'))
        code, response = self._GetMessage()
        if code in [2000, 2001]: return True
        else: return False
        
    # Set position update rate in ms
    def SetMonitoringInterval(self, ms):
        assert ms >= 0.001 and ms <= 1
        return self.SendCommand(f'SetMonitoringInterval({ms})', client='feedback')

    # Get robot status as list of booleans
    def GetStatus(self, status='all'):
        responseList = self.ReadResponse('GetStatusRobot').split(',')
        responseBool = [bool(int(response)) for response in responseList]
        if status != 'all':
            if status in statusDict.keys():
                return responseBool[statusDict[status]]
            else:
                print(f'Use an available value:\n{statusDict.keys()}')
        else:
            return responseBool

    # Send command and receive confirmation
    def SendCommand(self, cmd, client='command'):
        if self.connected is False: self.Connect()
        
        if client == 'command':
            self.controlClient.send(bytes(f'{cmd}\0','ascii'))
            code, response = self.controlClient.recv(1024).decode('ascii')[1:-2].split('][')
            if int(code) in self._getCodes(cmd): return True
            else:
                print(f'Error: {response}')
                self.ResetError()
                return False
        else:
            self.feedbackClient.send(bytes(f'{cmd}\0','ascii'))
            code, response = self.feedbackClient.recv(1024).decode('ascii')[1:-2].split('][')
            print(code, response)
            return True

    # Send command and receive message
    def ReadResponse(self, cmd):
        if self.connected is False: self.Connect()
        self.controlClient.send(bytes(f'{cmd}\0','ascii'))
        code, response = self.controlClient.recv(1024).decode('ascii')[1:-2].split('][')
        if int(code) in self._getCodes(cmd): return response
        else:
            logging.warning(f'Error: {response}')
            return None

    # Receive current joint or cartesian positions
    def ReadPosition(self, cmd):
        if self.connected is False: self.Connect()
        jointResponse, poseResponse = self.feedbackClient.recv(1024).decode('ascii').split('\x00')[:2]
        print(jointResponse, poseResponse)
        if cmd == 'GetJoints': msg = jointResponse
        elif cmd == 'GetPose': msg = poseResponse
        code, responseString = msg[1:-2].split('][')
        if not int(code) in self._getCodes(cmd):
            logging.warning(f'Error: {responseString}')
            return None
        
        responseList = responseString.split(',')
        responseFloat = [float(response) for response in responseList]
        return responseFloat
        
    # Look up corresponding error code in dictionary
    def _getCodes(self, cmd):
        if cmd.startswith('Move'):
            return [3004,3012]
        elif cmd.startswith('Set'):
            return [3012]
        else:
            return responseDict[cmd]

    # Move speed checks
    def _checkJointLimits(self, joints):
        assert abs(joints[0]) <= 175
        assert joints[1] >= -70 and joints[1] <= 90
        assert joints[2] >= -135 and joints[2] <= 70
        assert abs(joints[3]) <= 170
        assert abs(joints[4]) <= 115
        assert abs(joints[5]) <= 180
        return True

    def _checkJointSpeedLimits(self, joints):
        assert abs(joints[0]) <= 150
        assert abs(joints[1]) <= 150
        assert abs(joints[2]) <= 180
        assert abs(joints[3]) <= 300
        assert abs(joints[4]) <= 300
        assert abs(joints[5]) <= 500
        return True

    def _checkPoseSpeedLimits(self, pose):
        assert pose[0] >= 0.001 and pose[0] <= 1000
        assert pose[1] >= 0.001 and pose[1] <= 1000
        assert pose[2] >= 0.001 and pose[2] <= 1000
        assert pose[3] >= 0.001 and pose[3] <= 300
        assert pose[4] >= 0.001 and pose[4] <= 300
        assert pose[5] >= 0.001 and pose[5] <= 500
        return True

    def _checkPoseRotLimits(self, pose):
        for vector in pose:
            assert vector >= 0.001 and vector <= 300
