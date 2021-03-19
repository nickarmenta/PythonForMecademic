#!/usr/bin/env python3
import socket
import threading

class Robot:

    def __init__(self, ip, firmware):
        self.ip = ip   
        self.firmware = firmware 
        self.connected = False
        self.statusDict = {'activated': 0,
                        'homed': 1,
                        'simulating': 2,
                        'error': 3,
                        'paused': 4,
                        'EOB': 5,
                        'EOM': 6}

    def Connect(self):
        self.controlClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.controlClient.connect((self.ip, 10000))
        code, response = self._GetMessage()
        if code == 3001:
            print('Another user is already connected, closing connection.')
            self.connected = False
            return False
        elif code == 3000:
            self.connected = True
            return True
        else:
            print('Unexpected code returned.')
            print(f'response: {response}')
            raise RuntimeError

    def Activate(self):
        if self.GetStatus('activated'): return True
        self.controlClient.send(bytes(f'ActivateRobot\0','ascii'))
        code, response = self._GetMessage()
        if code in [2000, 2001]: return True
        else: return False

    def Home(self):
        if self.GetStatus('homed'): return True
        self.controlClient.send(bytes('Home\0','ascii'))
        code, response = self._GetMessage()
        if code in [2002, 2003]: return True
        else: return False

    def MoveJ(self, joints):
        if not self._checkJoint(joints): return False
        cmd = f'MoveJoints{tuple(joints)}'.replace(" ","")
        self.controlClient.send(bytes(f'{cmd}\0','ascii'))
        code, response = self._GetMessage()
        if code == 3012: return True

    def GetStatus(self, status='all'):
        self.controlClient.send(bytes('GetStatusRobot\0','ascii'))
        code, response = self._GetMessage()
        response_list = response.split(',')
        response_bool = [bool(int(resp)) for resp in response_list]
        if status != 'all':
            if status in self.statusDict.keys():
                return response_bool[self.statusDict[status]]
            else:
                print(f'Use an available value:\n{self.statusDict.keys()}')
        else:
            return response_bool

    def _GetMessage(self):
        raw_code, raw_response = self.controlClient.recv(1024).decode('ascii').split('][')
        code = int(raw_code[1:])
        response = raw_response[:-2]
        return code, response

    def sendCommand(self, cmd):
        self.controlClient.send(bytes(f'{cmd}\0','ascii'))
        raw_code, raw_response = self.controlClient.recv(1024).decode('ascii').split('][')

    def _checkJoint(self, joints):
        if abs(joints[0]) > 175: return False
        if joints[1] < -70 or joints[1] > 90: return False
        if joints[2] < -135 or joints[1] > 70: return False
        if abs(joints[3]) > 170: return False
        if abs(joints[4]) > 115: return False
        if abs(joints[5]) > 180: return False
        return True
