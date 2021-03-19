#!/usr/bin/env python3
import socket
import threading

class Robot:

    def __init__(self, ip, firmware):
        self.ip = ip   
        self.firmware = firmware 
        self.connected = False

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
        x = threading.Thread(target=self.sendCommand, args=('ActivateRobot',))
        x.start()   
        self.controlClient.send(bytes('ActivateRobot\0','ascii'))
        code, response = self._GetMessage()
        print(code, response)
        return code

    def Home(self):
        self.controlClient.send(bytes('Home\0','ascii'))
        code, response = self._GetMessage()
        print(code, response)
        return code

    def _GetMessage(self):
        raw_code, raw_response = self.controlClient.recv(1024).decode('ascii').split('][')
        code = int(raw_code[1:])
        response = raw_response[:-2]
        return code, response

    def sendCommand(self, cmd):
        self.controlClient.send(bytes(f'{cmd}\0','ascii'))
        raw_code, raw_response = self.controlClient.recv(1024).decode('ascii').split('][')
