#!/usr/bin/env python
"""
This module is for working with Micron Optics interrogators.

"""
from __future__ import print_function, division
import socket
import struct
import sys

    
class MicronInterrogator(object):
    def __init__(self, ip_address="10.0.0.126", port=1852):
        self.ip_address = ip_address
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.latest_response = ""
    
    def connect(self):
        self.socket.connect((self.ip_address, self.port))

    def send_command(self, command):
        if command[0] != "#":
            command = "#" + command
        if command[-1] != "\n":
            command += "\n"
        self.socket.send(command.encode("ascii"))
        respsize = int(self.socket.recv(10))
        self.latest_response = self.socket.recv(respsize)

    @property
    def idn(self):
        self.send_command("IDN?")
        return self.latest_response.decode()

    def get_data(self):
        self.send_command("GET_DATA")
        self.data = self.latest_response
        data = struct.unpack("88B", self.data)
        print(data)

    def disconnect(self):
        self.socket.close()

def test_class():
    interr = MicronInterrogator()
    interr.connect()
    print(interr.idn)
    interr.get_data()

def terminal(ip_address="10.0.0.126", port=1852):
    """Creates a communcation terminal to send commands."""
    if sys.version_info >= (3, 0):
        input_function = input
        raw_input = None
    else:
        input_function = raw_input
    message = "#IDN?\n"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip_address, port))
    while message != "exit":
        message = input_function("#")
        if message != "exit":
            s.send(b"#" + message.encode("ascii") + b"\n")
            respsize = int(s.recv(10))
            response = s.recv(respsize)
            print(response)
    s.close()

if __name__ == "__main__":
#    test_class()
    terminal()