#!/usr/bin/env python
"""
This module is for working with Micron Optics interrogators.

"""
from __future__ import print_function, division
import socket
import sys

if sys.version_info >= (3, 0):
    input_function = input
else:
    input_function = raw_input
    
class MicronInterrogator(object):
    def __init__(self, ip_address="10.0.0.126", port=1852):
        self.ip_address = ip_address
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.latest_response = ""
    def connect(self):
        self.socket.connect((self.ip_address, self.port))
    def send_command(self, command, buffsize=1024):
        if command[0] != "#":
            command = "#" + command
        if command[-1] != "\n":
            command += "\n"
        self.socket.send(command.encode("ascii"))
        self.latest_response = socket.recv(buffsize)
    def get_id(self, verbose=True):
        self.send_command("ID?")
        print(self.latest_response)
    def get_data(self):
        self.send_command("GET_DATA")


def test():
    ip_address = "10.0.0.126"
    port = 1852
    buffsize = 1024
    message = "#IDN?\n"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip_address, port))
    while message != "exit":
        message = input_function("~ #")
        if message != "exit":
            s.send(b"#" + message.encode("ascii") + b"\n")
            response = s.recv(buffsize)
            print(response[10:])
    s.close()

if __name__ == "__main__":
    test()