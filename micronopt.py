#!/usr/bin/env python
"""
This module is for working with Micron Optics interrogators.

"""
from __future__ import print_function, division
import socket
import struct
import sys

    
class MicronInterrogator(object):
    def __init__(self, ip_address="192.168.1.166", port=1852):
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
        
    @property
    def serial_no(self):
        self.send_command("GET_SN")
        return self.latest_response.decode()

    def get_data(self):
        self.send_command("GET_DATA")
        data = self.latest_response
        # unpack the struct into variables
        (
            fs_radix, cur_layer, fw_ver, abcde,
#            abcde, fw_ver, cur_layer, fs_radix,  # 0 needs parse
            fbg_thermistor, knpl, fghij,
#            fghij, knpl, fbg_thermistor,  # 1 needs parse
            reserved2, tx_ambient_temp,
#            tx_ambient_temp, reserved2,  # 2
            num_ffpi_peaks, num_fbg_peaks,  # 3
            num_dut2_peaks, num_dut1_peaks,  # 4
            num_dut4_peaks, num_dut3_peaks,  # 5
            reserved7, qr, acq_counter3,  # 6 needs parse
            serial_number,  # 7
            kernel_timestamp_microseconds,  # 8
            kernel_timestamp_seconds,  # 9
            kernel_src_buffer, kernel_buffers,  # 10
            error_and_kernel_rt_loc0,  # 11 needs parse
            header_ver, header_length, buffer,
#            header_length, header_ver, buffer,  # 12
            dut2_gain, dut1_gain,  # 13
            dut4_gain, dut3_gain,  # 14
            dut2_noise_thresh, dut1_noise_thresh,  # 15
            dut4_noise_thresh, dut3_noise_thresh,  # 16
            hw_clk_div, peak_data_rate_div,  # 17
            granularity,  # 18
            reserved4,  # 19
            starting_lambda,  # 20
            ending_lambda  # 21
        ) = struct.unpack(
            '<'  # big endian
            'BBBB'  # 0 needs parse
            'BBH'  # 1 needs parse
            'HH'  # 2
            'HH'  # 3
            'HH'  # 4
            'HH'  # 5
            'BBH'  # 6 needs parse
            'I'  # 7
            'I'  # 8
            'I'  # 9
            'HH'  # 10
            'I'  # 11 needs parse
            'HBB'  # 12
            'HH'  # 13
            'HH'  # 14
            'HH'  # 15
            'HH'  # 16
            'HH'  # 17
            'I'  # 18
            'I'  # 19
            'I'  # 20
            'I',  # 21
            data
        )
        print(data)
        print(hex(abcde))
        print(abcde & 0x10)
        # 0 parse abcde
        acq_triggered = bool(abcde & 0x80)
        calibration_fault = bool(abcde & 0x40)
        start_of_frame = bool(abcde & 0x20)
        primary_fan_state = bool(abcde & 0x10)
        secondary_fan_state = bool(abcde & 0x08)
        
        s0_mux_state = bool(abcde & 0x04)
        s1_mux_state = bool(abcde & 0x02)
        s2_mux_state = bool(abcde & 0x01)
        
        # 1 parse fghij
        xfer_type = fghij >> 4
        soa_therm_limit = bool(fghij & 0x08)
        soa_current_limit = bool(fghij & 0x04)
        tec_over_temp = bool(fghij & 0x02)
        tec_under_temp = bool(fghij & 0x01)
        
        # 1 parse knpl
        operating_mode = knpl >> 6
        triggering_mode = (knpl & 0x30) >> 4
        sm041_mux_level = (knpl & 0x0c) >> 2
        sw_position = knpl & 0x03
        
        # 6 parse qr
        nrz_command = qr >> 5
        reserved6 = qr & 0x1f
        
        # 11 parse
        error = error_and_kernel_rt_loc0 >> 24
        kernel_rt_loc0 = error_and_kernel_rt_loc0 & 0xffffff
        
        print("Serial number", serial_number)
        print("FBG thermistor:", fbg_thermistor)
        print("FS radix:", fs_radix)
        print("Firmware version:", fw_ver)
        print("Acquisition triggered:", acq_triggered)
        print("Calibration fault:", calibration_fault)
        print("start_of_frame:", start_of_frame)
        print("Primary fan state:", primary_fan_state)
        print("Secondary fan state:", secondary_fan_state)
        print("s0_mux_state:", s0_mux_state)
        
        print("Percent buffer:", buffer)
        
        print("Header length:", header_length)
        print("Header version:", header_ver)
        
        print("Tx ambient temp:", tx_ambient_temp)
        
        print("sm041_mux_level:", sm041_mux_level)
        
        print("hw_clk_div:", hw_clk_div)
        
        print("Granularity:", granularity)
        
        print("Operating mode:", operating_mode)
        
        print("Starting lambda:", starting_lambda)
        print("Ending lambda:", ending_lambda)

    def disconnect(self):
        self.socket.close()


def test_class():
    interr = MicronInterrogator()
    interr.connect()
    print(interr.idn)
    interr.get_data()
    print(interr.serial_no)

def terminal(ip_address="192.168.1.166", port=1852):
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
    test_class()
#    terminal()