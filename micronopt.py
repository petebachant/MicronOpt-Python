#!/usr/bin/env python
"""
This module is for working with Micron Optics interrogators.

"""
from __future__ import print_function, division
import datetime
import socket
import struct
import time
import json
import sys

    
class MicronInterrogator(object):
    def __init__(self, ip_address="192.168.1.166", port=1852):
        self.ip_address = ip_address
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.latest_response = ""
        self.sensors = []
    
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
        
    @property
    def operating_mode(self):
        self.send_command("GET_OPERATING_MODE")
        return int(self.latest_response)
    @operating_mode.setter
    def operating_mode(self, mode):
        self.send_command("SET_OPERATING_MODE {}".format(mode))
        if self.latest_response.decode() != "Setting Operating mode to {}.\n".format(mode):
            raise ValueError("Invalid value for operating mode.")
            
    @property
    def trig_mode(self):
        self.send_command("GET_TRIG_MODE")
        return int(self.latest_response)
    @trig_mode.setter
    def trig_mode(self, mode):
        self.send_command("SET_TRIG_MODE {}".format(mode))
        if self.latest_response.decode() != "Setting triggering mode to {}.\n".format(mode):
            raise ValueError("Invalid value for triggering mode.")
            
    def get_data(self):
        self.send_command("GET_DATA")
        status_header = self.latest_response[:88]
        data = self.latest_response[88:]
        # unpack the struct into variables
        (
            fs_radix, cur_layer, fw_ver, abcde, #  0 fixed
            fbg_thermistor, knpl, fghij, # 1 fixed
            reserved2, tx_ambient_temp, # 2 fixed
            num_fbg_peaks, num_ffpi_peaks, # 3 fixed
            num_dut1_peaks, num_dut2_peaks, # 4 fixed
            num_dut3_peaks, num_dut4_peaks, # 5 fixed
            acq_counter3, qr, reserved7, # 6 fixed
            serial_number,  # 7
            kernel_timestamp_microseconds,  # 8
            kernel_timestamp_seconds,  # 9
            kernel_buffers, kernel_src_buffer, # 10 fixed
            error_and_kernel_rt_loc0,  # 11 needs parse
            buffer, header_ver, header_length, # 12 fixed
            dut1_gain, dut2_gain, # 13 fixed
            dut3_gain, dut4_gain, # 14 fixed
            dut1_noise_thresh, dut2_noise_thresh, # 15 fixed
            dut3_noise_thresh, dut4_noise_thresh, # 16 fixed
            peak_data_rate_div, hw_clk_div, # 17 fixed
            granularity,  # 18
            reserved4,  # 19
            starting_lambda,  # 20
            ending_lambda  # 21
        ) = struct.unpack(
            '<'  # big endian
            'BBBB'  # 0 needs parse
            'HBB'  # 1 needs parse
            'HH'  # 2
            'HH'  # 3
            'HH'  # 4
            'HH'  # 5
            'HBB'  # 6 needs parse
            'I'  # 7
            'I'  # 8 
            'I'  # 9
            'HH'  # 10
            'I'  # 11 needs parse
            'BBH'  # 12
            'HH'  # 13
            'HH'  # 14
            'HH'  # 15
            'HH'  # 16
            'HH'  # 17
            'I'  # 18
            'I'  # 19
            'I'  # 20
            'I',  # 21
            status_header
        )

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
        
        data_dict = {"Serial number" : serial_number,
                     "FBG thermistor" : fbg_thermistor,
                     "FS radix" : fs_radix,
                     "Firmware version" : fw_ver,
                     "Acquisition triggered" : acq_triggered,
                     "Calibration fault" : calibration_fault,
                     "Start of frame" : start_of_frame,
                     "Primary fan state" : primary_fan_state,
                     "Secondary fan state" : secondary_fan_state,
                     "S0 mux state " : s0_mux_state,
                     "Percent buffer" : buffer,
                     "Header length" : header_length,
                     "Header version " : header_ver,
                     "Tx ambient temp" : tx_ambient_temp,
                     "SM041 mux level" : sm041_mux_level,
                     "HW clock div" : hw_clk_div,
                     "Granularity" : granularity,
                     "Operating mode" : operating_mode,
                     "Starting lambda" : starting_lambda,
                     "Ending lambda" : ending_lambda,
                     "Kernel timestamp (seconds)" : kernel_timestamp_seconds,
                     "Kernel timestamp (microseconds)" : kernel_timestamp_microseconds,
                     "Kernel timestamp" : datetime.datetime.fromtimestamp(kernel_timestamp_seconds),
                     "Triggering mode" : triggering_mode}
#        for k,v in sorted(data_dict.items()):
#            print("{}: {}".format(k,v))
                     
        self.data_serial_no = serial_number
        
        datapoints = len(data)//4
        for n in range(datapoints):
            (self.sensors[n].wavelength,) = struct.unpack("<I", data[n*4:(n+1)*4])
            self.sensors[n].wavelength /= granularity
                
    def add_sensors(self, properties_file="Config/fbg_properties.json"):
        with open(properties_file) as f:
            sensor_props = json.load(f)
        self.sensors = [None]*len(sensor_props)
        for sensor_name, properties in sensor_props.items():
            self.sensors[properties["position"]] = Sensor(sensor_name)
        for sensor in self.sensors:
            sensor.read_properties(properties_file)
            
    def zero_strain_sensors(self):
        self.get_data()
        for sensor in self.sensors:
            if sensor.type == "strain":
                sensor.initial_wavelength = sensor.wavelength

    def disconnect(self):
        self.socket.close()
        

class Sensor(object):
    def __init__(self, sensor_name):
        self.name = sensor_name
        self.properties = {}
        self.position = None
        self.type = None
        self.part_no = None
        self.serial_no = None
        self.nominal_wavelength = None
        self.gage_factor = None
        self.gage_constant_1 = None
        self.gage_constant_2 = None
        self.temperature_change = 0.0
        self.cte_specimen = None
        self.wavelength_shift = None
        self.wavelength = None
        self.initial_wavelength = None
        self.wavelength_offset = None
        self.cal_coeff_1 = None
        self.cal_coeff_2 = None
        self.cal_coeff_3 = None
        self.cal_coeff_0 = None
        self.temp_sens = None
    
    def read_properties(self, filename="Config/fbg_properties.json"):
        """Reads the properties in JSON format from the given file."""
        with open(filename) as f:
            self.properties = json.load(f)[self.name]
        self.type = self.properties["sensor type"]
        self.position = self.properties["position"]
        self.part_no = self.properties["part number"]
        self.serial_no = self.properties["serial number"]
        self.nominal_wavelength = self.properties["nominal wavelength"]
        if self.type == "strain":
            self.gage_factor = self.properties["gage factor"]
            self.gage_constant_1 = self.properties["gage constant 1"]
            self.gage_constant_2 = self.properties["gage constant 2"]
            self.cte_specimen = self.properties["CTE of test specimen"]
        elif self.type == "temperature":
            self.temp_at_nom_wavelength = self.properties["temperature at nominal wavelength"]
            self.wavelength_offset = self.properties["wavelength offset"]
            self.cal_coeff_0 = self.properties["calibration coeff. 0"]
            self.cal_coeff_1 = self.properties["calibration coeff. 1"]
            self.cal_coeff_2 = self.properties["calibration coeff. 2"]
            self.cal_coeff_3 = self.properties["calibration coeff. 3"]
            self.temp_sens = self.properties["temp. sensitivity"]
        
    @property
    def strain(self):
        if self.type.lower() == "strain":
            self.wavelength_shift = self.wavelength - self.initial_wavelength
            self.thermal_output = self.temperature_change*\
                    (self.gage_constant_1/self.gage_factor + self.cte_specimen\
                    - self.gage_constant_2) 
            return (self.wavelength_shift/self.nominal_wavelength)\
                    *1e6/self.gage_factor - self.thermal_output
        else:
            return None
            
    @property
    def temperature(self):
        if self.type.lower() == "temperature":
            return self.cal_coeff_3*(self.wavelength + self.wavelength_offset)**3 \
                    + self.cal_coeff_2*(self.wavelength + self.wavelength_offset)**2 \
                    + self.cal_coeff_1*(self.wavelength + self.wavelength_offset) \
                    + self.cal_coeff_0
        else:
            return None


def test_connection():
    interr = MicronInterrogator()
    interr.connect()
    print(interr.idn)
    interr.get_data()
    interr.disconnect()
    
def test_continuous(test_dur=20):
    import matplotlib.pyplot as plt
    interr = MicronInterrogator()
    interr.connect()
    interr.add_sensors("test/fbg_properties.json")
    interr.zero_strain_sensors()
    t0 = time.time()
    t = 0.0
    t_array = []
    data1 = []
    data2 = []
    serial_no = []
    while t < test_dur:
        t = time.time() - t0
        t_array.append(t)
        interr.get_data()
        data1.append(interr.sensors[0].temperature)
        data2.append(interr.sensors[1].strain)
        serial_no.append(interr.data_serial_no)
    for i, s in enumerate(serial_no):
        if i < len(serial_no) - 1:
            if serial_no[i + 1] - s != 1:
                print("Datapoint {} is not sequential".format(i))
    plt.plot(t_array, data2)
    interr.disconnect()
    
def test_sensor_class(name="os4300"):
    sensor = Sensor(name)
    sensor.read_properties("test/fbg_properties.json")
    print(sensor.name)
    
def test_add_sensors():
    micron = MicronInterrogator()
    micron.add_sensors("test/fbg_properties.json")
    for sensor in micron.sensors:
        print(sensor.name)
        print(sensor.properties)

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
    test_continuous()
#    test_sensor_class()
#    test_add_sensors()
    