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
import numpy as np

__version__ = "0.0.2"
    
class Interrogator(object):
    def __init__(self, ip_address="192.168.1.166", port=1852, fbg_props=None):
        self.ip_address = ip_address
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.latest_response = ""
        self.sensors = []
        if fbg_props:
            self.create_sensors(fbg_props)
        self.sample_rate = 1000
        self.append_data = False
        self.stream_data = False
        self.data = {}
        self.acq_counter = 0
    
    def connect(self):
        self.socket.connect((self.ip_address, self.port))

    def send_command(self, command, receive=True):
        if command[0] != "#":
            command = "#" + command
        if command[-1] != "\n":
            command += "\n"
        self.socket.send(command.encode("ascii"))
        if receive:
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
        vals = {0: "untriggered",
                1: "software",
                3: "hardware"}
        return vals[int(self.latest_response)]
    @trig_mode.setter
    def trig_mode(self, mode):
        """
        Mode=0 for untriggered, 1 for S/W triggered, and 3 for H/W triggered.
        Alternatively, this may be set as "untriggered", "software", or 
        "hardware".
        """
        if mode == "untriggered":
            mode = 0
        elif mode == "software":
            mode = 1
        elif mode == "hardware":
            mode = 3
        self.send_command("SET_TRIG_MODE {}".format(mode))
        if self.latest_response.decode() != "Setting triggering mode to {}.\n".format(mode):
            raise ValueError("Invalid value for triggering mode.")
            
    @property
    def trig_start_edge(self):
        self.send_command("GET_TRIG_START_EDGE")
        vals = {0: "rising",
                1: "falling"}
        return vals[int(self.latest_response)]
    @trig_start_edge.setter
    def trig_start_edge(self, value):
        """0 for rising, 1 for falling."""
        if value == "rising":
            value = 0
        elif value == "falling":
            value = 1
        self.send_command("SET_TRIG_START_EDGE {}".format(value))
        
    @property
    def trig_stop_type(self):
        """
        This command returns the configured trigger stop event for the 
        connected x30 core. Either edge stop triggering or fixed number of 
        acquisitions will be indicated.
        """
        self.send_command("GET_TRIG_STOP_TYPE")
        vals = {0: "num_acq",
                1: "edge"}
        return vals[int(self.latest_response)]
    @trig_stop_type.setter
    def trig_stop_type(self, value):
        """
        This command sets the trigger stop event from among two choices. The 
        first choice, indicated with a parameter value of zero, is a stop 
        trigger upon reaching a number of acquisitions following the start 
        trigger. The second choice, indicated with a parameter value of 1, is a 
        stop trigger on presence of another edge signal, be it rising or 
        falling, as determined by the #SET_TRIG_STOP_EDGE command.
        """
        if value == "num_acq":
            value = 0
        elif value == "edge":
            value = 1
        self.send_command("SET_TRIG_STOP_TYPE {}".format(value))
        
    @property
    def trig_stop_edge(self):
        self.send_command("GET_TRIG_STOP_EDGE")
        vals = {0: "rising",
                1: "falling"}
        return vals[int(self.latest_response)]
    @trig_stop_edge.setter
    def trig_stop_edge(self, value):
        """0 for rising and 1 for falling."""
        if value == "rising":
            value = 0
        elif value == "falling":
            value = 1
        self.send_command("SET_TRIG_STOP_EDGE {}".format(value))
        
    @property
    def trig_num_acq(self):
        self.send_command("GET_TRIG_NUM_ACQ")
        return int(self.latest_response)
    @trig_num_acq.setter
    def trig_num_acq(self, value):
        """Sets the number of acquisitions following a trigger."""
        self.send_command("SET_TRIG_NUM_ACQ {}".format(value))
        
    @property
    def auto_retrig(self):
        self.send_command("GET_AUTO_RETRIG")
        return bool(int(self.latest_response))
    @auto_retrig.setter
    def auto_retrig(self, value):
        """
        This command configures whether or not the x30 core will automatically 
        retrigger. If #SET_AUTO_RETRIG is set to zero, then only one start 
        trigger event will be observed, and only a single data acquisition 
        event will occur. If #SET_AUTO_RETRIG is set to 1, then the module will 
        continually retrigger on subsequent start trigger events, following 
        each successful stop trigger.
        """
        if value == True:
            value = 1
        elif value == False:
            value = 0
        self.send_command("SET_AUTO_RETRIG {}".format(value))
        
    def sw_trig_start(self):
        """
        This command initiates a software start trigger to the x30 core in 
        S/W triggering mode. The command can also be used to simulate a 
        hardware trigger start when the module is set to hardware triggering 
        mode.
        """
        self.send_command("SW_TRIG_START")
        
    def sw_trig_stop(self):
        """
        This command initiates a software stop trigger to the x30 core in 
        S/W triggering mode. The command can also be used to simulate a 
        hardware trigger stop when the module is set to hardware triggering 
        mode.
        """
        self.send_command("SW_TRIG_STOP")
        
    def set_trigger_defaults(self, on=True):
        """Sets default trigger settings:
          * Hardware triggered by falling edge
          * Stop after rising edge
          * Automatic retriggering on."""
        if on:
            self.trig_mode = "hardware"
            self.trig_start_edge = "falling"
            self.trig_stop_type = "edge"
            self.trig_stop_edge = "rising"
            self.auto_retrig = True
        else:
            self.trig_mode = "untriggered"
            self.trig_start_edge = "rising"
            self.trig_stop_edge = "falling"
            self.trig_stop_type = "edge"
            self.auto_retrig = False
            
    @property
    def capabilities(self):
        self.send_command("GET_CAPABILITIES")
        resp = int(self.latest_response)
        spec_diag_view = bin(resp)[-1]
        sensor_distance = bin(resp)[-2]
        return(spec_diag_view, sensor_distance)
        
    @property
    def ch1_gain(self):
        """Returns channel 1 gain in decibels."""
        self.send_command("GET_CH_GAIN_DB 1")
        return float(self.latest_response)
    @ch1_gain.setter
    def ch1_gain(self, gain):
        self.send_command("SET_CH_GAIN_DB 1 {}".format(gain))
        
    @property
    def ch1_noise_thresh(self):
        self.send_command("GET_CH_NOISE_THRESH 1")
        return float(self.latest_response)
    @ch1_noise_thresh.setter
    def ch1_noise_thresh(self, val):
        self.send_command("SET_CH_NOISE_THRESH 1 {}".format(val))
        
    @property 
    def data_interleave(self):
        self.send_command("GET_DATA_INTERLEAVE")
        return(int(self.latest_response))
    @data_interleave.setter
    def data_interleave(self, value):
        self.send_command("SET_DATA_INTERLEAVE {}".format(value))
        if self.latest_response.decode() == "Data interleave set to {}".format(value):
            self.sample_rate = 1000/value
        
    @property
    def data_rate_divider(self):
        self.send_command("GET_DATA_RATE_DIVIDER")
        return(int(self.latest_response))
    @data_rate_divider.setter
    def data_rate_divider(self, value):
        self.send_command("SET_DATA_RATE_DIVIDER {}".format(value))
        if self.latest_response.decode() == "Data rate divider set to {}".format(value):
            self.sample_rate = 1000/value
            
    @property
    def num_averages(self):
        """Gets number of averages for the first sensor on the first channel."""
        return self.get_num_averages(1, 1)
    @num_averages.setter
    def num_averages(self, value):
        """Sets num averages for all sensors on all channels."""
        self.set_num_averages(value)
            
    def get_num_averages(self, channel_no, sensor_no):
        self.send_command("GET_NUM_AVERAGES {} {}".format(channel_no, sensor_no))
        return(int(self.latest_response))
        
    def set_num_averages(self, avgs, channel_no="", sensor_no=""):
        if channel_no:
            channel_no += " "
        if sensor_no:
            sensor_no += " "
        self.send_command("SET_NUM_AVERAGES " + str(channel_no) \
                          + str(sensor_no) + str(avgs))
        if self.latest_response.decode().split()[0] != "Setting":
            print("Failed to set number of averages")
            
    def get_data(self):
        if self.stream_data:
            respsize = int(self.streaming_socket.recv(10))
            response = self.streaming_socket.recv(respsize)
            if self.stream_iteration < 1:
                token = self.streaming_socket.recv(8)
            else:
                token = response[-8:]
                response = response[:-8]
            self.stream_iteration += 1
        else:
            self.send_command("GET_DATA")
            response = self.latest_response
        status_header = response[:88]
        data = response[88:]
        # unpack the struct into variables
        (
            fs_radix, cur_layer, fw_ver, abcde, #  0 fixed
            fbg_thermistor, knpl, fghij, # 1 fixed
            reserved2, tx_ambient_temp, # 2 fixed
            num_fbg_peaks, num_ffpi_peaks, # 3 fixed
            num_dut1_peaks, num_dut2_peaks, # 4 fixed
            num_dut3_peaks, num_dut4_peaks, # 5 fixed
            acq_counter, qr, reserved7, # 6 fixed
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
        
        self.data_header = {"Serial number": serial_number,
                            "FBG thermistor": fbg_thermistor,
                            "FS radix": fs_radix,
                            "Firmware version": fw_ver,
                            "Acquisition triggered": acq_triggered,
                            "Calibration fault": calibration_fault,
                            "Start of frame": start_of_frame,
                            "Primary fan state": primary_fan_state,
                            "Secondary fan state": secondary_fan_state,
                            "S0 mux state": s0_mux_state,
                            "Percent buffer": buffer,
                            "Header length": header_length,
                            "Header version": header_ver,
                            "Tx ambient temp": tx_ambient_temp,
                            "SM041 mux level": sm041_mux_level,
                            "HW clock div": hw_clk_div,
                            "Granularity": granularity,
                            "Operating mode": operating_mode,
                            "Starting lambda": starting_lambda,
                            "Ending lambda": ending_lambda,
                            "Kernel timestamp (seconds)": kernel_timestamp_seconds,
                            "Kernel timestamp (microseconds)": kernel_timestamp_microseconds,
                            "Kernel timestamp": datetime.datetime.fromtimestamp(kernel_timestamp_seconds),
                            "Triggering mode": triggering_mode,
                            "Error": error,
                            "Acquisition counter": acq_counter}
                     
        self.data_serial_no = serial_number
        self.kernel_timestamp = float(kernel_timestamp_seconds) \
                                + float(kernel_timestamp_microseconds)*1e-6
        
        for n, sensor in enumerate(self.sensors):
            try:
                (sensor.wavelength,) = struct.unpack("<I", data[n*4:(n+1)*4])
                sensor.wavelength /= granularity
            except:
                sensor.wavelength = np.nan
        if self.append_data and (acq_triggered or self.trig_mode == \
                                 "untriggered"):
            self.do_append_data()
        self.acq_counter = acq_counter
            
    def flush_buffer(self, receive=True, verbose=False):
        """
        This command flushes out the contents of the data buffer for the 
        present socket connection, clearing all data and resetting the buffer 
        count to zero.
        """
        self.send_command("FLUSH_BUFFER", receive=receive)
        if verbose and receive:
            print(self.latest_response)
        
    def enable_buffer(self):
        self.send_command("SET_BUFFER_ENABLE 1")
        
    def disable_buffer(self):
        self.send_command("SET_BUFFER_ENABLE 0")
        
    @property
    def buffer_count(self):
        """
        This command returns the number of entries stored in the internal data 
        buffer for the present socket connection. This value can range from 0 
        to 60,000 entries.
        """
        self.send_command("GET_BUFFER_COUNT")
        return int(self.latest_response)
                
    def create_sensors_from_file(self, properties_file="Config/fbg_properties.json"):
        with open(properties_file) as f:
            self.fbg_properties = json.load(f)
        self.create_sensors()
    
    def create_sensors(self, fbg_props=None):
        if fbg_props:
            self.fbg_properties = fbg_props
        self.sensors = [None]*len(self.fbg_properties)
        for name, props in self.fbg_properties.items():
            self.sensors[props["position"]] = Sensor(name, properties=props)
            
    def setup_streaming(self, verbose=False):
        self.setup_append_data()
        self.streaming_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.streaming_socket.connect((self.ip_address, self.port))
        command = "#SET_STREAMING_DATA 1\n"
        self.streaming_socket.send(command.encode("ascii"))
        respsize = int(self.streaming_socket.recv(10))
        response = self.streaming_socket.recv(respsize)
        if verbose:
            print(response)
        self.stream_data = True
        self.stream_iteration = 0
        
    def disable_streaming(self):
        self.send_command("SET STREAMING DATA 0")
            
    def setup_append_data(self):
        self.create_data_dict()
        self.append_data = True
            
    def create_data_dict(self):
        """Technically this just creates new items rather than a new dict."""
        self.data.clear()
        for s in self.sensors:
            self.data[s.name + "_wavelength"] = np.array([])
            if "strain" in s.type:
                self.data[s.name + "_strain"] = np.array([])
            else:
                self.data[s.name + "_" + s.type] = np.array([])
            self.data["timestamp"] = np.array([])
            self.data["time"] = np.array([])
            self.data["serial_no"] = np.array([])
            
    def do_append_data(self):
        self.data["timestamp"] = np.append(self.data["timestamp"], 
                                           self.kernel_timestamp)
        self.data["serial_no"] = np.append(self.data["serial_no"],
                                           self.data_serial_no)
        if len(self.data["time"]) == 0:
            newtime = 0.0
        else:
            delta_t = self.data["timestamp"][-1] - self.data["timestamp"][-2]
            newtime = self.data["time"][-1] + delta_t
        self.data["time"] = np.append(self.data["time"], newtime)
        for s in self.sensors:
            self.data[s.name + "_wavelength"] = np.append(self.data[s.name + "_wavelength"],
                                                          s.wavelength)
            if s.type == "strain" or s.type == "bare strain":
                self.data[s.name + "_strain"] = np.append(self.data[s.name + "_strain"],
                                                          s.strain)
            elif s.type == "temperature":
                self.data[s.name + "_temperature"] = np.append(self.data[s.name + "_temperature"],
                                                               s.temperature)
                                                               
    def sleep(self):
        time.sleep(1/self.sample_rate/2)
            
    def zero_strain_sensors(self):
        self.get_data()
        for sensor in self.sensors:
            if "strain" in sensor.type:
                if not np.isnan(sensor.wavelength):
                    sensor.initial_wavelength = sensor.wavelength
                else:
                    sensor.initial_wavelenth = sensor.nominal_wavelength
    
    def save_settings(self):
        self.send_command("SAVE_SETTINGS")
        if self.latest_response.decode() == "Settings Saved.\n":
            print("Settings saved")
        else:
            print("Saving settings unsuccessful")
            
    def who(self):
        """Returns a list of IP addresses connected to the interrogator."""
        self.send_command("WHO?")
        return self.latest_response.decode()
        
    def whoami(self):
        """Returns the IP address of the remote PC that sent the command."""
        self.send_command("WHOAMI?")
        return self.latest_response.decode()
        
    def set_date(self, datestring):
        self.send_command("SET_DATE {}".format(datestring))
        
    def restart_network(self):
        self.send_command("RESTART_NETWORK", receive=False)
            
    def reboot(self):
        self.send_command("REBOOT")

    def disconnect(self):
        self.socket.close()
        

class Sensor(object):
    def __init__(self, name, properties=None):
        self.name = name
        self.properties = properties
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
        if self.properties:
            self.load_properties()
            
    def load_properties(self, properties=None):
        if properties:
            self.properties = properties
        self.type = self.properties["sensor type"]
        self.position = self.properties["position"]
        try:
            self.part_no = self.properties["part number"]
        except KeyError:
            pass
        try:
            self.serial_no = self.properties["serial number"]
        except:
            pass
        self.nominal_wavelength = self.properties["nominal wavelength"]
        if self.type == "strain":
            self.gage_factor = self.properties["gage factor"]
            self.gage_constant_1 = self.properties["gage constant 1"]
            self.gage_constant_2 = self.properties["gage constant 2"]
            self.cte_specimen = self.properties["CTE of test specimen"]
            self.initial_wavelength = self.nominal_wavelength
        elif self.type == "bare strain":
            self.ke = self.properties["ke"]
            self.initial_wavelength = self.nominal_wavelength
        elif self.type == "temperature":
            self.temp_at_nom_wavelength = self.properties["temperature at nominal wavelength"]
            self.wavelength_offset = self.properties["wavelength offset"]
            self.cal_coeff_0 = self.properties["calibration coeff. 0"]
            self.cal_coeff_1 = self.properties["calibration coeff. 1"]
            self.cal_coeff_2 = self.properties["calibration coeff. 2"]
            self.cal_coeff_3 = self.properties["calibration coeff. 3"]
            self.temp_sens = self.properties["temp. sensitivity"]        
    
    def load_properties_from_file(self, filename="Config/fbg_properties.json"):
        """Reads the properties in JSON format from the given file."""
        with open(filename) as f:
            self.properties = json.load(f)[self.name]
        
    @property
    def strain(self):
        if self.type.lower() == "strain":
            self.wavelength_shift = self.wavelength - self.initial_wavelength
            self.thermal_output = self.temperature_change*\
                    (self.gage_constant_1/self.gage_factor + self.cte_specimen\
                    - self.gage_constant_2) 
            return (self.wavelength_shift/self.initial_wavelength)\
                    *1e6/self.gage_factor - self.thermal_output
        elif self.type.lower() == "bare strain":
            self.wavelength_shift = self.wavelength - self.initial_wavelength
            return self.wavelength_shift/self.initial_wavelength/self.ke
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
    pass
    