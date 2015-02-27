# -*- coding: utf-8 -*-
"""
Created on Sun Nov 16 17:35:49 2014

@author: Pete
"""
from __future__ import division, print_function
from micronopt import Interrogator, Sensor
import time
import matplotlib.pyplot as plt

def test_connection():
    interr = Interrogator()
    interr.connect()
    print(interr.idn)
    interr.get_data()
    print(interr.capabilities)
    interr.who()
    interr.disconnect()
    
def test_continuous(test_dur=3):
    interr = Interrogator()
    interr.connect()
    interr.create_sensors_from_file("test/fbg_properties.json")
    interr.set_trigger_defaults(False)
    interr.zero_strain_sensors()
    data = interr.data
    interr.setup_append_data()
    interr.data_interleave = 2
    interr.num_averages = 2
    print(interr.data_rate_divider)
    print(interr.data_interleave)
    print(interr.get_num_averages(1, 1))
    t0 = time.time()
    while time.time() - t0 < test_dur:
        interr.get_data()
        interr.sleep()
    t = data["time"]
    data1 = data[interr.sensors[0].name + "_wavelength"]
    data2 = data[interr.sensors[1].name + "_wavelength"]
    plt.plot(t, data2)
    plt.xlabel("t (s)")
    plt.ylabel(r"Wavelength")
    plt.figure()
    plt.plot(t, data1)
    plt.xlabel("t (s)")
    plt.ylabel("Wavelength")
    print(interr.data_header)
    interr.disconnect()
    return data
    
def test_continuous_hwtrigger(test_dur=3):
    interr = Interrogator()
    interr.connect()
    interr.create_sensors_from_file("test/fbg_properties.json")
    interr.trig_mode = "untriggered"
    interr.auto_retrig = True
    interr.zero_strain_sensors()
    interr.trig_mode = "hardware"
    interr.trig_start_edge = "falling"
    interr.trig_stop_type = "edge"
    interr.trig_stop_edge = "rising"
    interr.auto_retrig = True
    print(interr.trig_mode)
    interr.setup_append_data()
    data = interr.data
    t0 = time.time()
    while time.time() - t0 < test_dur:
        interr.get_data()
        interr.sleep()
    t = data["time"]
    data2 = data[interr.sensors[1].name + "_strain"]
    try:
        data2 -= data2[0]
    except IndexError:
        pass
    plt.figure()
    plt.plot(t, data2)
    plt.xlabel("t (s)")
    plt.ylabel("T (deg. C)")
    print(interr.data_header)
    interr.disconnect()
    return data
    
def test_continuous_swtrigger(test_dur=3):
    interr = Interrogator()
    interr.connect()
    interr.create_sensors_from_file("test/fbg_properties.json")
    interr.trig_mode = "software"
    interr.trig_start_edge = "rising"
    interr.trig_stop_type = "edge"
    interr.trig_stop_edge = "rising"
    interr.auto_retrig = False
    print(interr.trig_mode)
    print(interr.trig_stop_type)
    interr.sw_trig_start()
    interr.zero_strain_sensors()
    data = interr.data
    interr.setup_append_data()
    t0 = time.time()
    while time.time() - t0 < test_dur:
        interr.get_data()
        interr.sleep()
    t = data["time"]
    data2 = data[interr.sensors[1].name + "_wavelength"]
    try:
        data2 -= data2[0]
    except IndexError:
        pass
    plt.figure()
    plt.plot(t, data2)
    plt.xlabel("t (s)")
    plt.ylabel("Wavelength (nm)")
    print(interr.data_header)
    interr.disconnect()
    return data
    
def test_num_acq_hwtrigger(test_dur=3):
    interr = Interrogator()
    interr.connect()
    interr.flush_buffer()
    interr.create_sensors_from_file("test/fbg_properties.json")
    interr.trig_mode = "hardware"
    interr.trig_start_edge = "falling"
    interr.trig_stop_type = "num_acq"
    interr.trig_num_acq = 1000
    interr.trig_stop_edge = "rising"
    interr.auto_retrig = False
    interr.zero_strain_sensors()
    data = interr.data
    interr.setup_append_data()
    t0 = time.time()
    while time.time() - t0 < test_dur:
        interr.get_data()
        interr.sleep()
    t = data["time"]
    data2 = data[interr.sensors[1].name + "_strain"]
    try:
        data2 -= data2[0]
    except IndexError:
        pass
    plt.figure()
    plt.plot(t, data2)
    plt.xlabel("t (s)")
    plt.ylabel("T (deg. C)")
    print(interr.data_header)
    interr.disconnect()
    return data
    
def test_streaming(test_dur=2):
    interr = Interrogator()
    interr.connect()
    interr.create_sensors_from_file("test/fbg_properties.json")
    interr.set_trigger_defaults(False)
    interr.zero_strain_sensors()
    interr.data_interleave = 2
    interr.set_num_averages = 2
    interr.setup_streaming()
    data = interr.data
    t0 = time.time()
    while time.time() - t0 < test_dur:
        interr.get_data()
        interr.sleep()
    t = data["time"]
    data1 = data[interr.sensors[0].name + "_temperature"]
    data2 = data[interr.sensors[1].name + "_strain"]
    try:
        data2 -= data2[0]
    except IndexError:
        pass
    plt.plot(t, data2)
    plt.xlabel("t (s)")
    plt.ylabel(r"$\mu$-strain")
    plt.figure()
    plt.plot(t, data1)
    plt.xlabel("t (s)")
    plt.ylabel("T (deg. C)")
    print(interr.data_header)
    interr.disconnect()
    return data
    
def test_sensor_class(name="os4300"):
    sensor = Sensor(name)
    sensor.read_properties("test/fbg_properties.json")
    print(sensor.name)
    
def test_add_sensors():
    micron = Interrogator()
    micron.add_sensors("test/fbg_properties.json")
    for sensor in micron.sensors:
        print(sensor.name)
        print(sensor.properties)
        
def test_flush_buffer(test_dur=2):
    interr = Interrogator()
    interr.connect()
    interr.create_sensors_from_file("test/fbg_properties.json")
    interr.set_trigger_defaults(False)
    interr.zero_strain_sensors()
    interr.setup_append_data()
    t0 = time.time()
    while time.time() - t0 < test_dur:
        interr.get_data()
        interr.sleep()
    print("Current buffer:", interr.buffer_count)
    interr.flush_buffer(verbose=True)
    print("Buffer after flushing:", interr.buffer_count)
    
def test_reboot():
    interr = Interrogator()
    interr.connect()
    interr.reboot()
    interr.disconnect()
        
if __name__ == "__main__":
    test_connection()
#    test_reboot()
#    data = test_continuous(2)
#    data = test_streaming(2)
    test_continuous_hwtrigger(5)
#    test_continuous_swtrigger(2)
#    test_num_acq_hwtrigger(10)
#    test_flush_buffer()