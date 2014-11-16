# -*- coding: utf-8 -*-
"""
Created on Sun Nov 16 17:35:49 2014

@author: Pete
"""

from micronopt import Interrogator, Sensor
import time

def test_connection():
    interr = Interrogator()
    interr.connect()
    print(interr.idn)
    interr.get_data()
    print(interr.capabilities)
    interr.who()
    interr.disconnect()
    
def test_continuous(test_dur=5):
    import matplotlib.pyplot as plt
    interr = Interrogator()
    interr.connect()
    interr.create_sensors_from_file("test/fbg_properties.json")
    interr.zero_strain_sensors()
    data = interr.data
    interr.setup_append_data()
    interr.data_rate_divider = 1
    interr.data_interleave = 10
    interr.set_num_averages(10)
    print(interr.data_rate_divider)
    print(interr.data_interleave)
    print(interr.get_num_averages(1, 1))
    t0 = time.time()
    while time.time() - t0 < test_dur:
        interr.get_data()
        interr.sleep()
    t = data["time"]
    data1 = data[interr.sensors[0].name + "_temperature"]
    data2 = data[interr.sensors[1].name + "_strain"]
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
    
def test_continuous_hwtrigger(test_dur=5):
    import matplotlib.pyplot as plt
    interr = Interrogator()
    interr.connect()
    interr.create_sensors_from_file("test/fbg_properties.json")
    interr.set_trigger_defaults()
    interr.zero_strain_sensors()
    data = interr.data
    interr.setup_append_data()
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
        
if __name__ == "__main__":
    data = test_continuous()