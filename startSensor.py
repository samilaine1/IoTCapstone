import smbus2
import time
from smbus2 import i2c_msg
import numpy as np
import struct
import json
import postData
import os
from datetime import datetime
loop_counter = 1

def date_info():
	now = datetime.now()
	dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
	return dt_string
	
def pack_struct(i, i1, i2, i3, measurement):
	return struct.pack('4B', extract_measurement(i, measurement), extract_measurement(i1, measurement), extract_measurement(i2, measurement), extract_measurement(i3, measurement))
	
def extract_measurement(i, measurement):
	return np.array(list(measurement))[i]
	
def crc8_update(b, crc):
	crc = crc ^ b
	for i in range (8):
		if (crc & 0x80) == 0x80:
			crc = (crc << 1) ^ 0x131
		else:
			crc <<= 1
	return crc
	
def calc_crc(d):
	crc = crc8_update((d >> 8) & 0x00FF, 0xff)
	crc = crc8_update((d & 0x00FF), crc)
	return crc
	
def get_crc(i, j, measurement):
	crc = crc8_update(np.array(list(measurement))[i], 0xff)
	crc = crc8_update(np.array(list(measurement))[j], crc)
	return crc
	
def get_measurement(measurement, val_list):
	crc = get_crc(val_list[0], val_list[1], measurement)
	unpacked_measurement = None
	
	if crc == extract_measurement(val_list[2], measurement):
		crc = get_crc(val_list[3], val_list[4], measurement)
		if crc == extract_measurement(val_list[5], measurement):
			tmp = pack_struct(val_list[0], val_list[1], val_list[3], val_list[4], measurement)
			unpacked_measurement = struct.unpack('>f', tmp)
		return unpacked_measurement
		
def create_cmd(payload, x_):
	ff = 0x00FF
	i = 8
	return [x_, (payload >> i) & ff, (payload & ff), calc_crc(payload)]
	
	
class SCD30():
	def __init__(self, measurement_interval, auto_calibration, pressure):
		super(SCD30, self).__init__()
		
		self.i2c_address = 0x61
		self.bus_number = smbus2.SMBus(1)
		
		self.set_measurement_interval(measurement_interval)
		time.sleep(0.1)
		self.set_auto_calibration(auto_calibration)
		time.sleep(0.1)
		self.start_periodic_measurement(pressure)
		time.sleep(0.1)
		
		#Measurement interval
	def set_measurement_interval(self, interval):
		try:
			cmd_set_measurement_interval = create_cmd(interval, 0x00)
			self.bus_number.write_i2c_block_data(self.i2c_address, 0X46, cmd_set_measurement_interval)
		except OSError as e:
			print("set_measurement_interval: " + str(e))
				
		#Periodic measurement
	def start_periodic_measurement(self, pressure):
		try:
			cmd_start_periodic_measurement = create_cmd(pressure, 0x10)
			self.bus_number.write_i2c_block_data(self.i2c_address, 0X00, cmd_start_periodic_measurement)
			time.sleep(0.1)
			print("%7.2f hPa 2" % pressure)
		except OSError as e:
			print("start_periodic_measurement: " + str(e))
				
		#Auto calibration
	def set_auto_calibration(self, on_off):
		try:
			cmd_set_auto_calibration = create_cmd(on_off, 0x06)
			self.bus_number.write_i2c_block_data(self.i2c_address, 0x53, cmd_set_auto_calibration)
		except OSError as e:
			print("set_auto_calibration: " + str(e))
	
	def data_ready(self):
		try:
			self.bus_number.write_byte_data(self.i2c_address, 0x02, 0x02)
			time.sleep(0.1)
			data_ready = i2c_msg.read(self.i2c_address, 3)
			time.sleep(0.1)
			self.bus_number.i2c_rdwr(data_ready)
			
			return bool(np.array(list(data_ready))[1])
		except OSError as e:
			print("OSError in scd3 data_ready: " + str(e))
			return False
	
		#Read measurement buffer
	def get_scd30_measurements(self, ):
		try:
			self.bus_number.write_byte_data(self.i2c_address, 0x03, 0x00)
			time.sleep(0.1)
			
			measurement = i2c_msg.read(self.i2c_address, 18)
			self.bus_number.i2c_rdwr(measurement)
				
			co2_list = [0, 1, 2, 3, 4, 5]
			co2_measurement = get_measurement(measurement, co2_list)
				
			temperature_list = [6, 7, 8, 9, 10, 11]
			temperature_measurement = get_measurement(measurement, temperature_list)
				
			humidity_list = [12, 13, 14, 15, 16, 17]
			humidity_measurement = get_measurement(measurement, humidity_list)
				
			if co2_measurement and humidity_measurement and temperature_measurement:
				write_json(temperature_measurement, humidity_measurement, co2_measurement)
				return co2_measurement, temperature_measurement, humidity_measurement
			else:
				return None
			
		except OSError as e:
			print("OSError in scd30 get_scd30_measurements: " + str(e))
			return None

def write_json(temperature_measurement, humidity_measurement, co2_measurement):
	global loop_counter
	jsonInfo = {
		"Date": date_info(),
		"Temperature": str(temperature_measurement)[1:-2],
		"Humidity": str(humidity_measurement)[1:-2],
		"CO2": str(co2_measurement)[1:-2]
		}
	print (jsonInfo)
	with open('sensorData.json', 'a') as sensorData:
		if loop_counter == 1:
			sensorData.write('[\n')
			json.dump(jsonInfo, sensorData)
			sensorData.write(',\n')
			loop_counter +=1
		elif loop_counter == 5:
			json.dump(jsonInfo, sensorData)
			sensorData.write('\n]')
			loop_counter +=1
		else:
			json.dump(jsonInfo, sensorData)
			sensorData.write(',\n')
			loop_counter +=1
	if loop_counter == 6:
		loop_counter = 1
		postData.post()
		os.remove("sensorData.json")
		exit()

if __name__ == "__main__":

	try:
		scd30 = SCD30(5, 0, 1000)

		while(1):
			if scd30.data_ready():
				scd30.get_scd30_measurements()
			else:
				print("Sensor not ready")
			time.sleep(5)
	except (KeyboardInterrupt, SystemError):
		print:("Stopped")
