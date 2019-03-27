import serial
import sys, os
import logging



print("Now opening the selected serial port")
while(True):
	print("Please select a serial port")
	ser_port = input()
	print("You selected COM" + str(ser_port))	
	print("Now opening the selected serial port")
	try: 
		ser = serial.Serial('COM' + str(ser_port), 115200)
		break
	except serial.SerialException: 
		print("ERROR: Cannot open serial port.")
		print("")
