#!/usr/bin/python3

# QMesh
# Copyright (C) 2022 Daniel R. Fay

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import serial
import sys
import time
import threading
import binascii
import time
from threading import Thread, Lock


ser = None


if __name__ == "__main__":
    # Open the serial port
    print("Opening serial port...")
    serial_ports = sys.argv[1:]
    while True:
        try:
            print("Trying serial port " + str(serial_ports[0]))
            ser = serial.Serial(serial_ports[0], baudrate=115200, timeout=0.1)
            break
        except serial.serialutil.SerialException:
            print("Failed to open port. Trying again in 1s...")
            if len(serial_ports) > 1:
                serial_ports = serial_ports[1:] + [serial_ports[0]]
            time.sleep(1)
    
    
    # Configure the ESP32
    
    # Set Wi-Fi to STA mode
    ser.write('AT+CWMODE=2\r\n')
    print(ser.readlines())
    # Set SSID, password, channel, and security type
    ser.write("AT+CWSAP=\"QMESH-KISS0\",\"password\",6,3\r\n")
    print(ser.readlines())
    time.sleep(1)
    # Set the address of the ESP32 SoftAP
    ser.write("AT+CIPAP=\"192.168.5.1\",\"192.168.5.1\",\"255.255.255.0\"\r\n")
    print(ser.readlines())
    # Enable DHCP
    ser.write("AT+CWDHCP=1,2\r\n")
    print(ser.readlines())
    # Set the DHCP address range
    ser.write("AT+CWDHCPS=1,3,\"192.168.5.10\",\"192.168.5.15\"\r\n")
    print(ser.readlines())
                        
    # Start the TCP server
    ser.write("AT+CIPMUX=1\r\n")
    print(ser.readlines())
    #ser.write("AT+CIPSERVERMAXCONN=1\r\n")
    ser.write("AT+CIPSERVERMAXCONN?\r\n")
    print(ser.readlines())
    ser.write("AT+CIPSERVER=1,8080\r\n")
    print(ser.readlines())
    
    # Wait for a connection
    while(True):
        lines_in = ser.readlines()
        if len(lines_in) > 0 : print(lines_in)
        found_connect = False
        for line_in in lines_in:
            if line_in == "0,CONNECT\r\n": 
                found_connect = True
                break
        if found_connect:
            break
    
    # Set up the serial passthrough
    #ser.write("AT+CIPMODE=1\r\n")
    #print(ser.readlines())
    ser.write("AT+CIPSEND\r\n")
    print(ser.readlines())
    
    

