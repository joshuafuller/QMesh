import serial
import sys, os
import logging
import json
import base64


# Open the serial port
serial_port = sys.argv[1]
ser = serial.Serial(serial_port, baudrate=921600)

while True:
    line = ser.getLine()
    parsed_line = json.loads(line)
    if parsed_line["Type"] == "Debug Msg":
        msg = base64.b64decode(unparsed_line["Message"])
        print(msg);
    elif parsed_line["Type"] == "Status":
        status = parsed_line["Status"]
        value = parsed_line["Value"]
        print("Status Msg: %s, %s" % (status, value))

