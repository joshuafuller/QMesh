# QMesh
# Copyright (C) 2019 Daniel R. Fay

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

#!/usr/bin/python3

import sys, os
import time
import json
import base64
import pika

# For now, let's just put the configuration in the same
# file as the one setting it.
settings = {}
settings["Freq"] = 433000000
settings["BW"] = 2 # 0=125KHz, 1=250KHz, 2=500KHz
settings["CR"] = 4
settings["SF"] = 12
settings["Preamble Length"] = 12
settings["Preamble Slots"] = 1
beacon_str = "KG5VBY Beacon Test"
settings["Beacon Message"] = base64.b64encode(beacon_str)
settings["Beacon Interval"] = 10
settings["Payload Length"] = len(beacon_str)
settings["FEC Algorithm"] = "None"
settings["Convolutional Rate"] = 2
settings["Convolutional Order"] = 9
settings["Reed-Solomon Number Roots"] = 32
settings["TX Power"] = 22

settings_rx = False
cur_status = "None"

def dbg_process(ch, method, properties, body):
    line = body.decode('utf-8')
    parsed_line = {}
    parsed_line["Type"] = ""
    try: parsed_line = json.loads(line)
    except Exception as e: pass
    if parsed_line["Type"] == "Settings":
        print("----------CONFIGURATION----------")
        print("Frequency: " + str(parsed_line["Freq"]))
        print("Coding Rate: " + str(parsed_line["CR"]))
        print("Spreading Factor: " + str(parsed_line["SF"]))
        print("Bandwidth: " + str(parsed_line["BW"]))
        print("Mode: " + str(parsed_line["Mode"]))
        print("Preamble Length: " + str(parsed_line["Preamble Length"]))
        print("Num Preamble Slots: " + str(parsed_line["Preamble Slots"]))
        print("Payload Length: " + str(parsed_line["Payload Length"]))
        print("Beacon Interval (s): " + str(parsed_line["Beacon Interval"]))
        msg = base64.b64decode(parsed_line["Beacon Message"]).decode('utf-8')
        print("Beacon Message: " + str(msg))
		print("FEC Algorithm: " + str(parsed_line"FEC Algorithm"))
	print("Convolution Rate: " + str(parsed_line["Convolutional Rate"]))
	print("Convolution Order: " + str(parsed_line["Convolutional Order"]))
	print("Reed-Solomon Number of Roots: " + \ 
                str(parsed_line["Reed-Solomon Number Roots"]))
        print("Transmit power: " + str(parsed_line["TX Power"]))
        # Save the settings to a file
	settings_file = open("board_settings.json", "w")
	settings_file.write(parsed_line)
	settings_file.close()		
        settings_rx = True
    elif parsed_line["Type"] == "Status":
        cur_status = parsed_line["Status"]
        print("Current status is " + parsed_line["Status"])
        print("Current time is " + str(parsed_line["Time"])

# Set up the RabbitMQ connection
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()
channel.queue_declare(queue='board_input')
channel.queue_declare(queue='board_output')
channel.basic_consume(queue='board_output', auto_ack=True, \
        on_message_callback=dbg_process)
channel.start_consuming()


# Check what mode the board is currently in
cur_status = "None"
print("Now checking the status of the board...")
msg_req["Type"] = "Get Status"
msg_req_str = json.dumps(msg_req)
channel.basic_publish(exchange='', routing_key='board_input', \
        body=msg_req_str)
while cur_status != "MANAGEMENT":
    if cur_status == "BOOTING":
        print("Board is BOOTING. Waiting for it to go into MANAGEMENT...")
    elif cur_status == "RUNNING":
        print("Board is RUNNING. Rebooting...")
        msg_req["Type"] = "Reboot"
        msg_req_str = json.dumps(msg_req)
        channel.basic_publish(exchange='', routing_key='board_input', \
                body=msg_req_str)
    elif cur_status == "MANAGEMENT":
        print("Board is in MANAGEMENT")
        msg_req["Type"] = "Stay in Management"
        msg_req_str = json.dumps(msg_req)
        channel.basic_publish(exchange='', routing_key='board_input', \
                body=msg_req_str)
    else:
        print("Board is in unknown mode! Rebooting...")
        msg_req["Type"] = "Reboot"
        msg_req_str = json.dumps(msg_req)
        channel.basic_publish(exchange='', routing_key='board_input', \
                body=msg_req_str)
    cur_status = "None"
    msg_req["Type"] = "Get Status"
    msg_req_str = json.dumps(msg_req)
    channel.basic_publish(exchange='', routing_key='board_input', \
            body=msg_req_str)

# Get the current configuration
settings_rx = False
msg_req = {}
msg_req["Type"] = "Config Req"
msg_req_str = json.dumps(msg_req)
channel.basic_publish(exchange='', routing_key='board_input', body=msg_req_str)

# Load up the settings
for setting_name, setting in settings:
    while !settings_rx: 
        pass
    settings_rx = False
    msg_req = {}
    msg_req["Type"] = "Put Setting"
    msg_req["Setting"] = setting_name
    msg_req[setting_name] = setting
    msg_req_str = json.dumps(msg_req)
    channel.basic_publish(exchange='', routing_key='board_input', \
            body=msg_req_str)

# Reboot the board
msg_req["Type"] = "Reboot"
msg_req_str = json.dumps(msg_req)
channel.basic_publish(exchange='', routing_key='board_input', \
        body=msg_req_str)

