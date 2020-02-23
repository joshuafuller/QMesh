#!/usr/bin/python3

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

import sys, os
import time
import json
import base64
import pika

if len(sys.argv) < 4:
    print("USAGE: set_config.py <node id> <beacon interval> <power level (dBm)>")
    sys.exit(0)

node_id = int(sys.argv[1])
beacon_interval = int(sys.argv[2])
tx_power = int(sys.argv[3])

# For now, let's just put the configuration in the same
# file as the one setting it.
settings = {}
settings["Address"] = node_id
settings["Freq"] = 433000000
settings["BW"] = 1 # 0=125KHz, 1=250KHz, 2=500KHz
settings["CR"] = 0
settings["SF"] = 9
settings["Preamble Length"] = 16
settings["Preamble Slots"] = 1
beacon_str = "KG5VBY Beacon Test"
settings["Beacon Message"] = str(base64.b64encode(beacon_str.encode('ascii')))
settings["Beacon Interval"] = beacon_interval
settings["Payload Length"] = len(beacon_str)
settings["FEC Algorithm"] = "Convolutional"
settings["Conv Rate"] = 2
settings["Conv Order"] = 7
settings["Reed-Solomon Number Roots"] = 8
settings["TX Power"] = tx_power
settings["CW Test Mode"] = 0 # 1 to enable CW test mode
settings["Preamble Test Mode"] = 0 # 1 to enable continuous-preamble test mode
settings["Test FEC"] = 0 # 1 to enable an FEC test

def dbg_process(ch, method, properties, body):
    line = body.decode('utf-8')
    parsed_line = {}
    parsed_line["Type"] = ""
    try: parsed_line = json.loads(line)
    except Exception as e: pass
    #print(parsed_line)
    if parsed_line["Type"] == "Status":
        cur_status = parsed_line["Status"]
        print("Current status is " + parsed_line["Status"])
        print("Current time is " + str(parsed_line["Time"]))
        if parsed_line["Status"] == "MANAGEMENT":
            print("Consuming done!\r\n")
            ch.stop_consuming()

# Set up the RabbitMQ connection
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()
channel.exchange_declare(exchange='board_output', exchange_type='fanout')
result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue
channel.queue_bind(exchange='board_output', queue=queue_name)
channel.basic_consume(queue=queue_name, auto_ack=True, \
        on_message_callback=dbg_process)

# Reboot the board
msg_req = {}
msg_req["Type"] = "Reboot"
msg_req_str = json.dumps(msg_req)
msg_req_str += str("\n")
channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(msg_req_str.encode('ascii')))
channel.start_consuming()

msg_req = {}
msg_req["Type"] = "Stay in Management"
msg_req_str = json.dumps(msg_req)
msg_req_str += str("\n")
channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(msg_req_str.encode('ascii')))
channel.start_consuming()

for (setting_name, setting_value) in settings.items():
	msg_req = {}
	msg_req["Type"] = "Put Setting"
	msg_req["Setting"] = str(setting_name)
	msg_req[str(setting_name)] = setting_value
	msg_req_str = json.dumps(msg_req)
	msg_req_str += str("\n")
	channel.basic_publish(exchange='', routing_key='board_input', \
			body=bytes(msg_req_str.encode('ascii')))
	print(msg_req_str)
	channel.basic_consume(queue=queue_name, auto_ack=True, \
        on_message_callback=dbg_process)
	channel.start_consuming()
	
# Reboot the board
msg_req = {}
msg_req["Type"] = "Reboot"
msg_req_str = json.dumps(msg_req)
msg_req_str += str("\n")
channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(msg_req_str.encode('ascii')))



