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
import time

# Set the RTC on the board to the current system time.
settings_rx = False
cur_status = "None"

def dbg_process(ch, method, properties, body):
    line = body.decode('utf-8')
    parsed_line = {}
    parsed_line["Type"] = ""
    try: parsed_line = json.loads(line)
    except Exception as e: pass
    print(parsed_line)
    if parsed_line["Type"] == "Status":
        cur_status = parsed_line["Status"]
        print("Current status is " + parsed_line["Status"])
        print("Current time is " + str(parsed_line["Time"]))
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

# Check what mode the board is currently in
cur_status = "None"
print("Now checking the status of the board...")
msg_req = {}
msg_req["Type"] = "Get Status"
msg_req_str = json.dumps(msg_req)
msg_req_str += str("\n")
channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(msg_req_str.encode('ascii')))
channel.start_consuming()

# Set the time
msg_req = {}
msg_req["Type"] = "Set Time"
msg_req["Time"] = str(int(time.time()))
msg_req_str = json.dumps(msg_req)
msg_req_str += str("\n")
channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(msg_req_str.encode('ascii')))
channel.start_consuming()

# Reboot the board
msg_req = {}
msg_req["Type"] = "Reboot"
msg_req_str = json.dumps(msg_req)
msg_req_str += str("\n")
channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(msg_req_str.encode('ascii')))
channel.start_consuming()

