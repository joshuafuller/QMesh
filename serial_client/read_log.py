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


# Write out to file
out_file = open("board_log.json", 'w')


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
        if parsed_line["Status"] == "MANAGEMENT":
            print("Consuming done!\r\n")
            ch.stop_consuming()
			

def log_process(ch, method, properties, body):
    line = body.decode('utf-8')
    parsed_line = {}
    parsed_line["Type"] = ""
    try: parsed_line = json.loads(line)
    except Exception as e: pass
    if parsed_line["Type"] == "Log Entry":
        if parsed_line["Count"] == -1:
            print("Finished reading in log entries")
            sys.exit(0)
        else:
            print(str(parsed_line))
            out_file.write(str(parsed_line) + "\r\n")
            out_file.flush()
            ch.stop_consuming()


# Set up the RabbitMQ connection
print("Setting up RabbitMQ connection...\r\n")
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()
channel.exchange_declare(exchange='board_output', exchange_type='fanout')
result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue
channel.queue_bind(exchange='board_output', queue=queue_name)
channel.basic_consume(queue=queue_name, auto_ack=True, \
        on_message_callback=dbg_process)

# Reboot the board
print("Rebooting board...\r\n")
msg_req = {}
msg_req["Type"] = "Reboot"
msg_req_str = json.dumps(msg_req)
msg_req_str += str("\n")
channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(msg_req_str.encode('ascii')))
channel.basic_consume(queue=queue_name, auto_ack=True, \
        on_message_callback=dbg_process)		
channel.start_consuming()

msg_req = {}
msg_req["Type"] = "Stay in Management"
msg_req_str = json.dumps(msg_req)
msg_req_str += str("\n")
channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(msg_req_str.encode('ascii')))
channel.basic_consume(queue=queue_name, auto_ack=True, \
        on_message_callback=dbg_process)			
channel.start_consuming()


channel.basic_consume(queue=queue_name, auto_ack=True, \
        on_message_callback=log_process)	
while True:
    msg_req = {}
    msg_req["Type"] = "Read Log"
    msg_req_str = json.dumps(msg_req)
    msg_req_str += str("\n")
    channel.basic_publish(exchange='', routing_key='board_input', body=msg_req_str)
    channel.basic_consume(queue=queue_name, auto_ack=True, \
        on_message_callback=log_process)	
    channel.start_consuming()
#    time.sleep(1)
