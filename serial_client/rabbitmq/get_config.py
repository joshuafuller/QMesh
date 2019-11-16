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
		print("Convolution Rate: " + str(parsed_line["Conv Rate"]))
		print("Convolution Order: " + str(parsed_line["Conv Order"]))
		print("Reed-Solomon Number of Roots: " + str(parsed_line["RS Num Roots"]))
		
		# Save the settings to a file
		settings_file = open("board_settings.json", "w")
		settings_file.write(parsed_line)
		settings_file.close()
		
        sys.exit(0)

# Set up the RabbitMQ connection
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()
channel.queue_declare(queue='board_input')
channel.queue_declare(queue='board_output')
channel.basic_consume(queue='board_output', auto_ack=True, \
        on_message_callback=dbg_process)
channel.start_consuming()

msg_req = {}
msg_req["Type"] = "Config Req"
msg_req_str = json.dumps(msg_req)
channel.basic_publish(exchange='', routing_key='board_input', body=msg_req_str)

while True: pass
