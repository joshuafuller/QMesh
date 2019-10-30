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
import json
import base64
import pika

# Set up the RabbitMQ connection
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()
channel.queue_declare(queue='board_input')

# Send the value
send_data = {}
send_data["Type"] = "Frame"
send_data["Data Payload"] = str(base64.b64encode(sys.argv[1].encode('utf-8')))
channel.basic_publish(exchange='', routing_key='board_input', 
        body=json.dumps(send_data))

connection.close()
