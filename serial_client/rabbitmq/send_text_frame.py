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
