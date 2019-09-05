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
    if parsed_line["Type"] == "Config Msg":
        print("----------CONFIGURATION----------")
        print("Frequency: " + str(parsed_line["freq"]))
        print("Coding Rate: " + str(parsed_line["cr"]))
        print("Spreading Factor: " + str(parsed_line["sf"]))
        print("Bandwidth: " + str(parsed_line["bw"]))
        print("Mode: " + str(parsed_line["mode"]))
        print("Preamble Length: " + str(parsed_line["pre_len"]))
        print("Num Preamble Slots: " + str(parsed_line["pre_slots"]))
        print("Message Length: " + str(parsed_line["msg_len"]))
        print("Beacon Period (s): " + str(parsed_line["beacon_period"]))
        msg = base64.b64decode(parsed_line["beacon_msg"]).decode('utf-8')
        print("Beacon Message: " + str(msg))
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
