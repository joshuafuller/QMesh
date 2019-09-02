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
    try: 
        parsed_line = json.loads(line)
    except Exception as e:
        print("\033[1;31;40m" + str(line[:-2]) + "\033[1;37;40m")
    if parsed_line["Type"] == "Debug Msg":
        msg = base64.b64decode(parsed_line["Message"]).decode('utf-8')
        print("\033[1;32;40m" + str(msg[:-2]) + "\033[1;37;40m")
    elif parsed_line["Type"] == "Status":
        status = parsed_line["Status"]
        value = parsed_line["Value"]
        print("Status Msg: %s, %s" % (status, value))
    elif parsed_line["Type"] == "Settings":
        freq = parsed_line["Freq"]
        sf = parsed_line["SF"]
        bw = parsed_line["BW"]
        cr = parsed_line["CR"]
        mode = parsed_line["Mode"]
    elif parsed_line["Type"] == "Frame":
        frame["HDR Pkt Type"] = parsed_line["HDR Pkt Type"]
        frame["HDR Stream ID"] = parsed_line["HDR Stream ID"]
        frame["HDR TTL"] = parsed_line["HDR TTL"]
        frame["HDR Sender"] = parsed_line["HDR Sender"]
        frame["HDR Pre Offset"] = parsed_line["HDR Pre Offset"]
        frame["HDR Num Sym Offset"] = parsed_line["HDR Num Sym Offset"]
        frame["HDR Sym Offset"] = parsed_line["HDR Sym Offset"]
        frame["Header CRC"] = parsed_line["Header CRC"]
        frame["Data CRC"] = parsed_line["Data CRC"]
        frame["Data Payload"] = base64.base64decode(frame["Data Payload"])

# Set up the RabbitMQ connection
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()
channel.queue_declare(queue='board_output')
channel.basic_consume(queue='board_output', auto_ack=True, \
        on_message_callback=dbg_process)
channel.start_consuming()
while True: time.sleep(60)
