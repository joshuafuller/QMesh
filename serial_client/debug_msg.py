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

def dbg_process(ch, method, properties, body):
    try:
        line = body.decode('ascii')
    except Exception as e: 
        return
    parsed_line = {}
    parsed_line["Type"] = ""
    try: 
        parsed_line = json.loads(line)
    except Exception as e:
        print("\033[1;31;40m" + str(line[:-2]) + "\033[1;37;40m")
    if parsed_line["Type"] == "Debug Msg":
        try: 
            msg = base64.b64decode(parsed_line["Message"]).decode('utf-8')
            print("\033[1;32;40m" + str(msg[:-2]) + "\033[1;37;40m")
        except Exception as e: pass
    elif parsed_line["Type"] == "Status":
        status = parsed_line["Status"]
        print("Status Msg: %s" % (status))
    elif parsed_line["Type"] == "Settings":
        freq = parsed_line["Freq"]
        sf = parsed_line["SF"]
        bw = parsed_line["BW"]
        cr = parsed_line["CR"]
        mode = parsed_line["Mode"]
    elif parsed_line["Type"] == "Frame":
        frame = {}
        frame["HDR Pkt Type"] = parsed_line["HDR Pkt Type"]
        frame["HDR Stream ID"] = parsed_line["HDR Stream ID"]
        frame["HDR TTL"] = parsed_line["HDR TTL"]
        frame["HDR Sender"] = parsed_line["HDR Sender"]
        frame["HDR Sym Offset"] = parsed_line["HDR Sym Offset"]
        frame["CRC"] = parsed_line["CRC"]
        print("-------------------")
        print("Received Frame")
        print(frame)
        print("-------------------")


# Set up the RabbitMQ connection
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()
channel.exchange_declare(exchange='board_output', exchange_type='fanout')
result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue
channel.queue_bind(exchange='board_output', queue=queue_name)
channel.basic_consume(queue=queue_name, auto_ack=True, \
        on_message_callback=dbg_process)
channel.start_consuming()
while True: time.sleep(60)
