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

import serial
import pika
import sys
import time
import threading

ser = None

# Callback whenever new received messages come in from the broker
def input_cb(ch, method, properties, body):
    global ser
    print(body)
    ser.writelines(str(body) + str("\r\n\0"))

def output_thread_fn():
    # Set up the RabbitMQ connections
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()
    channel.queue_declare(queue='board_input')
    channel.basic_consume(queue='board_input', auto_ack=True, \
                on_message_callback=input_cb)
    channel.start_consuming()


def input_thread_fn():
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()
    channel.queue_declare(queue='board_output')
    global ser
    while True:
        try:
            line = ser.readline();
        except serial.serialutil.SerialException:
            try:
                ser = serial.Serial(serial_port, baudrate=9600)
            except serial.serialutil.SerialException:
                print("Failed to open port. Trying again in 1s...")
                time.sleep(1)
            continue

        print(line)
        channel.basic_publish(exchange='', routing_key='board_output', body=line)


if __name__ == "__main__":
    # Open the serial port
    print("Opening serial port...")
    serial_port = sys.argv[1]
    while True:
        try:
            ser = serial.Serial(serial_port, baudrate=9600)
            break
        except serial.serialutil.SerialException:
            print("Failed to open port. Trying again in 1s...")
            time.sleep(1)

    threading.Thread(target=input_thread_fn).start()
    threading.Thread(target=output_thread_fn).start()

    while True: 
        time.sleep(60)



