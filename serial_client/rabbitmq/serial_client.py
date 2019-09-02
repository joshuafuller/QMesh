#!/usr/bin/python3

import serial
import pika
import sys

# Callback whenever new received messages come in from the broker
def input_cb(ch, method, properties, body):
    ser.writelines(body)

# Open the serial port
serial_port = sys.argv[1]
ser = serial.Serial(serial_port, baudrate=921600)

# Set up the RabbitMQ connections
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()
channel.queue_declare(queue='board_output')
channel.queue_declare(queue='board_input')
channel.basic_consume(queue='board_input', auto_ack=True, \
            on_message_callback=input_cb)

while True:
    line = ser.readline();
    print(line)
    channel.basic_publish(exchange='', routing_key='board_output', body=line)
