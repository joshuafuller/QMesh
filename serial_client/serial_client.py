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


import serial
import pika
import sys
import time
import threading
import qmesh_pb2
import binascii

ser = None
params = pika.ConnectionParameters('127.0.0.1', heartbeat=600, 
        blocked_connection_timeout=300)
        

def make_slip_frame(frame):
    out_frame = bytearray()
    for cur_byte in frame:
        if(cur_byte == FRAME_END or cur_byte == FRAME_ESC):
            out_frame.append(FRAME_ESC)
        out_frame.append(cur_byte)
    out_frame.append(FRAME_END)
    return bytearray(out_frame)


# Callback whenever new received messages come in from the broker
def input_cb(ch, method, properties, body):
    global ser
    frame = bytearray()
    frame += body
    crc = binascii.crc_hqx(frame)
    crc_bytes = crc.to_bytes(2, byteorder="little")
    frame += crc_bytes
    ser.write(bytearray(frame))


def output_thread_fn():
    # Set up the RabbitMQ connections
    connection = pika.BlockingConnection(params)
    channel = connection.channel()
    channel.queue_declare(queue='board_input')
    channel.basic_consume(queue='board_input', auto_ack=True, \
                on_message_callback=input_cb)
    channel.start_consuming()


class CRCError(Exception):
    pass


class ParseError(Exception):
    pass


FRAME_END = 0xC0
FRAME_ESC = 0xDB
FRAME_MAX_SIZE = 8192
def get_slip_frame(ser):
    # Initial pass: get to the next frame's delimiter
    while(True):
        cur_byte = ser.read(1)
        next_byte = ser.read(1)
        if(cur_byte == FRAME_ESC and next_byte == FRAME_END): continue
        elif(next_byte == FRAME_END): break
    # Extract bytes until we get to the next frame delimiter
    frame = bytearray()
    while(True):
        if(len(frame) > FRAME_MAX_SIZE): # If we get too big, we need to retry
            frame = bytearray()
        cur_byte = ser.read(1)
        if(cur_byte == FRAME_END): 
            yield bytearray(frame)
            frame = bytearray()
        elif(cur_byte == FRAME_ESC):
            frame.append(ser.read(1)) 
        else:
            frame.append(cur_byte)
        

def input_thread_fn(ser):
    connection = pika.BlockingConnection(params)
    channel = connection.channel()
    channel.exchange_declare(exchange='board_output', exchange_type='fanout')
    global ser
    while(True):
        frame_gen = get_slip_frame(ser)
        for frame in frame_gen:
            try:
                # First, get the SLIP frame
                crc = int.from_bytes(frame[-2:-1], "little", signed=False)
                calc_crc = binascii.crc_hqx(frame[:-2])
                pld = frame[:-2]
                if(calc_crc != crc): 
                    raise CRCError
                ser_msg = qmesh_pb2.SerialMsg()
                ser_msg.ParseFromString(pld)
                channel.basic_publish(exchange='board_output', routing_key='', 
                                        body=ser_msg.SerializeToString())
            except CRCError:
                print("CRC Error detected")


if __name__ == "__main__":
    # Open the serial port
    print("Opening serial port...")
    serial_ports = sys.argv[1:]
    while True:
        try:
            print("Trying serial port " + str(serial_ports[0]))
            ser = serial.Serial(serial_ports[0], baudrate=230400)
            break
        except serial.serialutil.SerialException:
            print("Failed to open port. Trying again in 1s...")
            if len(serial_ports) > 1:
                serial_ports = serial_ports[1:] + [serial_ports[0]]
            time.sleep(1)

    input_thread = threading.Thread(target=input_thread_fn)
    input_thread.start()
    output_thread = threading.Thread(target=output_thread_fn)
    output_thread.start()

    while True: 
        try: time.sleep(1)
        except (KeyboardInterrupt, SystemExit):
            input_thread.join()
            output_thread.join()
            sys.exit(0)



