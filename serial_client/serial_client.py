#!/usr/bin/python3

# QMesh
# Copyright (C) 2021 Daniel R. Fay

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
import crcmod

ser = None
params = pika.ConnectionParameters('127.0.0.1', heartbeat=600, 
        blocked_connection_timeout=300)
        

FEND = 0xC0.to_bytes(1, 'little')
FESC = 0xDB.to_bytes(1, 'little')
TFEND = 0xDC.to_bytes(1, 'little')
TFESC = 0xDD.to_bytes(1, 'little')
SETHW = 0x06.to_bytes(1, 'little')
DATAPKT = 0x00.to_bytes(1, 'little')
FRAME_MAX_SIZE = 8192
def make_kiss_frame(frame):
    out_frame = bytearray()
    out_frame += FEND
    out_frame += SETHW
    for cur_byte in frame:
        cur_byte_arr = bytearray()
        cur_byte_arr.append(cur_byte)
        if(cur_byte_arr == FEND):
            out_frame += FESC
            out_frame += TFEND
        elif(cur_byte_arr == FESC):
            out_frame += FESC
            out_frame += TFESC
        else:
            out_frame += cur_byte_arr
    out_frame += FEND
    return bytearray(out_frame)


# Callback whenever new received messages come in from the broker
def input_cb(ch, method, properties, body):
    print("Hello, a message has been received")
    global ser
    frame = bytearray()
    frame += FEND
    frame += SETHW
    print(body)
    for frame_byte in body:
        if(frame_byte == FEND):
            frame += FESC
            frame += TFEND
        elif(frame_byte == FESC):
            frame += FESC
            frame += TFESC
        else:
            frame.append(frame_byte)

    #crc = binascii.crc_hqx(body, 0) # This uses the CRC-CCITT polynomial

    _CRC_FUNC = crcmod.mkCrcFun(0x11021, initCrc=0xffff, rev=False)
    crc = _CRC_FUNC(body)
    crc_bytes = crc.to_bytes(2, byteorder="little")

    frame += crc_bytes
    frame += FEND
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


def get_kiss_frame(ser):
    # Initial pass: get to the next frame's delimiter
    print("in kiss frame")
    while(True):
        #print("Reading byte")
        my_byte = ser.read(1)
        if(my_byte == FEND): 
            print("End delimiter found")
            break
    # Extract bytes until we get to the next frame delimiter
    frame = bytearray()
    while(True):
        if(len(frame) > FRAME_MAX_SIZE): # If we get too big, we need to retry
            frame = bytearray()
        cur_byte = ser.read(1)
        #print("Read byte %s" % cur_byte)
        if(cur_byte == FEND): 
            # Try to deal with FEND bytes used for synchronization
            if(len(frame) > 0):
                frame = frame[1:]
                yield bytearray(frame)
            frame = bytearray()
            # Need to receive the command code
            cmd_byte = ser.read(1)
            if(cmd_byte == FEND): continue
        elif(cur_byte == FESC):
            next_byte = ser.read(1)
            if(next_byte == TFEND):
                frame += FEND 
            if(next_byte == TFESC):
                frame += FESC
        else:
            frame += cur_byte
        

def input_thread_fn():
    connection = pika.BlockingConnection(params)
    channel = connection.channel()
    channel.exchange_declare(exchange='board_output', exchange_type='fanout')
    global ser
    while(True):
        frame_gen = get_kiss_frame(ser)
        for frame in frame_gen:
            try:
                # First, get the KISS frame
                print("Full frame content is %s" % frame)
                crc = int.from_bytes(frame[-2:], "little", signed=False)
                frame = frame[:-2]
                _CRC_FUNC = crcmod.mkCrcFun(0x11021, initCrc=0xffff, rev=False)
                calc_crc = _CRC_FUNC(frame)
                pld = frame
                if(calc_crc != crc): 
                    raise CRCError
                ser_msg = qmesh_pb2.SerialMsg()
                try: ser_msg.ParseFromString(pld)
                except UnicodeDecodeError: print("Failed to decode")
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
    # Put the TNC in KISS+ mode
    #kiss_frame = make_kiss_frame(0xFF.to_bytes(1, 'little'))
    #ser.write(kiss_frame)
    # Set up the callbacks
    input_thread = threading.Thread(target=input_thread_fn)
    input_thread.start()
    output_thread = threading.Thread(target=output_thread_fn)
    output_thread.start()

    while True: 
        try: time.sleep(1)
        except (KeyboardInterrupt, SystemExit):
            input_thread.join()
            output_thread.join()
            # Put the node back in KISS mode
            ser_msg = qmesh_pb2.SerialMsg()
            ser_msg.type = ser_msg.ENTER_KISS_MODE
            frame = bytearray()
            frame += FEND
            frame += bytes(ser_msg.SerializeToString())
            crc = binascii.crc_hqx(frame, 0)
            crc_bytes = crc.to_bytes(2, byteorder="little")
            frame += crc_bytes
            frame += FEND
            ser.write(bytearray(frame))
            sys.exit(0)



