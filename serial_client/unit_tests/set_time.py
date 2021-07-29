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

import sys, os
import time
import json
import base64
import pika
sys.path.insert(0, os.getcwd())
import qmesh_common
import qmesh_pb2
import random

exit_code = 0
stage = 'set time'
random.seed(time.time())
time_val = random.randint(1, 1000000000)

def dbg_process(ch, method, properties, body):
    global exit_code
    global stage
    global time_val
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if(ser_msg.type == ser_msg.STATUS):
        qmesh_common.print_status_msg(ser_msg.status)
        if(ser_msg.status.status == ser_msg.status.MANAGEMENT):
            qmesh_common.channel.stop_consuming()
            if stage == 'get time':
                if abs(ser_msg.status.time-time_val) > 3:
                    exit_code = 1
    #elif ser_msg.type == ser_msg.ACK:
    #    qmesh_common.channel.stop_consuming()
    #elif ser_msg.type == ser_msg.ACKERR:
    #    qmesh_common.channel.stop_consuming()
    #    exit_code = 1


qmesh_common.setup_outgoing_rabbitmq(dbg_process)

qmesh_common.reboot_board()
qmesh_common.channel.start_consuming()
qmesh_common.stay_in_management()

# Set the time
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.CLOCK_SET
ser_msg.time_msg.time = time_val
qmesh_common.publish_msg(ser_msg)
qmesh_common.channel.start_consuming()

# Get the time
stage = 'get time'
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.GET_STATUS
qmesh_common.publish_msg(ser_msg)
qmesh_common.channel.start_consuming()

# Reboot the board
qmesh_common.reboot_board()
time.sleep(20)

sys.exit(exit_code)






