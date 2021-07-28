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


def dbg_process(ch, method, properties, body):
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if(ser_msg.type == ser_msg.STATUS):
        qmesh_common.print_status_msg(ser_msg.status)
        if(ser_msg.status.status == ser_msg.status.RUNNING):
            qmesh_common.channel.stop_consuming()
            if(ser_msg.status.oled_on == False):
                sys.exit(0)
            else:
                sys.exit(1)

qmesh_common.setup_outgoing_rabbitmq(dbg_process)

# Turn off the display
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.TURN_OLED_OFF
qmesh_common.publish_msg(ser_msg)
# Wait a bit
time.sleep(2)
# Check the status to see whether it actually turned off
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.GET_STATUS
qmesh_common.publish_msg(ser_msg)




