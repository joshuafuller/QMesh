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

test_passed = False

def callback(ch, method, properties, body):
    global test_passed
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if(ser_msg.type == ser_msg.STATUS):
        qmesh_common.print_status_msg(ser_msg.status)
        if(ser_msg.status.status == ser_msg.status.MANAGEMENT):
            qmesh_common.channel.stop_consuming()
            test_passed = True

test_passed = False
qmesh_common.setup_outgoing_rabbitmq(callback)
qmesh_common.reboot_board()
qmesh_common.channel.start_consuming()
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.GET_STATUS
qmesh_common.publish_msg(ser_msg)
qmesh_common.channel.start_consuming()

if(test_passed):
    sys.exit(0)
else:
    sys.exit(1)