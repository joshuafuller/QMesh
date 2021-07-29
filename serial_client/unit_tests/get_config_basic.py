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

#!/usr/bin/python3

import sys, os
import time
import json
import base64
import pika
sys.path.insert(0, os.getcwd())
import qmesh_pb2
import qmesh_common


def dbg_process(ch, method, properties, body):
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if(ser_msg.type == ser_msg.CONFIG):
        print("Got a config message")
        qmesh_common.print_cfg_msg(ser_msg)
        qmesh_common.channel.stop_consuming()
        time.sleep(20)
        sys.exit(0)

qmesh_common.setup_outgoing_rabbitmq(dbg_process)
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.GET_CONFIG
qmesh_common.publish_msg(ser_msg)
qmesh_common.channel.start_consuming()
