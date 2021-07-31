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
import pika
import yaml
sys.path.insert(0, os.getcwd())
import qmesh_pb2
import qmesh_common

stage = 'rebooting'
exit_value = 0

def dbg_process(ch, method, properties, body):
    global stage
    global exit_value
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if(ser_msg.type == ser_msg.STATUS):
        qmesh_common.print_status_msg(ser_msg.status)
        if(ser_msg.status.status == ser_msg.status.MANAGEMENT):
            qmesh_common.channel.stop_consuming()
        else:
            if stage == 'management':
                exit_value = 1
                qmesh_common.channel.stop_consuming()


# Set up the RabbitMQ connection
qmesh_common.setup_outgoing_rabbitmq(dbg_process)

# Reboot the board
qmesh_common.reboot_board()
qmesh_common.channel.start_consuming()

# Stay in management
qmesh_common.stay_in_management()

# Check the status
stage = 'management'
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.GET_STATUS
qmesh_common.publish_msg(ser_msg)
qmesh_common.channel.start_consuming()

# Reboot the board again
qmesh_common.reboot_board()
sys.exit(exit_value)


