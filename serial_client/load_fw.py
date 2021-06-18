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
import qmesh_pb2
import qmesh_common
import hashlib
import fastlz


CHUNK_SIZE = 4096

upd_hashgen = hashlib.sha256()
def get_file_data(fname):
    fdata = open(fname, "rb")
    while True:
        update_msg = qmesh_pb2.UpdateMsg()
        content = fdata.read(CHUNK_SIZE)
        hashgen = hashlib.sha256()
        hashgen.update(content)
        upd_hashgen.update(content) 
        update_msg = qmesh_pb2.UpdateMsg()
        if len(sys.argv) == 4 and sys.argv[3] == "golden":
            update_msg.path = '/fs/golden.bin.gz'
        else:
            update_msg.path = '/fs/update.bin.gz'
        update_msg.pld = content
        update_msg.sha256_pkt = hashgen.digest()
        yield update_msg


print("Loading firmware file %s..." % (sys.argv[2]))
ser_msg = qmesh_pb2.SerialMsg()
file_data = get_file_data(sys.argv[2]) # Open file  
update_msgs = []
while True:
    update_msg = next(file_data)
    if len(update_msg.pld) == 0: break
    update_msg.type = update_msg.Type.MIDDLE
    update_msgs.append(update_msg)
    update_msg.pkt_cnt = update_msgs.index(update_msg)
update_msgs[0].type = update_msg.Type.FIRST
update_msgs[-1].type = update_msg.Type.LAST
update_msgs[-1].sha256_upd = upd_hashgen.digest()


tx_byte_count = 0
def dbg_process(ch, method, properties, body):
    global tx_byte_count
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if(ser_msg.type == ser_msg.UPDATE):
        # If we get a proper ACK, send out the next packet; otherwise,
        #  just resend the last packet
        if(ser_msg.update_msg.type == ser_msg.update_msg.Type.ACK): 
            print("Sent %d bytes" % (tx_byte_count))
            if(update_msgs[ser_msg.update_msg.pkt_cnt].type == ser_msg.update_msg.Type.LAST): 
                print("Finished sending update")
                qmesh_common.channel.stop_consuming()
                time.sleep(1)
                qmesh_common.reboot_board()
                sys.exit(0)
        elif(ser_msg.update_msg.type == ser_msg.update_msg.ACKERR):
            err_reason = ser_msg.update_msg.err_reason
            print("ERROR: %s" % (err_reason))
            if err_reason == "No Update File":
                print("Update file issue, exiting...")
                sys.exit(0)
            elif err_reason == "No SHA256 checksum":
                print("SHA256 checksum missing, exiting...")
                sys.exit(0)
            elif err_reason == "SHA256 packet checksum failed":
                print("SHA256 checksum failed, retrying packet...")
            elif err_reason == "SHA256 checksum failed":
                print("SHA256 file checksum failed, exiting...")
                sys.exit(0)
            elif err_reason == "SHA256 open failed":
                print("SHA256 file open failed, exiting...")
                sys.exit(0)
            else:
                print("Other error, exiting...")
                sys.exit(0)
            print("Retransmitting packet %s" % (ser_msg.update_msg.pkt_cnt+1))
        update_msg = update_msgs[ser_msg.update_msg.pkt_cnt+1]
        ser_msg = qmesh_pb2.SerialMsg()
        ser_msg.type = ser_msg.UPDATE
        ser_msg.update_msg.ParseFromString(update_msg.SerializeToString())
        tx_byte_count += len(update_msg.pld)

        qmesh_common.publish_msg(ser_msg)


# Load blocks and push them out over the interface. 
qmesh_common.setup_outgoing_rabbitmq(dbg_process)
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.UPDATE
ser_msg.update_msg.ParseFromString(update_msgs[0].SerializeToString())
ser_msg.update_msg.pkt_cnt = 0
qmesh_common.publish_msg(ser_msg)
qmesh_common.channel.start_consuming()
