#!/usr/bin/python3

import sys, os

file_name = sys.argv[1]
rx_file = open(file_name, 'r')

stream_ids = []
for line in rx_file.readlines():
    split_line = line.split(',')
    subsplit_line = split_line[2]
    stream_ids.append(int(subsplit_line.split(':')[1]))
   
idx = stream_ids[0]
good_pkt = 0
bad_pkt = 0
for stream_id in stream_ids:
    while True:
        if stream_id == idx: 
            good_pkt += 1
            idx = (idx + 1) % 256
            break
        # cheap heuristic to skip periodic beacons from other nodes
        elif abs(stream_id-idx) > 3:
            break
        else:
            bad_pkt += 1
            idx = (idx + 1) % 256

print("Good packets: %d" % good_pkt)
print("Bad packets: %d" % bad_pkt)
print("Packet Receive Rate: %f" % (float(good_pkt)/(float(good_pkt)+float(bad_pkt))))
