import sys, os
import pycodec2
import wave
import pika
import qmesh_common
import qmesh_pb2
import numpy as np
import time


CODEC2_BIT_SIZE = 700
MAX_FRAME_QUEUE_LEVEL = 16

frames_per_pkt = 4
frame_len_ms = 40
frame_len_s = frame_len_ms/1000
sample_rate = int(8e6)
num_channels = 1
c2 = pycodec2.Codec2(1200)
size_in_bits = {450    : 18,
                700    : 28,
                1200   : 48,
                1300   : 52,
                1600   : 64,
                2400   : 96,
                3200   : 128}


frame_queue_depth = 0
def dbg_process(ch, method, properties, body):
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if ser_msg.type == ser_msg.STATUS:
        qmesh_common.print_status_msg(ser_msg.status)
        if(ser_msg.status.status == ser_msg.status.MANAGEMENT):
            qmesh_common.channel.stop_consuming()
    elif ser_msg.type == ser_msg.ACK_MSG:
        frame_queue_depth = ser_msg.ack_msg.radio_out_queue_level

qmesh_common.setup_outgoing_rabbitmq(dbg_process)


# Open up the WAV file
samples = []
wav_file_name = sys.argv[3]
wav_file = wave.open(wav_file_name, 'rb')
assert(wav_file.getnchannels() == 1)
assert(wav_file.getsampwidth() == 2)
assert(wav_file.getframerate() == sample_rate)
# Encode the WAV file into codec2 frames and send them out
while True:
    frames = wav_file.getframes(sample_rate*frame_len_s)
    assert(len(frames) > 320)
    if len(frames) != sample_rate*frame_len_s:
        print("Done with WAV file!")
        break
    samples = np.array(frames, dtype=np.int16)
    c2_frame = c2.encode(samples)
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.Type = ser_msg.VOICE_MSG
    ser_msg.voice_frame_msg.size_bits = size_in_bits[700]
    ser_msg.voice_frame_msg.end_stream = False
    payload = c2_frame
    qmesh_common.publish_msg(ser_msg)
    while frame_queue_depth >= MAX_FRAME_QUEUE_LEVEL:
        status_msg_req = qmesh_pb2.SerialMsg()
        status_msg_req.Type = status_msg_req.GET_STATUS
        qmesh_common.publish_msg(status_msg_req)
        time.sleep(float(frame_len_ms/1000.0))

        