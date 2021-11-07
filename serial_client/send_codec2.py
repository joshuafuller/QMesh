import sys, os
import pycodec2
import wave
import pika
import qmesh_common
import qmesh_pb2
import numpy as np


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


def dbg_process(ch, method, properties, body):
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if(ser_msg.type == ser_msg.STATUS):
        qmesh_common.print_status_msg(ser_msg.status)
        if(ser_msg.status.status == ser_msg.status.MANAGEMENT):
            qmesh_common.channel.stop_consuming()

qmesh_common.setup_outgoing_rabbitmq(dbg_process)

# Turn off the display
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.TURN_OLED_ON
qmesh_common.publish_msg(ser_msg)


# Open up the WAV file
samples = []
wav_file_name = sys.argv[3]
wav_file = wave.open(wav_file_name, 'rb')
assert(wav_file.getnchannels() == 1)
assert(wav_file.getsampwidth() == 2)
assert(wav_file.getframerate() == sample_rate)
c2_frames = []
while True:
    frames = wav_file.getframes(sample_rate*frame_len_s)
    assert(len(frames) > 320)
    if len(frames) != sample_rate*frame_len_s:
        print("Done with WAV file!")
        break
    samples = np.array(frames, dtype=np.int16)
    c2_frames.append(c2.encode(samples))
    if len(c2_frames) == frames_per_pkt: