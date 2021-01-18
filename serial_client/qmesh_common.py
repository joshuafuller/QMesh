import pika
import time
import binascii
import qmesh_pb2


channel = None
result = None
queue_name = None


def publish_msg(ser_msg):
    global channel
    channel.basic_publish(exchange='', routing_key='board_input', \
        body=bytes(ser_msg.SerializeToString()))


def print_data_msg(data_msg):
    print("--------------------")
    print("Type: %d, Stream ID: %d, TTL: %d, Sender: %d, Sym Offset: %d, CRC: %d" % 
        (data_msg.type, data_msg.stream_id, data_msg.ttl, data_msg.sender, data_msg.sym_offset, 
            data_msg.crc))
    print(binascii.hexlify(bytearray(data_msg.pld)))
    print("--------------------")


def print_dbg_msg(dbg_msg):
    print("%s" % dbg_msg.msg)


def print_status_msg(status_msg):
    if(status_msg.status == qmesh_pb2.StatusMsg.BOOTING):
        status_str = 'BOOTING'
    elif(status_msg.status == qmesh_pb2.StatusMsg.MANAGEMENT):
        status_str = 'MANAGEMENT'
    elif(status_msg.status == qmesh_pb2.StatusMsg.RUNNING):
        status_str = 'RUNNING'
    else:
        raise Exception
    print("----------")
    print("Status is %s" % (status_str))
    print("Time is %s" % (status_msg.time))
    print("Queue state is %s" % (status_msg.tx_full))
    print("----------")


def setup_outgoing_rabbitmq(my_callback):
    global channel, result, queue_name
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()
    channel.exchange_declare(exchange='board_output', exchange_type='fanout')
    result = channel.queue_declare(queue='', exclusive=True)
    queue_name = result.method.queue
    channel.queue_bind(exchange='board_output', queue=queue_name)
    channel.basic_consume(queue=queue_name, auto_ack=True, on_message_callback=my_callback)


def reboot_board():
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.type = ser_msg.REBOOT
    publish_msg(ser_msg)


def stay_in_management():
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.type = ser_msg.STAY_IN_MGT
    publish_msg(ser_msg)
    channel.start_consuming()


def print_log_msg(log_msg):
    ret_str = ''
    ret_str += "----------\r\n"
    ret_str += "Timestamp: %s\r\n" % (time.asctime(log_msg.timestamp))
    ret_str += "Count: %s\r\n" % (log_msg.count)
    ret_str += "Sender: %s\r\n" % (log_msg.sender)
    ret_str += "TTL: %s\r\n" % (log_msg.ttl)
    ret_str += "Stream ID: %s\r\n" % (log_msg.stream_id)
    ret_str += "RSSI: %s\r\n" % (log_msg.rssi)
    ret_str += "SNR: %s\r\n" % (log_msg.snr)
    ret_str += "RX Size: %s\r\n" % (log_msg.rx_size)
    ret_str += "Computed CRC: %s\r\n" % (log_msg.comp_crc)
    ret_str += "Received CRC: %s\r\n" % (log_msg.crc)
    ret_str += "GPS Valid: %s\r\n" % (log_msg.gps_msg.valid)
    ret_str += "GPS Latitude: %s\r\n" % (log_msg.gps_msg.lat)
    ret_str += "GPS Longitude: %s\r\n" % (log_msg.gps_msg.lon)
    print(ret_str)
    return ret_str


def print_bootlog_msg(bootlog_msg):
    ret_str = ''
    ret_str += "----------\r\n"
    ret_str += "Timestamp: %s\r\n" % (time.asctime(bootlog_msg.timestamp))
    ret_str += "Count: %s\r\n" % (bootlog_msg.count)
    print(ret_str)
    return ret_str