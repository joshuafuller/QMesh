import pika
import time
import binascii
import qmesh_pb2
import yaml


channel = None
result = None
queue_name = None


def cfg_to_yaml_file(ser_msg, yaml_file_path):
    cfg_dict = {}
    cfg_dict['frequency'] = ser_msg.sys_cfg.radio_cfg.frequency
    cfg_dict['frequencies'] = []
    for freq in ser_msg.sys_cfg.radio_cfg.frequencies:
        cfg_dict['frequencies'].append(freq) 
    cfg_dict['bw'] = ser_msg.sys_cfg.radio_cfg.lora_cfg.bw
    cfg_dict['sf'] = ser_msg.sys_cfg.radio_cfg.lora_cfg.sf 
    cfg_dict['cr'] = ser_msg.sys_cfg.radio_cfg.lora_cfg.cr 
    cfg_dict['preamble_length'] = ser_msg.sys_cfg.radio_cfg.lora_cfg.preamble_length 
    cfg_dict['cw_test_mode'] = ser_msg.sys_cfg.test_cfg.cw_test_mode
    cfg_dict['preamble_test_mode'] = ser_msg.sys_cfg.test_cfg.preamble_test_mode 
    cfg_dict['fec_test_mode'] = ser_msg.sys_cfg.test_cfg.test_fec
    cfg_dict['fec_algo'] = ser_msg.sys_cfg.fec_cfg.type
    cfg_dict['conv_rate'] = ser_msg.sys_cfg.fec_cfg.conv_rate
    cfg_dict['conv_order'] = ser_msg.sys_cfg.fec_cfg.conv_order
    cfg_dict['rs_num_roots'] = ser_msg.sys_cfg.fec_cfg.rs_num_roots
    cfg_dict['beacon_msg'] = ser_msg.sys_cfg.net_cfg.beacon_msg
    cfg_dict['num_offsets'] = ser_msg.sys_cfg.net_cfg.num_offsets
    cfg_dict['payload_length'] = ser_msg.sys_cfg.net_cfg.pld_len
    cfg_dict['pocsag_enabled'] = ser_msg.sys_cfg.pocsag_cfg.enabled
    cfg_dict['pocsag_freq'] = ser_msg.sys_cfg.pocsag_cfg.frequency
    cfg_dict['pocsag_beacon_interval'] = ser_msg.sys_cfg.pocsag_cfg.beacon_interval
    cfg_dict['gps_en'] = ser_msg.sys_cfg.gps_en
    cfg_dict['log_packets'] = ser_msg.sys_cfg.log_packets_en
    cfg_dict['log_boot'] = ser_msg.sys_cfg.boot_log_en
    cfg_dict['watchdog_timer_en'] = ser_msg.sys_cfg.watchdog_timer_en

    yaml_str = yaml.dump(cfg_dict)
    f = open(yaml_file_path, 'w')
    f.write(yaml_str)
    f.close()


def print_cfg_msg(ser_msg):
    sys_cfg = ser_msg.sys_cfg
    print("CONFIGURATION:")
    print("\tMode: %s" % (sys_cfg.mode))
    print("\tAddress: %s" % (sys_cfg.address))
    print("\tRADIO CONFIGURATION:")
    radio_cfg = sys_cfg.radio_cfg
    print("\t\tType: %s" % (radio_cfg.type))
    print("\t\tFrequency: %s" % (radio_cfg.frequency))
    print("\t\tHOPPING FREQUENCIES:")
    for freq in radio_cfg.frequencies:
        print("\t\t\t %s" % (freq))
    print("\t\tTransmit Power: %s" % (radio_cfg.tx_power))
    print("\t\tLORA CONFIG:")
    lora_cfg = radio_cfg.lora_cfg
    print("\t\t\tBandwidth: %s" % (lora_cfg.bw))
    print("\t\t\tCoding Rate: %s" % (lora_cfg.cr))
    print("\t\t\tSpreading Factor: %s" % (lora_cfg.sf))
    print("\t\t\tPreamble Length: %s" % (lora_cfg.preamble_length))
    print("\t\t\tFHSS Preamble Length: %s" % (lora_cfg.fhss_pre_len))
    print("\tTEST CONFIG:")
    test_cfg = sys_cfg.test_cfg
    print("\t\tCW Test Mode: %s" % (test_cfg.cw_test_mode))
    print("\t\tPreamble Test Mode: %s" % (test_cfg.preamble_test_mode))
    print("\t\tTest FEC Mode: %s" % (test_cfg.test_fec))   
    print("\tFEC CONFIG:")
    fec_cfg = sys_cfg.fec_cfg
    print("\t\tType: %s" % (fec_cfg.type))
    print("\t\tConvolutional Rate: %s" % (fec_cfg.conv_rate))
    print("\t\tConvolutional Order: %s" % (fec_cfg.conv_order))
    print("\t\tReed-Solomon Number of Roots: %s" % (fec_cfg.rs_num_roots))    
    print("\tNETWORK CONFIG:")
    net_cfg = sys_cfg.net_cfg
    print("\t\tBeacon Message: %s" % (net_cfg.beacon_msg))
    print("\t\tBeacon Interval: %s" % (net_cfg.beacon_interval))
    print("\t\tNumber of Offsets: %s" % (net_cfg.num_offsets))
    print("\t\tPayload Length: %s" % (net_cfg.pld_len))
    print("\t\tFull Packet Length: %s" % (net_cfg.full_pkt_len))
    print("\tPOCSAG CONFIG:")
    pocsag_cfg = sys_cfg.pocsag_cfg
    print("\t\tEnabled: %s" % (pocsag_cfg.enabled))
    print("\t\tFrequency: %s" % (pocsag_cfg.frequency))
    print("\t\tBeacon Interval: %s" % (pocsag_cfg.beacon_interval))
    print("\tGPS Enabled: %s" % (sys_cfg.gps_en))
    print("\tLog Packets Enabled: %s" % (sys_cfg.log_packets_en))
    print("\tBoot Log Enabled: %s" % (sys_cfg.boot_log_en))
    print("\tWatchdog Timer Enabled: %s" % (sys_cfg.watchdog_timer_en))


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
    ret_str += "Timestamp: %s\r\n" % (log_msg.timestamp)
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
    ret_str += "Boot Time: %s\r\n" % (bootlog_msg.boot_time)
    ret_str += "Count: %s\r\n" % (bootlog_msg.count)
    print(ret_str)
    return ret_str