import pika
import time
import binascii
import qmesh_pb2
import yaml
import sys, os


channel = None
result = None
queue_name = None
if len(sys.argv) < 2:
    print('USAGE: <command> <tag_name>')
    sys.exit()
tag_name = sys.argv[1]

def cfg_to_yaml_file(ser_msg, yaml_file_path):
    cfg_dict = {}
#    cfg_dict['frequency'] = ser_msg.sys_cfg.radio_cfg.frequency
    cfg_dict['frequencies'] = []
    for freq in ser_msg.sys_cfg.radio_cfg.frequencies:
        cfg_dict['frequencies'].append(freq) 
    cfg_dict['tcxo_time_us'] = ser_msg.sys_cfg.radio_cfg.tcxo_time_us
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
    cfg_dict['gps_en'] = ser_msg.sys_cfg.gps_en
    cfg_dict['log_packets'] = ser_msg.sys_cfg.log_packets_en
    cfg_dict['log_boot'] = ser_msg.sys_cfg.boot_log_en
    cfg_dict['watchdog_timer_en'] = ser_msg.sys_cfg.watchdog_timer_en

    cfg_dict['esp0_exists'] = ser_msg.esp_cfg_msg.esp0.exists
    cfg_dict['esp0_isbt'] = ser_msg.esp_cfg_msg.esp0.isBT
    cfg_dict['esp0_isap'] = ser_msg.esp_cfg_msg.esp0.isAP
    cfg_dict['esp0_ser_name'] = ser_msg.esp_cfg_msg.esp0.ser_name
    cfg_dict['esp0_bt_name'] = ser_msg.esp_cfg_msg.esp0.bt_name
    cfg_dict['esp0_bt_pin'] = ser_msg.esp_cfg_msg.esp0.bt_pin
    cfg_dict['esp0_wifi_chan'] = ser_msg.esp_cfg_msg.esp0.wifi_chan
    cfg_dict['esp0_ssid'] = ser_msg.esp_cfg_msg.esp0.ssid
    cfg_dict['esp0_pass'] = ser_msg.esp_cfg_msg.esp0.password
    cfg_dict['esp0_ip_addr'] = ser_msg.esp_cfg_msg.esp0.ip_addr
    cfg_dict['esp0_gateway_addr'] = ser_msg.esp_cfg_msg.esp0.gateway_addr
    cfg_dict['esp0_subnet_addr'] = ser_msg.esp_cfg_msg.esp0.subnet_addr
    cfg_dict['esp0_dhcp_range_lo'] = ser_msg.esp_cfg_msg.esp0.dhcp_range_lo
    cfg_dict['esp0_dhcp_range_hi'] = ser_msg.esp_cfg_msg.esp0.dhcp_range_hi
    cfg_dict['esp0_multicast_addr'] = ser_msg.esp_cfg_msg.esp0.multicast_addr
    cfg_dict['esp0_local_port'] = ser_msg.esp_cfg_msg.esp0.local_port
    cfg_dict['esp0_remote_port'] = ser_msg.esp_cfg_msg.esp0.remote_port

    cfg_dict['esp1_exists'] = ser_msg.esp_cfg_msg.esp1.exists
    cfg_dict['esp1_isbt'] = ser_msg.esp_cfg_msg.esp1.isBT
    cfg_dict['esp1_isap'] = ser_msg.esp_cfg_msg.esp1.isAP
    cfg_dict['esp1_ser_name'] = ser_msg.esp_cfg_msg.esp1.ser_name
    cfg_dict['esp1_bt_name'] = ser_msg.esp_cfg_msg.esp1.bt_name
    cfg_dict['esp1_bt_pin'] = ser_msg.esp_cfg_msg.esp1.bt_pin
    cfg_dict['esp1_wifi_chan'] = ser_msg.esp_cfg_msg.esp1.wifi_chan
    cfg_dict['esp1_ssid'] = ser_msg.esp_cfg_msg.esp1.ssid
    cfg_dict['esp1_pass'] = ser_msg.esp_cfg_msg.esp1.password
    cfg_dict['esp1_ip_addr'] = ser_msg.esp_cfg_msg.esp1.ip_addr
    cfg_dict['esp1_gateway_addr'] = ser_msg.esp_cfg_msg.esp1.gateway_addr
    cfg_dict['esp1_subnet_addr'] = ser_msg.esp_cfg_msg.esp1.subnet_addr
    cfg_dict['esp1_dhcp_range_lo'] = ser_msg.esp_cfg_msg.esp1.dhcp_range_lo
    cfg_dict['esp1_dhcp_range_hi'] = ser_msg.esp_cfg_msg.esp1.dhcp_range_hi
    cfg_dict['esp1_multicast_addr'] = ser_msg.esp_cfg_msg.esp1.multicast_addr
    cfg_dict['esp1_local_port'] = ser_msg.esp_cfg_msg.esp1.local_port
    cfg_dict['esp1_remote_port'] = ser_msg.esp_cfg_msg.esp1.remote_port

    yaml_str = yaml.dump(cfg_dict)
    f = open(yaml_file_path, 'w')
    f.write(yaml_str)
    f.close()


def print_ver_msg(ser_msg):
    print(ser_msg.ver_msg.msg)


def print_cfg_msg(ser_msg):
    sys_cfg = ser_msg.sys_cfg
    print("CONFIGURATION:")
    print("\tMode: %s" % (sys_cfg.mode))
    print("\tAddress: %s" % (sys_cfg.address))
    print("\tRADIO CONFIGURATION:")
    radio_cfg = sys_cfg.radio_cfg
    print("\t\tType: %s" % (radio_cfg.type))
#    print("\t\tFrequency: %s" % (radio_cfg.frequency))
    print("\t\tHOPPING FREQUENCIES:")
    for freq in radio_cfg.frequencies:
        print("\t\t\t %s" % (freq))
    print("\t\tTransmit Power: %s" % (radio_cfg.tx_power))
    print("\t\tTCXO Setup Time (ms): %s" % (radio_cfg.tcxo_time_us))
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
    print("\tGPS Enabled: %s" % (sys_cfg.gps_en))
    print("\tLog Packets Enabled: %s" % (sys_cfg.log_packets_en))
    print("\tBoot Log Enabled: %s" % (sys_cfg.boot_log_en))
    print("\tWatchdog Timer Enabled: %s" % (sys_cfg.watchdog_timer_en))
    print("\tWalsh Codes Enabled: %s" % (sys_cfg.net_cfg.walsh_codes))
    print("\tInvert Bits: %s" % (sys_cfg.net_cfg.invert_bits))
    print("\tESP32 CONFIG:")
    esp_cfg = sys_cfg.esp_cfg
    print("\t\tESP0:")
    print("\t\t\tExists: %s" % (esp_cfg.esp0_exists))
    print("\t\t\tIs Bluetooth: %s" % (esp_cfg.esp0_isbt))
    print("\t\t\tIs AP: %s" % (esp_cfg.esp0_isap))
    print("\t\t\tSerial Name: %s" % (esp_cfg.esp0_ser_name))
    print("\t\t\tBluetooth Name: %s" % (esp_cfg.esp0_bt_name))
    print("\t\t\tBluetooth PIN: %s" % (esp_cfg.esp0_bt_pin))
    print("\t\t\tWi-Fi Channel: %s" % (esp_cfg.esp0_wifi_chan))
    print("\t\t\tWi-Fi SSID: %s" % (esp_cfg.esp0_ssid))
    print("\t\t\tWi-Fi Password: %s" % (esp_cfg.esp0_pass))
    print("\t\t\tIP Address: %s" % (esp_cfg.esp0_ip_addr))
    print("\t\t\tGateway IP Address: %s" % (esp_cfg.esp0_gateway_addr))
    print("\t\t\tSubnet IP Address: %s" % (esp_cfg.esp0_subnet_addr))
    print("\t\t\tDHCP Range Low IP Address: %s" % (esp_cfg.esp0_dhcp_range_lo))
    print("\t\t\tDHCP Range High IP Address: %s" % (esp_cfg.esp0_dhcp_range_hi))
    print("\t\t\tMulticast IP Address: %s" % (esp_cfg.esp0_multicast_addr))
    print("\t\t\tLocal Port: %s" % (esp_cfg.esp0_local_port))
    print("\t\t\tRemote Port: %s" % (esp_cfg.esp0_remote_port))

    print("\t\tESP1:")
    print("\t\t\tExists: %s" % (esp_cfg.esp1_exists))
    print("\t\t\tIs Bluetooth: %s" % (esp_cfg.esp1_isbt))
    print("\t\t\tIs AP: %s" % (esp_cfg.esp1_isap))
    print("\t\t\tSerial Name: %s" % (esp_cfg.esp1_ser_name))
    print("\t\t\tBluetooth Name: %s" % (esp_cfg.esp1_bt_name))
    print("\t\t\tBluetooth PIN: %s" % (esp_cfg.esp1_bt_pin))
    print("\t\t\tWi-Fi Channel: %s" % (esp_cfg.esp1_wifi_chan))
    print("\t\t\tWi-Fi SSID: %s" % (esp_cfg.esp1_ssid))
    print("\t\t\tWi-Fi Password: %s" % (esp_cfg.esp1_pass))
    print("\t\t\tIP Address: %s" % (esp_cfg.esp1_ip_addr))
    print("\t\t\tGateway IP Address: %s" % (esp_cfg.esp1_gateway_addr))
    print("\t\t\tSubnet IP Address: %s" % (esp_cfg.esp1_subnet_addr))
    print("\t\t\tDHCP Range Low IP Address: %s" % (esp_cfg.esp1_dhcp_range_lo))
    print("\t\t\tDHCP Range High IP Address: %s" % (esp_cfg.esp1_dhcp_range_hi))
    print("\t\t\tMulticast IP Address: %s" % (esp_cfg.esp1_multicast_addr))
    print("\t\t\tLocal Port: %s" % (esp_cfg.esp1_local_port))
    print("\t\t\tRemote Port: %s" % (esp_cfg.esp1_remote_port))


def publish_msg(ser_msg):
    global channel
    global tag_name
    channel.basic_publish(exchange='', routing_key='board_input_' + tag_name, \
        body=bytes(ser_msg.SerializeToString()))


def print_data_msg(data_msg):
    print("--------------------")
    print("Type: %d, Stream ID: %d, TTL: %d, Sender: %d, Sym Offset: %d, CRC: %d" % 
        (data_msg.type, data_msg.stream_id, data_msg.ttl, data_msg.sender, data_msg.sym_offset, 
            data_msg.crc))
    print(binascii.hexlify(bytearray(data_msg.payload)))
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
    print("Radio out queue level is %d" % (status_msg.radio_out_queue_level))
    # uint32 heap_size = 10;
    # uint32 peak_mem_usage = 11;
    # uint32 radio_out_queue_level = 12;
    # uint32 missed_deadlines = 13;
    # uint32 total_deadlines = 14;
    print("Heap size is %d" % (status_msg.heap_size))
    print("Peak Memory usage is %d" % (status_msg.peak_mem_usage))
    print("Total protocol deadlines is %d" % (status_msg.total_deadlines))
    print("Missed protocol deadlines is %d" % (status_msg.missed_deadlines))
    print("----------")


def setup_outgoing_rabbitmq(my_callback):
    global channel, result, queue_name, tag_name
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()
    channel.exchange_declare(exchange='board_output_' + tag_name, exchange_type='fanout')
    result = channel.queue_declare(queue='', exclusive=True)
    queue_name = result.method.queue
    channel.queue_bind(exchange='board_output_' + tag_name, queue=queue_name)
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

