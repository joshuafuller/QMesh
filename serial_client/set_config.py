#!/usr/bin/python3

# QMesh
# Copyright (C) 2019 Daniel R. Fay

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
import qmesh_pb2
import qmesh_common

if len(sys.argv) < 5:
    print("USAGE: set_config.py <configfile> <node id> <beacon interval> <power level (dBm)>")
    sys.exit(0)

config_file = sys.argv[2]
node_id = int(sys.argv[3])
beacon_interval = int(sys.argv[4])
tx_power = int(sys.argv[5])

cfg_dict = None
with open(config_file) as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    cfg_dict = yaml.load(file, Loader=yaml.FullLoader)
    print(cfg_dict)

def dbg_process(ch, method, properties, body):
    ser_msg = qmesh_pb2.SerialMsg()
    ser_msg.ParseFromString(body)
    if(ser_msg.type == ser_msg.STATUS):
        qmesh_common.print_status_msg(ser_msg.status)
        if(ser_msg.status.status == ser_msg.status.MANAGEMENT):
            qmesh_common.channel.stop_consuming()

# Set up the RabbitMQ connection
qmesh_common.setup_outgoing_rabbitmq(dbg_process)

# Reboot the board
qmesh_common.reboot_board()
qmesh_common.channel.start_consuming()

# Stay in management
qmesh_common.stay_in_management()

# Send out the settings block
ser_msg = qmesh_pb2.SerialMsg()
ser_msg.type = ser_msg.SET_CONFIG
ser_msg.sys_cfg.address = node_id
ser_msg.sys_cfg.radio_cfg.type = qmesh_pb2.RadioCfg.LORA
#ser_msg.sys_cfg.radio_cfg.frequency = cfg_dict['frequency']
ser_msg.sys_cfg.radio_cfg.frequencies.extend(cfg_dict['frequencies'])
ser_msg.sys_cfg.radio_cfg.tx_power = tx_power
ser_msg.sys_cfg.radio_cfg.tcxo_time_us = cfg_dict['tcxo_time_us']
ser_msg.sys_cfg.radio_cfg.lora_cfg.bw = cfg_dict['bw']
ser_msg.sys_cfg.radio_cfg.lora_cfg.sf = cfg_dict['sf']
ser_msg.sys_cfg.radio_cfg.lora_cfg.cr = cfg_dict['cr']
ser_msg.sys_cfg.radio_cfg.lora_cfg.preamble_length = cfg_dict['preamble_length']
ser_msg.sys_cfg.test_cfg.cw_test_mode = cfg_dict['cw_test_mode']
ser_msg.sys_cfg.test_cfg.preamble_test_mode = cfg_dict['preamble_test_mode']
#ser_msg.sys_cfg.test_cfg.fec_test_mode = cfg_dict['fec_test_mode']
ser_msg.sys_cfg.fec_cfg.type = cfg_dict['fec_algo']
ser_msg.sys_cfg.fec_cfg.conv_rate = cfg_dict['conv_rate']
ser_msg.sys_cfg.fec_cfg.conv_order = cfg_dict['conv_order']
ser_msg.sys_cfg.fec_cfg.rs_num_roots = cfg_dict['rs_num_roots']
ser_msg.sys_cfg.net_cfg.beacon_msg = cfg_dict['beacon_msg']
ser_msg.sys_cfg.net_cfg.beacon_interval = beacon_interval
ser_msg.sys_cfg.net_cfg.num_offsets = cfg_dict['num_offsets']
#ser_msg.sys_cfg.net_cfg.pld_len = cfg_dict['payload_length']
ser_msg.sys_cfg.gps_en = cfg_dict['gps_en']
#ser_msg.sys_cfg.log_packets_en = cfg_dict['log_packets_en']
ser_msg.sys_cfg.boot_log_en = cfg_dict['log_boot']
ser_msg.sys_cfg.watchdog_timer_en = cfg_dict['watchdog_timer_en']
ser_msg.sys_cfg.net_cfg.walsh_codes = cfg_dict['walsh_codes']
ser_msg.sys_cfg.net_cfg.invert_bits = cfg_dict['invert_bits']

ser_msg.sys_cfg.net_cfg.voice_frames_per_frame = cfg_dict['voice_frames_per_frame']
ser_msg.sys_cfg.net_cfg.codec2_bitrate = cfg_dict['codec2_bitrate']

# ESP32 configuration
ser_msg.sys_cfg.esp_cfg_msg.esp0.exists = cfg_dict['esp0_exists'] 
ser_msg.sys_cfg.esp_cfg_msg.esp0.isBT = cfg_dict['esp0_isbt']
ser_msg.sys_cfg.esp_cfg_msg.esp0.isAP = cfg_dict['esp0_isap']
ser_msg.sys_cfg.esp_cfg_msg.esp0.ser_name = cfg_dict['esp0_ser_name']
ser_msg.sys_cfg.esp_cfg_msg.esp0.bt_name = cfg_dict['esp0_bt_name']
ser_msg.sys_cfg.esp_cfg_msg.esp0.bt_pin = cfg_dict['esp0_bt_pin']
ser_msg.sys_cfg.esp_cfg_msg.esp0.wifi_chan = cfg_dict['esp0_wifi_chan']
ser_msg.sys_cfg.esp_cfg_msg.esp0.ssid = cfg_dict['esp0_ssid']
ser_msg.sys_cfg.esp_cfg_msg.esp0.password = cfg_dict['esp0_pass']
ser_msg.sys_cfg.esp_cfg_msg.esp0.ip_addr = cfg_dict['esp0_ip_addr']
ser_msg.sys_cfg.esp_cfg_msg.esp0.gateway_addr = cfg_dict['esp0_gateway_addr']
ser_msg.sys_cfg.esp_cfg_msg.esp0.subnet_addr = cfg_dict['esp0_subnet_addr']
ser_msg.sys_cfg.esp_cfg_msg.esp0.dhcp_range_lo = cfg_dict['esp0_dhcp_range_lo']
ser_msg.sys_cfg.esp_cfg_msg.esp0.dhcp_range_hi = cfg_dict['esp0_dhcp_range_hi']
ser_msg.sys_cfg.esp_cfg_msg.esp0.multicast_addr = cfg_dict['esp0_multicast_addr']
ser_msg.sys_cfg.esp_cfg_msg.esp0.local_port = cfg_dict['esp0_local_port']
ser_msg.sys_cfg.esp_cfg_msg.esp0.remote_port = cfg_dict['esp0_remote_port']

ser_msg.sys_cfg.esp_cfg_msg.esp1.exists = cfg_dict['esp1_exists']
ser_msg.sys_cfg.esp_cfg_msg.esp1.isBT = cfg_dict['esp1_isbt']
ser_msg.sys_cfg.esp_cfg_msg.esp1.isAP = cfg_dict['esp1_isap']
ser_msg.sys_cfg.esp_cfg_msg.esp1.ser_name = cfg_dict['esp1_ser_name']
ser_msg.sys_cfg.esp_cfg_msg.esp1.bt_name = cfg_dict['esp1_bt_name']
ser_msg.sys_cfg.esp_cfg_msg.esp1.bt_pin = cfg_dict['esp1_bt_pin']
ser_msg.sys_cfg.esp_cfg_msg.esp1.wifi_chan = cfg_dict['esp1_wifi_chan']
ser_msg.sys_cfg.esp_cfg_msg.esp1.ssid = cfg_dict['esp1_ssid']
ser_msg.sys_cfg.esp_cfg_msg.esp1.password = cfg_dict['esp1_pass']
ser_msg.sys_cfg.esp_cfg_msg.esp1.ip_addr = cfg_dict['esp1_ip_addr']
ser_msg.sys_cfg.esp_cfg_msg.esp1.gateway_addr = cfg_dict['esp1_gateway_addr']
ser_msg.sys_cfg.esp_cfg_msg.esp1.subnet_addr = cfg_dict['esp1_subnet_addr']
ser_msg.sys_cfg.esp_cfg_msg.esp1.dhcp_range_lo = cfg_dict['esp1_dhcp_range_lo']
ser_msg.sys_cfg.esp_cfg_msg.esp1.dhcp_range_hi = cfg_dict['esp1_dhcp_range_hi']
ser_msg.sys_cfg.esp_cfg_msg.esp1.multicast_addr = cfg_dict['esp1_multicast_addr']
ser_msg.sys_cfg.esp_cfg_msg.esp1.local_port = cfg_dict['esp1_local_port']
ser_msg.sys_cfg.esp_cfg_msg.esp1.remote_port = cfg_dict['esp1_remote_port']


ser_msg.sys_cfg.valid = True

qmesh_common.publish_msg(ser_msg)
qmesh_common.channel.start_consuming()

# Reboot the board again
qmesh_common.reboot_board()



