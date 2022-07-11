/*
QMesh
Copyright (C) 2022 Daniel R. Fay

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mbed.h"
#include "pseudo_serial.hpp"
#include <vector>
#include <utility>
#include <memory>


void ESP32Manager::process_recv_data(const shared_ptr<vector<uint8_t>>& recv_data, const int32_t conn_num) {
    constexpr uint8_t KISS_DATA_FRAME = 0x00;
    constexpr uint8_t KISS_FEND = 0xC0;
    constexpr uint8_t KISS_PORT_MASK = 0x0F;
    PORTABLE_ASSERT((*recv_data)[0] == KISS_FEND);
    uint8_t cmd_code = (*recv_data)[1] >> 4U;
    PORTABLE_ASSERT(cmd_code == KISS_DATA_FRAME);
    uint8_t port = (*recv_data)[1] & KISS_PORT_MASK;
    PORTABLE_ASSERT(port > 0x02);
    if(static_cast<ser_port_type_t>(port) == DATA_PORT) { ps_data->recv_msg(recv_data); } 
    else { ps_voice->recv_msg(recv_data); }
    ps_debug->recv_msg(recv_data);
    // Update what the port belongs to
    ports.insert(pair<int, ser_port_type_t>(conn_num, static_cast<ser_port_type_t>(port)));
}

/// This method accepts incoming data from the ESP32 and decides what
///  to do with it
void ESP32Manager::oob_recv_data() {
    int32_t conn_num = -1;
    int32_t num_bytes = -1;
    constexpr int MAX_RECV_BYTES = 2048;
    PORTABLE_ASSERT(at_parser->recv("+IPD,%d,%d", &conn_num, &num_bytes));
    PORTABLE_ASSERT(num_bytes <= MAX_RECV_BYTES);
    auto recv_data = make_shared<vector<uint8_t>>();
    for(int i = 0; i < num_bytes; i++) {
        recv_data->push_back(at_parser->getc());
    }
    process_recv_data(recv_data, conn_num);
}

/// This method distributes outgoing data
void ESP32Manager::send_data(const ser_port_type_t port_type, vector<uint8_t> &data) {
    at_parser->abort();
    for(auto & port : ports) {
        if(port.second == port_type) {
            at_parser->send("AT+CIPSEND=%d,%d\r\n", port.first, data.size());
            PORTABLE_ASSERT(at_parser->recv("OK\r\n\r\n>"));
            at_parser->write((char *)(data.data()), data.size()); 
            size_t sent_bytes = 0;
            PORTABLE_ASSERT(at_parser->recv("Recv %d bytes\r\n", &sent_bytes));
            PORTABLE_ASSERT(sent_bytes == data.size());
            PORTABLE_ASSERT("\r\n");
            PORTABLE_ASSERT("SEND OK\r\n");
        }
    }
    data.clear();
}

void ESP32Manager::recv_data() {
    while(true) {
        // Dummy scanf designed to trigger the OOB receiver
        at_parser->recv("%s");
    }
}

ESP32Manager::ESP32Manager(PinName tx, PinName rx, PinName rst, PinName cts, PinName rts, 
                            ESP32CfgSubMsg &my_cfg) : 
    cfg(my_cfg),
    tx_port(tx),
    rx_port(rx),
    cts_port(cts),
    rts_port(rts),
    esp32_rst(DigitalInOut(rst)),
    ser(make_shared<UARTSerial>(tx_port, rx_port, SER_BAUD_RATE)),
    ps_voice(nullptr),
    ps_data(nullptr),
    ps_debug(nullptr),
    at_was_ok(false),
    at_was_err(false)
{
    PORTABLE_ASSERT(string(cfg.ssid).size() <= SSID_MAX_LEN);
    PORTABLE_ASSERT(string(cfg.pass).size() <= PASS_MAX_LEN);
    // Reset the ESP32 board
    esp32_rst.mode(OpenDrain);
    esp32_rst.mode(PullNone);
    esp32_rst = 0;
    portability::sleep(HALF_SECOND);
    esp32_rst = 1;
    portability::sleep(ONE_SECOND);
    // Set up the flow control on the STM32's UART
    ser->set_flow_control(mbed::SerialBase::RTSCTS, rts_port, cts_port);
    portability::sleep(QUARTER_SECOND);
    FILE *ser_fh = fdopen(&*ser, "rw");
    PORTABLE_ASSERT(ser_fh != nullptr);
    // Start the AT command parser
    at_parser = make_shared<ATCmdParser>(&*ser);
    at_parser->debug_on(1);
    at_parser->set_delimiter("\r\n");

    if(cfg.isAP) {
        // Set the Wi-Fi mode to SoftAP mode
        string wifi_mode_cmd("AT+CWMODE=2\r\n");
        at_parser->send("%s", wifi_mode_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set the ESP SoftAP params
        string wifi_softap_cmd("AT+CWSAP=");
        wifi_softap_cmd.append("\"");
        wifi_softap_cmd.append(cfg.ssid);
        wifi_softap_cmd.append("\"");
        wifi_softap_cmd.append(",");
        wifi_softap_cmd.append("\"");
        wifi_softap_cmd.append(cfg.pass);
        wifi_softap_cmd.append("\"");
        wifi_softap_cmd.append(",");
        wifi_softap_cmd.append(cfg.wifi_chan);
        wifi_softap_cmd.append(",");
        wifi_softap_cmd.append("3"); // Open; no encryption
        wifi_softap_cmd.append("\r\n");
        at_parser->send("%s", wifi_softap_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));            
        // Set address of the ESP32 SoftAP
        string wifi_ip_cmd("AT+CIPAP=");
        wifi_ip_cmd.append("\"");
        wifi_ip_cmd.append(cfg.ip_addr);
        wifi_ip_cmd.append("\"");
        wifi_ip_cmd.append(",");
        wifi_ip_cmd.append("\"");
        wifi_ip_cmd.append(cfg.gateway_addr);
        wifi_ip_cmd.append("\"");
        wifi_ip_cmd.append(",");
        wifi_ip_cmd.append("\"");
        wifi_ip_cmd.append(cfg.subnet_addr);
        wifi_ip_cmd.append("\"");
        wifi_ip_cmd.append("\r\n");
        at_parser->send("%s", wifi_ip_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Enable DHCP
        string dhcp_en_cmd("AT+CWDHCP=1,2\r\n");
        at_parser->send("%s", dhcp_en_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Setup the DHCP address ranges
        string dhcp_addr_cmd("AT+CWDHCPS=1,3,");
        dhcp_addr_cmd.append("\"");
        dhcp_addr_cmd.append(cfg.dhcp_range_lo);
        dhcp_addr_cmd.append("\"");
        dhcp_addr_cmd.append(",");
        dhcp_addr_cmd.append("\"");
        dhcp_addr_cmd.append(cfg.dhcp_range_hi);
        dhcp_addr_cmd.append("\"");            
        dhcp_addr_cmd.append("\r\n");
        at_parser->send("%s", dhcp_addr_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        //*****************************

        // Setup the DHCP server
        string wifi_dhcp_cmd("AT+CWDHCP=1,2\r\n");
        fprintf(ser_fh, "%s", wifi_dhcp_cmd.c_str());
        string wifi_dhcp_ip_range("AT+CWDHCPS=1,3,");
        wifi_dhcp_ip_range.append("\"");
        wifi_dhcp_ip_range.append(cfg.dhcp_range_lo);
        wifi_dhcp_ip_range.append("\"");
        wifi_dhcp_ip_range.append(",\"");
        wifi_dhcp_ip_range.append(cfg.dhcp_range_hi);
        wifi_dhcp_ip_range.append("\"\r\n");
        at_parser->send("%s", wifi_dhcp_ip_range.c_str());
    } else {
        // Enable DHCP
        string wifi_dhcp_cmd("AT+CWDHCP=1,1\r\n");
        fprintf(ser_fh, "%s", wifi_dhcp_cmd.c_str());
        // Setup and enable mDNS
        string wifi_mdns_cmd("AT+MDNS=1,\"QMesh-");
        wifi_mdns_cmd.append(cfg.ssid);
        wifi_mdns_cmd.append("\",\"_iot\",8080\r\n");
        fprintf(ser_fh, "%s", wifi_mdns_cmd.c_str()); 
        // Connect to the AP
        string wifi_conn_ap_cmd("AT+CWJAP=");
        wifi_conn_ap_cmd.append(cfg.ssid);
        wifi_conn_ap_cmd.append(",");
        wifi_conn_ap_cmd.append(cfg.pass);
        wifi_conn_ap_cmd.append("\r\n");
        at_parser->send("%s", wifi_conn_ap_cmd.c_str());
    }

    // Setup a UDP server to transmit/receive on a multicast IP address
    string wifi_conn_mux_cmd("AT+CIPMUX=0\r\n");
    fprintf(ser_fh, "%s", wifi_conn_mux_cmd.c_str());
    string wifi_udp_srvr_cmd("AT+CIPSTART=");
    wifi_udp_srvr_cmd.append(cfg.multicast_addr);
    wifi_udp_srvr_cmd.append(",");
    wifi_udp_srvr_cmd.append(cfg.remote_port);
    wifi_udp_srvr_cmd.append(",");
    wifi_udp_srvr_cmd.append(cfg.local_port);
    wifi_udp_srvr_cmd.append(",0\r\n");
    at_parser->send("%s", wifi_udp_srvr_cmd.c_str());

    // Enter passthrough mode
    string wifi_pass_cmd("AT+CIPSEND\r\n");
    at_parser->send("%s", wifi_pass_cmd.c_str());
    portability::sleep(QUARTER_SECOND);

    printf("Done with configuration\r\n");
}


auto ESP32PseudoSerial::getc() -> int {
    osEvent evt = rx_data->get(osWaitForever);
    if(evt.status == osEventMail) {
        char *val = static_cast<char *>(evt.value.p);
        return *val;
    }  
    return EOF;
}


auto ESP32PseudoSerial::putc(const int val) -> int {
    outbuf.push_back(val);
    if(val == KISS_FEND || outbuf.size() >= RX_BUF_SIZE) {
        PORTABLE_ASSERT(esp32_manager != nullptr);
        esp32_manager->send_data(port_type, outbuf);
    }
    return val;
}


#if MBED_CONF_APP_HAS_BLE == 1
Mail<pair<ser_port_type_t, shared_ptr<vector<uint8_t>>>, BLE_QUEUE_SIZE> ble_out_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> voice_in_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> aprs_in_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> dbg_in_queue;
#endif /* MBED_CONF_APP_HAS_BLE == 1 */