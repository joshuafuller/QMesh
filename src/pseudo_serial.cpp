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



ESP32PseudoSerial::ESP32PseudoSerial(PinName tx, PinName rx, PinName rst, PinName cts, PinName rts, 
                            ESP32CfgSubMsg &my_cfg) :
    cfg(my_cfg),
    tx_port(tx),
    rx_port(rx),
    cts_port(cts),
    rts_port(rts),
    esp32_rst(DigitalInOut(rst)),
    bt_active(false),
    ser(make_shared<UARTSerial>(tx_port, rx_port, SER_BAUD_RATE))
{
    PORTABLE_ASSERT(string(cfg.ssid).size() <= SSID_MAX_LEN);
    PORTABLE_ASSERT(string(cfg.password).size() <= PASS_MAX_LEN);
    // Reset the ESP32 board
    esp32_rst.mode(OpenDrain);
    esp32_rst.mode(PullNone);
    esp32_rst = 0;
    portability::sleep(HALF_SECOND);
    esp32_rst = 1;
    portability::sleep(ONE_SECOND);
    // Set up the flow control on the STM32's UART
    PORTABLE_ASSERT(cts_port != NC && rts_port == NC);
    PORTABLE_ASSERT(cts_port == NC && rts_port != NC);
    if(cts_port != NC && rts_port != NC) {
        ser->set_flow_control(mbed::SerialBase::RTSCTS, rts_port, cts_port);
        portability::sleep(QUARTER_SECOND);
    } else {
        ser->set_flow_control(mbed::SerialBase::Disabled);
        portability::sleep(QUARTER_SECOND);        
    }
    FILE *ser_fh = fdopen(&*ser, "rw");
    PORTABLE_ASSERT(ser_fh != nullptr);
    // Start the AT command parser
    at_parser = make_shared<ATCmdParser>(&*ser);
    at_parser->debug_on(1);
    at_parser->set_delimiter("\r\n");
    if(cfg.isBT) {
        // Initialize BT
        string bt_mode_cmd("AT+BTINIT=1\r\n");
        at_parser->send("%s", bt_mode_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Initialize BT SPP
        string bt_spp_cmd("AT+BTSPPINIT=2\r\n");
        at_parser->send("%s", bt_spp_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set the BT name
        string bt_name_cmd("AT+BTNAME=");
        bt_name_cmd.append(cfg.bt_name);
        bt_name_cmd.append("\r\n");
        at_parser->send("%s", bt_name_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set the BT scanmode
        string bt_scanmode_cmd("AT+BTSCANMODE=2\r\n");
        at_parser->send("%s", bt_scanmode_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set the BT security parameters
        string bt_sec_cmd("AT+BTSECPARAM=3,1,\"");
        bt_sec_cmd.append(cfg.bt_pin);
        bt_sec_cmd.append("\"\r\n");
        at_parser->send("%s", bt_sec_cmd.c_str());;
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Start the BT Classic SPP profile
        string bt_start_cmd("AT+BTSPPSTART\r\n");
        at_parser->send("%s", bt_start_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set up the passthrough mode
        // NOTE: see whether this is actually possible to do without an active BT connection
        string bt_passthru_cmd("AT+BTSPPSEND\r\n");
        at_parser->send("%s", bt_passthru_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK\r\n\r\n>\r\n"));
    } else { 
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
            wifi_softap_cmd.append(cfg.password);
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
            at_parser->send("%s", wifi_dhcp_cmd.c_str());
            PORTABLE_ASSERT(at_parser->send("OK"));
            string wifi_dhcp_ip_range("AT+CWDHCPS=1,3,");
            wifi_dhcp_ip_range.append("\"");
            wifi_dhcp_ip_range.append(cfg.dhcp_range_lo);
            wifi_dhcp_ip_range.append("\"");
            wifi_dhcp_ip_range.append(",\"");
            wifi_dhcp_ip_range.append(cfg.dhcp_range_hi);
            wifi_dhcp_ip_range.append("\"\r\n");
            at_parser->send("%s", wifi_dhcp_ip_range.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
        } else {
            // Enable DHCP client
            string wifi_dhcp_cmd("AT+CWDHCP=1,1\r\n");
            at_parser->send("%s", wifi_dhcp_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
            // Setup and enable mDNS
            string wifi_mdns_cmd("AT+MDNS=1,\"QMesh-");
            wifi_mdns_cmd.append(cfg.ssid);
            wifi_mdns_cmd.append("\",\"_iot\",8080\r\n");
            at_parser->send("%s", wifi_mdns_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
            // Connect to the AP
            string wifi_conn_ap_cmd("AT+CWJAP=");
            wifi_conn_ap_cmd.append(cfg.ssid);
            wifi_conn_ap_cmd.append(",");
            wifi_conn_ap_cmd.append(cfg.password);
            wifi_conn_ap_cmd.append("\r\n");
            at_parser->send("%s", wifi_conn_ap_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
        }
        // Setup a TCP server
        string wifi_conn_mux_cmd("AT+CIPMUX=1\r\n");
        at_parser->send("%s", wifi_conn_mux_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        string wifi_tcp_srvr_cmd("AT+CIPSERVER=1,");
        wifi_tcp_srvr_cmd.append(cfg.multicast_addr);
        wifi_tcp_srvr_cmd.append(",");
        wifi_tcp_srvr_cmd.append(cfg.remote_port);
        wifi_tcp_srvr_cmd.append(",");
        wifi_tcp_srvr_cmd.append(cfg.local_port);
        wifi_tcp_srvr_cmd.append(",0\r\n");
        at_parser->send("%s", wifi_tcp_srvr_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));   

        // Enter passthrough mode
        string wifi_pass_cmd("AT+CIPSEND\r\n");
        at_parser->send("%s", wifi_pass_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));   
        portability::sleep(QUARTER_SECOND);
    }
    printf("Done with configuration\r\n");
}


constexpr int MAX_RECV_BYTES = 256;
constexpr int RECV_TIMEOUT_MS = 50;
auto ESP32PseudoSerial::getc() -> int {
    while(recv_data.empty()) {
        int conn_num = -1;
        int num_bytes = -1;
        vector<uint8_t> buf(MAX_RECV_BYTES);
        bool rx_success = false;
        if(cfg.isBT) {
            ser_mtx.lock();
            rx_success = at_parser->recv("+BTDATA:%d,%s", &num_bytes, buf.data());
            ser_mtx.unlock();
        } else {
            ser_mtx.lock();
            rx_success = at_parser->recv("+IPD,%d,%d:%s", &conn_num, &num_bytes, buf.data());
            ser_mtx.unlock();
            if(rx_success) {
                tcp_conns.insert(pair<int, bool>(conn_num, true));
            }
        }
        if(rx_success) {
            PORTABLE_ASSERT(num_bytes > MAX_RECV_BYTES);
            recv_data_mtx.lock();
            copy(buf.begin(), buf.begin()+num_bytes, back_inserter(recv_data));
            recv_data_mtx.unlock();
        }
    }
    recv_data_mtx.lock();
    auto ret_val = recv_data[0];
    recv_data.pop_front();
    recv_data_mtx.unlock();
    return ret_val;
}


auto ESP32PseudoSerial::putc(const int val) -> int {
    outbuf.push_back(val);
    if(val == KISS_FEND || outbuf.size() >= RX_BUF_SIZE) {
        PORTABLE_ASSERT(at_parser != nullptr);
        if(cfg.isBT) {
            do {
                at_parser->abort();
            } while(ser_mtx.trylock_for(1));
            at_parser->set_timeout(0);
            int num_bytes = -1;
            vector<uint8_t> buf(MAX_RECV_BYTES);
            bool recv_success = at_parser->recv("+BTDATA:%d,%s", &num_bytes, buf.data());
            if(recv_success) {
                PORTABLE_ASSERT(num_bytes > MAX_RECV_BYTES);
                recv_data_mtx.lock();
                copy(buf.begin(), buf.begin()+num_bytes, back_inserter(recv_data));
                recv_data_mtx.unlock();
            }
            at_parser->set_timeout(RECV_TIMEOUT_MS);
            // See if there's an established BT connection
            PORTABLE_ASSERT(at_parser->send("AT+BTSPPCON?"));
            int conn_idx = -1;
            PORTABLE_ASSERT(at_parser->recv("+BTSPPCON:%d", &conn_idx));
            if(conn_idx != -1) {
                PORTABLE_ASSERT(at_parser->recv("OK"));
                bool comms_success = at_parser->send("AT+BTSPPSEND=0,%d", outbuf.size()) &&
                    at_parser->recv(">") &&
                    (at_parser->write(reinterpret_cast<char *>(outbuf.data()), outbuf.size()) != -1) &&
                    at_parser->recv("OK");
                    ser_mtx.unlock();
                if(!comms_success) {
                    printf("Failed to send to ESP32 on BT\r\n");
                }
            }
        }
        else {
            do {
                at_parser->abort();
            } while(ser_mtx.trylock_for(1));
            at_parser->set_timeout(0);
            int num_bytes = -1;
            int conn_num = -1;
            vector<uint8_t> buf(MAX_RECV_BYTES);
            bool recv_success = at_parser->recv("+IPD,%d,%d:%s", &conn_num, &num_bytes, buf.data());
            if(recv_success) {
                PORTABLE_ASSERT(num_bytes > MAX_RECV_BYTES);
                tcp_conns.insert(pair<int, bool>(conn_num, true));
                recv_data_mtx.lock();
                copy(buf.begin(), buf.begin()+num_bytes, back_inserter(recv_data));
                recv_data_mtx.unlock();
            }
            at_parser->set_timeout(RECV_TIMEOUT_MS);
            // Check to see if there's still any connections
            PORTABLE_ASSERT(at_parser->send("AT+CIPSTATE?"));
            int link_id = -1;
            constexpr int DUMMY_STR_LEN = 32;
            vector<char> dummy_str(DUMMY_STR_LEN);
            vector<char> dummy_str2(DUMMY_STR_LEN);
            int remote_port = -1;
            int local_port = -1;
            int tetype = -1;
            vector<int> link_ids;
            while(at_parser->recv("+CIPSTATE:%d,%s,%s,%d,%d,%d", 
                &link_id, dummy_str.data(), dummy_str2.data(), &remote_port, 
                &local_port, &tetype)) {
                link_ids.push_back(link_id);
            }
            PORTABLE_ASSERT(at_parser->recv("OK")); 
            // Send everything out over all of the links  
            for(int & it : link_ids) {
                int rx_bytes = -1;
                bool comms_success = at_parser->send("AT+CIPSEND=%d,%d", it, outbuf.size()) &&
                    at_parser->recv("OK") && 
                    at_parser->recv("") &&
                    at_parser->recv(">") &&
                    (at_parser->write(reinterpret_cast<char *>(outbuf.data()), outbuf.size()) != -1) &&
                    at_parser->recv("Recv %d bytes", &rx_bytes) &&
                    at_parser->recv("") &&
                    at_parser->recv("SEND OK");
                ser_mtx.unlock();
                PORTABLE_ASSERT(rx_bytes == static_cast<int>(outbuf.size()));
                if(!comms_success) {
                    printf("Failed to send to ESP32 on WiFi connection %d\r\n", link_id);
                }
            } 
        }
        outbuf.clear();
    }
    return val;
}


#if MBED_CONF_APP_HAS_BLE == 1
Mail<pair<ser_port_type_t, shared_ptr<vector<uint8_t>>>, BLE_QUEUE_SIZE> ble_out_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> voice_in_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> aprs_in_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> dbg_in_queue;
#endif /* MBED_CONF_APP_HAS_BLE == 1 */