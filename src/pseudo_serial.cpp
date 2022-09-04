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
#include <cstdlib>


auto PseudoSerial::safe_pcts(const string &send_str) -> string {
    string safe_str("%");
    safe_str.append(to_string(send_str.size()));
    safe_str.append("s");
    return safe_str;
}


constexpr int MAX_RECV_BYTES = 256;
constexpr int RECV_TIMEOUT_MS = 10;
constexpr int RECV_TIMEOUT_RX_MS = 25;
ESP32PseudoSerial::ESP32PseudoSerial(PinName tx, PinName rx, PinName rst, PinName cts, PinName rts, 
                            ESP32CfgSubMsg &my_cfg) :
    cfg(my_cfg),
    tx_port(tx),
    rx_port(rx),
    cts_port(cts),
    rts_port(rts),
    esp32_rst(DigitalInOut(rst)),
    bt_active(false),
    ser(make_shared<UARTSerial>(tx_port, rx_port, ESP_BAUD_RATE))
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
    PORTABLE_ASSERT(!((cts_port == NC) ^ (rts_port == NC)));
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
    at_parser->debug_on(0);
    at_parser->set_timeout(RECV_TIMEOUT_MS);
    at_parser->set_delimiter("\r\n");
    if(cfg.isBT) {
        string bt_mode_chk_cmd("AT+BTINIT?\r\n");
        at_parser->send(safe_pcts(bt_mode_chk_cmd).c_str(), bt_mode_chk_cmd.c_str());
        int32_t bt_init = -1;
        at_parser->recv("+BTINIT:%d\r\n", &bt_init);
        at_parser->recv("OK");
        if(bt_init == 0) {
            // Initialize BT
            string bt_mode_cmd("AT+BTINIT=1\r\n");
            at_parser->send(safe_pcts(bt_mode_cmd).c_str(), bt_mode_cmd.c_str());
            portability::sleep(ONE_SECOND);
            PORTABLE_ASSERT(at_parser->recv("OK"));
        }
        // Initialize BT SPP
        string bt_spp_cmd("AT+BTSPPINIT=2\r\n");
        at_parser->send(safe_pcts(bt_spp_cmd).c_str(), bt_spp_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set the BT name
        string bt_name_cmd("AT+BTNAME=");
        bt_name_cmd.append("\"");
        bt_name_cmd.append(cfg.bt_name);
        bt_name_cmd.append("\"\r\n");
        at_parser->send(safe_pcts(bt_name_cmd).c_str(), bt_name_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set the BT scanmode
        string bt_scanmode_cmd("AT+BTSCANMODE=2\r\n");
        at_parser->send(safe_pcts(bt_scanmode_cmd).c_str(), bt_scanmode_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set the BT security parameters
        string bt_sec_cmd("AT+BTSECPARAM=3,1,\"");
        bt_sec_cmd.append(cfg.bt_pin);
        bt_sec_cmd.append("\"\r\n");
        at_parser->send(safe_pcts(bt_sec_cmd).c_str(), bt_sec_cmd.c_str());;
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Start the BT Classic SPP profile
        string bt_start_cmd("AT+BTSPPSTART\r\n");
        at_parser->send(safe_pcts(bt_start_cmd).c_str(), bt_start_cmd.c_str());
        PORTABLE_ASSERT(at_parser->recv("OK"));
        // Set up the passthrough mode
        // NOTE: see whether this is actually possible to do without an active BT connection
        //string bt_passthru_cmd("AT+BTSPPSEND\r\n");
        //at_parser->send(safe_pcts(bt_passthru_cmd).c_str(), bt_passthru_cmd.c_str());
        //PORTABLE_ASSERT(at_parser->recv("OK\r\n\r\n>\r\n"));
    } else { 
        if(cfg.isAP) {
            // Set the Wi-Fi mode to SoftAP mode
            string wifi_mode_cmd("AT+CWMODE=2\r\n");
            at_parser->send(safe_pcts(wifi_mode_cmd).c_str(), wifi_mode_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
            string wifi_tcp_close_cmd("AT+CIPCLOSE=5\r\n");
            at_parser->send(safe_pcts(wifi_tcp_close_cmd).c_str(), wifi_tcp_close_cmd.c_str());
            vector<char> test_str(MAX_RECV_BYTES);
            string wifi_tcp_close_str("%");
            wifi_tcp_close_str.append(to_string(MAX_RECV_BYTES));
            wifi_tcp_close_str.append("s");
            PORTABLE_ASSERT(at_parser->recv(wifi_tcp_close_str.data(), test_str.data()));
            string wifi_conn_mux_cmd("AT+CIPMUX=1\r\n");
            at_parser->send(safe_pcts(wifi_conn_mux_cmd).c_str(), wifi_conn_mux_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
            // Setup a TCP server
            string wifi_tcp_srvr_cmd("AT+CIPSERVER=1,");
            wifi_tcp_srvr_cmd.append(cfg.local_port);
            wifi_tcp_srvr_cmd.append("\r\n");
            at_parser->send(safe_pcts(wifi_tcp_srvr_cmd).c_str(), wifi_tcp_srvr_cmd.c_str());
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
            at_parser->send(safe_pcts(wifi_softap_cmd).c_str(), wifi_softap_cmd.c_str());
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
            at_parser->send(safe_pcts(wifi_ip_cmd).c_str(), wifi_ip_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
            // Enable DHCP
            string dhcp_en_cmd("AT+CWDHCP=1,2\r\n");
            at_parser->send(safe_pcts(dhcp_en_cmd).c_str(), dhcp_en_cmd.c_str());
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
            at_parser->send(safe_pcts(dhcp_addr_cmd).c_str(), dhcp_addr_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
            //*****************************

            // Setup the DHCP server
            string wifi_dhcp_cmd("AT+CWDHCP=1,2\r\n");
            at_parser->send(safe_pcts(wifi_dhcp_cmd).c_str(), wifi_dhcp_cmd.c_str());
            PORTABLE_ASSERT(at_parser->send("OK"));
            string wifi_dhcp_ip_range("AT+CWDHCPS=1,3,");
            wifi_dhcp_ip_range.append("\"");
            wifi_dhcp_ip_range.append(cfg.dhcp_range_lo);
            wifi_dhcp_ip_range.append("\"");
            wifi_dhcp_ip_range.append(",\"");
            wifi_dhcp_ip_range.append(cfg.dhcp_range_hi);
            wifi_dhcp_ip_range.append("\"\r\n");
            at_parser->send(safe_pcts(wifi_dhcp_ip_range).c_str(), wifi_dhcp_ip_range.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
        } else {
            // Enable DHCP client
            string wifi_dhcp_cmd("AT+CWDHCP=1,1\r\n");
            at_parser->send(safe_pcts(wifi_dhcp_cmd).c_str(), wifi_dhcp_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
            // Setup and enable mDNS
            string wifi_mdns_cmd("AT+MDNS=1,\"QMesh-");
            wifi_mdns_cmd.append(cfg.ssid);
            wifi_mdns_cmd.append("\",\"_iot\",8080\r\n");
            at_parser->send(safe_pcts(wifi_mdns_cmd).c_str(), wifi_mdns_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
            // Connect to the AP
            string wifi_conn_ap_cmd("AT+CWJAP=");
            wifi_conn_ap_cmd.append(cfg.ssid);
            wifi_conn_ap_cmd.append(",");
            wifi_conn_ap_cmd.append(cfg.password);
            wifi_conn_ap_cmd.append("\r\n");
            at_parser->send(safe_pcts(wifi_conn_ap_cmd).c_str(), wifi_conn_ap_cmd.c_str());
            PORTABLE_ASSERT(at_parser->recv("OK"));
        }
        idle_time = static_cast<int32_t>(time(nullptr));
    }
}


static constexpr int MAX_IDLE_TIME_SEC = 5;
auto ESP32PseudoSerial::getc() -> int {
    while(recv_data.empty()) {
        // Send out dummy characters to prevent ESP32 TCP server from timing out
        if((static_cast<int32_t>(time(nullptr)) - idle_time) > MAX_IDLE_TIME_SEC) {
            putc(KISS_FEND);
            idle_time = time(nullptr);
        }
        int conn_num = -1;
        int num_bytes = -1;
        vector<uint8_t> buf(MAX_RECV_BYTES);
        bool rx_success = false;
        if(cfg.isBT) {
            ser_mtx.lock();
            at_parser->set_timeout(RECV_TIMEOUT_RX_MS);
            string at_fmt_str("+BTDATA:%d,%");
            at_fmt_str.append(to_string(MAX_RECV_BYTES));
            at_fmt_str.append("s");
            rx_success = at_parser->recv(at_fmt_str.c_str(), &num_bytes, buf.data());
            ser_mtx.unlock();
        } else {
            ser_mtx.lock();
            at_parser->set_timeout(RECV_TIMEOUT_RX_MS);
            string at_fmt_str("+IPD,%d,%d:");
            rx_success = at_parser->recv(at_fmt_str.c_str(), &conn_num, &num_bytes, buf.data());
            if(rx_success) {
                for(int i = 0; i < num_bytes; i++) {
                    buf[i] = at_parser->getc();
                }
            }
            ser_mtx.unlock();
            if(rx_success) {
                tcp_conns.insert(pair<int, bool>(conn_num, true));
            }
        }
        if(rx_success) {
            PORTABLE_ASSERT(num_bytes < MAX_RECV_BYTES);
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
    return putc(val, false);
}


auto ESP32PseudoSerial::putc(const int val, bool dummy_char) -> int {
    if(dummy_char && outbuf.empty()) {
        outbuf.push_back(val);
    } else if(dummy_char && !outbuf.empty()) {
        // Don't queue up the character, as another thread is loading a 
        //  KISS frame
    } else {
        outbuf.push_back(val);
    }
    if(val == KISS_FEND || outbuf.size() >= RX_BUF_SIZE) {
        PORTABLE_ASSERT(at_parser != nullptr);
        idle_time = static_cast<int32_t>(time(nullptr));
        if(cfg.isBT) {
            at_parser->abort();
            ser_mtx.lock();
            at_parser->set_timeout(0);
            int num_bytes = -1;
            vector<uint8_t> buf(MAX_RECV_BYTES);
            string btdata_fmt_str("+BTDATA:%8d,%");
            btdata_fmt_str.append(to_string(MAX_RECV_BYTES));
            btdata_fmt_str.append("s");
            bool recv_success = at_parser->recv(btdata_fmt_str.c_str(), &num_bytes, buf.data());
            if(recv_success) {
                PORTABLE_ASSERT(num_bytes > MAX_RECV_BYTES);
                recv_data_mtx.lock();
                copy(buf.begin(), buf.begin()+num_bytes, back_inserter(recv_data));
                recv_data_mtx.unlock();
            }
            at_parser->set_timeout(RECV_TIMEOUT_MS);
            // See if there's an established BT connection
            //PORTABLE_ASSERT(at_parser->send("AT+BTSPPCON?"));
            //string bt_recv_str("%");
            //bt_recv_str.append(to_string(outbuf.size()));
            //bt_recv_str.append("s");
            //PORTABLE_ASSERT(at_parser->recv(bt_recv_str.c_str(), outbuf.data()));
            //printf("sending outbuf\r\n");
            at_parser->write(reinterpret_cast<char *>(outbuf.data()), outbuf.size());
            /*
            if(strncmp("BTSPPCON", reinterpret_cast<char *>(outbuf.data()), outbuf.size()) == 0) {
                PORTABLE_ASSERT(at_parser->recv("OK"));
                bool comms_success = at_parser->send("AT+BTSPPSEND=0,%d", outbuf.size()) &&
                    at_parser->recv(">") &&
                    (at_parser->write(reinterpret_cast<char *>(outbuf.data()), outbuf.size()) != -1) &&
                    at_parser->recv("OK");
                if(!comms_success) {
                    printf("Failed to send to ESP32 on BT\r\n");
                }
            } else {
                PORTABLE_ASSERT(at_parser->recv("ERROR"));
            }
            */
            ser_mtx.unlock();
        } else {
            at_parser->abort();
            ser_mtx.lock();
            at_parser->set_timeout(0);
            int num_bytes = -1;
            int conn_num = -1;
            vector<uint8_t> buf(MAX_RECV_BYTES);
            string ipd_fmt_str("+IPD,%d,%d:%");
            ipd_fmt_str.append(to_string(MAX_RECV_BYTES));
            ipd_fmt_str.append("s");
            bool recv_success = at_parser->recv(ipd_fmt_str.c_str(), &conn_num, &num_bytes, buf.data());
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
            vector<int> link_ids;
            constexpr int RECV_STR_LEN = 128;
            vector<char> recv_char(RECV_STR_LEN);
            link_ids.clear();
            string cipstate_str("+CIPSTATE:%d,%");
            cipstate_str.append(to_string(RECV_STR_LEN));
            cipstate_str.append("s");
            for(;;) {
                if(!at_parser->recv(cipstate_str.c_str(), &link_id, recv_char.data())) {
                    break;
                } 
                link_ids.push_back(link_id);
            }
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
                PORTABLE_ASSERT(rx_bytes == static_cast<int>(outbuf.size()));
                if(!comms_success) {
                    printf("Failed to send to ESP32 on WiFi connection %d\r\n", link_id);
                }
            } 
            ser_mtx.unlock();
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