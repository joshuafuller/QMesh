#include "AFSK.h"
#include "AX25.h"
#include <string>
#include "serial_data.hpp"

Afsk modem;
AX25Ctx AX25;
static void aprs_msg_callback(struct AX25Msg *msg) {};
#define countof(a) sizeof(a)/sizeof(a[0])

bool LibAPRS_open_squelch = false;

unsigned long custom_preamble = 350UL;
unsigned long custom_tail = 50UL;

AX25Call src;
AX25Call dst;
AX25Call path1;
AX25Call path2;

char CALL[7] = "NOCALL";
int CALL_SSID = 0;
char DST[7] = "APZMDM";
int DST_SSID = 0;
char PATH1[7] = "WIDE1";
int PATH1_SSID = 1;
char PATH2[7] = "WIDE2";
int PATH2_SSID = 2;

AX25Call path[4];

// Location packet assembly fields
char latitude[9];
char longtitude[10];
char symbolTable = '/';
char symbol = 'n';

uint8_t power = 10;
uint8_t height = 10;
uint8_t gain = 10;
uint8_t directivity = 10;
/////////////////////////

// Message packet assembly fields
char message_recip[7];
int message_recip_ssid = -1;

int message_seq = 0;
char lastMessage[67];
size_t lastMessageLen;
bool message_autoAck = false;
/////////////////////////


void APRS_init(int reference, bool open_squelch) {
    LibAPRS_open_squelch = open_squelch;
    ax25_init(&AX25, aprs_msg_callback);
}


void APRS_setCallsign(char *call, int ssid) {
    memset(CALL, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        CALL[i] = call[i];
        i++;
    }
    CALL_SSID = ssid;
}


void APRS_setDestination(char *call, int ssid) {
    memset(DST, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        DST[i] = call[i];
        i++;
    }
    DST_SSID = ssid;
}


void APRS_setPath1(char *call, int ssid) {
    memset(PATH1, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        PATH1[i] = call[i];
        i++;
    }
    PATH1_SSID = ssid;
}


void APRS_setPath2(char *call, int ssid) {
    memset(PATH2, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        PATH2[i] = call[i];
        i++;
    }
    PATH2_SSID = ssid;
}


void APRS_setMessageDestination(char *call, int ssid) {
    memset(message_recip, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        message_recip[i] = call[i];
        i++;
    }
    message_recip_ssid = ssid;
}


void APRS_setPreamble(unsigned long pre) {
    custom_preamble = pre;
}


void APRS_setTail(unsigned long tail) {
    custom_tail = tail;
}


void APRS_useAlternateSymbolTable(bool use) {
    if (use) {
        symbolTable = '\\';
    } else {
        symbolTable = '/';
    }
}


void APRS_setSymbol(char sym) {
    symbol = sym;
}


void APRS_setLat(char *lat) {
    memset(latitude, 0, 9);
    int i = 0;
    while (i < 8 && lat[i] != 0) {
        latitude[i] = lat[i];
        i++;
    }
}


void APRS_setLon(char *lon) {
    memset(longtitude, 0, 10);
    int i = 0;
    while (i < 9 && lon[i] != 0) {
        longtitude[i] = lon[i];
        i++;
    }
}


void APRS_setPower(int s) {
    if (s >= 0 && s < 10) {
        power = s;
    }
}


void APRS_setHeight(int s) {
    if (s >= 0 && s < 10) {
        height = s;
    }
}


void APRS_setGain(int s) {
    if (s >= 0 && s < 10) {
        gain = s;
    }
}


void APRS_setDirectivity(int s) {
    if (s >= 0 && s < 10) {
        directivity = s;
    }
}


void APRS_printSettings() {
    debug_printf(DBG_INFO, "LibAPRS Settings:\r\n");
    debug_printf(DBG_INFO, "Callsign: %s-%d\r\n", CALL, CALL_SSID);
    debug_printf(DBG_INFO, "Destination: %s-%d\r\n", DST, DST_SSID);
    debug_printf(DBG_INFO, "Path1: %s-%d\r\n", PATH1, PATH1_SSID);
    debug_printf(DBG_INFO, "Path2: %s-%d\r\n", PATH2, PATH2_SSID);
    string msg_recip_str;
    if (message_recip[0] == 0) {
        msg_recip_str = "N/A"; 
    } else { 
        msg_recip_str = message_recip;
    }
    debug_printf(DBG_INFO, "Message dst: %s-%d\r\n", msg_recip_str.c_str(), message_recip_ssid);
    debug_printf(DBG_INFO, "TX Preamble: 0x%8x\r\n", custom_preamble);
    debug_printf(DBG_INFO, "TX Tail:     0x%8x\r\n", custom_tail);
    string norm_alt_str;
    if (symbolTable == '/') {
        msg_recip_str = "Normal";  
    } else { 
        msg_recip_str = "Alternate";
    }
    debug_printf(DBG_INFO, "Symbol table: %s\r\n", msg_recip_str.c_str()); 
    debug_printf(DBG_INFO, "Symbol:       %c\r\n", symbol);
    string power_str;
    if (power < 10) { 
        power_str = to_string(power); 
    } else { 
        power_str = "N/A"; 
    }
    debug_printf(DBG_INFO, "Power:        %s\r\n", power_str.c_str());
    string height_str;
    if (height < 10) { 
        height_str = height;
    } else { 
        height_str = "N/A"; 
    }
    debug_printf(DBG_INFO, "Height:       %s\r\n", height_str.c_str());
    string gain_str;
    if (gain < 10) { 
        gain_str = gain;
    } else { 
        gain_str = "N/A"; 
    }
    debug_printf(DBG_INFO, "Gain:         %s\r\n", gain_str.c_str());
    string dir_str;
    if (directivity < 10) { 
        dir_str = directivity; 
    } else { 
        dir_str = "N/A"; 
    }
    debug_printf(DBG_INFO, "Directivity:  %s\r\n", dir_str.c_str()); 
    string lat_str;
    if (latitude[0] != 0) {
        lat_str = latitude;  
    } else { 
        lat_str = "N/A";
    }
    debug_printf(DBG_INFO, "Latitude:     %s\r\n", lat_str.c_str());
    string long_str; 
    if (longtitude[0] != 0) { 
        long_str = longtitude; 
    } else { 
        long_str = "N/A"; 
    }
    debug_printf(DBG_INFO, "Longitude:   %s\r\n", long_str.c_str()); 
}


void APRS_sendPkt(void *_buffer, size_t length) {

    uint8_t *buffer = (uint8_t *)_buffer;

    memcpy(dst.call, DST, 6);
    dst.ssid = DST_SSID;

    memcpy(src.call, CALL, 6);
    src.ssid = CALL_SSID;

    memcpy(path1.call, PATH1, 6);
    path1.ssid = PATH1_SSID;

    memcpy(path2.call, PATH2, 6);
    path2.ssid = PATH2_SSID;

    path[0] = dst;
    path[1] = src;
    path[2] = path1;
    path[3] = path2;

    ax25_sendVia(&AX25, path, countof(path), buffer, length);
}


// Dynamic RAM usage of this function is 30 bytes
void APRS_sendLoc(void *_buffer, size_t length) {
    size_t payloadLength = 20+length;
    bool usePHG = false;
    if (power < 10 && height < 10 && gain < 10 && directivity < 9) {
        usePHG = true;
        payloadLength += 7;
    }
    uint8_t *packet = (uint8_t*)malloc(payloadLength);
    uint8_t *ptr = packet;
    packet[0] = '=';
    packet[9] = symbolTable;
    packet[19] = symbol;
    ptr++;
    memcpy(ptr, latitude, 8);
    ptr += 9;
    memcpy(ptr, longtitude, 9);
    ptr += 10;
    if (usePHG) {
        packet[20] = 'P';
        packet[21] = 'H';
        packet[22] = 'G';
        packet[23] = power+48;
        packet[24] = height+48;
        packet[25] = gain+48;
        packet[26] = directivity+48;
        ptr+=7;
    }
    if (length > 0) {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
    }

    APRS_sendPkt(packet, payloadLength);
    free(packet);
}


// Dynamic RAM usage of this function is 18 bytes
void APRS_sendMsg(void *_buffer, size_t length) {
    if (length > 67) length = 67;
    size_t payloadLength = 11+length+4;

    uint8_t *packet = (uint8_t*)malloc(payloadLength);
    uint8_t *ptr = packet;
    packet[0] = ':';
    int callSize = 6;
    int count = 0;
    while (callSize--) {
        if (message_recip[count] != 0) {
            packet[1+count] = message_recip[count];
            count++;
        }
    }
    if (message_recip_ssid != -1) {
        packet[1+count] = '-'; count++;
        if (message_recip_ssid < 10) {
            packet[1+count] = message_recip_ssid+48; count++;
        } else {
            packet[1+count] = 49; count++;
            packet[1+count] = message_recip_ssid-10+48; count++;
        }
    }
    while (count < 9) {
        packet[1+count] = ' '; count++;
    }
    packet[1+count] = ':';
    ptr += 11;
    if (length > 0) {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
        memcpy(lastMessage, buffer, length);
        lastMessageLen = length;
    }

    message_seq++;
    if (message_seq > 999) message_seq = 0;

    packet[11+length] = '{';
    int n = message_seq % 10;
    int d = ((message_seq % 100) - n)/10;
    int h = (message_seq - d - n) / 100;

    packet[12+length] = h+48;
    packet[13+length] = d+48;
    packet[14+length] = n+48;
    
    APRS_sendPkt(packet, payloadLength);
    free(packet);
}


void APRS_msgRetry() {
    message_seq--;
    APRS_sendMsg(lastMessage, lastMessageLen);
}

