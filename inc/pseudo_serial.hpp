#ifndef PSEUDO_SERIAL_HPP
#define PSEUDO_SERIAL_HPP

#include "mbed.h"
#include "peripherals.hpp"
#include "serial_data.hpp"
#include <string>
#include <utility>
#include <fstream>
#include "Adafruit_SSD1306.h"
#include "mesh_protocol.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "serial_msg.hpp"
#if MBED_CONF_APP_HAS_BLE == 1
#include "ble/BLE.h"
#endif /* MBED_CONF_APP_HAS_BLE == 1 */


class PseudoSerial {
public:
    virtual auto putc(int val) -> int = 0;
    virtual auto getc() -> int = 0;
};


class UARTPseudoSerial : public PseudoSerial {
private:
    FILE *f_rd, *f_wr;
public:

    explicit UARTPseudoSerial(UARTSerial &ser, const bool read) {
        f_rd = nullptr;
        f_wr = nullptr;
        if(read) {
            f_rd = fdopen(&ser, "r");
        } else {
            f_wr = fdopen(&ser, "w");
        }
    }

     auto putc(const int val) -> int override {
        MBED_ASSERT(f_wr != nullptr);
        return fputc(val, f_wr);
    }

     auto getc() -> int override {
        MBED_ASSERT(f_rd != nullptr);
        return fgetc(f_rd);
    }
};


class FilePseudoSerial : public PseudoSerial {
private:
    FILE *f_rd, *f_wr;
public:

    explicit FilePseudoSerial(FILE *f) {
        f_rd = f;
        f_wr = f;
    }

     auto putc(const int val) -> int override {
        return fputc(val, f_wr);
    }

     auto getc() -> int override {
        return fgetc(f_rd);
    }
};


#endif /* PSEUDO_SERIAL_HPP */