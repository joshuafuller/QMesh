#include "mbed.h"
#include "QSPIFBlockDevice.h"
#include "LittleFileSystem.h"
#include "Adafruit_SSD1306.h"
#include "sha256.h"
#include <string>
#include "zlib.h"


constexpr int ONE_SECOND = 1000;
constexpr uint32_t QSPI_CLOCK_SPEED = 40000000;
constexpr int SHA256_CHECKSUM_SIZE = 32;

static QSPIFBlockDevice bd(MBED_CONF_APP_QSPI_FLASH_IO0, MBED_CONF_APP_QSPI_FLASH_IO1,
                    MBED_CONF_APP_QSPI_FLASH_IO2, MBED_CONF_APP_QSPI_FLASH_IO3, 
                    MBED_CONF_APP_QSPI_FLASH_SCK, MBED_CONF_APP_QSPI_FLASH_CSN,
                    0, QSPI_CLOCK_SPEED);
LittleFileSystem fs("fs");
static FlashIAP flash;
extern Adafruit_SSD1306_I2c *oled;


constexpr int DECOMP_BUF_SIZE = 1024;
static auto decompress_file(const string &in_path, const string &out_path) -> bool;
static auto decompress_file(const string &in_path, const string &out_path) -> bool {
    gzFile my_gz_file = gzopen(in_path.c_str(), "rb");
    if(my_gz_file == nullptr) {
        return false;
    }
    FILE *output_file = fopen(out_path.c_str(), "wb");
    if(output_file == nullptr) {
        return false;
    }
    array<uint8_t, DECOMP_BUF_SIZE> buf{};
    while(true) {
        int bytes_read = gzread(my_gz_file, buf.data(), buf.size());
        if(bytes_read == 0) { break; }
        fwrite(buf.data(), 1, bytes_read, output_file);
    }
    fclose(output_file);
    gzclose(my_gz_file);
    return true;
}


auto start_fs() -> bool {
    oled->display();
    int err = bd.init();
    if(err != 0) {
        oled->printf("Error opening block device!\r\n");
        oled->display();
        ThisThread::sleep_for(ONE_SECOND);
        return false;
    }
    err = fs.mount(&bd);
    if(err != 0) {
        oled->printf("Error opening filesystem!\r\n");
        oled->display();
        ThisThread::sleep_for(ONE_SECOND);
        return false;
    }
    return true;
}


auto check_for_update(const string &fname) -> bool {
    oled->printf("Checking for FW upd\r\n");
    string upd_path("/fs/");
    upd_path.append(fname);
    FILE *f = fopen(upd_path.c_str(), "r");
    if(f != nullptr) {
        oled->printf("Firmware update found!\r\n");
        oled->display();
        ThisThread::sleep_for(ONE_SECOND);
        oled->clearDisplay();
        upd_path.append(".sha256");
        FILE *sha256_file = fopen(upd_path.c_str(), "r");
        if(sha256_file == nullptr) {
            oled->printf("No SHA256 hash file\r\n");
            ThisThread::sleep_for(ONE_SECOND);
            fclose(sha256_file);
            fclose(f);
            return false;
        }
        vector<uint8_t> sha256_checksum(SHA256_CHECKSUM_SIZE);
        int bytes_read = fread(sha256_checksum.data(), 1, SHA256_CHECKSUM_SIZE, sha256_file);
        fclose(sha256_file);
        if(bytes_read != SHA256_CHECKSUM_SIZE) {
            oled->printf("Err reading hash file\r\n");
            ThisThread::sleep_for(ONE_SECOND);
            fclose(f);
            return false;
        }
        vector<uint8_t> sha256_checksum_comp(SHA256_CHECKSUM_SIZE);
        memset(sha256_checksum_comp.data(), 0, SHA256_CHECKSUM_SIZE);
        mbedtls_sha256_context sha256_cxt;
        mbedtls_sha256_init(&sha256_cxt);
        mbedtls_sha256_starts(&sha256_cxt, 0);
        while(true) {
            int read_int = fgetc(f);
            if(read_int == EOF) {
                break;
            }
            auto read_byte = static_cast<uint8_t>(read_int);
            mbedtls_sha256_update(&sha256_cxt, &read_byte, 1);
        }
        fclose(f);
        mbedtls_sha256_finish(&sha256_cxt, sha256_checksum_comp.data());
        if(sha256_checksum != sha256_checksum_comp) {
            oled->printf("Hash check: FAIL\r\n");
            oled->display();
            ThisThread::sleep_for(ONE_SECOND);
            return false;
        }
        oled->printf("Hash check: SUCCESS\r\n");
        oled->display();
        ThisThread::sleep_for(ONE_SECOND);
        return true;
    } 
    oled->printf("No firmware update found!\r\n");
    oled->display();
    ThisThread::sleep_for(ONE_SECOND);
    fclose(f);
    return false;
}


void apply_update(const string &fname, const bool golden)
{
    uint32_t address = POST_APPLICATION_ADDR;
    string upd_path("/fs/");
    upd_path.append(fname);
    gzFile file = gzopen(upd_path.c_str(), "rb");
    if(file == nullptr) {
        oled->clearDisplay();
        oled->printf("Failed to open update file\r\n");
        gzclose(file);
        return;
    }
    gzseek(file, 0, SEEK_END);
    int32_t len = gztell(file);
    oled->clearDisplay();
    oled->printf("FW size is %ld b\r\n", static_cast<long>(len));
    oled->display();
    gzseek(file, 0, SEEK_SET);

    const uint32_t page_size = flash.get_page_size();
    char *page_buffer = new char[page_size];
    uint32_t addr = address;
    uint32_t next_sector = addr + flash.get_sector_size(addr);
    bool sector_erased = false;
    size_t pages_flashed = 0;
    uint32_t percent_done = 0;
    while (true) {
        // Read data for this page
        memset(page_buffer, 0, sizeof(char) * page_size);
        int size_read = gzread(file, page_buffer, page_size);
        if (size_read <= 0) {
            break;
        }

        // Erase this page if it hasn't been erased
        if (!sector_erased) {
            flash.erase(addr, flash.get_sector_size(addr));
            sector_erased = true;
        }

        // Program page
        flash.program(page_buffer, addr, page_size);

        addr += page_size;
        if (addr >= next_sector) {
            next_sector = addr + flash.get_sector_size(addr);
            sector_erased = false;
        }

        if (++pages_flashed % 3 == 0) {
            constexpr int32_t PERCENT = 100;
            uint32_t percent_done_new = gztell(file) * PERCENT / len;
            if (percent_done != percent_done_new) {
                percent_done = percent_done_new;
                oled->printf("Flashed %3ld%%\r", static_cast<long>(percent_done));
                oled->display();
            }
        }
    }
    oled->printf("Update complete!\r\n");
    oled->display();
    ThisThread::sleep_for(ONE_SECOND);

    delete[] page_buffer;

    flash.deinit();

    if(!golden) {
        string fname_rem(fname);
        fs.remove(fname_rem.c_str());
        fname_rem.append(".sha256");
        fs.remove(fname_rem.c_str());
    }

    gzclose(file);
    int err = Z_OK;
    string errstr(gzerror(file, &err));
    if(err != Z_OK) {
        oled->clearDisplay();
        oled->printf("GZIP Error: %s\r\n", errstr.c_str());
        oled->display();
    }
    fs.unmount();
    bd.deinit();
}