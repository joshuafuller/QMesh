#include "mbed.h"
#include "QSPIFBlockDevice.h"
#include "LittleFileSystem.h"
#include "Adafruit_SSD1306.h"
#include "sha256.h"


static QSPIFBlockDevice bd(MBED_CONF_APP_QSPI_FLASH_IO0, MBED_CONF_APP_QSPI_FLASH_IO1,
                    MBED_CONF_APP_QSPI_FLASH_IO2, MBED_CONF_APP_QSPI_FLASH_IO3, 
                    MBED_CONF_APP_QSPI_FLASH_SCK, MBED_CONF_APP_QSPI_FLASH_CSN,
                    0, 40000000);
static LittleFileSystem fs("fs");
static FlashIAP flash;

extern Adafruit_SSD1306_I2c *oled;


bool check_for_update(void) {
    oled->printf("Checking for FW upd...\r\n");
    oled->display();
    int err = bd.init();
    err = fs.mount(&bd);
    if(err) {
        oled->printf("Error opening filesystem!\r\n");
        oled->display();
        ThisThread::sleep_for(1000);
        return false;
    }
    FILE *f = fopen("/fs/update.bin", "r");
    if(f) {
        oled->printf("Firmware update found!\r\n");
        oled->display();
        ThisThread::sleep_for(1000);
        oled->clearDisplay();
        FILE *sha256_file = fopen("/fs/update.bin.sha256", "r");
        if(!sha256_file) {
            oled->printf("No SHA256 hash file\r\n");
            ThisThread::sleep_for(1000);
            return false;
        }
        uint8_t sha256_checksum[32];
        int bytes_read = fread(sha256_checksum, 1, 32, sha256_file);
        fclose(sha256_file);
        if(bytes_read != 32) {
            oled->printf("Err reading hash file\r\n");
            ThisThread::sleep_for(1000);
            return false;
        }
        uint8_t sha256_checksum_comp[32];
        memset(sha256_checksum_comp, 0, 32);
        mbedtls_sha256_context sha256_cxt;
        mbedtls_sha256_init(&sha256_cxt);
        mbedtls_sha256_starts(&sha256_cxt, 0);
        while(true) {
            int read_int = fgetc(f);
            if(read_int == EOF) {
                break;
            }
            uint8_t read_byte = (uint8_t) read_int;
            mbedtls_sha256_update(&sha256_cxt, &read_byte, 1);
        }
        fclose(f);
        mbedtls_sha256_finish(&sha256_cxt, sha256_checksum_comp);
        if(memcmp(sha256_checksum, sha256_checksum_comp, 32)) {
            oled->printf("Hash check: FAIL\r\n");
            oled->display();
            ThisThread::sleep_for(1000);
            return false;
        }
        oled->printf("Hash check: SUCCESS\r\n");
        oled->display();
        ThisThread::sleep_for(1000);
        return true;
    } else {
        oled->printf("No firmware update found!\r\n");
        oled->display();
        ThisThread::sleep_for(1000);
        return false;
    }
}


void apply_update(void)
{
    uint32_t address = POST_APPLICATION_ADDR;
    FILE *file = fopen("/fs/update.bin", "r");
    fseek(file, 0, SEEK_END);
    long len = ftell(file);
    oled->printf("Firmware size is %ld bytes\r\n", len);
    oled->display();
    fseek(file, 0, SEEK_SET);

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
        int size_read = fread(page_buffer, 1, page_size, file);
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
            uint32_t percent_done_new = ftell(file) * 100 / len;
            if (percent_done != percent_done_new) {
                percent_done = percent_done_new;
                oled->printf("Flashed %3ld%%\r", (long) percent_done);
                oled->display();
            }
        }
    }
    oled->printf("Flashed 100%%\r\n");
    oled->display();

    delete[] page_buffer;

    flash.deinit();

    fs.remove("update.bin");
    fs.remove("update.bin.sha256");

    fs.unmount();

    bd.deinit();
}