#ifndef I2C_HPP
#define I2C_HPP

#define ESP_IDF

#if defined(MBED_OS)
#include "mbed.h"
class SoftI2C;
using PORTABLE_I2C = SoftI2C;

#elif defined(ESP_IDF)
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

int i2c_master_port = 0;
i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,         // select GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,         // select GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,  // select frequency specific to your project
    // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
};

class PORTABLE_I2C {
private:
    int i2c_master_port;
    i2c_config_t cfg;
public:
    PORTABLE_I2C(PinName scl, PinName sda, PinName sclk, PinName ssel) {     
        i2c_master_port = I2C_MASTER_NUM;

        cfg.mode = I2C_MODE_MASTER,
        cfg.sda_io_num = I2C_MASTER_SDA_IO,
        cfg.scl_io_num = I2C_MASTER_SCL_IO,
        cfg.sda_pullup_en = GPIO_PULLUP_ENABLE,
        cfg.scl_pullup_en = GPIO_PULLUP_ENABLE,
        cfg.master.clk_speed = I2C_MASTER_FREQ_HZ,

        i2c_param_config(i2c_master_port, &conf);

        PORTABLE_ASSERT(i2c_driver_install(i2c_master_port, conf.mode, 
                                I2C_MASTER_RX_BUF_DISABLE, 
                                I2C_MASTER_TX_BUF_DISABLE, 0) == ESP_OK);
    }

    ~PORTABLE_I2C() {
        i2c_driver_delete(i2c_master_port);
    }
 
    void lock() { }

    void unlock() { }

    auto write(int address, const char *data, int length, bool repeated=false) -> int {
        PORTABLE_ASSERT(repeated == false);
        PORTABLE_ASSERT(length <= 256 && length > 0);
        PORTABLE_ASSERT(i2c_master_write_to_device(i2c_master_port, address, data, length, 
            I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS) == ESP_OK);
    }
};
#else
#error Need to define either MBED_OS or ESP_IDF
#endif


#endif /* I2C_HPP */