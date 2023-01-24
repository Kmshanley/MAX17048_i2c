#include "MAX17048_i2c.h"

static const char *TAG = "MAX17048 driver";

#define REV16_A(X) (((X) << 8) | ((X)>>8))

esp_err_t MAX17048_init_desc(MAX17048_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    if (addr != MAX17048_I2C_ADDR)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t MAX17048_get_adc(MAX17048_t *dev, uint16_t * adc) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17048_VCELL, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    data = REV16_A(data);
    *adc = data;
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t MAX17048_get_soc(MAX17048_t *dev, float * soc)
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17048_SOC, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    data = REV16_A(data);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *soc = (float)data / 256.f;
    return ESP_OK;
}

esp_err_t MAX17048_get_voltage(MAX17048_t *dev, float * volts) 
{
    uint16_t adc = 0;
    if (MAX17048_get_adc(dev, &adc) != ESP_OK) {
        return ESP_FAIL;
    }
    *volts = (float)(adc) * 78.125f / 1000000.f;
    return ESP_OK;
}

uint16_t MAX17048_get_mode(MAX17048_t *dev) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_read_reg(&dev->i2c_dev, MAX17048_MODE, &data, 2);
    data = REV16_A(data);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return data;
}

uint16_t MAX17048_get_version(MAX17048_t *dev)
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_read_reg(&dev->i2c_dev, MAX17048_VERSION, &data, 2);
    data = REV16_A(data);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return data;
}

esp_err_t MAX17048_get_crate(MAX17048_t *dev, float * crate)
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17048_CRATE, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    data = REV16_A(data);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *crate = (float)data * 0.208f;
    return ESP_OK;
}

void MAX17048_tempCompensate(MAX17048_t *dev, float temp)
{
    uint8_t v = 0;
    if (temp > 20.0) v = 0x97 + (temp - 20.0) * -0.5;
    else             v = 0x97 + (temp - 20.0) * -5.0;
    
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    i2c_dev_read_reg(&dev->i2c_dev, MAX17048_CONFIG, &data, 2);
    data = REV16_A(data);
    data |= (v << 8);
    i2c_dev_write_reg(&dev->i2c_dev, MAX17048_CONFIG, &data, 2);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}

void MAX17048_reset(MAX17048_t *dev) 
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint16_t rstcmd = 0x5400;
    i2c_dev_write_reg(&dev->i2c_dev, MAX17048_CMD, &rstcmd, 2);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}

uint8_t MAX17048_get_id(MAX17048_t *dev)
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_read_reg(&dev->i2c_dev, MAX17048_VRESET_ID, &data, 2);
    data = REV16_A(data);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return (uint8_t)data;
}

