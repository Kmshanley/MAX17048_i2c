#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <i2cdev.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define MAX17048_I2C_ADDR 0x36

#define MAX17048_VCELL 0x02
#define MAX17048_SOC 0x04
#define MAX17048_MODE 0x06
#define MAX17048_VERSION 0x08
#define MAX17048_HIBRT 0x0A
#define MAX17048_CONFIG 0x0C
#define MAX17048_VALRT 0x14
#define MAX17048_CRATE 0x16
#define MAX17048_VRESET_ID 0x18
#define MAX17048_STATUS 0x1A
#define MAX17048_TABLE 0x40
#define MAX17048_CMD 0xFE

//Max i2c clock of MAX17048 is 400kHz
#define I2C_FREQ_HZ 1000000

typedef enum 
{
    RI = (1 << 0),  // Reset indicator
    VH = (1 << 1),  // Voltage high alert
    VL = (1 << 2),  // Voltage low alert
    VR = (1 << 3),  // Voltage reset alert
    HD = (1 << 4),  // SOC low alert
    SC = (1 << 5)   // SOC change alert
} MAX17048_ALERT_t;

typedef struct
{
    i2c_dev_t i2c_dev;              //!< I2C device descriptor
} MAX17048_t;

esp_err_t MAX17048_init_desc(MAX17048_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
uint16_t MAX17048_get_adc(MAX17048_t *dev);
float MAX17048_get_voltage(MAX17048_t *dev);
float MAX17048_get_soc(MAX17048_t *dev);
uint16_t MAX17048_get_mode(MAX17048_t *dev);
uint16_t MAX17048_get_version(MAX17048_t *dev);
float MAX17048_get_crate(MAX17048_t *dev);
void MAX17048_tempCompensate(MAX17048_t *dev, float temp);

/**
 * @brief Issues a reset command to the MAX17048. Causes the device to completely reset as if power had been removed.
 * @param dev Device descriptor
 */
void MAX17048_reset(MAX17048_t *dev);

/**
 * @brief Returns an 8-bit read-only value that is one-time programmable at the factory, 
 * which can be used as an identifier to distinguish multiple cell types in production.
 * @param dev Device descriptor
 * @return 8-bit unique ID
 */
uint8_t MAX17048_get_id(MAX17048_t *dev);

/**
 * @brief Configures the VCELL range outside of which alerts are generated.
 * 
 * [ 1 LSb = 20mv ]
 * The IC alerts while VCELL > VALRT.MAX or VCELL < VALRT.MIN 
 * 
 * @param dev Device descriptor
 * @param min Minimum threshold voltage
 * @param max Maximum threshold voltage
 */
void MAX17048_set_valrt(MAX17048_t *dev, uint8_t min, uint8_t max);

/**
 * @brief 
 * @param dev Device descriptor
 */
void MAX17048_clear_alert(MAX17048_t *dev);

/**
 * @brief
 * @param threshold 
 * @param dev Device descriptor
 */
void MAX17048_set_emptyAlertThreshold(MAX17048_t *dev, uint8_t threshold);