/**
 * @file
 * @brief   Definition of system interface functions required by the Bosch BME680 driver code.
 * @author  Markku Kolkka <markku.kolkka@iki.fi>
 * 
 */

#include "util.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

#define ACK_CHECK_EN true                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS false                       /*!< I2C master will not check ack from slave */

static const char *TAG = "I2C utilities";

/**
 * Wait for a specified number of milliseconds.
 * @param period    the length of delay in milliseconds
 */
void user_delay_ms(uint32_t period)
{
    vTaskDelay(pdMS_TO_TICKS(period));
}

/**
 * Read a block of data from an I2C device.
 * @param dev_id    I2C address of device
 * @param reg_addr  register number where read starts
 * @param reg_data  pointer to buffer where read data is stored
 * @param len       number of bytes to read
 * @return Zero for success, -1 for error
 */
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    if (len == 0) {
        return 0;
    }
    esp_err_t rslt = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, reg_data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    rslt = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    if (rslt != ESP_OK) {
        ESP_LOGE(TAG, "Error on I2C read: (%x) %s ", rslt, esp_err_to_name(rslt));
    }
    i2c_cmd_link_delete(cmd);
    return rslt;
}

/**
 * Write a block of data to an I2C device.
 * @param dev_id    I2C address of device
 * @param reg_addr  register number where write starts
 * @param reg_data  pointer to buffer where data is stored
 * @param len       number of bytes to write
 * @return Zero for success, -1 for error
 */
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    esp_err_t rslt = ESP_OK;
   
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    rslt = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    if (rslt != ESP_OK) {
        ESP_LOGE(TAG, "Error on I2C write: (%x) %s ", rslt, esp_err_to_name(rslt));
    }
    i2c_cmd_link_delete(cmd);
    return rslt;
}