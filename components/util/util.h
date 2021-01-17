#ifndef BME680_COMPONENTS_UTIL_H
#define BME680_COMPONENTS_UTIL_H
#include <stdint.h>
#include "esp_err.h"  /* for esp_err_t */

void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
esp_err_t i2c_init(void);
#endif