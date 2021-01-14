#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bme680.h"
#include "util.h"

/* I2C pins on Adafuit Huzzah32 a.k.a. ESP32 Feather
 * TODO: put these in a Kconfig file
 */
#define SDA_PIN 23
#define SCL_PIN 22

/* tag for log output */
static const char *TAG = "BME680_logger";

/* global variables
 * keep large structs out of stack
 */
static struct bme680_dev sensor;
static struct bme680_field_data data;

/* setup function for I2C:
 * I2C controller 0, internal pullups enabled, 400kHz
 */
static esp_err_t i2c_init()
{
    const int port = I2C_NUM_0;
    i2c_config_t conf =
        {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = SDA_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = SCL_PIN,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000
        };
    i2c_param_config(port, &conf);
    return i2c_driver_install(port, conf.mode, 0, 0, 0);
}

void app_main()
{
    int8_t bme_rslt = BME680_OK;
    uint8_t bme_req_settings;
    uint16_t period;

    /* ESP32 initialization */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing I2C");
    ESP_ERROR_CHECK(i2c_init());

    /* BME 680 initialization */
    ESP_LOGI(TAG, "Initializing sensor");
    sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
    sensor.intf = BME680_I2C_INTF;
    sensor.read = user_i2c_read;
    sensor.write = user_i2c_write;
    sensor.delay_ms = user_delay_ms;
    sensor.amb_temp = 20;
    ESP_ERROR_CHECK((bme_rslt = bme680_init(&sensor)));

    ESP_LOGI(TAG, "Configuring sensor");
    sensor.tph_sett.os_hum = BME680_OS_2X;
    sensor.tph_sett.os_pres = BME680_OS_4X;
    sensor.tph_sett.os_temp = BME680_OS_8X;
    sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
    sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    sensor.gas_sett.heatr_temp = 320;
    sensor.gas_sett.heatr_dur = 150;
    sensor.power_mode = BME680_FORCED_MODE;
    bme_req_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;
    ESP_ERROR_CHECK((bme_rslt = bme680_set_sensor_settings(bme_req_settings, &sensor)));
    ESP_ERROR_CHECK((bme_rslt = bme680_set_sensor_mode(&sensor)));

    /* Main loop */
    ESP_LOGI(TAG, "Read sensor data");
    while (1) {
        /* Wait until measurement is ready */
        bme680_get_profile_dur(&period, &sensor);
        vTaskDelay(pdMS_TO_TICKS(period));

        /* Fetch and display measurement results*/
        ESP_ERROR_CHECK((bme_rslt = bme680_get_sensor_data(&data, &sensor)));
        printf("T: %.2f degC, P: %.2f hPa, H: %.2f %%rH ",
               data.temperature, data.pressure / 100.0f, data.humidity);
        /* Avoid using measurements from an unstable heating setup */
        if (data.status & BME680_GASM_VALID_MSK) {
            printf(", G: %f ohms", data.gas_resistance);
        }
        printf("\n");

        /* Run measurement once per five seconds */
        vTaskDelay(pdMS_TO_TICKS(5000-period));

        /* Trigger the next measurement  */
        if (sensor.power_mode == BME680_FORCED_MODE) {
            ESP_ERROR_CHECK((bme_rslt = bme680_set_sensor_mode(&sensor)));
        }
    }
}
