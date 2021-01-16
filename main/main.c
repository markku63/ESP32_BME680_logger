#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sensor_task.h"

/* tag for log output */
static const char *TAG = "BME680_logger";

void app_main()
{
    TaskHandle_t task = NULL;
    /* ESP32 initialization */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Starting sensor task");
    xTaskCreatePinnedToCore(sensor_task, "sensor task", 4096, NULL, 3, &task, APP_CPU_NUM);
    /* Main task is now finished*/
    ESP_LOGI(TAG, "Killing main task");
    vTaskDelete(NULL);
}
