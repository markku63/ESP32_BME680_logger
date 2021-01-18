#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bme680.h"
#include "util.h"
#include "bsec_interface.h"
#include "bsec_serialized_configurations_iaq.h"

/* tag for log output */
static const char *TAG = "Sensor task";

/* ESP-IDF non-volatile storage location */ 
static const char *MY_NVS_NAMESPACE = "bme680_logger";
static const char *MY_NVS_KEY = "bsec_status";

/* BSEC setup data */
#define REQ_SENSORS (4)
static const bsec_sensor_configuration_t requested_virtual_sensors[REQ_SENSORS] = {
  {.sensor_id = BSEC_OUTPUT_IAQ, .sample_rate = BSEC_SAMPLE_RATE_LP},
  {.sensor_id = BSEC_OUTPUT_RAW_PRESSURE, .sample_rate = BSEC_SAMPLE_RATE_LP},
  {.sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY, .sample_rate = BSEC_SAMPLE_RATE_LP},
  {.sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE, .sample_rate = BSEC_SAMPLE_RATE_LP}
};

/* global variables
 * keep large structs out of stack
 */
static struct bme680_dev sensor;
static struct bme680_field_data data;
static uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
static uint8_t serialized_settings[BSEC_MAX_PROPERTY_BLOB_SIZE];

static bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];

static void load_status() {
  size_t len = BSEC_MAX_PROPERTY_BLOB_SIZE;
  bsec_library_return_t bsec_res;
  nvs_handle_t nvs;
  esp_err_t res;

  res = nvs_open(MY_NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (res != ESP_OK) {
    if (res != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGE(TAG, "Error while opening NVS for reading: %s (%x)", esp_err_to_name(res), res);
    } else {
      ESP_LOGW(TAG, "Stored state not found");
    }
    return;
  }

  res = nvs_get_blob(nvs, MY_NVS_KEY, serialized_settings, &len);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "NVS error %s (%x) while loading stored state", esp_err_to_name(res), res);
    nvs_close(nvs);
    return;
  }

  bsec_res = bsec_set_state(serialized_settings, len, work_buffer, BSEC_MAX_WORKBUFFER_SIZE);
  if (bsec_res == BSEC_OK) {
    ESP_LOGI(TAG, "Stored state restored succesfully");
  } else {
    ESP_LOGE(TAG, "BSEC error %d while restoring stored state", bsec_res);
  }
  nvs_close(nvs);
}

static void store_status() {
  size_t len = 0;
  bsec_library_return_t bsec_res;
  nvs_handle_t nvs;
  esp_err_t res;

  res = nvs_open(MY_NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Error while opening NVS for writing: %s (%x)", esp_err_to_name(res), res);
    return;
  }

  bsec_res = bsec_get_state(0, serialized_settings, BSEC_MAX_PROPERTY_BLOB_SIZE, work_buffer, BSEC_MAX_WORKBUFFER_SIZE, &len);
  if (bsec_res != BSEC_OK) {
    ESP_LOGE(TAG, "BSEC error %d while getting state for storing", bsec_res);
    nvs_close(nvs);
    return;
  }

  res = nvs_set_blob(nvs, MY_NVS_KEY, serialized_settings, len);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "NVS error %s (%x) while writing state", esp_err_to_name(res), res);
    nvs_close(nvs);
    return;
  }

  res = nvs_commit(nvs);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "NVS error %s (%x) while committing", esp_err_to_name(res), res);
  } else {
    ESP_LOGI(TAG, "BSEC state stored successfully");
  }
  nvs_close(nvs);
}

void sensor_task(void * param) {
    int8_t bme_rslt = BME680_OK;
    uint8_t bme_req_settings;
    uint16_t period;
    bsec_version_t version;
    bsec_library_return_t bsec_res;
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

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

    /* BSEC library initialization */
    ESP_LOGI(TAG, "Initializing BSEC");
    ESP_ERROR_CHECK(bsec_get_version(&version));
    ESP_LOGI(TAG, "BSEC Library version %d.%d.%d.%d", version.major, version.minor, version.major_bugfix, version.minor_bugfix);
    ESP_ERROR_CHECK(bsec_init());

    ESP_ERROR_CHECK(bsec_set_configuration(bsec_config_iaq, sizeof(bsec_config_iaq), work_buffer, BSEC_MAX_WORKBUFFER_SIZE));
    load_status();

    bsec_res = bsec_update_subscription(requested_virtual_sensors, REQ_SENSORS, required_sensor_settings, &n_required_sensor_settings);
    if (bsec_res != BSEC_OK) {
      ESP_LOGE(TAG, "Failed to update BSEC subscription, error: %d", bsec_res);
      ESP_ERROR_CHECK(ESP_FAIL);
    }
    
    /* Main loop */
    ESP_LOGI(TAG, "Start reading sensor data");
    while (1) {
      int64_t timestamp;
      int64_t sleep_interval;
      bsec_bme_settings_t sensor_settings;
      uint8_t num_bsec_inputs;
      uint8_t num_bsec_outputs;
      bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
      bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];

      timestamp = esp_timer_get_time() * 1000LL;
      bsec_res = bsec_sensor_control(timestamp, &sensor_settings);

      if (sensor_settings.trigger_measurement) {
        sensor.tph_sett.os_hum = sensor_settings.humidity_oversampling;
        sensor.tph_sett.os_pres = sensor_settings.pressure_oversampling;
        sensor.tph_sett.os_temp = sensor_settings.temperature_oversampling;
        sensor.gas_sett.run_gas = sensor_settings.run_gas;
        sensor.gas_sett.heatr_temp = sensor_settings.heater_temperature;
        sensor.gas_sett.heatr_dur = sensor_settings.heating_duration;
        sensor.power_mode = BME680_FORCED_MODE;

        bme_req_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_GAS_SENSOR_SEL;
        bme_rslt = bme680_set_sensor_settings(bme_req_settings, &sensor);
        bme_rslt = bme680_set_sensor_mode(&sensor);

        /* Wait until measurement is ready */
        bme680_get_profile_dur(&period, &sensor);
        vTaskDelay(pdMS_TO_TICKS(period));
      }

      /* make sure the sensor is ready to be read */
      bme_rslt = bme680_get_sensor_mode(&sensor);
      while (sensor.power_mode == BME680_FORCED_MODE) {
        vTaskDelay(pdMS_TO_TICKS(5));
        bme_rslt = bme680_get_sensor_mode(&sensor);
      }

      num_bsec_inputs = 0;
      if (sensor_settings.process_data) {
        /* Fetch and process measurement results*/
        bme_rslt = bme680_get_sensor_data(&data, &sensor);
        if (data.status & BME680_NEW_DATA_MSK) {
          if (sensor_settings.process_data & BSEC_PROCESS_PRESSURE) {
            inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_PRESSURE;
            inputs[num_bsec_inputs].signal = data.pressure;
            inputs[num_bsec_inputs].time_stamp = timestamp;
            ++num_bsec_inputs;
          }
          if (sensor_settings.process_data & BSEC_PROCESS_TEMPERATURE) {
            inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
            inputs[num_bsec_inputs].signal = data.temperature;
            inputs[num_bsec_inputs].time_stamp = timestamp;
            ++num_bsec_inputs;
          }
          if (sensor_settings.process_data & BSEC_PROCESS_HUMIDITY) {
            inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
            inputs[num_bsec_inputs].signal = data.humidity;
            inputs[num_bsec_inputs].time_stamp = timestamp;
            ++num_bsec_inputs;
          }
          if (sensor_settings.process_data & BSEC_PROCESS_GAS) {
            if (data.status & BME680_GASM_VALID_MSK) {
              inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
              inputs[num_bsec_inputs].signal = data.gas_resistance;
              inputs[num_bsec_inputs].time_stamp = timestamp;
              ++num_bsec_inputs;
            }
          }
        }
      }

      if (num_bsec_inputs > 0) {
        float temperature = 0.0f;
        float pressure = 0.0f;
        float humidity = 0.0f;
        float iaq = 0.0f;

        num_bsec_outputs = BSEC_NUMBER_OUTPUTS;
        bsec_res = bsec_do_steps(inputs, num_bsec_inputs, outputs, &num_bsec_outputs);

        for (int i = 0; i < num_bsec_outputs; ++i) {
          switch(outputs[i].sensor_id) {
            case BSEC_OUTPUT_IAQ:
              iaq = outputs[i].signal;
              break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
              temperature = outputs[i].signal;
              break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
              humidity = outputs[i].signal;
              break;
            case BSEC_OUTPUT_RAW_PRESSURE:
              pressure = outputs[i].signal;
              break;
            default:
              break;
          }
        }
        printf("T: %.1f degC, P: %.1f hPa, H: %.1f %%rH ",
               temperature, pressure / 100.0f, humidity);
        printf(", IAQ: %.0f\n", iaq);
      }

      sleep_interval = (sensor_settings.next_call - esp_timer_get_time() * 1000LL) / 1000000LL;
      if (sleep_interval > 0) {
        vTaskDelay(pdMS_TO_TICKS(sleep_interval));
      }
    }
}