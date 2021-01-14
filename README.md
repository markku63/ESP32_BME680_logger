# ESP32_BME680_logger
Use ESP32 microcontroller to read and process data from a [BME680](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors-bme680/) sensor

Uses ESP-IDF v. 4, not Arduino

Includes BME680 driver from https://github.com/BoschSensortec/BME680_driver

## Usage
### Prerequisites
You will need a BME680 sensor on a suitable breakout board, and an ESP32 development board. This code is tested on [Adafruit HUZZAH32](https://www.adafruit.com/product/3405) and [Pimoroni BME680 breakout](https://shop.pimoroni.com/products/bme680-breakout) but any other board combination should work as well. Change the I2C pin numbers and device address in the code to match your configuration.

Setup ESP-IDF version 4.1 or later according to the [official instructions](https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/get-started/index.html#installation-step-by-step)

### Compile and run
Copy or clone this code. Use `idf.py -p $PORT flash monitor` from the command line, or use Visual Studio Code with the Espressif IDF extension to compile and flash the code and monitor the output. 

## TODO
- use Kconfig file(s) to set things like pin numbers
- use [BSEC library](https://www.bosch-sensortec.com/software-tools/software/bsec/) to further process the sensor data
- use MQTT (or other protocol) to publish processed data  
