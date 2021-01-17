idf_component_register(SRCS "sensor_task.c"
                        INCLUDE_DIRS "."
                        PRIV_REQUIRES nvs_flash util BME680_driver BSEC BSEC_config)