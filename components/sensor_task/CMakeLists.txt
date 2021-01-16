idf_component_register(SRCS "sensor_task.c"
                        INCLUDE_DIRS "."
                        PRIV_REQUIRES util BME680_driver)