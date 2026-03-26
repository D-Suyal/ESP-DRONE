idf_component_register(
    SRCS "bmp581_esp_if.c"
         "bosch/bmp5.c"
    INCLUDE_DIRS "include" "bosch"
    REQUIRES i2c_bus freertos)
