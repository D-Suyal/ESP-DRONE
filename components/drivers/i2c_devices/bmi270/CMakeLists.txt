idf_component_register(
    SRCS "bmi270_esp_if.c"
         "bosch/bmi2.c"
         "bosch/bmi270.c"
    INCLUDE_DIRS "include" "bosch"
    REQUIRES i2c_bus)
