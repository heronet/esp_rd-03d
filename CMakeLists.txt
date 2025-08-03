# Radar Sensor Component CMakeLists.txt

idf_component_register(
    SRCS "esp_rd-03d.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver
        esp_common
        log
)