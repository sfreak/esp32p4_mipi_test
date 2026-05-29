idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES esp_mm esp_driver_isp esp_driver_cam esp_driver_i2c soc)
