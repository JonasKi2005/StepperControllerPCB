file(GLOB SRC_FILES "src/*.cpp")

idf_component_register(SRCS "main.cpp" ${SRC_FILES}
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS "." "include"
                       REQUIRES esp_wifi esp_event esp_http_server nvs_flash esp_netif
                                esp_driver_gpio esp_driver_uart esp_driver_gptimer
                       )