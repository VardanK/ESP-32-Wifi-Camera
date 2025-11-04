# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(app_sources
    "${CMAKE_CURRENT_LIST_DIR}/main.c"
    "${CMAKE_CURRENT_LIST_DIR}/wifi_manager.c"
    "${CMAKE_CURRENT_LIST_DIR}/http_server_app.c"
    "${CMAKE_CURRENT_LIST_DIR}/camera_manager.c"
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_SOURCE_DIR}/include"
    REQUIRES nvs_flash esp_wifi esp_event esp_netif esp_http_server esp_timer esp32-camera driver
)
