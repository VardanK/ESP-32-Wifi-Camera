#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_camera.h"

esp_err_t camera_manager_init(void);
esp_err_t camera_manager_capture(httpd_req_t *req);
esp_err_t camera_manager_control(const char *var, int value);
bool camera_manager_is_initialized(void);
bool camera_manager_is_ready(void);
esp_err_t camera_manager_last_error(void);
const char *camera_manager_status_message(void);
bool camera_manager_is_low_mem_mode(void);
bool camera_manager_psram_detected(void);
framesize_t camera_manager_current_framesize(void);
void camera_manager_get_frame_dimensions(uint16_t *width, uint16_t *height);
esp_err_t camera_manager_set_flash_enabled(bool enabled);
bool camera_manager_is_flash_enabled(void);
bool camera_manager_is_flash_supported(void);
esp_err_t camera_manager_set_flash_brightness(int percent);
int camera_manager_flash_brightness(void);
