#include "camera_manager.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAMERA_MANAGER_TAG "camera_manager"

static const int CAMERA_XCLK_FREQ = 20000000;
#ifndef CAMERA_FLASH_GPIO
#define CAMERA_FLASH_GPIO GPIO_NUM_4
#endif
#ifndef CAMERA_FLASH_ON_LEVEL
#define CAMERA_FLASH_ON_LEVEL 1
#endif
#define CAMERA_FLASH_WARMUP_MS 80
#define CAMERA_FLASH_LEDC_MODE LEDC_LOW_SPEED_MODE
#define CAMERA_FLASH_LEDC_TIMER LEDC_TIMER_1
#define CAMERA_FLASH_LEDC_CHANNEL LEDC_CHANNEL_1
#define CAMERA_FLASH_LEDC_BITS LEDC_TIMER_10_BIT
#define CAMERA_FLASH_LEDC_FREQUENCY 5000

typedef enum {
    CAMERA_STATE_UNINITIALIZED = 0,
    CAMERA_STATE_READY,
    CAMERA_STATE_ERROR
} camera_internal_state_t;

static camera_internal_state_t s_camera_state = CAMERA_STATE_UNINITIALIZED;
static esp_err_t s_camera_last_err = ESP_OK;
static char s_camera_message[160] = "Camera not initialized.";
static bool s_camera_low_mem_mode = false;
static framesize_t s_camera_framesize = FRAMESIZE_INVALID;
static uint64_t s_last_frame_timestamp_us = 0;
static bool s_discard_next_frame = true;
static bool s_flash_supported = false;
static bool s_flash_enabled = false;
static bool s_flash_state = false;
static bool s_flash_ledc = false;
static uint8_t s_flash_brightness_percent = 100;
static uint32_t s_flash_ledc_max_duty = 0;
static uint16_t s_last_frame_width = 0;
static uint16_t s_last_frame_height = 0;

static void camera_manager_set_ready(const char *message);
static void camera_manager_set_error(esp_err_t err, const char *message);
static void camera_manager_set_ready_with_flash_info(const char *base_message);
static esp_err_t camera_apply_default_settings(void);
static size_t camera_manager_escape_json(const char *input, char *output, size_t output_len);
static esp_err_t camera_manager_send_error_response(httpd_req_t *req);
static void camera_apply_low_mem_profile(camera_config_t *config);
static uint64_t camera_frame_timestamp_us(const camera_fb_t *frame);
static void camera_flash_configure(void);
static uint32_t camera_flash_target_duty(void);
static bool camera_flash_should_emit(void);
static bool camera_flash_set(bool on);

esp_err_t camera_manager_init(void) {
    camera_config_t config = {
        .pin_pwdn = 32,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sscb_sda = 26,
        .pin_sscb_scl = 27,
        .pin_d7 = 35,
        .pin_d6 = 34,
        .pin_d5 = 39,
        .pin_d4 = 36,
        .pin_d3 = 21,
        .pin_d2 = 19,
        .pin_d1 = 18,
        .pin_d0 = 5,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,
        .xclk_freq_hz = CAMERA_XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_SVGA,
        .jpeg_quality = 12,
        .fb_count = 2,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

#if CONFIG_IDF_TARGET_ESP32
    config.fb_location = CAMERA_FB_IN_PSRAM;
#endif

    size_t psram_bytes = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    bool psram_available = psram_bytes > 0;

    bool low_mem_mode = false;

    if (!psram_available) {
        ESP_LOGW(CAMERA_MANAGER_TAG, "PSRAM not detected; using DRAM frame buffer and reduced frame size");
        camera_apply_low_mem_profile(&config);
        low_mem_mode = true;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK && psram_available) {
        ESP_LOGW(CAMERA_MANAGER_TAG, "Camera init failed (%s); retrying with reduced DRAM configuration",
                 esp_err_to_name(err));
        camera_apply_low_mem_profile(&config);
        low_mem_mode = true;
        err = esp_camera_init(&config);
    }
    if (err != ESP_OK) {
        ESP_LOGE(CAMERA_MANAGER_TAG, "Camera init failed: %s", esp_err_to_name(err));
        camera_manager_set_error(err, "Camera initialization failed. Verify the camera module wiring and power, then reset the device.");
        return err;
    }

    s_camera_framesize = config.frame_size;
    s_camera_low_mem_mode = low_mem_mode;
    s_last_frame_timestamp_us = 0;
    s_discard_next_frame = true;
    s_flash_enabled = false;
    s_flash_state = false;
    s_flash_supported = false;
    s_flash_ledc = false;
    s_flash_brightness_percent = 100;
    s_flash_ledc_max_duty = 0;
    s_last_frame_width = 0;
    s_last_frame_height = 0;

    camera_flash_configure();

    err = camera_apply_default_settings();
    if (err != ESP_OK) {
        camera_manager_set_error(err, "Camera sensor configuration failed. Try power cycling or reflashing the device.");
        return err;
    }

    const char *ready_message = s_camera_low_mem_mode
                                    ? "Camera ready. PSRAM fallback active; high resolutions above VGA are disabled."
                                    : "Camera ready.";
    camera_manager_set_ready_with_flash_info(ready_message);
    return ESP_OK;
}

static void camera_apply_low_mem_profile(camera_config_t *config) {
    config->frame_size = FRAMESIZE_VGA;
    config->fb_count = 1;
    config->jpeg_quality = 15;
#if CONFIG_IDF_TARGET_ESP32
    config->fb_location = CAMERA_FB_IN_DRAM;
#endif
}

static uint64_t camera_frame_timestamp_us(const camera_fb_t *frame) {
    if (!frame) {
        return 0;
    }
    uint64_t sec = (uint64_t)frame->timestamp.tv_sec;
    uint64_t usec = (uint64_t)frame->timestamp.tv_usec;
    return (sec * 1000000ULL) + usec;
}

static uint32_t camera_flash_target_duty(void) {
#if (CAMERA_FLASH_GPIO >= 0)
    if (!s_flash_supported) {
        return 0;
    }
    if (s_flash_ledc) {
        if (s_flash_ledc_max_duty == 0) {
            return 0;
        }
        return (s_flash_ledc_max_duty * s_flash_brightness_percent) / 100U;
    }
    return (s_flash_brightness_percent > 0) ? 1U : 0U;
#else
    return 0;
#endif
}

static bool camera_flash_should_emit(void) {
#if (CAMERA_FLASH_GPIO >= 0)
    if (!s_flash_supported || !s_flash_enabled) {
        return false;
    }
    if (s_flash_ledc) {
        return camera_flash_target_duty() > 0;
    }
    return s_flash_brightness_percent > 0;
#else
    return false;
#endif
}

static bool camera_flash_set(bool on) {
#if (CAMERA_FLASH_GPIO >= 0)
    if (!s_flash_supported) {
        return false;
    }

    bool emit = on && camera_flash_should_emit();

    if (s_flash_ledc) {
        uint32_t duty = emit ? camera_flash_target_duty() : 0;
        esp_err_t err = ledc_set_duty(CAMERA_FLASH_LEDC_MODE, CAMERA_FLASH_LEDC_CHANNEL, duty);
        if (err != ESP_OK) {
            ESP_LOGW(CAMERA_MANAGER_TAG, "Failed to set flash duty (%s)", esp_err_to_name(err));
            emit = false;
        } else {
            ledc_update_duty(CAMERA_FLASH_LEDC_MODE, CAMERA_FLASH_LEDC_CHANNEL);
        }
        s_flash_state = emit && duty > 0;
        return s_flash_state;
    }

    int off_level = CAMERA_FLASH_ON_LEVEL ? 0 : 1;
    int level = emit ? CAMERA_FLASH_ON_LEVEL : off_level;
    esp_err_t err = gpio_set_level(CAMERA_FLASH_GPIO, level);
    if (err != ESP_OK) {
        ESP_LOGW(CAMERA_MANAGER_TAG, "Failed to set flash level (%s)", esp_err_to_name(err));
        s_flash_state = false;
        return false;
    }
    s_flash_state = emit && (level == CAMERA_FLASH_ON_LEVEL);
    return s_flash_state;
#else
    (void)on;
    return false;
#endif
}

static void camera_flash_configure(void) {
#if (CAMERA_FLASH_GPIO >= 0)
    s_flash_supported = false;
    s_flash_ledc = false;
    s_flash_ledc_max_duty = 0;

    ledc_timer_config_t timer_config = {
        .speed_mode = CAMERA_FLASH_LEDC_MODE,
        .duty_resolution = CAMERA_FLASH_LEDC_BITS,
        .timer_num = CAMERA_FLASH_LEDC_TIMER,
        .freq_hz = CAMERA_FLASH_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&timer_config) == ESP_OK) {
        ledc_channel_config_t channel_config = {
            .speed_mode = CAMERA_FLASH_LEDC_MODE,
            .channel = CAMERA_FLASH_LEDC_CHANNEL,
            .timer_sel = CAMERA_FLASH_LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = CAMERA_FLASH_GPIO,
            .duty = 0,
            .hpoint = 0
        };
        if (ledc_channel_config(&channel_config) == ESP_OK) {
            s_flash_supported = true;
            s_flash_ledc = true;
            s_flash_ledc_max_duty = (1U << CAMERA_FLASH_LEDC_BITS) - 1U;
            camera_flash_set(false);
            ESP_LOGI(CAMERA_MANAGER_TAG, "Flashlight configured using LEDC on GPIO%d", CAMERA_FLASH_GPIO);
            return;
        } else {
            ESP_LOGW(CAMERA_MANAGER_TAG, "Failed to configure LEDC channel for flash; falling back to GPIO mode");
        }
    } else {
        ESP_LOGW(CAMERA_MANAGER_TAG, "Failed to configure LEDC timer for flash; falling back to GPIO mode");
    }

    gpio_config_t cfg = {
        .pin_bit_mask = ((uint64_t)1 << CAMERA_FLASH_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(CAMERA_MANAGER_TAG, "Failed to configure flash GPIO: %s", esp_err_to_name(err));
        s_flash_supported = false;
        return;
    }
    s_flash_supported = true;
    s_flash_ledc = false;
    s_flash_ledc_max_duty = 0;
    camera_flash_set(false);
    ESP_LOGI(CAMERA_MANAGER_TAG, "Flashlight GPIO configured on GPIO%d", CAMERA_FLASH_GPIO);
#else
    s_flash_supported = false;
    ESP_LOGI(CAMERA_MANAGER_TAG, "Flashlight control not configured; CAMERA_FLASH_GPIO disabled.");
#endif
}

esp_err_t camera_manager_capture(httpd_req_t *req) {
    bool flash_active = false;
#if (CAMERA_FLASH_GPIO >= 0)
    if (camera_flash_should_emit()) {
        flash_active = camera_flash_set(true);
        if (flash_active) {
            vTaskDelay(pdMS_TO_TICKS(CAMERA_FLASH_WARMUP_MS));
        }
    }
#endif
    int frames_to_skip = s_discard_next_frame ? 2 : 1;
    s_discard_next_frame = false;

    camera_fb_t *frame = NULL;
    for (int attempt = 0; attempt < 4; ++attempt) {
        frame = esp_camera_fb_get();
        if (!frame) {
            ESP_LOGE(CAMERA_MANAGER_TAG, "Failed to acquire camera frame");
            camera_manager_set_error(ESP_FAIL, "Unable to capture an image. Ensure the camera module is seated correctly and sufficient lighting/power is available, then try again.");
            if (flash_active) {
                camera_flash_set(false);
            }
            return camera_manager_send_error_response(req);
        }

        uint64_t timestamp_us = camera_frame_timestamp_us(frame);

        if (frames_to_skip > 0) {
            frames_to_skip--;
            s_last_frame_timestamp_us = timestamp_us;
            esp_camera_fb_return(frame);
            frame = NULL;
            continue;
        }

        if (s_last_frame_timestamp_us != 0 && timestamp_us <= s_last_frame_timestamp_us) {
            esp_camera_fb_return(frame);
            frame = NULL;
            continue;
        }

        s_last_frame_timestamp_us = timestamp_us;
        break;
    }

    if (!frame) {
        ESP_LOGE(CAMERA_MANAGER_TAG, "Exhausted capture attempts without fresh frame");
        camera_manager_set_error(ESP_FAIL, "Unable to capture a fresh image. Please try again.");
        if (flash_active) {
            camera_flash_set(false);
        }
        return camera_manager_send_error_response(req);
    }

    if (frame->width > 0 && frame->height > 0) {
        s_last_frame_width = frame->width;
        s_last_frame_height = frame->height;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t res = httpd_resp_send(req, (const char *)frame->buf, frame->len);
    esp_camera_fb_return(frame);

    if (res != ESP_OK) {
        ESP_LOGE(CAMERA_MANAGER_TAG, "Failed to send frame: %s", esp_err_to_name(res));
        camera_manager_set_error(res, "Image transmission failed. Check the Wi-Fi connection and retry.");
        if (flash_active) {
            camera_flash_set(false);
        }
        return res;
    }
    camera_manager_set_ready_with_flash_info("Capture completed.");
    if (flash_active) {
        camera_flash_set(false);
    }
    return res;
}

static sensor_t *camera_manager_sensor(void) {
    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGE(CAMERA_MANAGER_TAG, "Unable to obtain camera sensor handle");
    }
    return sensor;
}

static esp_err_t camera_apply_default_settings(void) {
    sensor_t *sensor = camera_manager_sensor();
    if (!sensor) {
        camera_manager_set_error(ESP_ERR_INVALID_STATE, "Camera sensor not detected. Confirm the camera module is connected to the ESP32-CAM board.");
        return ESP_ERR_INVALID_STATE;
    }

    if (sensor->id.PID == OV3660_PID) {
        sensor->set_vflip(sensor, 1);
        sensor->set_brightness(sensor, 1);
        sensor->set_saturation(sensor, -2);
    }
    framesize_t default_framesize = s_camera_low_mem_mode ? FRAMESIZE_VGA : FRAMESIZE_SVGA;
    sensor->set_framesize(sensor, default_framesize);
    s_camera_framesize = sensor->status.framesize;
    sensor->set_quality(sensor, s_camera_low_mem_mode ? 15 : 12);
    sensor->set_gainceiling(sensor, GAINCEILING_16X);
    return ESP_OK;
}

static int clamp(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

esp_err_t camera_manager_control(const char *var, int value) {
    sensor_t *sensor = camera_manager_sensor();
    if (!sensor || !var) {
        camera_manager_set_error(ESP_ERR_INVALID_STATE, "Camera sensor unavailable. Verify hardware connections and reset the device.");
        return ESP_ERR_INVALID_STATE;
    }

    if (strcmp(var, "flash") == 0) {
        return camera_manager_set_flash_enabled(value ? true : false);
    }

    if (strcmp(var, "flash_brightness") == 0) {
        return camera_manager_set_flash_brightness(value);
    }

    bool framesize_clamped = false;

    if (strcmp(var, "framesize") == 0) {
        int max_framesize = s_camera_low_mem_mode ? FRAMESIZE_VGA : FRAMESIZE_UXGA;
        int requested_value = value;
        int clamped_value = clamp(value, FRAMESIZE_96X96, max_framesize);
        framesize_t target_framesize = (framesize_t)clamped_value;
        if (requested_value != clamped_value) {
            ESP_LOGW(CAMERA_MANAGER_TAG, "Framesize %d out of bounds, clamped to %d", requested_value, clamped_value);
            framesize_clamped = true;
        }
        int set_result = sensor->set_framesize(sensor, target_framesize);
        if (set_result != 0) {
            ESP_LOGE(CAMERA_MANAGER_TAG, "Failed to set framesize %d (err=%d)", clamped_value, set_result);
            camera_manager_set_error(ESP_FAIL, "Unable to change resolution. Try a lower resolution or reboot the device.");
            return ESP_FAIL;
        }
        s_camera_framesize = sensor->status.framesize;
        s_last_frame_width = 0;
        s_last_frame_height = 0;
    } else if (strcmp(var, "quality") == 0) {
        int max_quality = s_camera_low_mem_mode ? 30 : 63;
        value = clamp(value, 10, max_quality);
        sensor->set_quality(sensor, value);
    } else if (strcmp(var, "flash") == 0) {
        esp_err_t flash_err = camera_manager_set_flash_enabled(value ? true : false);
        if (flash_err != ESP_OK) {
            return flash_err;
        }
    } else if (strcmp(var, "brightness") == 0) {
        value = clamp(value, -2, 2);
        sensor->set_brightness(sensor, value);
    } else if (strcmp(var, "contrast") == 0) {
        value = clamp(value, -2, 2);
        sensor->set_contrast(sensor, value);
    } else if (strcmp(var, "saturation") == 0) {
        value = clamp(value, -2, 2);
        sensor->set_saturation(sensor, value);
    } else if (strcmp(var, "sharpness") == 0) {
        value = clamp(value, -2, 2);
        sensor->set_sharpness(sensor, value);
    } else if (strcmp(var, "special_effect") == 0) {
        value = clamp(value, 0, 6);
        sensor->set_special_effect(sensor, value);
    } else if (strcmp(var, "awb") == 0) {
        sensor->set_whitebal(sensor, value ? 1 : 0);
    } else if (strcmp(var, "aec") == 0) {
        sensor->set_exposure_ctrl(sensor, value ? 1 : 0);
    } else if (strcmp(var, "agc") == 0) {
        sensor->set_gain_ctrl(sensor, value ? 1 : 0);
    } else if (strcmp(var, "hmirror") == 0) {
        sensor->set_hmirror(sensor, value ? 1 : 0);
    } else if (strcmp(var, "vflip") == 0) {
        sensor->set_vflip(sensor, value ? 1 : 0);
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    s_discard_next_frame = true;
    const char *ready_msg = (framesize_clamped && s_camera_low_mem_mode)
                                ? "Camera ready. Requested resolution reduced due to memory limits."
                                : (s_camera_low_mem_mode
                                       ? "Camera ready. PSRAM fallback active; high resolutions above VGA are disabled."
                                       : "Camera ready.");
    camera_manager_set_ready_with_flash_info(ready_msg);
    return ESP_OK;
}

static void camera_manager_set_ready(const char *message) {
    s_camera_state = CAMERA_STATE_READY;
    s_camera_last_err = ESP_OK;
    if (message && message[0] != '\0') {
        strncpy(s_camera_message, message, sizeof(s_camera_message) - 1);
        s_camera_message[sizeof(s_camera_message) - 1] = '\0';
    } else {
        strcpy(s_camera_message, "Camera ready.");
    }
}

static void camera_manager_set_error(esp_err_t err, const char *message) {
    s_camera_state = CAMERA_STATE_ERROR;
    s_camera_last_err = err;
    if (message && message[0] != '\0') {
        strncpy(s_camera_message, message, sizeof(s_camera_message) - 1);
        s_camera_message[sizeof(s_camera_message) - 1] = '\0';
    } else {
        snprintf(s_camera_message, sizeof(s_camera_message), "Camera error: %s", esp_err_to_name(err));
    }
}

static void camera_manager_set_ready_with_flash_info(const char *base_message) {
    if (s_flash_enabled) {
        char message[sizeof(s_camera_message)] = {0};
        if (!s_flash_ledc) {
            snprintf(message, sizeof(message), "%s Flashlight enabled (fixed brightness).",
                     base_message ? base_message : "Camera ready.");
        } else if (s_flash_brightness_percent > 0) {
            snprintf(message, sizeof(message), "%s Flashlight enabled (%u%%).",
                     base_message ? base_message : "Camera ready.",
                     (unsigned int)s_flash_brightness_percent);
        } else {
            snprintf(message, sizeof(message), "%s Flashlight enabled (0%%).",
                     base_message ? base_message : "Camera ready.");
        }
        camera_manager_set_ready(message);
    } else {
        camera_manager_set_ready(base_message ? base_message : "Camera ready.");
    }
}

bool camera_manager_is_initialized(void) {
    return s_camera_state != CAMERA_STATE_UNINITIALIZED;
}

bool camera_manager_is_ready(void) {
    return s_camera_state == CAMERA_STATE_READY;
}

esp_err_t camera_manager_last_error(void) {
    return s_camera_last_err;
}

const char *camera_manager_status_message(void) {
    return s_camera_message;
}

bool camera_manager_is_low_mem_mode(void) {
    return s_camera_low_mem_mode;
}

framesize_t camera_manager_current_framesize(void) {
    return s_camera_framesize;
}

void camera_manager_get_frame_dimensions(uint16_t *width, uint16_t *height) {
    if (width) {
        *width = 0;
    }
    if (height) {
        *height = 0;
    }
    if (s_last_frame_width > 0 && s_last_frame_height > 0) {
        if (width) {
            *width = s_last_frame_width;
        }
        if (height) {
            *height = s_last_frame_height;
        }
        return;
    }
    if (s_camera_framesize >= FRAMESIZE_96X96 && s_camera_framesize < FRAMESIZE_INVALID) {
        const resolution_info_t *info = &resolution[s_camera_framesize];
        if (info) {
            if (width) {
                *width = info->width;
            }
            if (height) {
                *height = info->height;
            }
        }
    }
}

bool camera_manager_is_flash_supported(void) {
    return s_flash_supported;
}

bool camera_manager_is_flash_enabled(void) {
    return s_flash_supported && s_flash_enabled;
}

int camera_manager_flash_brightness(void) {
    if (!s_flash_supported || !s_flash_ledc) {
        return -1;
    }
    return (int)s_flash_brightness_percent;
}

esp_err_t camera_manager_set_flash_brightness(int percent) {
    if (!s_flash_supported || !s_flash_ledc) {
        ESP_LOGW(CAMERA_MANAGER_TAG, "Flashlight brightness requested but not supported on this hardware");
        return ESP_ERR_NOT_SUPPORTED;
    }

    int clamped = clamp(percent, 0, 100);
    if (clamped != percent) {
        ESP_LOGW(CAMERA_MANAGER_TAG, "Requested flash brightness %d out of range, clamped to %d", percent, clamped);
    }

    if ((int)s_flash_brightness_percent == clamped) {
        return ESP_OK;
    }

    s_flash_brightness_percent = (uint8_t)clamped;
    if (s_flash_state) {
        camera_flash_set(true);
    }

    s_discard_next_frame = true;
    const char *base = s_camera_low_mem_mode
                           ? "Camera ready. PSRAM fallback active; high resolutions above VGA are disabled."
                           : "Camera ready.";
    camera_manager_set_ready_with_flash_info(base);
    ESP_LOGI(CAMERA_MANAGER_TAG, "Flashlight brightness set to %d%%", clamped);
    return ESP_OK;
}

esp_err_t camera_manager_set_flash_enabled(bool enabled) {
    if (enabled && !s_flash_supported) {
        ESP_LOGW(CAMERA_MANAGER_TAG, "Flashlight requested but not supported on this hardware");
        return ESP_ERR_NOT_SUPPORTED;
    }

    bool new_state = enabled && s_flash_supported;
    if (new_state == s_flash_enabled) {
        return ESP_OK;
    }

    s_flash_enabled = new_state;
    if (!s_flash_enabled && s_flash_state) {
        camera_flash_set(false);
    }

    s_discard_next_frame = true;
    const char *base = s_camera_low_mem_mode
                           ? "Camera ready. PSRAM fallback active; high resolutions above VGA are disabled."
                           : "Camera ready.";
    camera_manager_set_ready_with_flash_info(base);
    ESP_LOGI(CAMERA_MANAGER_TAG, "Flashlight %s", s_flash_enabled ? "enabled" : "disabled");
    return ESP_OK;
}

static size_t camera_manager_escape_json(const char *input, char *output, size_t output_len) {
    if (!output || output_len == 0) {
        return 0;
    }
    if (!input) {
        output[0] = '\0';
        return 0;
    }
    size_t written = 0;
    for (size_t i = 0; input[i] != '\0' && written + 1 < output_len; ++i) {
        unsigned char c = (unsigned char)input[i];
        if (c == '"' || c == '\\') {
            if (written + 2 >= output_len) {
                break;
            }
            output[written++] = '\\';
            output[written++] = (char)c;
        } else if (c < 0x20) {
            if (written + 6 >= output_len) {
                break;
            }
            int len = snprintf(output + written, output_len - written, "\\u%04x", c);
            if (len < 0 || (size_t)len >= output_len - written) {
                break;
            }
            written += (size_t)len;
        } else {
            output[written++] = (char)c;
        }
    }
    if (written < output_len) {
        output[written] = '\0';
    } else {
        output[output_len - 1] = '\0';
    }
    return written;
}

static esp_err_t camera_manager_send_error_response(httpd_req_t *req) {
    char escaped_message[192];
    camera_manager_escape_json(camera_manager_status_message(), escaped_message, sizeof(escaped_message));
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"error\":true,\"code\":%d,\"message\":\"%s\"}",
             (int)camera_manager_last_error(),
             escaped_message);
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, payload, HTTPD_RESP_USE_STRLEN);
}
