#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "wifi_manager.h"
#include "http_server_app.h"
#include "camera_manager.h"

static const char *APP_TAG = "wifi_camera_app";

void app_main(void) {
    ESP_LOGI(APP_TAG, "Initializing WiFi camera firmware");
    ESP_ERROR_CHECK(wifi_manager_init());

    esp_err_t cam_err = camera_manager_init();
    if (cam_err != ESP_OK) {
        ESP_LOGE(APP_TAG, "Camera initialization failed: %s", esp_err_to_name(cam_err));
    }

    if (wifi_manager_credentials_available()) {
        wifi_credentials_t credentials = {0};
        if (wifi_manager_load_credentials(&credentials) == ESP_OK) {
            ESP_LOGI(APP_TAG, "Found stored credentials, attempting STA connection");
            esp_err_t err = wifi_manager_connect_station(&credentials);
            if (err != ESP_OK) {
                ESP_LOGE(APP_TAG, "Failed to start STA mode (%s), enabling provisioning AP", esp_err_to_name(err));
                ESP_ERROR_CHECK(wifi_manager_start_provisioning_softap());
            }
        } else {
            ESP_LOGW(APP_TAG, "Unable to load stored credentials, enabling provisioning AP");
            ESP_ERROR_CHECK(wifi_manager_start_provisioning_softap());
        }
    } else {
        ESP_LOGI(APP_TAG, "No credentials stored, starting provisioning AP");
        ESP_ERROR_CHECK(wifi_manager_start_provisioning_softap());
    }

    ESP_ERROR_CHECK(http_server_app_start());

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
