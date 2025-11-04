#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi_types.h"

#define WIFI_MANAGER_MAX_SSID_LEN 32
#define WIFI_MANAGER_MAX_PASSWORD_LEN 64

typedef struct {
    char ssid[WIFI_MANAGER_MAX_SSID_LEN];
    char password[WIFI_MANAGER_MAX_PASSWORD_LEN];
} wifi_credentials_t;

typedef struct {
    bool connecting;
    uint8_t last_disconnect_reason;
    int retry_count;
    bool provisioning_mode;
} wifi_sta_status_t;

esp_err_t wifi_manager_init(void);
bool wifi_manager_credentials_available(void);
esp_err_t wifi_manager_load_credentials(wifi_credentials_t *credentials);
esp_err_t wifi_manager_save_credentials(const wifi_credentials_t *credentials);
esp_err_t wifi_manager_clear_credentials(void);
esp_err_t wifi_manager_connect_station(const wifi_credentials_t *credentials);
esp_err_t wifi_manager_start_provisioning_softap(void);
esp_err_t wifi_manager_scan_networks(char *json_buffer, size_t buffer_len);
bool wifi_manager_is_connected(void);
void wifi_manager_request_reconnect(void);
bool wifi_manager_is_provisioning(void);
esp_err_t wifi_manager_get_connected_ap(wifi_ap_record_t *ap_info);
esp_err_t wifi_manager_get_ip_info(esp_netif_ip_info_t *ip_info);
esp_err_t wifi_manager_disable_softap(void);
esp_err_t wifi_manager_factory_reset(void);
void wifi_manager_get_sta_status(wifi_sta_status_t *status);
const char *wifi_manager_reason_to_string(uint8_t reason);
