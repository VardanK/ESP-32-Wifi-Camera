#include "wifi_manager.h"

#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"

#define WIFI_MANAGER_TAG "wifi_manager"
#define WIFI_MANAGER_NVS_NAMESPACE "wifi"
#define WIFI_MANAGER_NVS_KEY "cred"
#define WIFI_MANAGER_MAX_RETRY 5

typedef enum {
    WIFI_STATE_IDLE = 0,
    WIFI_STATE_STA_CONNECTING,
    WIFI_STATE_STA_CONNECTED,
    WIFI_STATE_SOFTAP_RUNNING
} wifi_state_t;

static wifi_state_t s_state = WIFI_STATE_IDLE;
static bool s_wifi_started = false;
static int s_sta_retry_count = 0;
static wifi_credentials_t s_cached_credentials = {0};
static bool s_credentials_loaded = false;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static bool s_provisioning_mode = false;
static bool s_connected_ap_valid = false;
static wifi_ap_record_t s_connected_ap_info = {0};
static bool s_ip_info_valid = false;
static esp_netif_ip_info_t s_connected_ip_info = {0};
static bool s_sta_connecting = false;
static uint8_t s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
static int s_sta_last_retry_count = 0;

static esp_err_t wifi_manager_restore_credentials(void);
static esp_err_t wifi_manager_store_credentials(const wifi_credentials_t *credentials);
static void wifi_manager_handle_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static esp_err_t wifi_manager_configure_softap(void);
static void wifi_manager_set_state(wifi_state_t new_state);
static void wifi_manager_log_state_transition(wifi_state_t new_state);
static size_t wifi_manager_escape_json_string(const uint8_t *src, size_t src_len, char *dest, size_t dest_len);
static void wifi_manager_stop_wifi(void);
static const char *wifi_manager_reason_to_message(uint8_t reason);

static esp_err_t wifi_manager_restore_credentials(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_MANAGER_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    } else if (err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(s_cached_credentials);
    err = nvs_get_blob(handle, WIFI_MANAGER_NVS_KEY, &s_cached_credentials, &required_size);
    nvs_close(handle);
    if (err == ESP_OK && required_size == sizeof(s_cached_credentials)) {
        s_credentials_loaded = true;
    }
    return err;
}

static esp_err_t wifi_manager_store_credentials(const wifi_credentials_t *credentials) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_MANAGER_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(WIFI_MANAGER_TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(WIFI_MANAGER_TAG, "Storing WiFi credentials SSID='%s'", credentials->ssid);

    err = nvs_set_blob(handle, WIFI_MANAGER_NVS_KEY, credentials, sizeof(*credentials));
    ESP_LOGI(WIFI_MANAGER_TAG, "nvs_set_blob returned: %s", esp_err_to_name(err));

    if (err == ESP_OK) {
        err = nvs_commit(handle);
        ESP_LOGI(WIFI_MANAGER_TAG, "nvs_commit returned: %s", esp_err_to_name(err));
        if (err == ESP_OK) {
            s_cached_credentials = *credentials;
            s_credentials_loaded = true;
        }
    }
    nvs_close(handle);
    ESP_LOGI(WIFI_MANAGER_TAG, "WiFi credentials stored with result: %s", esp_err_to_name(err));
    return err;
}

static void wifi_manager_stop_wifi(void) {
    if (s_wifi_started) {
        esp_err_t err = esp_wifi_stop();
        if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT && err != ESP_ERR_WIFI_NOT_STARTED) {
            ESP_LOGW(WIFI_MANAGER_TAG, "Failed to stop WiFi: %s", esp_err_to_name(err));
        }
        if (err == ESP_OK || err == ESP_ERR_WIFI_NOT_INIT || err == ESP_ERR_WIFI_NOT_STARTED) {
            s_wifi_started = false;
        }
    }
}

static esp_err_t wifi_manager_erase_credentials(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_MANAGER_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    } else if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(handle, WIFI_MANAGER_NVS_KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err == ESP_OK) {
        memset(&s_cached_credentials, 0, sizeof(s_cached_credentials));
        s_credentials_loaded = false;
    }
    return err;
}

static esp_err_t wifi_manager_configure_softap(void) {
    uint8_t mac[6] = {0};
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, mac));
    wifi_config_t ap_config = {
        .ap = {
            .ssid_len = 0,
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .ssid_hidden = 0,
            .max_connection = 4,
            .beacon_interval = 100
        }
    };
    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "WifiCamera-%02X%02X", mac[4], mac[5]);
    memset(ap_config.ap.password, 0, sizeof(ap_config.ap.password));
    return esp_wifi_set_config(WIFI_IF_AP, &ap_config);
}

static void wifi_manager_log_state_transition(wifi_state_t new_state) {
    const char *state_str[] = {
        "IDLE",
        "STA_CONNECTING",
        "STA_CONNECTED",
        "SOFTAP_RUNNING"
    };
    ESP_LOGI(WIFI_MANAGER_TAG, "State transition: %s -> %s", state_str[s_state], state_str[new_state]);
}

static void wifi_manager_set_state(wifi_state_t new_state) {
    if (new_state != s_state) {
        wifi_manager_log_state_transition(new_state);
        s_state = new_state;
    }
}

static size_t wifi_manager_escape_json_string(const uint8_t *src, size_t src_len, char *dest, size_t dest_len) {
    if (!src || !dest || dest_len == 0) {
        return 0;
    }
    size_t written = 0;
    for (size_t i = 0; i < src_len && src[i] != '\0'; ++i) {
        unsigned char c = src[i];
        if (c == '"' || c == '\\') {
            if (written + 2 >= dest_len) {
                break;
            }
            dest[written++] = '\\';
            dest[written++] = (char)c;
        } else if (c < 0x20) {
            if (written + 6 >= dest_len) {
                break;
            }
            int len = snprintf(dest + written, dest_len - written, "\\u%04x", c);
            if (len < 0 || (size_t)len >= dest_len - written) {
                written = dest_len - 1;
                break;
            }
            written += (size_t)len;
        } else {
            if (written + 1 >= dest_len) {
                break;
            }
            dest[written++] = (char)c;
        }
    }
    if (written < dest_len) {
        dest[written] = '\0';
    } else {
        dest[dest_len - 1] = '\0';
    }
    return written;
}

static void wifi_manager_handle_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(WIFI_MANAGER_TAG, "Station started, connecting...");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(WIFI_MANAGER_TAG, "Station disconnected");
                s_sta_retry_count++;
                s_sta_last_retry_count = s_sta_retry_count;
                wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
                if (disc) {
                    s_last_disconnect_reason = disc->reason;
                    ESP_LOGW(WIFI_MANAGER_TAG, "Disconnect reason: %u", disc->reason);
                }
                s_sta_connecting = true;
                wifi_manager_set_state(WIFI_STATE_STA_CONNECTING);
                s_connected_ap_valid = false;
                s_ip_info_valid = false;
                if (s_sta_retry_count <= WIFI_MANAGER_MAX_RETRY) {
                    ESP_LOGI(WIFI_MANAGER_TAG, "Retrying connection (%d/%d)", s_sta_retry_count, WIFI_MANAGER_MAX_RETRY);
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(WIFI_MANAGER_TAG, "Max retries reached, switching to provisioning AP");
                    s_sta_connecting = false;
                    wifi_manager_start_provisioning_softap();
                }
                break;
            case WIFI_EVENT_AP_START:
                ESP_LOGI(WIFI_MANAGER_TAG, "Provisioning SoftAP started");
                wifi_manager_set_state(WIFI_STATE_SOFTAP_RUNNING);
                break;
            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(WIFI_MANAGER_TAG, "SoftAP stopped");
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_MANAGER_TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_sta_retry_count = 0;
        if (s_sta_netif) {
            esp_netif_ip_info_t ip_info = {0};
            if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
                s_connected_ip_info = ip_info;
                s_ip_info_valid = true;
            }
        }
        wifi_ap_record_t ap_info = {0};
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_connected_ap_info = ap_info;
            s_connected_ap_valid = true;
        }
        esp_err_t mode_err = wifi_manager_disable_softap();
        if (mode_err != ESP_OK) {
            ESP_LOGW(WIFI_MANAGER_TAG, "Failed to enforce STA-only mode: %s", esp_err_to_name(mode_err));
        }
        s_sta_connecting = false;
        s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
        s_sta_last_retry_count = 0;
        wifi_manager_set_state(WIFI_STATE_STA_CONNECTED);
    }
}

esp_err_t wifi_manager_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_manager_handle_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_manager_handle_event, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));

    wifi_manager_restore_credentials();
    return ESP_OK;
}

bool wifi_manager_credentials_available(void) {
    return s_credentials_loaded;
}

esp_err_t wifi_manager_load_credentials(wifi_credentials_t *credentials) {
    if (!s_credentials_loaded) {
        esp_err_t err = wifi_manager_restore_credentials();
        if (err != ESP_OK) {
            return err;
        }
    }
    *credentials = s_cached_credentials;
    return ESP_OK;
}

esp_err_t wifi_manager_save_credentials(const wifi_credentials_t *credentials) {
    return wifi_manager_store_credentials(credentials);
}

esp_err_t wifi_manager_clear_credentials(void) {
    return wifi_manager_erase_credentials();
}

esp_err_t wifi_manager_connect_station(const wifi_credentials_t *credentials) {
    if (!credentials || credentials->ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_manager_disable_softap();

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, credentials->ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, credentials->password, sizeof(wifi_config.sta.password));

    wifi_manager_stop_wifi();

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(WIFI_MANAGER_TAG, "Failed to set STA mode: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(WIFI_MANAGER_TAG, "Failed to apply STA config: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(WIFI_MANAGER_TAG, "Failed to start WiFi: %s", esp_err_to_name(err));
        return err;
    }
    s_wifi_started = true;

    s_connected_ap_valid = false;
    s_ip_info_valid = false;
    s_provisioning_mode = false;

    wifi_manager_set_state(WIFI_STATE_STA_CONNECTING);
    s_sta_retry_count = 0;
    s_sta_last_retry_count = 0;
    s_sta_connecting = true;
    s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(WIFI_MANAGER_TAG, "Failed to initiate STA connection: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t wifi_manager_start_provisioning_softap(void) {
    wifi_manager_stop_wifi();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(wifi_manager_configure_softap());
    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }
    s_wifi_started = true;

    s_connected_ap_valid = false;
    s_ip_info_valid = false;
    s_provisioning_mode = true;
    s_sta_connecting = false;
    s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
    s_sta_last_retry_count = 0;
    if (s_ap_netif) {
        esp_netif_dhcps_start(s_ap_netif);
    }
    wifi_manager_set_state(WIFI_STATE_SOFTAP_RUNNING);
    return ESP_OK;
}

esp_err_t wifi_manager_scan_networks(char *json_buffer, size_t buffer_len) {
    if (!json_buffer || buffer_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    if (mode != WIFI_MODE_APSTA && mode != WIFI_MODE_STA) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    }

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };

    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t ap_num = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
    if (ap_num == 0) {
        snprintf(json_buffer, buffer_len, "{\"networks\":[]}");
        return ESP_OK;
    }

    wifi_ap_record_t ap_records[16];
    uint16_t max_records = sizeof(ap_records) / sizeof(ap_records[0]);
    if (ap_num > max_records) {
        ap_num = max_records;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

    size_t offset = 0;
    offset += snprintf(json_buffer + offset, buffer_len - offset, "{\"networks\":[");
    for (int i = 0; i < ap_num && offset < buffer_len; ++i) {
        const wifi_ap_record_t *ap = &ap_records[i];
        char ssid_escaped[(WIFI_MANAGER_MAX_SSID_LEN * 4) + 4];
        wifi_manager_escape_json_string(ap->ssid, sizeof(ap->ssid), ssid_escaped, sizeof(ssid_escaped));
        int written = snprintf(
            json_buffer + offset,
            buffer_len - offset,
            "%s{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}",
            (i == 0) ? "" : ",",
            ssid_escaped,
            ap->rssi,
            ap->authmode
        );
        if (written < 0) {
            return ESP_FAIL;
        }
        offset += (size_t)written;
    }
    if (offset < buffer_len) {
        snprintf(json_buffer + offset, buffer_len - offset, "]}");
    } else if (buffer_len > 0) {
        json_buffer[buffer_len - 1] = '\0';
    }
    return ESP_OK;
}

bool wifi_manager_is_connected(void) {
    return s_state == WIFI_STATE_STA_CONNECTED;
}

void wifi_manager_request_reconnect(void) {
    if (!s_credentials_loaded) {
        ESP_LOGW(WIFI_MANAGER_TAG, "No stored credentials available for reconnect");
        return;
    }
    wifi_manager_connect_station(&s_cached_credentials);
}

bool wifi_manager_is_provisioning(void) {
    return s_provisioning_mode;
}

esp_err_t wifi_manager_get_connected_ap(wifi_ap_record_t *ap_info) {
    if (!ap_info) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_connected_ap_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    *ap_info = s_connected_ap_info;
    return ESP_OK;
}

esp_err_t wifi_manager_get_ip_info(esp_netif_ip_info_t *ip_info) {
    if (!ip_info) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_sta_netif) {
        esp_err_t err = esp_netif_get_ip_info(s_sta_netif, ip_info);
        if (err == ESP_OK) {
            return err;
        }
    }
    if (!s_ip_info_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    *ip_info = s_connected_ip_info;
    return ESP_OK;
}

esp_err_t wifi_manager_disable_softap(void) {
    wifi_mode_t current_mode;
    esp_err_t err = esp_wifi_get_mode(&current_mode);
    if (err != ESP_OK) {
        ESP_LOGW(WIFI_MANAGER_TAG, "Failed to get current WiFi mode: %s", esp_err_to_name(err));
        current_mode = WIFI_MODE_NULL;
    }

    if (!s_wifi_started) {
        s_provisioning_mode = false;
        return ESP_OK;
    }

    if (current_mode == WIFI_MODE_AP || current_mode == WIFI_MODE_APSTA) {
        esp_err_t mode_err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (mode_err == ESP_ERR_WIFI_NOT_INIT || mode_err == ESP_ERR_WIFI_NOT_STARTED) {
            mode_err = ESP_OK;
        }
        if (mode_err != ESP_OK) {
            ESP_LOGE(WIFI_MANAGER_TAG, "Failed to switch to STA mode: %s", esp_err_to_name(mode_err));
            return mode_err;
        }
    }

    if (s_ap_netif) {
        esp_netif_dhcps_stop(s_ap_netif);
    }
    s_provisioning_mode = false;
    return ESP_OK;
}

esp_err_t wifi_manager_factory_reset(void) {
    ESP_LOGI(WIFI_MANAGER_TAG, "Factory reset requested, clearing NVS and credentials");
    esp_wifi_disconnect();
    wifi_manager_stop_wifi();

    if (s_ap_netif) {
        esp_netif_dhcps_stop(s_ap_netif);
    }

    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
        ESP_LOGE(WIFI_MANAGER_TAG, "Failed to erase NVS: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE(WIFI_MANAGER_TAG, "Failed to reinitialize NVS: %s", esp_err_to_name(err));
        return err;
    }

    memset(&s_cached_credentials, 0, sizeof(s_cached_credentials));
    s_credentials_loaded = false;
    s_connected_ap_valid = false;
    s_ip_info_valid = false;
    s_wifi_started = false;
    s_provisioning_mode = true;
    s_state = WIFI_STATE_IDLE;
    s_sta_connecting = false;
    s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
    s_sta_last_retry_count = 0;
    ESP_LOGI(WIFI_MANAGER_TAG, "NVS cleared successfully");
    return ESP_OK;
}

void wifi_manager_get_sta_status(wifi_sta_status_t *status) {
    if (!status) {
        return;
    }
    status->connecting = s_sta_connecting;
    status->last_disconnect_reason = s_last_disconnect_reason;
    status->retry_count = s_sta_last_retry_count;
    status->provisioning_mode = s_provisioning_mode;
}

const char *wifi_manager_reason_to_string(uint8_t reason) {
    return wifi_manager_reason_to_message(reason);
}

static const char *wifi_manager_reason_to_message(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_AUTH_EXPIRE:
        case WIFI_REASON_AUTH_FAIL:
            return "Authentication failed. Check SSID/password.";
        case WIFI_REASON_BEACON_TIMEOUT:
            return "Connection timed out. Device may be out of range.";
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
            return "WPA handshake timed out. Retry or reboot router.";
        case WIFI_REASON_NO_AP_FOUND:
            return "Selected network not found. Ensure the router is on.";
        case WIFI_REASON_ASSOC_LEAVE:
            return "Disconnected from access point.";
        case WIFI_REASON_UNSPECIFIED:
            return "Waiting for connection.";
        default:
            return "Wi-Fi disconnected.";
    }
}
const char *wifi_manager_reason_to_string(uint8_t reason);
