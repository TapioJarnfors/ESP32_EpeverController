#include "wifi_manager.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_manager";

// Event group for WiFi status
static EventGroupHandle_t s_wifi_event_group = NULL;

// Retry counter
static int s_retry_num = 0;
#define MAX_RETRY_COUNT 5

// Forward declarations
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);

void wifi_init(void)
{
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create event group
    s_wifi_event_group = xEventGroupCreate();
    
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default WiFi station
    esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization complete. Connecting to SSID: %s", CONFIG_WIFI_SSID);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started, connecting...");
        esp_wifi_connect();
        
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY_COUNT) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to AP (attempt %d/%d)", s_retry_num, MAX_RETRY_COUNT);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGW(TAG, "Failed to connect to AP after %d attempts", MAX_RETRY_COUNT);
            // Reset retry counter after a delay to allow reconnection attempts
            s_retry_num = 0;
        }
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

bool wifi_is_connected(void)
{
    if (s_wifi_event_group == NULL) {
        return false;
    }
    
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

bool wifi_wait_connected(uint32_t timeout_ms)
{
    if (s_wifi_event_group == NULL) {
        return false;
    }
    
    TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           timeout_ticks);
    
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

EventGroupHandle_t wifi_get_event_group(void)
{
    return s_wifi_event_group;
}
