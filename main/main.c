#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// WiFi and MQTT integration
#include "epever_data.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

static const char *TAG = "main";

// MQTT publishing task - publishes sensor data every 30 seconds
static void mqtt_publish_task(void *arg)
{
    (void)arg;
    
    ESP_LOGI(TAG, "MQTT publish task started");
    
    // Wait for WiFi connection
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    wifi_wait_connected(0); // Wait indefinitely
    ESP_LOGI(TAG, "WiFi connected!");
    
    // Give MQTT some time to connect after WiFi is up
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1) {
        // Check if both WiFi and MQTT are connected
        if (wifi_is_connected() && mqtt_is_connected()) {
            // Read sensor data and publish
            epever_data_t data;
            epever_data_read(&data);
            
            if (data.valid) {
                mqtt_publish_sensors(&data);
            } else {
                ESP_LOGW(TAG, "Sensor data not valid yet, skipping publish");
            }
        } else {
            if (!wifi_is_connected()) {
                ESP_LOGW(TAG, "WiFi not connected, waiting...");
            }
            if (!mqtt_is_connected()) {
                ESP_LOGW(TAG, "MQTT not connected, waiting...");
            }
        }
        
        // Publish every 30 seconds
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Epever Controller starting...");
    
    // Initialize Epever controller communication
    ESP_LOGI(TAG, "Initializing Epever controller...");
    epever_init();
    
    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init();
    
    // Initialize MQTT
    ESP_LOGI(TAG, "Initializing MQTT...");
    mqtt_init();
    mqtt_start();
    
    // Create MQTT publishing task
    xTaskCreate(mqtt_publish_task, "mqtt_publish", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "All tasks started successfully");
}