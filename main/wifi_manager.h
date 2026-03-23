#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// Event bits for WiFi status
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1

/**
 * @brief Initialize WiFi in station mode
 * 
 * This function initializes NVS, creates the event loop, and configures
 * WiFi station mode with credentials from KConfig.
 * 
 * Must be called once during system initialization.
 */
void wifi_init(void);

/**
 * @brief Check if WiFi is currently connected
 * 
 * @return true if connected to AP and has IP address
 * @return false if not connected
 */
bool wifi_is_connected(void);

/**
 * @brief Wait for WiFi to connect
 * 
 * Blocks until WiFi connects or timeout occurs.
 * 
 * @param timeout_ms Timeout in milliseconds (0 for no timeout)
 * @return true if connected within timeout
 * @return false if timeout occurred
 */
bool wifi_wait_connected(uint32_t timeout_ms);

/**
 * @brief Get the WiFi event group handle
 * 
 * Can be used by other components to wait for WiFi events.
 * 
 * @return Event group handle
 */
EventGroupHandle_t wifi_get_event_group(void);

#endif // WIFI_MANAGER_H
