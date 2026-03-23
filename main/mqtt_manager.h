#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>
#include "epever_data.h"

/**
 * @brief Initialize MQTT client
 * 
 * Configures MQTT client with broker URI and credentials from KConfig.
 * Registers event handlers and prepares for connection.
 * 
 * Must be called after wifi_init() but before mqtt_start().
 */
void mqtt_init(void);

/**
 * @brief Start MQTT client
 * 
 * Begins connection to MQTT broker. Connection is asynchronous.
 * Use mqtt_is_connected() to check status.
 */
void mqtt_start(void);

/**
 * @brief Check if MQTT client is connected to broker
 * 
 * @return true if connected
 * @return false if not connected
 */
bool mqtt_is_connected(void);

/**
 * @brief Publish Home Assistant discovery configuration
 * 
 * Publishes MQTT discovery messages for all Epever sensors.
 * Creates a single "Epever Solar Controller" device in Home Assistant
 * with all sensors grouped together.
 * 
 * Should be called once after MQTT connection is established.
 */
void mqtt_publish_discovery(void);

/**
 * @brief Publish sensor data to MQTT
 * 
 * Reads current sensor values from shared data structure and publishes
 * to MQTT state topics. Messages are published with QoS 1 and retained flag.
 * 
 * @param data Pointer to sensor data to publish
 */
void mqtt_publish_sensors(const epever_data_t *data);

/**
 * @brief Publish availability status
 * 
 * Publishes "online" or "offline" to the availability topic.
 * 
 * @param online true for "online", false for "offline"
 */
void mqtt_publish_availability(bool online);

#endif // MQTT_MANAGER_H
