#include "mqtt_manager.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "mqtt_manager";

// MQTT client handle
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_mqtt_connected = false;

// Base topics for Home Assistant
#define HA_DISCOVERY_PREFIX "jussila/sensor"
#define HA_DEVICE_NAME "epever"
#define HA_AVAILABILITY_TOPIC "jussila/sensor/epever/availability"

// Forward declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void publish_sensor_discovery(const char *sensor_name, const char *friendly_name, 
                                    const char *unit, const char *device_class, const char *icon);

void mqtt_init(void)
{
    // Configure MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
        .credentials.username = CONFIG_MQTT_USERNAME,
        .credentials.authentication.password = CONFIG_MQTT_PASSWORD,
        .session.last_will.topic = HA_AVAILABILITY_TOPIC,
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 7,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };
    
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    // Register event handler
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    ESP_LOGI(TAG, "MQTT client initialized with broker: %s", CONFIG_MQTT_BROKER_URI);
}

void mqtt_start(void)
{
    if (s_mqtt_client != NULL) {
        esp_mqtt_client_start(s_mqtt_client);
        ESP_LOGI(TAG, "MQTT client started");
    }
}

bool mqtt_is_connected(void)
{
    return s_mqtt_connected;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            s_mqtt_connected = true;
            
            // Publish availability as online
            mqtt_publish_availability(true);
            
            // Publish discovery configuration
            mqtt_publish_discovery();
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
            s_mqtt_connected = false;
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            break;
            
        default:
            break;
    }
}

void mqtt_publish_discovery(void)
{
    if (!s_mqtt_connected) {
        ESP_LOGW(TAG, "Cannot publish discovery: MQTT not connected");
        return;
    }
    
    ESP_LOGI(TAG, "Publishing Home Assistant discovery configuration...");
    
    // PV sensors
    publish_sensor_discovery("pv_voltage", "PV Voltage", "V", "voltage", "mdi:solar-power");
    publish_sensor_discovery("pv_current", "PV Current", "A", "current", "mdi:solar-power");
    publish_sensor_discovery("pv_power", "PV Power", "W", "power", "mdi:solar-power");
    
    // Battery sensors
    publish_sensor_discovery("battery_voltage", "Battery Voltage", "V", "voltage", "mdi:battery");
    publish_sensor_discovery("battery_current", "Battery Current", "A", "current", "mdi:battery");
    publish_sensor_discovery("battery_power", "Battery Power", "W", "power", "mdi:battery");
    publish_sensor_discovery("battery_soc", "Battery SOC", "%", "battery", "mdi:battery");
    
    // Load sensors
    publish_sensor_discovery("load_voltage", "Load Voltage", "V", "voltage", "mdi:power-plug");
    publish_sensor_discovery("load_current", "Load Current", "A", "current", "mdi:power-plug");
    publish_sensor_discovery("load_power", "Load Power", "W", "power", "mdi:power-plug");
    
    // Temperature sensors
    publish_sensor_discovery("battery_temp", "Battery Temperature", "°C", "temperature", "mdi:thermometer");
    publish_sensor_discovery("controller_temp", "Controller Temperature", "°C", "temperature", "mdi:thermometer");
    publish_sensor_discovery("component_temp", "Component Temperature", "°C", "temperature", "mdi:thermometer");
    
    // System voltage
    publish_sensor_discovery("system_voltage", "System Voltage", "V", "voltage", "mdi:flash");
    
    ESP_LOGI(TAG, "Discovery configuration published");
}

static void publish_sensor_discovery(const char *sensor_name, const char *friendly_name, 
                                    const char *unit, const char *device_class, const char *icon)
{
    char topic[128];
    char payload[512];
    
    // Build discovery topic
    snprintf(topic, sizeof(topic), "%s/%s_%s/config", HA_DISCOVERY_PREFIX, HA_DEVICE_NAME, sensor_name);
    
    // Build JSON payload with device info
    snprintf(payload, sizeof(payload),
        "{"
        "\"name\":\"%s\","
        "\"unique_id\":\"epever_%s\","
        "\"state_topic\":\"%s/%s_%s/state\","
        "\"unit_of_measurement\":\"%s\","
        "\"device_class\":\"%s\","
        "\"icon\":\"%s\","
        "\"availability_topic\":\"%s\","
        "\"device\":{"
            "\"identifiers\":[\"epever_solar_controller\"],"
            "\"name\":\"Epever Solar Controller\","
            "\"manufacturer\":\"Epever\","
            "\"model\":\"Tracer 1210A\""
        "}"
        "}",
        friendly_name,
        sensor_name,
        HA_DISCOVERY_PREFIX, HA_DEVICE_NAME, sensor_name,
        unit,
        device_class,
        icon,
        HA_AVAILABILITY_TOPIC
    );
    
    // Publish with QoS 1 and retained
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 1);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish discovery for %s", sensor_name);
    }
}

void mqtt_publish_sensors(const epever_data_t *data)
{
    if (!s_mqtt_connected || data == NULL || !data->valid) {
        return;
    }
    
    char topic[128];
    char payload[32];
    
    // Helper macro to publish a sensor value
    #define PUBLISH_SENSOR(name, value, format) \
        snprintf(topic, sizeof(topic), "%s/%s_%s/state", HA_DISCOVERY_PREFIX, HA_DEVICE_NAME, name); \
        snprintf(payload, sizeof(payload), format, value); \
        esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 1);
    
    // Publish all sensor values
    PUBLISH_SENSOR("pv_voltage", data->pv_voltage, "%.2f");
    PUBLISH_SENSOR("pv_current", data->pv_current, "%.2f");
    PUBLISH_SENSOR("pv_power", data->pv_power, "%.2f");
    
    PUBLISH_SENSOR("battery_voltage", data->battery_voltage, "%.2f");
    PUBLISH_SENSOR("battery_current", data->battery_current, "%.2f");
    PUBLISH_SENSOR("battery_power", data->battery_power, "%.2f");
    PUBLISH_SENSOR("battery_soc", data->battery_soc, "%.2f");
    
    PUBLISH_SENSOR("load_voltage", data->load_voltage, "%.2f");
    PUBLISH_SENSOR("load_current", data->load_current, "%.2f");
    PUBLISH_SENSOR("load_power", data->load_power, "%.2f");
    
    PUBLISH_SENSOR("battery_temp", data->battery_temp, "%.2f");
    PUBLISH_SENSOR("controller_temp", data->controller_temp, "%.2f");
    PUBLISH_SENSOR("component_temp", data->component_temp, "%.2f");
    
    PUBLISH_SENSOR("system_voltage", data->system_voltage, "%.2f");
    
    #undef PUBLISH_SENSOR
    
    ESP_LOGI(TAG, "Published sensor data to MQTT");
}

void mqtt_publish_availability(bool online)
{
    if (s_mqtt_client == NULL) {
        return;
    }
    
    const char *status = online ? "online" : "offline";
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, HA_AVAILABILITY_TOPIC, status, 0, 1, 1);
    
    if (msg_id >= 0) {
        ESP_LOGI(TAG, "Published availability: %s", status);
    }
}
