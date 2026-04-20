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

// HiveMQ Cloud CA Certificate (ISRG Root X1 - Let's Encrypt)
/*
static const char hivemq_ca_cert[] = 
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----";
*/

// Forward declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void publish_sensor_discovery(const char *sensor_name, const char *friendly_name, 
                                    const char *unit, const char *device_class, const char *icon);

void mqtt_init(void)
{
    // Build broker URI and configure MQTT client based on selected broker
#ifdef CONFIG_MQTT_BROKER_MOSQUITTO
    // Mosquitto: Plain MQTT (no TLS)
    char broker_uri[128];
    snprintf(broker_uri, sizeof(broker_uri), "%s", CONFIG_MQTT_BROKER_URI);
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .credentials.username = CONFIG_MQTT_USERNAME,
        .credentials.authentication.password = CONFIG_MQTT_PASSWORD,
        .session.last_will.topic = HA_AVAILABILITY_TOPIC,
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 7,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };
    
    ESP_LOGI(TAG, "Configuring Mosquitto broker: %s", broker_uri);
    
#elif defined(CONFIG_MQTT_BROKER_HIVEMQ)
    // HiveMQ Cloud: MQTTS with TLS
    char broker_uri[128];
    snprintf(broker_uri, sizeof(broker_uri), "mqtts://%s:%d", CONFIG_HIVEMQ_HOST, CONFIG_HIVEMQ_PORT);
    
    // Debug: Print certificate info
    size_t cert_len = strlen(hivemq_ca_cert);
    ESP_LOGI(TAG, "Certificate size: %d bytes", cert_len);
    ESP_LOGI(TAG, "Certificate starts with: %.30s", hivemq_ca_cert);
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .broker.verification.certificate = hivemq_ca_cert,
        .credentials.username = CONFIG_HIVEMQ_USERNAME,
        .credentials.authentication.password = CONFIG_HIVEMQ_PASSWORD,
        .session.last_will.topic = HA_AVAILABILITY_TOPIC,
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 7,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };
    
    ESP_LOGI(TAG, "Configuring HiveMQ Cloud broker: %s (TLS enabled)", broker_uri);
    ESP_LOGI(TAG, "Username: %s, Cert len: %d", CONFIG_HIVEMQ_USERNAME, cert_len);
#else
    #error "No MQTT broker selected in configuration"
#endif
    
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    // Register event handler
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    ESP_LOGI(TAG, "MQTT client initialized successfully");
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
