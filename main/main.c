#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

// WiFi and MQTT integration
#include "epever_data.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

static const char *TAG = "epever_modbus_master";

// UART2 to Epever RS-485 adapter
#define MB_UART           UART_NUM_2
#define MB_RX_GPIO        16
#define MB_TX_GPIO        17

#define MB_BAUDRATE       115200
#define MB_BUF_SIZE       1024

// Modbus RTU limits
#define MB_MAX_FRAME      256

// Timeouts (tune if needed)
#define MB_RESP_TIMEOUT_MS  250   // wait for full response
#define MB_INTERCHAR_MS      20   // stop when no new bytes arrive

// Slave
#define MB_SLAVE_ID       1

// Standard Modbus CRC16 (poly 0xA001, init 0xFFFF)
static uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

static void crc_append(uint8_t *frame, uint16_t len_without_crc)
{
    uint16_t crc = modbus_crc16(frame, len_without_crc);
    frame[len_without_crc + 0] = (uint8_t)(crc & 0xFF);        // CRC Lo
    frame[len_without_crc + 1] = (uint8_t)((crc >> 8) & 0xFF); // CRC Hi
}

// Build: [id][fc][addr_hi][addr_lo][count_hi][count_lo][crc_lo][crc_hi]
static uint16_t build_read_input_regs(uint8_t slave, uint16_t start_addr, uint16_t count, uint8_t *out)
{
    out[0] = slave;
    out[1] = 0x04; // Read Input Registers
    out[2] = (uint8_t)((start_addr >> 8) & 0xFF);
    out[3] = (uint8_t)(start_addr & 0xFF);
    out[4] = (uint8_t)((count >> 8) & 0xFF);
    out[5] = (uint8_t)(count & 0xFF);
    crc_append(out, 6);
    return 8;
}

static esp_err_t uart_send_frame(const uint8_t *frame, uint16_t len)
{
    // Flush RX before request (avoid mixing old bytes with new response)
    uart_flush_input(MB_UART);

    int wr = uart_write_bytes(MB_UART, (const char *)frame, len);
    if (wr != len) {
        ESP_LOGE(TAG, "uart_write_bytes wrote %d of %u", wr, len);
        return ESP_FAIL;
    }

    // Ensure TX is physically complete
    ESP_ERROR_CHECK(uart_wait_tx_done(MB_UART, pdMS_TO_TICKS(100)));
    return ESP_OK;
}

// Read until either:
// - expected length satisfied (if expected_len > 0), OR
// - no new bytes arrive for MB_INTERCHAR_MS, OR
// - total time exceeds MB_RESP_TIMEOUT_MS
static int uart_read_frame(uint8_t *buf, int buf_max, int expected_len)
{
    int total = 0;
    int elapsed_ms = 0;

    while (elapsed_ms < MB_RESP_TIMEOUT_MS && total < buf_max) {
        int want = buf_max - total;
        // read a chunk with short timeout (inter-char)
        int got = uart_read_bytes(MB_UART, buf + total, want, pdMS_TO_TICKS(MB_INTERCHAR_MS));
        elapsed_ms += MB_INTERCHAR_MS;

        if (got > 0) {
            total += got;

            if (expected_len > 0 && total >= expected_len) {
                break;
            }

            // reset inter-char timer effect by not breaking; we keep collecting until quiet
            // (still bounded by MB_RESP_TIMEOUT_MS)
        } else {
            // No bytes this window: if we already have something, treat as end-of-frame
            if (total > 0) break;
        }
    }

    return total;
}

static bool modbus_validate_crc(const uint8_t *frame, int len)
{
    if (len < 4) return false;
    uint16_t rx_crc = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    uint16_t calc = modbus_crc16(frame, (uint16_t)(len - 2));
    return rx_crc == calc;
}

// Helpers for decoding
static int16_t s16(uint16_t v) { return (int16_t)v; }
static uint32_t u32_from_regs(uint16_t lo, uint16_t hi) { return ((uint32_t)hi << 16) | (uint32_t)lo; }

// Forward declarations
static void poll_task(void *arg);
static void mqtt_publish_task(void *arg);

static void poll_task(void *arg)
{
    (void)arg;

    uint8_t req[8];
    uint8_t resp[MB_MAX_FRAME];

    const uint16_t start = 0x3100;
//*    const uint16_t count = 0x20; // 32 registers => 0x3100..0x311F (includes temps, SOC, sys_v)
    const uint16_t count = 0x05; // 32 registers => 0x3100..0x311F (includes temps, SOC, sys_v)

    while (1) {
        uint16_t req_len = build_read_input_regs(MB_SLAVE_ID, start, count, req);

        ESP_LOGI(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X",
                 req[0], req[1], req[2], req[3], req[4], req[5], req[6], req[7]);

        if (uart_send_frame(req, req_len) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Response: [id][fc][bytecount][data...][crc_lo][crc_hi]
        const int expected = 1 + 1 + 1 + (2 * count) + 2; // = 69 bytes for count=32
        int got = uart_read_frame(resp, sizeof(resp), expected);

        if (got <= 0) {
            ESP_LOGW(TAG, "No response");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "RX len=%d", got);
        ESP_LOG_BUFFER_HEX(TAG, resp, got);

        if (!modbus_validate_crc(resp, got)) {
            ESP_LOGW(TAG, "Bad CRC");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (resp[0] != MB_SLAVE_ID) {
            ESP_LOGW(TAG, "Unexpected slave id 0x%02X", resp[0]);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Exception response: function | 0x80
        if (resp[1] & 0x80) {
            ESP_LOGE(TAG, "Modbus exception for fc=0x%02X, code=0x%02X", resp[1], resp[2]);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (resp[1] != 0x04) {
            ESP_LOGW(TAG, "Unexpected function 0x%02X", resp[1]);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        uint8_t bytecount = resp[2];
        if (bytecount != (uint8_t)(2 * count)) {
            ESP_LOGW(TAG, "Unexpected bytecount=%u expected=%u", bytecount, (unsigned)(2 * count));
            // continue anyway; we'll parse what we got safely
        }

        // Parse registers (big-endian per register)
        uint16_t regs[0x20] = {0};
        int data_bytes = got - 5; // subtract [id][fc][bc] + [crc_lo][crc_hi] => 3+2 = 5
        int regs_available = data_bytes / 2;
        if (regs_available > (int)count) regs_available = (int)count;
        if (regs_available < 0) regs_available = 0;

        for (int i = 0; i < regs_available; i++) {
            regs[i] = ((uint16_t)resp[3 + 2*i] << 8) | (uint16_t)resp[3 + 2*i + 1];
        }

        // 1) Raw dump in register form for validation
        // (each line: address: value)
        for (int i = 0; i < regs_available; i++) {
            ESP_LOGI(TAG, "REG 0x%04X = 0x%04X (%u)", (unsigned)(start + i), regs[i], (unsigned)regs[i]);
        }

        // 2) Decoded values (per your markdown map)
//*        if (regs_available >= 0x10) {
        if (regs_available >= 0x05) {
            // Decode sensor values
            float pv_v   = regs[0x00] / 100.0f; // 0x3100
            float pv_i   = regs[0x01] / 100.0f; // 0x3101
            float pv_p   = u32_from_regs(regs[0x02], regs[0x03]) / 100.0f; // 0x3102-0x3103

            float batt_v = regs[0x04] / 100.0f; // 0x3104
            float batt_i = regs[0x05] / 100.0f; // 0x3105
            float batt_p = u32_from_regs(regs[0x06], regs[0x07]) / 100.0f; // 0x3106-0x3107

            float load_v = regs[0x0C] / 100.0f; // 0x310C
            float load_i = regs[0x0D] / 100.0f; // 0x310D
            float load_p = u32_from_regs(regs[0x0E], regs[0x0F]) / 100.0f; // 0x310E-0x310F

            // Extended values (if available)
            float batt_temp = (regs_available > 0x10) ? s16(regs[0x10]) / 100.0f : 0.0f; // 0x3110
            float ctrl_temp = (regs_available > 0x11) ? s16(regs[0x11]) / 100.0f : 0.0f; // 0x3111
            float comp_temp = (regs_available > 0x12) ? s16(regs[0x12]) / 100.0f : 0.0f; // 0x3112
            float soc       = (regs_available > 0x1A) ? regs[0x1A] / 100.0f : 0.0f;      // 0x311A
            float sys_v     = (regs_available > 0x1D) ? regs[0x1D] / 100.0f : 0.0f;      // 0x311D

            // Log key values (reduced verbosity)
            ESP_LOGI(TAG, "PV: %.2fV %.2fA %.2fW | BATT: %.2fV %.2fA %.2fW %.1f%% | LOAD: %.2fV %.2fA %.2fW",
                     pv_v, pv_i, pv_p, batt_v, batt_i, batt_p, soc, load_v, load_i, load_p);
            if (regs_available > 0x12) {
                ESP_LOGI(TAG, "TEMP: Batt=%.1fC Ctrl=%.1fC Comp=%.1fC | Sys=%.1fV",
                         batt_temp, ctrl_temp, comp_temp, sys_v);
            }

            // Write to shared data structure for MQTT publishing
            epever_data_t data = {
                .pv_voltage = pv_v,
                .pv_current = pv_i,
                .pv_power = pv_p,
                .battery_voltage = batt_v,
                .battery_current = batt_i,
                .battery_power = batt_p,
                .battery_soc = soc,
                .load_voltage = load_v,
                .load_current = load_i,
                .load_power = load_p,
                .battery_temp = batt_temp,
                .controller_temp = ctrl_temp,
                .component_temp = comp_temp,
                .system_voltage = sys_v,
                .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS,
                .valid = true
            };
            epever_data_write(&data);

        } else {
            ESP_LOGW(TAG, "Not enough registers received to decode block (regs_available=%d)", regs_available);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Booting Epever Modbus RTU master (UART2 TX=%d RX=%d, %d 8N1)",
             MB_TX_GPIO, MB_RX_GPIO, MB_BAUDRATE);

    uart_config_t cfg = {
        .baud_rate = MB_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(MB_UART, MB_BUF_SIZE, MB_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MB_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(MB_UART, MB_TX_GPIO, MB_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // If you later add DE/RE control, we can switch to:
    // uart_set_mode(MB_UART, UART_MODE_RS485_HALF_DUPLEX) + set RTS pin for DE/RE.

    // Initialize shared data structure
    epever_data_init();
    
    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init();
    
    // Initialize MQTT
    ESP_LOGI(TAG, "Initializing MQTT...");
    mqtt_init();
    mqtt_start();
    
    // Create Modbus polling task
    xTaskCreate(poll_task, "modbus_poll", 4096, NULL, 5, NULL);
    
    // Create MQTT publishing task
    xTaskCreate(mqtt_publish_task, "mqtt_publish", 4096, NULL, 4, NULL);
}

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