#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

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

static void poll_task(void *arg)
{
    (void)arg;

    uint8_t req[8];
    uint8_t resp[MB_MAX_FRAME];

    // Your first test: 01 04 31 00 00 01 CRC
    const uint16_t start = 0x3100;
    const uint16_t count = 1;

    while (1) {
        uint16_t req_len = build_read_input_regs(MB_SLAVE_ID, start, count, req);

        ESP_LOGI(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X",
                 req[0], req[1], req[2], req[3], req[4], req[5], req[6], req[7]);

        if (uart_send_frame(req, req_len) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Expected response length for FC=0x04, count=1:
        // [id][fc][bytecount=2][data_hi][data_lo][crc_lo][crc_hi] = 7 bytes
        int got = uart_read_frame(resp, sizeof(resp), 7);

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

        // Basic Modbus checks
        if (got >= 5 && resp[0] == MB_SLAVE_ID && resp[1] == 0x04) {
            uint8_t bytecount = resp[2];

            // Exception response: fc | 0x80
            if (resp[1] & 0x80) {
                ESP_LOGE(TAG, "Modbus exception code=0x%02X", resp[2]);
            } else if (bytecount == 2 && got >= 7) {
                uint16_t reg = ((uint16_t)resp[3] << 8) | (uint16_t)resp[4];

                // 0x3100 PV array voltage: ÷100 V
                float pv_v = ((float)reg) / 100.0f;
                ESP_LOGI(TAG, "0x3100 PV voltage raw=0x%04X => %.2f V", reg, pv_v);
            } else {
                ESP_LOGW(TAG, "Unexpected bytecount=%u (got len=%d)", bytecount, got);
            }
        } else {
            ESP_LOGW(TAG, "Unexpected response header");
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

    xTaskCreate(poll_task, "modbus_poll", 4096, NULL, 5, NULL);
}