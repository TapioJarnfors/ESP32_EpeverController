#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_log_level.h"

static const char *TAG = "rtu_sniffer";

#define DEBUG              true // true for debug logging, false for production 

// RS-485 sniff input UART (UART2)
#define SNIFF_UART         UART_NUM_2
#define SNIFF_RX_GPIO      16
#define SNIFF_TX_GPIO      17   // unused

// Output to PC over USB serial (UART0)
#define OUT_UART           UART_NUM_0

// Rates
#define SNIFF_BAUDRATE     115200
#define OUT_BAUDRATE       460800

// Buffering
#define SNIFF_BUF_SIZE     4096
#define UART_EVENT_QUEUE_SIZE 20

// Modbus RTU framing
// 115200 8N1 => 1 char ~ 86.8 us; 3.5 chars ~ 304 us.
// UART RX timeout: configured in symbol times (characters)
#define RTU_IDLE_THRESH_SYMBOLS  4  // ~4 character times for frame boundary

// Frame buffer limits
#define RTU_FRAME_MAX      512

// Frame tracking
static uint32_t frame_count = 0;
static uint32_t frame_start_ts_us = 0;
static uint32_t frame_end_ts_us = 0;

// UART event queue
static QueueHandle_t uart_queue;

static void write_record_to_pc(uint32_t ts_us, uint8_t flags, const uint8_t *data, uint16_t len)
{
    // Header: sync(2) + ts(4) + flags(1) + len(2) = 9 bytes
    uint8_t hdr[8];
    hdr[0] = 0xA5;
    hdr[1] = 0x5A;

    hdr[2] = (uint8_t)(ts_us & 0xFF);
    hdr[3] = (uint8_t)((ts_us >> 8) & 0xFF);
    hdr[4] = (uint8_t)((ts_us >> 16) & 0xFF);
    hdr[5] = (uint8_t)((ts_us >> 24) & 0xFF);

//*    hdr[6] = flags; // flags reserved for future use

    hdr[7] = (uint8_t)(len & 0xFF);
    hdr[8] = (uint8_t)((len >> 8) & 0xFF);

//* write to file    uart_write_bytes(OUT_UART, (const char *)hdr, sizeof(hdr));
//* write to file    uart_write_bytes(OUT_UART, (const char *)data, len);
    
    if (DEBUG) {
        ESP_LOGI(TAG, "Frame %u: ts=%u us, data_len=%u, flags=0x%02X", ++frame_count, ts_us, len, flags);
        ESP_LOG_BUFFER_HEX(TAG, data, len);
    }
}

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

static void handle_frame(const uint8_t *frame, uint16_t len, uint32_t frame_start_us, uint32_t frame_end_us)
{
    (void)frame_start_us;

    if (len < 4) return;

    uint16_t rx_crc = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    uint16_t calc_crc = modbus_crc16(frame, (uint16_t)(len - 2));
    bool crc_ok = (rx_crc == calc_crc);

    uint8_t flags = 0;
    if (crc_ok) {
        flags |= 0x01; // bit0 = crc_ok
    }

    write_record_to_pc(frame_end_us, flags, frame, len);
}

static void sniff_task(void *arg)
{
    uint8_t frame[RTU_FRAME_MAX];
    uint16_t frame_len = 0;
    uart_event_t event;

    while (1) {
        // Wait for UART event (blocks, allowing other tasks to run)
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    // Data available - read it
                    if (frame_len == 0) {
                        frame_start_ts_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFu);
                    }
                    
                    // Read the data that triggered this event
                    size_t to_read = (event.size > (RTU_FRAME_MAX - frame_len)) ? 
                                    (RTU_FRAME_MAX - frame_len) : event.size;
                    int len = uart_read_bytes(SNIFF_UART, frame + frame_len, to_read, 0);
                    if (len > 0) {
                        frame_len += len;
                        frame_end_ts_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFu);
                    }
                    
                    // Check if there's more data immediately available
                    size_t available = 0;
                    uart_get_buffered_data_len(SNIFF_UART, &available);
                    
                    // If no more data is available, this is the end of a frame (RX timeout occurred)
                    if (available == 0 && frame_len > 0) {
                        handle_frame(frame, frame_len, frame_start_ts_us, frame_end_ts_us);
                        frame_len = 0;
                        frame_start_ts_us = 0;
                        frame_end_ts_us = 0;
                    }
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer full - flushing");
                    uart_flush_input(SNIFF_UART);
                    frame_len = 0;
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow - flushing");
                    uart_flush_input(SNIFF_UART);
                    frame_len = 0;
                    break;
                    // test row from github copilot

                default:
                    break;
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Booting Modbus RTU sniffer (UART HW idle detection)");
    ESP_LOGI(TAG, "Sniff UART2: %d 8N1 (RX GPIO%d). Output UART0: %d baud.",
             SNIFF_BAUDRATE, SNIFF_RX_GPIO, OUT_BAUDRATE);
    ESP_LOGI(TAG, "UART RX idle threshold: %u symbol times", RTU_IDLE_THRESH_SYMBOLS);

    uart_config_t sniff_cfg = {
        .baud_rate = SNIFF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        .rx_flow_ctrl_thresh = 122,
    };

    // Install UART driver with event queue
    ESP_ERROR_CHECK(uart_driver_install(SNIFF_UART, SNIFF_BUF_SIZE, 0, UART_EVENT_QUEUE_SIZE, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(SNIFF_UART, &sniff_cfg));
    ESP_ERROR_CHECK(uart_set_pin(SNIFF_UART, SNIFF_TX_GPIO, SNIFF_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Set RX timeout threshold (in symbol times)
    ESP_ERROR_CHECK(uart_set_rx_timeout(SNIFF_UART, RTU_IDLE_THRESH_SYMBOLS));

    // Install UART0 driver and set higher baud rate for output
    ESP_ERROR_CHECK(uart_driver_install(OUT_UART, SNIFF_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_baudrate(OUT_UART, OUT_BAUDRATE));

    xTaskCreate(sniff_task, "sniff_task", 4096, NULL, 5, NULL);
}