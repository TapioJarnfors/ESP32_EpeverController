#include "epever_data.h"
#include <string.h>

// Global shared data structure
static epever_data_t g_epever_data = {0};

// Mutex for thread-safe access
static SemaphoreHandle_t g_data_mutex = NULL;

void epever_data_init(void)
{
    // Create mutex for thread-safe access
    g_data_mutex = xSemaphoreCreateMutex();
    
    // Initialize data structure with zeros
    memset(&g_epever_data, 0, sizeof(epever_data_t));
    g_epever_data.valid = false;
}

void epever_data_write(const epever_data_t *data)
{
    if (g_data_mutex != NULL && data != NULL) {
        // Take mutex with timeout
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Copy data to shared structure
            memcpy(&g_epever_data, data, sizeof(epever_data_t));
            
            // Release mutex
            xSemaphoreGive(g_data_mutex);
        }
    }
}

void epever_data_read(epever_data_t *data)
{
    if (g_data_mutex != NULL && data != NULL) {
        // Take mutex with timeout
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Copy shared structure to output
            memcpy(data, &g_epever_data, sizeof(epever_data_t));
            
            // Release mutex
            xSemaphoreGive(g_data_mutex);
        }
    }
}
