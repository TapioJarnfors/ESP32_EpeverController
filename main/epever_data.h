#ifndef EPEVER_DATA_H
#define EPEVER_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @brief Structure to hold all Epever solar charge controller sensor data
 * 
 * This structure is shared between the Modbus polling task and MQTT publishing task.
 * Access must be protected by the mutex to ensure thread safety.
 */
typedef struct {
    // PV (Solar Panel) values
    float pv_voltage;          // Volts
    float pv_current;          // Amps
    float pv_power;            // Watts
    
    // Battery values
    float battery_voltage;     // Volts
    float battery_current;     // Amps
    float battery_power;       // Watts
    float battery_soc;         // State of Charge (%)
    
    // Load values
    float load_voltage;        // Volts
    float load_current;        // Amps
    float load_power;          // Watts
    
    // Temperature values
    float battery_temp;        // Celsius
    float controller_temp;     // Celsius
    float component_temp;      // Celsius
    
    // System rated voltage
    float system_voltage;      // Volts
    
    // Metadata
    uint32_t timestamp_ms;     // Timestamp when data was collected
    bool valid;                // True if data is valid and fresh
} epever_data_t;

/**
 * @brief Initialize the shared data structure and mutex
 * 
 * Must be called once during system initialization before any tasks
 * attempt to access the shared data.
 */
void epever_data_init(void);

/**
 * @brief Write sensor data to the shared structure (thread-safe)
 * 
 * @param data Pointer to the data structure to write
 */
void epever_data_write(const epever_data_t *data);

/**
 * @brief Read sensor data from the shared structure (thread-safe)
 * 
 * @param data Pointer to the data structure to populate
 */
void epever_data_read(epever_data_t *data);

#endif // EPEVER_DATA_H
