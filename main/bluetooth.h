#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include <time.h>
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define BLE_DEVICE_NAME_MAX_LEN 31
#define MAX_TRACKED_DEVICES 50
#define DEVICE_TIMEOUT_SEC 300  // 5 minutes
#define MAX_SERVICES 8
#define MANUFACTURER_NAME_MAX_LEN 20

// BLE device info from scan
typedef struct {
    esp_bd_addr_t bda;                      // Bluetooth Device Address
    int8_t rssi;                            // Signal strength (dBm)
    char name[BLE_DEVICE_NAME_MAX_LEN + 1]; // Device name
    esp_ble_addr_type_t addr_type;          // Address type (public/random)
    uint16_t appearance;                    // Device appearance
    uint16_t manufacturer_id;               // Manufacturer company ID
    char manufacturer_name[MANUFACTURER_NAME_MAX_LEN + 1]; // Manufacturer name
    uint16_t service_uuids[MAX_SERVICES];   // List of 16-bit service UUIDs
    uint8_t service_count;                  // Number of services found
} ble_device_t;

// Tracked device with timestamp
typedef struct {
    ble_device_t device;
    time_t last_seen;                       // Timestamp of last detection
    time_t first_seen;                      // Timestamp when first detected
    bool is_active;                         // Device slot is in use
} tracked_device_t;

// Device list with mutex protection
typedef struct {
    tracked_device_t devices[MAX_TRACKED_DEVICES];
    SemaphoreHandle_t mutex;
    int count;
} device_list_t;

// Initialize BLE stack
void setup_bluetooth(void);

// Start BLE scanning task
void start_ble_scan_task(void);

// Get device list for display
device_list_t* get_device_list(void);

// Get human-readable name for common BLE service UUIDs
const char* get_ble_service_name(uint16_t uuid);

#endif // BLUETOOTH_H