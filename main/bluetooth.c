#include "bluetooth.h"
#include <string.h>
#include <sys/time.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/task.h"

static const char *TAG = "BLE";

// Global device list
static device_list_t g_device_list = {
    .mutex = NULL,
    .count = 0
};

static bool g_scanning = false;

// Compare two BLE addresses
static bool addr_equal(const esp_bd_addr_t a, const esp_bd_addr_t b) {
    return memcmp(a, b, ESP_BD_ADDR_LEN) == 0;
}

// Find device in list by address
static int find_device_index(const esp_bd_addr_t addr) {
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        if (g_device_list.devices[i].is_active &&
            addr_equal(g_device_list.devices[i].device.bda, addr)) {
            return i;
        }
    }
    return -1;
}

// Add or update device in the list
static void add_or_update_device(ble_device_t *device) {
    if (!g_device_list.mutex) return;

    xSemaphoreTake(g_device_list.mutex, portMAX_DELAY);

    time_t now = time(NULL);
    int idx = find_device_index(device->bda);

    if (idx >= 0) {
        // Update existing device
        g_device_list.devices[idx].device.rssi = device->rssi;
        g_device_list.devices[idx].last_seen = now;

        // Update name if we got one and didn't have it before
        if (device->name[0] != '\0' && g_device_list.devices[idx].device.name[0] == '\0') {
            strncpy(g_device_list.devices[idx].device.name, device->name, BLE_DEVICE_NAME_MAX_LEN);
        }
    } else {
        // Find empty slot
        for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
            if (!g_device_list.devices[i].is_active) {
                memcpy(&g_device_list.devices[i].device, device, sizeof(ble_device_t));
                g_device_list.devices[i].first_seen = now;
                g_device_list.devices[i].last_seen = now;
                g_device_list.devices[i].is_active = true;
                g_device_list.count++;
                ESP_LOGI(TAG, "New device added: %02x:%02x:%02x:%02x:%02x:%02x",
                         device->bda[0], device->bda[1], device->bda[2],
                         device->bda[3], device->bda[4], device->bda[5]);
                break;
            }
        }
    }

    xSemaphoreGive(g_device_list.mutex);
}

// Remove old devices (not seen for >5 minutes)
static void cleanup_old_devices(void) {
    if (!g_device_list.mutex) return;

    xSemaphoreTake(g_device_list.mutex, portMAX_DELAY);

    time_t now = time(NULL);

    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        if (g_device_list.devices[i].is_active) {
            time_t age = now - g_device_list.devices[i].last_seen;

            if (age > DEVICE_TIMEOUT_SEC) {
                ESP_LOGI(TAG, "Removing old device: %02x:%02x:%02x:%02x:%02x:%02x (age: %lld sec)",
                         g_device_list.devices[i].device.bda[0],
                         g_device_list.devices[i].device.bda[1],
                         g_device_list.devices[i].device.bda[2],
                         g_device_list.devices[i].device.bda[3],
                         g_device_list.devices[i].device.bda[4],
                         g_device_list.devices[i].device.bda[5],
                         (long long)age);

                g_device_list.devices[i].is_active = false;
                g_device_list.count--;
            }
        }
    }

    xSemaphoreGive(g_device_list.mutex);
}

// Extended scan parameters for BLE 5.0
static esp_ble_ext_scan_params_t ext_scan_params = {
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy          = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE,
    .cfg_mask               = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK,
    .uncoded_cfg            = {
        .scan_type          = BLE_SCAN_TYPE_ACTIVE,
        .scan_interval      = 0x50,  // 50ms
        .scan_window        = 0x30,  // 30ms
    }
};

// Обработчик событий BLE GAP
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(TAG, "GAP event: %d", event);
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Параметры сканирования установлены");
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Сканирование BLE запущено");
        } else {
            ESP_LOGE(TAG, "Ошибка запуска сканирования: %d", param->scan_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
        // Extended scan result (BLE 5.0)
        esp_ble_gap_cb_param_t *scan_result = param;
        ble_device_t device;

        // Copy device address
        memcpy(device.bda, scan_result->ext_adv_report.params.addr, ESP_BD_ADDR_LEN);

        // RSSI
        device.rssi = scan_result->ext_adv_report.params.rssi;

        // Address type
        device.addr_type = scan_result->ext_adv_report.params.addr_type;

        // Extract device name from advertising data
        device.name[0] = '\0';
        uint8_t *adv_name = NULL;
        uint8_t adv_name_len = 0;

        adv_name = esp_ble_resolve_adv_data(scan_result->ext_adv_report.params.adv_data,
                                            ESP_BLE_AD_TYPE_NAME_CMPL,
                                            &adv_name_len);

        if (adv_name) {
            if (adv_name_len > BLE_DEVICE_NAME_MAX_LEN) {
                adv_name_len = BLE_DEVICE_NAME_MAX_LEN;
            }
            memcpy(device.name, adv_name, adv_name_len);
            device.name[adv_name_len] = '\0';
        } else {
            // Try short name
            adv_name = esp_ble_resolve_adv_data(scan_result->ext_adv_report.params.adv_data,
                                                ESP_BLE_AD_TYPE_NAME_SHORT,
                                                &adv_name_len);
            if (adv_name) {
                if (adv_name_len > BLE_DEVICE_NAME_MAX_LEN) {
                    adv_name_len = BLE_DEVICE_NAME_MAX_LEN;
                }
                memcpy(device.name, adv_name, adv_name_len);
                device.name[adv_name_len] = '\0';
            }
        }

        // Add or update device in list
        add_or_update_device(&device);
        break;
    }

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        // Regular scan result (fallback for BLE 4.x)
        esp_ble_gap_cb_param_t *scan_result = param;

        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            ble_device_t device;

            // Copy device address
            memcpy(device.bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);

            // RSSI
            device.rssi = scan_result->scan_rst.rssi;

            // Address type
            device.addr_type = scan_result->scan_rst.ble_addr_type;

            // Extract device name from advertising data
            device.name[0] = '\0';
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;

            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL,
                                                &adv_name_len);

            if (adv_name) {
                if (adv_name_len > BLE_DEVICE_NAME_MAX_LEN) {
                    adv_name_len = BLE_DEVICE_NAME_MAX_LEN;
                }
                memcpy(device.name, adv_name, adv_name_len);
                device.name[adv_name_len] = '\0';
            } else {
                // Try short name
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                    ESP_BLE_AD_TYPE_NAME_SHORT,
                                                    &adv_name_len);
                if (adv_name) {
                    if (adv_name_len > BLE_DEVICE_NAME_MAX_LEN) {
                        adv_name_len = BLE_DEVICE_NAME_MAX_LEN;
                    }
                    memcpy(device.name, adv_name, adv_name_len);
                    device.name[adv_name_len] = '\0';
                }
            }

            // Add or update device in list
            add_or_update_device(&device);
            break;
        }

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG, "Scan complete");
            g_scanning = false;
            break;

        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Сканирование остановлено");
            g_scanning = false;
        } else {
            ESP_LOGE(TAG, "Ошибка остановки сканирования: %d", param->scan_stop_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_SCAN_TIMEOUT_EVT:
        ESP_LOGI(TAG, "Scan timeout - scan duration expired");
        g_scanning = false;
        break;

    default:
        break;
    }
}

void setup_bluetooth(void)
{
    ESP_LOGI(TAG, "Initializing BLE");

    // Create mutex for device list
    g_device_list.mutex = xSemaphoreCreateMutex();
    if (!g_device_list.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Initialize device list
    memset(g_device_list.devices, 0, sizeof(g_device_list.devices));
    g_device_list.count = 0;

    // Initialize NVS (required for Bluetooth)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release memory for Classic Bluetooth (use BLE only)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Enable controller in BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Enable Bluedroid stack
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register BLE GAP callback
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP callback registration failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "BLE initialized successfully");
}

// BLE scanning task (runs every 10 seconds)
static void ble_scan_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BLE scan task started");

    while (1) {
        if (!g_scanning) {
            // Set extended scan parameters (BLE 5.0)
            esp_err_t ret = esp_ble_gap_set_ext_scan_params(&ext_scan_params);
            if (ret) {
                ESP_LOGE(TAG, "Set scan params failed: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "Scan params set, starting scan...");
                // Start extended scanning (duration: 10 seconds)
                ret = esp_ble_gap_start_ext_scan(10, 0);
                if (ret) {
                    ESP_LOGE(TAG, "Start scan failed: %s", esp_err_to_name(ret));
                } else {
                    ESP_LOGI(TAG, "BLE scan started");
                    g_scanning = true;
                }
            }
        }

        // Cleanup old devices every scan cycle
        cleanup_old_devices();

        // Wait 10 seconds before next scan
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Start BLE scanning task
void start_ble_scan_task(void)
{
    xTaskCreate(ble_scan_task, "ble_scan", 4096, NULL, 5, NULL);
}

// Get pointer to device list for display
device_list_t* get_device_list(void)
{
    return &g_device_list;
}