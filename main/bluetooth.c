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

// Get manufacturer name from MAC address OUI (first 3 bytes)
static const char* get_manufacturer_from_mac(const esp_bd_addr_t addr) {
    uint32_t oui = (addr[0] << 16) | (addr[1] << 8) | addr[2];

    // Common manufacturers by OUI
    switch (oui) {
        case 0x001124: case 0x001125: case 0x001f5b: case 0x0050e4:
        case 0x0023df: case 0x002608: case 0x003EE1:
        case 0x0056CD: case 0x0088E1: case 0x00F4B9: case 0x1499E2:
        case 0x18F1D8: case 0x20A2E4: case 0x28E0E5: case 0x3451C9:
        case 0x38892C: case 0x40A6D9: case 0x48D2E1: case 0x50DE06:
        case 0x54E4BD: case 0x5C803B: case 0x6476BA: case 0x70DEE2:
        case 0x74E2F5: case 0x787B8A: case 0x7C6193: case 0x7C6D62:
        case 0x8451FB: case 0x888322: case 0x8C7C92: case 0x90B21F:
        case 0x9CDE5E: case 0xA4D1D2: case 0xAC3C0B: case 0xB065BD:
        case 0xB09FBA: case 0xB418D1: case 0xB8782E: case 0xBC6778:
        case 0xC82A14: case 0xCC08E0: case 0xD0D2B0: case 0xD48F33:
        case 0xDC2B2A: case 0xDC56E7: case 0xE0ACCB: case 0xE4C63D:
        case 0xF0D1A9: case 0xF45C89: case 0xF82FA8: case 0xFCFC48:
            return "Apple";

        case 0x000A95: case 0x000E6D: case 0x001D0F: case 0x001E4C:
        case 0x002454: case 0x00E091: case 0x20FD0B:
        case 0x24F677: case 0x30B4B8: case 0x40F021: case 0x4C3275:
        case 0x5C1D6E: case 0x6C4008: case 0x783A84:
        case 0x94A7B7: case 0x9802D8: case 0xA0CE8C: case 0xA86BEC:
        case 0xB0CA68: case 0xD05FB8: case 0xDC1A73: case 0xE805BF:
        case 0xF0A3A2:
            return "Google";

        case 0x000195: case 0x000F86: case 0x001377: case 0x001D6E:
        case 0x001EE1: case 0x001EE2: case 0x002264: case 0x002491:
        case 0x0027E3: case 0x0050F2: case 0x182195:
        case 0x1C4BD6: case 0x20689D: case 0x2CF0A2: case 0x34159E:
        case 0x3C0518: case 0x4065A8: case 0x4439B4: case 0x485929:
        case 0x509EA7: case 0x60F626: case 0x68A3C4: case 0x68C906:
        case 0x708BCD: case 0x78F8DC: case 0x805719: case 0x84A134:
        case 0x8C3BAD: case 0xA41731: case 0xA85B78: case 0xA886DD:
        case 0xB067E0: case 0xB0DF3A: case 0xC86C87: case 0xD0E140:
        case 0xD48564: case 0xE43E12: case 0xEC0BB8:
            return "Samsung";

        case 0x001E8C: case 0x0025BC: case 0x0080E5:
        case 0x00A0D5: case 0x20C9D0: case 0x2C598A: case 0x54271E:
        case 0x5C969D: case 0x90004E: case 0x983B16: case 0xA8FE26:
        case 0xD8EB97: case 0xF0B0E7:
            return "Xiaomi";

        case 0x000DB0: case 0x001F01: case 0x001FE2: case 0x0023AE:
        case 0x002566:
            return "Huawei";

        default:
            return "Unknown";
    }
}

// Compare two BLE addresses
static bool addr_equal(const esp_bd_addr_t a, const esp_bd_addr_t b) {
    return memcmp(a, b, ESP_BD_ADDR_LEN) == 0;
}

// Parse additional advertising data
static void parse_adv_data(ble_device_t *device, uint8_t *adv_data, uint16_t adv_data_len) {
    // Initialize
    device->appearance = 0;
    device->manufacturer_id = 0;
    device->service_count = 0;

    // Set manufacturer name from MAC address first (will be used as fallback)
    const char* mfr = get_manufacturer_from_mac(device->bda);
    strncpy(device->manufacturer_name, mfr, MANUFACTURER_NAME_MAX_LEN);
    device->manufacturer_name[MANUFACTURER_NAME_MAX_LEN] = '\0';  // Ensure null termination

    // Parse advertising data
    uint16_t offset = 0;
    while (offset < adv_data_len) {
        uint8_t length = adv_data[offset];
        if (length == 0 || offset + length >= adv_data_len) break;

        uint8_t type = adv_data[offset + 1];
        uint8_t *data = &adv_data[offset + 2];
        uint8_t data_len = length - 1;

        switch (type) {
            case ESP_BLE_AD_TYPE_APPEARANCE:
                if (data_len >= 2) {
                    device->appearance = data[0] | (data[1] << 8);
                }
                break;

            case ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE:
                if (data_len >= 2) {
                    device->manufacturer_id = data[0] | (data[1] << 8);
                }
                break;

            case ESP_BLE_AD_TYPE_16SRV_CMPL:
            case ESP_BLE_AD_TYPE_16SRV_PART:
                // Parse 16-bit service UUIDs
                for (int i = 0; i < data_len && device->service_count < MAX_SERVICES; i += 2) {
                    if (i + 1 < data_len) {
                        uint16_t uuid = data[i] | (data[i + 1] << 8);
                        device->service_uuids[device->service_count++] = uuid;
                    }
                }
                break;

            case 0x16:  // ESP_BLE_AD_TYPE_SERVICE_DATA (Service Data - 16 bit UUID)
                // Service Data contains UUID + data
                if (data_len >= 2 && device->service_count < MAX_SERVICES) {
                    uint16_t uuid = data[0] | (data[1] << 8);
                    device->service_uuids[device->service_count++] = uuid;
                }
                break;

            case 0x20:  // ESP_BLE_AD_TYPE_SERVICE_DATA_32BIT (Service Data - 32 bit UUID)
            case 0x21:  // ESP_BLE_AD_TYPE_SERVICE_DATA_128BIT (Service Data - 128 bit UUID)
                // Skip 32-bit and 128-bit service data for now
                break;
        }

        offset += length + 1;
    }

    // manufacturer_name was already set at the beginning from MAC address
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

        // Update services if we found new ones
        if (device->service_count > 0) {
            // Merge services - avoid duplicates
            for (int i = 0; i < device->service_count && g_device_list.devices[idx].device.service_count < MAX_SERVICES; i++) {
                bool found = false;
                for (int j = 0; j < g_device_list.devices[idx].device.service_count; j++) {
                    if (g_device_list.devices[idx].device.service_uuids[j] == device->service_uuids[i]) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    g_device_list.devices[idx].device.service_uuids[g_device_list.devices[idx].device.service_count++] = device->service_uuids[i];
                }
            }
        }

        // Update manufacturer info if better data available
        if (device->manufacturer_id != 0 && g_device_list.devices[idx].device.manufacturer_id == 0) {
            g_device_list.devices[idx].device.manufacturer_id = device->manufacturer_id;
        }

        if (strcmp(device->manufacturer_name, "Unknown") != 0 &&
            strcmp(g_device_list.devices[idx].device.manufacturer_name, "Unknown") == 0) {
            strncpy(g_device_list.devices[idx].device.manufacturer_name, device->manufacturer_name, MANUFACTURER_NAME_MAX_LEN);
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
                ESP_LOGI(TAG, "New device added: %02x:%02x:%02x:%02x:%02x:%02x (%s) - %d services",
                         device->bda[0], device->bda[1], device->bda[2],
                         device->bda[3], device->bda[4], device->bda[5],
                         device->name[0] != '\0' ? device->name : device->manufacturer_name,
                         device->service_count);
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
    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE,  // Enable to get all advertisements
    .cfg_mask               = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK,
    .uncoded_cfg            = {
        .scan_type          = BLE_SCAN_TYPE_PASSIVE,  // Try passive scan - catches all advertisements
        .scan_interval      = 0x100,  // 160ms - more time between scans
        .scan_window        = 0x100,  // 160ms - scan continuously (100% duty cycle)
    }
};

// BLE GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(TAG, "GAP event: %d", event);
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan parameters set");
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BLE scan started");
        } else {
            ESP_LOGE(TAG, "Scan start error: %d", param->scan_start_cmpl.status);
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

        // Parse additional advertising data (services, manufacturer, etc.)
        parse_adv_data(&device, scan_result->ext_adv_report.params.adv_data,
                       scan_result->ext_adv_report.params.adv_data_len);

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

            // Parse additional advertising data (services, manufacturer, etc.)
            parse_adv_data(&device, scan_result->scan_rst.ble_adv,
                          scan_result->scan_rst.adv_data_len);

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
            ESP_LOGI(TAG, "Scan stopped");
            g_scanning = false;
        } else {
            ESP_LOGE(TAG, "Scan stop error: %d", param->scan_stop_cmpl.status);
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

// Get human-readable name for common BLE service UUIDs
const char* get_ble_service_name(uint16_t uuid)
{
    switch (uuid) {
        // Standard GATT Services (0x18xx)
        case 0x1800: return "Gen Access";
        case 0x1801: return "Gen Attrib";
        case 0x1802: return "Immed Alert";
        case 0x1803: return "Link Loss";
        case 0x1804: return "Tx Power";
        case 0x1805: return "Time";
        case 0x1806: return "User Data";
        case 0x1807: return "Health Therm";
        case 0x1808: return "Device Info";
        case 0x1809: return "Heart Rate";
        case 0x180A: return "Device Info";
        case 0x180D: return "Heart Rate";
        case 0x180F: return "Battery";
        case 0x1810: return "Blood Press";
        case 0x1811: return "Alert Notif";
        case 0x1812: return "HID";
        case 0x1813: return "Scan Param";
        case 0x1814: return "Running";
        case 0x1815: return "Auto IO";
        case 0x1816: return "Cycling";
        case 0x1818: return "Cycling Pwr";
        case 0x1819: return "Location Nav";
        case 0x181A: return "Environ Sens";
        case 0x181B: return "Body Comp";
        case 0x181C: return "User Data";
        case 0x181D: return "Weight Scl";
        case 0x181E: return "Bond Mgmt";
        case 0x181F: return "Glucose";
        case 0x1820: return "IP Support";
        case 0x1821: return "Indoor Pos";
        case 0x1822: return "Pulse Oxim";
        case 0x1823: return "HTTP Proxy";
        case 0x1824: return "Transport";
        case 0x1825: return "Object Tran";
        case 0x1826: return "Fitness Mac";

        // Vendor-specific Services (0xFExx, 0xFDxx, etc.)
        // Google
        case 0xFE2C: return "Chromecast";
        case 0xFE9F: return "Google";
        case 0xFEAA: return "Google";

        // Xiaomi
        case 0xFE95: return "Xiaomi";

        // Apple
        case 0xFDCD: return "Apple Media";
        case 0xFE07: return "Apple";
        case 0xFE2B: return "Apple";
        case 0xD0FF: return "Apple Nearby";

        // Fitness & Sports devices
        case 0xFEB9: return "Garmin";
        case 0xFEBA: return "Fitbit";
        case 0xFEF3: return "Polar";
        case 0xFE26: return "Suunto";
        case 0xFEE7: return "Wahoo Fitness";

        // Samsung
        case 0xFD5A: return "Samsung";

        // Tile (item tracker)
        case 0xFEED: return "Tile";

        // Nordic UART Service (very common for DIY/dev devices)
        case 0x6E40: return "Nordic UART";

        // Exposure Notification (COVID tracing)
        case 0xFD6F: return "Exposure Notif";

        // Amazon
        case 0xFE03: return "Amazon";

        // Microsoft
        case 0xFDF1: return "Microsoft";
        case 0xFDF2: return "MS Swift Pair";

        default:
            return NULL;
    }
}