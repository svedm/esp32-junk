#include "esp_wifi.h"

#define DEFAULT_SCAN_LIST_SIZE 15

typedef void (*wifi_ready_callback_t)(void);

void setup_nvs();
void scan_wifi(wifi_ap_record_t* ap_info, uint16_t* number);
void connect_to_wifi();
void wifi_set_ready_callback(wifi_ready_callback_t callback);