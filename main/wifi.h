#include "esp_wifi.h"

#define DEFAULT_SCAN_LIST_SIZE 15

void setup_nvs();
void scan_wifi(wifi_ap_record_t* ap_info, uint16_t* number);