#include "esp_netif.h"
#include "app_time.h"
#include <esp_netif_sntp.h>
#include "esp_log.h"
#include <time.h>

#define TAG "APP_TIME"

void setup_time(void) {
    ESP_LOGI(TAG, "Initializing SNTP with time.google.com");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("time.google.com");
    esp_netif_sntp_init(&config);
    ESP_LOGI(TAG, "SNTP initialized");
}

void update_time(void) {
    ESP_LOGI(TAG, "Waiting for NTP time synchronization...");

    esp_err_t ret = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Time synchronized successfully!");
        time_t now;
        time(&now);
        ESP_LOGI(TAG, "Current timestamp: %ld", (long)now);
    } else {
        ESP_LOGE(TAG, "Failed to synchronize time with NTP server (timeout or network error)");
    }
}
