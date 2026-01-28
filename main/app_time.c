#include "esp_netif.h"
#include "app_time.h"
#include <esp_netif_sntp.h>
#include "esp_log.h"
#include <time.h>

#define TAG "APP_TIME"

void setup_time(void) {
    ESP_LOGI(TAG, "Initializing SNTP");

    esp_sntp_config_t config = {
        .smooth_sync = false,
        .server_from_dhcp = false,
        .wait_for_sync = true,
        .start = true,
        .sync_cb = NULL,
        .renew_servers_after_new_IP = false,
        .ip_event_to_renew = (ip_event_t)0,
        .index_of_first_server = 0,
        .num_of_servers = CONFIG_LWIP_SNTP_MAX_SERVERS
    };
    config.servers[0] = "time.cloudflare.com";
    if (CONFIG_LWIP_SNTP_MAX_SERVERS > 1) {
        config.servers[1] = "time.google.com";
    }
    esp_netif_sntp_init(&config);

    ESP_LOGI(TAG, "SNTP initialized");
}

void update_time(void) {
    ESP_LOGI(TAG, "Waiting for NTP time synchronization...");

    esp_err_t ret = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(30 * 1000));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Time synchronized successfully!");
        time_t now;
        time(&now);
        ESP_LOGI(TAG, "Current timestamp: %ld", (long)now);
    } else {
        ESP_LOGE(TAG, "Failed to synchronize time with NTP server (timeout or network error)");
    }
}
