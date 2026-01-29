#include "geolocation.h"
#include "network.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "cJSON.h"

#define TAG "GEOLOCATION"
#define API_KEY CONFIG_GOOGLE_API_KEY

typedef struct {
    geolocation_callback_t user_callback;
} geolocation_context_t;

static geolocation_context_t g_geo_context;

static void geolocation_response_handler(int status_code, const char *body) {
    geolocation_data_t geo = {.latitude = 0.0, .longitude = 0.0};

    if (status_code != 200) {
        ESP_LOGE(TAG, "Geolocation API returned status code: %d", status_code);
        if (g_geo_context.user_callback) {
            g_geo_context.user_callback(&geo);
        }
        return;
    }

    cJSON *root = cJSON_Parse(body);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON response");
        if (g_geo_context.user_callback) {
            g_geo_context.user_callback(&geo);
        }
        return;
    }

    cJSON *location = cJSON_GetObjectItem(root, "location");
    if (location == NULL) {
        ESP_LOGE(TAG, "No location field in response");
        cJSON_Delete(root);
        if (g_geo_context.user_callback) {
            g_geo_context.user_callback(&geo);
        }
        return;
    }

    cJSON *lat = cJSON_GetObjectItem(location, "lat");
    cJSON *lng = cJSON_GetObjectItem(location, "lng");

    if (lat == NULL || lng == NULL) {
        ESP_LOGE(TAG, "Missing lat/lng in response");
        cJSON_Delete(root);
        if (g_geo_context.user_callback) {
            g_geo_context.user_callback(&geo);
        }
        return;
    }

    geo.latitude = lat->valuedouble;
    geo.longitude = lng->valuedouble;

    ESP_LOGI(TAG, "Geolocation: lat=%.6f, lng=%.6f", geo.latitude, geo.longitude);

    cJSON_Delete(root);

    if (g_geo_context.user_callback) {
        g_geo_context.user_callback(&geo);
    }
}

void get_geolocation(wifi_ap_record_t* ap_info, int count, geolocation_callback_t callback) {
    char url[256];
    snprintf(
        url,
        sizeof(url),
        "https://www.googleapis.com/geolocation/v1/geolocate?key=%s",
        API_KEY
    );

    g_geo_context.user_callback = callback;

    cJSON *root = cJSON_CreateObject();
    cJSON *wifi_access_points = cJSON_CreateArray();

    for (int i = 0; i < count; i++) {
        if (ap_info[i].rssi == 0) {
            break;
        }

        cJSON *ap = cJSON_CreateObject();

        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 ap_info[i].bssid[0], ap_info[i].bssid[1], ap_info[i].bssid[2],
                 ap_info[i].bssid[3], ap_info[i].bssid[4], ap_info[i].bssid[5]);

        cJSON_AddStringToObject(ap, "macAddress", mac_str);
        cJSON_AddNumberToObject(ap, "signalStrength", ap_info[i].rssi);
        cJSON_AddNumberToObject(ap, "channel", ap_info[i].primary);

        cJSON_AddItemToArray(wifi_access_points, ap);
    }

    cJSON_AddItemToObject(root, "wifiAccessPoints", wifi_access_points);

    char *json_str = cJSON_PrintUnformatted(root);
    ESP_LOGI(TAG, "Request JSON: %s", json_str);

    http_post(url, json_str, geolocation_response_handler);

    cJSON_Delete(root);
    free(json_str);
}

