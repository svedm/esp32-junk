#include "weather.h"
#include "network.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "WEATHER";
static weather_callback_t user_callback = NULL;

#define API_KEY CONFIG_GOOGLE_API_KEY

// Внутренний коллбэк для обработки HTTP ответа
static void http_response_callback(int status_code, const char *response) {
    ESP_LOGI(TAG, "HTTP Response: status=%d", status_code);

    if (status_code != 200 || response == NULL) {
        ESP_LOGE(TAG, "Failed to fetch weather data: status=%d", status_code);
        if (response) {
            ESP_LOGE(TAG, "Response: %s", response);
        }
        if (user_callback) {
            weather_data_t invalid_weather = {0};
            invalid_weather.is_valid = false;
            user_callback(&invalid_weather);
        }
        return;
    }

    cJSON *root = cJSON_Parse(response);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON response");
        if (user_callback) {
            weather_data_t invalid_weather = {0};
            invalid_weather.is_valid = false;
            user_callback(&invalid_weather);
        }
        return;
    }

    weather_data_t weather = {0};
    weather.is_valid = true;

    // Температура
    cJSON *temperature = cJSON_GetObjectItem(root, "temperature");
    if (temperature) {
        cJSON *degrees = cJSON_GetObjectItem(temperature, "degrees");
        if (degrees && cJSON_IsNumber(degrees)) {
            weather.temperature = (float)degrees->valuedouble;
            ESP_LOGI(TAG, "Temperature: %.1f°C", weather.temperature);
        }
    }

    // Ощущаемая температура
    cJSON *feels_like = cJSON_GetObjectItem(root, "feelsLikeTemperature");
    if (feels_like) {
        cJSON *degrees = cJSON_GetObjectItem(feels_like, "degrees");
        if (degrees && cJSON_IsNumber(degrees)) {
            weather.feels_like = (float)degrees->valuedouble;
            ESP_LOGI(TAG, "Feels like: %.1f°C", weather.feels_like);
        }
    }

    // Влажность
    cJSON *humidity = cJSON_GetObjectItem(root, "relativeHumidity");
    if (humidity && cJSON_IsNumber(humidity)) {
        weather.humidity = (uint8_t)humidity->valueint;
        ESP_LOGI(TAG, "Humidity: %d%%", weather.humidity);
    }

    // Ветер
    cJSON *wind = cJSON_GetObjectItem(root, "wind");
    if (wind) {
        // Направление ветра
        cJSON *direction = cJSON_GetObjectItem(wind, "direction");
        if (direction) {
            cJSON *degrees = cJSON_GetObjectItem(direction, "degrees");
            if (degrees && cJSON_IsNumber(degrees)) {
                weather.wind_direction = (uint16_t)degrees->valueint;
                ESP_LOGI(TAG, "Wind direction: %d°", weather.wind_direction);
            }
        }

        // Скорость ветра (км/ч -> узлы: 1 км/ч = 0.539957 узла)
        cJSON *speed = cJSON_GetObjectItem(wind, "speed");
        if (speed) {
            cJSON *value = cJSON_GetObjectItem(speed, "value");
            if (value && cJSON_IsNumber(value)) {
                float kmh = (float)value->valuedouble;
                weather.wind_speed = kmh * 0.539957f;
                ESP_LOGI(TAG, "Wind speed: %.1f kn (%.1f km/h)", weather.wind_speed, kmh);
            }
        }

        // Порывы ветра
        cJSON *gust = cJSON_GetObjectItem(wind, "gust");
        if (gust) {
            cJSON *value = cJSON_GetObjectItem(gust, "value");
            if (value && cJSON_IsNumber(value)) {
                float kmh = (float)value->valuedouble;
                weather.wind_gusts = kmh * 0.539957f;
                ESP_LOGI(TAG, "Wind gusts: %.1f kn (%.1f km/h)", weather.wind_gusts, kmh);
            }
        }
    }

    // Осадки
    cJSON *precipitation = cJSON_GetObjectItem(root, "precipitation");
    if (precipitation) {
        cJSON *qpf = cJSON_GetObjectItem(precipitation, "qpf");
        if (qpf) {
            cJSON *quantity = cJSON_GetObjectItem(qpf, "quantity");
            if (quantity && cJSON_IsNumber(quantity)) {
                weather.precipitation = (float)quantity->valuedouble;
                weather.rain = weather.precipitation;
                ESP_LOGI(TAG, "Precipitation: %.1f mm", weather.precipitation);
            }
        }
    }

    cJSON_Delete(root);

    // Вызываем коллбэк пользователя
    if (user_callback) {
        user_callback(&weather);
    }
}

void get_weather(double latitude, double longitude, weather_callback_t callback) {
    ESP_LOGI(TAG, "Fetching weather data...");
    user_callback = callback;

    // Если координаты не переданы (равны 0), используем Санкт-Петербург
    if (latitude == 0.0 && longitude == 0.0) {
        latitude = WEATHER_DEFAULT_LATITUDE;
        longitude = WEATHER_DEFAULT_LONGITUDE;
        ESP_LOGI(TAG, "Using default coordinates (Saint Petersburg)");
    }

    char url[256];
    snprintf(url, sizeof(url),
        "https://weather.googleapis.com/v1/currentConditions:lookup"
        "?key=%s&location.latitude=%.6f&location.longitude=%.6f",
        API_KEY, latitude, longitude);

    ESP_LOGI(TAG, "Requesting weather for lat=%.6f, lon=%.6f", latitude, longitude);

    http_get(url, http_response_callback);
}
