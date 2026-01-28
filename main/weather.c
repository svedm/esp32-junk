#include "weather.h"
#include "network.h"
#include "esp_log.h"
#include "json_parser.h"
#include <string.h>

static const char *TAG = "WEATHER";
static weather_callback_t user_callback = NULL;

// Внутренний коллбэк для обработки HTTP ответа
static void http_response_callback(int status_code, const char *response) {
    ESP_LOGI(TAG, "HTTP Response: status=%d", status_code);

    if (status_code != 200 || response == NULL) {
        ESP_LOGE(TAG, "Failed to fetch weather data: status=%d", status_code);
        if (user_callback) {
            weather_data_t invalid_weather = {0};
            invalid_weather.is_valid = false;
            user_callback(&invalid_weather);
        }
        return;
    }

    // Парсим JSON
    jparse_ctx_t jctx;
    int ret = json_parse_start(&jctx, response, strlen(response));
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to start JSON parsing");
        if (user_callback) {
            weather_data_t invalid_weather = {0};
            invalid_weather.is_valid = false;
            user_callback(&invalid_weather);
        }
        return;
    }

    weather_data_t weather = {0};
    weather.is_valid = true;

    // Парсим поля из JSON
    // Ищем объект "current"
    ret = json_obj_get_object(&jctx, "current");
    if (ret == 0) {
        // Температура
        float temp;
        if (json_obj_get_float(&jctx, "temperature_2m", &temp) == 0) {
            weather.temperature = temp;
            ESP_LOGI(TAG, "Temperature: %.1f°C", temp);
        }

        // Скорость ветра
        float wind_sp;
        if (json_obj_get_float(&jctx, "wind_speed_10m", &wind_sp) == 0) {
            weather.wind_speed = wind_sp;
            ESP_LOGI(TAG, "Wind speed: %.1f kn", wind_sp);
        }

        // Направление ветра
        int wind_dir;
        if (json_obj_get_int(&jctx, "wind_direction_10m", &wind_dir) == 0) {
            weather.wind_direction = (uint16_t)wind_dir;
            ESP_LOGI(TAG, "Wind direction: %d°", wind_dir);
        }

        // Порывы ветра
        float wind_g;
        if (json_obj_get_float(&jctx, "wind_gusts_10m", &wind_g) == 0) {
            weather.wind_gusts = wind_g;
            ESP_LOGI(TAG, "Wind gusts: %.1f kn", wind_g);
        }

        // Осадки
        float precip;
        if (json_obj_get_float(&jctx, "precipitation", &precip) == 0) {
            weather.precipitation = precip;
            ESP_LOGI(TAG, "Precipitation: %.1f mm", precip);
        }

        // Дождь
        float r;
        if (json_obj_get_float(&jctx, "rain", &r) == 0) {
            weather.rain = r;
            ESP_LOGI(TAG, "Rain: %.1f mm", r);
        }

        // Ливни
        float sh;
        if (json_obj_get_float(&jctx, "showers", &sh) == 0) {
            weather.showers = sh;
            ESP_LOGI(TAG, "Showers: %.1f mm", sh);
        }

        // Снег
        float snow;
        if (json_obj_get_float(&jctx, "snowfall", &snow) == 0) {
            weather.snowfall = snow;
            ESP_LOGI(TAG, "Snowfall: %.1f cm", snow);
        }

        // Влажность
        int hum;
        if (json_obj_get_int(&jctx, "relative_humidity_2m", &hum) == 0) {
            weather.humidity = (uint8_t)hum;
            ESP_LOGI(TAG, "Humidity: %d%%", hum);
        }

        json_obj_leave_object(&jctx);
    } else {
        ESP_LOGE(TAG, "Failed to find 'current' object in JSON");
        weather.is_valid = false;
    }

    json_parse_end(&jctx);

    // Вызываем коллбэк пользователя
    if (user_callback) {
        user_callback(&weather);
    }
}

void get_weather(weather_callback_t callback) {
    ESP_LOGI(TAG, "Fetching weather data...");
    user_callback = callback;

    // URL для API Open-Meteo (Санкт-Петербург)
    const char *url = "https://api.open-meteo.com/v1/forecast?"
                     "latitude=0&longitude=0"
                     "&hourly=temperature_2m"
                     "&current=temperature_2m,rain,showers,snowfall,precipitation,"
                     "wind_speed_10m,wind_direction_10m,wind_gusts_10m,relative_humidity_2m"
                     "&timezone=Europe%2FMoscow&forecast_days=1&wind_speed_unit=kn";

    http_get(url, http_response_callback);
}
