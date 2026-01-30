#ifndef WEATHER_H
#define WEATHER_H

#include <stdint.h>
#include <stdbool.h>

// Типы иконок погоды
typedef enum {
    WEATHER_ICON_UNKNOWN = 0,
    WEATHER_ICON_CLEAR_DAY,
    WEATHER_ICON_CLEAR_NIGHT,
    WEATHER_ICON_PARTLY_CLOUDY_DAY,
    WEATHER_ICON_PARTLY_CLOUDY_NIGHT,
    WEATHER_ICON_CLOUDY,
    WEATHER_ICON_RAIN_LIGHT,
    WEATHER_ICON_RAIN_HEAVY,
    WEATHER_ICON_SNOW_LIGHT,
    WEATHER_ICON_SNOW_HEAVY,
    WEATHER_ICON_THUNDERSTORM,
    WEATHER_ICON_FOG,
    WEATHER_ICON_WIND,
    WEATHER_ICON_COUNT
} weather_icon_t;

// Структура для хранения данных о погоде
typedef struct {
    float temperature;           // Текущая температура (°C)
    float feels_like;           // Ощущаемая температура (°C)
    float wind_speed;           // Скорость ветра (узлы)
    uint16_t wind_direction;    // Направление ветра (градусы)
    float wind_gusts;           // Порывы ветра (узлы)
    float precipitation;        // Осадки (мм)
    float rain;                 // Дождь (мм)
    float showers;              // Ливни (мм)
    float snowfall;             // Снег (см)
    uint8_t humidity;           // Влажность (%)
    weather_icon_t icon;        // Тип иконки погоды
    bool is_valid;              // Флаг корректности данных
} weather_data_t;

// Тип коллбэка для получения данных о погоде
typedef void (*weather_callback_t)(weather_data_t *weather);

// Координаты центра Санкт-Петербурга по умолчанию
#define WEATHER_DEFAULT_LATITUDE  59.9343
#define WEATHER_DEFAULT_LONGITUDE 30.3351

// Функция для получения данных о погоде
// Если latitude или longitude равны 0, используются координаты Санкт-Петербурга
void get_weather(double latitude, double longitude, weather_callback_t callback);

#endif // WEATHER_H
