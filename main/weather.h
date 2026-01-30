#ifndef WEATHER_H
#define WEATHER_H

#include <stdint.h>
#include <stdbool.h>

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
