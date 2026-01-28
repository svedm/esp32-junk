#ifndef WEATHER_H
#define WEATHER_H

#include <stdint.h>
#include <stdbool.h>

// Структура для хранения данных о погоде
typedef struct {
    float temperature;           // Текущая температура (°C)
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

// Функция для получения данных о погоде
void get_weather(weather_callback_t callback);

#endif // WEATHER_H
