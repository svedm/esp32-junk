#ifndef WEATHER_ICONS_H
#define WEATHER_ICONS_H

#include "lvgl.h"
#include "weather.h"

// Размер иконок 32x32
#define WEATHER_ICON_WIDTH  32
#define WEATHER_ICON_HEIGHT 32

// Получить изображение иконки по типу
const lv_image_dsc_t *weather_get_icon(weather_icon_t icon);

// Получить текстовое описание погоды
const char *weather_get_description(weather_icon_t icon);

#endif // WEATHER_ICONS_H
