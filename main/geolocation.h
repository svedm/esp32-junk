#include "wifi.h"

typedef struct
{
    double latitude;
    double longitude;
} geolocation_data_t;

typedef void (*geolocation_callback_t)(geolocation_data_t *location);


void get_geolocation(wifi_ap_record_t* ap_info, int count, geolocation_callback_t callback);