#include "network.h"
#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_http_client.h"

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
#define TAG "NETWORK"

typedef struct {
    http_response_callback_t callback;
    int status_code;
} http_user_data_t;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    http_user_data_t *user_data = (http_user_data_t *)evt->user_data;

    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            if (user_data && strcmp(evt->header_key, "Status") == 0) {
                user_data->status_code = esp_http_client_get_status_code(evt->client);
            }
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);

            if (esp_http_client_is_chunked_response(evt->client)) {
                // Chunked response - динамически расширяем буфер
                if (output_buffer == NULL) {
                    output_buffer = (char *) calloc(evt->data_len + 1, sizeof(char));
                    output_len = 0;
                } else {
                    char *new_buffer = (char *) realloc(output_buffer, output_len + evt->data_len + 1);
                    if (new_buffer == NULL) {
                        ESP_LOGE(TAG, "Failed to reallocate memory for output buffer");
                        free(output_buffer);
                        output_buffer = NULL;
                        return ESP_FAIL;
                    }
                    output_buffer = new_buffer;
                }
                if (output_buffer == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                    return ESP_FAIL;
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len);
                output_len += evt->data_len;
                output_buffer[output_len] = '\0';
            } else {
                // Обычный response с известным Content-Length
                int content_len = esp_http_client_get_content_length(evt->client);
                if (output_buffer == NULL) {
                    output_buffer = (char *) calloc(content_len + 1, sizeof(char));
                    output_len = 0;
                    if (output_buffer == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                int copy_len = MIN(evt->data_len, (content_len - output_len));
                if (copy_len) {
                    memcpy(output_buffer + output_len, evt->data, copy_len);
                }
                output_len += copy_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                ESP_LOGI(TAG, "Response body:\n%s", output_buffer);
                if (user_data && user_data->callback) {
                    user_data->status_code = esp_http_client_get_status_code(evt->client);
                    user_data->callback(user_data->status_code, output_buffer);
                }
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}


void http_get(const char *url, http_response_callback_t callback)
{
    http_user_data_t user_data = {
        .callback = callback,
        .status_code = 0
    };

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .user_data = &user_data,
        .skip_cert_common_name_check = true,
    };

    ESP_LOGI(TAG, "HTTPS GET request to %s", url);
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}