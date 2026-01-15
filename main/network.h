typedef void (*http_response_callback_t)(int status_code, const char *body);

void http_get(const char *url, http_response_callback_t callback);