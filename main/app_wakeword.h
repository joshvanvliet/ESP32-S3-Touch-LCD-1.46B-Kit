#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    void (*on_detected)(void);
} app_wakeword_callbacks_t;

esp_err_t app_wakeword_init(const app_wakeword_callbacks_t *callbacks);
esp_err_t app_wakeword_start(void);
bool app_wakeword_stop(uint32_t timeout_ms);
bool app_wakeword_is_running(void);
