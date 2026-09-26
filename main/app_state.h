#pragma once

#include "esp_err.h"
#include <stdint.h>

esp_err_t app_state_init(void);
void app_state_process(void);
/* Wait on the task that called app_state_init(); queued events wake it early. */
void app_state_wait(uint32_t timeout_ms);
