#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_protocol.h"

typedef struct {
    void (*on_primary_action)(void);
} app_ui_callbacks_t;

void app_ui_init(const app_ui_callbacks_t *callbacks);
void app_ui_set_state(app_device_state_t state);
void app_ui_set_status_text(const char *text);
void app_ui_set_pairing_passkey(uint32_t passkey, bool visible);
void app_ui_set_recording_level(uint16_t level);
void app_ui_set_transcript(const char *text, uint8_t status);

