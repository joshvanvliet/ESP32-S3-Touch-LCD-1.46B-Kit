#pragma once

#include <stdint.h>

#include "app_protocol.h"

typedef struct {
    uint8_t codec;
    uint32_t sample_rate_hz;
    const char *label;
} app_audio_profile_t;

void app_audio_profile_reset(void);
const app_audio_profile_t *app_audio_profile_active(void);
void app_audio_profile_note_session_quality(app_capture_stop_reason_t reason);
