#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "app_protocol.h"

typedef void (*app_audio_downlink_done_cb_t)(uint16_t session_id, uint8_t status);

esp_err_t app_audio_downlink_init(app_audio_downlink_done_cb_t done_cb);
bool app_audio_downlink_enqueue(uint16_t session_id,
                                uint16_t seq,
                                uint8_t flags,
                                uint8_t codec,
                                uint16_t pcm_sample_count,
                                int16_t predictor,
                                uint8_t step_index,
                                const uint8_t *payload,
                                uint16_t payload_len);
void app_audio_downlink_reset(void);
bool app_audio_downlink_active(void);
uint8_t app_audio_downlink_queue_free_slots(void);
uint8_t app_audio_downlink_available_credits(void);
