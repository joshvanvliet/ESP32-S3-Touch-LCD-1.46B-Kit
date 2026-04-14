#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "app_protocol.h"

typedef struct {
    uint32_t frames_generated;
    uint32_t frames_enqueued;
    uint32_t queue_overflows;
    uint32_t queue_high_watermark;
    uint32_t source_sample_rate_hz;
    uint32_t level_updates;
    uint32_t i2s_read_errors;
    uint32_t i2s_read_timeouts;
    uint32_t frame_interval_max_ms;
    uint32_t frame_interval_avg_ms;
    uint32_t frame_timing_slip_ms;
    bool speech_detected;
} app_audio_capture_stats_t;

typedef struct {
    void (*on_level)(uint16_t level);
    void (*on_packet)(uint16_t session_id,
                      uint16_t seq,
                      uint32_t capture_elapsed_ms,
                      uint8_t flags,
                      const uint8_t *payload,
                      uint16_t payload_len);
    void (*on_stopped)(uint16_t session_id, app_capture_stop_reason_t reason, const app_audio_capture_stats_t *stats);
} app_audio_capture_callbacks_t;

esp_err_t app_audio_capture_init(const app_audio_capture_callbacks_t *callbacks);
bool app_audio_capture_start(uint16_t session_id, uint8_t codec, uint32_t sample_rate_hz);
void app_audio_capture_request_stop(app_capture_stop_reason_t reason);
bool app_audio_capture_is_running(void);
