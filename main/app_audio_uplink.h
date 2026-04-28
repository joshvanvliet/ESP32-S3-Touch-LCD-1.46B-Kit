#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint16_t session_id;
    uint32_t frames_enqueued;
    uint32_t queue_overflows;
    uint32_t queue_high_watermark;
    uint32_t pending_packets;
    uint32_t tx_queue_delay_max_ms;
    uint32_t tx_queue_delay_avg_ms;
    bool end_sent;
    bool overflow_stop_requested;
} app_audio_uplink_stats_t;

typedef struct {
    void (*on_link_slow)(uint16_t session_id, uint32_t overflow_count);
} app_audio_uplink_callbacks_t;

esp_err_t app_audio_uplink_init(const app_audio_uplink_callbacks_t *callbacks);
void app_audio_uplink_clear(void);
void app_audio_uplink_start_session(uint16_t session_id, int64_t capture_start_us);
void app_audio_uplink_snapshot_stats(app_audio_uplink_stats_t *out);
bool app_audio_uplink_wait_for_drain(uint16_t session_id, uint32_t timeout_ms);
bool app_audio_uplink_enqueue(uint16_t session_id,
                              uint16_t seq,
                              uint32_t capture_elapsed_ms,
                              uint8_t codec,
                              uint8_t flags,
                              const uint8_t *payload,
                              uint16_t payload_len);
