#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "app_protocol.h"

typedef struct {
    void (*on_pairing_started)(void);
    void (*on_link_secured)(void);
    void (*on_app_ready)(void);
    void (*on_pairing_failed)(void);
    void (*on_disconnected)(void);
    void (*on_pairing_passkey)(uint32_t passkey);
    void (*on_control_start_capture)(void);
    void (*on_control_stop_capture)(void);
    void (*on_result_text)(uint16_t session_id, uint8_t status, const char *text);
    void (*on_audio_downlink_packet)(uint16_t session_id,
                                     uint16_t seq,
                                     uint8_t flags,
                                     uint8_t codec,
                                     const uint8_t *payload,
                                     uint16_t payload_len);
} app_ble_callbacks_t;

typedef struct {
    uint16_t session_id;
    uint32_t frames_notify_ok;
    uint32_t frames_notify_fail;
    uint32_t notify_retry_count;
} app_ble_audio_tx_stats_t;

esp_err_t app_ble_link_init(const app_ble_callbacks_t *callbacks);
bool app_ble_link_is_connected(void);
bool app_ble_link_audio_notify_ready(void);

void app_ble_link_begin_audio_session(uint16_t session_id);
void app_ble_link_get_audio_tx_stats(uint16_t session_id, app_ble_audio_tx_stats_t *stats_out);

void app_ble_link_notify_state(app_device_state_t state);
void app_ble_link_notify_level(uint16_t level);
void app_ble_link_notify_capture_stopped(uint16_t session_id, app_capture_stop_reason_t reason);
void app_ble_link_notify_error(uint8_t error_code);
void app_ble_link_notify_agent_status(uint8_t status, const char *detail);
void app_ble_link_notify_audio_downlink_ready(uint8_t credits);
void app_ble_link_notify_audio_downlink_done(uint16_t session_id, uint8_t status);

void app_ble_link_send_audio_packet(uint16_t session_id,
                                    uint16_t seq,
                                    uint16_t capture_elapsed_ms,
                                    uint8_t codec,
                                    uint8_t flags,
                                    const uint8_t *payload,
                                    uint16_t payload_len);
