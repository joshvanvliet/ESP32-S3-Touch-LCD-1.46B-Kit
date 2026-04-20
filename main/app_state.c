#include "app_state.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "PWR_Key.h"
#include "app_audio_capture.h"
#include "app_audio_downlink.h"
#include "app_ble_link.h"
#include "app_protocol.h"
#include "app_ui.h"
#include "esp_log.h"
#include "esp_timer.h"

typedef enum {
    APP_EVENT_NONE = 0,
    APP_EVENT_PRIMARY_ACTION,
    APP_EVENT_PWRSHORT,
    APP_EVENT_BLE_PAIRING_STARTED,
    APP_EVENT_BLE_LINK_SECURED,
    APP_EVENT_BLE_APP_READY,
    APP_EVENT_BLE_PAIRING_FAILED,
    APP_EVENT_BLE_DISCONNECTED,
    APP_EVENT_PAIRING_PASSKEY,
    APP_EVENT_BLE_START_CAPTURE,
    APP_EVENT_BLE_STOP_CAPTURE,
    APP_EVENT_CAPTURE_STOPPED,
    APP_EVENT_TRANSCRIPT,
    APP_EVENT_AUDIO_DOWNLINK_DONE,
} app_event_type_t;

typedef struct {
    app_event_type_t type;
    union {
        uint32_t passkey;
        struct {
            uint16_t session_id;
            app_capture_stop_reason_t reason;
            app_audio_capture_stats_t stats;
        } capture;
        struct {
            uint16_t session_id;
            uint8_t status;
            char text[APP_MAX_TRANSCRIPT_BYTES + 1];
        } transcript;
        struct {
            uint16_t session_id;
            uint8_t status;
        } downlink_done;
    } data;
} app_event_t;

typedef struct {
    app_device_state_t state;
    uint16_t current_session_id;
    uint32_t passkey;
    char transcript[APP_MAX_TRANSCRIPT_BYTES + 1];
} app_runtime_t;

static const char *TAG = "APP_STATE";
static QueueHandle_t s_event_queue;
static QueueHandle_t s_audio_tx_queue;
static TaskHandle_t s_audio_tx_task;
static app_runtime_t s_rt;
static portMUX_TYPE s_level_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_audio_tx_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_latest_level;
static bool s_latest_level_pending;
static int64_t s_last_level_ui_us;
static int64_t s_last_level_notify_us;
static uint8_t s_active_capture_codec = APP_AUDIO_CODEC_IMA_ADPCM_16K;
static uint32_t s_active_capture_sample_rate_hz = APP_AUDIO_SAMPLE_RATE_16K;
static uint8_t s_audio_profile_index;
static uint8_t s_healthy_session_streak;
static int64_t s_capture_wall_start_us;
static int64_t s_uploading_state_enter_us;
static int64_t s_last_downlink_ready_us;

#define APP_LEVEL_NOTIFY_INTERVAL_US 50000
#define APP_AUDIO_TX_QUEUE_LEN 128
#define APP_AUDIO_TX_TASK_STACK_SIZE 4096
#define APP_AUDIO_TX_TASK_PRIORITY 5
#define APP_AUDIO_TX_TASK_CORE 0
#define APP_AUDIO_TX_OVERFLOW_STOP_COUNT 3
#define APP_AUDIO_TX_DRAIN_TIMEOUT_MS 4000
#define APP_AUDIO_PROFILE_RECOVERY_SESSIONS 3
#define APP_TRANSCRIPT_WAIT_TIMEOUT_MS 25000
#define APP_PKT_FLAG_END 0x02
#define APP_DOWNLINK_READY_NOTIFY_INTERVAL_US 120000

typedef struct {
    uint16_t session_id;
    uint16_t seq;
    uint16_t capture_elapsed_ms;
    uint8_t codec;
    uint8_t flags;
    uint16_t payload_len;
    uint32_t enqueued_at_ms;
    uint8_t payload[APP_AUDIO_MAX_ADPCM_BYTES];
} app_audio_tx_item_t;

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
} app_audio_tx_runtime_stats_t;

typedef struct {
    uint8_t codec;
    uint32_t sample_rate_hz;
    const char *label;
} app_audio_profile_t;

static app_audio_tx_runtime_stats_t s_audio_tx_stats;

static const app_audio_profile_t s_audio_profiles[] = {
    {.codec = APP_AUDIO_CODEC_IMA_ADPCM_16K, .sample_rate_hz = APP_AUDIO_SAMPLE_RATE_16K, .label = "16k"},
    {.codec = APP_AUDIO_CODEC_IMA_ADPCM_12K, .sample_rate_hz = APP_AUDIO_SAMPLE_RATE_12K, .label = "12k"},
    {.codec = APP_AUDIO_CODEC_IMA_ADPCM_8K, .sample_rate_hz = APP_AUDIO_SAMPLE_RATE_8K, .label = "8k"},
};

static void app_state_set_state(app_device_state_t new_state)
{
    if (s_rt.state == new_state) {
        return;
    }

    s_rt.state = new_state;
    if (new_state == APP_STATE_UPLOADING) {
        s_uploading_state_enter_us = esp_timer_get_time();
    } else {
        s_uploading_state_enter_us = 0;
    }
    app_ui_set_state(new_state);
    app_ble_link_notify_state(new_state);
    if (new_state == APP_STATE_UPLOADING) {
        app_ble_link_notify_agent_status(1, "Thinking...");
    } else if (new_state == APP_STATE_RESULT) {
        app_ble_link_notify_agent_status(0, "Idle");
    }
}

static bool app_state_post(app_event_t ev)
{
    if (!s_event_queue) {
        return false;
    }
    return xQueueSend(s_event_queue, &ev, 0) == pdTRUE;
}

static const app_audio_profile_t *app_state_active_profile(void)
{
    return &s_audio_profiles[s_audio_profile_index];
}

static void app_state_reset_audio_tx_stats(uint16_t session_id)
{
    portENTER_CRITICAL(&s_audio_tx_lock);
    memset(&s_audio_tx_stats, 0, sizeof(s_audio_tx_stats));
    s_audio_tx_stats.session_id = session_id;
    portEXIT_CRITICAL(&s_audio_tx_lock);
}

static void app_state_snapshot_audio_tx_stats(app_audio_tx_runtime_stats_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_audio_tx_lock);
    *out = s_audio_tx_stats;
    portEXIT_CRITICAL(&s_audio_tx_lock);
}

static void app_state_clear_audio_tx_queue(void)
{
    if (s_audio_tx_queue) {
        xQueueReset(s_audio_tx_queue);
    }
    app_state_reset_audio_tx_stats(0);
}

static void app_state_audio_tx_task(void *arg)
{
    (void)arg;
    app_audio_tx_item_t item;
    uint64_t tx_queue_delay_total_ms = 0;
    uint32_t tx_queue_delay_count = 0;
    uint16_t delay_stats_session_id = 0;
    while (true) {
        if (xQueueReceive(s_audio_tx_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        uint32_t now_ms = (uint32_t)(now_us / 1000);
        uint32_t queue_delay_ms = (item.enqueued_at_ms <= now_ms) ? (now_ms - item.enqueued_at_ms) : 0;

        app_ble_link_send_audio_packet(item.session_id,
                                       item.seq,
                                       item.capture_elapsed_ms,
                                       item.codec,
                                       item.flags,
                                       item.payload_len > 0 ? item.payload : NULL,
                                       item.payload_len);

        portENTER_CRITICAL(&s_audio_tx_lock);
        if (s_audio_tx_stats.session_id == item.session_id) {
            if (s_audio_tx_stats.pending_packets > 0) {
                s_audio_tx_stats.pending_packets--;
            }
            if (delay_stats_session_id != item.session_id) {
                delay_stats_session_id = item.session_id;
                tx_queue_delay_total_ms = 0;
                tx_queue_delay_count = 0;
            }
            tx_queue_delay_total_ms += queue_delay_ms;
            tx_queue_delay_count++;
            if (queue_delay_ms > s_audio_tx_stats.tx_queue_delay_max_ms) {
                s_audio_tx_stats.tx_queue_delay_max_ms = queue_delay_ms;
            }
            s_audio_tx_stats.tx_queue_delay_avg_ms = (uint32_t)(tx_queue_delay_total_ms / tx_queue_delay_count);
            if (item.flags & APP_PKT_FLAG_END) {
                s_audio_tx_stats.end_sent = true;
            }
        }
        portEXIT_CRITICAL(&s_audio_tx_lock);

        if (queue_delay_ms >= 120) {
            uint32_t capture_age_ms = 0;
            if (s_capture_wall_start_us > 0 && now_us > s_capture_wall_start_us) {
                capture_age_ms = (uint32_t)((now_us - s_capture_wall_start_us) / 1000);
            }
            ESP_LOGW(TAG,
                     "Audio TX queue delay session=%u seq=%u queue_delay_ms=%lu cap_ms=%u tx_elapsed_ms=%lu",
                     item.session_id,
                     item.seq,
                     (unsigned long)queue_delay_ms,
                     (unsigned)item.capture_elapsed_ms,
                     (unsigned long)capture_age_ms);
        }
    }
}

static bool app_state_wait_for_audio_tx_drain(uint16_t session_id, uint32_t timeout_ms)
{
    int64_t start_us = esp_timer_get_time();
    while (true) {
        bool drained;
        portENTER_CRITICAL(&s_audio_tx_lock);
        if (s_audio_tx_stats.session_id != session_id) {
            drained = true;
        } else {
            drained = s_audio_tx_stats.pending_packets == 0 &&
                      s_audio_tx_stats.end_sent;
        }
        portEXIT_CRITICAL(&s_audio_tx_lock);
        if (drained) {
            return true;
        }
        if (((esp_timer_get_time() - start_us) / 1000) >= timeout_ms) {
            return false;
        }
        vTaskDelay(1);
    }
}

static void app_state_note_session_quality(app_capture_stop_reason_t reason)
{
    bool changed = false;
    if (reason == APP_STOP_REASON_LINK_SLOW) {
        s_healthy_session_streak = 0;
        if ((s_audio_profile_index + 1U) < (sizeof(s_audio_profiles) / sizeof(s_audio_profiles[0]))) {
            s_audio_profile_index++;
            changed = true;
        }
    } else if (reason == APP_STOP_REASON_SILENCE || reason == APP_STOP_REASON_MANUAL || reason == APP_STOP_REASON_MAX_LEN) {
        if (s_audio_profile_index > 0) {
            s_healthy_session_streak++;
            if (s_healthy_session_streak >= APP_AUDIO_PROFILE_RECOVERY_SESSIONS) {
                s_audio_profile_index--;
                s_healthy_session_streak = 0;
                changed = true;
            }
        }
    } else {
        s_healthy_session_streak = 0;
    }

    if (changed) {
        const app_audio_profile_t *profile = app_state_active_profile();
        ESP_LOGI(TAG,
                 "Audio profile adjusted for next session: codec=0x%02X sample_rate=%lu profile=%s",
                 profile->codec,
                 (unsigned long)profile->sample_rate_hz,
                 profile->label);
    }
}

static void app_state_on_primary_action(void)
{
    app_event_t ev = {.type = APP_EVENT_PRIMARY_ACTION};
    app_state_post(ev);
}

static void app_state_on_pwr_short_press(void)
{
    app_event_t ev = {.type = APP_EVENT_PWRSHORT};
    app_state_post(ev);
}

static void app_state_on_ble_link_secured(void)
{
    app_event_t ev = {.type = APP_EVENT_BLE_LINK_SECURED};
    app_state_post(ev);
}

static void app_state_on_ble_app_ready(void)
{
    app_event_t ev = {.type = APP_EVENT_BLE_APP_READY};
    app_state_post(ev);
}

static void app_state_on_ble_pairing_failed(void)
{
    app_event_t ev = {.type = APP_EVENT_BLE_PAIRING_FAILED};
    app_state_post(ev);
}

static void app_state_on_ble_pairing_started(void)
{
    app_event_t ev = {.type = APP_EVENT_BLE_PAIRING_STARTED};
    app_state_post(ev);
}

static void app_state_on_ble_disconnected(void)
{
    app_event_t ev = {.type = APP_EVENT_BLE_DISCONNECTED};
    app_state_post(ev);
}

static void app_state_on_ble_passkey(uint32_t passkey)
{
    app_event_t ev = {
        .type = APP_EVENT_PAIRING_PASSKEY,
        .data.passkey = passkey,
    };
    app_state_post(ev);
}

static void app_state_on_ble_start_capture(void)
{
    app_event_t ev = {.type = APP_EVENT_BLE_START_CAPTURE};
    app_state_post(ev);
}

static void app_state_on_ble_stop_capture(void)
{
    app_event_t ev = {.type = APP_EVENT_BLE_STOP_CAPTURE};
    app_state_post(ev);
}

static void app_state_on_ble_result_text(uint16_t session_id, uint8_t status, const char *text)
{
    app_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = APP_EVENT_TRANSCRIPT;
    ev.data.transcript.session_id = session_id;
    ev.data.transcript.status = status;
    if (text) {
        strncpy(ev.data.transcript.text, text, APP_MAX_TRANSCRIPT_BYTES);
        ev.data.transcript.text[APP_MAX_TRANSCRIPT_BYTES] = '\0';
    }
    app_state_post(ev);
}

static void app_state_on_audio_downlink_done(uint16_t session_id, uint8_t status)
{
    app_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = APP_EVENT_AUDIO_DOWNLINK_DONE;
    ev.data.downlink_done.session_id = session_id;
    ev.data.downlink_done.status = status;
    app_state_post(ev);
}

static void app_state_on_ble_audio_downlink_packet(uint16_t session_id,
                                                   uint16_t seq,
                                                   uint8_t flags,
                                                   uint8_t codec,
                                                   const uint8_t *payload,
                                                   uint16_t payload_len)
{
    bool ok = app_audio_downlink_enqueue(session_id, seq, flags, codec, payload, payload_len);
    if (!ok) {
        ESP_LOGW(TAG,
                 "Downlink enqueue failed sid=%u seq=%u len=%u",
                 session_id,
                 seq,
                 payload_len);
        app_ble_link_notify_agent_status(3, "Downlink enqueue failed");
        app_ble_link_notify_audio_downlink_done(session_id, 1);
        return;
    }
    if (flags & 0x01) {
        app_ble_link_notify_agent_status(2, "Speaking...");
    }
    app_ble_link_notify_audio_downlink_ready(app_audio_downlink_queue_free_slots());
}

static void app_state_on_audio_level(uint16_t level)
{
    portENTER_CRITICAL(&s_level_lock);
    s_latest_level = level;
    s_latest_level_pending = true;
    portEXIT_CRITICAL(&s_level_lock);
}

static void app_state_on_audio_packet(uint16_t session_id,
                                      uint16_t seq,
                                      uint32_t capture_elapsed_ms,
                                      uint8_t flags,
                                      const uint8_t *payload,
                                      uint16_t payload_len)
{
    if (!s_audio_tx_queue) {
        return;
    }

    app_audio_tx_item_t item;
    memset(&item, 0, sizeof(item));
    item.session_id = session_id;
    item.seq = seq;
    if (capture_elapsed_ms > UINT16_MAX) {
        item.capture_elapsed_ms = UINT16_MAX;
    } else {
        item.capture_elapsed_ms = (uint16_t)capture_elapsed_ms;
    }
    item.codec = s_active_capture_codec;
    item.flags = flags;
    item.enqueued_at_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (payload_len > APP_AUDIO_MAX_ADPCM_BYTES) {
        payload_len = APP_AUDIO_MAX_ADPCM_BYTES;
    }
    item.payload_len = payload_len;
    if (payload_len > 0 && payload) {
        memcpy(item.payload, payload, payload_len);
    }

    TickType_t send_timeout = (flags & APP_PKT_FLAG_END) ? pdMS_TO_TICKS(50) : 0;
    if (xQueueSend(s_audio_tx_queue, &item, send_timeout) != pdTRUE) {
        bool request_link_slow_stop = false;
        uint32_t overflow_count = 0;
        portENTER_CRITICAL(&s_audio_tx_lock);
        if (s_audio_tx_stats.session_id == session_id) {
            s_audio_tx_stats.queue_overflows++;
            overflow_count = s_audio_tx_stats.queue_overflows;
            if (payload_len > 0 &&
                !s_audio_tx_stats.overflow_stop_requested &&
                s_audio_tx_stats.queue_overflows >= APP_AUDIO_TX_OVERFLOW_STOP_COUNT) {
                s_audio_tx_stats.overflow_stop_requested = true;
                request_link_slow_stop = true;
            }
        }
        portEXIT_CRITICAL(&s_audio_tx_lock);

        if (request_link_slow_stop) {
            ESP_LOGW(TAG,
                     "Audio TX queue overflow: session=%u overflows=%lu -> stopping capture as LINK_SLOW",
                     session_id,
                     (unsigned long)overflow_count);
            app_audio_capture_request_stop(APP_STOP_REASON_LINK_SLOW);
        }
        return;
    }

    UBaseType_t depth = uxQueueMessagesWaiting(s_audio_tx_queue);
    portENTER_CRITICAL(&s_audio_tx_lock);
    if (s_audio_tx_stats.session_id == session_id) {
        s_audio_tx_stats.pending_packets++;
        if (payload_len > 0) {
            s_audio_tx_stats.frames_enqueued++;
        }
        if ((uint32_t)depth > s_audio_tx_stats.queue_high_watermark) {
            s_audio_tx_stats.queue_high_watermark = (uint32_t)depth;
        }
    }
    portEXIT_CRITICAL(&s_audio_tx_lock);
}

static void app_state_on_audio_stopped(uint16_t session_id, app_capture_stop_reason_t reason, const app_audio_capture_stats_t *stats)
{
    app_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = APP_EVENT_CAPTURE_STOPPED;
    ev.data.capture.session_id = session_id;
    ev.data.capture.reason = reason;
    if (stats) {
        ev.data.capture.stats = *stats;
    }
    app_state_post(ev);
}

static bool app_state_read_latest_level(uint16_t *level_out, bool clear_pending)
{
    bool has_level = false;
    portENTER_CRITICAL(&s_level_lock);
    if (s_latest_level_pending) {
        has_level = true;
        if (level_out) {
            *level_out = s_latest_level;
        }
        if (clear_pending) {
            s_latest_level_pending = false;
        }
    }
    portEXIT_CRITICAL(&s_level_lock);
    return has_level;
}

static void app_state_start_capture(void)
{
    if (s_rt.state != APP_STATE_READY) {
        return;
    }

    if (!app_ble_link_audio_notify_ready()) {
        app_ui_set_status_text("Audio channel not ready in app");
        app_ble_link_notify_error(2);
        ESP_LOGW(TAG, "Capture start blocked: audio notify disabled");
        return;
    }

    const app_audio_profile_t *profile = app_state_active_profile();

    s_rt.current_session_id++;
    if (s_rt.current_session_id == 0) {
        s_rt.current_session_id = 1;
    }

    app_state_clear_audio_tx_queue();
    app_state_reset_audio_tx_stats(s_rt.current_session_id);
    app_ble_link_begin_audio_session(s_rt.current_session_id);
    s_active_capture_codec = profile->codec;
    s_active_capture_sample_rate_hz = profile->sample_rate_hz;

    s_capture_wall_start_us = esp_timer_get_time();
    bool ok = app_audio_capture_start(s_rt.current_session_id, profile->codec, profile->sample_rate_hz);
    if (!ok) {
        s_capture_wall_start_us = 0;
        app_ui_set_status_text("Capture start failed");
        app_ble_link_notify_error(1);
        return;
    }

    ESP_LOGI(TAG,
             "Capture session started: session=%u codec=0x%02X sample_rate=%lu profile=%s",
             s_rt.current_session_id,
             s_active_capture_codec,
             (unsigned long)s_active_capture_sample_rate_hz,
             profile->label);
    app_state_set_state(APP_STATE_RECORDING);
    app_ui_set_status_text("Listening...");
    app_ui_set_recording_level(0);
    s_last_level_ui_us = 0;
    s_last_level_notify_us = 0;
    portENTER_CRITICAL(&s_level_lock);
    s_latest_level = 0;
    s_latest_level_pending = false;
    portEXIT_CRITICAL(&s_level_lock);
}

static void app_state_stop_capture_manual(void)
{
    if (s_rt.state == APP_STATE_RECORDING) {
        app_audio_capture_request_stop(APP_STOP_REASON_MANUAL);
    }
}

static void app_state_handle_event(const app_event_t *ev)
{
    switch (ev->type) {
        case APP_EVENT_PRIMARY_ACTION:
            if (s_rt.state == APP_STATE_READY) {
                app_state_start_capture();
            } else if (s_rt.state == APP_STATE_RECORDING) {
                app_state_stop_capture_manual();
            } else if (s_rt.state == APP_STATE_RESULT) {
                app_state_set_state(APP_STATE_READY);
                app_state_start_capture();
            }
            break;

        case APP_EVENT_PWRSHORT:
            if (s_rt.state == APP_STATE_RECORDING) {
                app_state_stop_capture_manual();
            }
            break;

        case APP_EVENT_BLE_LINK_SECURED:
            s_rt.passkey = 0;
            app_state_set_state(APP_STATE_LINKED);
            app_ui_set_pairing_passkey(0, false);
            app_ui_set_status_text("Paired - open companion app");
            break;

        case APP_EVENT_BLE_APP_READY:
            s_rt.passkey = 0;
            app_state_set_state(APP_STATE_READY);
            app_ui_set_pairing_passkey(0, false);
            app_ui_set_status_text("Phone app connected");
            break;

        case APP_EVENT_BLE_PAIRING_FAILED:
            s_rt.passkey = 0;
            app_state_set_state(APP_STATE_UNPAIRED);
            app_ui_set_pairing_passkey(0, false);
            app_ui_set_status_text("Pairing failed - try again");
            break;

        case APP_EVENT_BLE_PAIRING_STARTED:
            s_rt.passkey = 0;
            app_state_set_state(APP_STATE_PAIRING);
            app_ui_set_pairing_passkey(0, false);
            app_ui_set_status_text("Secure pairing in progress");
            break;

        case APP_EVENT_BLE_DISCONNECTED:
            if (app_audio_capture_is_running()) {
                app_audio_capture_request_stop(APP_STOP_REASON_MANUAL);
            }
            s_capture_wall_start_us = 0;
            app_audio_downlink_reset();
            app_state_clear_audio_tx_queue();
            portENTER_CRITICAL(&s_level_lock);
            s_latest_level = 0;
            s_latest_level_pending = false;
            portEXIT_CRITICAL(&s_level_lock);
            s_rt.passkey = 0;
            app_state_set_state(APP_STATE_UNPAIRED);
            app_ui_set_status_text("Disconnected - waiting");
            app_ui_set_pairing_passkey(0, false);
            break;

        case APP_EVENT_PAIRING_PASSKEY:
            s_rt.passkey = ev->data.passkey;
            if (s_rt.state != APP_STATE_PAIRING) {
                app_state_set_state(APP_STATE_PAIRING);
            }
            app_ui_set_pairing_passkey(s_rt.passkey, true);
            app_ui_set_status_text("Confirm passkey on phone");
            break;

        case APP_EVENT_BLE_START_CAPTURE:
            ESP_LOGI(TAG, "BLE requested capture start");
            if (s_rt.state == APP_STATE_RESULT) {
                app_state_set_state(APP_STATE_READY);
            }
            app_state_start_capture();
            break;

        case APP_EVENT_BLE_STOP_CAPTURE:
            ESP_LOGI(TAG, "BLE requested capture stop");
            app_state_stop_capture_manual();
            break;

        case APP_EVENT_CAPTURE_STOPPED:
            app_capture_stop_reason_t effective_reason = ev->data.capture.reason;
            app_audio_capture_stats_t capture_stats = ev->data.capture.stats;
            s_capture_wall_start_us = 0;

            bool drained = app_state_wait_for_audio_tx_drain(ev->data.capture.session_id, APP_AUDIO_TX_DRAIN_TIMEOUT_MS);
            if (!drained) {
                app_audio_tx_runtime_stats_t tx_pending;
                memset(&tx_pending, 0, sizeof(tx_pending));
                app_state_snapshot_audio_tx_stats(&tx_pending);
                ESP_LOGW(TAG,
                         "Audio TX drain timeout: session=%u pending=%lu end_sent=%d",
                         ev->data.capture.session_id,
                         (unsigned long)tx_pending.pending_packets,
                         tx_pending.end_sent ? 1 : 0);
            }

            app_audio_tx_runtime_stats_t tx_runtime;
            memset(&tx_runtime, 0, sizeof(tx_runtime));
            app_state_snapshot_audio_tx_stats(&tx_runtime);
            capture_stats.frames_enqueued = tx_runtime.frames_enqueued;
            capture_stats.queue_overflows = tx_runtime.queue_overflows;
            capture_stats.queue_high_watermark = tx_runtime.queue_high_watermark;

            app_ble_audio_tx_stats_t tx_stats;
            app_ble_link_get_audio_tx_stats(ev->data.capture.session_id, &tx_stats);
            ESP_LOGI(TAG,
                     "Capture stopped. session=%u reason=%u sr=%lu generated=%lu enqueued=%lu queue_overflows=%lu queue_max=%lu notify_ok=%lu notify_fail=%lu retries=%lu level_updates=%lu i2s_timeouts=%lu i2s_errors=%lu speech=%d",
                     ev->data.capture.session_id,
                     (unsigned)effective_reason,
                     (unsigned long)capture_stats.source_sample_rate_hz,
                     (unsigned long)capture_stats.frames_generated,
                     (unsigned long)capture_stats.frames_enqueued,
                     (unsigned long)capture_stats.queue_overflows,
                     (unsigned long)capture_stats.queue_high_watermark,
                     (unsigned long)tx_stats.frames_notify_ok,
                     (unsigned long)tx_stats.frames_notify_fail,
                     (unsigned long)tx_stats.notify_retry_count,
                     (unsigned long)capture_stats.level_updates,
                     (unsigned long)capture_stats.i2s_read_timeouts,
                     (unsigned long)capture_stats.i2s_read_errors,
                     capture_stats.speech_detected ? 1 : 0);
            app_ble_link_notify_capture_stopped(ev->data.capture.session_id, effective_reason);
            if (effective_reason == APP_STOP_REASON_NO_SPEECH) {
                app_state_set_state(APP_STATE_RESULT);
                app_ui_set_transcript("No speech detected. Try again.", 1);
                app_ui_set_status_text("No speech captured");
            } else if (effective_reason == APP_STOP_REASON_LINK_SLOW) {
                app_state_set_state(APP_STATE_RESULT);
                app_ui_set_transcript("Link too slow. Capture stopped to avoid corrupted audio.", 1);
                app_ui_set_status_text("Link too slow - retrying lower quality");
            } else {
                app_state_set_state(APP_STATE_UPLOADING);
                app_ui_set_status_text("Waiting for phone transcript...");
            }
            app_ui_set_recording_level(0);
            portENTER_CRITICAL(&s_level_lock);
            s_latest_level = 0;
            s_latest_level_pending = false;
            portEXIT_CRITICAL(&s_level_lock);
            app_state_note_session_quality(effective_reason);
            break;

        case APP_EVENT_TRANSCRIPT:
            if (ev->data.transcript.session_id != s_rt.current_session_id) {
                break;
            }
            strncpy(s_rt.transcript, ev->data.transcript.text, APP_MAX_TRANSCRIPT_BYTES);
            s_rt.transcript[APP_MAX_TRANSCRIPT_BYTES] = '\0';
            app_ui_set_transcript(s_rt.transcript, ev->data.transcript.status);
            app_state_set_state(APP_STATE_RESULT);
            app_ui_set_status_text("Transcript mirrored from phone");
            break;

        case APP_EVENT_AUDIO_DOWNLINK_DONE:
            app_ble_link_notify_audio_downlink_done(ev->data.downlink_done.session_id, ev->data.downlink_done.status);
            if (ev->data.downlink_done.status == 0) {
                app_ui_set_status_text("Playback complete");
                app_ble_link_notify_agent_status(0, "Idle");
            } else if (ev->data.downlink_done.status == 1) {
                app_ui_set_status_text("Playback failed (enqueue)");
                app_ble_link_notify_agent_status(3, "Playback enqueue error");
            } else if (ev->data.downlink_done.status == 2) {
                app_ui_set_status_text("Playback failed (sequence)");
                app_ble_link_notify_agent_status(3, "Playback sequence mismatch");
            } else if (ev->data.downlink_done.status == 3) {
                app_ui_set_status_text("Playback failed (i2s)");
                app_ble_link_notify_agent_status(3, "Playback I2S write failed");
            } else {
                app_ui_set_status_text("Playback failed");
                app_ble_link_notify_agent_status(3, "Playback error");
            }
            break;

        case APP_EVENT_NONE:
        default:
            break;
    }
}

esp_err_t app_state_init(void)
{
    memset(&s_rt, 0, sizeof(s_rt));
    s_rt.state = APP_STATE_UNPAIRED;
    s_latest_level = 0;
    s_latest_level_pending = false;
    s_last_level_ui_us = 0;
    s_last_level_notify_us = 0;
    s_audio_profile_index = 0;
    s_healthy_session_streak = 0;
    s_active_capture_codec = APP_AUDIO_CODEC_IMA_ADPCM_16K;
    s_active_capture_sample_rate_hz = APP_AUDIO_SAMPLE_RATE_16K;
    s_uploading_state_enter_us = 0;
    s_last_downlink_ready_us = 0;
    memset(&s_audio_tx_stats, 0, sizeof(s_audio_tx_stats));

    s_event_queue = xQueueCreate(32, sizeof(app_event_t));
    if (!s_event_queue) {
        return ESP_ERR_NO_MEM;
    }
    s_audio_tx_queue = xQueueCreate(APP_AUDIO_TX_QUEUE_LEN, sizeof(app_audio_tx_item_t));
    if (!s_audio_tx_queue) {
        return ESP_ERR_NO_MEM;
    }
    BaseType_t tx_ok = xTaskCreatePinnedToCore(app_state_audio_tx_task,
                                               "app_audio_tx",
                                               APP_AUDIO_TX_TASK_STACK_SIZE,
                                               NULL,
                                               APP_AUDIO_TX_TASK_PRIORITY,
                                               &s_audio_tx_task,
                                               APP_AUDIO_TX_TASK_CORE);
    if (tx_ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    app_ui_callbacks_t ui_callbacks = {
        .on_primary_action = app_state_on_primary_action,
    };
    app_ui_init(&ui_callbacks);
    app_ui_set_state(APP_STATE_UNPAIRED);
    app_ui_set_status_text("Starting BLE...");

    app_ble_callbacks_t ble_callbacks = {
        .on_pairing_started = app_state_on_ble_pairing_started,
        .on_link_secured = app_state_on_ble_link_secured,
        .on_app_ready = app_state_on_ble_app_ready,
        .on_pairing_failed = app_state_on_ble_pairing_failed,
        .on_disconnected = app_state_on_ble_disconnected,
        .on_pairing_passkey = app_state_on_ble_passkey,
        .on_control_start_capture = app_state_on_ble_start_capture,
        .on_control_stop_capture = app_state_on_ble_stop_capture,
        .on_result_text = app_state_on_ble_result_text,
        .on_audio_downlink_packet = app_state_on_ble_audio_downlink_packet,
    };

    ESP_ERROR_CHECK(app_ble_link_init(&ble_callbacks));
    app_ui_set_status_text("Open Android app and connect");

    app_audio_capture_callbacks_t audio_callbacks = {
        .on_level = app_state_on_audio_level,
        .on_packet = app_state_on_audio_packet,
        .on_stopped = app_state_on_audio_stopped,
    };
    ESP_ERROR_CHECK(app_audio_capture_init(&audio_callbacks));
    ESP_ERROR_CHECK(app_audio_downlink_init(app_state_on_audio_downlink_done));

    PWR_RegisterShortPressCallback(app_state_on_pwr_short_press);

    ESP_LOGI(TAG, "App state initialized");
    return ESP_OK;
}

void app_state_process(void)
{
    app_event_t ev;
    while (xQueueReceive(s_event_queue, &ev, 0) == pdTRUE) {
        app_state_handle_event(&ev);
    }

    if (s_rt.state == APP_STATE_RECORDING) {
        int64_t now_us = esp_timer_get_time();
        if (s_last_level_ui_us == 0 || (now_us - s_last_level_ui_us) >= APP_LEVEL_NOTIFY_INTERVAL_US) {
            uint16_t level = 0;
            if (app_state_read_latest_level(&level, true)) {
                app_ui_set_recording_level(level);
                s_last_level_ui_us = now_us;
            }
        }
        if (s_last_level_notify_us == 0 || (now_us - s_last_level_notify_us) >= APP_LEVEL_NOTIFY_INTERVAL_US) {
            uint16_t level = 0;
            if (app_state_read_latest_level(&level, false)) {
                app_ble_link_notify_level(level);
                s_last_level_notify_us = now_us;
            }
        }
    } else if (s_rt.state == APP_STATE_UPLOADING && s_uploading_state_enter_us > 0) {
        int64_t now_us = esp_timer_get_time();
        int64_t waited_ms = (now_us - s_uploading_state_enter_us) / 1000;
        if (waited_ms >= APP_TRANSCRIPT_WAIT_TIMEOUT_MS) {
            ESP_LOGW(TAG,
                     "Transcript wait timeout: session=%u waited_ms=%lld",
                     s_rt.current_session_id,
                     (long long)waited_ms);
            app_ui_set_transcript("Phone transcript timeout. Try again.", 1);
            app_ui_set_status_text("Phone transcript timeout");
            app_state_set_state(APP_STATE_RESULT);
        }
    }

    if (app_ble_link_is_connected()) {
        int64_t now_us = esp_timer_get_time();
        if (s_last_downlink_ready_us == 0 || (now_us - s_last_downlink_ready_us) >= APP_DOWNLINK_READY_NOTIFY_INTERVAL_US) {
            app_ble_link_notify_audio_downlink_ready(app_audio_downlink_queue_free_slots());
            s_last_downlink_ready_us = now_us;
        }
    }
}
