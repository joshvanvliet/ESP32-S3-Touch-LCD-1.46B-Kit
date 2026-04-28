#include "app_state.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "PWR_Key.h"
#include "app_audio_capture.h"
#include "app_audio_downlink.h"
#include "app_audio_profile.h"
#include "app_audio_uplink.h"
#include "app_ble_link.h"
#include "app_protocol.h"
#include "app_ui.h"
#if CONFIG_APP_WAKEWORD_ENABLED
#include "app_wakeword.h"
#endif
#include "esp_heap_caps.h"
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
    APP_EVENT_AGENT_ACTIVITY,
    APP_EVENT_WAKEWORD_DETECTED,
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
        struct {
            uint8_t status;
        } agent_activity;
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
static app_runtime_t s_rt;
static portMUX_TYPE s_level_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_latest_level;
static bool s_latest_level_pending;
static int64_t s_last_level_ui_us;
static int64_t s_last_level_notify_us;
static int64_t s_last_downlink_ready_us;
static int64_t s_wake_resume_after_us;
static uint8_t s_active_capture_codec = APP_AUDIO_CODEC_IMA_ADPCM_16K;
static uint32_t s_active_capture_sample_rate_hz = APP_AUDIO_SAMPLE_RATE_16K;
static int64_t s_capture_wall_start_us;
static int64_t s_uploading_state_enter_us;
static bool s_agent_activity_busy;
static uint32_t s_wake_ignored_count;

static void app_state_log_internal_heap(const char *label)
{
    ESP_LOGI(TAG,
             "%s heap internal_free=%lu internal_largest=%lu psram_free=%lu psram_largest=%lu",
             label,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

#define APP_LEVEL_NOTIFY_INTERVAL_US 50000
#define APP_DOWNLINK_READY_NOTIFY_INTERVAL_US 40000
#define APP_AUDIO_TX_DRAIN_TIMEOUT_MS 4000
#define APP_TRANSCRIPT_WAIT_TIMEOUT_MS 25000
#define APP_AGENT_ACTIVITY_IDLE 0
#define APP_AGENT_ACTIVITY_THINKING 1
#define APP_AGENT_ACTIVITY_SPEAKING 2
#define APP_AGENT_ACTIVITY_ERROR 3

#if CONFIG_APP_WAKEWORD_ENABLED
static void app_state_wake_pause(const char *status);
static void app_state_schedule_wake_rearm(uint32_t delay_ms);
#endif

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

#if CONFIG_APP_WAKEWORD_ENABLED
    if (new_state == APP_STATE_RECORDING ||
        new_state == APP_STATE_UPLOADING ||
        new_state == APP_STATE_UNPAIRED ||
        new_state == APP_STATE_PAIRING ||
        new_state == APP_STATE_LINKED) {
        app_state_wake_pause("Wake paused");
    } else if (new_state == APP_STATE_READY || new_state == APP_STATE_RESULT) {
        app_state_schedule_wake_rearm(CONFIG_APP_WAKEWORD_RESUME_DELAY_MS);
    }
#endif
}

static bool app_state_post(app_event_t ev)
{
    if (!s_event_queue) {
        return false;
    }
    return xQueueSend(s_event_queue, &ev, 0) == pdTRUE;
}

#if CONFIG_APP_WAKEWORD_ENABLED
static bool app_state_wake_allowed(void)
{
    return app_ble_link_is_connected() &&
           !s_agent_activity_busy &&
           !app_audio_capture_is_running() &&
           !app_audio_downlink_active() &&
           (s_rt.state == APP_STATE_READY || s_rt.state == APP_STATE_RESULT);
}

static void app_state_wake_pause(const char *status)
{
    s_wake_resume_after_us = 0;
    if (app_wakeword_is_running()) {
        if (!app_wakeword_stop(CONFIG_APP_WAKEWORD_STOP_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "WakeNet stop timed out");
        }
    }
    if (status && status[0] != '\0') {
        app_ui_set_status_text(status);
    }
}

static bool app_state_stop_wake_before_capture(void)
{
    if (!app_wakeword_is_running()) {
        return true;
    }
    if (app_wakeword_stop(CONFIG_APP_WAKEWORD_STOP_TIMEOUT_MS)) {
        return true;
    }
    app_ui_set_status_text("Wake busy - try button");
    app_ble_link_notify_error(3);
    ESP_LOGW(TAG, "Capture blocked: WakeNet stop timeout");
    return false;
}

static void app_state_schedule_wake_rearm(uint32_t delay_ms)
{
    if (!app_state_wake_allowed()) {
        s_wake_resume_after_us = 0;
        return;
    }
    int64_t now_us = esp_timer_get_time();
    s_wake_resume_after_us = now_us + ((int64_t)delay_ms * 1000);
    if (delay_ms > 0) {
        app_ui_set_status_text("Wake paused");
    }
}

static void app_state_try_wake_rearm(void)
{
    if (s_wake_resume_after_us == 0 || esp_timer_get_time() < s_wake_resume_after_us) {
        return;
    }
    s_wake_resume_after_us = 0;
    if (!app_state_wake_allowed() || app_wakeword_is_running()) {
        return;
    }
    app_audio_downlink_release_transient_resources();
    app_state_log_internal_heap("Before WakeNet rearm");
    esp_err_t err = app_wakeword_start();
    if (err == ESP_OK) {
        app_ui_set_status_text("Wake listening");
    } else {
        ESP_LOGW(TAG, "WakeNet start failed: %s", esp_err_to_name(err));
        if (err == ESP_ERR_NO_MEM) {
            app_state_schedule_wake_rearm(3000);
        }
    }
}

static void app_state_on_wakeword_detected(void)
{
    app_event_t ev = {.type = APP_EVENT_WAKEWORD_DETECTED};
    app_state_post(ev);
}
#endif

static void app_state_on_primary_action(void)
{
    app_event_t ev = {.type = APP_EVENT_PRIMARY_ACTION};
    app_state_post(ev);
}

static void app_state_on_audio_uplink_slow(uint16_t session_id, uint32_t overflow_count)
{
    (void)session_id;
    (void)overflow_count;
    app_audio_capture_request_stop(APP_STOP_REASON_LINK_SLOW);
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

static void app_state_on_ble_agent_activity(uint8_t status)
{
    app_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = APP_EVENT_AGENT_ACTIVITY;
    ev.data.agent_activity.status = status;
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
                                                   uint16_t pcm_sample_count,
                                                   uint16_t codec_param0,
                                                   uint8_t codec_param1,
                                                   const uint8_t *payload,
                                                   uint16_t payload_len)
{
    if (flags & 0x01) {
#if CONFIG_APP_WAKEWORD_ENABLED
        s_agent_activity_busy = true;
        app_state_wake_pause("Wake paused");
#endif
    }
    bool ok = app_audio_downlink_enqueue(
        session_id,
        seq,
        flags,
        codec,
        pcm_sample_count,
        codec_param0,
        codec_param1,
        payload,
        payload_len);
    if (!ok) {
        ESP_LOGW(TAG,
                 "Downlink enqueue failed sid=%u seq=%u len=%u",
                 session_id,
                 seq,
                 payload_len);
        app_audio_downlink_reset();
        app_ble_link_notify_agent_status(3, "Downlink enqueue failed");
        app_ble_link_notify_audio_downlink_done(session_id, 1);
        return;
    }
    if (flags & 0x01) {
        app_ble_link_notify_agent_status(2, "Speaking...");
        app_ui_set_agent_activity(APP_AGENT_ACTIVITY_SPEAKING);
    }
    app_ble_link_notify_audio_downlink_ready(app_audio_downlink_available_credits());
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
    (void)app_audio_uplink_enqueue(session_id,
                                   seq,
                                   capture_elapsed_ms,
                                   s_active_capture_codec,
                                   flags,
                                   payload,
                                   payload_len);
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

#if CONFIG_APP_WAKEWORD_ENABLED
    if (!app_state_stop_wake_before_capture()) {
        return;
    }
#endif

    const app_audio_profile_t *profile = app_audio_profile_active();

    s_rt.current_session_id++;
    if (s_rt.current_session_id == 0) {
        s_rt.current_session_id = 1;
    }

    app_ble_link_begin_audio_session(s_rt.current_session_id);
    s_active_capture_codec = profile->codec;
    s_active_capture_sample_rate_hz = profile->sample_rate_hz;

    s_capture_wall_start_us = esp_timer_get_time();
    app_audio_uplink_start_session(s_rt.current_session_id, s_capture_wall_start_us);
    bool ok = app_audio_capture_start(s_rt.current_session_id, profile->codec, profile->sample_rate_hz);
    if (!ok) {
        s_capture_wall_start_us = 0;
        app_audio_uplink_clear();
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
    s_last_downlink_ready_us = 0;
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
            app_ble_link_notify_audio_downlink_ready(app_audio_downlink_available_credits());
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
            app_audio_uplink_clear();
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

            bool drained = app_audio_uplink_wait_for_drain(ev->data.capture.session_id, APP_AUDIO_TX_DRAIN_TIMEOUT_MS);
            if (!drained) {
                app_audio_uplink_stats_t tx_pending;
                memset(&tx_pending, 0, sizeof(tx_pending));
                app_audio_uplink_snapshot_stats(&tx_pending);
                ESP_LOGW(TAG,
                         "Audio TX drain timeout: session=%u pending=%lu end_sent=%d",
                         ev->data.capture.session_id,
                         (unsigned long)tx_pending.pending_packets,
                         tx_pending.end_sent ? 1 : 0);
            }

            app_audio_uplink_stats_t tx_runtime;
            memset(&tx_runtime, 0, sizeof(tx_runtime));
            app_audio_uplink_snapshot_stats(&tx_runtime);
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
            app_audio_profile_note_session_quality(effective_reason);
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

        case APP_EVENT_AGENT_ACTIVITY:
            app_ui_set_agent_activity(ev->data.agent_activity.status);
#if CONFIG_APP_WAKEWORD_ENABLED
            if (ev->data.agent_activity.status == APP_AGENT_ACTIVITY_THINKING ||
                ev->data.agent_activity.status == APP_AGENT_ACTIVITY_SPEAKING) {
                s_agent_activity_busy = true;
                app_state_wake_pause("Wake paused");
            } else {
                s_agent_activity_busy = false;
                app_state_schedule_wake_rearm(CONFIG_APP_WAKEWORD_RESUME_DELAY_MS);
            }
#endif
            break;

        case APP_EVENT_WAKEWORD_DETECTED:
#if CONFIG_APP_WAKEWORD_ENABLED
            ESP_LOGI(TAG, "Wake event received in state=%u busy=%d", (unsigned)s_rt.state, s_agent_activity_busy ? 1 : 0);
            if (!app_ble_link_audio_notify_ready()) {
                app_ui_set_status_text("Wake detected - app audio not ready");
                app_state_schedule_wake_rearm(CONFIG_APP_WAKEWORD_RESUME_DELAY_MS);
                break;
            }
            if (s_rt.state == APP_STATE_RESULT) {
                app_state_set_state(APP_STATE_READY);
            }
            if (s_rt.state != APP_STATE_READY || s_agent_activity_busy || app_audio_downlink_active()) {
                s_wake_ignored_count++;
                ESP_LOGW(TAG,
                         "Wake ignored state=%u busy=%d downlink=%d ignored=%lu",
                         (unsigned)s_rt.state,
                         s_agent_activity_busy ? 1 : 0,
                         app_audio_downlink_active() ? 1 : 0,
                         (unsigned long)s_wake_ignored_count);
                app_state_schedule_wake_rearm(CONFIG_APP_WAKEWORD_RESUME_DELAY_MS);
                break;
            }
            app_ui_set_status_text("Wake detected");
            app_state_start_capture();
#endif
            break;

        case APP_EVENT_AUDIO_DOWNLINK_DONE:
#if CONFIG_APP_WAKEWORD_ENABLED
            s_agent_activity_busy = false;
#endif
            app_ui_set_agent_activity(APP_AGENT_ACTIVITY_IDLE);
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
                app_ui_set_status_text("Playback failed (audio)");
                app_ble_link_notify_agent_status(3, "Playback audio failed");
            } else {
                app_ui_set_status_text("Playback failed");
                app_ble_link_notify_agent_status(3, "Playback error");
            }
#if CONFIG_APP_WAKEWORD_ENABLED
            app_state_schedule_wake_rearm(CONFIG_APP_WAKEWORD_RESUME_DELAY_MS);
#endif
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
    s_active_capture_codec = APP_AUDIO_CODEC_IMA_ADPCM_16K;
    s_active_capture_sample_rate_hz = APP_AUDIO_SAMPLE_RATE_16K;
    s_uploading_state_enter_us = 0;
    s_wake_resume_after_us = 0;
    s_agent_activity_busy = false;
    s_wake_ignored_count = 0;
    app_audio_profile_reset();
    s_event_queue = xQueueCreate(32, sizeof(app_event_t));
    if (!s_event_queue) {
        return ESP_ERR_NO_MEM;
    }

    app_audio_uplink_callbacks_t uplink_callbacks = {
        .on_link_slow = app_state_on_audio_uplink_slow,
    };
    esp_err_t init_err = app_audio_uplink_init(&uplink_callbacks);
    if (init_err != ESP_OK) {
        return ESP_ERR_NO_MEM;
    }

    app_ui_callbacks_t ui_callbacks = {
        .on_primary_action = app_state_on_primary_action,
    };
    app_ui_init(&ui_callbacks);
    app_ui_set_state(APP_STATE_UNPAIRED);
    app_ui_set_status_text("Starting BLE...");
    app_state_log_internal_heap("Before BLE init");

    app_ble_callbacks_t ble_callbacks = {
        .on_pairing_started = app_state_on_ble_pairing_started,
        .on_link_secured = app_state_on_ble_link_secured,
        .on_app_ready = app_state_on_ble_app_ready,
        .on_pairing_failed = app_state_on_ble_pairing_failed,
        .on_disconnected = app_state_on_ble_disconnected,
        .on_pairing_passkey = app_state_on_ble_passkey,
        .on_control_start_capture = app_state_on_ble_start_capture,
        .on_control_stop_capture = app_state_on_ble_stop_capture,
        .on_control_agent_activity = app_state_on_ble_agent_activity,
        .on_result_text = app_state_on_ble_result_text,
        .on_audio_downlink_packet = app_state_on_ble_audio_downlink_packet,
    };

    init_err = app_ble_link_init(&ble_callbacks);
    app_state_log_internal_heap("After BLE init");
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", esp_err_to_name(init_err));
        app_ui_set_status_text("BLE init failed");
        app_ui_set_agent_activity(APP_AGENT_ACTIVITY_ERROR);
    } else {
        app_ui_set_status_text("Open Android app and connect");
    }

    app_audio_capture_callbacks_t audio_callbacks = {
        .on_level = app_state_on_audio_level,
        .on_packet = app_state_on_audio_packet,
        .on_stopped = app_state_on_audio_stopped,
    };
    init_err = app_audio_capture_init(&audio_callbacks);
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "Audio capture init failed: %s", esp_err_to_name(init_err));
        app_ui_set_status_text("Mic init failed");
        app_ui_set_agent_activity(APP_AGENT_ACTIVITY_ERROR);
    }
    init_err = app_audio_downlink_init(app_state_on_audio_downlink_done);
    app_state_log_internal_heap("After audio init");
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "Audio downlink init failed: %s", esp_err_to_name(init_err));
        app_ui_set_status_text("Speaker init failed");
        app_ui_set_agent_activity(APP_AGENT_ACTIVITY_ERROR);
    }
#if CONFIG_APP_WAKEWORD_ENABLED
    app_wakeword_callbacks_t wake_callbacks = {
        .on_detected = app_state_on_wakeword_detected,
    };
    init_err = app_wakeword_init(&wake_callbacks);
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "Wake word init failed: %s", esp_err_to_name(init_err));
        app_ui_set_status_text("Wake init failed");
        app_ui_set_agent_activity(APP_AGENT_ACTIVITY_ERROR);
    }
#endif

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

    if (app_ble_link_is_connected() && app_audio_downlink_active()) {
        int64_t now_us = esp_timer_get_time();
        if (s_last_downlink_ready_us == 0 ||
            (now_us - s_last_downlink_ready_us) >= APP_DOWNLINK_READY_NOTIFY_INTERVAL_US) {
            app_ble_link_notify_audio_downlink_ready(app_audio_downlink_available_credits());
            s_last_downlink_ready_us = now_us;
        }
    } else {
        s_last_downlink_ready_us = 0;
    }

#if CONFIG_APP_WAKEWORD_ENABLED
    app_state_try_wake_rearm();
#endif
}
