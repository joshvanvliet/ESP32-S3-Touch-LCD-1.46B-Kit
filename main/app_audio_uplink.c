#include "app_audio_uplink.h"

#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_ble_link.h"
#include "app_protocol.h"
#include "esp_log.h"
#include "esp_timer.h"

#define APP_AUDIO_UPLINK_QUEUE_LEN 128
#define APP_AUDIO_UPLINK_TASK_STACK_SIZE 4096
#define APP_AUDIO_UPLINK_TASK_PRIORITY 5
#define APP_AUDIO_UPLINK_TASK_CORE 0
#define APP_AUDIO_UPLINK_OVERFLOW_STOP_COUNT 3
#define APP_PKT_FLAG_END 0x02

typedef struct {
    uint16_t session_id;
    uint16_t seq;
    uint16_t capture_elapsed_ms;
    uint8_t codec;
    uint8_t flags;
    uint16_t payload_len;
    uint32_t enqueued_at_ms;
    uint8_t payload[APP_AUDIO_MAX_ADPCM_BYTES];
} app_audio_uplink_item_t;

static const char *TAG = "APP_UPLINK";

static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static app_audio_uplink_callbacks_t s_callbacks;
static app_audio_uplink_stats_t s_stats;
static int64_t s_capture_start_us;

static void app_audio_uplink_reset_stats(uint16_t session_id)
{
    portENTER_CRITICAL(&s_lock);
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.session_id = session_id;
    portEXIT_CRITICAL(&s_lock);
}

void app_audio_uplink_snapshot_stats(app_audio_uplink_stats_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *out = s_stats;
    portEXIT_CRITICAL(&s_lock);
}

void app_audio_uplink_clear(void)
{
    if (s_queue) {
        xQueueReset(s_queue);
    }
    s_capture_start_us = 0;
    app_audio_uplink_reset_stats(0);
}

void app_audio_uplink_start_session(uint16_t session_id, int64_t capture_start_us)
{
    if (s_queue) {
        xQueueReset(s_queue);
    }
    s_capture_start_us = capture_start_us;
    app_audio_uplink_reset_stats(session_id);
}

static void app_audio_uplink_task(void *arg)
{
    (void)arg;
    app_audio_uplink_item_t item;
    uint64_t tx_queue_delay_total_ms = 0;
    uint32_t tx_queue_delay_count = 0;
    uint16_t delay_stats_session_id = 0;

    while (true) {
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) {
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

        portENTER_CRITICAL(&s_lock);
        if (s_stats.session_id == item.session_id) {
            if (s_stats.pending_packets > 0) {
                s_stats.pending_packets--;
            }
            if (delay_stats_session_id != item.session_id) {
                delay_stats_session_id = item.session_id;
                tx_queue_delay_total_ms = 0;
                tx_queue_delay_count = 0;
            }
            tx_queue_delay_total_ms += queue_delay_ms;
            tx_queue_delay_count++;
            if (queue_delay_ms > s_stats.tx_queue_delay_max_ms) {
                s_stats.tx_queue_delay_max_ms = queue_delay_ms;
            }
            s_stats.tx_queue_delay_avg_ms = (uint32_t)(tx_queue_delay_total_ms / tx_queue_delay_count);
            if (item.flags & APP_PKT_FLAG_END) {
                s_stats.end_sent = true;
            }
        }
        portEXIT_CRITICAL(&s_lock);

        if (queue_delay_ms >= 120) {
            uint32_t capture_age_ms = 0;
            int64_t capture_start_us = s_capture_start_us;
            if (capture_start_us > 0 && now_us > capture_start_us) {
                capture_age_ms = (uint32_t)((now_us - capture_start_us) / 1000);
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

esp_err_t app_audio_uplink_init(const app_audio_uplink_callbacks_t *callbacks)
{
    if (callbacks) {
        s_callbacks = *callbacks;
    } else {
        memset(&s_callbacks, 0, sizeof(s_callbacks));
    }

    app_audio_uplink_reset_stats(0);
    s_capture_start_us = 0;

    s_queue = xQueueCreate(APP_AUDIO_UPLINK_QUEUE_LEN, sizeof(app_audio_uplink_item_t));
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(app_audio_uplink_task,
                                            "app_audio_tx",
                                            APP_AUDIO_UPLINK_TASK_STACK_SIZE,
                                            NULL,
                                            APP_AUDIO_UPLINK_TASK_PRIORITY,
                                            &s_task,
                                            APP_AUDIO_UPLINK_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool app_audio_uplink_wait_for_drain(uint16_t session_id, uint32_t timeout_ms)
{
    int64_t start_us = esp_timer_get_time();
    while (true) {
        bool drained;
        portENTER_CRITICAL(&s_lock);
        if (s_stats.session_id != session_id) {
            drained = true;
        } else {
            drained = s_stats.pending_packets == 0 && s_stats.end_sent;
        }
        portEXIT_CRITICAL(&s_lock);

        if (drained) {
            return true;
        }
        if (((esp_timer_get_time() - start_us) / 1000) >= timeout_ms) {
            return false;
        }
        vTaskDelay(1);
    }
}

bool app_audio_uplink_enqueue(uint16_t session_id,
                              uint16_t seq,
                              uint32_t capture_elapsed_ms,
                              uint8_t codec,
                              uint8_t flags,
                              const uint8_t *payload,
                              uint16_t payload_len)
{
    if (!s_queue) {
        return false;
    }

    app_audio_uplink_item_t item;
    memset(&item, 0, sizeof(item));
    item.session_id = session_id;
    item.seq = seq;
    item.capture_elapsed_ms = (capture_elapsed_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t)capture_elapsed_ms;
    item.codec = codec;
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
    if (xQueueSend(s_queue, &item, send_timeout) != pdTRUE) {
        bool request_link_slow_stop = false;
        uint32_t overflow_count = 0;
        portENTER_CRITICAL(&s_lock);
        if (s_stats.session_id == session_id) {
            s_stats.queue_overflows++;
            overflow_count = s_stats.queue_overflows;
            if (payload_len > 0 &&
                !s_stats.overflow_stop_requested &&
                s_stats.queue_overflows >= APP_AUDIO_UPLINK_OVERFLOW_STOP_COUNT) {
                s_stats.overflow_stop_requested = true;
                request_link_slow_stop = true;
            }
        }
        portEXIT_CRITICAL(&s_lock);

        if (request_link_slow_stop) {
            ESP_LOGW(TAG,
                     "Audio TX queue overflow: session=%u overflows=%lu -> stopping capture as LINK_SLOW",
                     session_id,
                     (unsigned long)overflow_count);
            if (s_callbacks.on_link_slow) {
                s_callbacks.on_link_slow(session_id, overflow_count);
            }
        }
        return false;
    }

    UBaseType_t depth = uxQueueMessagesWaiting(s_queue);
    portENTER_CRITICAL(&s_lock);
    if (s_stats.session_id == session_id) {
        s_stats.pending_packets++;
        if (payload_len > 0) {
            s_stats.frames_enqueued++;
        }
        if ((uint32_t)depth > s_stats.queue_high_watermark) {
            s_stats.queue_high_watermark = (uint32_t)depth;
        }
    }
    portEXIT_CRITICAL(&s_lock);

    return true;
}
