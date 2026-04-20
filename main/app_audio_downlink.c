#include "app_audio_downlink.h"

#include <string.h>

#include "Audio_Driver/PCM5101.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define DOWNLINK_TASK_STACK 6144
#define DOWNLINK_TASK_PRIO 5
#define DOWNLINK_QUEUE_LEN 24
#define DOWNLINK_FLAG_START 0x01
#define DOWNLINK_FLAG_END   0x02
#define DOWNLINK_PREBUFFER_PACKETS 4
#define DOWNLINK_PREBUFFER_MAX_WAIT_MS 90
#define DOWNLINK_PLAYBACK_FALLBACK_RATE APP_AUDIO_SAMPLE_RATE_16K

typedef struct {
    uint16_t session_id;
    uint16_t seq;
    uint8_t flags;
    uint8_t codec;
    uint16_t payload_len;
    uint8_t payload[APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES];
} downlink_item_t;

typedef struct {
    int predictor;
    int index;
} ima_state_t;

typedef enum {
    DOWNLINK_STATE_IDLE = 0,
    DOWNLINK_STATE_ACTIVE,
    DOWNLINK_STATE_FAILED,
} downlink_state_t;

typedef struct {
    uint32_t duplicate_packets;
    uint32_t stale_packets;
    uint32_t late_retry_packets;
    uint32_t forward_gaps;
    uint32_t i2s_fails;
    uint32_t rate_fallbacks;
} downlink_diag_t;

static const char *TAG = "APP_DOWNLINK";
static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static app_audio_downlink_done_cb_t s_done_cb;

static downlink_state_t s_state;
static uint16_t s_active_session;
static uint16_t s_expected_seq;
static uint16_t s_failed_session;
static bool s_done_sent;
static bool s_force_16k_output;
static ima_state_t s_ima;
static uint32_t s_session_packets;
static uint32_t s_session_adpcm_bytes;
static uint32_t s_session_pcm_samples;
static downlink_diag_t s_diag;
static int16_t s_decode_pcm[APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES * 2];

static const int s_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
};

static const int8_t s_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
};

static int16_t ima_decode_nibble(ima_state_t *st, uint8_t nibble)
{
    int step = s_step_table[st->index];
    int diff = step >> 3;

    if (nibble & 0x01) {
        diff += step >> 2;
    }
    if (nibble & 0x02) {
        diff += step >> 1;
    }
    if (nibble & 0x04) {
        diff += step;
    }

    if (nibble & 0x08) {
        st->predictor -= diff;
    } else {
        st->predictor += diff;
    }

    if (st->predictor > 32767) {
        st->predictor = 32767;
    } else if (st->predictor < -32768) {
        st->predictor = -32768;
    }

    st->index += s_index_table[nibble & 0x0F];
    if (st->index < 0) {
        st->index = 0;
    } else if (st->index > 88) {
        st->index = 88;
    }

    return (int16_t)st->predictor;
}

static uint32_t codec_sample_rate(uint8_t codec)
{
    if (codec == APP_AUDIO_CODEC_IMA_ADPCM_16K) {
        return APP_AUDIO_SAMPLE_RATE_16K;
    }
    if (codec == APP_AUDIO_CODEC_IMA_ADPCM_12K) {
        return APP_AUDIO_SAMPLE_RATE_12K;
    }
    return APP_AUDIO_SAMPLE_RATE_8K;
}

static void decode_adpcm(const uint8_t *in, uint16_t in_len, int16_t *out, uint16_t *out_len)
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < in_len; ++i) {
        uint8_t b = in[i];
        out[count++] = ima_decode_nibble(&s_ima, b & 0x0F);
        out[count++] = ima_decode_nibble(&s_ima, (b >> 4) & 0x0F);
    }
    *out_len = count;
}

static void upsample_2x(const int16_t *in, uint16_t in_len, int16_t *out, uint16_t *out_len)
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < in_len; ++i) {
        int16_t s = in[i];
        out[count++] = s;
        out[count++] = s;
    }
    *out_len = count;
}

static void downlink_reset_session(uint16_t session_id, uint16_t seq)
{
    s_state = DOWNLINK_STATE_ACTIVE;
    s_active_session = session_id;
    s_expected_seq = seq;
    s_failed_session = 0;
    s_done_sent = false;
    s_force_16k_output = false;
    s_ima.predictor = 0;
    s_ima.index = 0;
    s_session_packets = 0;
    s_session_adpcm_bytes = 0;
    s_session_pcm_samples = 0;
    memset(&s_diag, 0, sizeof(s_diag));
}

static void downlink_emit_done(uint16_t session_id, uint8_t status)
{
    if (!s_done_sent && s_done_cb) {
        s_done_cb(session_id, status);
    }
    s_done_sent = true;

    ESP_LOGI(TAG,
             "Downlink summary sid=%u status=%u packets=%lu adpcm_bytes=%lu pcm_samples=%lu dup=%lu stale=%lu retry=%lu gaps=%lu i2s=%lu fallback16k=%lu stack_hw=%lu",
             session_id,
             status,
             (unsigned long)s_session_packets,
             (unsigned long)s_session_adpcm_bytes,
             (unsigned long)s_session_pcm_samples,
             (unsigned long)s_diag.duplicate_packets,
             (unsigned long)s_diag.stale_packets,
             (unsigned long)s_diag.late_retry_packets,
             (unsigned long)s_diag.forward_gaps,
             (unsigned long)s_diag.i2s_fails,
             (unsigned long)s_diag.rate_fallbacks,
             (unsigned long)uxTaskGetStackHighWaterMark(NULL));

    if (status == 0) {
        s_state = DOWNLINK_STATE_IDLE;
        s_failed_session = 0;
    } else {
        s_state = DOWNLINK_STATE_FAILED;
        s_failed_session = session_id;
    }
}

static void downlink_task(void *arg)
{
    (void)arg;
    downlink_item_t item;
    int16_t pcm_16k[APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES * 4];
    bool wait_prebuffer = false;

    while (true) {
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (item.flags & DOWNLINK_FLAG_START) {
            downlink_reset_session(item.session_id, item.seq);
            ESP_LOGI(TAG,
                     "Downlink start sid=%u seq=%u codec=0x%02X q_used=%u",
                     item.session_id,
                     item.seq,
                     item.codec,
                     (unsigned int)(s_queue ? uxQueueMessagesWaiting(s_queue) : 0));
            wait_prebuffer = true;
        }

        if (s_state == DOWNLINK_STATE_FAILED && item.session_id == s_failed_session) {
            s_diag.stale_packets++;
            ESP_LOGD(TAG,
                     "Downlink post-fail packet ignored sid=%u seq=%u expected=%u",
                     item.session_id,
                     item.seq,
                     s_expected_seq);
            continue;
        }

        if (s_state != DOWNLINK_STATE_ACTIVE) {
            s_diag.stale_packets++;
            ESP_LOGD(TAG,
                     "Downlink packet ignored while idle sid=%u seq=%u",
                     item.session_id,
                     item.seq);
            continue;
        }

        if (item.session_id != s_active_session) {
            s_diag.stale_packets++;
            ESP_LOGD(TAG,
                     "Downlink stale packet sid=%u active=%u seq=%u expected=%u",
                     item.session_id,
                     s_active_session,
                     item.seq,
                     s_expected_seq);
            continue;
        }

        if (item.seq < s_expected_seq) {
            if ((uint16_t)(item.seq + 1) == s_expected_seq) {
                s_diag.duplicate_packets++;
            } else {
                s_diag.late_retry_packets++;
            }
            ESP_LOGD(TAG,
                     "Downlink retry packet sid=%u seq=%u expected=%u (ignored)",
                     item.session_id,
                     item.seq,
                     s_expected_seq);
            continue;
        }

        if (item.seq > s_expected_seq) {
            s_diag.forward_gaps++;
            ESP_LOGW(TAG,
                     "Downlink sequence gap sid=%u active=%u seq=%u expected=%u",
                     item.session_id,
                     s_active_session,
                     item.seq,
                     s_expected_seq);
            downlink_emit_done(item.session_id, 2);
            if (s_queue) {
                xQueueReset(s_queue);
            }
            wait_prebuffer = false;
            s_active_session = 0;
            s_expected_seq = 0;
            continue;
        }

        s_expected_seq++;
        s_session_packets++;
        s_session_adpcm_bytes += item.payload_len;

        if (wait_prebuffer && !(item.flags & DOWNLINK_FLAG_END)) {
            TickType_t wait_start = xTaskGetTickCount();
            while (true) {
                UBaseType_t queued = s_queue ? uxQueueMessagesWaiting(s_queue) : 0;
                if ((queued + 1) >= DOWNLINK_PREBUFFER_PACKETS) {
                    break;
                }
                TickType_t elapsed = xTaskGetTickCount() - wait_start;
                if (elapsed >= pdMS_TO_TICKS(DOWNLINK_PREBUFFER_MAX_WAIT_MS)) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            wait_prebuffer = false;
        }

        uint16_t pcm_count = 0;
        decode_adpcm(item.payload, item.payload_len, s_decode_pcm, &pcm_count);
        s_session_pcm_samples += pcm_count;
        if (pcm_count > 0) {
            uint32_t play_rate = codec_sample_rate(item.codec);
            bool play_ok = false;

            if (s_force_16k_output && play_rate == APP_AUDIO_SAMPLE_RATE_8K) {
                uint16_t up_count = 0;
                upsample_2x(s_decode_pcm, pcm_count, pcm_16k, &up_count);
                play_ok = Audio_Play_PCM16_Mono(pcm_16k, up_count, DOWNLINK_PLAYBACK_FALLBACK_RATE);
            } else {
                play_ok = Audio_Play_PCM16_Mono(s_decode_pcm, pcm_count, play_rate);
            }

            if (!play_ok && play_rate == APP_AUDIO_SAMPLE_RATE_8K) {
                uint16_t up_count = 0;
                upsample_2x(s_decode_pcm, pcm_count, pcm_16k, &up_count);
                bool fallback_ok = Audio_Play_PCM16_Mono(pcm_16k, up_count, DOWNLINK_PLAYBACK_FALLBACK_RATE);
                if (fallback_ok) {
                    s_force_16k_output = true;
                    s_diag.rate_fallbacks++;
                    play_ok = true;
                    ESP_LOGW(TAG,
                             "Downlink switched to 16k output sid=%u seq=%u after 8k write fail",
                             item.session_id,
                             item.seq);
                }
            }

            if (!play_ok) {
                s_diag.i2s_fails++;
                ESP_LOGW(TAG,
                         "Downlink playback write failed sid=%u seq=%u pcm=%u q_used=%u rate=%lu",
                         item.session_id,
                         item.seq,
                         pcm_count,
                         (unsigned int)(s_queue ? uxQueueMessagesWaiting(s_queue) : 0),
                         (unsigned long)play_rate);
                downlink_emit_done(item.session_id, 3);
                if (s_queue) {
                    xQueueReset(s_queue);
                }
                wait_prebuffer = false;
                s_active_session = 0;
                s_expected_seq = 0;
                continue;
            }
        }

        if (item.flags & DOWNLINK_FLAG_END) {
            downlink_emit_done(item.session_id, 0);
            s_active_session = 0;
            s_expected_seq = 0;
        }
    }
}

esp_err_t app_audio_downlink_init(app_audio_downlink_done_cb_t done_cb)
{
    s_done_cb = done_cb;
    if (!s_queue) {
        s_queue = xQueueCreate(DOWNLINK_QUEUE_LEN, sizeof(downlink_item_t));
        if (!s_queue) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            downlink_task,
            "app_downlink",
            DOWNLINK_TASK_STACK,
            NULL,
            DOWNLINK_TASK_PRIO,
            &s_task,
            1);
        if (ok != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }

    s_state = DOWNLINK_STATE_IDLE;
    s_active_session = 0;
    s_expected_seq = 0;
    s_failed_session = 0;
    s_done_sent = false;
    s_force_16k_output = false;
    s_ima.predictor = 0;
    s_ima.index = 0;
    s_session_packets = 0;
    s_session_adpcm_bytes = 0;
    s_session_pcm_samples = 0;
    memset(&s_diag, 0, sizeof(s_diag));
    return ESP_OK;
}

bool app_audio_downlink_enqueue(uint16_t session_id,
                                uint16_t seq,
                                uint8_t flags,
                                uint8_t codec,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
    if (!s_queue || !payload || payload_len == 0 || payload_len > APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES) {
        return false;
    }

    if (flags & DOWNLINK_FLAG_START) {
        xQueueReset(s_queue);
    }

    downlink_item_t item;
    memset(&item, 0, sizeof(item));
    item.session_id = session_id;
    item.seq = seq;
    item.flags = flags;
    item.codec = codec;
    item.payload_len = payload_len;
    memcpy(item.payload, payload, payload_len);

    return xQueueSend(s_queue, &item, 0) == pdTRUE;
}

void app_audio_downlink_reset(void)
{
    if (s_queue) {
        xQueueReset(s_queue);
    }
    s_state = DOWNLINK_STATE_IDLE;
    s_active_session = 0;
    s_expected_seq = 0;
    s_failed_session = 0;
    s_done_sent = false;
    s_force_16k_output = false;
    s_ima.predictor = 0;
    s_ima.index = 0;
    s_session_packets = 0;
    s_session_adpcm_bytes = 0;
    s_session_pcm_samples = 0;
    memset(&s_diag, 0, sizeof(s_diag));
}

uint8_t app_audio_downlink_queue_free_slots(void)
{
    if (!s_queue) {
        return 0;
    }
    UBaseType_t waiting = uxQueueMessagesWaiting(s_queue);
    if (waiting >= DOWNLINK_QUEUE_LEN) {
        return 0;
    }
    return (uint8_t)(DOWNLINK_QUEUE_LEN - waiting);
}
