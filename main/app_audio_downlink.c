#include "app_audio_downlink.h"

#include <limits.h>
#include <string.h>

#include "Audio_Driver/PCM5101.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define DOWNLINK_RX_TASK_STACK 6144
#define DOWNLINK_PLAY_TASK_STACK 4096
#define DOWNLINK_TASK_PRIO 5
#define DOWNLINK_QUEUE_LEN 32
#define DOWNLINK_FLAG_START 0x01
#define DOWNLINK_FLAG_END   0x02
#define DOWNLINK_PLAYBACK_RATE APP_AUDIO_SAMPLE_RATE_16K
#define DOWNLINK_PLAY_FRAME_MS 20
#define DOWNLINK_PLAY_SAMPLES ((DOWNLINK_PLAYBACK_RATE * DOWNLINK_PLAY_FRAME_MS) / 1000)
#define DOWNLINK_PREBUFFER_MS 240
#define DOWNLINK_PREBUFFER_SAMPLES ((DOWNLINK_PLAYBACK_RATE * DOWNLINK_PREBUFFER_MS) / 1000)
#define DOWNLINK_RING_SAMPLES 8192
#define DOWNLINK_RING_WRITE_WAIT_MS 350
#define DOWNLINK_UNDERRUN_FADE_SAMPLES 24

#define DOWNLINK_STATUS_OK 0
#define DOWNLINK_STATUS_ENQUEUE_FAIL 1
#define DOWNLINK_STATUS_SEQUENCE_FAIL 2
#define DOWNLINK_STATUS_PLAYBACK_FAIL 3

_Static_assert(DOWNLINK_PLAY_SAMPLES <= (APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES * 2), "play chunk too large");

typedef struct {
    uint16_t session_id;
    uint16_t seq;
    uint8_t flags;
    uint8_t codec;
    uint16_t pcm_sample_count;
    int16_t predictor;
    uint8_t step_index;
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
    uint32_t invalid_packets;
    uint32_t unsupported_codec;
    uint32_t ring_write_timeouts;
    uint32_t ring_overflows;
    uint32_t underruns;
    uint32_t i2s_fails;
    uint32_t queue_high_watermark;
} downlink_diag_t;

typedef struct {
    uint32_t packets;
    uint32_t adpcm_bytes;
    uint32_t pcm_samples_in;
    uint32_t pcm_samples_out;
    downlink_diag_t diag;
} downlink_summary_t;

static const char *TAG = "APP_DOWNLINK";
static QueueHandle_t s_queue;
static TaskHandle_t s_rx_task;
static TaskHandle_t s_play_task;
static app_audio_downlink_done_cb_t s_done_cb;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static downlink_state_t s_state;
static uint16_t s_active_session;
static uint16_t s_failed_session;
static uint16_t s_expected_seq;
static bool s_done_sent;
static bool s_input_finished;
static bool s_playback_started;
static uint8_t s_active_codec;
static int16_t s_last_output_sample;
static uint32_t s_session_packets;
static uint32_t s_session_adpcm_bytes;
static uint32_t s_session_pcm_samples_in;
static uint32_t s_session_pcm_samples_out;
static downlink_diag_t s_diag;
static int16_t s_pcm_ring[DOWNLINK_RING_SAMPLES];
static uint16_t s_ring_read;
static uint16_t s_ring_count;
static int16_t s_decode_pcm[APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES * 2];
static int16_t s_resample_pcm[APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES * 4];

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

static TickType_t downlink_delay_ticks(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks > 0 ? ticks : 1;
}

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

    if (st->predictor > INT16_MAX) {
        st->predictor = INT16_MAX;
    } else if (st->predictor < INT16_MIN) {
        st->predictor = INT16_MIN;
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
    if (codec == APP_AUDIO_CODEC_IMA_ADPCM_8K) {
        return APP_AUDIO_SAMPLE_RATE_8K;
    }
    return 0;
}

static uint16_t downlink_output_samples_per_full_packet(uint8_t codec)
{
    uint32_t source_rate = codec_sample_rate(codec);
    uint32_t decoded_samples = APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES * 2;
    if (source_rate == APP_AUDIO_SAMPLE_RATE_8K) {
        return (uint16_t)(decoded_samples * 2);
    }
    if (source_rate == APP_AUDIO_SAMPLE_RATE_12K) {
        return (uint16_t)(((decoded_samples * DOWNLINK_PLAYBACK_RATE) + source_rate - 1) / source_rate);
    }
    return (uint16_t)decoded_samples;
}

static bool decode_adpcm_packet(const downlink_item_t *item, int16_t *out, uint16_t *out_len)
{
    if (item->step_index > 88 || item->pcm_sample_count > (uint16_t)(item->payload_len * 2)) {
        return false;
    }

    ima_state_t st = {
        .predictor = item->predictor,
        .index = item->step_index,
    };
    uint16_t count = 0;
    for (uint16_t i = 0; i < item->payload_len && count < item->pcm_sample_count; ++i) {
        uint8_t b = item->payload[i];
        out[count++] = ima_decode_nibble(&st, b & 0x0F);
        if (count < item->pcm_sample_count) {
            out[count++] = ima_decode_nibble(&st, (b >> 4) & 0x0F);
        }
    }
    *out_len = count;
    return count == item->pcm_sample_count;
}

static uint16_t resample_to_16k(const int16_t *in, uint16_t in_len, uint32_t source_rate, int16_t *out)
{
    if (in_len == 0) {
        return 0;
    }
    if (source_rate == DOWNLINK_PLAYBACK_RATE) {
        memcpy(out, in, (size_t)in_len * sizeof(int16_t));
        return in_len;
    }
    if (source_rate == APP_AUDIO_SAMPLE_RATE_8K) {
        uint16_t count = 0;
        for (uint16_t i = 0; i < in_len; ++i) {
            out[count++] = in[i];
            out[count++] = in[i];
        }
        return count;
    }
    if (source_rate != APP_AUDIO_SAMPLE_RATE_12K) {
        return 0;
    }

    uint32_t out_count = ((uint32_t)in_len * DOWNLINK_PLAYBACK_RATE) / source_rate;
    if (out_count > (APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES * 4)) {
        out_count = APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES * 4;
    }
    for (uint32_t i = 0; i < out_count; ++i) {
        uint64_t src_q16_wide = (((uint64_t)i * (uint64_t)source_rate) << 16) / DOWNLINK_PLAYBACK_RATE;
        uint32_t src_index = (uint32_t)(src_q16_wide >> 16);
        uint32_t frac = (uint32_t)(src_q16_wide & 0xFFFF);
        if (src_index >= in_len) {
            src_index = in_len - 1;
            frac = 0;
        }
        uint32_t next_index = (src_index + 1 < in_len) ? (src_index + 1) : src_index;
        int32_t a = in[src_index];
        int32_t b = in[next_index];
        out[i] = (int16_t)(a + (((b - a) * (int32_t)frac) >> 16));
    }
    return (uint16_t)out_count;
}

static void ring_clear_locked(void)
{
    s_ring_read = 0;
    s_ring_count = 0;
}

static uint16_t ring_free_locked(void)
{
    return (uint16_t)(DOWNLINK_RING_SAMPLES - s_ring_count);
}

static uint16_t ring_count(void)
{
    uint16_t count;
    portENTER_CRITICAL(&s_lock);
    count = s_ring_count;
    portEXIT_CRITICAL(&s_lock);
    return count;
}

static bool input_finished(void)
{
    bool finished;
    portENTER_CRITICAL(&s_lock);
    finished = s_input_finished;
    portEXIT_CRITICAL(&s_lock);
    return finished;
}

static bool playback_active(uint16_t *session_id)
{
    bool active;
    portENTER_CRITICAL(&s_lock);
    active = (s_state == DOWNLINK_STATE_ACTIVE) && !s_done_sent;
    if (session_id) {
        *session_id = s_active_session;
    }
    portEXIT_CRITICAL(&s_lock);
    return active;
}

static uint16_t ring_write_some(const int16_t *samples, uint16_t count)
{
    uint16_t written = 0;
    portENTER_CRITICAL(&s_lock);
    uint16_t free_count = ring_free_locked();
    if (count > free_count) {
        count = free_count;
        s_diag.ring_overflows++;
    }
    for (uint16_t i = 0; i < count; ++i) {
        uint16_t write_index = (uint16_t)((s_ring_read + s_ring_count) % DOWNLINK_RING_SAMPLES);
        s_pcm_ring[write_index] = samples[i];
        s_ring_count++;
        written++;
    }
    portEXIT_CRITICAL(&s_lock);
    return written;
}

static bool ring_write_all(uint16_t session_id, const int16_t *samples, uint16_t count)
{
    uint16_t offset = 0;
    TickType_t start = xTaskGetTickCount();
    while (offset < count) {
        uint16_t active_session = 0;
        if (!playback_active(&active_session) || active_session != session_id) {
            return false;
        }
        uint16_t written = ring_write_some(samples + offset, (uint16_t)(count - offset));
        offset += written;
        if (offset >= count) {
            break;
        }
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= pdMS_TO_TICKS(DOWNLINK_RING_WRITE_WAIT_MS)) {
            portENTER_CRITICAL(&s_lock);
            s_diag.ring_write_timeouts++;
            portEXIT_CRITICAL(&s_lock);
            return false;
        }
        vTaskDelay(downlink_delay_ticks(1));
    }
    return true;
}

static uint16_t ring_read_some(int16_t *out, uint16_t max_count)
{
    uint16_t count = 0;
    portENTER_CRITICAL(&s_lock);
    if (max_count > s_ring_count) {
        max_count = s_ring_count;
    }
    for (uint16_t i = 0; i < max_count; ++i) {
        out[count++] = s_pcm_ring[s_ring_read];
        s_ring_read = (uint16_t)((s_ring_read + 1) % DOWNLINK_RING_SAMPLES);
        s_ring_count--;
    }
    portEXIT_CRITICAL(&s_lock);
    return count;
}

static void downlink_snapshot_summary(downlink_summary_t *summary)
{
    memset(summary, 0, sizeof(*summary));
    portENTER_CRITICAL(&s_lock);
    summary->packets = s_session_packets;
    summary->adpcm_bytes = s_session_adpcm_bytes;
    summary->pcm_samples_in = s_session_pcm_samples_in;
    summary->pcm_samples_out = s_session_pcm_samples_out;
    summary->diag = s_diag;
    portEXIT_CRITICAL(&s_lock);
}

static void downlink_emit_done(uint16_t session_id, uint8_t status)
{
    bool should_send = false;
    portENTER_CRITICAL(&s_lock);
    if (!s_done_sent) {
        s_done_sent = true;
        should_send = true;
    }
    s_input_finished = true;
    s_playback_started = false;
    ring_clear_locked();
    if (status == DOWNLINK_STATUS_OK) {
        s_state = DOWNLINK_STATE_IDLE;
        s_failed_session = 0;
    } else {
        s_state = DOWNLINK_STATE_FAILED;
        s_failed_session = session_id;
    }
    s_active_session = 0;
    s_expected_seq = 0;
    portEXIT_CRITICAL(&s_lock);

    if (!should_send) {
        return;
    }

    if (s_done_cb) {
        s_done_cb(session_id, status);
    }

    downlink_summary_t summary;
    downlink_snapshot_summary(&summary);
    ESP_LOGI(TAG,
             "Downlink summary sid=%u status=%u packets=%lu adpcm_bytes=%lu pcm_in=%lu pcm_out=%lu dup=%lu stale=%lu retry=%lu gaps=%lu invalid=%lu unsupported=%lu ring_timeout=%lu ring_overflow=%lu underruns=%lu i2s=%lu q_high=%lu stack_hw=%lu",
             session_id,
             status,
             (unsigned long)summary.packets,
             (unsigned long)summary.adpcm_bytes,
             (unsigned long)summary.pcm_samples_in,
             (unsigned long)summary.pcm_samples_out,
             (unsigned long)summary.diag.duplicate_packets,
             (unsigned long)summary.diag.stale_packets,
             (unsigned long)summary.diag.late_retry_packets,
             (unsigned long)summary.diag.forward_gaps,
             (unsigned long)summary.diag.invalid_packets,
             (unsigned long)summary.diag.unsupported_codec,
             (unsigned long)summary.diag.ring_write_timeouts,
             (unsigned long)summary.diag.ring_overflows,
             (unsigned long)summary.diag.underruns,
             (unsigned long)summary.diag.i2s_fails,
             (unsigned long)summary.diag.queue_high_watermark,
             (unsigned long)uxTaskGetStackHighWaterMark(NULL));
}

static void downlink_begin_session(const downlink_item_t *item)
{
    portENTER_CRITICAL(&s_lock);
    s_state = DOWNLINK_STATE_ACTIVE;
    s_active_session = item->session_id;
    s_failed_session = 0;
    s_expected_seq = item->seq;
    s_done_sent = false;
    s_input_finished = false;
    s_playback_started = false;
    s_active_codec = item->codec;
    s_last_output_sample = 0;
    s_session_packets = 0;
    s_session_adpcm_bytes = 0;
    s_session_pcm_samples_in = 0;
    s_session_pcm_samples_out = 0;
    memset(&s_diag, 0, sizeof(s_diag));
    ring_clear_locked();
    portEXIT_CRITICAL(&s_lock);
}

static void mark_input_finished(void)
{
    portENTER_CRITICAL(&s_lock);
    s_input_finished = true;
    portEXIT_CRITICAL(&s_lock);
}

static void mark_playback_started(void)
{
    portENTER_CRITICAL(&s_lock);
    s_playback_started = true;
    portEXIT_CRITICAL(&s_lock);
}

static bool playback_started(void)
{
    bool started;
    portENTER_CRITICAL(&s_lock);
    started = s_playback_started;
    portEXIT_CRITICAL(&s_lock);
    return started;
}

static void note_queue_depth(void)
{
    if (!s_queue) {
        return;
    }
    UBaseType_t waiting = uxQueueMessagesWaiting(s_queue);
    portENTER_CRITICAL(&s_lock);
    if (waiting > s_diag.queue_high_watermark) {
        s_diag.queue_high_watermark = waiting;
    }
    portEXIT_CRITICAL(&s_lock);
}

static void fill_underrun_tail(int16_t *chunk, uint16_t start, uint16_t total)
{
    int16_t from = s_last_output_sample;
    for (uint16_t i = start; i < total; ++i) {
        uint16_t fade_pos = (uint16_t)(i - start);
        if (fade_pos < DOWNLINK_UNDERRUN_FADE_SAMPLES) {
            int32_t scale = DOWNLINK_UNDERRUN_FADE_SAMPLES - fade_pos;
            chunk[i] = (int16_t)(((int32_t)from * scale) / DOWNLINK_UNDERRUN_FADE_SAMPLES);
        } else {
            chunk[i] = 0;
        }
    }
    s_last_output_sample = 0;
}

static void note_underrun(uint16_t session_id, uint16_t got, bool input_was_finished)
{
    uint32_t count;
    portENTER_CRITICAL(&s_lock);
    s_diag.underruns++;
    count = s_diag.underruns;
    portEXIT_CRITICAL(&s_lock);

    if (count <= 5 || (count % 25) == 0) {
        ESP_LOGW(TAG,
                 "Downlink underrun sid=%u count=%lu got=%u ring=%u input_finished=%d",
                 session_id,
                 (unsigned long)count,
                 got,
                 ring_count(),
                 input_was_finished ? 1 : 0);
    }
}

static void downlink_rx_task(void *arg)
{
    (void)arg;
    downlink_item_t item;

    while (true) {
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        note_queue_depth();

        if (item.flags & DOWNLINK_FLAG_START) {
            downlink_begin_session(&item);
            ESP_LOGI(TAG,
                     "Downlink start sid=%u seq=%u codec=0x%02X pcm=%u payload=%u",
                     item.session_id,
                     item.seq,
                     item.codec,
                     item.pcm_sample_count,
                     item.payload_len);
        }

        portENTER_CRITICAL(&s_lock);
        downlink_state_t state = s_state;
        uint16_t active_session = s_active_session;
        uint16_t failed_session = s_failed_session;
        uint16_t expected_seq = s_expected_seq;
        portEXIT_CRITICAL(&s_lock);

        if (state == DOWNLINK_STATE_FAILED && item.session_id == failed_session) {
            portENTER_CRITICAL(&s_lock);
            s_diag.stale_packets++;
            portEXIT_CRITICAL(&s_lock);
            continue;
        }
        if (state != DOWNLINK_STATE_ACTIVE || item.session_id != active_session) {
            portENTER_CRITICAL(&s_lock);
            s_diag.stale_packets++;
            portEXIT_CRITICAL(&s_lock);
            continue;
        }

        if (item.seq < expected_seq) {
            portENTER_CRITICAL(&s_lock);
            if ((uint16_t)(item.seq + 1) == expected_seq) {
                s_diag.duplicate_packets++;
            } else {
                s_diag.late_retry_packets++;
            }
            portEXIT_CRITICAL(&s_lock);
            continue;
        }

        if (item.seq > expected_seq) {
            portENTER_CRITICAL(&s_lock);
            s_diag.forward_gaps++;
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGW(TAG,
                     "Downlink sequence gap sid=%u seq=%u expected=%u",
                     item.session_id,
                     item.seq,
                     expected_seq);
            downlink_emit_done(item.session_id, DOWNLINK_STATUS_SEQUENCE_FAIL);
            if (s_queue) {
                xQueueReset(s_queue);
            }
            continue;
        }

        portENTER_CRITICAL(&s_lock);
        s_expected_seq++;
        s_session_packets++;
        s_session_adpcm_bytes += item.payload_len;
        portEXIT_CRITICAL(&s_lock);

        uint32_t source_rate = codec_sample_rate(item.codec);
        if (source_rate == 0) {
            portENTER_CRITICAL(&s_lock);
            s_diag.unsupported_codec++;
            portEXIT_CRITICAL(&s_lock);
            downlink_emit_done(item.session_id, DOWNLINK_STATUS_PLAYBACK_FAIL);
            continue;
        }

        uint16_t decoded_count = 0;
        if (!decode_adpcm_packet(&item, s_decode_pcm, &decoded_count)) {
            portENTER_CRITICAL(&s_lock);
            s_diag.invalid_packets++;
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGW(TAG,
                     "Downlink invalid packet sid=%u seq=%u pcm=%u payload=%u step=%u",
                     item.session_id,
                     item.seq,
                     item.pcm_sample_count,
                     item.payload_len,
                     item.step_index);
            downlink_emit_done(item.session_id, DOWNLINK_STATUS_PLAYBACK_FAIL);
            continue;
        }

        uint16_t output_count = resample_to_16k(s_decode_pcm, decoded_count, source_rate, s_resample_pcm);
        if (output_count == 0) {
            portENTER_CRITICAL(&s_lock);
            s_diag.invalid_packets++;
            portEXIT_CRITICAL(&s_lock);
            downlink_emit_done(item.session_id, DOWNLINK_STATUS_PLAYBACK_FAIL);
            continue;
        }

        portENTER_CRITICAL(&s_lock);
        s_session_pcm_samples_in += decoded_count;
        s_session_pcm_samples_out += output_count;
        portEXIT_CRITICAL(&s_lock);

        if (!ring_write_all(item.session_id, s_resample_pcm, output_count)) {
            ESP_LOGW(TAG, "Downlink ring write failed sid=%u seq=%u out=%u", item.session_id, item.seq, output_count);
            downlink_emit_done(item.session_id, DOWNLINK_STATUS_PLAYBACK_FAIL);
            continue;
        }

        if (item.flags & DOWNLINK_FLAG_END) {
            mark_input_finished();
        }
    }
}

static void downlink_playback_task(void *arg)
{
    (void)arg;
    int16_t chunk[DOWNLINK_PLAY_SAMPLES];
    TickType_t last_wake = xTaskGetTickCount();
    bool cadence_active = false;

    while (true) {
        uint16_t session_id = 0;
        if (!playback_active(&session_id)) {
            cadence_active = false;
            vTaskDelay(downlink_delay_ticks(5));
            continue;
        }

        if (!playback_started()) {
            uint16_t buffered = ring_count();
            bool finished = input_finished();
            if (buffered >= DOWNLINK_PREBUFFER_SAMPLES || (finished && buffered > 0)) {
                mark_playback_started();
                last_wake = xTaskGetTickCount();
                cadence_active = true;
                ESP_LOGI(TAG, "Downlink playback start sid=%u buffered=%u", session_id, buffered);
            } else if (finished && buffered == 0) {
                cadence_active = false;
                downlink_emit_done(session_id, DOWNLINK_STATUS_OK);
                continue;
            } else {
                cadence_active = false;
                vTaskDelay(downlink_delay_ticks(4));
                continue;
            }
        } else if (!cadence_active) {
            last_wake = xTaskGetTickCount();
            cadence_active = true;
        }

        bool finished_before_read = input_finished();
        uint16_t got = ring_read_some(chunk, DOWNLINK_PLAY_SAMPLES);
        if (got == 0) {
            if (finished_before_read) {
                downlink_emit_done(session_id, DOWNLINK_STATUS_OK);
                continue;
            }
            note_underrun(session_id, got, finished_before_read);
            fill_underrun_tail(chunk, 0, DOWNLINK_PLAY_SAMPLES);
            got = DOWNLINK_PLAY_SAMPLES;
        } else if (got < DOWNLINK_PLAY_SAMPLES && !finished_before_read) {
            note_underrun(session_id, got, finished_before_read);
            s_last_output_sample = chunk[got - 1];
            fill_underrun_tail(chunk, got, DOWNLINK_PLAY_SAMPLES);
            got = DOWNLINK_PLAY_SAMPLES;
        } else if (got > 0) {
            s_last_output_sample = chunk[got - 1];
        }

        if (got > 0) {
            bool play_ok = Audio_Play_PCM16_Mono(chunk, got, DOWNLINK_PLAYBACK_RATE);
            if (!play_ok) {
                portENTER_CRITICAL(&s_lock);
                s_diag.i2s_fails++;
                portEXIT_CRITICAL(&s_lock);
                ESP_LOGW(TAG, "Downlink playback write failed sid=%u samples=%u", session_id, got);
                cadence_active = false;
                downlink_emit_done(session_id, DOWNLINK_STATUS_PLAYBACK_FAIL);
                continue;
            }
            vTaskDelayUntil(&last_wake, downlink_delay_ticks(DOWNLINK_PLAY_FRAME_MS));
        }

        if (input_finished() && ring_count() == 0) {
            cadence_active = false;
            downlink_emit_done(session_id, DOWNLINK_STATUS_OK);
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

    if (!s_rx_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            downlink_rx_task,
            "app_downlink_rx",
            DOWNLINK_RX_TASK_STACK,
            NULL,
            DOWNLINK_TASK_PRIO,
            &s_rx_task,
            1);
        if (ok != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_play_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            downlink_playback_task,
            "app_downlink_play",
            DOWNLINK_PLAY_TASK_STACK,
            NULL,
            DOWNLINK_TASK_PRIO,
            &s_play_task,
            1);
        if (ok != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }

    app_audio_downlink_reset();
    return ESP_OK;
}

bool app_audio_downlink_enqueue(uint16_t session_id,
                                uint16_t seq,
                                uint8_t flags,
                                uint8_t codec,
                                uint16_t pcm_sample_count,
                                int16_t predictor,
                                uint8_t step_index,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
    if (!s_queue || !payload || payload_len == 0 || payload_len > APP_AUDIO_DOWNLINK_MAX_ADPCM_BYTES) {
        return false;
    }
    if (pcm_sample_count == 0 || pcm_sample_count > (uint16_t)(payload_len * 2) || step_index > 88) {
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
    item.pcm_sample_count = pcm_sample_count;
    item.predictor = predictor;
    item.step_index = step_index;
    item.payload_len = payload_len;
    memcpy(item.payload, payload, payload_len);

    return xQueueSend(s_queue, &item, 0) == pdTRUE;
}

void app_audio_downlink_reset(void)
{
    if (s_queue) {
        xQueueReset(s_queue);
    }
    portENTER_CRITICAL(&s_lock);
    s_state = DOWNLINK_STATE_IDLE;
    s_active_session = 0;
    s_failed_session = 0;
    s_expected_seq = 0;
    s_done_sent = false;
    s_input_finished = false;
    s_playback_started = false;
    s_active_codec = APP_AUDIO_CODEC_IMA_ADPCM_16K;
    s_last_output_sample = 0;
    s_session_packets = 0;
    s_session_adpcm_bytes = 0;
    s_session_pcm_samples_in = 0;
    s_session_pcm_samples_out = 0;
    memset(&s_diag, 0, sizeof(s_diag));
    ring_clear_locked();
    portEXIT_CRITICAL(&s_lock);
}

bool app_audio_downlink_active(void)
{
    bool active;
    portENTER_CRITICAL(&s_lock);
    active = (s_state == DOWNLINK_STATE_ACTIVE) && !s_done_sent;
    portEXIT_CRITICAL(&s_lock);
    return active;
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

uint8_t app_audio_downlink_available_credits(void)
{
    if (!s_queue) {
        return 0;
    }

    UBaseType_t waiting = uxQueueMessagesWaiting(s_queue);
    uint16_t queue_free = waiting >= DOWNLINK_QUEUE_LEN ? 0 : (uint16_t)(DOWNLINK_QUEUE_LEN - waiting);

    bool active;
    uint16_t ring_free;
    uint8_t codec;
    portENTER_CRITICAL(&s_lock);
    active = (s_state == DOWNLINK_STATE_ACTIVE) && !s_done_sent;
    ring_free = ring_free_locked();
    codec = s_active_codec;
    portEXIT_CRITICAL(&s_lock);

    if (!active) {
        return queue_free > UINT8_MAX ? UINT8_MAX : (uint8_t)queue_free;
    }

    uint16_t samples_per_packet = downlink_output_samples_per_full_packet(codec);
    if (samples_per_packet == 0) {
        return 0;
    }

    uint16_t ring_packet_capacity = ring_free / samples_per_packet;
    uint16_t reserved_by_queue = waiting > UINT16_MAX ? UINT16_MAX : (uint16_t)waiting;
    uint16_t pipeline_credits = ring_packet_capacity > reserved_by_queue
        ? (uint16_t)(ring_packet_capacity - reserved_by_queue)
        : 0;
    uint16_t credits = queue_free < pipeline_credits ? queue_free : pipeline_credits;
    return credits > UINT8_MAX ? UINT8_MAX : (uint8_t)credits;
}
