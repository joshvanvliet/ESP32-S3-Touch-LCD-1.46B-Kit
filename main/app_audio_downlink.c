#include "app_audio_downlink.h"

#include <limits.h>
#include <string.h>

#include "Audio_Driver/PCM5101.h"
#include "esp_audio_dec.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_opus_dec.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define DOWNLINK_RX_TASK_STACK 20480
#define DOWNLINK_RX_TASK_STACK_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define DOWNLINK_PLAY_TASK_STACK 5120
#define DOWNLINK_TASK_PRIO 5
#define DOWNLINK_QUEUE_LEN 32
#define DOWNLINK_FLAG_START 0x01
#define DOWNLINK_FLAG_END   0x02
#define DOWNLINK_FLAG_FRAME_START 0x04
#define DOWNLINK_FLAG_FRAME_END   0x08
#define DOWNLINK_PLAY_FRAME_MS 20
#define DOWNLINK_MAX_PLAYBACK_RATE APP_AUDIO_SAMPLE_RATE_24K
#define DOWNLINK_MAX_PLAY_SAMPLES ((DOWNLINK_MAX_PLAYBACK_RATE * DOWNLINK_PLAY_FRAME_MS) / 1000)
#define DOWNLINK_OPUS_FRAME_MS 20
#define DOWNLINK_OPUS_FRAME_SAMPLES ((APP_AUDIO_SAMPLE_RATE_24K * DOWNLINK_OPUS_FRAME_MS) / 1000)
#define DOWNLINK_OPUS_MAX_FRAME_BYTES 1275
#define DOWNLINK_PREBUFFER_MS 320
#define DOWNLINK_RING_SAMPLES 16384
#define DOWNLINK_RING_WRITE_WAIT_MS 350
#define DOWNLINK_UNDERRUN_FADE_SAMPLES 24
#define DOWNLINK_ADPCM_PLAYBACK_GAIN_NUM 1
#define DOWNLINK_ADPCM_PLAYBACK_GAIN_DEN 10
#define DOWNLINK_OPUS_PLAYBACK_GAIN_NUM 1
#define DOWNLINK_OPUS_PLAYBACK_GAIN_DEN 2
#define DOWNLINK_POSTFILTER_PREV_NUM 1
#define DOWNLINK_POSTFILTER_CUR_NUM 1
#define DOWNLINK_POSTFILTER_DEN 2
#define DOWNLINK_CLIP_SAMPLE_THRESHOLD 32760U

#define DOWNLINK_STATUS_OK 0
#define DOWNLINK_STATUS_ENQUEUE_FAIL 1
#define DOWNLINK_STATUS_SEQUENCE_FAIL 2
#define DOWNLINK_STATUS_PLAYBACK_FAIL 3

_Static_assert(sizeof(app_audio_downlink_packet_header_t) == 14, "downlink header must stay 14 bytes");
_Static_assert(DOWNLINK_MAX_PLAY_SAMPLES <= (APP_AUDIO_DOWNLINK_MAX_PAYLOAD_BYTES * 3), "play chunk too large");
_Static_assert(DOWNLINK_OPUS_FRAME_SAMPLES == 480, "opus frame must stay 20ms at 24k");
_Static_assert(DOWNLINK_ADPCM_PLAYBACK_GAIN_DEN > 0, "adpcm playback gain denominator must be positive");
_Static_assert(DOWNLINK_OPUS_PLAYBACK_GAIN_DEN > 0, "opus playback gain denominator must be positive");
_Static_assert(DOWNLINK_POSTFILTER_DEN > 0, "postfilter denominator must be positive");

typedef struct {
    uint16_t session_id;
    uint16_t seq;
    uint8_t flags;
    uint8_t codec;
    uint16_t pcm_sample_count;
    uint16_t codec_param0;
    uint8_t codec_param1;
    uint16_t payload_len;
    uint8_t payload[APP_AUDIO_DOWNLINK_MAX_PAYLOAD_BYTES];
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
    uint32_t source_clip_samples;
    uint32_t playback_clip_samples;
    uint32_t opus_frames;
    uint32_t opus_fragments;
    uint32_t opus_decode_fail;
    uint32_t opus_oversize;
    uint32_t opus_malformed_fragments;
    uint16_t source_peak;
    uint16_t playback_peak;
} downlink_diag_t;

typedef struct {
    uint32_t packets;
    uint32_t encoded_bytes;
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
static uint32_t s_active_sample_rate;
static int16_t s_last_output_sample;
static int16_t s_postfilter_prev_sample;
static bool s_postfilter_has_prev;
static uint32_t s_session_packets;
static uint32_t s_session_encoded_bytes;
static uint32_t s_session_pcm_samples_in;
static uint32_t s_session_pcm_samples_out;
static downlink_diag_t s_diag;
static int16_t *s_pcm_ring;
static uint16_t s_ring_read;
static uint16_t s_ring_count;
static int16_t s_decode_pcm[APP_AUDIO_DOWNLINK_MAX_PAYLOAD_BYTES * 2];
static int16_t s_resample_pcm[APP_AUDIO_DOWNLINK_MAX_PAYLOAD_BYTES * 6];
static esp_audio_dec_handle_t s_opus_decoder;
static bool s_opus_registered;
static uint8_t s_opus_frame[DOWNLINK_OPUS_MAX_FRAME_BYTES];
static uint16_t s_opus_frame_len;
static uint16_t s_opus_frame_pcm_samples;
static bool s_opus_frame_active;
static int16_t s_opus_pcm[DOWNLINK_OPUS_FRAME_SAMPLES];

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
    if (codec == APP_AUDIO_CODEC_OPUS_24K) {
        return APP_AUDIO_SAMPLE_RATE_24K;
    }
    if (codec == APP_AUDIO_CODEC_IMA_ADPCM_24K) {
        return APP_AUDIO_SAMPLE_RATE_24K;
    }
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

static bool is_opus_codec(uint8_t codec)
{
    return codec == APP_AUDIO_CODEC_OPUS_24K;
}

static bool is_adpcm_codec(uint8_t codec)
{
    return codec == APP_AUDIO_CODEC_IMA_ADPCM_24K ||
           codec == APP_AUDIO_CODEC_IMA_ADPCM_16K ||
           codec == APP_AUDIO_CODEC_IMA_ADPCM_12K ||
           codec == APP_AUDIO_CODEC_IMA_ADPCM_8K;
}

static uint16_t downlink_output_samples_per_full_packet(uint8_t codec)
{
    if (is_opus_codec(codec)) {
        return DOWNLINK_OPUS_FRAME_SAMPLES;
    }
    return is_adpcm_codec(codec) ? (uint16_t)(APP_AUDIO_DOWNLINK_MAX_PAYLOAD_BYTES * 2) : 0;
}

static bool decode_adpcm_packet(const downlink_item_t *item, int16_t *out, uint16_t *out_len)
{
    uint8_t step_index = item->codec_param1;
    if (step_index > 88 || item->pcm_sample_count > (uint16_t)(item->payload_len * 2)) {
        return false;
    }

    ima_state_t st = {
        .predictor = (int16_t)item->codec_param0,
        .index = step_index,
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

static void reset_opus_fragment_state(void)
{
    s_opus_frame_len = 0;
    s_opus_frame_pcm_samples = 0;
    s_opus_frame_active = false;
}

static bool ensure_opus_decoder(void)
{
    if (s_opus_decoder) {
        return true;
    }

    if (!s_opus_registered) {
        esp_audio_err_t reg = esp_opus_dec_register();
        if (reg != ESP_AUDIO_ERR_OK && reg != ESP_AUDIO_ERR_ALREADY_EXIST) {
            ESP_LOGE(TAG, "Opus decoder register failed ret=%d", (int)reg);
            return false;
        }
        s_opus_registered = true;
    }

    esp_opus_dec_cfg_t opus_cfg = ESP_OPUS_DEC_CONFIG_DEFAULT();
    opus_cfg.sample_rate = APP_AUDIO_SAMPLE_RATE_24K;
    opus_cfg.channel = ESP_AUDIO_MONO;
    opus_cfg.frame_duration = ESP_OPUS_DEC_FRAME_DURATION_20_MS;
    opus_cfg.self_delimited = false;

    esp_audio_dec_cfg_t dec_cfg = {
        .type = ESP_AUDIO_TYPE_OPUS,
        .cfg = &opus_cfg,
        .cfg_sz = sizeof(opus_cfg),
    };
    esp_audio_err_t ret = esp_audio_dec_open(&dec_cfg, &s_opus_decoder);
    if (ret != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "Opus decoder open failed ret=%d", (int)ret);
        s_opus_decoder = NULL;
        return false;
    }
    ESP_LOGI(TAG,
             "Opus decoder opened sr=%u frame_ms=%u rx_stack_hw=%lu",
             APP_AUDIO_SAMPLE_RATE_24K,
             DOWNLINK_OPUS_FRAME_MS,
             (unsigned long)uxTaskGetStackHighWaterMark(NULL));
    return true;
}

static bool decode_opus_frame(int16_t *out, uint16_t *out_len)
{
    *out_len = 0;
    if (!s_opus_frame_active || s_opus_frame_len == 0 || !ensure_opus_decoder()) {
        return false;
    }

    esp_audio_dec_in_raw_t raw = {
        .buffer = s_opus_frame,
        .len = s_opus_frame_len,
        .consumed = 0,
    };
    esp_audio_dec_out_frame_t frame = {
        .buffer = (uint8_t *)out,
        .len = DOWNLINK_OPUS_FRAME_SAMPLES * sizeof(int16_t),
        .needed_size = 0,
        .decoded_size = 0,
    };

    esp_audio_err_t ret = esp_audio_dec_process(s_opus_decoder, &raw, &frame);
    if (ret != ESP_AUDIO_ERR_OK ||
        raw.consumed != s_opus_frame_len ||
        frame.decoded_size != (uint32_t)(s_opus_frame_pcm_samples * sizeof(int16_t))) {
        ESP_LOGW(TAG,
                 "Opus decode failed ret=%d consumed=%lu/%u decoded=%lu expected=%u",
                 (int)ret,
                 (unsigned long)raw.consumed,
                 s_opus_frame_len,
                 (unsigned long)frame.decoded_size,
                 (unsigned)(s_opus_frame_pcm_samples * sizeof(int16_t)));
        return false;
    }

    esp_audio_dec_info_t info;
    if (esp_audio_dec_get_info(s_opus_decoder, &info) == ESP_AUDIO_ERR_OK) {
        if (info.sample_rate != APP_AUDIO_SAMPLE_RATE_24K || info.channel != ESP_AUDIO_MONO || info.bits_per_sample != 16) {
            ESP_LOGW(TAG,
                     "Opus decode info mismatch sr=%lu ch=%u bits=%u",
                     (unsigned long)info.sample_rate,
                     info.channel,
                     info.bits_per_sample);
            return false;
        }
    }

    *out_len = (uint16_t)(frame.decoded_size / sizeof(int16_t));
    return *out_len == s_opus_frame_pcm_samples;
}

static uint16_t downlink_samples_for_ms(uint32_t sample_rate, uint32_t ms)
{
    if (sample_rate == 0) {
        sample_rate = APP_AUDIO_SAMPLE_RATE_16K;
    }
    return (uint16_t)((sample_rate * ms) / 1000);
}

static uint32_t playback_sample_rate(void)
{
    uint32_t rate;
    portENTER_CRITICAL(&s_lock);
    rate = s_active_sample_rate;
    portEXIT_CRITICAL(&s_lock);
    return rate > 0 ? rate : APP_AUDIO_SAMPLE_RATE_16K;
}

static uint16_t resample_to_playback_rate(const int16_t *in,
                                          uint16_t in_len,
                                          uint32_t source_rate,
                                          uint32_t target_rate,
                                          int16_t *out)
{
    if (in_len == 0) {
        return 0;
    }
    if (source_rate == 0 || target_rate == 0) {
        return 0;
    }
    if (source_rate == target_rate) {
        memcpy(out, in, (size_t)in_len * sizeof(int16_t));
        return in_len;
    }

    uint32_t out_capacity = APP_AUDIO_DOWNLINK_MAX_PAYLOAD_BYTES * 6;
    uint32_t out_count = ((uint32_t)in_len * target_rate) / source_rate;
    if (out_count > out_capacity) {
        out_count = out_capacity;
    }
    for (uint32_t i = 0; i < out_count; ++i) {
        uint64_t src_q16_wide = (((uint64_t)i * (uint64_t)source_rate) << 16) / target_rate;
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

static void reset_playback_postfilter_locked(void)
{
    s_postfilter_prev_sample = 0;
    s_postfilter_has_prev = false;
}

static uint16_t ring_free_locked(void)
{
    if (!s_pcm_ring) {
        return 0;
    }
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
    if (!s_pcm_ring) {
        return 0;
    }

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
    if (!s_pcm_ring) {
        return 0;
    }

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
    summary->encoded_bytes = s_session_encoded_bytes;
    summary->pcm_samples_in = s_session_pcm_samples_in;
    summary->pcm_samples_out = s_session_pcm_samples_out;
    summary->diag = s_diag;
    portEXIT_CRITICAL(&s_lock);
}

static void log_task_create_failed(const char *name, uint32_t stack_bytes, UBaseType_t caps)
{
    ESP_LOGE(TAG,
             "Failed to create task %s stack=%lu caps=0x%lx internal_free=%lu internal_largest=%lu psram_free=%lu psram_largest=%lu",
             name,
             (unsigned long)stack_bytes,
             (unsigned long)caps,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
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
             "Downlink summary sid=%u status=%u packets=%lu encoded_bytes=%lu pcm_in=%lu pcm_out=%lu src_peak=%u src_clip=%lu play_peak=%u play_clip=%lu opus_frames=%lu opus_fragments=%lu opus_decode_fail=%lu opus_oversize=%lu opus_malformed=%lu dup=%lu stale=%lu retry=%lu gaps=%lu invalid=%lu unsupported=%lu ring_timeout=%lu ring_overflow=%lu underruns=%lu i2s=%lu q_high=%lu stack_hw=%lu",
             session_id,
             status,
             (unsigned long)summary.packets,
             (unsigned long)summary.encoded_bytes,
             (unsigned long)summary.pcm_samples_in,
             (unsigned long)summary.pcm_samples_out,
             (unsigned int)summary.diag.source_peak,
             (unsigned long)summary.diag.source_clip_samples,
             (unsigned int)summary.diag.playback_peak,
             (unsigned long)summary.diag.playback_clip_samples,
             (unsigned long)summary.diag.opus_frames,
             (unsigned long)summary.diag.opus_fragments,
             (unsigned long)summary.diag.opus_decode_fail,
             (unsigned long)summary.diag.opus_oversize,
             (unsigned long)summary.diag.opus_malformed_fragments,
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
    uint32_t sample_rate = codec_sample_rate(item->codec);
    if (sample_rate == 0) {
        sample_rate = APP_AUDIO_SAMPLE_RATE_16K;
    }

    portENTER_CRITICAL(&s_lock);
    s_state = DOWNLINK_STATE_ACTIVE;
    s_active_session = item->session_id;
    s_failed_session = 0;
    s_expected_seq = item->seq;
    s_done_sent = false;
    s_input_finished = false;
    s_playback_started = false;
    s_active_codec = item->codec;
    s_active_sample_rate = sample_rate;
    s_last_output_sample = 0;
    reset_playback_postfilter_locked();
    s_session_packets = 0;
    s_session_encoded_bytes = 0;
    s_session_pcm_samples_in = 0;
    s_session_pcm_samples_out = 0;
    memset(&s_diag, 0, sizeof(s_diag));
    reset_opus_fragment_state();
    ring_clear_locked();
    portEXIT_CRITICAL(&s_lock);

    if (item->codec == APP_AUDIO_CODEC_OPUS_24K && s_opus_decoder) {
        esp_audio_dec_reset(s_opus_decoder);
    }
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

static uint16_t pcm_abs16(int16_t sample)
{
    if (sample == INT16_MIN) {
        return 32768U;
    }
    return (uint16_t)(sample < 0 ? -sample : sample);
}

static void note_pcm_level(const int16_t *samples, uint16_t count, bool playback)
{
    uint16_t peak = 0;
    uint32_t clip_samples = 0;

    for (uint16_t i = 0; i < count; ++i) {
        uint16_t mag = pcm_abs16(samples[i]);
        if (mag > peak) {
            peak = mag;
        }
        if (mag >= DOWNLINK_CLIP_SAMPLE_THRESHOLD) {
            clip_samples++;
        }
    }

    portENTER_CRITICAL(&s_lock);
    if (playback) {
        if (peak > s_diag.playback_peak) {
            s_diag.playback_peak = peak;
        }
        s_diag.playback_clip_samples += clip_samples;
    } else {
        if (peak > s_diag.source_peak) {
            s_diag.source_peak = peak;
        }
        s_diag.source_clip_samples += clip_samples;
    }
    portEXIT_CRITICAL(&s_lock);
}

static void apply_playback_postfilter(int16_t *samples, uint16_t count)
{
    if (count == 0 ||
        (DOWNLINK_POSTFILTER_PREV_NUM == 0 &&
         DOWNLINK_POSTFILTER_CUR_NUM == DOWNLINK_POSTFILTER_DEN)) {
        return;
    }

    int32_t prev = s_postfilter_has_prev ? s_postfilter_prev_sample : samples[0];
    for (uint16_t i = 0; i < count; ++i) {
        int32_t current = samples[i];
        int32_t filtered =
            ((prev * DOWNLINK_POSTFILTER_PREV_NUM) +
             (current * DOWNLINK_POSTFILTER_CUR_NUM)) /
            DOWNLINK_POSTFILTER_DEN;
        if (filtered > INT16_MAX) {
            filtered = INT16_MAX;
        } else if (filtered < INT16_MIN) {
            filtered = INT16_MIN;
        }
        samples[i] = (int16_t)filtered;
        prev = current;
    }
    s_postfilter_prev_sample = (int16_t)prev;
    s_postfilter_has_prev = true;
}

static void apply_playback_gain(int16_t *samples, uint16_t count)
{
    uint8_t codec;
    portENTER_CRITICAL(&s_lock);
    codec = s_active_codec;
    portEXIT_CRITICAL(&s_lock);

    int gain_num = is_opus_codec(codec) ? DOWNLINK_OPUS_PLAYBACK_GAIN_NUM : DOWNLINK_ADPCM_PLAYBACK_GAIN_NUM;
    int gain_den = is_opus_codec(codec) ? DOWNLINK_OPUS_PLAYBACK_GAIN_DEN : DOWNLINK_ADPCM_PLAYBACK_GAIN_DEN;
    if (gain_num == gain_den) {
        return;
    }

    for (uint16_t i = 0; i < count; ++i) {
        int32_t scaled = ((int32_t)samples[i] * gain_num) / gain_den;
        if (scaled > INT16_MAX) {
            scaled = INT16_MAX;
        } else if (scaled < INT16_MIN) {
            scaled = INT16_MIN;
        }
        samples[i] = (int16_t)scaled;
    }
}

static void playback_gain(uint8_t codec, int *num, int *den)
{
    if (is_opus_codec(codec)) {
        *num = DOWNLINK_OPUS_PLAYBACK_GAIN_NUM;
        *den = DOWNLINK_OPUS_PLAYBACK_GAIN_DEN;
    } else {
        *num = DOWNLINK_ADPCM_PLAYBACK_GAIN_NUM;
        *den = DOWNLINK_ADPCM_PLAYBACK_GAIN_DEN;
    }
}

static bool playback_uses_postfilter(uint8_t codec)
{
    return is_adpcm_codec(codec);
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

static bool process_opus_item(const downlink_item_t *item)
{
    bool frame_start = (item->flags & DOWNLINK_FLAG_FRAME_START) != 0;
    bool frame_end = (item->flags & DOWNLINK_FLAG_FRAME_END) != 0;

    if (item->pcm_sample_count != DOWNLINK_OPUS_FRAME_SAMPLES ||
        item->codec_param0 != 0 ||
        item->codec_param1 != DOWNLINK_OPUS_FRAME_MS) {
        portENTER_CRITICAL(&s_lock);
        s_diag.invalid_packets++;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGW(TAG,
                 "Downlink invalid Opus params sid=%u seq=%u pcm=%u param0=%u param1=%u",
                 item->session_id,
                 item->seq,
                 item->pcm_sample_count,
                 item->codec_param0,
                 item->codec_param1);
        return false;
    }

    if (frame_start) {
        if (s_opus_frame_active) {
            portENTER_CRITICAL(&s_lock);
            s_diag.opus_malformed_fragments++;
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGW(TAG, "Downlink Opus frame restarted before end sid=%u seq=%u", item->session_id, item->seq);
            reset_opus_fragment_state();
            return false;
        }
        reset_opus_fragment_state();
        s_opus_frame_active = true;
        s_opus_frame_pcm_samples = item->pcm_sample_count;
    } else if (!s_opus_frame_active) {
        portENTER_CRITICAL(&s_lock);
        s_diag.opus_malformed_fragments++;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGW(TAG, "Downlink Opus fragment without frame start sid=%u seq=%u", item->session_id, item->seq);
        return false;
    }

    if ((uint32_t)s_opus_frame_len + item->payload_len > DOWNLINK_OPUS_MAX_FRAME_BYTES) {
        portENTER_CRITICAL(&s_lock);
        s_diag.opus_oversize++;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGW(TAG,
                 "Downlink Opus frame oversize sid=%u seq=%u len=%u add=%u",
                 item->session_id,
                 item->seq,
                 s_opus_frame_len,
                 item->payload_len);
        reset_opus_fragment_state();
        return false;
    }

    memcpy(s_opus_frame + s_opus_frame_len, item->payload, item->payload_len);
    s_opus_frame_len += item->payload_len;
    portENTER_CRITICAL(&s_lock);
    s_diag.opus_fragments++;
    portEXIT_CRITICAL(&s_lock);

    if (frame_end) {
        uint16_t decoded_count = 0;
        if (!decode_opus_frame(s_opus_pcm, &decoded_count)) {
            portENTER_CRITICAL(&s_lock);
            s_diag.opus_decode_fail++;
            portEXIT_CRITICAL(&s_lock);
            reset_opus_fragment_state();
            return false;
        }

        portENTER_CRITICAL(&s_lock);
        s_diag.opus_frames++;
        s_session_pcm_samples_in += decoded_count;
        s_session_pcm_samples_out += decoded_count;
        portEXIT_CRITICAL(&s_lock);
        note_pcm_level(s_opus_pcm, decoded_count, false);

        if (!ring_write_all(item->session_id, s_opus_pcm, decoded_count)) {
            ESP_LOGW(TAG, "Downlink Opus ring write failed sid=%u seq=%u out=%u", item->session_id, item->seq, decoded_count);
            reset_opus_fragment_state();
            return false;
        }
        reset_opus_fragment_state();
    }

    if ((item->flags & DOWNLINK_FLAG_END) != 0) {
        if (s_opus_frame_active) {
            portENTER_CRITICAL(&s_lock);
            s_diag.opus_malformed_fragments++;
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGW(TAG, "Downlink Opus stream ended with partial frame sid=%u seq=%u", item->session_id, item->seq);
            reset_opus_fragment_state();
            return false;
        }
        mark_input_finished();
    }

    return true;
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
        s_session_encoded_bytes += item.payload_len;
        portEXIT_CRITICAL(&s_lock);

        uint32_t source_rate = codec_sample_rate(item.codec);
        if (source_rate == 0) {
            portENTER_CRITICAL(&s_lock);
            s_diag.unsupported_codec++;
            portEXIT_CRITICAL(&s_lock);
            downlink_emit_done(item.session_id, DOWNLINK_STATUS_PLAYBACK_FAIL);
            continue;
        }

        if (is_opus_codec(item.codec)) {
            if (!process_opus_item(&item)) {
                downlink_emit_done(item.session_id, DOWNLINK_STATUS_PLAYBACK_FAIL);
            }
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
                     item.codec_param1);
            downlink_emit_done(item.session_id, DOWNLINK_STATUS_PLAYBACK_FAIL);
            continue;
        }

        uint32_t target_rate = playback_sample_rate();
        uint16_t output_count = resample_to_playback_rate(
            s_decode_pcm,
            decoded_count,
            source_rate,
            target_rate,
            s_resample_pcm);
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
        note_pcm_level(s_resample_pcm, output_count, false);

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
    int16_t chunk[DOWNLINK_MAX_PLAY_SAMPLES];
    TickType_t last_wake = xTaskGetTickCount();
    bool cadence_active = false;

    while (true) {
        uint16_t session_id = 0;
        if (!playback_active(&session_id)) {
            cadence_active = false;
            vTaskDelay(downlink_delay_ticks(5));
            continue;
        }

        uint32_t sample_rate = playback_sample_rate();
        uint16_t play_samples = downlink_samples_for_ms(sample_rate, DOWNLINK_PLAY_FRAME_MS);
        if (play_samples == 0 || play_samples > DOWNLINK_MAX_PLAY_SAMPLES) {
            sample_rate = APP_AUDIO_SAMPLE_RATE_16K;
            play_samples = downlink_samples_for_ms(sample_rate, DOWNLINK_PLAY_FRAME_MS);
        }

        if (!playback_started()) {
            uint16_t buffered = ring_count();
            bool finished = input_finished();
            uint16_t prebuffer_samples = downlink_samples_for_ms(sample_rate, DOWNLINK_PREBUFFER_MS);
            if (buffered >= prebuffer_samples || (finished && buffered > 0)) {
                mark_playback_started();
                last_wake = xTaskGetTickCount();
                cadence_active = true;
                uint8_t codec;
                portENTER_CRITICAL(&s_lock);
                codec = s_active_codec;
                portEXIT_CRITICAL(&s_lock);
                int gain_num = 1;
                int gain_den = 1;
                playback_gain(codec, &gain_num, &gain_den);
                ESP_LOGI(TAG,
                         "Downlink playback start sid=%u codec=0x%02X rate=%lu buffered=%u prebuffer=%u gain=%d/%d postfilter=%d",
                         session_id,
                         codec,
                         (unsigned long)sample_rate,
                         buffered,
                         prebuffer_samples,
                         gain_num,
                         gain_den,
                         playback_uses_postfilter(codec) ? 1 : 0);
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
        uint16_t got = ring_read_some(chunk, play_samples);
        if (got == 0) {
            if (finished_before_read) {
                downlink_emit_done(session_id, DOWNLINK_STATUS_OK);
                continue;
            }
            note_underrun(session_id, got, finished_before_read);
            fill_underrun_tail(chunk, 0, play_samples);
            got = play_samples;
        } else if (got < play_samples && !finished_before_read) {
            note_underrun(session_id, got, finished_before_read);
            s_last_output_sample = chunk[got - 1];
            fill_underrun_tail(chunk, got, play_samples);
            got = play_samples;
        } else if (got > 0) {
            s_last_output_sample = chunk[got - 1];
        }

        if (got > 0) {
            uint8_t codec;
            portENTER_CRITICAL(&s_lock);
            codec = s_active_codec;
            portEXIT_CRITICAL(&s_lock);
            if (playback_uses_postfilter(codec)) {
                apply_playback_postfilter(chunk, got);
            }
            apply_playback_gain(chunk, got);
            note_pcm_level(chunk, got, true);
            bool play_ok = Audio_Play_PCM16_Mono(chunk, got, sample_rate);
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

    if (!s_pcm_ring) {
        s_pcm_ring = heap_caps_calloc(DOWNLINK_RING_SAMPLES, sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_pcm_ring) {
            ESP_LOGE(TAG,
                     "Failed to allocate downlink PCM ring in PSRAM samples=%u bytes=%u",
                     DOWNLINK_RING_SAMPLES,
                     (unsigned)(DOWNLINK_RING_SAMPLES * sizeof(int16_t)));
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG,
                 "Downlink PCM ring allocated in PSRAM samples=%u bytes=%u",
                 DOWNLINK_RING_SAMPLES,
                 (unsigned)(DOWNLINK_RING_SAMPLES * sizeof(int16_t)));
    }

    if (!s_queue) {
        s_queue = xQueueCreate(DOWNLINK_QUEUE_LEN, sizeof(downlink_item_t));
        if (!s_queue) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_rx_task) {
        BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(
            downlink_rx_task,
            "app_downlink_rx",
            DOWNLINK_RX_TASK_STACK,
            NULL,
            DOWNLINK_TASK_PRIO,
            &s_rx_task,
            1,
            DOWNLINK_RX_TASK_STACK_CAPS);
        if (ok != pdPASS) {
            log_task_create_failed("app_downlink_rx", DOWNLINK_RX_TASK_STACK, DOWNLINK_RX_TASK_STACK_CAPS);
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
            log_task_create_failed("app_downlink_play", DOWNLINK_PLAY_TASK_STACK, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
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
                                uint16_t codec_param0,
                                uint8_t codec_param1,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
    if (!s_queue || !payload || payload_len == 0 || payload_len > APP_AUDIO_DOWNLINK_MAX_PAYLOAD_BYTES) {
        return false;
    }
    if (pcm_sample_count == 0) {
        return false;
    }
    if (is_adpcm_codec(codec)) {
        if (pcm_sample_count > (uint16_t)(payload_len * 2) || codec_param1 > 88) {
            return false;
        }
    } else if (is_opus_codec(codec)) {
        if (pcm_sample_count != DOWNLINK_OPUS_FRAME_SAMPLES ||
            codec_param0 != 0 ||
            codec_param1 != DOWNLINK_OPUS_FRAME_MS) {
            return false;
        }
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
    item.codec_param0 = codec_param0;
    item.codec_param1 = codec_param1;
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
    s_active_sample_rate = APP_AUDIO_SAMPLE_RATE_16K;
    s_last_output_sample = 0;
    reset_playback_postfilter_locked();
    s_session_packets = 0;
    s_session_encoded_bytes = 0;
    s_session_pcm_samples_in = 0;
    s_session_pcm_samples_out = 0;
    memset(&s_diag, 0, sizeof(s_diag));
    reset_opus_fragment_state();
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
