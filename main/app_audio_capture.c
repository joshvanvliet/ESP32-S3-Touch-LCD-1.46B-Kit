#include "app_audio_capture.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define APP_I2S_PORT I2S_NUM_1
#define APP_VAD_CALIBRATION_MS 300
#define APP_VAD_NO_SPEECH_TIMEOUT_MS 2500
#define APP_VAD_MIN_SPEECH_MS 400
#define APP_VAD_SILENCE_HANGOVER_MS 2500
#define APP_CAPTURE_MAX_MS 12000
#define APP_I2S_READ_TIMEOUT_MS 120
#define APP_MIC_LEVEL_LOG_INTERVAL_MS 250
#define APP_I2S_RECOVERY_TIMEOUTS 200
#define APP_CAPTURE_TASK_STACK_SIZE 8192
#define APP_CAPTURE_TASK_PRIORITY 4
#define APP_CAPTURE_TASK_CORE 1

#define APP_PKT_FLAG_START 0x01
#define APP_PKT_FLAG_END   0x02

#define APP_MIC_I2S_USE_RIGHT_SLOT 1

static const char *TAG = "APP_AUDIO";
static bool s_use_right_slot = APP_MIC_I2S_USE_RIGHT_SLOT ? true : false;

typedef struct {
    int predictor;
    int index;
} app_adpcm_state_t;

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

static i2s_chan_handle_t s_rx_handle;
static bool s_i2s_ready;
static bool s_running;
static volatile bool s_stop_requested;
static volatile app_capture_stop_reason_t s_requested_stop_reason;
static uint16_t s_session_id;
static uint8_t s_capture_codec = APP_AUDIO_CODEC_IMA_ADPCM_16K;
static uint32_t s_capture_sample_rate_hz = APP_AUDIO_SAMPLE_RATE_16K;
static uint16_t s_pcm_samples_per_frame = (uint16_t)((APP_AUDIO_SAMPLE_RATE_16K * APP_AUDIO_FRAME_MS) / 1000);
static app_audio_capture_callbacks_t s_callbacks;
static TaskHandle_t s_capture_task;
/* Single capture task instance: keep frame buffers out of task stack. */
static int32_t s_i2s_frame[APP_AUDIO_MAX_PCM_SAMPLES_PER_FRAME];
static int16_t s_pcm_frame[APP_AUDIO_MAX_PCM_SAMPLES_PER_FRAME];
static uint8_t s_adpcm_frame[APP_AUDIO_MAX_ADPCM_BYTES];

static const char *app_audio_slot_name(bool use_right_slot)
{
    return use_right_slot ? "RIGHT" : "LEFT";
}

static void app_audio_build_slot_cfg(i2s_std_slot_config_t *slot_cfg, bool use_right_slot)
{
    i2s_std_slot_config_t cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
    *slot_cfg = cfg;
    slot_cfg->slot_mask = use_right_slot ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT;
}

static esp_err_t app_audio_reconfig_slot(bool use_right_slot)
{
    if (!s_rx_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    i2s_std_slot_config_t slot_cfg;
    app_audio_build_slot_cfg(&slot_cfg, use_right_slot);

    esp_err_t ret = i2s_channel_disable(s_rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_disable failed during slot switch: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2s_channel_reconfig_std_slot(s_rx_handle, &slot_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_reconfig_std_slot failed: %s", esp_err_to_name(ret));
        (void)i2s_channel_enable(s_rx_handle);
        return ret;
    }
    ret = i2s_channel_enable(s_rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_enable failed after slot switch: %s", esp_err_to_name(ret));
        return ret;
    }

    s_use_right_slot = use_right_slot;
    ESP_LOGW(TAG, "I2S RX slot switched to %s", app_audio_slot_name(s_use_right_slot));
    return ESP_OK;
}

static void app_audio_drop_stale_rx_samples(void)
{
    if (!s_rx_handle) {
        return;
    }

    int32_t scratch[APP_AUDIO_MAX_PCM_SAMPLES_PER_FRAME];
    size_t bytes_read = 0;
    uint32_t dropped_bytes = 0;

    /* Drain only what is already buffered; do not restart the RX channel. */
    for (int i = 0; i < 24; ++i) {
        esp_err_t err = i2s_channel_read(s_rx_handle, scratch, sizeof(scratch), &bytes_read, 0);
        if (err != ESP_OK || bytes_read == 0) {
            break;
        }
        dropped_bytes += (uint32_t)bytes_read;
    }

    if (dropped_bytes > 0) {
        ESP_LOGI(TAG, "Dropped stale RX bytes=%lu before capture", (unsigned long)dropped_bytes);
    }
}

static uint8_t app_adpcm_encode_sample(app_adpcm_state_t *st, int16_t sample)
{
    int step = s_step_table[st->index];
    int diff = sample - st->predictor;
    uint8_t nibble = 0;

    if (diff < 0) {
        nibble = 8;
        diff = -diff;
    }

    int tempstep = step;
    int delta = step >> 3;

    if (diff >= tempstep) {
        nibble |= 4;
        diff -= tempstep;
        delta += step;
    }
    tempstep >>= 1;
    if (diff >= tempstep) {
        nibble |= 2;
        diff -= tempstep;
        delta += step >> 1;
    }
    tempstep >>= 1;
    if (diff >= tempstep) {
        nibble |= 1;
        delta += step >> 2;
    }

    if (nibble & 8) {
        st->predictor -= delta;
    } else {
        st->predictor += delta;
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

    return nibble & 0x0F;
}

static uint16_t app_adpcm_encode_frame(app_adpcm_state_t *st, const int16_t *pcm, uint16_t samples, uint8_t *out)
{
    uint16_t out_idx = 0;
    for (uint16_t i = 0; i < samples; i += 2) {
        uint8_t lo = app_adpcm_encode_sample(st, pcm[i]);
        uint8_t hi = 0;
        if ((i + 1) < samples) {
            hi = app_adpcm_encode_sample(st, pcm[i + 1]);
        }
        out[out_idx++] = (uint8_t)(lo | (hi << 4));
    }
    return out_idx;
}

static esp_err_t app_audio_i2s_init(void)
{
    if (s_i2s_ready) {
        return ESP_OK;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(APP_I2S_PORT, I2S_ROLE_MASTER);
    /* Increase DMA ring to 6 × 320 frames = 120 ms of buffered audio at 16 kHz.
     * Each descriptor: 320 frames × 4 bytes (32-bit mono) = 1280 bytes (< 4092 limit).
     * This prevents I2S DMA overflow during brief scheduling jitter now that the
     * capture loop is decoupled from BLE notify. */
    chan_cfg.dma_desc_num  = 6;
    chan_cfg.dma_frame_num = 320;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_capture_sample_rate_hz),
        .slot_cfg = {
            0
        },
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = GPIO_NUM_15,
            .ws = GPIO_NUM_2,
            .dout = GPIO_NUM_NC,
            .din = GPIO_NUM_39,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    app_audio_build_slot_cfg(&std_cfg.slot_cfg, s_use_right_slot);

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_handle));

    s_i2s_ready = true;
    ESP_LOGI(TAG, "I2S capture initialized (port=%d, sr=%lu, slot=%s, pins bclk=%d ws=%d din=%d)",
             APP_I2S_PORT,
             (unsigned long)s_capture_sample_rate_hz,
             app_audio_slot_name(s_use_right_slot),
             GPIO_NUM_15,
             GPIO_NUM_2,
             GPIO_NUM_39);
    return ESP_OK;
}

static void app_audio_i2s_deinit(void)
{
    if (!s_rx_handle) {
        s_i2s_ready = false;
        return;
    }

    (void)i2s_channel_disable(s_rx_handle);
    (void)i2s_del_channel(s_rx_handle);
    s_rx_handle = NULL;
    s_i2s_ready = false;
    ESP_LOGI(TAG, "I2S capture channel deinitialized");
}

static esp_err_t app_audio_i2s_recreate(void)
{
    app_audio_i2s_deinit();
    return app_audio_i2s_init();
}

static uint16_t app_compute_energy(const int16_t *samples, uint16_t count)
{
    uint32_t acc = 0;
    for (uint16_t i = 0; i < count; ++i) {
        int32_t v = samples[i];
        if (v < 0) {
            v = -v;
        }
        acc += (uint32_t)v;
    }
    return (uint16_t)(acc / count);
}

static void app_capture_task(void *arg)
{
    (void)arg;
    memset(s_i2s_frame, 0, sizeof(s_i2s_frame));
    memset(s_pcm_frame, 0, sizeof(s_pcm_frame));
    memset(s_adpcm_frame, 0, sizeof(s_adpcm_frame));

    app_adpcm_state_t adpcm_state = {
        .predictor = 0,
        .index = 0,
    };

    uint16_t seq = 0;
    uint16_t noise_floor = 0;
    uint16_t vad_threshold = 300;
    uint32_t speech_ms = 0;
    bool speech_started = false;
    app_audio_capture_stats_t stats;
    memset(&stats, 0, sizeof(stats));
    stats.source_sample_rate_hz = s_capture_sample_rate_hz;
    app_capture_stop_reason_t stop_reason = APP_STOP_REASON_SILENCE;
    int64_t capture_start_us = esp_timer_get_time();
    int64_t last_speech_us = capture_start_us;
    int64_t next_level_log_us = capture_start_us;
    uint32_t consecutive_timeouts = 0;
    bool recovery_slot_toggled = false;
    bool first_frame_logged = false;
    uint32_t frames_since_yield = 0;
    int64_t prev_frame_us = 0;
    uint64_t frame_interval_total_us = 0;
    uint32_t frame_interval_count = 0;
    uint32_t frame_interval_max_ms = 0;
    uint32_t frame_timing_slip_ms = 0;

    while (true) {
        if (s_stop_requested) {
            stop_reason = s_requested_stop_reason;
            break;
        }

        size_t frame_bytes = 0;
        size_t target_frame_bytes = (size_t)s_pcm_samples_per_frame * sizeof(int32_t);
        esp_err_t frame_err = ESP_OK;
        while (frame_bytes < target_frame_bytes) {
            size_t bytes_read = 0;
            frame_err = i2s_channel_read(
                s_rx_handle,
                ((uint8_t *)s_i2s_frame) + frame_bytes,
                target_frame_bytes - frame_bytes,
                &bytes_read,
                pdMS_TO_TICKS(APP_I2S_READ_TIMEOUT_MS));

            if (s_stop_requested) {
                stop_reason = s_requested_stop_reason;
                break;
            }

            if (frame_err == ESP_OK && bytes_read > 0) {
                frame_bytes += bytes_read;
                consecutive_timeouts = 0;
                continue;
            }

            if (frame_err == ESP_ERR_TIMEOUT || bytes_read == 0) {
                stats.i2s_read_timeouts++;
                consecutive_timeouts++;
                if ((consecutive_timeouts % 100) == 0) {
                    ESP_LOGW(TAG,
                             "I2S RX starving: consecutive_timeouts=%lu session=%u slot=%s",
                             (unsigned long)consecutive_timeouts,
                             (unsigned)s_session_id,
                             app_audio_slot_name(s_use_right_slot));
                }

                if (!recovery_slot_toggled &&
                    stats.frames_generated == 0 &&
                    frame_bytes == 0 &&
                    consecutive_timeouts >= APP_I2S_RECOVERY_TIMEOUTS) {
                    bool next_slot_right = !s_use_right_slot;
                    ESP_LOGW(TAG,
                             "Attempting I2S RX recovery by slot toggle: %s -> %s (session=%u)",
                             app_audio_slot_name(s_use_right_slot),
                             app_audio_slot_name(next_slot_right),
                             (unsigned)s_session_id);
                    esp_err_t sw = app_audio_reconfig_slot(next_slot_right);
                    ESP_LOGW(TAG,
                             "I2S RX recovery result: %s",
                             esp_err_to_name(sw));
                    recovery_slot_toggled = true;
                    consecutive_timeouts = 0;
                }
            } else {
                stats.i2s_read_errors++;
                ESP_LOGW(TAG,
                         "I2S RX read error: err=%s session=%u",
                         esp_err_to_name(frame_err),
                         (unsigned)s_session_id);
                consecutive_timeouts = 0;
            }

            if (s_stop_requested) {
                stop_reason = s_requested_stop_reason;
                break;
            }

            /* Keep waiting until a full frame is assembled, but avoid busy spin when starving. */
            vTaskDelay(1);
        }

        if (s_stop_requested) {
            stop_reason = s_requested_stop_reason;
            break;
        }

        if (frame_err != ESP_OK || frame_bytes < target_frame_bytes) {
            continue;
        }

        if (!first_frame_logged) {
            first_frame_logged = true;
            ESP_LOGI(TAG,
                     "I2S RX first frame session=%u bytes=%u slot=%s",
                     (unsigned)s_session_id,
                     (unsigned)frame_bytes,
                     app_audio_slot_name(s_use_right_slot));
        }

        for (uint16_t i = 0; i < s_pcm_samples_per_frame; ++i) {
            s_pcm_frame[i] = (int16_t)(s_i2s_frame[i] >> 14);
        }

        uint16_t energy = app_compute_energy(s_pcm_frame, s_pcm_samples_per_frame);
        if (s_callbacks.on_level) {
            s_callbacks.on_level(energy);
        }
        stats.level_updates++;

        int64_t now_us = esp_timer_get_time();
        uint32_t elapsed_ms = (uint32_t)((now_us - capture_start_us) / 1000);

        if (elapsed_ms < APP_VAD_CALIBRATION_MS) {
            noise_floor = (uint16_t)((noise_floor + energy) / 2);
            vad_threshold = (uint16_t)(noise_floor * 2 + 80);
            if (vad_threshold < 220) {
                vad_threshold = 220;
            }
        }

        if (energy > vad_threshold) {
            speech_started = true;
            speech_ms += APP_AUDIO_FRAME_MS;
            last_speech_us = now_us;
        }

        if (now_us >= next_level_log_us) {
            ESP_LOGI(TAG,
                     "MIC level=%u threshold=%u session=%u elapsed_ms=%lu speech=%d",
                     (unsigned)energy,
                     (unsigned)vad_threshold,
                     (unsigned)s_session_id,
                     (unsigned long)elapsed_ms,
                     speech_started ? 1 : 0);
            next_level_log_us = now_us + ((int64_t)APP_MIC_LEVEL_LOG_INTERVAL_MS * 1000LL);
        }

        uint16_t adpcm_len = app_adpcm_encode_frame(
            &adpcm_state,
            s_pcm_frame,
            s_pcm_samples_per_frame,
            s_adpcm_frame);

        if (s_callbacks.on_packet) {
            uint8_t flags = (seq == 0) ? APP_PKT_FLAG_START : 0;
            s_callbacks.on_packet(s_session_id, seq, elapsed_ms, flags, s_adpcm_frame, adpcm_len);
        }
        stats.frames_generated++;

        if (prev_frame_us > 0) {
            uint32_t interval_ms = (uint32_t)((now_us - prev_frame_us) / 1000);
            frame_interval_total_us += (uint64_t)(now_us - prev_frame_us);
            frame_interval_count++;
            if (interval_ms > frame_interval_max_ms) {
                frame_interval_max_ms = interval_ms;
            }
            if (interval_ms > (APP_AUDIO_FRAME_MS + 2)) {
                frame_timing_slip_ms += (interval_ms - APP_AUDIO_FRAME_MS);
                if ((frame_timing_slip_ms % 200) < 20) {
                    ESP_LOGW(TAG,
                             "Capture timing slip session=%u seq=%u interval_ms=%lu cum_slip_ms=%lu",
                             (unsigned)s_session_id,
                             (unsigned)seq,
                             (unsigned long)interval_ms,
                             (unsigned long)frame_timing_slip_ms);
                }
            }
        }
        prev_frame_us = now_us;

        seq++;
        frames_since_yield++;
        if (frames_since_yield >= 5) {
            frames_since_yield = 0;
            taskYIELD();
        }

        if (s_stop_requested) {
            stop_reason = s_requested_stop_reason;
            break;
        }

        if (!speech_started && elapsed_ms >= APP_VAD_NO_SPEECH_TIMEOUT_MS) {
            stop_reason = APP_STOP_REASON_NO_SPEECH;
            break;
        }

        uint32_t since_last_speech_ms = (uint32_t)((now_us - last_speech_us) / 1000);
        if (speech_started && speech_ms >= APP_VAD_MIN_SPEECH_MS && since_last_speech_ms >= APP_VAD_SILENCE_HANGOVER_MS) {
            stop_reason = APP_STOP_REASON_SILENCE;
            break;
        }

        if (elapsed_ms >= APP_CAPTURE_MAX_MS) {
            stop_reason = APP_STOP_REASON_MAX_LEN;
            break;
        }
    }

    uint32_t capture_elapsed_ms = (uint32_t)((esp_timer_get_time() - capture_start_us) / 1000);
    if (s_callbacks.on_packet) {
        s_callbacks.on_packet(s_session_id, seq, capture_elapsed_ms, APP_PKT_FLAG_END, NULL, 0);
    }

    stats.frame_interval_max_ms = frame_interval_max_ms;
    if (frame_interval_count > 0) {
        stats.frame_interval_avg_ms = (uint32_t)((frame_interval_total_us / frame_interval_count) / 1000ULL);
    }
    stats.frame_timing_slip_ms = frame_timing_slip_ms;
    stats.speech_detected = speech_started;
    ESP_LOGI(TAG,
             "Capture stop session=%u reason=%u frames=%lu levels=%lu i2s_timeouts=%lu i2s_errors=%lu frame_avg_ms=%lu frame_max_ms=%lu slip_ms=%lu speech=%d",
             s_session_id,
             (unsigned)stop_reason,
             (unsigned long)stats.frames_generated,
             (unsigned long)stats.level_updates,
             (unsigned long)stats.i2s_read_timeouts,
             (unsigned long)stats.i2s_read_errors,
             (unsigned long)stats.frame_interval_avg_ms,
             (unsigned long)stats.frame_interval_max_ms,
             (unsigned long)stats.frame_timing_slip_ms,
             stats.speech_detected ? 1 : 0);

    if (s_callbacks.on_stopped) {
        s_callbacks.on_stopped(s_session_id, stop_reason, &stats);
    }

    app_audio_i2s_deinit();

    s_running = false;
    s_stop_requested = false;
    s_capture_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t app_audio_capture_init(const app_audio_capture_callbacks_t *callbacks)
{
    memset(&s_callbacks, 0, sizeof(s_callbacks));
    if (callbacks) {
        s_callbacks = *callbacks;
    }
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Mic level monitor log enabled (%d ms interval)", APP_MIC_LEVEL_LOG_INTERVAL_MS);
    ESP_LOGI(TAG, "I2S capture will initialize lazily on capture start");
    return ESP_OK;
}

bool app_audio_capture_start(uint16_t session_id, uint8_t codec, uint32_t sample_rate_hz)
{
    if (s_running) {
        return false;
    }

    uint16_t frame_samples = (uint16_t)((sample_rate_hz * APP_AUDIO_FRAME_MS) / 1000);
    if (frame_samples == 0 ||
        frame_samples > APP_AUDIO_MAX_PCM_SAMPLES_PER_FRAME ||
        ((frame_samples % 2) != 0)) {
        ESP_LOGW(TAG,
                 "Invalid capture profile: codec=%u sample_rate=%lu frame_samples=%u",
                 (unsigned)codec,
                 (unsigned long)sample_rate_hz,
                 (unsigned)frame_samples);
        return false;
    }

    s_capture_codec = codec;
    s_capture_sample_rate_hz = sample_rate_hz;
    s_pcm_samples_per_frame = frame_samples;

    if (app_audio_i2s_recreate() != ESP_OK) {
        return false;
    }

    app_audio_drop_stale_rx_samples();

    s_session_id = session_id;
    s_stop_requested = false;
    s_running = true;
    ESP_LOGI(TAG,
             "Capture start session=%u codec=%u sample_rate=%lu frame_samples=%u",
             s_session_id,
             (unsigned)s_capture_codec,
             (unsigned long)s_capture_sample_rate_hz,
             (unsigned)s_pcm_samples_per_frame);

    BaseType_t ok = xTaskCreatePinnedToCore(
        app_capture_task,
        "app_capture",
        APP_CAPTURE_TASK_STACK_SIZE,
        NULL,
        APP_CAPTURE_TASK_PRIORITY,
        &s_capture_task,
        APP_CAPTURE_TASK_CORE);

    if (ok != pdPASS) {
        s_running = false;
        s_capture_task = NULL;
        return false;
    }

    return true;
}

void app_audio_capture_request_stop(app_capture_stop_reason_t reason)
{
    if (!s_running) {
        return;
    }
    s_requested_stop_reason = reason;
    s_stop_requested = true;
}

bool app_audio_capture_is_running(void)
{
    return s_running;
}
