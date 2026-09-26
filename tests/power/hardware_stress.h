/* Optional board test; never include in release firmware. Generate the Opus
 * fixture first. Call power_stress_start() after app_state_init() and
 * power_stress_poll() in the main loop. The packet-only feeder
 * has its own task, like real BLE input, so a display wait cannot underfeed it.
 * Runs real audio drivers/decoders while rendering and the BLE link remain live.
 * Capture packets are counted locally, not forwarded to the phone/backend.
 * Real phone-to-board audio transport still needs an end-to-end voice test. */
#pragma once
#include "tone_fixture.h"
#include "app_audio_capture.h"
#include "app_audio_downlink.h"
#include "app_ble_link.h"
#include "app_wakeword.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include <assert.h>
#include <stdatomic.h>

static atomic_uint phase, rejected;
static void power_stress_feed_poll(void);
static void power_stress_task(void *arg)
{
    (void)arg;
    while (esp_timer_get_time() < 100000000) {
        power_stress_feed_poll();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    vTaskDeleteWithCaps(NULL);
}

static void power_stress_start(void)
{
    BaseType_t created = xTaskCreatePinnedToCoreWithCaps(power_stress_task,
        "audio_test_feed", 4096, NULL, 3, NULL, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(created == pdPASS);
}

static unsigned power_capture_packets;
static void power_capture_packet(uint16_t sid, uint16_t seq, uint32_t elapsed,
                                 uint8_t flags, const uint8_t *data, uint16_t len)
{
    (void)sid; (void)seq; (void)elapsed; (void)flags; (void)data; (void)len;
    ++power_capture_packets;
}
static void power_capture_stopped(uint16_t sid, app_capture_stop_reason_t reason,
                                  const app_audio_capture_stats_t *stats)
{
    ESP_LOGI("POWER_STRESS", "capture sid=%u reason=%u packets=%u errors=%lu timeouts=%lu avg_ms=%lu max_ms=%lu",
             sid, reason, power_capture_packets, (unsigned long)stats->i2s_read_errors,
             (unsigned long)stats->i2s_read_timeouts, (unsigned long)stats->frame_interval_avg_ms,
             (unsigned long)stats->frame_interval_max_ms);
}

static void power_stress_feed_poll(void)
{
    static unsigned previous_phase, seq;
    static int64_t next_packet;
    int64_t now = esp_timer_get_time();
    unsigned current_phase = atomic_load(&phase);
    if (current_phase != 1 && current_phase != 3 && current_phase != 5) return;
    if (current_phase != previous_phase) {
        previous_phase = current_phase;
        seq = 0;
        next_packet = now;
    }
    if (seq < 400 && now >= next_packet) {
        static const uint8_t silence[160] = {0};
        bool opus = current_phase == 5;
        const uint8_t *payload = opus ? power_opus_fixture[seq % (sizeof(power_opus_fixture) / sizeof(power_opus_fixture[0]))] : silence;
        uint8_t flags = (seq == 0 ? 1 : 0) | (seq == 399 ? 2 : 0) | (opus ? 12 : 0);
        if (app_audio_downlink_enqueue((uint16_t)(0xf000 + current_phase), seq, flags,
                opus ? APP_AUDIO_CODEC_OPUS_24K : APP_AUDIO_CODEC_IMA_ADPCM_16K,
                opus ? 480 : 320, 0, opus ? 20 : 0, payload,
                opus ? sizeof(power_opus_fixture[0]) : sizeof(silence))) {
            ++seq;
            next_packet += 20000;
        } else {
            ++rejected;
            ESP_LOGE("POWER_STRESS", "Enqueue rejected phase=%u seq=%u", current_phase, seq);
        }
    }
}

static void power_stress_poll(void)
{
    static unsigned ble_missing;
    static int64_t start;
    static uint16_t capture_sid = 0xf100;
    int64_t now = esp_timer_get_time();
    unsigned seconds = (unsigned)(now / 1000000);
    if (seconds < 30 || phase == 6) return;
    if (!app_ble_link_is_connected()) ++ble_missing;
    if (phase == 0) {
        if (!app_wakeword_is_running()) {
            ESP_LOGE("POWER_STRESS", "WakeNet must be running before the test"); phase = 6; return;
        }
        ESP_LOGI("POWER_STRESS", "Begin ADPCM playback with WakeNet, rendering and BLE");
        phase = 1; start = now;
    }
    if (phase == 1 && now - start > 12000000) {
        ESP_LOGI("POWER_STRESS", "Switch microphone from WakeNet to capture");
        if (!app_wakeword_stop(1200)) {
            ESP_LOGE("POWER_STRESS", "Wake stop failed"); phase = 6; return;
        }
        const app_audio_capture_callbacks_t callbacks = {
            .on_packet = power_capture_packet, .on_stopped = power_capture_stopped,
        };
        ESP_ERROR_CHECK(app_audio_capture_init(&callbacks));
        phase = 2; start = now;
    }
    if (phase == 2 && now - start > 1000000) {
        ESP_LOGI("POWER_STRESS", "Begin simultaneous microphone capture and ADPCM playback");
        phase = 3; start = now;
    }
    if (phase == 3) {
        if (now - start < 8000000 && !app_audio_capture_is_running()) {
            power_capture_packets = 0;
            if (!app_audio_capture_start(++capture_sid, APP_AUDIO_CODEC_IMA_ADPCM_16K, 16000))
                ESP_LOGE("POWER_STRESS", "Capture start failed");
        }
        if (now - start > 12000000) {
            app_audio_capture_request_stop(APP_STOP_REASON_MANUAL);
            phase = 4; start = now;
        }
    }
    if (phase == 4 && now - start > 1000000 && !app_audio_capture_is_running()) {
        if (!app_wakeword_stop(1200)) {
            ESP_LOGE("POWER_STRESS", "Wake stop before Opus failed"); phase = 6; return;
        }
        ESP_LOGI("POWER_STRESS", "Begin Opus playback with rendering and BLE");
        phase = 5; start = now;
    }
    if (phase == 5 && now - start > 12000000) {
        ESP_LOGI("POWER_STRESS", "Finished rejected=%u ble_missing_polls=%u; restoring WakeNet", rejected, ble_missing);
        app_audio_downlink_release_transient_resources();
        esp_err_t wake_err = app_wakeword_start();
        if (wake_err != ESP_OK)
            ESP_LOGE("POWER_STRESS", "Wake restore failed: %s", esp_err_to_name(wake_err));
        phase = 6;
    }
}
