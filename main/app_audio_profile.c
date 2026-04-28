#include "app_audio_profile.h"

#include <stdbool.h>

#include "esp_log.h"

#define APP_AUDIO_PROFILE_RECOVERY_SESSIONS 3

static const char *TAG = "APP_AUDIO_PROFILE";

static const app_audio_profile_t s_profiles[] = {
    {.codec = APP_AUDIO_CODEC_IMA_ADPCM_16K, .sample_rate_hz = APP_AUDIO_SAMPLE_RATE_16K, .label = "16k"},
    {.codec = APP_AUDIO_CODEC_IMA_ADPCM_12K, .sample_rate_hz = APP_AUDIO_SAMPLE_RATE_12K, .label = "12k"},
    {.codec = APP_AUDIO_CODEC_IMA_ADPCM_8K, .sample_rate_hz = APP_AUDIO_SAMPLE_RATE_8K, .label = "8k"},
};

static uint8_t s_profile_index;
static uint8_t s_healthy_session_streak;

void app_audio_profile_reset(void)
{
    s_profile_index = 0;
    s_healthy_session_streak = 0;
}

const app_audio_profile_t *app_audio_profile_active(void)
{
    return &s_profiles[s_profile_index];
}

void app_audio_profile_note_session_quality(app_capture_stop_reason_t reason)
{
    bool changed = false;
    if (reason == APP_STOP_REASON_LINK_SLOW) {
        s_healthy_session_streak = 0;
        if ((s_profile_index + 1U) < (sizeof(s_profiles) / sizeof(s_profiles[0]))) {
            s_profile_index++;
            changed = true;
        }
    } else if (reason == APP_STOP_REASON_SILENCE || reason == APP_STOP_REASON_MANUAL || reason == APP_STOP_REASON_MAX_LEN) {
        if (s_profile_index > 0) {
            s_healthy_session_streak++;
            if (s_healthy_session_streak >= APP_AUDIO_PROFILE_RECOVERY_SESSIONS) {
                s_profile_index--;
                s_healthy_session_streak = 0;
                changed = true;
            }
        }
    } else {
        s_healthy_session_streak = 0;
    }

    if (changed) {
        const app_audio_profile_t *profile = app_audio_profile_active();
        ESP_LOGI(TAG,
                 "Audio profile adjusted for next session: codec=0x%02X sample_rate=%lu profile=%s",
                 profile->codec,
                 (unsigned long)profile->sample_rate_hz,
                 profile->label);
    }
}
