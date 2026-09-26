/* Temporary diagnostic: include from main.c and call power_profile_poll() in
 * its service loop. Enable FreeRTOS runtime stats (ESP_TIMER) and face profile.
 * No extra task, stack, or periodic timer is allocated. Remove for release. */
#pragma once
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "freertos/task.h"
#include "app_ble_link.h"
#include "app_wakeword.h"
#include "app_audio_capture.h"
#include "app_audio_downlink.h"

static void power_profile_poll(void)
{
    enum { CAPACITY = 64 };
    static TaskStatus_t *previous, *current;
    static UBaseType_t previous_count;
    static int64_t previous_us;
    int64_t now = esp_timer_get_time();
    if (now - previous_us < 5000000) return;
    if (!previous) {
        previous = heap_caps_calloc(CAPACITY, sizeof(*previous), MALLOC_CAP_SPIRAM);
        current = heap_caps_calloc(CAPACITY, sizeof(*current), MALLOC_CAP_SPIRAM);
        assert(previous && current);
    }
    configRUN_TIME_COUNTER_TYPE total;
    UBaseType_t count = uxTaskGetSystemState(current, CAPACITY, &total);
    if (!count) { ESP_LOGE("POWER_TEST", "Task snapshot exceeded capacity"); return; }
    if (previous_count) {
        ESP_LOGI("POWER_TEST", "window_us=%lld ble=%d wake=%d capture=%d playback=%d",
                 now - previous_us, app_ble_link_is_connected(), app_wakeword_is_running(),
                 app_audio_capture_is_running(), app_audio_downlink_active());
        for (UBaseType_t i = 0; i < count; ++i) {
            for (UBaseType_t j = 0; j < previous_count; ++j) {
                if (current[i].xTaskNumber != previous[j].xTaskNumber) continue;
                uint32_t delta = current[i].ulRunTimeCounter - previous[j].ulRunTimeCounter;
                ESP_LOGI("POWER_TEST", "task=%s run_us=%lu", current[i].pcTaskName, (unsigned long)delta);
                break;
            }
        }
#if CONFIG_PM_PROFILING
        esp_pm_dump_locks(stdout);
#endif
    }
    TaskStatus_t *swap = previous; previous = current; current = swap;
    previous_count = count;
    previous_us = now;
}
