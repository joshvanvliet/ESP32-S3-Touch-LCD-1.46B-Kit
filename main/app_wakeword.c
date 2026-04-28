#include "app_wakeword.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_APP_WAKEWORD_ENABLED
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_afe_sr_models.h"
#include "esp_wn_models.h"
#include "model_path.h"

#define WAKE_I2S_PORT I2S_NUM_1
#define WAKE_I2S_READ_TIMEOUT_MS 120
#define WAKE_TASK_STACK_FEED 4096
#define WAKE_TASK_STACK_DETECT 4096
#define WAKE_TASK_PRIO 5
#define WAKE_TASK_CORE 0
#define WAKE_FEED_STOP_WAIT_MS 1200
#define WAKE_I2S_USE_RIGHT_SLOT 1

static const char *TAG = "APP_WAKE";

static app_wakeword_callbacks_t s_callbacks;
static const esp_afe_sr_iface_t *s_afe_handle;
static esp_afe_sr_data_t *s_afe_data;
static srmodel_list_t *s_models;
static i2s_chan_handle_t s_rx_handle;
static TaskHandle_t s_feed_task;
static TaskHandle_t s_detect_task;
static volatile bool s_running;
static volatile bool s_stop_requested;
static volatile bool s_detected;

static esp_err_t wake_i2s_init(void)
{
    if (s_rx_handle) {
        return ESP_OK;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(WAKE_I2S_PORT, I2S_ROLE_MASTER);
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &s_rx_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
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
#if WAKE_I2S_USE_RIGHT_SLOT
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
#else
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
#endif

    ret = i2s_channel_init_std_mode(s_rx_handle, &std_cfg);
    if (ret == ESP_OK) {
        ret = i2s_channel_enable(s_rx_handle);
    }
    if (ret != ESP_OK) {
        (void)i2s_del_channel(s_rx_handle);
        s_rx_handle = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "WakeNet I2S started slot=%s", WAKE_I2S_USE_RIGHT_SLOT ? "RIGHT" : "LEFT");
    return ESP_OK;
}

static void wake_i2s_deinit(void)
{
    if (!s_rx_handle) {
        return;
    }
    (void)i2s_channel_disable(s_rx_handle);
    (void)i2s_del_channel(s_rx_handle);
    s_rx_handle = NULL;
}

static void wake_cleanup_resources(void)
{
    wake_i2s_deinit();
    if (s_afe_handle && s_afe_data) {
        s_afe_handle->destroy(s_afe_data);
        s_afe_data = NULL;
    }
    if (s_models) {
        esp_srmodel_deinit(s_models);
        s_models = NULL;
    }
    s_afe_handle = NULL;
}

static void wake_feed_task(void *arg)
{
    (void)arg;
    int audio_chunksize = s_afe_handle->get_feed_chunksize(s_afe_data);
    int32_t *i2s_buff = (int32_t *)calloc((size_t)audio_chunksize, sizeof(int32_t));
    if (!i2s_buff) {
        ESP_LOGE(TAG, "Wake feed allocation failed");
        s_stop_requested = true;
        s_feed_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (!s_stop_requested) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(
            s_rx_handle,
            i2s_buff,
            (size_t)audio_chunksize * sizeof(int32_t),
            &bytes_read,
            pdMS_TO_TICKS(WAKE_I2S_READ_TIMEOUT_MS));
        if (err != ESP_OK || bytes_read == 0) {
            continue;
        }

        int samples = (int)(bytes_read / sizeof(int32_t));
        for (int i = 0; i < samples; ++i) {
            i2s_buff[i] = i2s_buff[i] >> 14;
        }
        s_afe_handle->feed(s_afe_data, (int16_t *)i2s_buff);
    }

    free(i2s_buff);
    s_feed_task = NULL;
    vTaskDelete(NULL);
}

static void wake_detect_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "WakeNet listening");

    while (!s_stop_requested) {
        afe_fetch_result_t *res = s_afe_handle->fetch(s_afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            ESP_LOGW(TAG, "WakeNet fetch failed");
            break;
        }
        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, "Wake word detected");
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            ESP_LOGI(TAG, "Wake word channel verified");
            s_detected = true;
            s_stop_requested = true;
            break;
        }
    }

    uint32_t waited_ms = 0;
    while (s_feed_task && waited_ms < WAKE_FEED_STOP_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(20));
        waited_ms += 20;
    }
    if (s_feed_task) {
        ESP_LOGW(TAG, "Wake feed task did not stop within %lu ms", (unsigned long)waited_ms);
    }

    bool detected = s_detected;
    wake_cleanup_resources();
    s_running = false;
    s_stop_requested = false;
    s_detected = false;
    s_detect_task = NULL;

    if (detected && s_callbacks.on_detected) {
        s_callbacks.on_detected();
    }
    vTaskDelete(NULL);
}
#endif

esp_err_t app_wakeword_init(const app_wakeword_callbacks_t *callbacks)
{
#if CONFIG_APP_WAKEWORD_ENABLED
    memset(&s_callbacks, 0, sizeof(s_callbacks));
    if (callbacks) {
        s_callbacks = *callbacks;
    }
    return ESP_OK;
#else
    (void)callbacks;
    return ESP_OK;
#endif
}

esp_err_t app_wakeword_start(void)
{
#if CONFIG_APP_WAKEWORD_ENABLED
    if (s_running) {
        return ESP_OK;
    }

    s_afe_handle = &ESP_AFE_SR_HANDLE;
    s_models = esp_srmodel_init("model");
    if (!s_models) {
        ESP_LOGW(TAG, "WakeNet model init failed");
        return ESP_FAIL;
    }

    afe_config_t afe_config = {
        .aec_init = false,
        .se_init = false,
        .vad_init = false,
        .wakenet_init = true,
        .voice_communication_init = false,
        .voice_communication_agc_init = false,
        .voice_communication_agc_gain = 15,
        .vad_mode = VAD_MODE_3,
        .wakenet_model_name = esp_srmodel_filter(s_models, ESP_WN_PREFIX, NULL),
        .wakenet_model_name_2 = NULL,
        .wakenet_mode = DET_MODE_2CH_90,
        .afe_mode = SR_MODE_LOW_COST,
        .afe_perferred_core = 0,
        .afe_perferred_priority = 5,
        .afe_ringbuf_size = 10,
        .memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM,
        .afe_linear_gain = 1.0,
        .agc_mode = AFE_MN_PEAK_AGC_MODE_2,
        .pcm_config = {
            .total_ch_num = 2,
            .mic_num = 1,
            .ref_num = 1,
            .sample_rate = 16000,
        },
        .debug_init = false,
        .debug_hook = {{AFE_DEBUG_HOOK_MASE_TASK_IN, NULL}, {AFE_DEBUG_HOOK_FETCH_TASK_IN, NULL}},
    };
    s_afe_data = s_afe_handle->create_from_config(&afe_config);
    if (!s_afe_data) {
        ESP_LOGW(TAG, "WakeNet AFE create failed");
        wake_cleanup_resources();
        return ESP_FAIL;
    }
    esp_err_t ret = wake_i2s_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WakeNet I2S init failed: %s", esp_err_to_name(ret));
        wake_cleanup_resources();
        return ret;
    }

    s_stop_requested = false;
    s_detected = false;
    s_running = true;
    BaseType_t feed_ok = xTaskCreatePinnedToCore(
        wake_feed_task,
        "app_wake_feed",
        WAKE_TASK_STACK_FEED,
        NULL,
        WAKE_TASK_PRIO,
        &s_feed_task,
        WAKE_TASK_CORE);
    BaseType_t detect_ok = xTaskCreatePinnedToCore(
        wake_detect_task,
        "app_wake_detect",
        WAKE_TASK_STACK_DETECT,
        NULL,
        WAKE_TASK_PRIO,
        &s_detect_task,
        WAKE_TASK_CORE);
    if (feed_ok != pdPASS || detect_ok != pdPASS) {
        ESP_LOGW(TAG, "WakeNet task create failed");
        s_stop_requested = true;
        s_running = false;
        wake_cleanup_resources();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

bool app_wakeword_stop(uint32_t timeout_ms)
{
#if CONFIG_APP_WAKEWORD_ENABLED
    if (!s_running) {
        return true;
    }
    s_stop_requested = true;
    uint32_t waited_ms = 0;
    while (s_running && waited_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(20));
        waited_ms += 20;
    }
    return !s_running;
#else
    (void)timeout_ms;
    return true;
#endif
}

bool app_wakeword_is_running(void)
{
#if CONFIG_APP_WAKEWORD_ENABLED
    return s_running;
#else
    return false;
#endif
}
