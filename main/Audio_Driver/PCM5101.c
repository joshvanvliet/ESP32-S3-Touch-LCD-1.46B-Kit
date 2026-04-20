#include "PCM5101.h"
#include <math.h>

static const char *TAG = "AUDIO PCM5101"; 

static i2s_chan_handle_t i2s_tx_chan; 
static i2s_chan_handle_t i2s_rx_chan; 
static uint32_t s_pcm_tx_rate_hz;
static bool s_pcm_tx_stereo_mode;
static SemaphoreHandle_t s_tx_mutex;

#define AUDIO_TX_LOCK_TIMEOUT_TICKS pdMS_TO_TICKS(450)
#define AUDIO_TX_WRITE_TIMEOUT_TICKS pdMS_TO_TICKS(320)
#define AUDIO_TX_WRITE_PROGRESS_RETRY_LIMIT 16
#define AUDIO_PCM_MONO_CHUNK 128

static int16_t s_pcm_stereo_buf[AUDIO_PCM_MONO_CHUNK * 2];

uint8_t Volume = Volume_MAX - 2;
bool Music_Next_Flag = 0;
static bool audio_tx_lock(void)
{
    if (s_tx_mutex == NULL) {
        return false;
    }
    return xSemaphoreTake(s_tx_mutex, AUDIO_TX_LOCK_TIMEOUT_TICKS) == pdTRUE;
}

static void audio_tx_unlock(void)
{
    if (s_tx_mutex) {
        xSemaphoreGive(s_tx_mutex);
    }
}

static void audio_tx_update_format(uint32_t rate, i2s_slot_mode_t ch)
{
    s_pcm_tx_rate_hz = rate;
    s_pcm_tx_stereo_mode = (ch == I2S_SLOT_MODE_STEREO);
}

static esp_err_t audio_tx_reconfig_locked(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)bits_cfg, ch),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };
    ret |= i2s_channel_disable(i2s_tx_chan);
    ret |= i2s_channel_reconfig_std_clock(i2s_tx_chan, &std_cfg.clk_cfg);
    ret |= i2s_channel_reconfig_std_slot(i2s_tx_chan, &std_cfg.slot_cfg);
    ret |= i2s_channel_enable(i2s_tx_chan);
    if (ret == ESP_OK) {
        audio_tx_update_format(rate, ch);
    }
    return ret;
}

static esp_err_t audio_tx_recover_locked(uint32_t rate)
{
    if (rate == 0) {
        rate = s_pcm_tx_rate_hz > 0 ? s_pcm_tx_rate_hz : 8000;
    }
    ESP_LOGW(TAG, "Audio TX recover start rate=%lu", (unsigned long)rate);
    esp_err_t ret = audio_tx_reconfig_locked(rate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Audio TX recover failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Audio TX recover ok rate=%lu", (unsigned long)rate);
    }
    return ret;
}

static esp_err_t audio_tx_write_all_locked(const int16_t *audio_buffer, size_t len, TickType_t timeout_ticks)
{
    const uint8_t *cursor = (const uint8_t *)audio_buffer;
    size_t remaining = len;
    int no_progress_retries = 0;

    while (remaining > 0) {
        size_t bytes_written = 0;
        esp_err_t write_ret = i2s_channel_write(
            i2s_tx_chan,
            (void *)cursor,
            remaining,
            &bytes_written,
            timeout_ticks);

        if (bytes_written > 0) {
            cursor += bytes_written;
            remaining -= bytes_written;
            no_progress_retries = 0;
            continue;
        }

        no_progress_retries++;
        if (write_ret != ESP_OK || no_progress_retries > AUDIO_TX_WRITE_PROGRESS_RETRY_LIMIT) {
            return (write_ret == ESP_OK) ? ESP_ERR_TIMEOUT : write_ret;
        }
        vTaskDelay(1);
    }
    return ESP_OK;
}

static esp_err_t bsp_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    if (!audio_buffer || i2s_tx_chan == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int16_t *samples = (int16_t *)audio_buffer;
    size_t sample_count = len / sizeof(int16_t);
    uint32_t volume = Volume;
    for (size_t i = 0; i < sample_count; i++) {
        samples[i] = (int16_t)(((int32_t)samples[i] * (int32_t)volume) / (int32_t)Volume_MAX);
    }

    if (!audio_tx_lock()) {
        return ESP_ERR_TIMEOUT;
    }
    // audio_player passes timeout in FreeRTOS ticks (despite the field name).
    esp_err_t ret = audio_tx_write_all_locked(samples, len, (TickType_t)timeout_ms);
    audio_tx_unlock();

    if (bytes_written) {
        *bytes_written = (ret == ESP_OK) ? len : 0;
    }
    return ret;
}

static esp_err_t bsp_i2s_reconfig_clk(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    if (!audio_tx_lock()) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t ret = audio_tx_reconfig_locked(rate, bits_cfg, ch);
    audio_tx_unlock();
    return ret;
}

static esp_err_t audio_mute_function(AUDIO_PLAYER_MUTE_SETTING setting) {                                                       // audio mute function
    ESP_LOGI(TAG, "mute setting %d", setting); 
    return ESP_OK; 
}

static esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel) {     // Audio Init
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; 
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, tx_channel, rx_channel)); 
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050); 
    const i2s_std_config_t *p_i2s_cfg = (i2s_config != NULL) ? i2s_config : &std_cfg_default; 
    if (tx_channel) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(*tx_channel, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(*tx_channel)); 
    }
    if (rx_channel) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(*rx_channel, p_i2s_cfg)); 
        ESP_ERROR_CHECK(i2s_channel_enable(*rx_channel));
    }
    return ESP_OK; 
}

static FILE * Music_File = NULL;
static audio_player_callback_event_t expected_event; 
static QueueHandle_t event_queue; 
static audio_player_callback_event_t event; 

static void audio_player_callback(audio_player_cb_ctx_t *ctx) {
    if (ctx->audio_event == AUDIO_PLAYER_CALLBACK_EVENT_IDLE) {
        ESP_LOGI(TAG, "Playback finished");
        Music_Next_Flag = 1;
    }
    if (ctx->audio_event == expected_event) {
        xQueueSend(event_queue, &(ctx->audio_event), 0);
    }
}

void Audio_Init(void) 
{
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };
    esp_err_t ret = bsp_audio_init(&std_cfg, &i2s_tx_chan, &i2s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(ret));
        return;
    }
    s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_tx_mutex) {
        ESP_LOGE(TAG, "Failed to create audio TX mutex");
        return;
    }
    audio_player_config_t config = { 
        .mute_fn = audio_mute_function,
        .write_fn = bsp_i2s_write,
        .clk_set_fn = bsp_i2s_reconfig_clk,
        .priority = 3,
        .coreID = 1 
    };
    ret = audio_player_new(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create audio player: %s", esp_err_to_name(ret));
        return;
    }
    event_queue = xQueueCreate(1, sizeof(audio_player_callback_event_t));
    if (!event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return;
    }
    ret = audio_player_callback_register(audio_player_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callback: %s", esp_err_to_name(ret));
        return;
    }
    if (audio_player_get_state() != AUDIO_PLAYER_STATE_IDLE) {
        ESP_LOGE(TAG, "Expected state to be IDLE");                 // The player is not idle
        return;
    }

    s_pcm_tx_rate_hz = 44100;
    s_pcm_tx_stereo_mode = true;
}

void Audio_Play_Test_Tone(uint16_t freq_hz, uint16_t duration_ms)
{
    if (i2s_tx_chan == NULL || freq_hz == 0 || duration_ms == 0) {
        ESP_LOGW(TAG, "Audio test tone skipped (I2S not ready or invalid args)");
        return;
    }

    const float sample_rate = 44100.0f;
    const float two_pi = 6.283185307f;
    const float step = two_pi * ((float)freq_hz / sample_rate);
    const int total_samples = (int)((sample_rate * duration_ms) / 1000.0f);
    const int chunk_samples = 256;
    int16_t *buffer = (int16_t *)malloc((size_t)chunk_samples * 2 * sizeof(int16_t));

    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate tone buffer");
        return;
    }

    if (!audio_tx_lock()) {
        ESP_LOGW(TAG, "Audio test tone skipped (audio lock timeout)");
        free(buffer);
        return;
    }

    if (s_pcm_tx_rate_hz != (uint32_t)sample_rate || !s_pcm_tx_stereo_mode) {
        esp_err_t cfg_ret = audio_tx_reconfig_locked((uint32_t)sample_rate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
        if (cfg_ret != ESP_OK) {
            ESP_LOGW(TAG, "Audio test tone reconfig failed: %s", esp_err_to_name(cfg_ret));
            audio_tx_unlock();
            free(buffer);
            return;
        }
    }

    float phase = 0.0f;
    int generated = 0;
    while (generated < total_samples) {
        int count = total_samples - generated;
        if (count > chunk_samples) {
            count = chunk_samples;
        }

        for (int i = 0; i < count; i++) {
            int16_t sample = (int16_t)(sinf(phase) * 2200.0f);
            buffer[(i * 2)] = sample;
            buffer[(i * 2) + 1] = sample;
            phase += step;
            if (phase > two_pi) {
                phase -= two_pi;
            }
        }

        size_t bytes_to_write = (size_t)count * 2 * sizeof(int16_t);
        esp_err_t write_ret = audio_tx_write_all_locked(buffer, bytes_to_write, AUDIO_TX_WRITE_TIMEOUT_TICKS);
        if (write_ret != ESP_OK) {
            ESP_LOGW(TAG, "Audio test tone write failed: %s", esp_err_to_name(write_ret));
            break;
        }
        generated += count;
    }

    audio_tx_unlock();
    free(buffer);
}

bool Audio_Play_PCM16_Mono(const int16_t *samples, size_t sample_count, uint32_t sample_rate_hz)
{
    if (samples == NULL || sample_count == 0 || i2s_tx_chan == NULL) {
        return false;
    }

    if (sample_rate_hz == 0) {
        sample_rate_hz = 8000;
    }

    if (!audio_tx_lock()) {
        ESP_LOGW(TAG, "Audio_Play_PCM16_Mono lock timeout");
        return false;
    }

    if (s_pcm_tx_rate_hz != sample_rate_hz || !s_pcm_tx_stereo_mode) {
        esp_err_t clk_ret = audio_tx_reconfig_locked(sample_rate_hz, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
        if (clk_ret != ESP_OK) {
            ESP_LOGW(TAG, "Audio_Play_PCM16_Mono reconfig failed: %s", esp_err_to_name(clk_ret));
            audio_tx_unlock();
            return false;
        }
    }

    bool ok = true;

    for (size_t offset = 0; offset < sample_count; offset += AUDIO_PCM_MONO_CHUNK) {
        size_t chunk_samples = sample_count - offset;
        if (chunk_samples > AUDIO_PCM_MONO_CHUNK) {
            chunk_samples = AUDIO_PCM_MONO_CHUNK;
        }
        for (size_t i = 0; i < chunk_samples; ++i) {
            int16_t s = samples[offset + i];
            s_pcm_stereo_buf[(i * 2)] = s;
            s_pcm_stereo_buf[(i * 2) + 1] = s;
        }

        size_t bytes_to_write = chunk_samples * 2 * sizeof(int16_t);
        esp_err_t write_ret = audio_tx_write_all_locked(s_pcm_stereo_buf, bytes_to_write, AUDIO_TX_WRITE_TIMEOUT_TICKS);
        if (write_ret != ESP_OK) {
            ESP_LOGW(TAG,
                     "Audio_Play_PCM16_Mono write failed: %s rate=%lu chunk_samples=%lu remaining_samples=%lu",
                     esp_err_to_name(write_ret),
                     (unsigned long)sample_rate_hz,
                     (unsigned long)chunk_samples,
                     (unsigned long)(sample_count - offset));
            esp_err_t recover_ret = audio_tx_recover_locked(sample_rate_hz);
            if (recover_ret != ESP_OK) {
                ok = false;
                break;
            }
            write_ret = audio_tx_write_all_locked(s_pcm_stereo_buf, bytes_to_write, AUDIO_TX_WRITE_TIMEOUT_TICKS);
            if (write_ret != ESP_OK) {
                ESP_LOGW(TAG,
                         "Audio_Play_PCM16_Mono retry write failed: %s rate=%lu chunk_samples=%lu remaining_samples=%lu",
                         esp_err_to_name(write_ret),
                         (unsigned long)sample_rate_hz,
                         (unsigned long)chunk_samples,
                         (unsigned long)(sample_count - offset));
                ok = false;
                break;
            }
        }
    }
    audio_tx_unlock();
    return ok;
}

void Play_Music(const char* directory, const char* fileName)
{  
    Music_pause();
    const int maxPathLength = 100; 
    char filePath[maxPathLength];
    if (strcmp(directory, "/") == 0) {                                               
        snprintf(filePath, maxPathLength, "%s%s", directory, fileName);   
    } else {                                                            
        snprintf(filePath, maxPathLength, "%s/%s", directory, fileName);
    }
    Music_File = Open_File(filePath);
    if (!Music_File) {
        ESP_LOGE(TAG, "Failed to open MP3 file: %s", filePath);
        return;
    }

    expected_event = AUDIO_PLAYER_CALLBACK_EVENT_PLAYING;
    esp_err_t ret = audio_player_play(Music_File);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to play audio: %s", esp_err_to_name(ret));
        fclose(Music_File);
        return;
    }
    if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to receive playing event");
        fclose(Music_File);
        return;
    }
    if (audio_player_get_state() != AUDIO_PLAYER_STATE_PLAYING) {
        ESP_LOGE(TAG, "Expected state to be PLAYING");
        fclose(Music_File);
        return;
    }
}
void Music_resume(void)
{
    if (audio_player_get_state() != AUDIO_PLAYER_STATE_PLAYING){
        expected_event = AUDIO_PLAYER_CALLBACK_EVENT_PLAYING;
        esp_err_t ret = audio_player_resume();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to resume audio: %s", esp_err_to_name(ret));
            fclose(Music_File);
            return;
        }
        if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to receive playing event after resume");
            fclose(Music_File);
            return;
        }
        if (audio_player_get_state() != AUDIO_PLAYER_STATE_PLAYING) {
            ESP_LOGE(TAG, "Expected state to be RESUME");
            fclose(Music_File);
            return;
        }
    }
}
void Music_pause(void) 
{
    if (audio_player_get_state() == AUDIO_PLAYER_STATE_PLAYING){
        expected_event = AUDIO_PLAYER_CALLBACK_EVENT_PAUSE;
        esp_err_t ret = audio_player_pause();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to pause audio: %s", esp_err_to_name(ret));
            fclose(Music_File);
            return;
        }
        if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to receive pause event");
            fclose(Music_File);
            return;
        }
        if (audio_player_get_state() != AUDIO_PLAYER_STATE_PAUSE) {
            ESP_LOGE(TAG, "Expected state to be PAUSE");
            fclose(Music_File);
            return;
        }
    }
}


void Volume_adjustment(uint8_t Vol) {
    if(Vol > Volume_MAX )
        printf("Audio : The volume value is incorrect. Please enter 0 to 21\r\n");
    else  
        Volume = Vol;
    ESP_LOGI(TAG, "Volume set to %d", Volume);
}
