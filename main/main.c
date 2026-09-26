#include "Display_SPD2010.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "SD_MMC.h"
#include "TCA9554PWR.h"
#include "BAT_Driver.h"
#include "PWR_Key.h"
#include "PCM5101.h"
#include "LVGL_Driver.h"
#include "app_motion.h"
#include "app_state.h"
#include "app_ui.h"
#include "app_face.h"

#include "esp_log.h"
#include "esp_pm.h"
#include "nvs_flash.h"

static const char *TAG = "APP";

static void driver_loop(void *parameter)
{
    (void)parameter;
    uint8_t slow_div = 0;

    while (1) {
        PWR_Loop();
        QMI8658_Loop();
        app_motion_update_from_imu(Accel.x, Accel.y, Accel.z, Gyro.x, Gyro.y, Gyro.z);

        slow_div++;
        if (slow_div >= 5) {
            slow_div = 0;
            PCF85063_Loop();
            BAT_Get_Volts();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void driver_init(void)
{
    PWR_Init();
    BAT_Init();
    I2C_Init();
    EXIO_Init();
    Flash_Searching();
    PCF85063_Init();
    QMI8658_Init();

    xTaskCreatePinnedToCore(
        driver_loop,
        "driver_loop",
        4096,
        NULL,
        3,
        NULL,
        0);
}

void app_main(void)
{
    esp_log_level_set("lcd_panel.io.spi", ESP_LOG_NONE);
    esp_log_level_set("spd2010", ESP_LOG_NONE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if CONFIG_PM_ENABLE
    /* Keep peripheral clocks at 80 MHz and full CPU speed whenever tasks are
     * runnable. Only reduce CPU frequency while both cores are idle. Audio
     * DMA and the live BLE link must continue, so do not enable light sleep. */
    const esp_pm_config_t power_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = 80,
        .light_sleep_enable = false,
    };
    ret = esp_pm_configure(&power_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Power management unavailable: %s; keeping startup clocks", esp_err_to_name(ret));
    }
#endif

    ESP_LOGI(TAG, "Initializing board peripherals");
    driver_init();

    LCD_Init();
    LVGL_Init();
    Audio_Init();

    Audio_Play_Test_Tone(1000, 100);
    vTaskDelay(pdMS_TO_TICKS(60));
    Audio_Play_Test_Tone(1400, 100);

    ESP_ERROR_CHECK(app_state_init());

    uint32_t wait_ms = 1;
    while (1) {
        app_state_wait(wait_ms);
        app_state_process();
        app_ui_process();
        uint32_t lvgl_wait_ms = lv_timer_handler();
        /* Sleep until useful work is due. Events wake us immediately; cap the
         * timeout at 5 ms for state timers and audio credit/level servicing. */
        wait_ms = app_face_next_frame_delay_ms();
        if (wait_ms > lvgl_wait_ms) wait_ms = lvgl_wait_ms;
        if (wait_ms > 5) wait_ms = 5;
        if (!wait_ms) wait_ms = 1;
    }
}
