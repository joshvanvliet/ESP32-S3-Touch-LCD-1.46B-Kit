#include "Display_SPD2010.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "SD_MMC.h"
#include "TCA9554PWR.h"
#include "BAT_Driver.h"
#include "PWR_Key.h"
#include "PCM5101.h"
#include "LVGL_Driver.h"
#include "app_state.h"
#if CONFIG_APP_WAKEWORD_ENABLED
#include "MIC_Speech.h"
#endif

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "APP";

static void driver_loop(void *parameter)
{
    (void)parameter;
    uint8_t slow_div = 0;

    while (1) {
        PWR_Loop();

        slow_div++;
        if (slow_div >= 5) {
            slow_div = 0;
            QMI8658_Loop();
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
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing board peripherals");
    driver_init();

    LCD_Init();
    LVGL_Init();
    Audio_Init();
#if CONFIG_APP_WAKEWORD_ENABLED
    MIC_Speech_init();
#endif

    Audio_Play_Test_Tone(1000, 100);
    vTaskDelay(pdMS_TO_TICKS(60));
    Audio_Play_Test_Tone(1400, 100);

    ESP_ERROR_CHECK(app_state_init());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
        app_state_process();
    }
}
