#include "Display_SPD2010.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "SD_MMC.h"
#include "TCA9554PWR.h"
#include "BAT_Driver.h"
#include "PWR_Key.h"
#include "PCM5101.h"
#include "MIC_Speech.h"
#include "LVGL_Driver.h"
#include "starter_ui.h"

#include "esp_log.h"

static const char *TAG = "STARTER";

static void driver_loop(void *parameter)
{
    (void)parameter;
    while (1) {
        QMI8658_Loop();
        PCF85063_Loop();
        BAT_Get_Volts();
        PWR_Loop();
        vTaskDelay(pdMS_TO_TICKS(100));
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
        "starter_driver",
        4096,
        NULL,
        3,
        NULL,
        0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing board peripherals");
    driver_init();

    LCD_Init();
    LVGL_Init();

    // Speaker and microphone bring-up
    Audio_Init();
    MIC_Speech_init();

    // Short startup chirp on speaker for quick audio sanity-check
    Audio_Play_Test_Tone(1000, 120);
    vTaskDelay(pdMS_TO_TICKS(80));
    Audio_Play_Test_Tone(1400, 120);

    starter_ui_init();

    TickType_t last_ui_update = xTaskGetTickCount();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();

        TickType_t now = xTaskGetTickCount();
        if ((now - last_ui_update) >= pdMS_TO_TICKS(250)) {
            starter_ui_update(BAT_Get_Volts(), &datetime, &Accel, LCD_Backlight);
            last_ui_update = now;
        }
    }
}
