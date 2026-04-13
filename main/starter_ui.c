#include "starter_ui.h"

#include "Display_SPD2010.h"
#include "PCM5101.h"
#include "esp_log.h"

static const char *TAG = "STARTER_UI";

static lv_obj_t *s_battery_label;
static lv_obj_t *s_rtc_label;
static lv_obj_t *s_accel_label;
static lv_obj_t *s_backlight_label;
static lv_obj_t *s_touch_label;
static uint32_t s_touch_count;

static void on_touch_test_btn(lv_event_t *e)
{
    (void)e;
    s_touch_count++;
    lv_label_set_text_fmt(s_touch_label, "Touch OK count: %lu", (unsigned long)s_touch_count);
}

static void on_beep_btn(lv_event_t *e)
{
    (void)e;
    Audio_Play_Test_Tone(1200, 120);
    vTaskDelay(pdMS_TO_TICKS(60));
    Audio_Play_Test_Tone(1700, 120);
}

static void on_backlight_slider(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int value = lv_slider_get_value(slider);

    LCD_Backlight = (uint8_t)value;
    Set_Backlight(LCD_Backlight);
    lv_label_set_text_fmt(s_backlight_label, "Backlight: %d", value);
}

void starter_ui_init(void)
{
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x0F172A), 0);
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xE2E8F0), 0);

    lv_obj_t *container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(container, lv_pct(96), lv_pct(96));
    lv_obj_center(container);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x111827), 0);
    lv_obj_set_style_border_color(container, lv_color_hex(0x334155), 0);
    lv_obj_set_style_border_width(container, 2, 0);
    lv_obj_set_style_radius(container, 12, 0);
    lv_obj_set_style_pad_all(container, 12, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *title = lv_label_create(container);
    lv_label_set_text(title, "Waveshare ESP32-S3 Touch 1.46 Starter");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);

    lv_obj_t *subtitle = lv_label_create(container);
    lv_label_set_text(subtitle, "Screen/touch/battery/mic/speaker bring-up");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0x94A3B8), 0);

    s_battery_label = lv_label_create(container);
    lv_label_set_text(s_battery_label, "Battery: --.- V");

    s_rtc_label = lv_label_create(container);
    lv_label_set_text(s_rtc_label, "RTC: --:--:--");

    s_accel_label = lv_label_create(container);
    lv_label_set_text(s_accel_label, "Accel: x=0.00 y=0.00 z=0.00");

    s_backlight_label = lv_label_create(container);
    lv_label_set_text_fmt(s_backlight_label, "Backlight: %u", (unsigned int)LCD_Backlight);

    lv_obj_t *slider = lv_slider_create(container);
    lv_obj_set_width(slider, lv_pct(100));
    lv_slider_set_range(slider, 0, Backlight_MAX);
    lv_slider_set_value(slider, LCD_Backlight, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider, on_backlight_slider, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *touch_btn = lv_btn_create(container);
    lv_obj_set_width(touch_btn, lv_pct(100));
    lv_obj_add_event_cb(touch_btn, on_touch_test_btn, LV_EVENT_CLICKED, NULL);
    lv_obj_t *touch_btn_label = lv_label_create(touch_btn);
    lv_label_set_text(touch_btn_label, "Touch Test Button");
    lv_obj_center(touch_btn_label);

    s_touch_label = lv_label_create(container);
    lv_label_set_text(s_touch_label, "Touch OK count: 0");

    lv_obj_t *beep_btn = lv_btn_create(container);
    lv_obj_set_width(beep_btn, lv_pct(100));
    lv_obj_add_event_cb(beep_btn, on_beep_btn, LV_EVENT_CLICKED, NULL);
    lv_obj_t *beep_btn_label = lv_label_create(beep_btn);
    lv_label_set_text(beep_btn_label, "Play Speaker Beep");
    lv_obj_center(beep_btn_label);

    lv_obj_t *mic_hint = lv_label_create(container);
    lv_label_set_text(mic_hint, "Mic level appears on serial: App/Speech MIC level avg/peak");
    lv_obj_set_style_text_color(mic_hint, lv_color_hex(0x94A3B8), 0);

    ESP_LOGI(TAG, "Starter UI initialized");
}

void starter_ui_update(float battery_volts, const datetime_t *rtc, const IMUdata *accel, uint8_t backlight)
{
    if (s_battery_label) {
        lv_label_set_text_fmt(s_battery_label, "Battery: %.2f V", (double)battery_volts);
    }

    if (s_rtc_label && rtc) {
        lv_label_set_text_fmt(s_rtc_label, "RTC: %02u:%02u:%02u", rtc->hour, rtc->minute, rtc->second);
    }

    if (s_accel_label && accel) {
        lv_label_set_text_fmt(s_accel_label, "Accel: x=%.2f y=%.2f z=%.2f", (double)accel->x, (double)accel->y, (double)accel->z);
    }

    if (s_backlight_label) {
        lv_label_set_text_fmt(s_backlight_label, "Backlight: %u", (unsigned int)backlight);
    }
}
