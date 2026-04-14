#include "app_ui.h"

#include <stdio.h>
#include <string.h>

#include "Display_SPD2010.h"
#include "esp_log.h"
#include "lvgl.h"

static const char *TAG = "APP_UI";

static app_ui_callbacks_t s_callbacks;
static app_device_state_t s_state = APP_STATE_UNPAIRED;

static lv_obj_t *s_root;
static lv_obj_t *s_title;
static lv_obj_t *s_status_chip;
static lv_obj_t *s_primary_btn;
static lv_obj_t *s_primary_btn_label;
static lv_obj_t *s_hint;
static lv_obj_t *s_arc;
static lv_obj_t *s_transcript;
static lv_obj_t *s_passkey;

static void app_ui_apply_state(void)
{
    switch (s_state) {
        case APP_STATE_UNPAIRED:
            lv_label_set_text(s_status_chip, "Not connected");
            lv_label_set_text(s_primary_btn_label, "Waiting...");
            lv_label_set_text(s_hint, "Waiting for phone Bluetooth connection");
            lv_obj_add_state(s_primary_btn, LV_STATE_DISABLED);
            lv_obj_add_flag(s_primary_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
            break;

        case APP_STATE_PAIRING:
            lv_label_set_text(s_status_chip, "Pairing");
            lv_label_set_text(s_primary_btn_label, "Waiting...");
            lv_label_set_text(s_hint, "Confirm passkey in phone app");
            lv_obj_add_state(s_primary_btn, LV_STATE_DISABLED);
            lv_obj_add_flag(s_primary_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
            break;

        case APP_STATE_LINKED:
            lv_label_set_text(s_status_chip, "Paired");
            lv_label_set_text(s_primary_btn_label, "Waiting...");
            lv_label_set_text(s_hint, "Open companion app to finish connection");
            lv_obj_add_state(s_primary_btn, LV_STATE_DISABLED);
            lv_obj_add_flag(s_primary_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
            break;

        case APP_STATE_READY:
            lv_label_set_text(s_status_chip, "Ready");
            lv_label_set_text(s_primary_btn_label, "Tap to Talk");
            lv_label_set_text(s_hint, "Audio streams to phone");
            lv_obj_clear_state(s_primary_btn, LV_STATE_DISABLED);
            lv_obj_clear_flag(s_primary_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
            break;

        case APP_STATE_RECORDING:
            lv_label_set_text(s_status_chip, "Recording");
            lv_label_set_text(s_primary_btn_label, "Stop");
            lv_label_set_text(s_hint, "Press PWR to stop");
            lv_obj_clear_state(s_primary_btn, LV_STATE_DISABLED);
            lv_obj_clear_flag(s_primary_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
            break;

        case APP_STATE_UPLOADING:
            lv_label_set_text(s_status_chip, "Uploading");
            lv_label_set_text(s_primary_btn_label, "Waiting...");
            lv_label_set_text(s_hint, "Transcribing on phone");
            lv_obj_add_state(s_primary_btn, LV_STATE_DISABLED);
            lv_obj_clear_flag(s_primary_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
            break;

        case APP_STATE_RESULT:
            lv_label_set_text(s_status_chip, "Result");
            lv_label_set_text(s_primary_btn_label, "New Capture");
            lv_label_set_text(s_hint, "Review transcript on phone");
            lv_obj_clear_state(s_primary_btn, LV_STATE_DISABLED);
            lv_obj_clear_flag(s_primary_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);
            break;

        default:
            break;
    }
}

static void app_ui_on_primary(lv_event_t *e)
{
    (void)e;
    if (s_callbacks.on_primary_action) {
        s_callbacks.on_primary_action();
    }
}

void app_ui_init(const app_ui_callbacks_t *callbacks)
{
    memset(&s_callbacks, 0, sizeof(s_callbacks));
    if (callbacks) {
        s_callbacks = *callbacks;
    }

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xF5F7FB), 0);
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0x111827), 0);

    s_root = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_root, lv_pct(86), lv_pct(86));
    lv_obj_center(s_root);
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 24, 0);
    lv_obj_set_style_pad_all(s_root, 14, 0);
    lv_obj_set_style_pad_row(s_root, 10, 0);
    lv_obj_set_scrollbar_mode(s_root, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_shadow_width(s_root, 14, 0);
    lv_obj_set_style_shadow_opa(s_root, LV_OPA_20, 0);
    lv_obj_set_style_shadow_color(s_root, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_root, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_title = lv_label_create(s_root);
    lv_obj_set_width(s_title, lv_pct(100));
    lv_obj_set_style_text_align(s_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(s_title, &lv_font_montserrat_16, 0);
    lv_label_set_text(s_title, "ESP Voice Bridge");

    s_status_chip = lv_label_create(s_root);
    lv_obj_set_style_bg_color(s_status_chip, lv_color_hex(0xE2E8F0), 0);
    lv_obj_set_style_radius(s_status_chip, 12, 0);
    lv_obj_set_style_pad_hor(s_status_chip, 10, 0);
    lv_obj_set_style_pad_ver(s_status_chip, 6, 0);
    lv_label_set_text(s_status_chip, "Booting");

    s_passkey = lv_label_create(s_root);
    lv_obj_set_width(s_passkey, lv_pct(100));
    lv_obj_set_style_text_align(s_passkey, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_passkey, lv_color_hex(0x334155), 0);
    lv_label_set_text(s_passkey, "Passkey: ------");
    lv_obj_add_flag(s_passkey, LV_OBJ_FLAG_HIDDEN);

    s_arc = lv_arc_create(s_root);
    lv_obj_set_size(s_arc, 150, 150);
    lv_arc_set_range(s_arc, 0, 1000);
    lv_arc_set_value(s_arc, 0);
    lv_obj_remove_style(s_arc, NULL, LV_PART_KNOB);
    lv_obj_add_flag(s_arc, LV_OBJ_FLAG_HIDDEN);

    s_primary_btn = lv_btn_create(s_root);
    lv_obj_set_width(s_primary_btn, lv_pct(88));
    lv_obj_set_height(s_primary_btn, 58);
    lv_obj_set_style_bg_color(s_primary_btn, lv_color_hex(0x0EA5E9), 0);
    lv_obj_set_style_radius(s_primary_btn, 14, 0);
    lv_obj_add_event_cb(s_primary_btn, app_ui_on_primary, LV_EVENT_CLICKED, NULL);

    s_primary_btn_label = lv_label_create(s_primary_btn);
    lv_obj_set_style_text_font(s_primary_btn_label, &lv_font_montserrat_16, 0);
    lv_label_set_text(s_primary_btn_label, "Tap");
    lv_obj_center(s_primary_btn_label);

    s_hint = lv_label_create(s_root);
    lv_obj_set_width(s_hint, lv_pct(96));
    lv_obj_set_style_text_align(s_hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_hint, lv_color_hex(0x64748B), 0);
    lv_label_set_text(s_hint, "");

    s_transcript = lv_label_create(s_root);
    lv_obj_set_width(s_transcript, lv_pct(92));
    lv_label_set_long_mode(s_transcript, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(s_transcript, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(s_transcript, lv_color_hex(0xF8FAFC), 0);
    lv_obj_set_style_pad_all(s_transcript, 10, 0);
    lv_obj_set_style_radius(s_transcript, 10, 0);
    lv_obj_set_style_text_color(s_transcript, lv_color_hex(0x0F172A), 0);
    lv_label_set_text(s_transcript, "");
    lv_obj_add_flag(s_transcript, LV_OBJ_FLAG_HIDDEN);

    app_ui_apply_state();
    ESP_LOGI(TAG, "UI initialized");
}

void app_ui_set_state(app_device_state_t state)
{
    s_state = state;
    app_ui_apply_state();
}

void app_ui_set_status_text(const char *text)
{
    if (!s_hint) {
        return;
    }
    lv_label_set_text(s_hint, text ? text : "");
}

void app_ui_set_pairing_passkey(uint32_t passkey, bool visible)
{
    if (!s_passkey) {
        return;
    }

    if (!visible) {
        lv_obj_add_flag(s_passkey, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "Passkey: %06lu", (unsigned long)passkey);
    lv_label_set_text(s_passkey, buf);
    lv_obj_clear_flag(s_passkey, LV_OBJ_FLAG_HIDDEN);
}

void app_ui_set_recording_level(uint16_t level)
{
    if (!s_arc) {
        return;
    }
    uint16_t clipped = level > 1000 ? 1000 : level;
    lv_arc_set_value(s_arc, clipped);
}

void app_ui_set_transcript(const char *text, uint8_t status)
{
    if (!s_transcript) {
        return;
    }

    char buf[APP_MAX_TRANSCRIPT_BYTES + 20];
    snprintf(buf, sizeof(buf), "%s%s", status == 0 ? "" : "[Check] ", text ? text : "");
    lv_label_set_text(s_transcript, buf);
}
