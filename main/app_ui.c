#include "app_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Display_SPD2010.h"
#include "app_face.h"
#include "esp_log.h"
#include "lvgl.h"
#include "sdkconfig.h"

static const char *TAG = "APP_UI";

static app_ui_callbacks_t s_callbacks;
static app_device_state_t s_state = APP_STATE_UNPAIRED;
static lv_obj_t *s_root;
static lv_obj_t *s_status;
static lv_obj_t *s_passkey;
static float s_recording_energy;
static bool s_recording_energy_valid;

#define APP_AGENT_ACTIVITY_IDLE 0
#define APP_AGENT_ACTIVITY_THINKING 1
#define APP_AGENT_ACTIVITY_SPEAKING 2
#define APP_AGENT_ACTIVITY_ERROR 3

static app_face_mode_t app_ui_face_mode_for_state(app_device_state_t state)
{
    switch (state) {
        case APP_STATE_UNPAIRED:
        case APP_STATE_LINKED:
            return APP_FACE_SLEEPY;
        case APP_STATE_PAIRING:
            return APP_FACE_CURIOUS;
        case APP_STATE_RECORDING:
            return APP_FACE_LISTENING;
        case APP_STATE_UPLOADING:
            return APP_FACE_THINKING;
        case APP_STATE_READY:
        case APP_STATE_RESULT:
        default:
            return APP_FACE_IDLE;
    }
}

static const char *app_ui_state_text(app_device_state_t state)
{
    switch (state) {
        case APP_STATE_UNPAIRED:
            return "Waiting for phone";
        case APP_STATE_PAIRING:
            return "Pairing";
        case APP_STATE_LINKED:
            return "Paired";
        case APP_STATE_READY:
            return "Ready";
        case APP_STATE_RECORDING:
            return "Listening";
        case APP_STATE_UPLOADING:
            return "Thinking";
        case APP_STATE_RESULT:
            return "Ready";
        default:
            return "";
    }
}

static void app_ui_on_face_clicked(lv_event_t *event)
{
    (void)event;
    app_face_tap();
    if (s_callbacks.on_primary_action) {
        s_callbacks.on_primary_action();
    }
}

static void app_ui_add_protected_label(lv_obj_t *label, lv_area_t *areas, size_t *count, size_t cap)
{
    if (!label || !areas || !count || *count >= cap || lv_obj_has_flag(label, LV_OBJ_FLAG_HIDDEN)) {
        return;
    }
    lv_obj_update_layout(label);
    lv_obj_get_coords(label, &areas[*count]);
    (*count)++;
}

static void app_ui_update_face_protection(void)
{
    lv_area_t areas[2];
    size_t count = 0;
    app_ui_add_protected_label(s_status, areas, &count, 2);
    app_ui_add_protected_label(s_passkey, areas, &count, 2);
    app_face_set_protected_areas(areas, count);
}

static lv_obj_t *app_ui_create_overlay_label(lv_obj_t *parent, lv_align_t align, lv_coord_t y_offset)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, EXAMPLE_LCD_WIDTH - 48);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xEFFFFD), 0);
    lv_obj_set_style_bg_color(label, lv_color_hex(0x05070B), 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_70, 0);
    lv_obj_set_style_radius(label, 8, 0);
    lv_obj_set_style_pad_hor(label, 12, 0);
    lv_obj_set_style_pad_ver(label, 7, 0);
    lv_obj_align(label, align, 0, y_offset);
    return label;
}

static void app_ui_apply_state(void)
{
    app_face_set_mode(app_ui_face_mode_for_state(s_state));
    if (s_status) {
        lv_label_set_text(s_status, app_ui_state_text(s_state));
        app_ui_update_face_protection();
    }
}

void app_ui_init(const app_ui_callbacks_t *callbacks)
{
    memset(&s_callbacks, 0, sizeof(s_callbacks));
    if (callbacks) {
        s_callbacks = *callbacks;
    }

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x010205), 0);
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xEFFFFD), 0);

    s_root = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_root, EXAMPLE_LCD_WIDTH, EXAMPLE_LCD_HEIGHT);
    lv_obj_center(s_root);
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x010205), 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 0, 0);
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    esp_err_t err = app_face_init(s_root);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Face init failed: %s", esp_err_to_name(err));
    }

    lv_obj_t *face = app_face_get_obj();
#if CONFIG_APP_FACE_TOUCH_START_ENABLED
    if (face) {
        lv_obj_add_event_cb(face, app_ui_on_face_clicked, LV_EVENT_CLICKED, NULL);
    }
#else
    (void)face;
#endif

    s_status = app_ui_create_overlay_label(s_root, LV_ALIGN_TOP_MID, 14);
    s_passkey = app_ui_create_overlay_label(s_root, LV_ALIGN_CENTER, 90);

    lv_label_set_text(s_passkey, "");
    lv_obj_add_flag(s_passkey, LV_OBJ_FLAG_HIDDEN);

    app_ui_apply_state();
    app_ui_update_face_protection();
    ESP_LOGI(TAG, "Face UI initialized");
}

void app_ui_process(void)
{
    uint32_t now_ms = lv_tick_get();
    app_face_tick(now_ms);
}

void app_ui_set_state(app_device_state_t state)
{
    if (s_state != state) {
        s_recording_energy_valid = false;
    }
    s_state = state;
    app_ui_apply_state();
}

void app_ui_set_agent_activity(uint8_t status)
{
    switch (status) {
        case APP_AGENT_ACTIVITY_THINKING:
            app_face_set_mode(APP_FACE_THINKING);
            if (s_status) {
                lv_label_set_text(s_status, "Thinking");
                app_ui_update_face_protection();
            }
            break;
        case APP_AGENT_ACTIVITY_SPEAKING:
            app_face_set_mode(APP_FACE_SPEAKING);
            app_face_set_energy(0.82f);
            if (s_status) {
                lv_label_set_text(s_status, "Speaking");
                app_ui_update_face_protection();
            }
            break;
        case APP_AGENT_ACTIVITY_ERROR:
            app_face_pulse_mode(APP_FACE_ERROR, 1800);
            if (s_status) {
                lv_label_set_text(s_status, "Error");
                app_ui_update_face_protection();
            }
            break;
        case APP_AGENT_ACTIVITY_IDLE:
        default:
            app_face_set_energy(s_state == APP_STATE_RECORDING ? 0.55f : 0.42f);
            app_face_set_mode(app_ui_face_mode_for_state(s_state));
            if (s_status) {
                lv_label_set_text(s_status, app_ui_state_text(s_state));
                app_ui_update_face_protection();
            }
            break;
    }
}

void app_ui_set_status_text(const char *text)
{
    if (!s_status) {
        return;
    }
    lv_label_set_text(s_status, text && text[0] != '\0' ? text : app_ui_state_text(s_state));
    app_ui_update_face_protection();

    if (text && strstr(text, "Wake detected")) {
        app_face_pulse_mode(APP_FACE_SURPRISED, 700);
    } else if (text && strstr(text, "failed")) {
        app_face_pulse_mode(APP_FACE_ERROR, 1800);
    } else if (text && strstr(text, "Playback complete")) {
        app_face_pulse_mode(APP_FACE_HAPPY, 1100);
    } else if (text && strstr(text, "Listening")) {
        app_face_set_mode(APP_FACE_LISTENING);
    }
}

void app_ui_set_pairing_passkey(uint32_t passkey, bool visible)
{
    if (!s_passkey) {
        return;
    }
    if (!visible) {
        lv_obj_add_flag(s_passkey, LV_OBJ_FLAG_HIDDEN);
        app_ui_update_face_protection();
        return;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "Passkey %06lu", (unsigned long)passkey);
    lv_label_set_text(s_passkey, buf);
    lv_obj_clear_flag(s_passkey, LV_OBJ_FLAG_HIDDEN);
    app_ui_update_face_protection();
    app_face_set_mode(APP_FACE_CURIOUS);
}

void app_ui_set_recording_level(uint16_t level)
{
    if (s_state != APP_STATE_RECORDING) {
        s_recording_energy = 0.42f;
        s_recording_energy_valid = false;
        app_face_set_energy(s_recording_energy);
        return;
    }

    uint16_t clipped = level > 1000 ? 1000 : level;
    float normalized = (float)clipped / 1000.0f;
    float gated = normalized <= 0.08f ? 0.0f : (normalized - 0.08f) / 0.92f;
    float compressed = sqrtf(gated > 0.0f ? gated : 0.0f);
    float target = 0.40f + compressed * 0.24f;
    if (!s_recording_energy_valid) {
        s_recording_energy = target;
        s_recording_energy_valid = true;
    } else {
        float smoothing = target > s_recording_energy ? 0.16f : 0.06f;
        s_recording_energy += (target - s_recording_energy) * smoothing;
    }
    app_face_set_energy(s_recording_energy);
}

void app_ui_set_transcript(const char *text, uint8_t status)
{
    (void)text;

    if (status == 0) {
        app_face_pulse_mode(APP_FACE_HAPPY, 900);
    } else {
        app_face_pulse_mode(APP_FACE_ERROR, 1600);
    }
}
