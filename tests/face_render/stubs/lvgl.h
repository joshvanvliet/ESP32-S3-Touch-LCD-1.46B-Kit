#pragma once
#include <stdint.h>
#include "esp_timer.h"
typedef int16_t lv_coord_t;
typedef struct { lv_coord_t x1, y1, x2, y2; } lv_area_t;
/* Match the board's LV_COLOR_16_SWAP=1 wire representation. */
typedef union {
    struct { uint16_t green_h:3, red:5, blue:5, green_l:3; } ch;
    uint16_t full;
} lv_color_t;
static inline void lv_color_fill(lv_color_t *dst, lv_color_t c, uint32_t n) {
    while (n--) *dst++ = c;
}
#define LV_COLOR_GET_R(c) ((c).ch.red)
#define LV_COLOR_GET_G(c) (((c).ch.green_h << 3) + (c).ch.green_l)
#define LV_COLOR_GET_B(c) ((c).ch.blue)
#define LV_COLOR_SET_R(c,v) ((c).ch.red = (v))
#define LV_COLOR_SET_G(c,v) do { (c).ch.green_h = (v) >> 3; (c).ch.green_l = (v) & 7; } while (0)
#define LV_COLOR_SET_B(c,v) ((c).ch.blue = (v))
static inline lv_color_t lv_color_make(uint8_t r, uint8_t g, uint8_t b) {
    lv_color_t c;
    LV_COLOR_SET_R(c,r >> 3); LV_COLOR_SET_G(c,g >> 2); LV_COLOR_SET_B(c,b >> 3);
    return c;
}
typedef struct { int unused; } lv_obj_t;
#define LV_OPA_TRANSP 0
#define LV_OBJ_FLAG_CLICKABLE 1
#define LV_OBJ_FLAG_SCROLLABLE 2
static inline lv_obj_t *lv_obj_create(lv_obj_t *p) { static lv_obj_t o; (void)p; return &o; }
#define lv_obj_set_size(...) ((void)0)
#define lv_obj_center(...) ((void)0)
#define lv_obj_set_style_bg_opa(...) ((void)0)
#define lv_obj_set_style_border_width(...) ((void)0)
#define lv_obj_set_style_radius(...) ((void)0)
#define lv_obj_set_style_pad_all(...) ((void)0)
#define lv_obj_clear_flag(...) ((void)0)
#define lv_obj_add_flag(...) ((void)0)
static inline uint32_t lv_tick_get(void) { return (uint32_t)(test_time_us / 1000); }
