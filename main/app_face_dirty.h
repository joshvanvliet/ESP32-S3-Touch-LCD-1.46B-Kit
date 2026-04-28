#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lvgl.h"

#define APP_FACE_MAX_DIRTY_RECTS 10
#define APP_FACE_MAX_BLIT_FRAGMENTS 16
#define APP_FACE_DIRTY_MERGE_GAP 3

typedef struct {
    lv_area_t rects[APP_FACE_MAX_DIRTY_RECTS];
    size_t count;
} app_face_dirty_list_t;

int app_face_clamp_int(int value, int min_value, int max_value);
void app_face_area_reset(lv_area_t *area, int width, int height);
bool app_face_area_is_empty(const lv_area_t *area);
void app_face_area_include_rect(lv_area_t *area, float x1, float y1, float x2, float y2);
void app_face_area_include_area(lv_area_t *area, const lv_area_t *other);
void app_face_area_include_transformed_ellipse(lv_area_t *area,
                                               float cx,
                                               float cy,
                                               float ux,
                                               float uy,
                                               float vx,
                                               float vy,
                                               float rx,
                                               float ry,
                                               float margin);
void app_face_area_inflate_clip(lv_area_t *area, int margin, int width, int height);
bool app_face_area_clip_to_bounds(lv_area_t *area, int width, int height);
int app_face_area_width(const lv_area_t *area);
int app_face_area_height(const lv_area_t *area);
uint32_t app_face_area_pixel_count(const lv_area_t *area);
bool app_face_area_intersects(const lv_area_t *a, const lv_area_t *b);
bool app_face_clip_area_to_area(lv_area_t *area, const lv_area_t *clip, int width, int height);

void app_face_dirty_list_reset(app_face_dirty_list_t *list);
void app_face_dirty_list_add_area(app_face_dirty_list_t *list, lv_area_t area, int width, int height);
void app_face_dirty_list_add_list(app_face_dirty_list_t *dst,
                                  const app_face_dirty_list_t *src,
                                  int width,
                                  int height);
size_t app_face_dirty_subtract_protected_areas(const lv_area_t *area,
                                               const lv_area_t *protected_areas,
                                               size_t protected_area_count,
                                               int protected_margin,
                                               int width,
                                               int height,
                                               lv_area_t *out_fragments,
                                               size_t max_fragments);

#ifndef APP_FACE_DIRTY_NO_SHORT_NAMES
#define clamp_int app_face_clamp_int
#define area_reset app_face_area_reset
#define area_is_empty app_face_area_is_empty
#define area_include_rect app_face_area_include_rect
#define area_include_area app_face_area_include_area
#define area_include_transformed_ellipse app_face_area_include_transformed_ellipse
#define area_inflate_clip app_face_area_inflate_clip
#define area_clip_to_bounds app_face_area_clip_to_bounds
#define area_width app_face_area_width
#define area_height app_face_area_height
#define area_pixel_count app_face_area_pixel_count
#define area_intersects app_face_area_intersects
#define clip_area_to_area app_face_clip_area_to_area
#define dirty_list_reset app_face_dirty_list_reset
#define dirty_list_add_area app_face_dirty_list_add_area
#define dirty_list_add_list app_face_dirty_list_add_list
#endif
