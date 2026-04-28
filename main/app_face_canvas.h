#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_face_config.h"
#include "app_face_math.h"
#include "lvgl.h"

#define APP_FACE_MAX_POLY_POINTS 96

typedef struct {
    lv_color_t *pixels;
    int width;
    int height;
    bool clip_rect_enabled;
    int clip_x1;
    int clip_y1;
    int clip_x2;
    int clip_y2;
    bool clip_circle_enabled;
    float clip_cx;
    float clip_cy;
    float clip_r;
    float clip_r_sq;
} app_face_canvas_t;

void app_face_canvas_init(app_face_canvas_t *canvas, lv_color_t *pixels, int width, int height);
void app_face_canvas_clear(app_face_canvas_t *canvas, app_face_rgb_t color);
void app_face_canvas_restore_region(app_face_canvas_t *canvas,
                                    const app_face_canvas_t *source,
                                    int x1,
                                    int y1,
                                    int x2,
                                    int y2);
void app_face_canvas_set_clip_rect(app_face_canvas_t *canvas, int x1, int y1, int x2, int y2);
void app_face_canvas_set_clip_circle(app_face_canvas_t *canvas, float cx, float cy, float radius);
void app_face_canvas_clear_clip(app_face_canvas_t *canvas);
void app_face_canvas_fill_rect(app_face_canvas_t *canvas,
                               int x,
                               int y,
                               int width,
                               int height,
                               app_face_rgb_t color,
                               uint8_t alpha);
void app_face_canvas_fill_ellipse(app_face_canvas_t *canvas,
                                  float cx,
                                  float cy,
                                  float rx,
                                  float ry,
                                  float rotation,
                                  app_face_rgb_t color,
                                  uint8_t alpha);
void app_face_canvas_stroke_ellipse(app_face_canvas_t *canvas,
                                    float cx,
                                    float cy,
                                    float rx,
                                    float ry,
                                    float rotation,
                                    float line_width,
                                    app_face_rgb_t color,
                                    uint8_t alpha);
void app_face_canvas_fill_transformed_ellipse(app_face_canvas_t *canvas,
                                              float cx,
                                              float cy,
                                              float ux,
                                              float uy,
                                              float vx,
                                              float vy,
                                              float rx,
                                              float ry,
                                              app_face_rgb_t color,
                                              uint8_t alpha);
void app_face_canvas_fill_polygon(app_face_canvas_t *canvas,
                                  const app_face_vec2_t *points,
                                  size_t point_count,
                                  app_face_rgb_t color,
                                  uint8_t alpha);
void app_face_canvas_stroke_line(app_face_canvas_t *canvas,
                                 float x1,
                                 float y1,
                                 float x2,
                                 float y2,
                                 float line_width,
                                 app_face_rgb_t color,
                                 uint8_t alpha);
