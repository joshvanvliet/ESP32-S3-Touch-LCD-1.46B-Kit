#include "app_face_canvas.h"

#include <math.h>
#include <string.h>

static inline int clamp_i(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static inline bool in_clip(const app_face_canvas_t *canvas, int x, int y)
{
    if (canvas->clip_rect_enabled &&
        (x < canvas->clip_x1 || x > canvas->clip_x2 || y < canvas->clip_y1 || y > canvas->clip_y2)) {
        return false;
    }
    if (!canvas->clip_circle_enabled) {
        return true;
    }
    float dx = ((float)x + 0.5f) - canvas->clip_cx;
    float dy = ((float)y + 0.5f) - canvas->clip_cy;
    return dx * dx + dy * dy <= canvas->clip_r_sq;
}

static inline bool clip_bounds(const app_face_canvas_t *canvas, int *x1, int *y1, int *x2, int *y2)
{
    *x1 = clamp_i(*x1, 0, canvas->width - 1);
    *y1 = clamp_i(*y1, 0, canvas->height - 1);
    *x2 = clamp_i(*x2, 0, canvas->width - 1);
    *y2 = clamp_i(*y2, 0, canvas->height - 1);
    if (canvas->clip_rect_enabled) {
        *x1 = clamp_i(*x1, canvas->clip_x1, canvas->clip_x2);
        *y1 = clamp_i(*y1, canvas->clip_y1, canvas->clip_y2);
        *x2 = clamp_i(*x2, canvas->clip_x1, canvas->clip_x2);
        *y2 = clamp_i(*y2, canvas->clip_y1, canvas->clip_y2);
    }
    return *x1 <= *x2 && *y1 <= *y2;
}

static inline bool row_clip_bounds(const app_face_canvas_t *canvas, int y, int *x1, int *x2)
{
    if (!canvas->clip_circle_enabled) {
        return *x1 <= *x2;
    }
    float dy = ((float)y + 0.5f) - canvas->clip_cy;
    float remaining = canvas->clip_r_sq - dy * dy;
    if (remaining <= 0.0f) {
        return false;
    }
    float dx = sqrtf(remaining);
    int cx1 = (int)ceilf(canvas->clip_cx - dx - 0.5f);
    int cx2 = (int)floorf(canvas->clip_cx + dx - 0.5f);
    if (*x1 < cx1) {
        *x1 = cx1;
    }
    if (*x2 > cx2) {
        *x2 = cx2;
    }
    return *x1 <= *x2;
}

static inline void blend_pixel_raw(lv_color_t *dst, app_face_rgb_t color, uint8_t alpha)
{
    if (alpha >= 250) {
        *dst = lv_color_make(color.r, color.g, color.b);
        return;
    }
    if (alpha == 0) {
        return;
    }

    uint16_t inv = (uint16_t)(255 - alpha);
    lv_color_t out;
    LV_COLOR_SET_R(out, (uint8_t)(((uint16_t)(color.r >> 3) * alpha + (uint16_t)LV_COLOR_GET_R(*dst) * inv) / 255));
    LV_COLOR_SET_G(out, (uint8_t)(((uint16_t)(color.g >> 2) * alpha + (uint16_t)LV_COLOR_GET_G(*dst) * inv) / 255));
    LV_COLOR_SET_B(out, (uint8_t)(((uint16_t)(color.b >> 3) * alpha + (uint16_t)LV_COLOR_GET_B(*dst) * inv) / 255));
    *dst = out;
}

static inline void blend_pixel(app_face_canvas_t *canvas, int x, int y, app_face_rgb_t color, uint8_t alpha)
{
    if ((unsigned)x >= (unsigned)canvas->width || (unsigned)y >= (unsigned)canvas->height || !in_clip(canvas, x, y)) {
        return;
    }
    blend_pixel_raw(&canvas->pixels[y * canvas->width + x], color, alpha);
}

static inline void blend_span(app_face_canvas_t *canvas, int y, int x1, int x2, app_face_rgb_t color, uint8_t alpha)
{
    if (x1 > x2 || (unsigned)y >= (unsigned)canvas->height || alpha == 0) {
        return;
    }
    x1 = clamp_i(x1, 0, canvas->width - 1);
    x2 = clamp_i(x2, 0, canvas->width - 1);
    if (!row_clip_bounds(canvas, y, &x1, &x2)) {
        return;
    }
    lv_color_t *row = &canvas->pixels[y * canvas->width];
    if (alpha >= 250) {
        lv_color_t fill = lv_color_make(color.r, color.g, color.b);
        for (int x = x1; x <= x2; x++) {
            row[x] = fill;
        }
        return;
    }
    for (int x = x1; x <= x2; x++) {
        blend_pixel_raw(&row[x], color, alpha);
    }
}

void app_face_canvas_init(app_face_canvas_t *canvas, lv_color_t *pixels, int width, int height)
{
    memset(canvas, 0, sizeof(*canvas));
    canvas->pixels = pixels;
    canvas->width = width;
    canvas->height = height;
}

void app_face_canvas_clear(app_face_canvas_t *canvas, app_face_rgb_t color)
{
    lv_color_t lv_color = lv_color_make(color.r, color.g, color.b);
    for (int i = 0; i < canvas->width * canvas->height; i++) {
        canvas->pixels[i] = lv_color;
    }
}

void app_face_canvas_restore_region(app_face_canvas_t *canvas,
                                    const app_face_canvas_t *source,
                                    int x1,
                                    int y1,
                                    int x2,
                                    int y2)
{
    if (!canvas || !source || !canvas->pixels || !source->pixels ||
        canvas->width != source->width || canvas->height != source->height) {
        return;
    }
    if (!clip_bounds(canvas, &x1, &y1, &x2, &y2)) {
        return;
    }
    size_t count = (size_t)(x2 - x1 + 1) * sizeof(lv_color_t);
    for (int y = y1; y <= y2; y++) {
        memcpy(&canvas->pixels[y * canvas->width + x1],
               &source->pixels[y * source->width + x1],
               count);
    }
}

void app_face_canvas_set_clip_rect(app_face_canvas_t *canvas, int x1, int y1, int x2, int y2)
{
    canvas->clip_rect_enabled = true;
    canvas->clip_x1 = clamp_i(x1, 0, canvas->width - 1);
    canvas->clip_y1 = clamp_i(y1, 0, canvas->height - 1);
    canvas->clip_x2 = clamp_i(x2, 0, canvas->width - 1);
    canvas->clip_y2 = clamp_i(y2, 0, canvas->height - 1);
}

void app_face_canvas_set_clip_circle(app_face_canvas_t *canvas, float cx, float cy, float radius)
{
    canvas->clip_circle_enabled = true;
    canvas->clip_cx = cx;
    canvas->clip_cy = cy;
    canvas->clip_r = radius;
    canvas->clip_r_sq = radius * radius;
}

void app_face_canvas_clear_clip(app_face_canvas_t *canvas)
{
    canvas->clip_rect_enabled = false;
    canvas->clip_circle_enabled = false;
}

void app_face_canvas_fill_rect(app_face_canvas_t *canvas,
                               int x,
                               int y,
                               int width,
                               int height,
                               app_face_rgb_t color,
                               uint8_t alpha)
{
    int x1 = clamp_i(x, 0, canvas->width - 1);
    int y1 = clamp_i(y, 0, canvas->height - 1);
    int x2 = clamp_i(x + width - 1, 0, canvas->width - 1);
    int y2 = clamp_i(y + height - 1, 0, canvas->height - 1);
    if (!clip_bounds(canvas, &x1, &y1, &x2, &y2)) {
        return;
    }
    for (int yy = y1; yy <= y2; yy++) {
        blend_span(canvas, yy, x1, x2, color, alpha);
    }
}

void app_face_canvas_fill_ellipse(app_face_canvas_t *canvas,
                                  float cx,
                                  float cy,
                                  float rx,
                                  float ry,
                                  float rotation,
                                  app_face_rgb_t color,
                                  uint8_t alpha)
{
    if (rx <= 0.5f || ry <= 0.5f || alpha == 0) {
        return;
    }
    float c = cosf(rotation);
    float s = sinf(rotation);
    float extent = fmaxf(rx, ry) + 1.0f;
    int x1 = clamp_i((int)floorf(cx - extent), 0, canvas->width - 1);
    int y1 = clamp_i((int)floorf(cy - extent), 0, canvas->height - 1);
    int x2 = clamp_i((int)ceilf(cx + extent), 0, canvas->width - 1);
    int y2 = clamp_i((int)ceilf(cy + extent), 0, canvas->height - 1);
    if (!clip_bounds(canvas, &x1, &y1, &x2, &y2)) {
        return;
    }
    if (fabsf(rotation) < 0.001f) {
        float inv_ry_sq = 1.0f / (ry * ry);
        for (int y = y1; y <= y2; y++) {
            float py = ((float)y + 0.5f) - cy;
            float qy = py * py * inv_ry_sq;
            if (qy > 1.0f) {
                continue;
            }
            float dx = rx * sqrtf(1.0f - qy);
            int sx = (int)ceilf(cx - dx - 0.5f);
            int ex = (int)floorf(cx + dx - 0.5f);
            if (sx < x1) {
                sx = x1;
            }
            if (ex > x2) {
                ex = x2;
            }
            blend_span(canvas, y, sx, ex, color, alpha);
        }
        return;
    }
    for (int y = y1; y <= y2; y++) {
        int rx1 = x1;
        int rx2 = x2;
        if (!row_clip_bounds(canvas, y, &rx1, &rx2)) {
            continue;
        }
        for (int x = rx1; x <= rx2; x++) {
            float px = ((float)x + 0.5f) - cx;
            float py = ((float)y + 0.5f) - cy;
            float lx = px * c + py * s;
            float ly = -px * s + py * c;
            float q = (lx * lx) / (rx * rx) + (ly * ly) / (ry * ry);
            if (q <= 1.0f) {
                blend_pixel_raw(&canvas->pixels[y * canvas->width + x], color, alpha);
            }
        }
    }
}

void app_face_canvas_stroke_ellipse(app_face_canvas_t *canvas,
                                    float cx,
                                    float cy,
                                    float rx,
                                    float ry,
                                    float rotation,
                                    float line_width,
                                    app_face_rgb_t color,
                                    uint8_t alpha)
{
    if (rx <= 0.5f || ry <= 0.5f || line_width <= 0.5f || alpha == 0) {
        return;
    }
    float c = cosf(rotation);
    float s = sinf(rotation);
    float extent = fmaxf(rx, ry) + line_width + 1.0f;
    float inner_rx = fmaxf(0.5f, rx - line_width);
    float inner_ry = fmaxf(0.5f, ry - line_width);
    int x1 = clamp_i((int)floorf(cx - extent), 0, canvas->width - 1);
    int y1 = clamp_i((int)floorf(cy - extent), 0, canvas->height - 1);
    int x2 = clamp_i((int)ceilf(cx + extent), 0, canvas->width - 1);
    int y2 = clamp_i((int)ceilf(cy + extent), 0, canvas->height - 1);
    if (!clip_bounds(canvas, &x1, &y1, &x2, &y2)) {
        return;
    }
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            float px = ((float)x + 0.5f) - cx;
            float py = ((float)y + 0.5f) - cy;
            float lx = px * c + py * s;
            float ly = -px * s + py * c;
            float q_outer = (lx * lx) / (rx * rx) + (ly * ly) / (ry * ry);
            float q_inner = (lx * lx) / (inner_rx * inner_rx) + (ly * ly) / (inner_ry * inner_ry);
            if (q_outer <= 1.0f && q_inner >= 1.0f) {
                blend_pixel(canvas, x, y, color, alpha);
            }
        }
    }
}

static void sort_intersections(float *values, size_t count)
{
    for (size_t i = 1; i < count; i++) {
        float value = values[i];
        size_t j = i;
        while (j > 0 && values[j - 1] > value) {
            values[j] = values[j - 1];
            j--;
        }
        values[j] = value;
    }
}

void app_face_canvas_fill_polygon(app_face_canvas_t *canvas,
                                  const app_face_vec2_t *points,
                                  size_t point_count,
                                  app_face_rgb_t color,
                                  uint8_t alpha)
{
    if (!points || point_count < 3 || alpha == 0) {
        return;
    }
    if (point_count > APP_FACE_MAX_POLY_POINTS) {
        point_count = APP_FACE_MAX_POLY_POINTS;
    }

    float min_x = points[0].x;
    float max_x = points[0].x;
    float min_y = points[0].y;
    float max_y = points[0].y;
    for (size_t i = 1; i < point_count; i++) {
        min_x = fminf(min_x, points[i].x);
        max_x = fmaxf(max_x, points[i].x);
        min_y = fminf(min_y, points[i].y);
        max_y = fmaxf(max_y, points[i].y);
    }
    int x1 = clamp_i((int)floorf(min_x), 0, canvas->width - 1);
    int y1 = clamp_i((int)floorf(min_y), 0, canvas->height - 1);
    int x2 = clamp_i((int)ceilf(max_x), 0, canvas->width - 1);
    int y2 = clamp_i((int)ceilf(max_y), 0, canvas->height - 1);
    if (!clip_bounds(canvas, &x1, &y1, &x2, &y2)) {
        return;
    }
    float intersections[APP_FACE_MAX_POLY_POINTS];
    for (int y = y1; y <= y2; y++) {
        int rx1 = x1;
        int rx2 = x2;
        if (!row_clip_bounds(canvas, y, &rx1, &rx2)) {
            continue;
        }
        float scan_y = (float)y + 0.5f;
        size_t intersection_count = 0;
        for (size_t i = 0, j = point_count - 1; i < point_count; j = i++) {
            float y0 = points[j].y;
            float y1p = points[i].y;
            bool crosses = (y0 <= scan_y && y1p > scan_y) || (y1p <= scan_y && y0 > scan_y);
            if (crosses && intersection_count < APP_FACE_MAX_POLY_POINTS) {
                float x0 = points[j].x;
                float x1p = points[i].x;
                intersections[intersection_count++] = x0 + (scan_y - y0) * (x1p - x0) / (y1p - y0);
            }
        }
        sort_intersections(intersections, intersection_count);
        for (size_t i = 0; i + 1 < intersection_count; i += 2) {
            int sx = (int)ceilf(intersections[i] - 0.5f);
            int ex = (int)floorf(intersections[i + 1] - 0.5f);
            if (sx < rx1) {
                sx = rx1;
            }
            if (ex > rx2) {
                ex = rx2;
            }
            if (sx <= ex) {
                blend_span(canvas, y, sx, ex, color, alpha);
            }
        }
    }
}

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
                                              uint8_t alpha)
{
    if (rx <= 0.5f || ry <= 0.5f || alpha == 0) {
        return;
    }

    float det = ux * vy - uy * vx;
    if (fabsf(det) < 0.0001f) {
        return;
    }

    float extent_x = fabsf(ux) * rx + fabsf(vx) * ry + 1.0f;
    float extent_y = fabsf(uy) * rx + fabsf(vy) * ry + 1.0f;
    int x1 = clamp_i((int)floorf(cx - extent_x), 0, canvas->width - 1);
    int y1 = clamp_i((int)floorf(cy - extent_y), 0, canvas->height - 1);
    int x2 = clamp_i((int)ceilf(cx + extent_x), 0, canvas->width - 1);
    int y2 = clamp_i((int)ceilf(cy + extent_y), 0, canvas->height - 1);
    if (!clip_bounds(canvas, &x1, &y1, &x2, &y2)) {
        return;
    }

    float inv_det = 1.0f / det;
    float a = vy * inv_det;
    float b = -vx * inv_det;
    float c = -uy * inv_det;
    float d = ux * inv_det;
    float inv_rx_sq = 1.0f / (rx * rx);
    float inv_ry_sq = 1.0f / (ry * ry);
    float qa = a * a * inv_rx_sq + c * c * inv_ry_sq;
    float qb_scale = 2.0f * (a * b * inv_rx_sq + c * d * inv_ry_sq);
    float qc_scale = b * b * inv_rx_sq + d * d * inv_ry_sq;
    if (qa <= 0.000001f) {
        return;
    }

    for (int y = y1; y <= y2; y++) {
        int rx1 = x1;
        int rx2 = x2;
        if (!row_clip_bounds(canvas, y, &rx1, &rx2)) {
            continue;
        }
        float dy = ((float)y + 0.5f) - cy;
        float qb = qb_scale * dy;
        float qc = qc_scale * dy * dy - 1.0f;
        float disc = qb * qb - 4.0f * qa * qc;
        if (disc < 0.0f) {
            continue;
        }
        float root = sqrtf(disc);
        float inv_2qa = 0.5f / qa;
        float dx0 = (-qb - root) * inv_2qa;
        float dx1 = (-qb + root) * inv_2qa;
        int sx = (int)ceilf(cx + fminf(dx0, dx1) - 0.5f);
        int ex = (int)floorf(cx + fmaxf(dx0, dx1) - 0.5f);
        if (sx < rx1) {
            sx = rx1;
        }
        if (ex > rx2) {
            ex = rx2;
        }
        if (sx <= ex) {
            blend_span(canvas, y, sx, ex, color, alpha);
        }
    }
}

void app_face_canvas_stroke_line(app_face_canvas_t *canvas,
                                 float x1,
                                 float y1,
                                 float x2,
                                 float y2,
                                 float line_width,
                                 app_face_rgb_t color,
                                 uint8_t alpha)
{
    if (line_width <= 0.5f || alpha == 0) {
        return;
    }
    float dx = x2 - x1;
    float dy = y2 - y1;
    float len_sq = dx * dx + dy * dy;
    float radius = line_width * 0.5f;
    int bx1 = clamp_i((int)floorf(fminf(x1, x2) - radius - 1.0f), 0, canvas->width - 1);
    int by1 = clamp_i((int)floorf(fminf(y1, y2) - radius - 1.0f), 0, canvas->height - 1);
    int bx2 = clamp_i((int)ceilf(fmaxf(x1, x2) + radius + 1.0f), 0, canvas->width - 1);
    int by2 = clamp_i((int)ceilf(fmaxf(y1, y2) + radius + 1.0f), 0, canvas->height - 1);
    if (!clip_bounds(canvas, &bx1, &by1, &bx2, &by2)) {
        return;
    }

    for (int y = by1; y <= by2; y++) {
        for (int x = bx1; x <= bx2; x++) {
            float px = (float)x + 0.5f;
            float py = (float)y + 0.5f;
            float t = len_sq > 0.0001f ? ((px - x1) * dx + (py - y1) * dy) / len_sq : 0.0f;
            t = app_face_clampf(t, 0.0f, 1.0f);
            float cx = x1 + dx * t;
            float cy = y1 + dy * t;
            float ddx = px - cx;
            float ddy = py - cy;
            if (ddx * ddx + ddy * ddy <= radius * radius) {
                blend_pixel(canvas, x, y, color, alpha);
            }
        }
    }
}
