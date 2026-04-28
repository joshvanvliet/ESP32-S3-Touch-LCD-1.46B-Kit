#define APP_FACE_DIRTY_NO_SHORT_NAMES
#include "app_face_dirty.h"

#include <math.h>
#include <string.h>

int app_face_clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void app_face_area_reset(lv_area_t *area, int width, int height)
{
    area->x1 = width;
    area->y1 = height;
    area->x2 = -1;
    area->y2 = -1;
}

bool app_face_area_is_empty(const lv_area_t *area)
{
    return area->x1 > area->x2 || area->y1 > area->y2;
}

void app_face_area_include_rect(lv_area_t *area, float x1, float y1, float x2, float y2)
{
    int ix1 = (int)floorf(fminf(x1, x2));
    int iy1 = (int)floorf(fminf(y1, y2));
    int ix2 = (int)ceilf(fmaxf(x1, x2));
    int iy2 = (int)ceilf(fmaxf(y1, y2));
    if (app_face_area_is_empty(area)) {
        area->x1 = ix1;
        area->y1 = iy1;
        area->x2 = ix2;
        area->y2 = iy2;
        return;
    }
    area->x1 = area->x1 < ix1 ? area->x1 : ix1;
    area->y1 = area->y1 < iy1 ? area->y1 : iy1;
    area->x2 = area->x2 > ix2 ? area->x2 : ix2;
    area->y2 = area->y2 > iy2 ? area->y2 : iy2;
}

void app_face_area_include_area(lv_area_t *area, const lv_area_t *other)
{
    if (app_face_area_is_empty(other)) {
        return;
    }
    app_face_area_include_rect(area, other->x1, other->y1, other->x2, other->y2);
}

void app_face_area_include_transformed_ellipse(lv_area_t *area,
                                               float cx,
                                               float cy,
                                               float ux,
                                               float uy,
                                               float vx,
                                               float vy,
                                               float rx,
                                               float ry,
                                               float margin)
{
    float half_x = fabsf(ux) * rx + fabsf(vx) * ry + margin;
    float half_y = fabsf(uy) * rx + fabsf(vy) * ry + margin;
    app_face_area_include_rect(area, cx - half_x, cy - half_y, cx + half_x, cy + half_y);
}

void app_face_area_inflate_clip(lv_area_t *area, int margin, int width, int height)
{
    if (app_face_area_is_empty(area)) {
        area->x1 = 0;
        area->y1 = 0;
        area->x2 = width - 1;
        area->y2 = height - 1;
        return;
    }
    area->x1 = app_face_clamp_int(area->x1 - margin, 0, width - 1);
    area->y1 = app_face_clamp_int(area->y1 - margin, 0, height - 1);
    area->x2 = app_face_clamp_int(area->x2 + margin, 0, width - 1);
    area->y2 = app_face_clamp_int(area->y2 + margin, 0, height - 1);
    area->x1 = (area->x1 >> 2) << 2;
    area->x2 = app_face_clamp_int(((area->x2 >> 2) << 2) + 3, 0, width - 1);
}

bool app_face_area_clip_to_bounds(lv_area_t *area, int width, int height)
{
    if (app_face_area_is_empty(area)) {
        return false;
    }
    area->x1 = app_face_clamp_int(area->x1, 0, width - 1);
    area->y1 = app_face_clamp_int(area->y1, 0, height - 1);
    area->x2 = app_face_clamp_int(area->x2, 0, width - 1);
    area->y2 = app_face_clamp_int(area->y2, 0, height - 1);
    area->x1 = (area->x1 >> 2) << 2;
    area->x2 = app_face_clamp_int(((area->x2 >> 2) << 2) + 3, 0, width - 1);
    return !app_face_area_is_empty(area);
}

int app_face_area_width(const lv_area_t *area)
{
    return app_face_area_is_empty(area) ? 0 : (int)(area->x2 - area->x1 + 1);
}

int app_face_area_height(const lv_area_t *area)
{
    return app_face_area_is_empty(area) ? 0 : (int)(area->y2 - area->y1 + 1);
}

uint32_t app_face_area_pixel_count(const lv_area_t *area)
{
    return (uint32_t)app_face_area_width(area) * (uint32_t)app_face_area_height(area);
}

bool app_face_area_intersects(const lv_area_t *a, const lv_area_t *b)
{
    return !app_face_area_is_empty(a) && !app_face_area_is_empty(b) &&
           a->x1 <= b->x2 && a->x2 >= b->x1 && a->y1 <= b->y2 && a->y2 >= b->y1;
}

bool app_face_clip_area_to_area(lv_area_t *area, const lv_area_t *clip, int width, int height)
{
    if (app_face_area_is_empty(area) || app_face_area_is_empty(clip)) {
        return false;
    }
    area->x1 = app_face_clamp_int(area->x1, clip->x1, clip->x2);
    area->y1 = app_face_clamp_int(area->y1, clip->y1, clip->y2);
    area->x2 = app_face_clamp_int(area->x2, clip->x1, clip->x2);
    area->y2 = app_face_clamp_int(area->y2, clip->y1, clip->y2);
    return app_face_area_clip_to_bounds(area, width, height);
}

static bool area_near_or_overlaps(const lv_area_t *a, const lv_area_t *b, int gap)
{
    if (app_face_area_is_empty(a) || app_face_area_is_empty(b)) {
        return false;
    }
    return a->x1 <= b->x2 + gap && a->x2 + gap >= b->x1 &&
           a->y1 <= b->y2 + gap && a->y2 + gap >= b->y1;
}

void app_face_dirty_list_reset(app_face_dirty_list_t *list)
{
    list->count = 0;
}

void app_face_dirty_list_add_area(app_face_dirty_list_t *list, lv_area_t area, int width, int height)
{
    if (!app_face_area_clip_to_bounds(&area, width, height)) {
        return;
    }

    for (size_t i = 0; i < list->count; i++) {
        if (area_near_or_overlaps(&list->rects[i], &area, APP_FACE_DIRTY_MERGE_GAP)) {
            app_face_area_include_area(&list->rects[i], &area);
            app_face_area_clip_to_bounds(&list->rects[i], width, height);
            return;
        }
    }

    if (list->count < APP_FACE_MAX_DIRTY_RECTS) {
        list->rects[list->count++] = area;
        return;
    }

    size_t smallest = 0;
    uint32_t smallest_pixels = UINT32_MAX;
    for (size_t i = 0; i < list->count; i++) {
        uint32_t pixels = app_face_area_pixel_count(&list->rects[i]);
        if (pixels < smallest_pixels) {
            smallest_pixels = pixels;
            smallest = i;
        }
    }
    app_face_area_include_area(&list->rects[smallest], &area);
    app_face_area_clip_to_bounds(&list->rects[smallest], width, height);
}

void app_face_dirty_list_add_list(app_face_dirty_list_t *dst,
                                  const app_face_dirty_list_t *src,
                                  int width,
                                  int height)
{
    for (size_t i = 0; i < src->count; i++) {
        app_face_dirty_list_add_area(dst, src->rects[i], width, height);
    }
}

static void fragment_list_add(lv_area_t *fragments, size_t *count, size_t max_count, const lv_area_t *area)
{
    if (app_face_area_is_empty(area) || *count >= max_count) {
        return;
    }
    fragments[(*count)++] = *area;
}

size_t app_face_dirty_subtract_protected_areas(const lv_area_t *area,
                                               const lv_area_t *protected_areas,
                                               size_t protected_area_count,
                                               int protected_margin,
                                               int width,
                                               int height,
                                               lv_area_t *out_fragments,
                                               size_t max_fragments)
{
    if (app_face_area_is_empty(area) || max_fragments == 0) {
        return 0;
    }

    lv_area_t fragments[APP_FACE_MAX_BLIT_FRAGMENTS];
    size_t fragment_count = 1;
    fragments[0] = *area;

    for (size_t p = 0; protected_areas && p < protected_area_count; p++) {
        lv_area_t protect = protected_areas[p];
        app_face_area_inflate_clip(&protect, protected_margin, width, height);
        if (app_face_area_is_empty(&protect)) {
            continue;
        }

        lv_area_t next[APP_FACE_MAX_BLIT_FRAGMENTS];
        size_t next_count = 0;
        for (size_t i = 0; i < fragment_count; i++) {
            lv_area_t src = fragments[i];
            if (!app_face_area_intersects(&src, &protect)) {
                fragment_list_add(next, &next_count, APP_FACE_MAX_BLIT_FRAGMENTS, &src);
                continue;
            }

            lv_area_t top = src;
            top.y2 = protect.y1 - 1;
            fragment_list_add(next, &next_count, APP_FACE_MAX_BLIT_FRAGMENTS, &top);

            lv_area_t bottom = src;
            bottom.y1 = protect.y2 + 1;
            fragment_list_add(next, &next_count, APP_FACE_MAX_BLIT_FRAGMENTS, &bottom);

            lv_area_t left = src;
            left.y1 = fmax(src.y1, protect.y1);
            left.y2 = fmin(src.y2, protect.y2);
            left.x2 = protect.x1 - 1;
            fragment_list_add(next, &next_count, APP_FACE_MAX_BLIT_FRAGMENTS, &left);

            lv_area_t right = src;
            right.y1 = fmax(src.y1, protect.y1);
            right.y2 = fmin(src.y2, protect.y2);
            right.x1 = protect.x2 + 1;
            fragment_list_add(next, &next_count, APP_FACE_MAX_BLIT_FRAGMENTS, &right);
        }
        memcpy(fragments, next, next_count * sizeof(fragments[0]));
        fragment_count = next_count;
        if (fragment_count == 0) {
            break;
        }
    }

    size_t copied = fragment_count < max_fragments ? fragment_count : max_fragments;
    memcpy(out_fragments, fragments, copied * sizeof(out_fragments[0]));
    return copied;
}
