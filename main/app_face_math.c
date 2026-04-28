#include "app_face_math.h"

#include <math.h>

float app_face_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float app_face_lerpf(float from, float to, float t)
{
    return from + (to - from) * t;
}

float app_face_smoothf(float t)
{
    float x = app_face_clampf(t, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

float app_face_min_jerk(float t)
{
    float x = app_face_clampf(t, 0.0f, 1.0f);
    return x * x * x * (10.0f + x * (-15.0f + x * 6.0f));
}

float app_face_smooth_range(float edge0, float edge1, float value)
{
    if (fabsf(edge1 - edge0) < 0.0001f) {
        return value >= edge1 ? 1.0f : 0.0f;
    }
    return app_face_smoothf((value - edge0) / (edge1 - edge0));
}

float app_face_wrap_unit(float value)
{
    while (value > 1.0f) {
        value -= 2.0f;
    }
    while (value < -1.0f) {
        value += 2.0f;
    }
    return value;
}

float app_face_shortest_unit_delta(float from, float to)
{
    return app_face_wrap_unit(to - from);
}

static app_face_vec2_t clamp_basis(float x,
                                   float y,
                                   float min_len,
                                   float max_len,
                                   float fallback_x,
                                   float fallback_y,
                                   float *raw_len_out)
{
    float len = sqrtf(x * x + y * y);
    if (len < 0.0001f) {
        x = fallback_x;
        y = fallback_y;
        len = sqrtf(x * x + y * y);
        if (len < 0.0001f) {
            x = 1.0f;
            y = 0.0f;
            len = 1.0f;
        }
    }
    if (raw_len_out) {
        *raw_len_out = len;
    }
    float target = app_face_clampf(len, min_len, max_len);
    return (app_face_vec2_t){
        .x = (x / len) * target,
        .y = (y / len) * target,
    };
}

app_face_vec3_t app_face_rotate3d(app_face_vec3_t point, app_face_orientation_t orientation)
{
    float x = point.x;
    float y = point.y;
    float z = point.z;

    float c = cosf(orientation.yaw);
    float s = sinf(orientation.yaw);
    float nx = x * c + z * s;
    float nz = -x * s + z * c;
    x = nx;
    z = nz;

    c = cosf(orientation.pitch);
    s = sinf(orientation.pitch);
    float ny = y * c - z * s;
    nz = y * s + z * c;
    y = ny;
    z = nz;

    c = cosf(orientation.roll);
    s = sinf(orientation.roll);
    nx = x * c - y * s;
    ny = x * s + y * c;

    return (app_face_vec3_t){.x = nx, .y = ny, .z = z};
}

app_face_projected_point_t app_face_sphere_point(float local_x,
                                                 float local_y,
                                                 float radius,
                                                 app_face_orientation_t orientation,
                                                 float clamp_radius,
                                                 float camera_distance,
                                                 float perspective_strength)
{
    float limit = radius * clamp_radius;
    float local_radius = sqrtf(local_x * local_x + local_y * local_y);
    float local_scale = local_radius > limit && local_radius > 0.0001f ? limit / local_radius : 1.0f;
    float x = local_x * local_scale;
    float y = local_y * local_scale;
    float z = sqrtf(fmaxf(1.0f, radius * radius - x * x - y * y));
    app_face_vec3_t rotated = app_face_rotate3d((app_face_vec3_t){.x = x, .y = y, .z = z}, orientation);
    float camera = radius * camera_distance;
    float denom = camera - rotated.z * perspective_strength;
    float perspective = fabsf(denom) > 0.0001f ? camera / denom : 1.0f;

    return (app_face_projected_point_t){
        .x = rotated.x * perspective,
        .y = rotated.y * perspective,
        .z = rotated.z / radius,
        .perspective = perspective,
        .was_clamped = local_scale < 0.999f,
    };
}

app_face_surface_t app_face_eye_surface(float local_x,
                                        float local_y,
                                        float eye_w,
                                        float eye_h,
                                        float radius,
                                        float display_radius,
                                        app_face_orientation_t orientation)
{
    const float clamp_radius = 0.985f;
    const float camera_distance = 4.8f;
    const float perspective_strength = 0.42f;
    const float min_basis_x = 0.22f;
    const float min_basis_y = 0.34f;
    const float max_basis = 1.28f;

    app_face_projected_point_t center = app_face_sphere_point(
        local_x, local_y, radius, orientation, clamp_radius, camera_distance, perspective_strength);
    app_face_projected_point_t left = app_face_sphere_point(
        local_x - eye_w * 0.5f, local_y, radius, orientation, clamp_radius, camera_distance, perspective_strength);
    app_face_projected_point_t right = app_face_sphere_point(
        local_x + eye_w * 0.5f, local_y, radius, orientation, clamp_radius, camera_distance, perspective_strength);
    app_face_projected_point_t top = app_face_sphere_point(
        local_x, local_y - eye_h * 0.5f, radius, orientation, clamp_radius, camera_distance, perspective_strength);
    app_face_projected_point_t bottom = app_face_sphere_point(
        local_x, local_y + eye_h * 0.5f, radius, orientation, clamp_radius, camera_distance, perspective_strength);

    float raw_ux = (right.x - left.x) / fmaxf(1.0f, eye_w);
    float raw_uy = (right.y - left.y) / fmaxf(1.0f, eye_w);
    float raw_vx = (bottom.x - top.x) / fmaxf(1.0f, eye_h);
    float raw_vy = (bottom.y - top.y) / fmaxf(1.0f, eye_h);
    float raw_len_x = 1.0f;
    float raw_len_y = 1.0f;
    app_face_vec2_t u = clamp_basis(raw_ux, raw_uy, min_basis_x, max_basis, raw_vy, -raw_vx, &raw_len_x);
    app_face_vec2_t v = clamp_basis(raw_vx, raw_vy, min_basis_y, max_basis, -u.y, u.x, &raw_len_y);

    float raw_det = raw_ux * raw_vy - raw_uy * raw_vx;
    float center_dist = sqrtf(center.x * center.x + center.y * center.y);
    float radial_x = center_dist > 0.0001f ? center.x / center_dist : 0.0f;
    float radial_y = center_dist > 0.0001f ? center.y / center_dist : 0.0f;
    float radial_half = (fabsf((u.x * radial_x + u.y * radial_y) * eye_w * 0.5f) +
                         fabsf((v.x * radial_x + v.y * radial_y) * eye_h * 0.5f)) *
                        1.35f;
    float rim_clearance = display_radius * 0.965f - center_dist - radial_half;
    float det_visibility = app_face_smooth_range(0.035f, 0.18f, raw_det);
    float depth_visibility = app_face_smooth_range(0.08f, 0.34f, center.z);
    float rim_visibility = app_face_smooth_range(-10.0f, 2.0f, rim_clearance);
    float clamp_visibility = (left.was_clamped || right.was_clamped || top.was_clamped || bottom.was_clamped || center.was_clamped)
                                  ? 0.72f
                                  : 1.0f;

    return (app_face_surface_t){
        .x = center.x,
        .y = center.y,
        .z = center.z,
        .ux = u.x,
        .uy = u.y,
        .vx = v.x,
        .vy = v.y,
        .scale_x = raw_len_x,
        .scale_y = raw_len_y,
        .raw_det = raw_det,
        .visibility = app_face_clampf(det_visibility * depth_visibility * rim_visibility * clamp_visibility, 0.0f, 1.0f),
    };
}
