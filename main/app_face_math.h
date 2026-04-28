#pragma once

#include <stdbool.h>
#include <stdint.h>

#define APP_FACE_TAU 6.2831853071795864769f
#define APP_FACE_PI 3.14159265358979323846f

typedef struct {
    float x;
    float y;
} app_face_vec2_t;

typedef struct {
    float x;
    float y;
    float z;
} app_face_vec3_t;

typedef struct {
    float yaw;
    float pitch;
    float roll;
} app_face_orientation_t;

typedef struct {
    float x;
    float y;
    float z;
    float perspective;
    bool was_clamped;
} app_face_projected_point_t;

typedef struct {
    float x;
    float y;
    float z;
    float ux;
    float uy;
    float vx;
    float vy;
    float scale_x;
    float scale_y;
    float raw_det;
    float visibility;
} app_face_surface_t;

float app_face_clampf(float value, float min_value, float max_value);
float app_face_lerpf(float from, float to, float t);
float app_face_smoothf(float t);
float app_face_min_jerk(float t);
float app_face_smooth_range(float edge0, float edge1, float value);
float app_face_wrap_unit(float value);
float app_face_shortest_unit_delta(float from, float to);

app_face_vec3_t app_face_rotate3d(app_face_vec3_t point, app_face_orientation_t orientation);
app_face_projected_point_t app_face_sphere_point(float local_x,
                                                 float local_y,
                                                 float radius,
                                                 app_face_orientation_t orientation,
                                                 float clamp_radius,
                                                 float camera_distance,
                                                 float perspective_strength);
app_face_surface_t app_face_eye_surface(float local_x,
                                        float local_y,
                                        float eye_w,
                                        float eye_h,
                                        float radius,
                                        float display_radius,
                                        app_face_orientation_t orientation);
