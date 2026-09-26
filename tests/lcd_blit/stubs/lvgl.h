#pragma once
#include <stdint.h>
typedef int16_t lv_coord_t;
typedef struct { lv_coord_t x1, y1, x2, y2; } lv_area_t;
typedef union { uint16_t full; } lv_color_t;
