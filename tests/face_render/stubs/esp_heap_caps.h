#pragma once
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_8BIT 2
#define MALLOC_CAP_DMA 4
#define MALLOC_CAP_INTERNAL 8
static inline void *heap_caps_malloc(size_t n, unsigned caps) { (void)caps; return malloc(n); }
#define heap_caps_free free
