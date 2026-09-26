#pragma once
#define portMAX_DELAY (-1)
extern unsigned test_yields;
#define portYIELD_FROM_ISR() (++test_yields)
