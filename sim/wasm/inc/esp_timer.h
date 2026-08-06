#pragma once
#include <stdint.h>
/* sim: glue.c 가 emscripten_get_now() 로 구현 */
int64_t esp_timer_get_time(void);
