#pragma once
#include <stdint.h>
#include <stdbool.h>
/* sim: somfy_rts.h 가 include 하지만 oled_ui 는 RF 를 안 씀 → 불투명 타입만. */
typedef struct cc1101_s { int _dummy; } cc1101_t;
