#pragma once
/* sim 빌드용 보드 정의 — 실제 boards/board_select.h 대체(-I sim/wasm/inc 우선).
 *  기본 128×64 가로 패널. 72×40 테스트하려면 WIDTH/HEIGHT 만 바꿔 재빌드. */
#define BOARD_OLED_WIDTH        128
#define BOARD_OLED_HEIGHT       64
#define BOARD_OLED_ROTATE_90    0
#define BOARD_OLED_ROTATE_180   0   /* 회전은 웹에서 CSS 로 처리 → FB 는 0 */
#define BOARD_OLED_FLIP_X       0
#define BOARD_OLED_FIXUP_72X40  0
#define BOARD_OLED_ADDR         0x3C
#define BOARD_PIN_OLED_SDA      0
#define BOARD_PIN_OLED_SCL      0
#define BOARD_PIN_OLED_RST      (-1)
