#pragma once
#include "driver/gpio.h"
#include "esp_err.h"        // esp_err_t
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "somfy_config.h"   // CFG_PCF8574_INT, CFG_CHG_STAT_PIN, CFG_VIBE_PIN
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─── 핀 정의 (PCB v2.0 — 핀 재배치) ──────────────────────────────
 *  GPIO 직결 (light sleep wake source):
 *    IO17 → PCF8574 ~INT  (open-drain, 외부 10kΩ pull-up) — 구 IO2
 *    IO3  → MCP73831 STAT (active-LOW, 충전 상태, wake 가능)
 *    IO16 → VS1 진동센서   (active-LOW, wake 가능)
 *    IO2  → 예약 (미사용)
 *
 *  PCF8574 (I2C 0x20) — IO18/IO19 (bit-bang I2C) — 8개 P 핀 사용:
 *    P0 = ROT_A           P4 = SW1 BTN_UP
 *    P1 = ROT_B           P5 = SW2 BTN_DOWN
 *    P2 = ROT_BTN         P6 = SW3 BTN_SELECT
 *    P3 = SW6 SETUP       P7 = SW4 BTN_PROG
 *
 *  IO0/IO1 = HP I2C0 SDA/SCL (OLED SSD1315 전용, 내부 배선)
 *
 *  v2.0 변경: PCF8574 I2C 가 LP_GPIO6/7 (LP_I2C0) → IO18/IO19 (bit-bang)
 *    이유: LP_GPIO 핀 제약 해제, PCB 라우팅 자유도 ↑.
 *    HP_I2C0 는 OLED 전용 (1 컨트롤러 제약) → PCF8574 는 SW bit-bang.
 * ───────────────────────────────────────────────────────────────── */
#define PCF8574_INT_PIN  ((gpio_num_t)CFG_PCF8574_INT)  // IO17
#define CHG_STAT_PIN     ((gpio_num_t)CFG_CHG_STAT_PIN) // IO3
#define VIBE_PIN         ((gpio_num_t)CFG_VIBE_PIN)     // IO16

/* ─── PCF8574 bit-bang I2C 설정 ─────────────── */
#define PCF8574_I2C_SDA  ((gpio_num_t)CFG_PCF8574_SDA)  // IO19
#define PCF8574_I2C_SCL  ((gpio_num_t)CFG_PCF8574_SCL)  // IO18
#define PCF8574_I2C_ADDR 0x20
/* Backward-compatibility — 일부 코드는 LP_ 접두 사용 (실제로는 bit-bang) */
#define PCF8574_LP_I2C_SDA PCF8574_I2C_SDA
#define PCF8574_LP_I2C_SCL PCF8574_I2C_SCL

/* PCF8574 비트맵 (active-LOW, 내부 pull-up) */
#define PCF8574_BIT_ROT_A     0
#define PCF8574_BIT_ROT_B     1
#define PCF8574_BIT_ROT_BTN   2
#define PCF8574_BIT_SETUP_BTN 3
#define PCF8574_BIT_BTN_UP    4
#define PCF8574_BIT_BTN_DOWN  5
#define PCF8574_BIT_BTN_SEL   6
#define PCF8574_BIT_BTN_PROG  7

/* ─── 확장: 좌/우 버튼 (PCF8575 16비트) ──────────────────────────────
 *  BOARD_HAS_LR_BUTTONS=1 → PCF8575 사용(P10=LEFT, P11=RIGHT), read 2바이트.
 *  =0 → 기존 PCF8574(8비트). 주소·~INT·동작은 동일(PCF8575 는 16비트 형제). */
#if BOARD_HAS_LR_BUTTONS
typedef uint16_t pcf_state_t;
#define PCF_NBYTES            2
#define PCF8574_BIT_BTN_LEFT  8   /* PCF8575 P10 */
#define PCF8574_BIT_BTN_RIGHT 9   /* PCF8575 P11 */
#else
typedef uint8_t  pcf_state_t;
#define PCF_NBYTES            1
#endif

/* ─── 버튼 이벤트 타입 ───────────────────────── */
typedef enum {
  BTN_EVT_NONE = 0,
  BTN_EVT_UP_PRESS = 1,       // UP 버튼 누름
  BTN_EVT_UP_RELEASE = 2,     // UP 버튼 뗌
  BTN_EVT_DOWN_PRESS = 3,     // DOWN 버튼 누름
  BTN_EVT_DOWN_RELEASE = 4,   // DOWN 버튼 뗌
  BTN_EVT_SELECT_PRESS = 5,   // SELECT 버튼 누름
  BTN_EVT_SELECT_RELEASE = 6, // SELECT 버튼 뗌
  BTN_EVT_PROG_PRESS = 7,     // PROG 버튼 누름
  BTN_EVT_PROG_RELEASE = 8,   // PROG 버튼 뗌
  BTN_EVT_ROT_CW = 9,         // 로터리 시계방향
  BTN_EVT_ROT_CCW = 10,       // 로터리 반시계방향
  BTN_EVT_ROT_CLICK = 11,     // 로터리 짧은 클릭 (정지/MY 커맨드)
  BTN_EVT_ROT_LONG = 12,      // 로터리 길게 클릭 (주파수 편집 모드)
  BTN_EVT_SETUP_SHORT = 13,   // SW6 SETUP 짧은 누름
  BTN_EVT_SETUP_LONG = 14,    // SW6 SETUP 5초 롱프레스 (WiFi 페어링)
  BTN_EVT_ROT_PRESS = 15,     // 로터리 버튼(STOP/MY) 누름 — 송신 시작
  BTN_EVT_LEFT_PRESS = 16,    // 좌 버튼 누름 (PCF8575 P10, BOARD_HAS_LR_BUTTONS=1)
  BTN_EVT_LEFT_RELEASE = 17,  // 좌 버튼 뗌
  BTN_EVT_RIGHT_PRESS = 18,   // 우 버튼 누름 (PCF8575 P11)
  BTN_EVT_RIGHT_RELEASE = 19, // 우 버튼 뗌
} btn_event_t;

/* ─── 버튼 누름 시간 정보 ───────────────────────
   press → release 시 hold_ms에 누름 시간 기록
────────────────────────────────────────────── */
typedef struct {
  btn_event_t type;
  uint32_t hold_ms; // 버튼을 누른 시간 (ms), release 이벤트에만 유효
} btn_event_data_t;

/* 호환성: 일부 코드가 BTN_PIN_UP/DOWN 으로 _hold_repeat_task 에서
 * "현재 UP/DOWN 눌림 중인지" 판정 — 이제는 PCF8574 비트로 매핑.
 * btn_is_pressed() 가 이 값을 키로 받아 PCF8574 캐시를 조회한다. */
typedef enum {
  BTN_KEY_UP = PCF8574_BIT_BTN_UP,
  BTN_KEY_DOWN = PCF8574_BIT_BTN_DOWN,
  BTN_KEY_SELECT = PCF8574_BIT_BTN_SEL,
  BTN_KEY_PROG = PCF8574_BIT_BTN_PROG,
  BTN_KEY_ROT = PCF8574_BIT_ROT_BTN,   /* 동시작동(MY+UP/MY+DOWN) 감지용 — MY=로터리 클릭 */
#if BOARD_HAS_LR_BUTTONS
  BTN_KEY_LEFT = PCF8574_BIT_BTN_LEFT,
  BTN_KEY_RIGHT = PCF8574_BIT_BTN_RIGHT,
#endif
} btn_key_t;

/* ─── 이벤트 콜백 타입 ──────────────────────── */
typedef void (*btn_event_cb_t)(btn_event_data_t *evt, void *user_data);

/* ─── API ────────────────────────────────────── */

/**
 * @brief 버튼 핸들러 초기화
 *        oled_ui_init() 이후에 호출해야 I2C 버스가 준비됨
 * @param cb        이벤트 콜백 함수
 * @param user_data 콜백에 전달할 사용자 데이터
 */
void btn_handler_init(btn_event_cb_t cb, void *user_data);

/**
 * @brief 버튼 폴링 태스크 시작
 */
void btn_handler_start_task(void);

/**
 * @brief 특정 버튼 눌림 여부 (PCF8574 캐시값 기반, lock-free).
 *        btn_key_t 값을 받음. _hold_repeat_task 등에서 사용.
 */
bool btn_is_pressed(btn_key_t key);

/**
 * @brief MCP73831 STAT pin (IO3) 검사 — active-LOW, 충전 중이면 true.
 */
bool btn_handler_is_charging(void);

/**
 * @brief 진동 wake 플래그 — main.c 에서 light_sleep wake 직후 검사.
 *        VS1 가 IO16 GPIO 직결로 변경됨에 따라 light sleep 중에도
 *        gpio_wakeup_enable(IO16, LOW) 로 진동만으로 wake 가능.
 */
bool btn_handler_was_vibration_wake(void);

/**
 * @brief 진동 wake 플래그 클리어.
 */
void btn_handler_clear_vibration_wake(void);

/**
 * @brief 진동 wake 플래그 설정 (sleep wake 후 main.c 에서 호출).
 */
void btn_handler_set_vibration_wake(void);

/**
 * @brief 현재 진동 센서 상태 (IO16 직접 read).
 */
bool btn_handler_is_vibrating(void);

/**
 * @brief 마지막 진동 감지 시각 (esp_timer_get_time(), µs).
 */
int64_t btn_handler_last_vibration_us(void);

/**
 * @brief I2C 공유 뮤텍스 핸들 반환
 */
SemaphoreHandle_t btn_handler_get_i2c_mutex(void);

#ifdef __cplusplus
}
#endif
