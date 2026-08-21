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
/* ★★★2026-08-17 로터리 A/B 스왑을 보드/빌드 설정으로.
 *  기판마다 엔코더 A/B 배선이 뒤바뀐 개체가 있다(COM3 H2 가 그렇다).
 *  A/B 가 뒤집히면 쿼드러처 방향이 통째로 반대가 된다.
 *    BOARD_ROT_AB_SWAP 0 (기본) : P0=A, P1=B
 *    BOARD_ROT_AB_SWAP 1        : P0=B, P1=A  ← 배선이 뒤바뀐 기판
 *  빌드에서 `-Rot ab|ba` 로도 덮을 수 있다(board_select.h 의 오버라이드).
 *  ※LP 코어를 쓰는 보드는 lp_core/pcf_lp_config.h 의 LP_BIT_ROT_A/B 도
 *    같이 맞춰야 한다 — 어긋나면 button_handler.c 의 _Static_assert 가 잡는다. */
#if BOARD_ROT_AB_SWAP
#  define PCF8574_BIT_ROT_A     1
#  define PCF8574_BIT_ROT_B     0
#else
#  define PCF8574_BIT_ROT_A     0
#  define PCF8574_BIT_ROT_B     1
#endif
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

/* ─── 진동센서 진단 (★2026-08-13 추가) ─────────────────────────────────────
 *  전부 **자유 증가 카운터**다. somfy_app 이 주기적으로 두 시점의 차(델타)를 떠서
 *  NVS 링에 남긴다 — 버튼 태스크(prio 10)에서 NVS 를 만지면 flash cache 정지가
 *  ISR·비트뱅을 깨므로, 여기서는 카운터만 올리고 저장은 somfy_app(prio 4)이 한다.
 *
 *  읽는 법: 한 창에서 poll 이 N 이고 high 가 H 면
 *    H == N  → 계속 HIGH (풀업 상태 그대로 = 접점 안 닫힘)
 *    H == 0  → 계속 LOW  (접점 붙어있음 / GND 단락)
 *    0<H<N   → **섞임 = 실제 접점 동작**(이게 나와야 진동이 잡힌다)
 *  isr 은 에지 수 — 레벨이 안 변해도 이게 늘면 폴링보다 짧은 글리치가 있다는 뜻. */
/**
 * @brief ②(버스트 폴링) 깨우기 출처 통계 — `~INT` 가 실제로 쓸 만한지 판정용.
 *
 * 코드 주석에 "현 HW 는 ~INT 가 불안정" 이라는 경고가 있어, 믿지 말고 **재서**
 * 판단한다. `int_cnt` 비율이 높으면 유휴 주기를 더 늘려도 되고, 0 에 가까우면
 * `~INT` 가 죽은 것이라 안전망 주기가 곧 버튼 반응 지연이 된다.
 */
/* ★2026-08-20 절전 진단: LP 코어 폴링이 실제로 붙었는지 / 현재 폴 주기(ms). */
bool btn_handler_lp_active(void);
void btn_handler_lp_counters(uint32_t *poll_cnt, uint32_t *seq);
int  btn_handler_poll_ms(void);

void btn_handler_wake_stats(uint32_t *int_cnt, uint32_t *vibe_cnt,
                            uint32_t *tmo_cnt, uint32_t *idle_cnt);

/**
 * @brief `~INT`(PCF8574 인터럽트 선) 관찰 진단 요청 — 콘솔 `intdiag`.
 *
 * 폴링을 **멈춘 구간(A)** 과 **돌리는 구간(B)** 을 나눠 GPIO 레벨/전이를 센다.
 * A 가 조용하고 B 만 시끄러우면 원인은 우리 자신의 I2C 폴링이고, 폴링을 멈추는
 * ②(`~INT` 기반 버스트 폴링)가 성립한다. A 도 시끄러우면 배선/보드 문제다.
 * 버튼을 **누른 채** 실행하면 A 구간에서 `~INT` 가 LOW 로 떨어지는지도 볼 수 있다.
 *
 * ※실제 관찰은 버튼 태스크가 수행하며, 그동안(약 4초) 버튼 입력이 멈춘다.
 */
void btn_handler_int_diag_request(void);

uint32_t btn_handler_vibe_isr_count(void);   /**< ANY_EDGE ISR 발생 누적 */
uint32_t btn_handler_vibe_poll_total(void);  /**< 진동핀 폴링 누적 */
uint32_t btn_handler_vibe_high_total(void);  /**< 그중 HIGH 로 읽힌 누적 */
int      btn_handler_vibe_level(void);       /**< 지금 핀 레벨(0/1) */
bool     btn_handler_vibe_stuck(void);       /**< 고장(한쪽 고정) 판정 상태 */

#ifdef __cplusplus
}
#endif
