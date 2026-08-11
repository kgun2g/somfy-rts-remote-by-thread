#include "button_handler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"       // esp_rom_delay_us — bit-bang timing
#include "esp_task_wdt.h"      // Task WDT — _btn_task hang 자동 리부트
#include "esp_system.h"        // esp_restart — SETUP 15초 hold 강제 재부팅
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "somfy_config.h"      // CFG_BTN_DEBOUNCE_MS, CFG_BTN_LONG_PRESS_MS
#include "driver/gpio.h"
#include "hal/gpio_ll.h"       // ★ IRAM 인라인 GPIO LL — ISR 안전 (gpio_intr_disable 은 flash 거주라 NVS write 중 cache error 유발)
#include "soc/gpio_struct.h"   // GPIO 구조체 (gpio_ll_intr_disable 인자)
#include <string.h>

/* BOARD_I2C_SHARED(예: ESP32-H2): OLED 와 PCF8574 가 하드웨어 I2C 한 버스를 공유.
 *   PCF8574 는 비트뱅 대신 OLED 가 만든 i2c_master 버스에 device 로 붙어 폴링한다.
 *   (LP_I2C 없고 핀이 부족한 보드용 — 자세한 설명은 boards/esp32-h2.h.) */
#if BOARD_I2C_SHARED
#include "driver/i2c_master.h"
#include "oled_ui.h"           // oled_ui_get_i2c_bus()
#endif

/* somfy_app.c 가 노출하는 breadcrumb 갱신 콜백 (RTC 메모리에 tick 기록).
 *  silent hang 시 다음 boot 의 reset_reason=TASK_WDT 와 함께 마지막 tick
 *  타임스탬프를 덤프해 어느 task 가 어디서 멈췄는지 추정 가능. */
extern void somfy_app_bc_btn_tick(void);

static const char *TAG = "BTN";

/* ─── 내부 helper 전방 선언 ──────────────────────────── */
static void _vibration_track(bool vibe_active);
static void IRAM_ATTR _vibe_isr_handler(void *arg);
/* 진동 ISR 카운터 / disable 플래그 — 정의는 아래 진동 섹션 */
static volatile uint32_t s_vibe_isr_count;
static volatile bool     s_vibe_isr_disabled_flag;

/* ─── 디바운스 / 롱프레스 설정 ───────────────── */
#define DEBOUNCE_MS CFG_BTN_DEBOUNCE_MS
#define LONG_PRESS_MS CFG_BTN_LONG_PRESS_MS
#define ROT_MIN_INTERVAL_MS 30 // 로터리 최소 이벤트 간격 (ms)
/* ★ SETUP 롱프레스 임계값 2000→1000ms. 2초는 체감상 너무 길어 "길게
 *  눌렀는데 인식 안 됨"으로 느껴졌다. 1초면 일반 탭(~0.1~0.3초)과 명확히
 *  구분되면서 길게 누름이 잘 인식된다. */
#define SETUP_LONG_PRESS_MS 1000
/* SETUP 을 이 시간 이상 계속 누르면 강제 재부팅 (기기 멈춤/응답불가 대비).
 *  롱프레스(1s)·메뉴 동작과 무관하게, 누름 유지 시간만으로 판정 — 화면 상태 불문. */
#define SETUP_REBOOT_HOLD_MS 15000

/* ─── PCF8574 bit-bang I2C 타이밍 (~50kHz, 안전 마진) ─────────────
 * v2.0: HP_I2C0 는 OLED 가 점유, LP_GPIO 핀 변경으로 LP_I2C 사용 불가
 *       → IO18/IO19 GPIO 비트뱅으로 PCF8574 통신.
 *       PCF8574 는 표준 100kHz 까지 지원, 50kHz 면 충분히 안정.
 * ────────────────────────────────────────────────────────────── */
#define I2C_BB_HALF_PERIOD_US 10  // SCL HIGH 또는 LOW 시간 (50kHz)

static SemaphoreHandle_t s_i2c_mutex = NULL;

/* ── XIAO: LP_I2C → 공유 HW I2C 런타임 자동 폴백 ────────────────────────────
 *  BOARD_I2C_LP_FALLBACK(=1, XIAO): 부팅 시 LP_I2C 전용핀(6/7) 비트뱅으로 PCF8574 를
 *  먼저 프로브 → 무응답이면 공유 HW I2C(OLED 버스, 22/23)로 전환. 한 펌웨어가 두 배선 지원.
 *  비-폴백 보드(GNPE 비트뱅 / H2 공유)는 매크로가 같은 이름으로 전개돼 기존 단일 경로 그대로. */
#if BOARD_I2C_LP_FALLBACK
#  define PCF_BB_SDA      ((gpio_num_t)BOARD_PIN_PCF_LP_SDA)
#  define PCF_BB_SCL      ((gpio_num_t)BOARD_PIN_PCF_LP_SCL)
#  define PCF_RD_SHARED   _pcf_read_shared   /* 폴백: 공유/LP 읽기 함수 이름 분리 */
#  define PCF_RD_BB       _pcf_read_lp
enum { PCF_BUS_LP = 0, PCF_BUS_SHARED = 1 };
static volatile uint8_t s_pcf_bus = PCF_BUS_LP;   /* btn_handler_init 의 프로브로 확정 */
#else
#  define PCF_BB_SDA      PCF8574_I2C_SDA
#  define PCF_BB_SCL      PCF8574_I2C_SCL
#  define PCF_RD_SHARED   _pcf8574_read          /* 비-폴백: 단일 _pcf8574_read */
#  define PCF_RD_BB       _pcf8574_read
#endif

/* ─── PCF8574 비트 → 이벤트 매핑 ────────────────────────────────
 * UP/DOWN/SELECT/PROG/SETUP 모두 PCF8574 비트로 폴링.
 * SETUP/PROG 는 롱프레스도 발생시킴.
 * ────────────────────────────────────────────────────────────── */
typedef struct {
  uint8_t bit;            // PCF8574 비트 위치
  bool last_state;        // true=HIGH(released)
  bool raw_state;
  uint32_t press_time_ms;
  uint32_t debounce_ms;
  bool long_fired;
  uint32_t long_press_ms; // 0 = 롱프레스 비활성
  btn_event_t press_evt;
  btn_event_t release_evt;
  btn_event_t long_evt;   // BTN_EVT_NONE 이면 롱프레스 비활성
} btn_state_t;

static btn_state_t s_btns[] = {
    {PCF8574_BIT_BTN_UP,   true, true, 0, 0, false,
     0,                  BTN_EVT_UP_PRESS,    BTN_EVT_UP_RELEASE,    BTN_EVT_NONE},
    {PCF8574_BIT_BTN_DOWN, true, true, 0, 0, false,
     0,                  BTN_EVT_DOWN_PRESS,  BTN_EVT_DOWN_RELEASE,  BTN_EVT_NONE},
    {PCF8574_BIT_BTN_SEL,  true, true, 0, 0, false,
     0,                  BTN_EVT_SELECT_PRESS,BTN_EVT_SELECT_RELEASE,BTN_EVT_NONE},
    {PCF8574_BIT_BTN_PROG, true, true, 0, 0, false,
     LONG_PRESS_MS,      BTN_EVT_PROG_PRESS,  BTN_EVT_PROG_RELEASE,  BTN_EVT_PROG_PRESS},
    {PCF8574_BIT_SETUP_BTN,true, true, 0, 0, false,
     SETUP_LONG_PRESS_MS,BTN_EVT_NONE,        BTN_EVT_SETUP_SHORT,   BTN_EVT_SETUP_LONG},
#if BOARD_HAS_LR_BUTTONS
    {PCF8574_BIT_BTN_LEFT, true, true, 0, 0, false,
     0,                  BTN_EVT_LEFT_PRESS,  BTN_EVT_LEFT_RELEASE,  BTN_EVT_NONE},
    {PCF8574_BIT_BTN_RIGHT,true, true, 0, 0, false,
     0,                  BTN_EVT_RIGHT_PRESS, BTN_EVT_RIGHT_RELEASE, BTN_EVT_NONE},
#endif
};
#define BTN_COUNT (sizeof(s_btns) / sizeof(s_btns[0]))

/* ─── 로터리 엔코더 상태 (PCF8574 P0/P1/P2) ───── */
static uint32_t s_rot_last_ms = 0;
static uint32_t s_rot_btn_press_ms = 0;
static bool s_rot_btn_state = true;
static bool s_rot_btn_long_fired = false;
static volatile pcf_state_t s_pcf_last = (pcf_state_t)0xFFFFu;

/* ★ v3.14: detent 잠금 + 이탈 첫엣지 방향결정 디코더.
 *  배경: 이 인코더+10ms 비트뱅 폴링은 quadrature 를 언더샘플해 한 클릭에
 *  이탈엣지(3→1=+1)와 복귀엣지(1→3=-1)가 둘 다 나가거나, 잡히는 중간값
 *  (1 vs 2)이 달라 방향이 뒤죽박죽 → "이상함". 해결:
 *   - rest(ab=3) 에서 처음 벗어나는 안정 ab(1 또는 2)로 방향 1회 결정
 *   - 그 후 rest(3) 로 안정 복귀할 때까지 잠금 → 복귀엣지·바운스 전부 무시
 *   - 2-연속표본 디바운스로 서브폴 글리치(유령) 제거
 *  ⇒ 클릭당 정확히 1 이벤트, 방향 결정적. (3→1=CW/tiltUP, 3→2=CCW/tiltDN
 *     — 반대로 느껴지면 _ROT_CW_ON_AB1 만 뒤집으면 됨) */
#define _ROT_CW_ON_AB1  1               /* 3→1 을 CW 로 (0 이면 반전) */
static uint8_t s_rot_ab_stable = 0x03;  /* 디바운스 통과한 마지막 A/B */
static uint8_t s_rot_ab_raw    = 0x03;  /* 직전 폴 원시 A/B */
static bool    s_rot_armed     = true;  /* true=새 클릭 감지 준비(rest 상태) */
#if BOARD_ROT_HALF_STEP
static int8_t  s_rot_accum     = 0;     /* 하프스텝 그레이코드 누산기 (±2 = 1디텐트) */
#endif

/* ─── 이벤트 콜백 ────────────────────────────── */
static btn_event_cb_t s_callback = NULL;
static void *s_user_data = NULL;

static inline uint32_t _ms_now(void) {
  return (uint32_t)(esp_timer_get_time() / 1000);
}

static void _send_event(btn_event_t type, uint32_t hold_ms) {
  btn_event_data_t evt = {.type = type, .hold_ms = hold_ms};
  if (s_callback) {
    s_callback(&evt, s_user_data);
  }
}

/* ─── PCF8574 bit-bang I2C 드라이버 (v2.0) ─────────────────────────
 *  배경: HP_I2C0 는 OLED 전용 (IO0/1), LP_I2C0 는 LP_GPIO6/7 고정 핀
 *        제약으로 IO18/19 에서 사용 불가. 따라서 PCF8574 통신은
 *        소프트웨어 비트뱅. PCF8574 는 idle 폴링만 (~10ms 주기) 이라
 *        50kHz 비트뱅으로 충분.
 *
 *  GPIO 모드: open-drain 시뮬레이션 — output LOW 또는 input(외부 pull-up).
 * ──────────────────────────────────────────────────────────────── */
static volatile bool s_pcf_present = false;
static volatile bool s_pcf_initialized = false;

#if (!BOARD_I2C_SHARED) || BOARD_I2C_LP_FALLBACK
static inline void _sda_low(void) {
  /* output 강제 LOW */
  gpio_set_direction(PCF_BB_SDA, GPIO_MODE_OUTPUT);
  gpio_set_level(PCF_BB_SDA, 0);
}
static inline void _sda_release(void) {
  /* input → pull-up 으로 HIGH (open-drain release) */
  gpio_set_direction(PCF_BB_SDA, GPIO_MODE_INPUT);
}
static inline void _scl_low(void) {
  gpio_set_direction(PCF_BB_SCL, GPIO_MODE_OUTPUT);
  gpio_set_level(PCF_BB_SCL, 0);
}
static inline void _scl_release(void) {
  gpio_set_direction(PCF_BB_SCL, GPIO_MODE_INPUT);
}
static inline int _sda_read(void) {
  return gpio_get_level(PCF_BB_SDA);
}
static inline void _i2c_delay(void) {
  esp_rom_delay_us(I2C_BB_HALF_PERIOD_US);
}

static void _i2c_start(void) {
  _sda_release(); _scl_release(); _i2c_delay();
  _sda_low();     _i2c_delay();
  _scl_low();     _i2c_delay();
}

static void _i2c_stop(void) {
  _sda_low();     _i2c_delay();
  _scl_release(); _i2c_delay();
  _sda_release(); _i2c_delay();
}

/* 1 비트 송신. ACK 받기는 별도. */
static void _i2c_write_bit(int bit) {
  if (bit) _sda_release(); else _sda_low();
  _i2c_delay();
  _scl_release(); _i2c_delay();
  _scl_low();     _i2c_delay();
}

static int _i2c_read_bit(void) {
  _sda_release(); _i2c_delay();
  _scl_release(); _i2c_delay();
  int b = _sda_read();
  _scl_low();     _i2c_delay();
  return b;
}

/* byte write → returns 0 if ACK, 1 if NACK */
static int _i2c_write_byte(uint8_t b) {
  for (int i = 7; i >= 0; i--) _i2c_write_bit((b >> i) & 1);
  return _i2c_read_bit(); // ACK bit
}

/* byte read. ack=0 → send ACK (more bytes coming), ack=1 → NACK (last byte) */
static uint8_t _i2c_read_byte(int nack) {
  uint8_t b = 0;
  for (int i = 7; i >= 0; i--) b = (b << 1) | _i2c_read_bit();
  _i2c_write_bit(nack);
  return b;
}
#endif /* bit-bang 헬퍼: (!BOARD_I2C_SHARED) || BOARD_I2C_LP_FALLBACK */

#if BOARD_I2C_SHARED
/* ── 공유 HW I2C: PCF8574 = OLED 와 같은 i2c_master 버스의 device ──────── */
static i2c_master_dev_handle_t s_pcf_dev = NULL;

static esp_err_t _pcf_i2c_shared_init(void) {
  i2c_master_bus_handle_t bus = oled_ui_get_i2c_bus();   /* OLED 가 만든 버스 */
  if (bus == NULL) {
    ESP_LOGE(TAG, "공유 I2C: OLED 버스 핸들 NULL — oled_ui_init() 이 먼저 호출돼야 함");
    return ESP_ERR_INVALID_STATE;
  }
  /* ── [임시 진단] I2C 버스 스캔 — PCF8574 실제 주소 확인용 ──────────────
   *   버튼 미동작 진단: OLED(0x3C)는 응답하는데 PCF8574(0x20)가 NACK 이면
   *   주소 스트랩(A0~A2) 불일치 또는 PCF8574A(0x38~0x3F) 변종 가능성.
   *   ACK 하는 모든 주소를 찍어 PCF 의 실제 주소를 특정한다. (진단 후 제거) */
  {
    int found = 0;
    ESP_LOGW(TAG, "[I2C SCAN] 시작 (0x08~0x77) — 버튼 PCF8574 주소 확인");
    for (uint8_t a = 0x08; a <= 0x77; a++) {
      if (i2c_master_probe(bus, a, 50) == ESP_OK) {
        const char *what = (a == 0x3C || a == 0x3D) ? "OLED?"
                         : (a >= 0x20 && a <= 0x27) ? "PCF8574?"
                         : (a >= 0x38 && a <= 0x3F) ? "PCF8574A?" : "";
        ESP_LOGW(TAG, "[I2C SCAN] ACK @ 0x%02X  %s", a, what);
        found++;
      }
    }
    ESP_LOGW(TAG, "[I2C SCAN] 끝 — %d개 응답 (PCF 가 0x20 아니면 위 주소로 strap/주소수정)", found);
  }
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address  = PCF8574_I2C_ADDR,
      .scl_speed_hz    = 400000,   /* ★ OLED(ssd1306)와 동일 400kHz. 같은 HW I2C 버스에서
                                    *   device 마다 클럭이 다르면(OLED 400k / PCF 100k) 전환 시
                                    *   버스 재구성 글리치 → ESP_ERR_INVALID_STATE 격번 발생. */
  };
  esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_pcf_dev);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "공유 I2C: PCF8574 device add 실패: %s", esp_err_to_name(err));
    return err;
  }
  /* PCF8574 quasi-bidirectional: 입력으로 쓰려면 해당 비트에 1 출력 → 0xFF write.
   *   PCF8575(BOARD_HAS_LR_BUTTONS=1) 면 2바이트(0xFF 0xFF). */
  uint8_t all_hi[2] = {0xFF, 0xFF};
  i2c_master_transmit(s_pcf_dev, all_hi, PCF_NBYTES, 50);
  s_pcf_initialized = true;
  ESP_LOGI(TAG, "PCF8574 공유 HW I2C 초기화 (OLED 버스 공유, SDA=IO%d SCL=IO%d, addr=0x%02X)",
           PCF8574_I2C_SDA, PCF8574_I2C_SCL, PCF8574_I2C_ADDR);
  return ESP_OK;
}

static pcf_state_t PCF_RD_SHARED(void) {
  pcf_state_t data = s_pcf_last;
  if (!s_pcf_initialized || s_pcf_dev == NULL) return data;

  /* PCF8574 read(1바이트). PCF8575(BOARD_HAS_LR_BUTTONS=1) 면 2바이트(P0~7, P10~17).
   * 공유 버스 INVALID_STATE 격번의 근본 원인은 OLED(400k)/PCF(100k) scl 불일치 → _pcf_i2c_shared_init
   * 에서 PCF scl 을 400k 로 통일해 글리치를 짧게 줄였다. 남은 짧은 글리치는 scl 일치 덕에
   * 즉시 재호출로 흡수된다(최대 2회 재시도 → ~격번 소거). scl 100k 일 땐 글리치가 길어 재시도 무효였음. */
  uint8_t buf[2] = {0xFF, 0xFF};
  oled_ui_i2c_lock();   /* 공유 HW I2C: OLED flush 와 직렬화 */
  esp_err_t err = i2c_master_receive(s_pcf_dev, buf, PCF_NBYTES, 50 /*ms*/);
  /* INVALID_STATE 격번은 같은 버스 OLED↔PCF 디바이스 전환 시 드라이버 FSM 의 잔여
   * 상태 때문 — 동시성이 아니므로 직렬화(mutex)로는 안 잡힌다. 짧은 지연 뒤 재시도하면
   * 버스 FSM 이 안정돼 대부분 그 자리에서 흡수된다(최대 4회 → 무응답 자체가 격감). */
  for (int _retry = 0; err == ESP_ERR_INVALID_STATE && _retry < 4; _retry++) {
      esp_rom_delay_us(200);
      err = i2c_master_receive(s_pcf_dev, buf, PCF_NBYTES, 50 /*ms*/);
  }
  oled_ui_i2c_unlock();
  /* 로그 throttle: 간헐 격번(이번 주기 실패→다음 주기 즉시 복구)은 무음 처리하고,
   * 연속 다수 실패(=PCF 물리 분리)만 1회 경고 → UART/페어링 타이밍 보호. */
  static int s_last_err    = -1;
  static int s_fail_streak = 0;
  if (err == ESP_OK) {
    data = (pcf_state_t)buf[0];
#if BOARD_HAS_LR_BUTTONS
    data |= (pcf_state_t)((uint16_t)buf[1] << 8);
#endif
    s_pcf_last = data;
    s_pcf_present = true;
    s_fail_streak = 0;
    if (s_last_err == 1) {   /* 진짜 무응답(분리) 상태에서 복구된 경우만 1회 */
      ESP_LOGI(TAG, "PCF8574 통신 복구 (공유 HW I2C, 0x%02X = 0x%02X)",
               PCF8574_I2C_ADDR, data);
    }
    s_last_err = 0;
  } else {
    s_pcf_present = false;
    if (++s_fail_streak >= 20 && s_last_err != 1) {
      ESP_LOGW(TAG, "PCF8574 무응답 (연속 %d회, 공유 HW I2C): %s",
               s_fail_streak, esp_err_to_name(err));
      s_last_err = 1;
    }
    data = s_pcf_last;
  }
  return data;
}
#endif /* BOARD_I2C_SHARED : 공유 HW I2C 경로 */

#if (!BOARD_I2C_SHARED) || BOARD_I2C_LP_FALLBACK
/* ── 비트뱅 폴링 (GNPE: 단일 경로 / XIAO: LP_I2C 경로) ──────────────────── */
static pcf_state_t PCF_RD_BB(void) {
  pcf_state_t data = s_pcf_last;
  if (!s_pcf_initialized) return data;

  if (s_i2c_mutex && xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    _i2c_start();
    int nack = _i2c_write_byte((PCF8574_I2C_ADDR << 1) | 1);  // read
    if (!nack) {
#if BOARD_HAS_LR_BUTTONS
      uint8_t b0 = _i2c_read_byte(0);          // P0~P7  (ACK, 더 읽음)
      uint8_t b1 = _i2c_read_byte(1);          // P10~P17 (NACK, 마지막)
      data = (pcf_state_t)(b0 | ((uint16_t)b1 << 8));
#else
      data = _i2c_read_byte(1);  // single byte, NACK
#endif
    }
    _i2c_stop();
    xSemaphoreGive(s_i2c_mutex);

    static int s_last_nack = -1;
    if (!nack) {
      s_pcf_last = data;
      s_pcf_present = true;
      if (s_last_nack != 0) {
        ESP_LOGI(TAG, "PCF8574 통신 OK (bit-bang IO%d/IO%d, 0x%02X = 0x%02X)",
                 PCF_BB_SDA, PCF_BB_SCL, PCF8574_I2C_ADDR, data);
        s_last_nack = 0;
      }
    } else {
      s_pcf_present = false;
      if (s_last_nack != 1) {
        ESP_LOGW(TAG, "PCF8574 NACK (bit-bang IO%d/IO%d — 미배선?)",
                 PCF_BB_SDA, PCF_BB_SCL);
        s_last_nack = 1;
      }
      data = s_pcf_last;
    }
  }
  return data;
}
#endif /* 비트뱅 폴링 */

#if BOARD_I2C_LP_FALLBACK
/* LP_I2C(6/7)에 PCF8574 가 ACK 하는지 1회 프로브(write 주소). true=연결됨. */
static bool _pcf_lp_probe(void) {
  if (!(s_i2c_mutex && xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(5)) == pdTRUE))
    return false;
  _i2c_start();
  int nack = _i2c_write_byte((PCF8574_I2C_ADDR << 1) | 0);  // write 주소 → ACK 확인
  _i2c_stop();
  xSemaphoreGive(s_i2c_mutex);
  return (nack == 0);
}

/* 런타임 디스패치: LP 연결이면 비트뱅, 아니면 공유 HW I2C 읽기. */
static pcf_state_t _pcf8574_read(void) {
  return (s_pcf_bus == PCF_BUS_LP) ? PCF_RD_BB() : PCF_RD_SHARED();
}
#endif /* BOARD_I2C_LP_FALLBACK */

/* ─── 버튼 폴링 태스크 ─────────────────────────
 * 모든 버튼이 PCF8574 로 이관되어 단일 read 로 8개 신호를 동시 획득.
 * VIBRATION/CHG_STAT 는 GPIO 직결로 별도 read.
 * ─────────────────────────────────────────────── */
static void _btn_task(void *pvParam) {
  /* Task WDT subscribe — bit-bang I2C 무한 대기 등 hang 자동 리부트.
   *  타임아웃은 sdkconfig 의 CONFIG_ESP_TASK_WDT_TIMEOUT_S (기본 5s).
   *  매 polling tick (10ms) 마다 reset 하므로 정상 동작 시 절대 안 터짐. */
  esp_err_t wdt_err = esp_task_wdt_add(NULL);
  if (wdt_err != ESP_OK) {
    ESP_LOGW(TAG, "_btn_task esp_task_wdt_add 실패: %s", esp_err_to_name(wdt_err));
  }
  while (1) {
    esp_task_wdt_reset();
    somfy_app_bc_btn_tick();
    uint32_t now = _ms_now();

    /* ══ 1. PCF8574 1-byte read = 모든 버튼 + 로터리 + SETUP ══
     *  버튼은 아래 per-button 20ms 디바운스(연속 2폴 이상 동일해야
     *  edge 인정)로 단발 I2C 글리치가 자동 제거된다. 로터리는 자체
     *  detent 디코더가 처리한다. 둘은 독립 처리(바이트 단위로 묶어
     *  게이팅하면 로터리 비트 흔들림이 버튼 처리를 굶긴다). */
    pcf_state_t pcf = _pcf8574_read();
    /* ── (a) UP/DOWN/SELECT/PROG/SETUP 디바운스 처리 ── */
    for (int i = 0; i < BTN_COUNT; i++) {
      btn_state_t *btn = &s_btns[i];
      bool current = ((pcf >> btn->bit) & 1) != 0; // active-LOW: 0=pressed

      if (current != btn->raw_state) {
        btn->raw_state = current;
        btn->debounce_ms = now;
      }

      if ((now - btn->debounce_ms) >= DEBOUNCE_MS) {
        bool stable = btn->raw_state;

        if (!stable && btn->last_state) {
          /* 눌림 */
          btn->press_time_ms = now;
          btn->long_fired = false;
          btn->last_state = false;
          if (btn->press_evt != BTN_EVT_NONE) {
            _send_event(btn->press_evt, 0);
          }

        } else if (stable && !btn->last_state) {
          /* 뗌 */
          uint32_t hold = now - btn->press_time_ms;
          btn->last_state = true;
          if (btn->long_fired) {
            /* 롱프레스가 이미 발사됐으면 release-only 로 처리:
             * - PROG: PROG_RELEASE 발사 (정상)
             * - SETUP: SETUP_SHORT 발사하지 않음 (LONG 이미 처리됨) */
            if (btn->bit == PCF8574_BIT_BTN_PROG) {
              _send_event(btn->release_evt, hold);
            }
          } else {
            _send_event(btn->release_evt, hold);
          }

        } else if (!stable && !btn->last_state) {
          /* 눌린 채 유지 — 롱프레스 1회 발사 + SETUP 15초 강제 재부팅 감시 */
          uint32_t hold = now - btn->press_time_ms;
          if (!btn->long_fired && btn->long_press_ms > 0 && btn->long_evt != BTN_EVT_NONE &&
              hold >= btn->long_press_ms) {
            btn->long_fired = true;
            _send_event(btn->long_evt, hold);
          }
          /* SETUP 15초 이상 hold → 강제 재부팅 (기기 멈춤/응답불가 대비) */
          if (btn->bit == PCF8574_BIT_SETUP_BTN && hold >= SETUP_REBOOT_HOLD_MS) {
            ESP_LOGW(TAG, "[BTN] SETUP %us hold → 강제 재부팅", (unsigned)(hold / 1000));
            esp_restart();
          }
        }
      }
    }

    /* ── (b) 로터리 엔코더 (P0/P1/P2) ── */
    bool rot_a = (pcf >> PCF8574_BIT_ROT_A) & 1;
    bool rot_b = (pcf >> PCF8574_BIT_ROT_B) & 1;
    bool rot_btn = (pcf >> PCF8574_BIT_ROT_BTN) & 1;

    uint8_t ab = ((rot_a ? 1 : 0) << 1) | (rot_b ? 1 : 0);
#if BOARD_ROT_HALF_STEP
    /* ── EC05 등 하프스텝 엔코더 (11·00 양쪽 디텐트 = 2디텐트/사이클, 1디텐트=2스텝) ──
     *  4상 그레이코드 전이 LUT 누산. 바운스(왕복)는 +1/−1 로 상쇄돼 자동 제거
     *  (2-폴 디바운스 불필요), 2스텝(=1디텐트)마다 1 이벤트.
     *  ★ 기존 "11에서만 이탈감지" 디코더는 00 디텐트에서 시작하는 클릭을 놓쳐
     *    EC05 에서 한 방향이 거의 안 먹었다(XIAO 실기 캡처로 확인). */
    if (ab != s_rot_ab_raw) {
      static const int8_t kQuad[16] = {
         0, +1, -1,  0,  -1,  0,  0, +1,
        +1,  0,  0, -1,   0, -1, +1,  0,
      };
      s_rot_accum += kQuad[((s_rot_ab_raw & 3) << 2) | (ab & 3)];
      if (s_rot_accum >= 2 || s_rot_accum <= -2) {
        bool neg = (s_rot_accum < 0);
        s_rot_accum -= (neg ? -2 : 2);
        bool cw = (neg == (_ROT_CW_ON_AB1 != 0));   /* 기존 극성 유지(반대면 _ROT_CW_ON_AB1 뒤집기) */
        if ((now - s_rot_last_ms) >= ROT_MIN_INTERVAL_MS) {
          s_rot_last_ms = now;
          _send_event(cw ? BTN_EVT_ROT_CW : BTN_EVT_ROT_CCW, 0);
        }
      }
      s_rot_ab_raw = ab;
    }
#else
    /* 2-연속표본 디바운스: 같은 값이 직전 폴과 동일할 때만 stable 갱신 */
    if (ab == s_rot_ab_raw && ab != s_rot_ab_stable) {
      uint8_t prev = s_rot_ab_stable;
      s_rot_ab_stable = ab;
      if (s_rot_armed && prev == 0x03 && (ab == 0x01 || ab == 0x02)) {
        /* rest 이탈 첫 안정 엣지 → 방향 1회 결정 + 잠금 */
        bool cw = (ab == 0x01) ? (_ROT_CW_ON_AB1 != 0) : (_ROT_CW_ON_AB1 == 0);
        if ((now - s_rot_last_ms) >= ROT_MIN_INTERVAL_MS) {
          s_rot_last_ms = now;
          _send_event(cw ? BTN_EVT_ROT_CW : BTN_EVT_ROT_CCW, 0);
        }
        s_rot_armed = false;            /* rest 복귀까지 잠금 */
      } else if (!s_rot_armed && ab == 0x03) {
        s_rot_armed = true;             /* detent 복귀 → 재무장(이벤트 없음) */
      }
      /* ab==0 으로 이탈(양 엣지 동시 스킵)은 모호 → armed 유지(클릭 누락 허용) */
    }
    s_rot_ab_raw = ab;
#endif

    bool rot_pressed = !rot_btn;
    if (rot_pressed && s_rot_btn_state) {
      s_rot_btn_press_ms = now;
      s_rot_btn_long_fired = false;
      s_rot_btn_state = false;
      /* 로터리 버튼(STOP/MY) 누름 — UP/DOWN 처럼 누르는 동안 송신 시작.
       *  release 시 BTN_EVT_ROT_CLICK 이 송신을 멈춘다(설정모드는 메뉴용). */
      _send_event(BTN_EVT_ROT_PRESS, 0);
    } else if (!rot_pressed && !s_rot_btn_state) {
      uint32_t hold = now - s_rot_btn_press_ms;
      s_rot_btn_state = true;
      /* v3.0+: 로터리 long press(BTN_EVT_ROT_LONG) 제거 — 설정 버튼과 기능 중복.
       *        길게 눌러도 ROT_CLICK 만 발사 (release 시점에 STOP/취소 처리). */
      _send_event(BTN_EVT_ROT_CLICK, hold);
    }
    /* (v3.0+: ROT_LONG 발사 블록 제거) */

    /* ══ 2. VIBRATION (IO16, JYX-1210-X160) ══
     *  X160 평상시 closed(LOW), 진동 시 brief open(HIGH). HIGH duty-cycle 로
     *  진동 검출(_vibration_track 이 30폴 윈도우 임계값으로 timestamp 갱신). */
    {
      int lvl = gpio_get_level(VIBE_PIN);
      _vibration_track(lvl == 1);

      /* 진단 로그(3초 주기) — HIGH 샘플 비율 / ISR 누적 */
      static uint32_t s_vibe_isr_seen = 0;
      static int      s_vibe_high_cnt = 0;
      static int      s_vibe_periodic_cnt = 0;
      static uint8_t  s_vibe_reenable_cd = 0;

      if (lvl == 1) s_vibe_high_cnt++;
      uint32_t isr_cnt = s_vibe_isr_count;
      if (++s_vibe_periodic_cnt >= 300) {        /* 10ms × 300 = 3초 */
        uint32_t delta = isr_cnt - s_vibe_isr_seen;
        s_vibe_isr_seen = isr_cnt;
        ESP_LOGI(TAG, "[VIBE-stat] 레벨=%d ISR누적=%u (3초+%u) HIGH=%d/300",
                 lvl, (unsigned)isr_cnt, (unsigned)delta, s_vibe_high_cnt);
        s_vibe_periodic_cnt = 0;
        s_vibe_high_cnt = 0;
      }

      /* ISR 재활성(chatter 폭주 cap) */
      if (s_vibe_isr_disabled_flag) {
        if (s_vibe_reenable_cd == 0) s_vibe_reenable_cd = 3;
        s_vibe_reenable_cd--;
        if (s_vibe_reenable_cd == 0) {
          s_vibe_isr_disabled_flag = false;
          gpio_intr_enable(VIBE_PIN);
        }
      }
    }

    /* ★2026-08-11 배터리 절약(C안): 화면이 꺼져 있으면 사용자가 보고 있지 않으므로
     *  폴링을 10ms → 30ms 로 늦춘다. 버튼 인식은 눌림이 보통 100ms 이상이라 30ms 로도
     *  놓치지 않고, PCF8574 비트뱅 I2C 호출이 1/3 로 줄어 CPU 유휴 시간이 늘어난다
     *  (light sleep 진입에 직접 기여). 화면이 켜지면 즉시 10ms 로 복귀한다. */
    /* ★2026-08-12 화면 OFF 시 150ms — light sleep 이 실제로 잘 수 있게 한다.
     *  이 태스크가 30ms 로 돌면 초당 33회 CPU 를 깨워 light sleep 구간이 30ms 를
     *  못 넘긴다(진입·복귀 오버헤드에 절전이 다 깎임). 150ms 는 임의값이 아니라
     *  이미 esp_sleep_enable_timer_wakeup(150000) 백스톱이 쓰는 값이다 —
     *  설계가 이미 150ms 응답을 수용하고 있다. 버튼 누름은 보통 100ms 이상이라
     *  놓치지 않는다. 화면이 켜지면 즉시 10ms 로 복귀한다. */
    vTaskDelay(pdMS_TO_TICKS(oled_ui_is_panel_on() ? 10 : 150));
  }
}

/* ─── 공개 API ───────────────────────────────── */

void btn_handler_init(btn_event_cb_t cb, void *user_data) {
  s_callback = cb;
  s_user_data = user_data;

  if (s_i2c_mutex == NULL) {
    s_i2c_mutex = xSemaphoreCreateMutex();
  }

#if BOARD_I2C_LP_FALLBACK
  /* ── LP_I2C(뒷면 6/7) 비트뱅 핀 idle 설정 후 PCF8574 프로브 ──
   *   응답 있으면 LP 비트뱅 사용, 없으면 공유 HW I2C(OLED 버스)로 자동 전환. */
  gpio_config_t i2c_io = {
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
      .pin_bit_mask = (1ULL << PCF_BB_SDA) | (1ULL << PCF_BB_SCL),
  };
  gpio_config(&i2c_io);
  gpio_set_level(PCF_BB_SDA, 1);
  gpio_set_level(PCF_BB_SCL, 1);
  if (_pcf_lp_probe()) {
    s_pcf_bus = PCF_BUS_LP;
    s_pcf_initialized = true;
    ESP_LOGI(TAG, "PCF8574 LP_I2C 연결 확인 (bit-bang IO%d/IO%d, addr=0x%02X) → LP 사용",
             PCF_BB_SDA, PCF_BB_SCL, PCF8574_I2C_ADDR);
  } else {
    ESP_LOGW(TAG, "PCF8574 LP_I2C(IO%d/IO%d) 무응답 → 공유 HW I2C(OLED 버스)로 자동 전환",
             PCF_BB_SDA, PCF_BB_SCL);
    s_pcf_bus = PCF_BUS_SHARED;
    _pcf_i2c_shared_init();
  }
#elif BOARD_I2C_SHARED
  /* ── 공유 HW I2C: OLED 버스에 PCF8574 device 추가 (비트뱅 안 함) ──
   *   oled_ui_init() 이 먼저 호출돼 버스가 생성돼 있어야 한다
   *   (somfy_app.c: oled_ui_init() → btn_handler_init() 순서). */
  _pcf_i2c_shared_init();
#else
  /* ── PCF8574 bit-bang I2C 핀 초기화 (v2.0) ──────────────────────
   * IO18(SCL), IO19(SDA) 를 내부 pull-up + INPUT 으로 idle.
   * I2C 동작 시 _sda_low/_scl_low 가 OUTPUT 으로 전환 후 LOW drive.
   * 외부 4.7kΩ pull-up 권장 (내부 pull-up 만으로는 marginal).
   * ───────────────────────────────────────────────────────────── */
  gpio_config_t i2c_io = {
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
      .pin_bit_mask = (1ULL << PCF_BB_SDA) | (1ULL << PCF_BB_SCL),
  };
  gpio_config(&i2c_io);
  /* idle 상태 (release) */
  gpio_set_level(PCF_BB_SDA, 1);
  gpio_set_level(PCF_BB_SCL, 1);
  s_pcf_initialized = true;

  ESP_LOGI(TAG, "PCF8574 bit-bang I2C 초기화 (SDA=IO%d SCL=IO%d, addr=0x%02X)",
           PCF_BB_SDA, PCF_BB_SCL, PCF8574_I2C_ADDR);
#endif

  /* ── GPIO 직결 핀 설정 ──────────────────────────────────────
   * IO17 = PCF8574 ~INT (input, pull-up — 외부 10kΩ 권장) ★ v2.0
   * IO3  = CHG_STAT     (input, pull-up — STAT open-drain)
   * IO16 = VIBRATION    (input, pull-up — VS1 GND 측 접점)
   * 모두 light sleep wake source 후보 — main.c _enter_sleep() 에서 등록.
   * ─────────────────────────────────────────────────────────── */
  gpio_config_t io_in = {
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
      .pin_bit_mask = (1ULL << PCF8574_INT_PIN) | (1ULL << VIBE_PIN)
#if !BOARD_CHG_STAT_ACTIVE_HIGH
                      | (1ULL << CHG_STAT_PIN)   /* active-LOW STAT → pull-up */
#endif
      ,
  };
  gpio_config(&io_in);
#if BOARD_CHG_STAT_ACTIVE_HIGH
  /* active-HIGH(VBUS 분압). ★내부 풀다운(~45kΩ)은 분압을 끌어내려 USB 를 LOW 로
   *   오독시킨다: 100k/150k 분압은 출력임피던스 60kΩ 라 45kΩ 가 병렬로 물리면
   *   3.0V → 1.29V 로 떨어져 VIH(≈2.48V) 미달 → _is_usb_powered() 항상 false
   *   → (a) USB 인데 배터리 모드로 1분 유휴 절전(화면 꺼짐),
   *      (b) 배터리 미연결 판정(_nobat_track)이 USB=0 로 영영 성립 못 함.
   *   분압 하단 저항이 이미 풀다운(USB 빠지면 0V)이므로
   *   BOARD_CHG_STAT_EXT_PULLDOWN=1 인 보드는 내부 풀다운을 끈다. (2026-07-17 실측) */
  gpio_config_t chg_io = {
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = BOARD_CHG_STAT_EXT_PULLDOWN ? GPIO_PULLDOWN_DISABLE
                                                  : GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
      .pin_bit_mask = (1ULL << CHG_STAT_PIN),
  };
  gpio_config(&chg_io);
  ESP_LOGI(TAG, "[CHG] IO%d 초기 레벨=%d (USB 연결 시 1 기대 — 내부풀다운 %s)",
           CHG_STAT_PIN, gpio_get_level(CHG_STAT_PIN),
           BOARD_CHG_STAT_EXT_PULLDOWN ? "OFF(외부분압이 풀다운)" : "ON");
#endif

  /* ★ 진동센서 초기 상태 로그 — 배선/극성 진단:
   *   1 → 대기(pull-up 정상, 진동 안 함) — 정상 idle 상태
   *   0 → 진동중 또는 stuck-LOW(센서가 GND 단락 / 배선 문제) */
  ESP_LOGI(TAG, "[VIBE] 초기 IO%d 레벨=%d (1 기대 — pull-up + 진동스위치 open)",
           VIBE_PIN, gpio_get_level(VIBE_PIN));

  /* ★ IO16 any-edge GPIO 인터럽트 — JYX-1210-X160 의 짧은 contact 변화
   *  (수 ms)를 폴링이 놓치지 않게 ISR 로 즉시 잡는다. polarity 와 무관 —
   *  진동만 감지하면 되므로 ANY_EDGE 사용. */
  gpio_set_intr_type(VIBE_PIN, GPIO_INTR_ANYEDGE);
  esp_err_t isr_err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "gpio_install_isr_service: %s", esp_err_to_name(isr_err));
  }
  esp_err_t add_err = gpio_isr_handler_add(VIBE_PIN, _vibe_isr_handler, NULL);
  if (add_err != ESP_OK) {
    ESP_LOGW(TAG, "gpio_isr_handler_add(VIBE): %s", esp_err_to_name(add_err));
  } else {
    ESP_LOGI(TAG, "[VIBE] IO%d any-edge ISR 등록 완료", VIBE_PIN);
  }

  /* ── PCF8574 초기 read (~INT 래치 클리어 포함) ── */
  pcf_state_t pcf_init = _pcf8574_read();
  if (s_pcf_present) {
    ESP_LOGI(TAG, "PCF8574 검출 OK (0x%02X, 초기=0x%02X)",
             PCF8574_I2C_ADDR, pcf_init);
  } else {
    ESP_LOGW(TAG, "PCF8574 미응답 — IO18/IO19 배선 및 4.7kΩ pull-up 확인");
  }
  /* 로터리 초기 A/B 를 실제 idle 값으로 정렬 (헛 델타 방지) */
  {
    uint8_t ab0 = (uint8_t)(((pcf_init >> PCF8574_BIT_ROT_A) & 1) << 1) |
                  (uint8_t)((pcf_init >> PCF8574_BIT_ROT_B) & 1);
    s_rot_ab_stable = ab0;
    s_rot_ab_raw    = ab0;
    s_rot_armed     = true;            /* prev==3 전이에서만 방출하므로 안전 */
#if BOARD_ROT_HALF_STEP
    s_rot_accum     = 0;
#endif
  }

  ESP_LOGI(TAG,
           "버튼 핸들러 초기화 완료 v2.0 — 모든 버튼 PCF8574 (UP=P4 DOWN=P5 "
           "SEL=P6 PROG=P7 SETUP=P3 ROT=P0/1/2). INT=IO%d CHG=IO%d VIBE=IO%d",
           PCF8574_INT_PIN, CHG_STAT_PIN, VIBE_PIN);
}

void btn_handler_start_task(void) {
  xTaskCreate(_btn_task, "btn_handler", 3072, NULL, 10, NULL);   /* composed 로 free 확보 → 전 보드 3072 통일 */
}

bool btn_is_pressed(btn_key_t key) {
  /* PCF8574 캐시 비트 (active-LOW) */
  return ((s_pcf_last >> (uint8_t)key) & 1) == 0;
}

bool btn_handler_is_charging(void) {
#if BOARD_CHG_STAT_ACTIVE_HIGH
  /* VBUS 분압 → HIGH = USB 연결(충전 중/만충). active-HIGH (XIAO/H2) */
  return gpio_get_level(CHG_STAT_PIN) == 1;
#else
  /* 충전 IC STAT open-drain, active-LOW (GNPE MCP73831) */
  return gpio_get_level(CHG_STAT_PIN) == 0;
#endif
}

/* ─── 진동 wake 플래그 ─────────────────────────────────────────
 * VS1 가 IO16 GPIO 직결이 되어 light sleep 중 gpio_wakeup_enable 로
 * 진동만으로 wake 가능. main.c 가 sleep 직후 wake_cause 분석하여
 * 진동에 의한 것이면 set_vibration_wake() 호출.
 * ──────────────────────────────────────────────────────────── */
static volatile bool s_vibration_wake_flag = false;

bool btn_handler_was_vibration_wake(void) { return s_vibration_wake_flag; }
void btn_handler_clear_vibration_wake(void) { s_vibration_wake_flag = false; }
void btn_handler_set_vibration_wake(void) { s_vibration_wake_flag = true; }

/* ─── 진동 감지 ──────────────────────────────────────────────
 *  센서: JYX-1210-X160 SMD 진동 스위치 (평상시 닫힘, 진동시 순간 OPEN).
 *  10ms 폴링은 ms 단위의 짧은 contact 변화를 놓치므로 GPIO any-edge ISR
 *  을 추가해 어느 방향이든 엣지가 발생하면 즉시 카운트한다(polarity 와
 *  무관 — 진동만 감지하면 됨). 폴링은 평상시 레벨 모니터/디버그 용도. */
static volatile int64_t s_last_vibration_us = 0;
/* s_vibe_isr_count / s_vibe_isr_disabled_flag 정의는 파일 상단 forward
 *  declaration (line 26-27) 으로 통합 — 별도 정의 불필요 (static 변수는
 *  tentative declaration 으로 자동 0 초기화). */

static void IRAM_ATTR _vibe_isr_handler(void *arg) {
    /* 즉시 자기 인터럽트 disable — contact bounce 폭주 차단(폴링이 재활성).
     *  ★ s_last_vibration_us 는 ISR 이 갱신하지 않는다 — X160 은 가만히
     *  둬도 chatter 로 ISR 이 cap 한계까지 계속 발사돼 진동/대기 구분 불가.
     *  대신 폴링이 HIGH duty-cycle 임계값 기반으로 timestamp 갱신.
     *  ★★ gpio_intr_disable(driver/gpio.c)는 flash 거주 함수 — NVS write 중
     *  flash cache 가 잠시 비활성화되는데 그 사이 ISR 이 발사되면 cache error
     *  panic. IRAM 인라인인 gpio_ll_intr_disable 을 직접 호출하여 우회한다
     *  (HAL 레이어, 항상 IRAM-safe). 폴링 태스크의 gpio_intr_enable 은 task
     *  context 라 flash 접근 OK — 그대로 둔다. */
    gpio_ll_intr_disable(&GPIO, VIBE_PIN);
    s_vibe_isr_disabled_flag = true;
    s_vibe_isr_count++;
}

bool btn_handler_is_vibrating(void) {
  /* X160: 평상시 closed(LOW), 진동 시 brief open(HIGH). 폴링은 30폴 윈도우
   *  HIGH duty-cycle 임계값으로 s_last_vibration_us 를 갱신한다. 진동 활성
   *  판정은 그 timestamp 가 최근(500ms 이내)인지로 결정. */
  if (s_last_vibration_us == 0) return false;
  int64_t since = esp_timer_get_time() - s_last_vibration_us;
  return since >= 0 && since < 500000;   /* 500ms recency */
}

int64_t btn_handler_last_vibration_us(void) { return s_last_vibration_us; }

/* ISR 카운터 노출 — 디버그 로그용 */
uint32_t btn_handler_vibe_isr_count(void) { return s_vibe_isr_count; }

/* HIGH 샘플 누적 — 30폴(=300ms) 윈도우 안에 2개 이상 잡히면 즉시 발사
 *  (윈도우 종료까지 안 기다림 → 최저지연 ~20ms). X160 에서 가만≈0/30,
 *  흔듦≈10~25/30 라 임계값 2 면 첫 흔들림에서 바로 트리거. */
/* ★2026-07-24 진동센서 고장(stuck) 판별 추가.
 *  증상: COM7 에서 화면이 저절로 켜지고 안 꺼졌다. 로그
 *    `[VIBE-stat] 진동=1 ISR누적=70338 (3초+100) HIGH=300/300`
 *  = 핀이 **300/300 전부 HIGH** 로 붙어 있는데 ISR 은 초당 33회씩 발생.
 *  기존 판정은 "HIGH 2회면 진동"이라 **핀이 고정 HIGH 면 무조건 통과** →
 *  매번 _mark_activity()+화면 깨우기가 반복돼 화면이 영영 안 꺼졌다.
 *  정상 진동은 접점이 떨렸다 붙었다 하므로 HIGH/LOW 가 **섞여야** 한다.
 *  → 최근 윈도우가 전부 HIGH(또는 전부 LOW)로 고정되면 배선/스위치 고장으로 보고
 *    진동 이벤트를 무시한다. 섞인 패턴이 돌아오면 자동으로 다시 인정한다. */
#define VIBE_STUCK_WIN   200          /* 고장 판정 관찰 샘플 수 */
static volatile bool s_vibe_stuck = false;
bool btn_handler_vibe_stuck(void) { return s_vibe_stuck; }

static void _vibration_track(bool high_now) {
  static int s_win_cnt = 0;
  static int s_win_high = 0;
  /* ── 고장(stuck) 감지: 최근 VIBE_STUCK_WIN 샘플이 한쪽으로만 고정인가 ── */
  {
    static int st_cnt = 0, st_high = 0;
    st_cnt++;
    if (high_now) st_high++;
    if (st_cnt >= VIBE_STUCK_WIN) {
      bool stuck = (st_high == st_cnt) || (st_high == 0);   /* 전부 HIGH 또는 전부 LOW */
      if (stuck != s_vibe_stuck) {
        s_vibe_stuck = stuck;
        ESP_LOGW(TAG, "[VIBE] %s (최근 %d샘플 HIGH=%d) — 진동 %s",
                 stuck ? "센서 고장 판정(핀 고정)" : "정상 복귀",
                 st_cnt, st_high, stuck ? "무시함" : "다시 인정");
      }
      st_cnt = 0; st_high = 0;
    }
  }
  if (s_vibe_stuck) return;           /* 고장 상태 — 진동으로 인정하지 않음 */

  s_win_cnt++;
  if (high_now) s_win_high++;
  if (s_win_high >= 2) {              /* 임계값 도달 — 즉시 timestamp 갱신 */
    s_last_vibration_us = esp_timer_get_time();
    s_win_cnt = 0;
    s_win_high = 0;
  } else if (s_win_cnt >= 30) {       /* 윈도우 만료 (임계값 미달) — 리셋 */
    s_win_cnt = 0;
    s_win_high = 0;
  }
}

SemaphoreHandle_t btn_handler_get_i2c_mutex(void) { return s_i2c_mutex; }
