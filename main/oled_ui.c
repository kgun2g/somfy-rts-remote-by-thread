#include "oled_ui.h"
#include "ssd1306.h"     // esp-idf-ssd1306 라이브러리
#include "font8x8_basic.h"
#include "font5x7_basic.h"   // 72×40 화면용 narrow 5×7 폰트
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"    // 공유 I2C 버스 직렬화 뮤텍스(OLED flush ↔ PCF8574 read)
#include "driver/i2c_master.h"  // SSD1315 보정 명령 송신용 (new driver)
#include "driver/gpio.h"        // 2026-07-17: 물린 I2C 버스 라인레벨 관찰 + 9클럭 복구 비트뱅
#include "soc/gpio_reg.h"       // 2026-07-17: OUT_EN/OUT/IN 레지스터 직접 확인(진단)
#include "soc/io_mux_reg.h"     //             IO_MUX 기능 선택 확인(진단)
#include "soc/lp_aon_reg.h"     // 2026-07-17: LP_AON_GPIO_HOLD0_REG — 핀 hold(래치) 확인(진단)
#include "esp_rom_sys.h"        // esp_rom_delay_us — 페이지 write 재시도 간 짧은 대기
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>           // 화면 보호기 시계/날짜용

static const char *TAG = "OLED_UI";

/* ─── SSD1306 장치 핸들 ──────────────────────── */
static SSD1306_t s_dev;

/* OLED 와 PCF8574(버튼)가 HW I2C 버스를 공유. _ui_task flush(콜론 anim 으로 잦음)와
 * btn_task PCF read 가 동시에 같은 버스를 건드리면 unexpected nack 발생 → 뮤텍스로 직렬화한다.
 * (scl 400k 로 INVALID_STATE 격번은 해결됐지만, 동시 접근 nack 은 별개라 직렬화가 필요.) */
/* ★★2026-08-11 **재귀 뮤텍스로 변경** — 충전률 측정 재활성의 전제조건.
 *
 *  왜 재귀여야 하나:
 *    보호 범위를 `_fb_flush` 에서 **`_bbo_write()` 전송 함수 자체**로 옮겼는데,
 *    `_fb_flush` 는 바깥 락을 쥔 채 `_bbo_write` 를 부른다. 일반 뮤텍스면 자기
 *    자신을 기다려 **즉시 데드락**(화면 영구 정지)이다.
 *
 *  왜 범위를 옮겼나:
 *    기존엔 `_fb_flush`(oled_ui.c) 만 락이 있어 아래 두 경로가 **무방비**였다.
 *      · `_oled_send_cmds()`  ← oled_ui_set_display_on() : 화면 자동 OFF/ON 마다
 *      · `_bbo_probe()`       ← _oled_try_detect()       : 미검출 시 5초마다
 *    두 경로 모두 **somfy_app(prio 4)** 에서 불리고, OLED flush 는 **oled_ui(prio 3)**
 *    라 somfy_app 이 전송 도중에 끼어든다. 비트뱅은 CPU 가 곧 클럭이므로 선점당한
 *    전송은 SCL/SDA 가 중간 상태로 수백 us 멈춰 SSD1306 이 고착된다.
 *    (sim/tools/adc_oled_mutex_sim.py — 8회x10분 중 6회 고착, 최빠른 80초.
 *     락을 _bbo_write 로 옮기면 손상 0 / 고착 0, 배터리 측정 952/952 정상.)
 *
 *  ※ 해제는 반드시 획득한 태스크가 한다(FreeRTOS 재귀 뮤텍스 제약). */
static SemaphoreHandle_t s_i2c_mutex = NULL;

/* OLED 존재 여부 — 미연결(예: 배선 전 보드) 시 flush 를 건너뛰어 I2C NACK
 * 로그 스팸(50ms마다)을 막는다. 미검출이면 5초마다 자동 재검출(hot-plug 지원). */
static bool     s_oled_present = false;
static uint32_t s_oled_last_probe_ms = 0;
/* ★2026-07-24 모니터용 검출상태 미러 — **비트뱅 가드 밖**에 두어야 한다.
 *  (BOARD_OLED_BITBANG=0 인 보드(H2 등)에서 선언만 사라져 빌드가 깨졌던 이력) */
volatile bool g_oled_present_mon = false;
/* 계측 카운터도 **가드 밖**에 둔다 — somfy_app 의 [OLEDMON] 이 보드 무관하게 참조하므로
 *  BOARD_OLED_BITBANG=0 인 보드(H2)에서 링크 에러가 났던 이력. */
volatile uint32_t g_bbo_tx_cnt = 0, g_bbo_fail_cnt = 0;
/* ★2026-08-11 _bbo_write 의 락 획득 타임아웃 횟수. 정상이면 0 이어야 한다.
 *  0 이 아니면 누군가 뮤텍스를 오래 쥐고 있다는 뜻 → [OLEDMON] 으로 관찰한다. */
volatile uint32_t g_bbo_lock_to_cnt = 0;
/* 2026-07-23 모듈 고착 자동복구: 연속 검출실패 횟수(성공 시 0 으로 리셋) */
static uint32_t s_oled_recover_tries = 0;

/* ★2026-07-23 RF 송신 중 OLED 정지 (somfy_app.c 의 _do_rf_send 가 설정).
 *  왜: 447MHz CC1101 송신(1~1.5초, 안테나가 I2C 배선 근처) 동안 I2C 트랜잭션이
 *  깨져 **SSD1306 이 전송 중간 상태로 고착**되는 것이 실사용에서 확인됐다
 *  (좌/우 버튼=RF 없음 → 정상 / 상/하 버튼=RF 송신 → 느려지다 멈춤).
 *  고착되면 모듈 전원을 끊기 전엔 안 풀리므로(RES 핀 없는 4핀 모듈),
 *  가장 위험한 구간에는 아예 버스를 건드리지 않는다. */
static volatile bool s_rf_tx_active = false;
void oled_ui_set_rf_tx(bool active) { s_rf_tx_active = active; }
static void _oled_panel_init(void);   /* 패널 init 시퀀스 (정의는 oled_ui_init 직전) */

/* ── CC1101(RF) 검출 여부 — app_main.cpp 가 부팅 시 1회 설정(extern "C") ──
 *   false(= CC1101 미연결/미응답)면 메인화면 상단 주파수 자리에 "NONE" 표시.
 *   ※ GNPE 포함 모든 보드에 적용 — CC1101 유무는 보드/개체마다 다를 수 있다
 *     (사용자 확인 2026-06-19). */
extern bool g_rf_ready;

/* 메인화면 상단 주파수 문자열: RF 정상이면 "447.62", CC1101 미검출이면 "NONE". */
static void _main_freq_str(const oled_ui_ctx_t *ctx, char *buf, size_t n) {
    if (g_rf_ready) snprintf(buf, n, "%.2f", ctx->freq_mhz);
    else            snprintf(buf, n, "NONE");
}

/* ─── SSD1315(72×40) 전용 보정 명령 ────────────────────────────────
 * esp-idf-ssd1306 라이브러리는 height=40을 인식하지 못해 multiplex
 * (0xA8)와 COM pin map(0xDA) 명령을 누락합니다. 결과적으로 컨트롤러는
 * 64행으로 스캔하고 패널 절반만 출력됩니다.
 *
 * 0.42" 72×40 SSD1315 권장 시퀀스:
 *   0xA8 0x27   – multiplex ratio = 39 (40 rows)
 *   0xDA 0x12   – COM pin alt, no remap
 *   0xD3 0x00   – display offset = 0
 *   0xAD 0x30   – internal IREF (SSD1315 only)
 * ─────────────────────────────────────────────────────────────────── */
#define OLED_I2C_TIMEOUT_MS 50
/* ★2026-07-19: OLED I2C 클럭 — 라이브러리 하드코딩 400k 대신 저속 구동.
 *  이 보드의 OLED 버스는 400k 에서 트래픽 하 글리치 폭주 → INVALID_STATE → flush 정지
 *  → 화면 멈춤/부팅깨짐. 금요일 04:38 '정상 출력' 버전이 쓰던 100kHz 로 마진 확보.
 *  (히스토리 07-17 04:38:33 의 _oled_i2c_init_at 방식 복원.) */
#define OLED_I2C_HZ 100000

/* ═══════════════════════════════════════════════════════════════════════
   ★2026-07-23 OLED 소프트웨어 비트뱅 I2C 전송 (BOARD_OLED_BITBANG)

   왜: 이 보드에서 ESP32 의 **HW I2C0 페리페럴이 "bus busy" 로 고착**되는 현상이
   재현된다(실측). 라인은 idle 1/1 인데 `i2c.master: clear bus failed` +
   `reset hardware failed` 가 뜨고, 버스 del→재생성으로도 안 풀린다.
   반면 **순수 GPIO 비트뱅은 같은 순간에도 0x3C ACK 를 받아낸다.**
   → 전송 계층을 비트뱅으로 바꿔 페리페럴을 통째로 우회한다.

   open-drain 에뮬: 1=INPUT(외부 4.7k 풀업이 HIGH) / 0=OUTPUT LOW.
   속도: half-period 5us ≈ 100kHz (HW 와 동일 마진).
   ═══════════════════════════════════════════════════════════════════════ */
#if BOARD_OLED_BITBANG
#define BBO_SDA ((gpio_num_t)BOARD_PIN_OLED_SDA)
#define BBO_SCL ((gpio_num_t)BOARD_PIN_OLED_SCL)
/* ★2026-07-23 속도: half-period 5us(=100kHz)면 128×64 한 프레임(1KB)에 약 93ms 걸려
 *  화면이 눈에 띄게 느려진다(실사용 확인). SSD1306 규격 상한은 400kHz 이므로 2us(≈200kHz)
 *  로 올려 프레임을 ~45ms 로 단축. 불안정하면 3~5 로 되돌릴 것. */
/* ★2026-07-23 속도: 라이브러리 기본값과 같은 400kHz 목표(half-period 1.25us→1us).
 *  이전에 1us 로 올렸을 때 화면에 랜덤 점이 찍힌 것은 속도 자체가 아니라
 *  gpio_set_direction() HAL 호출의 큰/불규칙한 지연 때문이었다. 위 레지스터 직접
 *  접근으로 파형이 깨끗해져 이 속도를 감당할 수 있다.
 *  ※그래도 점/깨짐이 보이면 2~3 으로 올릴 것(값이 클수록 느리고 안정). */
#define BBO_HALF_US 1

/* ★2026-07-23 GPIO 레지스터 직접 접근으로 전환.
 *  이전엔 비트마다 gpio_set_direction() (HAL 함수)을 불렀는데, 호출 비용이 커서
 *  (a) 실제 속도가 공칭의 몇 분의 1이고 (b) 타이밍이 불규칙해 400kHz 시도 시
 *  데이터가 깨졌다(화면에 랜덤 점). 레지스터 쓰기는 수십 ns라 파형이 깨끗하다.
 *
 *  핀을 **INPUT_OUTPUT_OD**(오픈드레인+입력버퍼 ON)로 한 번만 설정해두면
 *  출력 레지스터만 토글해도 I2C 오픈드레인 동작이 그대로 된다:
 *     1 쓰기 = 라인 릴리즈(외부 풀업이 HIGH) / 0 쓰기 = LOW 구동.
 *  방향 레지스터를 건드릴 필요가 없어 더 빠르고 안전하다.
 *  ※OUTPUT_OD(입력버퍼 OFF)로 하면 gpio 읽기가 늘 0이 된다 — 반드시 INPUT_OUTPUT_OD. */
#define BBO_SDA_MASK (1UL << BOARD_PIN_OLED_SDA)
#define BBO_SCL_MASK (1UL << BOARD_PIN_OLED_SCL)

static inline void _bbo_hi(uint32_t mask) { REG_WRITE(GPIO_OUT_W1TS_REG, mask); }
static inline void _bbo_lo(uint32_t mask) { REG_WRITE(GPIO_OUT_W1TC_REG, mask); }
static inline int  _bbo_rd(uint32_t mask) { return (REG_READ(GPIO_IN_REG) & mask) ? 1 : 0; }
/* 구 인터페이스 유지(호출부 변경 최소화) */
static inline void _bbo(gpio_num_t p, int v) {
    uint32_t m = (1UL << (uint32_t)p);
    if (v) _bbo_hi(m); else _bbo_lo(m);
}
/* 핀을 오픈드레인 GPIO 로 확보. HW I2C 를 쓰지 않으므로 버스 생성도 하지 않는다. */
static void _bbo_init_pins(void) {
    gpio_reset_pin(BBO_SDA); gpio_reset_pin(BBO_SCL);
    gpio_config_t io = {
        .pin_bit_mask = BBO_SDA_MASK | BBO_SCL_MASK,
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,   /* 오픈드레인 + 입력버퍼 ON */
        .pull_up_en   = GPIO_PULLUP_ENABLE,          /* 외부 4.7k 와 병렬(보조) */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    _bbo_hi(BBO_SDA_MASK | BBO_SCL_MASK);            /* idle = 둘 다 릴리즈 */
    esp_rom_delay_us(10);
}
/* 1바이트 송신 후 ACK 수신. true=ACK */
static bool _bbo_byte(uint8_t b) {
    /* ★2026-07-23 속도수정: 데이터 setup/hold 에 3us 를 하드코딩해 두었던 탓에
     *  비트당 7us(≈143kHz)가 걸려 BBO_HALF_US 를 낮춘 효과가 상쇄되고 있었다.
     *  레지스터 직접 쓰기는 수십 ns 라 SSD1306 의 setup(~100ns) 요구를 이미 만족한다.
     *  → 모든 구간을 BBO_HALF_US 로 통일해 비트당 2×HALF 로 만든다(1us → ≈400kHz). */
    for (int i = 0; i < 8; i++) {
        _bbo(BBO_SDA, (b & 0x80) ? 1 : 0); b <<= 1; esp_rom_delay_us(BBO_HALF_US);
        _bbo(BBO_SCL, 1); esp_rom_delay_us(BBO_HALF_US);
        _bbo(BBO_SCL, 0);
    }
    _bbo(BBO_SDA, 1); esp_rom_delay_us(BBO_HALF_US);         /* SDA 릴리즈 → 슬레이브 ACK */
    _bbo(BBO_SCL, 1); esp_rom_delay_us(BBO_HALF_US);
    int ack = _bbo_rd(BBO_SDA_MASK);
    _bbo(BBO_SCL, 0);
    return ack == 0;
}
/* addr7 로 buf[len] 쓰기. START→addr(W)→데이터…→STOP. true=전 바이트 ACK */
/* ★2026-07-24 계측: ADC 읽기와 OLED 전송 실패의 상관관계를 잡기 위한 전역.
 *  somfy_app.c 의 _read_bat_mv 가 읽기 직전/직후에 기록한다. */
volatile int64_t g_adc_enter_us = 0;   /* ADC 읽기 진입 시각 */
volatile int64_t g_adc_exit_us  = 0;   /* ADC 읽기 종료 시각 */

/* ★★2026-08-11 전송 1건을 통째로 뮤텍스로 보호한다(충전률 측정 재활성의 핵심).
 *
 *  이전에는 `_fb_flush` 만 락을 잡아 `_oled_send_cmds`(화면 OFF/ON)와 `_bbo_probe`
 *  (5초 재검출)가 무방비였다. 두 경로는 somfy_app(prio 4)에서 불려 oled_ui(prio 3)의
 *  전송을 선점 → 비트뱅 파형이 중간에 멈춰 SSD1306 고착. 위 s_i2c_mutex 주석 참조.
 *
 *  락은 **여기 한 곳**에 두는 것이 안전하다. 호출 지점이 늘어도 자동으로 보호되고,
 *  `_fb_flush` 의 바깥 락과 중첩되면 재귀 획득으로 흡수된다(depth 증가).
 *
 *  타임아웃 시 동작: **그냥 전송한다**(건너뛰지 않는다). 락은 겹침 방지용 최적화지,
 *  전송의 전제조건이 아니다. 200ms 나 못 잡았다면 보유자가 이상한 상태라는 뜻인데,
 *  그때 화면을 영구히 멈추는 것보다 한 프레임 깨지는 편이 낫다. 발생 횟수는
 *  g_bbo_lock_to_cnt 로 관찰한다(정상=0). */
#define BBO_LOCK_WAIT_MS 200

static bool _bbo_write(uint8_t addr7, const uint8_t *buf, size_t len) {
    bool held = false;
    if (s_i2c_mutex) {
        held = (xSemaphoreTakeRecursive(s_i2c_mutex,
                                        pdMS_TO_TICKS(BBO_LOCK_WAIT_MS)) == pdTRUE);
        if (!held) g_bbo_lock_to_cnt++;   /* 락 없이 진행 — 위 주석의 판단 근거 */
    }
    _bbo(BBO_SDA, 1); _bbo(BBO_SCL, 1); esp_rom_delay_us(BBO_HALF_US);
    _bbo(BBO_SDA, 0); esp_rom_delay_us(BBO_HALF_US);         /* START */
    _bbo(BBO_SCL, 0); esp_rom_delay_us(BBO_HALF_US);
    bool ok = _bbo_byte((uint8_t)(addr7 << 1));              /* write */
    for (size_t i = 0; ok && i < len; i++) ok = _bbo_byte(buf[i]);
    _bbo(BBO_SDA, 0); esp_rom_delay_us(BBO_HALF_US);         /* STOP */
    _bbo(BBO_SCL, 1); esp_rom_delay_us(BBO_HALF_US);
    _bbo(BBO_SDA, 1); esp_rom_delay_us(BBO_HALF_US);
    if (held) xSemaphoreGiveRecursive(s_i2c_mutex);   /* ★STOP 까지 끝낸 뒤 해제 */
    g_bbo_tx_cnt++;
    if (!ok) {
        /* ★실패 순간을 ADC 읽기와 대조 — "ADC 구간 중/직후에 깨지는가"를 실측한다.
         *  adc_in_window: 이 전송이 ADC 읽기 구간과 겹쳤는지(진입<시작 && 종료없음/이후) */
        int64_t now = esp_timer_get_time();
        int64_t ent = g_adc_enter_us, ext = g_adc_exit_us;
        g_bbo_fail_cnt++;
        if ((g_bbo_fail_cnt % 20) == 1)   /* 20회마다 1줄(로그 폭주 방지) */
            ESP_LOGE(TAG, "[BBFAIL] #%u len=%u  ADC진입후 %lldms / ADC종료후 %lldms %s",
                     (unsigned)g_bbo_fail_cnt, (unsigned)len,
                     ent ? (now - ent) / 1000 : -1,
                     ext ? (now - ext) / 1000 : -1,
                     (ent > ext) ? "★ADC 진행중!" : "");
    }
    return ok;
}
static bool _bbo_probe(uint8_t addr7) { return _bbo_write(addr7, NULL, 0); }
#endif /* BOARD_OLED_BITBANG */

/* 라이브러리가 init 시 보유한 device handle을 통해 직접 명령을 전송 */
static esp_err_t _oled_send_cmds(const uint8_t *cmds, size_t len)
{
    /* "Co=0, D/C#=0" control byte 0x00 + cmd stream */
    uint8_t buf[1 + 16];
    if (len > sizeof(buf) - 1) return ESP_ERR_INVALID_SIZE;
    buf[0] = 0x00;
    memcpy(&buf[1], cmds, len);
#if BOARD_OLED_BITBANG
    return _bbo_write(s_dev._address ? s_dev._address : BOARD_OLED_ADDR, buf, len + 1)
               ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
#else
    return i2c_master_transmit(s_dev._i2c_dev_handle, buf, len + 1,
                                OLED_I2C_TIMEOUT_MS);
#endif
}

#if OLED_PANEL_FIXUP_72X40
static void _ssd1315_apply_72x40_fixup(void)
{
    /* SSD1315 패널 보정 시퀀스 — 라이브러리가 누락한 명령들 */
    static const uint8_t fixup[] = {
        0xAE,                 // Display OFF
        0xA8, 0x27,           // Multiplex ratio = 39
        0xDA, 0x12,           // COM pins: alternative, no left/right remap
        0xD3, 0x00,           // Display offset = 0
        0xAD, 0x30,           // Internal IREF (SSD1315)
        0x40,                 // Display start line = 0
        0xAF,                 // Display ON
    };
    esp_err_t r = _oled_send_cmds(fixup, sizeof(fixup));
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "SSD1315 72×40 보정 명령 실패: 0x%02X", r);
    }
}
#endif /* OLED_PANEL_FIXUP_72X40 */

/* ─── 태스크 컨텍스트 ────────────────────────── */
static oled_ui_ctx_t *s_ctx = NULL;

/* ─── 간단한 ms 타임스탬프 ──────────────────── */
static inline uint32_t _ms_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ═══════════════════════════════════════════════
   아이콘 비트맵 (8×8 픽셀)
   - 72×40 화면에서 1픽셀=1비트 렌더링
═══════════════════════════════════════════════ */

/* ↑ 화살표 (UP) */
static const uint8_t icon_up[8] = {
    0b00011000,
    0b00111100,
    0b01111110,
    0b11111111,
    0b00011000,
    0b00011000,
    0b00011000,
    0b00011000,
};

/* ↓ 화살표 (DOWN) */
static const uint8_t icon_down[8] = {
    0b00011000,
    0b00011000,
    0b00011000,
    0b00011000,
    0b11111111,
    0b01111110,
    0b00111100,
    0b00011000,
};

/* ■ 정지 (STOP) */
static const uint8_t icon_stop[8] = {
    0b00000000,
    0b01111110,
    0b01111110,
    0b01111110,
    0b01111110,
    0b01111110,
    0b01111110,
    0b00000000,
};

/* ◎ 프로그램 */
static const uint8_t icon_prog[8] = {
    0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10100101,
    0b10011001,
    0b01000010,
    0b00111100,
};

/* ← 틸트 UP */
static const uint8_t icon_tilt_up[8] = {
    0b00011000,
    0b00111100,
    0b01111110,
    0b00011000,
    0b00011000,
    0b11111111,
    0b01111110,
    0b00111100,
};

/* → 틸트 DOWN */
static const uint8_t icon_tilt_dn[8] = {
    0b00111100,
    0b01111110,
    0b11111111,
    0b00011000,
    0b00011000,
    0b01111110,
    0b00111100,
    0b00011000,
};

/* WiFi 아이콘 */
static const uint8_t icon_wifi[8] = {
    0b00111100,
    0b01000010,
    0b10011001,
    0b00100100,
    0b01011010,
    0b00011000,
    0b00011000,
    0b00011000,
};

/* ═══════════════════════════════════════════════
   저수준 드로잉 헬퍼
   - esp-idf-ssd1306 라이브러리의 픽셀 단위 접근
═══════════════════════════════════════════════ */

/* 화면 버퍼 — "물리 패널" 크기 (GNPE 72×40=360B, XIAO 128×64=1024B). */
#define OLED_PAGES   (OLED_PANEL_H / 8)   // GNPE 5 / XIAO 8 pages
static uint8_t s_fb[OLED_PAGES][OLED_PANEL_W];

/* ★2026-07-24 전송량 감축(dirty-page).
 *  왜: 계측 결과 화면 전체를 초당 ~11장 다시 보내 **초당 약 450건**의 I2C 트랜잭션이
 *  발생했다. 실제로 바뀌는 건 시계 등 일부인데 전량을 매번 재전송한 것.
 *  전송 1건마다 실패(고착) 기회가 생기므로, **직전에 보낸 내용과 같은 페이지는 건너뛴다.**
 *  기대: 40건/화면 → 5~10건(75~87% 감축).
 *  s_shadow 는 "마지막으로 성공적으로 보낸" 페이지 내용. 전송 실패 시 갱신하지 않아
 *  다음 주기에 자동 재시도된다. 검출 실패/재검출 시에는 전체 무효화(강제 전량 전송). */
static uint8_t s_shadow[OLED_PAGES][OLED_PANEL_W];
static bool    s_shadow_valid = false;
volatile uint32_t g_page_skip = 0, g_page_sent = 0;   /* 감축 효과 계측(모니터에서 출력) */
#define s_skip_cnt g_page_skip
#define s_sent_cnt g_page_sent
static inline void _shadow_invalidate(void) { s_shadow_valid = false; }
/* 이 페이지를 보내야 하는가(내용이 바뀌었나) */
static inline bool _page_dirty(int p, const uint8_t *src, int width) {
    if (!s_shadow_valid) return true;
    if (p < 0 || p >= OLED_PAGES || width > OLED_PANEL_W) return true;  /* 방어 */
    return memcmp(s_shadow[p], src, (size_t)width) != 0;
}
static inline void _shadow_store(int p, const uint8_t *src, int width) {
    if (p < 0 || p >= OLED_PAGES || width > OLED_PANEL_W) return;       /* 방어 */
    memcpy(s_shadow[p], src, (size_t)width);
}

static void _fb_clear(void) {
    memset(s_fb, 0, sizeof(s_fb));
}

/* 논리 캔버스(72×40)를 물리 패널 크기로 nearest-neighbor 스케일해 화면을 꽉 채운다.
 *  - GNPE(패널==캔버스 72×40): 각 논리픽셀 → 1×1 블록 → 완전 무변경.
 *  - XIAO(128×64): 각 논리픽셀 → 약 1.78×1.6 블록 → 전체화면 표시.
 *  논리 좌표(x,y) 1픽셀이 물리 [px0,px1)×[py0,py1) 사각형으로 확대된다. */
static void _fb_set_pixel(int x, int y, bool on) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;  // 논리 캔버스 클립
    int px0 = (x * OLED_PANEL_W) / OLED_WIDTH;
    int px1 = ((x + 1) * OLED_PANEL_W) / OLED_WIDTH;
    int py0 = (y * OLED_PANEL_H) / OLED_HEIGHT;
    int py1 = ((y + 1) * OLED_PANEL_H) / OLED_HEIGHT;
    for (int py = py0; py < py1; py++) {
        int page = py / 8, bit = py % 8;
        for (int px = px0; px < px1; px++) {
            if (on) s_fb[page][px] |=  (1 << bit);
            else    s_fb[page][px] &= ~(1 << bit);
        }
    }
}

/* 논리 좌표(x,y)의 현재 픽셀 읽기 (XOR/반전 효과용 — 블록 좌상단 대표 픽셀). */
static bool _fb_get_pixel(int x, int y) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return false;
    int px = (x * OLED_PANEL_W) / OLED_WIDTH;
    int py = (y * OLED_PANEL_H) / OLED_HEIGHT;
    return (s_fb[py / 8][px] >> (py % 8)) & 1;
}

/* 물리 패널 좌표에 전체폭 1px 가로선 (블록스케일 거치지 않음 → 정확히 1px). */
__attribute__((unused))
static void _fb_hline_phys(int py) {
    if (py < 0 || py >= OLED_PANEL_H) return;
    int page = py / 8, bit = py % 8;
    for (int px = 0; px < OLED_PANEL_W; px++) s_fb[page][px] |= (1 << bit);
}

#if OLED_RENDER_NATIVE
/* ══ 네이티브(물리 1:1) 렌더 프리미티브 — 풀스크린 네이티브 패널 공용 ══
 *  (해상도 기준 선택: OLED_RENDER_NATIVE = 128×64(가로) 또는 64×128(세로).
 *   보드 무관 — BOARD_OLED_* 가 그 해상도면 이 경로를 탄다. 프리미티브는
 *   OLED_PANEL_W/H 로 클리핑하므로 가로/세로 어느 패널에도 그대로 동작.) */
static inline void _px(int x, int y, bool on) {
    if (x < 0 || x >= OLED_PANEL_W || y < 0 || y >= OLED_PANEL_H) return;
    if (on) s_fb[y / 8][x] |=  (1 << (y % 8));
    else    s_fb[y / 8][x] &= ~(1 << (y % 8));
}
static void _pfill(int x, int y, int w, int h, bool on) {
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++) _px(x + i, y + j, on);
}
/* 깔끔한 산세리프(고딕) 6×9 글리프 — 메인 화면 사용 문자 전용.
 *  row-major(9행), bit col(0=좌). 8-wide 셀 안에 6px 글자(좌측 1px 여백). */
static const uint8_t kG_dig[10][9] = {
    {0x1E,0x21,0x21,0x21,0x21,0x21,0x21,0x21,0x1E}, // 0
    {0x04,0x06,0x04,0x04,0x04,0x04,0x04,0x04,0x0E}, // 1
    {0x1E,0x21,0x20,0x10,0x08,0x04,0x02,0x01,0x3F}, // 2
    {0x1E,0x21,0x20,0x10,0x1C,0x10,0x20,0x21,0x1E}, // 3
    {0x10,0x18,0x14,0x12,0x11,0x3F,0x10,0x10,0x10}, // 4
    {0x3F,0x01,0x01,0x1F,0x20,0x20,0x20,0x21,0x1E}, // 5
    {0x1E,0x21,0x01,0x01,0x1F,0x21,0x21,0x21,0x1E}, // 6
    {0x3F,0x20,0x10,0x08,0x04,0x04,0x04,0x04,0x04}, // 7
    {0x1E,0x21,0x21,0x21,0x1E,0x21,0x21,0x21,0x1E}, // 8
    {0x1E,0x21,0x21,0x21,0x3E,0x20,0x20,0x21,0x1E}, // 9
};
/* 대문자 A~Z (5×9, col0=좌). 소문자는 _pchar8 에서 대문자로 매핑. */
static const uint8_t kG_AZ[26][9] = {
    {0x0E,0x11,0x11,0x11,0x1F,0x11,0x11,0x11,0x11}, // A
    {0x0F,0x11,0x11,0x0F,0x11,0x11,0x11,0x11,0x0F}, // B
    {0x0E,0x11,0x01,0x01,0x01,0x01,0x01,0x11,0x0E}, // C
    {0x0F,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x0F}, // D
    {0x1F,0x01,0x01,0x01,0x0F,0x01,0x01,0x01,0x1F}, // E
    {0x1F,0x01,0x01,0x01,0x0F,0x01,0x01,0x01,0x01}, // F
    {0x0E,0x11,0x01,0x01,0x19,0x11,0x11,0x11,0x0E}, // G
    {0x11,0x11,0x11,0x11,0x1F,0x11,0x11,0x11,0x11}, // H
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x0E}, // I
    {0x1C,0x08,0x08,0x08,0x08,0x08,0x09,0x09,0x06}, // J
    {0x11,0x09,0x05,0x03,0x03,0x05,0x09,0x09,0x11}, // K
    {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x1F}, // L
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11,0x11,0x11}, // M
    {0x11,0x13,0x13,0x15,0x15,0x15,0x19,0x19,0x11}, // N
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // O
    {0x0F,0x11,0x11,0x11,0x0F,0x01,0x01,0x01,0x01}, // P
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x15,0x09,0x16}, // Q
    {0x0F,0x11,0x11,0x0F,0x05,0x09,0x09,0x11,0x11}, // R
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x10,0x11,0x0E}, // S
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04}, // T
    {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // U
    {0x11,0x11,0x11,0x11,0x0A,0x0A,0x0A,0x04,0x04}, // V
    {0x11,0x11,0x11,0x11,0x15,0x15,0x15,0x1B,0x11}, // W
    {0x11,0x11,0x0A,0x0A,0x04,0x0A,0x0A,0x11,0x11}, // X
    {0x11,0x11,0x0A,0x0A,0x04,0x04,0x04,0x04,0x04}, // Y
    {0x1F,0x10,0x08,0x08,0x04,0x02,0x02,0x01,0x1F}, // Z
};
static const uint8_t kG_dot[9]   = {0,0,0,0,0,0,0,0x06,0x06};
static const uint8_t kG_pct[9]   = {0x23,0x13,0x08,0x04,0x02,0x19,0x18,0,0};
static const uint8_t kG_dash[9]  = {0,0,0,0,0x1E,0,0,0,0};
static const uint8_t kG_slash[9] = {0x20,0x10,0x10,0x08,0x04,0x04,0x02,0x02,0x01};
static const uint8_t kG_gt[9]    = {0x01,0x02,0x04,0x08,0x10,0x08,0x04,0x02,0x01};
static const uint8_t kG_colon[9] = {0,0,0x06,0x06,0,0x06,0x06,0,0};

static const uint8_t *_gothic_glyph(char c) {
    if (c >= '0' && c <= '9') return kG_dig[c - '0'];
    if (c >= 'A' && c <= 'Z') return kG_AZ[c - 'A'];
    switch (c) {
        case '.': return kG_dot;
        case '%': return kG_pct;
        case '-': return kG_dash;
        case '/': return kG_slash;
        case '>': return kG_gt;
        case ':': return kG_colon;
        default:  return NULL;   // 공백 등 → 빈칸
    }
}

/* 산세리프(고딕) 글자 (셀 8-wide, 글리프 6×9 — advance/위치 불변).
 *  소문자는 대문자 글리프로 매핑. color=true: ON / false: OFF(반전박스용) */
static void _pchar8(int x, int y, char c, bool color) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    const uint8_t *g = _gothic_glyph(c);
    if (!g) return;
    for (int r = 0; r < 9; r++)
        for (int col = 0; col < 6; col++)
            if ((g[r] >> col) & 1) _px(x + 1 + col, y + r, color);
}
static int _pstr8(int x, int y, const char *s, bool color) {
    while (*s) { _pchar8(x, y, *s++, color); x += 8; }
    return x;
}
static int _pstr8_w(const char *s) { return (int)strlen(s) * 8; }
static void _pstr8_center(int y, const char *s) {
    _pstr8((OLED_PANEL_W - _pstr8_w(s)) / 2, y, s, true);
}

/* 7-세그먼트 큰 숫자 (셀 13w×23h) — 시계용, 또렷하고 블록감 없음 */
static void _draw_7seg(int x, int y, int d) {
    static const uint8_t seg[10] = {0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B};
    if (d < 0 || d > 9) return;
    uint8_t s = seg[d];
    const int T = 3, hv = 7, W = 13;
    if (s & 0x40) _pfill(x + T,     y,              W - 2*T, T,  true); // a 상단
    if (s & 0x02) _pfill(x,         y + T,          T,       hv, true); // f 좌상
    if (s & 0x20) _pfill(x + W - T, y + T,          T,       hv, true); // b 우상
    if (s & 0x01) _pfill(x + T,     y + T + hv,     W - 2*T, T,  true); // g 중간
    if (s & 0x04) _pfill(x,         y + 2*T + hv,   T,       hv, true); // e 좌하
    if (s & 0x10) _pfill(x + W - T, y + 2*T + hv,   T,       hv, true); // c 우하
    if (s & 0x08) _pfill(x + T,     y + 2*T + 2*hv, W - 2*T, T,  true); // d 하단
}
static void _draw_colon(int x, int y) {   // 셀 23-tall 기준 위/아래 점(3×3)
    _pfill(x, y + 6,  3, 3, true);
    _pfill(x, y + 15, 3, 3, true);
}
#endif /* OLED_RENDER_NATIVE */

static void _fb_draw_icon(int x, int y, const uint8_t *icon) {
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            bool on = (icon[row] >> (7 - col)) & 1;
            _fb_set_pixel(x + col, y + row, on);
        }
    }
}

/* ── 5×7 narrow 폰트 (default) ──
 * 글리프는 column-major (5 byte = 5 column, 각 byte = 7 bit row).
 * 화면이 72×40으로 작기 때문에 기본 글자 출력에 5×7을 사용한다.
 * pitch = 6 px (글자 5 + 1 spacing), 높이 = 7 px.
 * 8×8 폰트가 필요할 때만 _fb_draw_char_8x8 호출. */
#define FONT_W 6
#define FONT_H 8

static void _fb_draw_char(int x, int y, char c) {
    if (c < 0x20 || c > 0x7E) return;
    const uint8_t *g = font5x7_basic[(uint8_t)c - 0x20];
    for (int col = 0; col < 5; col++) {
        uint8_t line = g[col];
        for (int row = 0; row < 7; row++) {
            bool on = (line >> row) & 1;
            _fb_set_pixel(x + col, y + row, on);
        }
    }
}

static void _fb_draw_string(int x, int y, const char *str) {
#if OLED_RENDER_NATIVE
    /* 128×64/64×128 네이티브: 논리(72×40) 좌표를 물리 패널로 매핑하고 또렷한 8×8
     *  폰트(_pstr8)로 출력한다. 논리 5×7 폰트를 _fb_set_pixel 이 1.78배 비정수
     *  확대하면 글자가 깨져 보이는 문제(설정 항목 상세 화면 등)를 native 로 해결. */
    _pstr8((x * OLED_PANEL_W) / OLED_WIDTH, (y * OLED_PANEL_H) / OLED_HEIGHT, str, true);
#else
    while (*str) {
        _fb_draw_char(x, y, *str++);
        x += FONT_W;
        if (x + FONT_W > OLED_WIDTH) break;
    }
#endif
}

/* 8×8 폰트 — 강조/큰 글자용 (액션 화면 등). 기존 font8x8_basic 사용. */
__attribute__((unused))
static void _fb_draw_char_8x8(int x, int y, char c) {
    if (c < 0x20 || c > 0x7F) return;
    const uint8_t *glyph = font8x8_basic[(uint8_t)c - 0x20];
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            bool on = (glyph[row] >> col) & 1;
            _fb_set_pixel(x + col, y + row, on);
        }
    }
}

/* 2× 스케일 5×7 폰트 (10×14) — _render_normal 에서 사용.
 * 정의는 파일 하단에 있음, 전방 선언만 추가. */
static void _fb_draw_char_2x(int x, int y, char c);
static void _fb_draw_string_2x(int x, int y, const char *s);

/* 4×7 작은 폰트 (숫자 전용, 0-9) */
static const uint8_t s_small_digits[10][7] = {
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}, // 0
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // 1
    {0x1E,0x01,0x01,0x1E,0x10,0x10,0x1F}, // 2
    {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}, // 3
    {0x11,0x11,0x11,0x1F,0x01,0x01,0x01}, // 4
    {0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E}, // 5
    {0x1E,0x10,0x10,0x1E,0x11,0x11,0x1E}, // 6
    {0x1F,0x01,0x02,0x04,0x04,0x04,0x04}, // 7
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, // 8
    {0x1E,0x11,0x11,0x1F,0x01,0x01,0x1E}, // 9
};

static void _fb_draw_small_digit(int x, int y, int d, bool invert) {
    for (int row = 0; row < 7; row++) {
        uint8_t line = s_small_digits[d][row];
        for (int col = 0; col < 5; col++) {
            bool on = (line >> (4 - col)) & 1;
            if (invert) on = !on;
            _fb_set_pixel(x + col, y + row, on);
        }
    }
}

__attribute__((unused))
static void _fb_draw_small_number(int x, int y, int n, bool invert) {
    _fb_draw_small_digit(x,     y, n / 10, invert);
    _fb_draw_small_digit(x + 6, y, n % 10, invert);
}

/* 수평선 그리기 */
static void _fb_hline(int x0, int x1, int y) {
    for (int x = x0; x <= x1; x++) _fb_set_pixel(x, y, true);
}

/* 사각형 윤곽 */
static void _fb_rect(int x, int y, int w, int h) {
    _fb_hline(x, x + w - 1, y);
    _fb_hline(x, x + w - 1, y + h - 1);
    for (int i = y; i < y + h; i++) {
        _fb_set_pixel(x, i, true);
        _fb_set_pixel(x + w - 1, i, true);
    }
}

/* 채워진 사각형 */
static void _fb_fill_rect(int x, int y, int w, int h) {
    for (int cy = y; cy < y + h; cy++)
        for (int cx = x; cx < x + w; cx++)
            _fb_set_pixel(cx, cy, true);
}

/* 원 (Midpoint circle) */
static void _fb_circle(int cx, int cy, int r, bool fill) {
    for (int y = -r; y <= r; y++)
        for (int x = -r; x <= r; x++)
            if (fill ? (x*x + y*y <= r*r) : (abs(x*x + y*y - r*r) <= r + 1))
                _fb_set_pixel(cx + x, cy + y, true);
}

/* OLED_ROTATE_180: 사용자가 디바이스를 거꾸로 들고 사용할 수 있도록
 * 프레임 버퍼를 180° 회전하여 OLED에 전송한다.
 *
 * 회전 변환 (각 픽셀 (x, y) → (W-1-x, H-1-y)):
 *   1. 페이지 순서 역순:   page p → row (PAGES-1 - p)
 *   2. 페이지 내 컬럼 역순: col c → col (WIDTH-1 - c)
 *   3. 페이지 내 바이트 비트 역순 (8픽셀 vertical column flip)
 *
 * 회전 여부는 보드별: boards/<board>.h 의 BOARD_OLED_ROTATE_180
 * (oled_ui.h 에서 OLED_ROTATE_180 으로 전달). GNPE=1(거꾸로 장착), XIAO=0(정방향).
 * 향후 사용자 설정으로 노출하려면 NVS 변수로 대체 가능. */

static inline uint8_t _bit_reverse_8(uint8_t b) {
    b = (uint8_t)((b >> 4) | (b << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

/* 프레임 버퍼에 페이드 마스크 적용 (모노크롬 → 4×4 Bayer 디더링).
 *  level 0..16: 0=완전 사라짐, 16=완전 표시. 화면 보호기 fade in/out 용.
 *  level 이 임계보다 큰 픽셀만 남겨 '서서히 나타남/사라짐' 효과. */
static void _fb_apply_fade(uint8_t level) {
    if (level >= 16) return;            /* 완전 표시 — 마스크 불필요 */
    if (level == 0) { _fb_clear(); return; }
    static const uint8_t bayer4[4][4] = {
        {  0,  8,  2, 10 },
        { 12,  4, 14,  6 },
        {  3, 11,  1,  9 },
        { 15,  7, 13,  5 },
    };
    for (int y = 0; y < OLED_PANEL_H; y++) {          /* 물리 버퍼 전체 디더 */
        for (int x = 0; x < OLED_PANEL_W; x++) {
            if (bayer4[y & 3][x & 3] >= level) {
                int page = y / 8, bit = y % 8;
                s_fb[page][x] &= ~(1 << bit);   /* 디더 컷오프 → 픽셀 off */
            }
        }
    }
}


/* I2C 버스 스캔 — 응답하는 주소를 로그로 덤프(OLED 미검출 진단용). */
static void _oled_i2c_scan(void) {
    char buf[80]; int n = 0; buf[0] = '\0';
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(s_dev._i2c_bus_handle, a, 20) == ESP_OK) {
            n += snprintf(buf + n, sizeof(buf) - n, "0x%02X ", a);
            if (n >= (int)sizeof(buf) - 6) break;
        }
    }
    if (buf[0]) ESP_LOGW(TAG, "I2C 스캔: 응답 주소 = %s(OLED 는 보통 0x3C, 변종 0x3D)", buf);
    else        ESP_LOGW(TAG, "I2C 스캔: 응답 장치 없음 — OLED SDA→IO%d / SCL→IO%d / VCC→3V3 / GND 배선 확인",
                         BOARD_PIN_OLED_SDA, BOARD_PIN_OLED_SCL);
}

/* ── 2026-07-17 추가: 물린(stuck) I2C 버스 라인 레벨 관찰 ────────────────────
 *  I2C 페리페럴이 핀을 잡고 있어도 **GPIO 입력 레지스터는 실제 패드 레벨**을 읽으므로
 *  버스를 건드리지 않고 관찰할 수 있다. 판독법:
 *    SDA=1 SCL=1 → 버스 idle(정상). 이 상태에서 probe 실패면 모듈 무응답(NACK).
 *    SDA=0 SCL=1 → **슬레이브가 SDA 를 물고 있음** → 9클럭 복구로 풀 수 있다.
 *    SDA=0 SCL=0 → 둘 다 눌림 → 전원/단락 계열(소프트로 복구 불가).
 *  ※`probe device timeout` 은 NACK 이 아니라 **전송 미완료** = 버스가 물렸다는 뜻. */
static int _oled_log_lines(const char *when) {
    int sda = gpio_get_level(BOARD_PIN_OLED_SDA);
    int scl = gpio_get_level(BOARD_PIN_OLED_SCL);
    ESP_LOGW(TAG, "[OLED] 라인레벨 %s: SDA(IO%d)=%d SCL(IO%d)=%d  (1/1=idle, 0/x=물림)",
             when, BOARD_PIN_OLED_SDA, sda, BOARD_PIN_OLED_SCL, scl);
    return sda;   /* 호출부가 **같은 판독값**으로 판단하도록 돌려준다.
                   * (따로 gpio_get_level 을 또 부르면 그 사이 레벨이 달라져
                   *  "SDA=1 로 찍어놓고 물림으로 분기" 하는 모순이 난다 — 실제로 겪음) */
}

/* ── 2026-07-17 추가: 비트뱅 ACK 진단 (검출 실패 시 부팅 1회) ────────────────
 *  왜 필요한가: `i2c_master_probe` 가 **NACK 이 아니라 TIMEOUT**(= 전송 미완료) 으로
 *  실패한다. 빈 버스면 NACK 이 즉시 나야 하므로, TIMEOUT 은 **마스터(페리페럴) 쪽**이
 *  전송을 못 끝낸다는 뜻이다. 모듈 탓인지 마스터 탓인지 소프트로 가른다:
 *    ①핀 구동 검사: SCL/SDA 를 직접 LOW 로 끌고 되읽는다. 1 이 나오면 **핀이 안 눌린다**
 *      (= 페리페럴/핀 설정 문제. 라인이 어딘가에 강하게 물려 있어도 이렇게 나온다).
 *    ②ACK 검사: START + 0x3C<<1 을 손으로 쳐서 9번째 클럭에 SDA 를 읽는다.
 *      SDA=0 → **모듈이 ACK = 살아있음** → I2C 페리페럴만 고장 → 소프트로 해결 가능.
 *      SDA=1 → 모듈이 주소에 응답 안 함.
 *  ※관찰 전용. 진단 후 버스를 원래대로 재생성한다. */
#define OLED_BB_DLY_US 5
static void _oled_bb_scl(int v) { gpio_set_level(BOARD_PIN_OLED_SCL, v); esp_rom_delay_us(OLED_BB_DLY_US); }
static void _oled_bb_sda(int v) { gpio_set_level(BOARD_PIN_OLED_SDA, v); esp_rom_delay_us(OLED_BB_DLY_US); }

static void _oled_bitbang_diag(void) {
    ESP_LOGW(TAG, "[OLED-DIAG] ── 비트뱅 진단 시작 (I2C 페리페럴 분리) ──");
    if (s_dev._i2c_bus_handle) {          /* 핀을 페리페럴에서 떼어내야 GPIO 로 몰 수 있다 */
        i2c_del_master_bus(s_dev._i2c_bus_handle);
        s_dev._i2c_bus_handle = NULL;
    }
    /* ★gpio_config 는 IOMUX/GPIO매트릭스 라우팅을 **떼지 않는다**. 페리페럴이 아직 핀을
     *  몰고 있으면 gpio_set_level 이 안 먹혀 "핀이 안 눌린다" 는 **가짜 결론**이 난다.
     *  → gpio_reset_pin 으로 순수 GPIO 로 되돌린 뒤 측정한다. */
    gpio_reset_pin(BOARD_PIN_OLED_SCL);
    gpio_reset_pin(BOARD_PIN_OLED_SDA);

    /* ★★2026-07-17: hold / sleep-isolate 해제 후 재측정.
     *  부팅 로그에 다음이 찍힌다(= light sleep + Thread SED 로 전원관리 활성):
     *     sleep_gpio: Configure to isolate all GPIO pins in sleep state
     *     sleep_gpio: Enable automatic switching of GPIO sleep configuration
     *  핀이 hold 되거나 sleep 설정으로 자동 전환되면 **gpio_set_level 이 먹지 않는다**
     *  (출력이 래치/격리됨) → 마스터가 SCL 을 못 내려 전송이 시작조차 안 되고
     *  probe 가 **NACK 이 아니라 TIMEOUT** 으로 실패한다 = 지금 증상과 정확히 일치.
     *  여기서 풀고 나서 SCL 이 LOW 로 떨어지면 **원인은 이 메커니즘(소프트)** 이다. */
    /* ★★hold 레지스터를 **직접** 읽는다. hold 되면 레지스터(OUT_EN/OUT)는 자유롭게 써지지만
     *  패드는 래치된 값에 고정 → `OUT_EN=1 OUT=0 인데 패드=1` 이라는 정확한 지문이 남는다.
     *  ※gpio_hold_dis() 는 반환값을 반드시 확인할 것(핀 종류에 따라 미지원일 수 있다). */
    uint32_t hold_before = REG_READ(LP_AON_GPIO_HOLD0_REG);
    esp_err_t h1 = gpio_hold_dis(BOARD_PIN_OLED_SCL);
    esp_err_t h2 = gpio_hold_dis(BOARD_PIN_OLED_SDA);
    /* ※gpio_deep_sleep_hold_dis() 는 C6 에 없다(빌드 에러) — gpio_hold_dis 로 충분. */
    uint32_t hold_after = REG_READ(LP_AON_GPIO_HOLD0_REG);
    ESP_LOGW(TAG, "[OLED-DIAG] ★HOLD0 레지스터: 전=0x%08lX 후=0x%08lX  "
                  "(IO%d bit=%d→%d, IO%d bit=%d→%d)  hold_dis 반환: SCL=%s SDA=%s",
             (unsigned long)hold_before, (unsigned long)hold_after,
             BOARD_PIN_OLED_SCL, (int)((hold_before >> BOARD_PIN_OLED_SCL) & 1),
                                 (int)((hold_after  >> BOARD_PIN_OLED_SCL) & 1),
             BOARD_PIN_OLED_SDA, (int)((hold_before >> BOARD_PIN_OLED_SDA) & 1),
                                 (int)((hold_after  >> BOARD_PIN_OLED_SDA) & 1),
             esp_err_to_name(h1), esp_err_to_name(h2));
#if SOC_GPIO_SUPPORT_SLP_SWITCH
    gpio_sleep_sel_dis(BOARD_PIN_OLED_SCL);
    gpio_sleep_sel_dis(BOARD_PIN_OLED_SDA);
#endif

    /* ★★외부가 핀을 붙잡고 있는지 판별 — 내부 풀다운(약 45k) 으로 당겨 보고 읽는다.
     *  1 이면 **외부가 강하게 HIGH 로 몰고 있다**(45k 를 이김) = 소프트로 어쩔 수 없음.
     *  0 이면 외부 구동원 없음 = 핀은 자유롭다(내 출력 경로 문제).
     *  ※IO22(SDA) 를 **대조군**으로 같이 잰다 — 같은 코드/같은 순간에 22 는 정상이고
     *    23 만 이상하면 방법론이 아니라 그 핀이 이상한 것이다. */
    gpio_config_t in = { .pin_bit_mask = (1ULL << BOARD_PIN_OLED_SCL) | (1ULL << BOARD_PIN_OLED_SDA),
                         .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_ENABLE };
    gpio_config(&in);
    esp_rom_delay_us(200);
    ESP_LOGW(TAG, "[OLED-DIAG] ⓪내부풀다운으로 당김: SDA(IO%d)=%d SCL(IO%d)=%d  "
                  "(0=자유 / 1=외부가 HIGH 로 붙잡음)",
             BOARD_PIN_OLED_SDA, gpio_get_level(BOARD_PIN_OLED_SDA),
             BOARD_PIN_OLED_SCL, gpio_get_level(BOARD_PIN_OLED_SCL));

    /* ★INPUT_OUTPUT_OD 필수 — OUTPUT_OD 는 입력버퍼가 꺼져 gpio_get_level 이 늘 0 이다. */
    gpio_config_t io = { .pin_bit_mask = (1ULL << BOARD_PIN_OLED_SCL) | (1ULL << BOARD_PIN_OLED_SDA),
                         .mode = GPIO_MODE_INPUT_OUTPUT_OD,
                         .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE };
    gpio_config(&io);
    _oled_bb_sda(1); _oled_bb_scl(1);
    ESP_LOGW(TAG, "[OLED-DIAG] ①idle(풀업만): SDA=%d SCL=%d  (1/1 이어야 정상)",
             gpio_get_level(BOARD_PIN_OLED_SDA), gpio_get_level(BOARD_PIN_OLED_SCL));

    /* ★★2026-07-17: 레지스터 직접 확인 — "설정했다고 믿는 것"과 "칩이 실제로 하는 것"을
     *  가른다. OUT_EN=1 인데도 패드가 1 이면 소프트로는 더 할 게 없다는 뜻이고,
     *  OUT_EN=0 이면 **뭔가가 출력을 꺼버린 것**(소프트로 해결 가능). */
    _oled_bb_scl(0);   /* SCL 을 LOW 로 몰아둔 상태에서 레지스터를 본다 */
    for (int i = 0; i < 2; i++) {
        int n = i ? BOARD_PIN_OLED_SCL : BOARD_PIN_OLED_SDA;
        ESP_LOGW(TAG, "[OLED-DIAG] ★IO%d 레지스터: OUT_EN=%d OUT=%d IN(패드)=%d "
                      "func_out_sel=0x%02lX iomux_fn=%lu (OUT_EN=1,OUT=0,sel=128 이어야 정상)",
                 n,
                 (int)((REG_READ(GPIO_ENABLE_REG) >> n) & 1),
                 (int)((REG_READ(GPIO_OUT_REG)    >> n) & 1),
                 (int)((REG_READ(GPIO_IN_REG)     >> n) & 1),
                 (unsigned long)(REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * n) & 0xFF),
                 (unsigned long)((REG_READ(IO_MUX_GPIO0_REG + 4 * n) >> 12) & 0x7));
    }
    _oled_bb_scl(1);

    /* ★푸시풀 구동 시험 — OD 는 N-FET 만 쓴다. 혹시 OD 설정이 실제로 안 먹는 것이라면
     *  푸시풀에서는 LOW 가 나온다(= 소프트 문제). 같은 N-FET 을 쓰므로 여기서도 1 이면
     *  드라이버가 전류를 못 흘리는 것이다. 짧게만 시험(단락 시 전류 시간 최소화). */
    gpio_set_direction(BOARD_PIN_OLED_SCL, GPIO_MODE_INPUT_OUTPUT);  /* 푸시풀 + 입력버퍼 */
    gpio_set_level(BOARD_PIN_OLED_SCL, 0); esp_rom_delay_us(20);
    int pp_lo = gpio_get_level(BOARD_PIN_OLED_SCL);
    gpio_set_level(BOARD_PIN_OLED_SCL, 1); esp_rom_delay_us(20);
    int pp_hi = gpio_get_level(BOARD_PIN_OLED_SCL);
    ESP_LOGW(TAG, "[OLED-DIAG] ★푸시풀 SCL lo/hi=%d/%d  (0/1 이면 OD 설정 문제=소프트 해결 가능)",
             pp_lo, pp_hi);
    gpio_config(&io);   /* OD 로 복구 */

    _oled_bb_scl(0);
    int scl_lo = gpio_get_level(BOARD_PIN_OLED_SCL);
    _oled_bb_scl(1);
    int scl_hi = gpio_get_level(BOARD_PIN_OLED_SCL);
    _oled_bb_sda(0);
    int sda_lo = gpio_get_level(BOARD_PIN_OLED_SDA);
    _oled_bb_sda(1);
    int sda_hi = gpio_get_level(BOARD_PIN_OLED_SDA);
    ESP_LOGW(TAG, "[OLED-DIAG] ②핀구동: SCL lo/hi=%d/%d, SDA lo/hi=%d/%d  (0/1 이어야 정상)",
             scl_lo, scl_hi, sda_lo, sda_hi);

    /* ③START + addr(0x3C<<1|W=0x78) + ACK 읽기 — 손으로 친다 */
    _oled_bb_sda(1); _oled_bb_scl(1);
    _oled_bb_sda(0); _oled_bb_scl(0);                 /* START */
    uint8_t byte = (0x3C << 1) | 0;
    for (int i = 7; i >= 0; i--) {
        _oled_bb_sda((byte >> i) & 1);
        _oled_bb_scl(1); _oled_bb_scl(0);
    }
    _oled_bb_sda(1);                                   /* SDA 놓고 ACK 받기 */
    _oled_bb_scl(1);
    int ack = gpio_get_level(BOARD_PIN_OLED_SDA);      /* 0 = ACK = 모듈 살아있음 */
    _oled_bb_scl(0);
    _oled_bb_sda(0); _oled_bb_scl(1); _oled_bb_sda(1); /* STOP */
    ESP_LOGW(TAG, "[OLED-DIAG] ③0x3C ACK 비트=%d → %s", ack,
             ack == 0 ? "★ACK! 모듈 살아있음 = I2C 페리페럴 문제(소프트 해결 가능)"
                      : "무응답(모듈이 주소에 답 안 함)");
    ESP_LOGW(TAG, "[OLED-DIAG] ── 끝. 버스 재생성 ──");
    i2c_master_init(&s_dev, OLED_PIN_SDA, OLED_PIN_SCL, -1);
}

/* ── 2026-07-17 추가: 물린 버스 9클럭 복구 ──────────────────────────────────
 *  ★`i2c_master_bus_reset()` 은 C6 에서 stuck 슬레이브를 못 푼다: IDF 의
 *  `s_i2c_hw_fsm_reset()` 안에서 9클럭 복구(`s_i2c_master_clear_bus()`)가
 *  `#if !SOC_I2C_SUPPORT_HW_FSM_RST` 로 감싸져 있는데 **C6 는 =1** 이라 건너뛰고
 *  FSM 만 리셋한다. → 직접 비트뱅으로 푼다.
 *  왜 필요한가: ESP32 가 전송 도중 리셋되면(USB 재연결 등) 슬레이브가 SDA 를 문 채
 *  클럭을 기다린다. 다음 부팅에서 마스터는 SDA 가 눌린 걸 보고 **모든 전송이 timeout**
 *  → OLED 영구 미검출. 클럭 9개를 억지로 넣어 슬레이브가 남은 비트를 뱉게 하고 STOP.
 *  ※핀을 페리페럴에서 떼야 비트뱅이 먹히므로 **버스 del → 비트뱅 → 재생성** 순서 필수. */
/* ★2026-07-23 기본 ON 으로 전환.
 *  과거 OFF 이유: "정상 idle 버스에 9클럭을 쓰면 해로웠다"(에러 258 자작).
 *  → 이제 **검출 실패 시에만** 호출하도록 _fb_flush() 재검출 경로에서 조건부 실행하므로
 *    정상 버스를 건드리지 않는다. 실기(COM7)에서 모듈이 "버스 정상인데 무응답"으로
 *    고착돼 화면이 멈추는 현상이 반복 확인됐고, 9클럭+STOP 반복으로 전원차단 없이
 *    되살아나는 것을 실측했다. 되돌리려면 0. */
#ifndef OLED_BUS_RECOVER_ENABLE
#define OLED_BUS_RECOVER_ENABLE 1
#endif
#if OLED_BUS_RECOVER_ENABLE
static void _oled_i2c_bus_recover(void) {
    ESP_LOGW(TAG, "[OLED] 버스 물림 감지 → 9클럭 복구 시도");
    /* ★2026-07-23 크래시 수정(실기 재부팅 루프 유발):
     *  이전 코드는 device 핸들을 남긴 채 i2c_del_master_bus() 만 불렀다.
     *  → "Bus not freed entirely" 로 삭제 실패 → 이어지는 i2c_master_init() 의
     *    i2c_new_master_bus() 가 "bus id(0) has already been acquired" 로 실패 →
     *    라이브러리의 ESP_ERROR_CHECK 가 **abort()** → 패닉 재부팅 반복.
     *  반드시 **device 제거 → 버스 삭제** 순서로 내리고, 실패하면 복구를 포기한다
     *  (abort 금지 — 화면만 못 쓰지 기기는 계속 동작해야 한다). */
    if (s_dev._i2c_dev_handle) {
        esp_err_t rd = i2c_master_bus_rm_device(s_dev._i2c_dev_handle);
        if (rd != ESP_OK) {
            ESP_LOGW(TAG, "[OLED] device 제거 실패(%s) — 복구 중단", esp_err_to_name(rd));
            return;
        }
        s_dev._i2c_dev_handle = NULL;
    }
    if (s_dev._i2c_bus_handle) {                 /* 핀을 페리페럴에서 떼어낸다 */
        esp_err_t rb = i2c_del_master_bus(s_dev._i2c_bus_handle);
        if (rb != ESP_OK) {
            ESP_LOGW(TAG, "[OLED] 버스 삭제 실패(%s) — 복구 중단", esp_err_to_name(rb));
            return;                               /* 핸들 유지: 다음 기회에 재시도 */
        }
        s_dev._i2c_bus_handle = NULL;
    }
    /* ★GPIO_MODE_**INPUT_**OUTPUT_OD 여야 한다. OUTPUT_OD 는 **입력 버퍼가 꺼져** 있어
     *  gpio_get_level() 이 실제 패드 레벨이 아니라 늘 0 을 준다(실측으로 당함:
     *  "복구후 SDA=0 SCL=0" 은 진짜 눌린 게 아니라 **못 읽은 것**이었고, 아래 9클럭
     *  루프의 조기 종료도 영영 안 걸렸다). open-drain 이라 출력 기능은 동일. */
    gpio_config_t io = { .pin_bit_mask = (1ULL << BOARD_PIN_OLED_SCL),
                         .mode = GPIO_MODE_INPUT_OUTPUT_OD, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&io);
    io.pin_bit_mask = (1ULL << BOARD_PIN_OLED_SDA);
    gpio_config(&io);
    gpio_set_level(BOARD_PIN_OLED_SDA, 1);
    for (int i = 0; i < 9; i++) {                /* 9클럭: 슬레이브가 남은 비트를 뱉음 */
        gpio_set_level(BOARD_PIN_OLED_SCL, 0); esp_rom_delay_us(5);
        gpio_set_level(BOARD_PIN_OLED_SCL, 1); esp_rom_delay_us(5);
        if (gpio_get_level(BOARD_PIN_OLED_SDA)) break;   /* 풀렸으면 조기 종료 */
    }
    gpio_set_level(BOARD_PIN_OLED_SDA, 0); esp_rom_delay_us(5);   /* STOP: SCL↑ 중 SDA↑ */
    gpio_set_level(BOARD_PIN_OLED_SCL, 1); esp_rom_delay_us(5);
    gpio_set_level(BOARD_PIN_OLED_SDA, 1); esp_rom_delay_us(5);
    _oled_log_lines("복구후");
    /* 버스+device 재생성 — ★라이브러리 i2c_master_init() 은 내부가 ESP_ERROR_CHECK 라
     *  실패 시 abort() 한다(위 크래시의 직접 원인). 여기서는 동일 설정을 직접 만들되
     *  **에러를 반환값으로 처리**해 절대 abort 하지 않는다.
     *  (패널 init 은 하지 않는다 — 검출 성공 후 _oled_panel_init 담당.) */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = BOARD_PIN_OLED_SCL,
        .sda_io_num = BOARD_PIN_OLED_SDA,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t e = i2c_new_master_bus(&bus_cfg, &bus);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "[OLED] 버스 재생성 실패(%s) — 다음 주기에 재시도", esp_err_to_name(e));
        return;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BOARD_OLED_ADDR,
        .scl_speed_hz    = OLED_I2C_HZ,
    };
    i2c_master_dev_handle_t devh = NULL;
    e = i2c_master_bus_add_device(bus, &dev_cfg, &devh);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "[OLED] device 재등록 실패(%s) — 버스 정리 후 재시도", esp_err_to_name(e));
        i2c_del_master_bus(bus);
        return;
    }
    s_dev._address        = BOARD_OLED_ADDR;
    s_dev._i2c_num        = I2C_NUM_0;
    s_dev._i2c_bus_handle = bus;
    s_dev._i2c_dev_handle = devh;
}
#endif /* OLED_BUS_RECOVER_ENABLE */

/* OLED 검출 시도: 0x3C 우선, 없으면 0x3D(변종). 검출 시 해당 주소로 device handle
 * 재지정 후 패널 init + s_oled_present=true. (반환: 검출 여부) */
static bool _oled_try_detect(void) {
    uint8_t addr = 0;
    /* ★2026-07-23 실측 근거로 재시도 추가 — "화면 멈춤"의 직접 원인이었다.
     *  로그에서 모순이 잡혔다: 같은 버스에서
     *      [OLED] 검출실패 (probe 0x3C 1회 → 실패)
     *      I2C 스캔: 응답 주소 = 0x3C      ← 바로 뒤 스캔은 성공
     *  즉 **모듈은 살아 있는데 첫 probe 만 실패**한다(첫 트랜잭션 글리치).
     *  기존 코드는 주소당 1회만 시도하고 포기 → s_oled_present=false 유지 →
     *  화면이 마지막 프레임에 멈춘 채로 남았다.
     *  → 각 주소를 여러 번 시도한다. 스캔이 찾아내는 것과 동일한 조건이 된다. */
    for (int t = 0; t < 4 && !addr; t++) {
#if BOARD_OLED_BITBANG
        if      (_bbo_probe(0x3C)) addr = 0x3C;
        else if (_bbo_probe(0x3D)) addr = 0x3D;
#else
        if      (i2c_master_probe(s_dev._i2c_bus_handle, 0x3C, 80) == ESP_OK) addr = 0x3C;
        else if (i2c_master_probe(s_dev._i2c_bus_handle, 0x3D, 80) == ESP_OK) addr = 0x3D;
#endif
        if (!addr) vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!addr) {
        /* 2026-07-17: 실패 원인을 라인 레벨로 구분한다(관찰만 — 버스를 건드리지 않음). */
        int sda = _oled_log_lines("검출실패");
#if OLED_BUS_RECOVER_ENABLE
        /* ★기본 OFF — 실측 결과 이 보드에선 **해로웠다**(2026-07-17).
         *  전제가 틀렸다: 검출 실패 시점의 라인은 SDA=1 SCL=1 = **idle**(물림 아님)이었다.
         *  물리지도 않은 멀쩡한 버스에 9클럭+STOP 을 쑤셔넣자 SSD1306 에러 258 폭주.
         *  → 진짜로 SDA=0 으로 물리는 사례를 확인하기 전엔 켜지 말 것.
         *  (로직은 지우지 않고 보존 — 다른 보드/사례에서 필요할 수 있음) */
        if (sda == 0) {                                    /* SDA 물림일 때만 */
            _oled_i2c_bus_recover();
            if      (i2c_master_probe(s_dev._i2c_bus_handle, 0x3C, 80) == ESP_OK) addr = 0x3C;
            else if (i2c_master_probe(s_dev._i2c_bus_handle, 0x3D, 80) == ESP_OK) addr = 0x3D;
        }
#else
        (void)sda;
#endif
    }
    if (!addr) return false;
    if (addr != s_dev._address) {
        /* 검출 주소가 기본(0x3C)과 다르면 device handle 을 그 주소로 재지정 */
        i2c_device_add(&s_dev, I2C_NUM_0, -1, addr);
    }
    _oled_panel_init();
    /* 2026-07-19: SSD1306 가 init 직후 곧바로 대용량 flush(8p×129B)를 받으면 아직 덜
     *  깨어나 첫 프레임이 깨지고(부팅 점깨짐), 그 연속 실패가 flush 정지를 유발해 화면이
     *  얼어붙었다(실측). init 후 잠깐 정착시켜 첫 flush 를 안정화한다. */
    vTaskDelay(pdMS_TO_TICKS(200));
    s_oled_present = true; g_oled_present_mon = true; _shadow_invalidate();
    ESP_LOGI(TAG, "OLED 검출 (addr 0x%02X, 패널 %d×%d) — 표시 활성화", addr, OLED_PANEL_W, OLED_PANEL_H);
    return true;
}

/* ★★라이브러리 ssd1306_display_image 대체 — 재시도 + **실패 시 flush 정지**.
 *
 *  왜 필요한가(2026-07-17 실측): OLED 가 간헐적으로 NACK 을 내면 ESP-IDF I2C
 *  드라이버가 esp_driver_i2c/i2c_master.c 의
 *      else if (event == I2C_EVENT_NACK) {
 *          while (i2c_ll_is_bus_busy(hal->dev)) { __asm__ __volatile__("nop"); }
 *      }
 *  에서 **타임아웃 없이 무한 바쁜대기**한다. 이때 oled_ui 가 **I2C 버스 락을 쥔 채**
 *  안 놓으므로, 그 락을 기다리는 somfy_app 이 **블록되어** Task Watchdog 폭주
 *  (실측 65~385건, RA=s_i2c_send_commands, running=oled_ui) → RF·버튼·Matter 정지.
 *  ★우선순위를 낮춰도(3<4) 소용없다 — somfy_app 은 선점당한 게 아니라 **락에 막힌** 것.
 *  ★C6 는 SOC_I2C_SUPPORT_HW_FSM_RST=1 이라 i2c_master_bus_reset 의 9클럭 복구도 건너뜀.
 *  → 드라이버에서 빠져나올 방법이 없으므로 **애초에 죽은 버스를 두드리지 않는 것**이
 *    유일한 해법: 한 프레임 분량을 연속 실패하면 s_oled_present=false 로 내려
 *    _fb_flush 가 통째로 멈추고, 복구는 타임아웃이 있는 5초 재프로브가 맡는다.
 *  라이브러리와 동일 시퀀스(컬럼+페이지 주소 → 데이터)이므로 출력은 같다. */
#define OLED_WR_RETRY 4
/* ★2026-07-19: 데이터 전송을 이 바이트 수로 쪼갠다(마진 버스 글리치 노출↓). 사용자 선택(B).
 *  129바이트 1회(≈13ms@100k) → 32바이트씩(≈3ms) 여러 번. 폭주 심하면 16 으로 낮출 것. */
#define OLED_WR_CHUNK 32
#ifndef CONFIG_OFFSETX
#define CONFIG_OFFSETX 0
#endif
static void _oled_write_page_locked(SSD1306_t *dev, int page, int seg,
                                    const uint8_t *images, int width) {
    if (page >= dev->_pages) return;
    if (seg  >= dev->_width) return;
    int _seg  = seg + CONFIG_OFFSETX;
    int _page = page;
    if (dev->_flip) _page = (dev->_pages - page) - 1;      /* 라이브러리와 동일 */

    uint8_t cmd[4] = { 0x00,                                    /* CMD stream  */
                       (uint8_t)(0x00 + (_seg & 0x0F)),         /* lower col   */
                       (uint8_t)(0x10 + ((_seg >> 4) & 0x0F)),  /* higher col  */
                       (uint8_t)(0xB0 | _page) };               /* page addr   */
    if (width > 128) width = 128;

    static int s_fail_run = 0;              /* 연속 소진 페이지 수 */
    for (int t = 0; t < OLED_WR_RETRY; t++) {
        /* ★청크 전송: 컬럼+페이지 주소(cmd) 1회 → 데이터를 OLED_WR_CHUNK 바이트씩 나눠 전송.
         *  SSD1306 은 write 마다 컬럼 포인터를 자동증가하므로 각 청크에 0x40(DATA)를 붙여
         *  이어 쓰면 라이브러리 한 방 전송과 출력이 동일하다. 짧은 전송 = 글리치 중 깨질
         *  확률↓ → 마진 버스 폭주 감소. 한 청크라도 실패하면 그 시도 중단 후 재시도. */
#if BOARD_OLED_BITBANG
        uint8_t a7 = (uint8_t)(dev->_address ? dev->_address : BOARD_OLED_ADDR);
        bool ok = _bbo_write(a7, cmd, 4);
#else
        bool ok = (i2c_master_transmit(dev->_i2c_dev_handle, cmd, 4, OLED_I2C_TIMEOUT_MS) == ESP_OK);
#endif
        for (int off = 0; ok && off < width; off += OLED_WR_CHUNK) {
            int n = width - off;
            if (n > OLED_WR_CHUNK) n = OLED_WR_CHUNK;
            uint8_t buf[1 + OLED_WR_CHUNK];
            buf[0] = 0x40;                                      /* DATA stream */
            memcpy(&buf[1], images + off, n);
#if BOARD_OLED_BITBANG
            if (!_bbo_write(a7, buf, n + 1)) ok = false;
#else
            if (i2c_master_transmit(dev->_i2c_dev_handle, buf, n + 1, OLED_I2C_TIMEOUT_MS) != ESP_OK) ok = false;
#endif
        }
        if (ok) { s_fail_run = 0; return; }                    /* 전체 성공 */
        esp_rom_delay_us(50);                                  /* 글리치 흡수 후 재시도 */
    }
    /* 한 프레임(OLED_PAGES) 분량 연속 실패 = 패널이 떨어짐 → flush 정지(위 주석 참조).
     * 5초 주기 재프로브(i2c_master_probe, 타임아웃 있음)가 복구를 맡는다. */
    /* 2026-07-19: 부팅 직후(≤4초)엔 flush 정지 안 함 — SSD1306 기동 과도기의 일시적
     *  폭주로 표시를 꺼버리면 화면이 그대로 얼어붙는다(실측: +1.8초 정지 후 영영 멈춤).
     *  계속 flush 하면 클럭이 나가 과도기가 지나간 뒤 자연히 성공한다(+5초 이후 에러 0).
     *  4초 이후의 연속 실패만 진짜 탈락으로 보고 정지시킨다. */
    if (++s_fail_run >= OLED_PAGES && _ms_now() > 4000) {
        s_fail_run = 0;
        s_oled_present = false; g_oled_present_mon = false; _shadow_invalidate();
        ESP_LOGW(TAG, "[OLED] 연속 write 실패 → flush 정지 (5초 주기 재프로브로 자동 복구)");
    }
}

/* 프레임 버퍼 → SSD1306 전송 */
static void _fb_flush(void) {
    /* ★RF 송신 중에는 I2C 를 아예 건드리지 않는다(모듈 고착 방지 — 위 주석 참조).
     *  송신은 1~1.5초라 그동안 화면이 안 갱신되지만, 고착돼 영구히 멈추는 것보다 낫다.
     *  검출(재프로브)도 하지 않는다 — 노이즈 구간의 프로브가 곧 고착 유발 트랜잭션이다. */
    /* ★2026-07-24 RF 송신 중 차단 **해제**.
     *  넣은 이유는 "RF 노이즈가 모듈을 고착시킨다"는 가설이었으나, 이후 이분 탐색에서
     *  RF 를 켜도 멈추지 않고 **충전측정(ADC)이 진범**임이 밝혀져 근거가 사라졌다.
     *  반대로 부작용이 컸다: 버튼을 누르고 있으면 RF 가 연속 송신되는 동안 화면 갱신이
     *  통째로 막혀 **애니메이션이 멈춰 보였다**(손을 떼면 복귀). 그래서 차단을 끈다.
     *  되살리려면 OLED_BLOCK_ON_RF 를 1 로. */
#ifndef OLED_BLOCK_ON_RF
#define OLED_BLOCK_ON_RF 0
#endif
#if OLED_BLOCK_ON_RF
    if (s_rf_tx_active) return;
#else
    (void)s_rf_tx_active;
#endif
    if (!s_oled_present) {
        /* OLED 미검출: ssd1306_display_image 호출 안 함 → I2C NACK 로그 스팸 차단.
         * 5초마다 1회 재검출(0x3C/0x3D) → 나중에 연결하면 자동 활성화(hot-plug). */
        uint32_t now = _ms_now();
        if (now - s_oled_last_probe_ms >= 5000) {
            s_oled_last_probe_ms = now;
            if (!_oled_try_detect()) {   /* 성공 시 내부에서 s_oled_present=true + 로그 */
                /* ★2026-07-23 자동복구: 모듈이 "버스는 정상(SDA/SCL 둘 다 HIGH)인데
                 *  응답만 안 하는" 상태로 고착되는 현상이 실기에서 반복 확인됐다
                 *  (화면이 마지막 프레임에 멈춤). 원인은 슬레이브 I2C 상태머신 고착.
                 *  실측: 표준 9클럭+STOP 복구를 **반복**하면 전원차단 없이 되살아난다
                 *  (1회로는 안 풀리는 경우가 있어 재검출 실패마다 시도).
                 *  검출 실패 시에만 수행하므로 정상 버스를 건드리지 않는다
                 *  (과거 "idle 버스에 쓰면 해로웠다"는 지적을 이 조건으로 회피). */
                s_oled_recover_tries++;
                _oled_i2c_bus_recover();
                if ((s_oled_recover_tries % 6) == 1) {   /* 30초마다 1회만 로그 */
                    ESP_LOGW(TAG, "[OLED] 검출 실패 %u회 → 9클럭 버스복구 시도 중"
                                  " (모듈 고착 자동해제)", (unsigned)s_oled_recover_tries);
                }
            } else if (s_oled_recover_tries) {
                ESP_LOGW(TAG, "[OLED] ★버스복구 성공 — %u회 시도 후 모듈 재검출됨",
                         (unsigned)s_oled_recover_tries);
                s_oled_recover_tries = 0;
            }
        }
        return;
    }
    oled_ui_i2c_lock();   /* 공유 HW I2C: flush 동안 PCF8574 read 가 끼어들지 못하게 직렬화 */
#if OLED_ROTATE_90
    /* 90° 소프트웨어 회전 — 렌더러 FB(논리, 물리와 W↔H swap)를 물리 패널로 재배치.
     *  s_fb 는 OLED_PANEL_W×H(=물리 H×W), 물리 출력은 OLED_PHYS_W×H.
     *  90(시계): 물리(px,py) ← FB(py, PANEL_H-1-px)
     *  270(반시계): 물리(px,py) ← FB(PANEL_W-1-py, px) */
    uint8_t rot[OLED_PHYS_H / 8][OLED_PHYS_W];
    memset(rot, 0, sizeof(rot));
    for (int px = 0; px < OLED_PHYS_W; px++) {
        for (int py = 0; py < OLED_PHYS_H; py++) {
#if OLED_ROTATE_90 == 270
            const int fx = OLED_PANEL_W - 1 - py;
            const int fy = px;
#else
            const int fx = py;
            const int fy = OLED_PANEL_H - 1 - px;
#endif
            if ((s_fb[fy >> 3][fx] >> (fy & 7)) & 1)
                rot[py >> 3][px] |= (uint8_t)(1u << (py & 7));
        }
    }
    for (int p = 0; p < OLED_PHYS_H / 8; p++) {
        if (!_page_dirty(p, rot[p], OLED_PHYS_W)) { s_skip_cnt++; continue; }  /* dirty-page */
        _oled_write_page_locked(&s_dev, p, 0, rot[p], OLED_PHYS_W);
        _shadow_store(p, rot[p], OLED_PHYS_W); s_sent_cnt++;
    }
#elif OLED_FLIP_X && !OLED_ROTATE_180
    /* 좌우(가로) 반전만 — 각 페이지의 열을 역순으로 (x → W-1-x). */
    uint8_t flipped[OLED_PAGES][OLED_PANEL_W];
    for (int p = 0; p < OLED_PAGES; p++) {
        for (int c = 0; c < OLED_PANEL_W; c++) {
            flipped[p][OLED_PANEL_W - 1 - c] = s_fb[p][c];
        }
    }
    for (int p = 0; p < OLED_PAGES; p++) {
        if (!_page_dirty(p, flipped[p], OLED_PANEL_W)) { s_skip_cnt++; continue; }
        _oled_write_page_locked(&s_dev, p, 0, flipped[p], OLED_PANEL_W);
        _shadow_store(p, flipped[p], OLED_PANEL_W); s_sent_cnt++;
    }
#elif OLED_FLIP_X && OLED_ROTATE_180
    /* 좌우반전 + 180° = 상하(세로) 반전 — 페이지 역순 + 바이트 비트 역순(열 유지).
     *  (180°가 이미 좌우+상하이므로 좌우를 한 번 더 하면 상하만 남는다.) */
    uint8_t flipped[OLED_PAGES][OLED_PANEL_W];
    for (int p = 0; p < OLED_PAGES; p++) {
        const int dst_p = OLED_PAGES - 1 - p;
        for (int c = 0; c < OLED_PANEL_W; c++) {
            flipped[dst_p][c] = _bit_reverse_8(s_fb[p][c]);
        }
    }
    for (int p = 0; p < OLED_PAGES; p++) {
        if (!_page_dirty(p, flipped[p], OLED_PANEL_W)) { s_skip_cnt++; continue; }
        _oled_write_page_locked(&s_dev, p, 0, flipped[p], OLED_PANEL_W);
        _shadow_store(p, flipped[p], OLED_PANEL_W); s_sent_cnt++;
    }
#elif OLED_ROTATE_180
    /* 회전 버퍼는 스택에 임시 할당 (물리 패널 크기). 매 frame 50ms 주기. */
    uint8_t rotated[OLED_PAGES][OLED_PANEL_W];
    for (int p = 0; p < OLED_PAGES; p++) {
        const int dst_p = OLED_PAGES - 1 - p;
        for (int c = 0; c < OLED_PANEL_W; c++) {
            const int dst_c = OLED_PANEL_W - 1 - c;
            rotated[dst_p][dst_c] = _bit_reverse_8(s_fb[p][c]);
        }
    }
    for (int p = 0; p < OLED_PAGES; p++) {
        if (!_page_dirty(p, rotated[p], OLED_PANEL_W)) { s_skip_cnt++; continue; }
        _oled_write_page_locked(&s_dev, p, 0, rotated[p], OLED_PANEL_W);
        _shadow_store(p, rotated[p], OLED_PANEL_W); s_sent_cnt++;
    }
#else
    for (int p = 0; p < OLED_PAGES; p++) {
        if (!_page_dirty(p, s_fb[p], OLED_PANEL_W)) { s_skip_cnt++; continue; }
        _oled_write_page_locked(&s_dev, p, 0, s_fb[p], OLED_PANEL_W);
        _shadow_store(p, s_fb[p], OLED_PANEL_W); s_sent_cnt++;
    }
#endif
    s_shadow_valid = true;   /* 이번 프레임 반영 완료 */
    oled_ui_i2c_unlock();
}

/* ═══════════════════════════════════════════════
   일반 동작 화면 (v1.2 재배치)
   레이아웃 (72×40):
   ┌────────────────────────────────────────────────┐
   │ 447.62              📶 100%        y= 0.. 7    │  status row
   │ ────────────────────────────────── y= 8        │  separator
   │                                                │
   │             21:34                  y=10..23    │  big clock (2x)
   │          2026-05-12                y=25..31    │  small date
   │ ────────────────────────────────── y=32        │  separator
   │   1  2  3  4  5  ALL               y=33..39    │  blind selection
   └────────────────────────────────────────────────┘
═══════════════════════════════════════════════ */

/* Thread 메시 아이콘 (8×7 px) — 부착 시 노드 + 메시 라인, 미부착 시 X */
static void _draw_thread_signal(int x, int y, bool attached)
{
    if (!attached) {
        /* 미부착: X 표시 */
        _fb_set_pixel(x,   y,   true); _fb_set_pixel(x+6, y,   true);
        _fb_set_pixel(x+1, y+1, true); _fb_set_pixel(x+5, y+1, true);
        _fb_set_pixel(x+2, y+2, true); _fb_set_pixel(x+4, y+2, true);
        _fb_set_pixel(x+3, y+3, true);
        _fb_set_pixel(x+2, y+4, true); _fb_set_pixel(x+4, y+4, true);
        _fb_set_pixel(x+1, y+5, true); _fb_set_pixel(x+5, y+5, true);
        _fb_set_pixel(x,   y+6, true); _fb_set_pixel(x+6, y+6, true);
        return;
    }
    /* 부착됨: 중앙 노드(self) + 3개 메시 이웃 노드 + 연결선 */
    /* 중앙 노드 (3x3 채움) */
    _fb_fill_rect(x+2, y+2, 3, 3);
    /* 이웃 노드 점 (좌상/우상/하) */
    _fb_set_pixel(x,   y,   true); _fb_set_pixel(x+1, y,   true);
    _fb_set_pixel(x,   y+1, true);
    _fb_set_pixel(x+5, y,   true); _fb_set_pixel(x+6, y,   true);
    _fb_set_pixel(x+6, y+1, true);
    _fb_set_pixel(x+2, y+6, true); _fb_set_pixel(x+3, y+6, true);
    _fb_set_pixel(x+4, y+6, true);
    /* 메시 연결선 — 중앙 → 3 이웃 */
    _fb_set_pixel(x+1, y+1, true);  _fb_set_pixel(x+1, y+2, true);  // 좌상
    _fb_set_pixel(x+5, y+1, true);  _fb_set_pixel(x+5, y+2, true);  // 우상
    _fb_set_pixel(x+3, y+5, true);                                  // 하
}
/* v2.x 호환: 기존 코드의 _draw_wifi_signal 호출은 thread 변환으로 */
#define _draw_wifi_signal _draw_thread_signal

/* RSSI(dBm) → 신호 막대 레벨 0..4 매핑.
 *  INVALID(127) 인데 CONNECTED 면 링크는 있으나 측정불가 → 풀바(4). */
static int _rssi_to_level(int8_t rssi)
{
    if (rssi == OLED_RSSI_INVALID) return 4;   /* connected, RSSI 미상 */
    if (rssi >= -55) return 4;
    if (rssi >= -65) return 3;
    if (rssi >= -78) return 2;
    if (rssi >= -90) return 1;
    return 0;
}

/* 신호막대만 (8×7) — level 0..4 (RSSI 가변). 안테나 기둥/도형 없음.
 *  채워진 막대 = 신호 세기, 미채움 막대는 바닥 1px 만 (윤곽). */
static void _draw_signal_bars(int x, int y, int level)
{
    /* 4개 막대, 너비 1px, x+0/+2/+4/+6, 윗 픽셀 y+5/+3/+1/+0 (바닥 y+6) */
    const int bx[4]   = {0, 2, 4, 6};
    const int btop[4] = {5, 3, 1, 0};
    for (int i = 0; i < 4; i++) {
        int cx = x + bx[i];
        if (i < level) {
            for (int yy = btop[i]; yy <= 6; yy++) _fb_set_pixel(cx, y + yy, true);
        } else {
            _fb_set_pixel(cx, y + 6, true);     /* 빈 막대: 바닥점만 */
        }
    }
}

/* 메인 화면 상단 Matter 상태 표시 (x..x+7, y..y+6).
 *  UNPAIRED : X (미페어링/대기)
 *  PAIRING  : 'P' 점멸 (페어링 트랜잭션 진행 중)
 *  CONNECTED: 신호막대만 (부모 RSSI 기반 4단계 가변, 안테나 도형 없음) */
static void _draw_matter_status(int x, int y, const oled_ui_ctx_t *ctx)
{
    switch (ctx->matter_state) {
    case OLED_MT_CONNECTED:
        _draw_signal_bars(x, y, _rssi_to_level(ctx->parent_rssi));
        break;
    case OLED_MT_PAIRING:
        /* 'P' 점멸 (~0.4s 주기). 페어링 중임을 명확히. */
        if ((ctx->anim_frame / 8) % 2 == 0) {
            _fb_draw_char(x + 1, y, 'P');
        }
        break;
    case OLED_MT_UNPAIRED:
    default:
        /* X 표시 (미페어링) */
        _fb_set_pixel(x,   y,   true); _fb_set_pixel(x+6, y,   true);
        _fb_set_pixel(x+1, y+1, true); _fb_set_pixel(x+5, y+1, true);
        _fb_set_pixel(x+2, y+2, true); _fb_set_pixel(x+4, y+2, true);
        _fb_set_pixel(x+3, y+3, true);
        _fb_set_pixel(x+2, y+4, true); _fb_set_pixel(x+4, y+4, true);
        _fb_set_pixel(x+1, y+5, true); _fb_set_pixel(x+5, y+5, true);
        _fb_set_pixel(x,   y+6, true); _fb_set_pixel(x+6, y+6, true);
        break;
    }
}

/* 작은 배터리 아이콘 (12×7) — fill_pct 0..100 */
static void _draw_battery_icon(int x, int y, uint8_t pct)
{
    /* 본체 12×7, 우측 단자 1×3 */
    _fb_rect(x, y, 12, 7);
    _fb_fill_rect(x + 12, y + 2, 1, 3);
    /* 내부 fill (10×5 영역에 pct/100 만큼 채움) */
    if (pct > 100) pct = 100;
    int fw = (10 * pct + 50) / 100;     // 반올림
    if (fw > 0) _fb_fill_rect(x + 1, y + 1, fw, 5);
}

static void _render_normal(oled_ui_ctx_t *ctx)
{
    _fb_clear();

#if OLED_RENDER_128X64
    {   /* ══ 128×64 네이티브 (또렷한 1:1, 형태 유지) — 해상도 기준 선택 ══ */
        /* ── 상단 상태줄 (y=2): 좌=연결 · 중앙=주파수 · 우=배터리% ── */
        if (ctx->matter_state == OLED_MT_CONNECTED) {        /* 신호바 3개 (맨 위) */
            _pfill(2, 6, 2, 2, true);
            _pfill(5, 3, 2, 5, true);
            _pfill(8, 0, 2, 8, true);
        } else if (ctx->matter_state == OLED_MT_PAIRING) {
            _pchar8(2, 0, 'P', true);
        } else {
            _pchar8(2, 0, 'x', true);
        }
        char fs[12]; _main_freq_str(ctx, fs, sizeof(fs));
        _pstr8((OLED_PANEL_W - _pstr8_w(fs)) / 2, 0, fs, true);
        char bs[8];
#if BOARD_BAT_SWAPPED
        /* 현 기판(GP12=ADC 불가): 잔량 % 불가 → 상태(USB/정상/저전압) */
        snprintf(bs, sizeof(bs), "%s", ctx->usb_pwr ? "USB" : (ctx->bat_low ? "LOW" : "BAT"));
#else
        /* 정상 기판(GP3=BAT ADC): 충전률 % (기존 로직) */
        if (ctx->chg_percent <= 100) snprintf(bs, sizeof(bs), "%d%%", ctx->chg_percent);
        else                         snprintf(bs, sizeof(bs), "--%%");
#endif
        _pstr8(OLED_PANEL_W - _pstr8_w(bs) - 2, 0, bs, true);

        /* ── 구분선 (방금 정한 1px 위치) ── */
        _fb_hline_phys(13);
        _fb_hline_phys(49);   /* 하단 bar 1px 아래로 */

#if !BOARD_DISABLE_TIME
        /* ── 중앙: 좌=월/일(8×8) · 우=시계(7-seg 큰 숫자) ── */
        time_t t0 = time(NULL); struct tm lt; localtime_r(&t0, &lt);
        static const char *kWd3[7] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
        const char *wd = (lt.tm_wday >= 0 && lt.tm_wday < 7) ? kWd3[lt.tm_wday] : "---";
        char md[8];
        snprintf(md, sizeof(md), "%02d/%02d", (lt.tm_mon + 1) % 100, lt.tm_mday % 100); /* MM/DD */
        const int dcx = 24;   /* 날짜 블록 중심(가운데 정렬) */
        _pstr8(dcx - _pstr8_w(wd) / 2, 21, wd, true);   /* 요일 3자 */
        _pstr8(dcx - _pstr8_w(md) / 2, 33, md, true);   /* MM/DD */

        const int cx = 54, cy = 20;                 /* 시계 (셀 13w×23h) — 우측 +10px */
        _draw_7seg(cx,      cy, lt.tm_hour / 10);
        _draw_7seg(cx + 15, cy, lt.tm_hour % 10);
        if ((lt.tm_sec & 1) == 0) _draw_colon(cx + 31, cy);   /* 콜론 매 초 깜빡 = 작동 표시 */
        _draw_7seg(cx + 38, cy, lt.tm_min / 10);
        _draw_7seg(cx + 53, cy, lt.tm_min % 10);
#else
        /* ── 시간/날짜 비활성(BOARD_DISABLE_TIME): 중앙에 선택된 블라인드를 크게 ── */
        {
            int sb = ctx->selected_blind;
            if (sb >= 0 && sb < BLIND_MAX_COUNT) {
                int bcx = (OLED_PANEL_W - 13) / 2;          /* 7-seg 1자리 가운데 */
                _draw_7seg(bcx, 20, (sb + 1) % 10);         /* 블라인드 1-based 번호 */
            } else {
                _pstr8_center(28, "ALL");                   /* 전체 선택 */
            }
        }
#endif

        /* ── 하단: 블라인드 선택 1..N + ALL (8개+ALL 은 윈도우 스크롤 + ‹›) ── */
        int sel = ctx->selected_blind;
        int total = BLIND_MAX_COUNT + 1;            /* 채널 N + ALL */
        int win = 7; if (win > total) win = total;  /* 128px 에 보이는 항목 수 */
        int ws = sel - win / 2; if (ws < 0) ws = 0;
        if (ws > total - win) ws = total - win;
        int disp_w = 0;                             /* 표시 폭(숫자 14 / ALL 26)으로 가운데 정렬 */
        for (int j = 0; j < win && ws + j < total; j++) disp_w += (ws + j < BLIND_MAX_COUNT) ? 14 : 26;
        int bx = (OLED_PANEL_W - disp_w) / 2;
        if (ws > 0) _pchar8(1, 55, '<', true);
        for (int j = 0; j < win && ws + j < total; j++) {
            int i = ws + j; bool is_sel = (sel == i);
            if (i < BLIND_MAX_COUNT) {
                char ch = (char)('1' + i);
                if (is_sel) { _pfill(bx - 1, 54, 10, 10, true); _pchar8(bx, 55, ch, false); }
                else        { _pchar8(bx, 55, ch, true); }
                bx += 14;
            } else {
                if (is_sel) { _pfill(bx - 1, 54, 26, 10, true);
                              _pchar8(bx, 55, 'A', false); _pchar8(bx + 8, 55, 'L', false); _pchar8(bx + 16, 55, 'L', false); }
                else        { _pstr8(bx, 55, "ALL", true); }
                bx += 26;
            }
        }
        if (ws + win < total) _pchar8(OLED_PANEL_W - 6, 55, '>', true);

        _fb_flush();
        return;
    }
#elif OLED_RENDER_64X128
    {   /* ══ 64×128 세로(포트레이트) 네이티브 — 위→아래 적층 레이아웃 ══ */
        time_t t0 = time(NULL); struct tm lt; localtime_r(&t0, &lt);

        /* ── 상단 상태줄: 좌=연결 신호바 · 우=배터리% ── */
        if (ctx->matter_state == OLED_MT_CONNECTED) {
            _pfill(2, 6, 2, 2, true);
            _pfill(5, 3, 2, 5, true);
            _pfill(8, 0, 2, 8, true);
        } else if (ctx->matter_state == OLED_MT_PAIRING) {
            _pchar8(2, 0, 'P', true);
        } else {
            _pchar8(2, 0, 'x', true);
        }
        char bs[8];
#if BOARD_BAT_SWAPPED
        /* 현 기판(GP12=ADC 불가): 잔량 % 불가 → 상태(USB/정상/저전압) */
        snprintf(bs, sizeof(bs), "%s", ctx->usb_pwr ? "USB" : (ctx->bat_low ? "LOW" : "BAT"));
#else
        /* 정상 기판(GP3=BAT ADC): 충전률 % (기존 로직) */
        if (ctx->chg_percent <= 100) snprintf(bs, sizeof(bs), "%d%%", ctx->chg_percent);
        else                         snprintf(bs, sizeof(bs), "--%%");
#endif
        _pstr8(OLED_PANEL_W - _pstr8_w(bs) - 2, 0, bs, true);

        /* ── 주파수 (중앙) ── */
        char fs[12]; _main_freq_str(ctx, fs, sizeof(fs));
        _pstr8_center(11, fs);

        _fb_hline_phys(21);

        /* ── 시계: HH(위) / MM(아래) 7-seg 2자리 가운데 적층 ── */
        const int hx = (OLED_PANEL_W - 28) / 2;   /* 2자리(13+15)=28 중앙 */
        _draw_7seg(hx,      24, lt.tm_hour / 10);
        _draw_7seg(hx + 15, 24, lt.tm_hour % 10);
        _draw_7seg(hx,      51, lt.tm_min / 10);
        _draw_7seg(hx + 15, 51, lt.tm_min % 10);
        if ((lt.tm_sec & 1) == 0) {                /* 두 줄 사이 깜빡 콜론 */
            _pfill(OLED_PANEL_W / 2 - 4, 47, 3, 3, true);
            _pfill(OLED_PANEL_W / 2 + 1, 47, 3, 3, true);
        }

        _fb_hline_phys(77);

        /* ── 날짜: 요일(위) / MM/DD(아래) ── */
        static const char *kWd3[7] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
        const char *wd = (lt.tm_wday >= 0 && lt.tm_wday < 7) ? kWd3[lt.tm_wday] : "---";
        char md[8];
        snprintf(md, sizeof(md), "%02d/%02d", (lt.tm_mon + 1) % 100, lt.tm_mday % 100);
        _pstr8_center(80, wd);
        _pstr8_center(91, md);

        _fb_hline_phys(102);

        /* ── 하단: 블라인드 선택 — 채널 윈도우(위, 8개 스크롤 + ‹›) / "ALL"(아래) ── */
        int sel = ctx->selected_blind;
        int cwin = 5;                               /* 64px 에 보이는 채널 수 */
        int cref = (sel < BLIND_MAX_COUNT) ? sel : BLIND_MAX_COUNT - 1;
        int cws = cref - cwin / 2; if (cws < 0) cws = 0;
        if (BLIND_MAX_COUNT > cwin && cws > BLIND_MAX_COUNT - cwin) cws = BLIND_MAX_COUNT - cwin;
        if (BLIND_MAX_COUNT <= cwin) cws = 0;
        int bx = 4;
        if (cws > 0) _pchar8(0, 106, '<', true);
        for (int j = 0; j < cwin && cws + j < BLIND_MAX_COUNT; j++) {
            int i = cws + j; char ch = (char)('1' + i);
            if (sel == i) { _pfill(bx, 105, 10, 10, true); _pchar8(bx + 1, 106, ch, false); }
            else          { _pchar8(bx + 1, 106, ch, true); }
            bx += 12;
        }
        if (cws + cwin < BLIND_MAX_COUNT) _pchar8(OLED_PANEL_W - 5, 106, '>', true);
        if (sel >= BLIND_MAX_COUNT) {               /* ALL (아래줄, 선택 시 반전) */
            int ax = (OLED_PANEL_W - 26) / 2;
            _pfill(ax, 117, 26, 10, true);
            _pchar8(ax + 1, 118, 'A', false);
            _pchar8(ax + 9, 118, 'L', false);
            _pchar8(ax + 17, 118, 'L', false);
        } else {
            _pstr8_center(118, "ALL");
        }

        _fb_flush();
        return;
    }
#endif

    /* ═══ Row 0 (y=0..6): 좌=안테나(Matter), 중앙=주파수, 우=배터리 % ═══
     *  배치 변경(2026-05): 안테나(Matter 상태)를 좌측 끝(x=0)으로 옮기고
     *  주파수를 안테나·배터리 사이 중앙으로 이동.
     *    안테나(Matter): x=0..7  (8 px)
     *    주파수 "447.72": 안테나↔배터리 사이(x=8..47, 40 px) 중앙 → x=10
     *    배터리 "100%"  : x=48..71 (24 px) */

    /* 좌측 끝: Matter 상태(안테나/신호막대) — UNPAIRED=X / PAIRING='P'점멸 /
     *  CONNECTED=신호막대만(부모 RSSI 4단계). */
    _draw_matter_status(0, 0, ctx);

    /* 중앙: 주파수 값 "447.72" (6 chars × 6 = 36 px) — 양 끝 요소 사이 중앙.
     *  ★ 표시값은 register 설정값(예 447.72)이며, 보드 크리스털 오차로 실제
     *  안테나 출력은 약 -41.5 kHz(≈447.678) 다. 자세한 내용은 README 참고. */
    char freq_str[12];
    _main_freq_str(ctx, freq_str, sizeof(freq_str));
    _fb_draw_string(10, 0, freq_str);

    /* 우측: 배터리 상태/충전률 (BOARD_BAT_SWAPPED 분기) */
#if BOARD_BAT_SWAPPED
    /* 현 기판(GP12=ADC 불가): 상태(USB/정상/저전압) */
    const char *bat_str = ctx->usb_pwr ? "USB" : (ctx->bat_low ? "LOW" : "BAT");
    _fb_draw_string(48, 0, bat_str);
#else
    /* 정상 기판(GP3=BAT ADC): 충전률 % (기존 로직) */
    char bat_str[8];
    if (ctx->chg_percent <= 100) snprintf(bat_str, sizeof(bat_str), "%3d%%", ctx->chg_percent);
    else                         snprintf(bat_str, sizeof(bat_str), "---%%");
    _fb_draw_string(48, 0, bat_str);
#endif

    /* ═══ y=8 구분선 ═══ */
    _fb_hline(0, OLED_WIDTH - 1, 8);

    /* ═══ 중앙 영역 (y=10..26): 좌=월/일 두 줄, 우=시간 2× 큰 폰트 ═══
     *  (72×40 보드 — H2 외 보드는 시간/날짜 유지) */
    time_t tnow = time(NULL);
    struct tm tm;
    localtime_r(&tnow, &tm);

    const int date_x = 3;
    const int time_x = 18;

    /* 좌측: 월 / 일 (5×7 폰트), 두 줄 간격 3 px */
    char mm[4], dd[4];
    snprintf(mm, sizeof(mm), "%02d", (tm.tm_mon + 1) % 100);
    snprintf(dd, sizeof(dd), "%02d", tm.tm_mday % 100);
    _fb_draw_string(date_x, 10, mm);   // 월: y=10..16
    _fb_draw_string(date_x, 20, dd);   // 일: y=20..26 (gap 3 px : y=17,18,19)

    /* 우측: 시간 HH:MM — 2× scaled 5×7 (10×14), 글자 간 0 spacing */
    char digits[5];
    digits[0] = '0' + ((tm.tm_hour / 10) % 10);
    digits[1] = '0' + (tm.tm_hour % 10);
    digits[2] = '0' + ((tm.tm_min  / 10) % 10);
    digits[3] = '0' + (tm.tm_min  % 10);
    digits[4] = 0;

    _fb_draw_char_2x(time_x + 0 * 10, 12, digits[0]);   // H tens
    _fb_draw_char_2x(time_x + 1 * 10, 12, digits[1]);   // H units
    if ((tm.tm_sec & 1) == 0) {
        _fb_draw_char_2x(time_x + 2 * 10, 12, ':');     // 콜론 (짝수 초)
    }
    _fb_draw_char_2x(time_x + 3 * 10, 12, digits[2]);   // M tens
    _fb_draw_char_2x(time_x + 4 * 10, 12, digits[3]);   // M units

    /* ═══ y=31 구분선 (하단 블라인드 슬롯과 2 px gap 확보) ═══ */
    _fb_hline(0, OLED_WIDTH - 1, 31);

    /* ═══ Row 3 (y=33..39): 블라인드 선택 — 채널 N + ALL (윈도우 스크롤 + ‹›) ═══
     *  8개+ALL 은 72px 에 다 안 들어가므로 선택 항목 중심 6개 윈도우를 슬라이드.
     *  양끝에 더 있으면 '<' '>' 인디케이터. (항목 i: 0..N-1=채널, N=ALL) */
    const int row_y = 33;
    int total = BLIND_MAX_COUNT + 1;
    int win = 6;                              /* 72px 에 보이는 항목 수 */
    int sel_i = ctx->selected_blind;
    int ws = sel_i - win / 2; if (ws < 0) ws = 0;
    if (ws > total - win) ws = total - win;
    if (total <= win) ws = 0;
    if (ws > 0) _fb_draw_char(0, row_y, '<');
    int x = (ws > 0) ? 8 : 2;
    for (int j = 0; j < win && ws + j < total; j++) {
        int i = ws + j;
        bool sel = (ctx->selected_blind == i);
        if (i < BLIND_MAX_COUNT) {
            /* 개별 슬롯: '1'.. 단일 문자 (1-based 표시 번호) */
            char ch = (char)('1' + i);
            if (sel) {
                _fb_fill_rect(x - 1, row_y - 1, 7, 9);
                const uint8_t *g = font5x7_basic[(uint8_t)ch - 0x20];
                for (int col = 0; col < 5; col++) {
                    uint8_t line = g[col];
                    for (int row = 0; row < 7; row++) {
                        if ((line >> row) & 1) {
                            int px = x + col, py = row_y + row;
                            if (px < OLED_WIDTH && py < OLED_HEIGHT) {
                                bool cur = _fb_get_pixel(px, py);
                                _fb_set_pixel(px, py, !cur);
                            }
                        }
                    }
                }
            } else {
                _fb_draw_char(x, row_y, ch);
            }
            x += 8;
        } else {
            /* ALL 슬롯: 3글자 */
            const char *all = "ALL";
            if (sel) {
                _fb_fill_rect(x - 1, row_y - 1, 3 * FONT_W + 1, 9);
                for (int si = 0; si < 3; si++) {
                    const uint8_t *g = font5x7_basic[(uint8_t)all[si] - 0x20];
                    for (int col = 0; col < 5; col++) {
                        uint8_t line = g[col];
                        for (int row = 0; row < 7; row++) {
                            if ((line >> row) & 1) {
                                int px = x + si * 6 + col, py = row_y + row;
                                if (px < OLED_WIDTH && py < OLED_HEIGHT) {
                                    bool cur = _fb_get_pixel(px, py);
                                    _fb_set_pixel(px, py, !cur);
                                }
                            }
                        }
                    }
                }
            } else {
                _fb_draw_string(x, row_y, all);
            }
            x += 3 * FONT_W + 2;
        }
    }
    if (ws + win < total) _fb_draw_char(OLED_WIDTH - 5, row_y, '>');

    _fb_flush();
}

/* ═══════════════════════════════════════════════
   버튼 동작 애니메이션 화면 — 엘리베이터 floor-indicator 스타일
   72×40 전체화면 큰 아이콘 (≈30×30) + 슬라이드 애니메이션.
   2~3초간 표시 후 자동으로 NORMAL 화면 복귀.
   RF 송신은 _btn_event_cb에서 즉시 fire-and-forget이므로 본 함수는 시각화 전용.
═══════════════════════════════════════════════ */

/* 큰 화살표(30×30) — 위방향 (DOWN은 vertical-flip 으로 재사용) */
/* 각 행은 30비트, MSB가 좌측 픽셀. 30개 행. */
static const uint32_t big_arrow_up[30] = {
    0x00060000U, 0x000F0000U, 0x001F8000U, 0x003FC000U,
    0x007FE000U, 0x00FFF000U, 0x01FFF800U, 0x03FFFC00U,
    0x07FFFE00U, 0x0FFFFF00U, 0x1FFFFF80U, 0x3FFFFFC0U,
    0x7FFFFFE0U, 0x00FFF000U, 0x00FFF000U, 0x00FFF000U,
    0x00FFF000U, 0x00FFF000U, 0x00FFF000U, 0x00FFF000U,
    0x00FFF000U, 0x00FFF000U, 0x00FFF000U, 0x00FFF000U,
    0x00FFF000U, 0x00FFF000U, 0x00FFF000U, 0x00FFF000U,
    0x00FFF000U, 0x00FFF000U,
};

static void _draw_big_arrow(int cx, int cy, bool flip_v, int slide_off, int target_w)
{
    /* 30(높이) × target_w(폭) 화살표. 소스(30px폭) 를 target_w 로 선형
     *  매핑(정수 비례) → 1.0x 이외의 폭도 깔끔히 그려진다.
     *  중심 (cx, cy) + slide_off(세로 이동). */
    if (target_w < 8) target_w = 8;
    int ox = cx - target_w / 2;
    int oy = cy - 15 + slide_off;
    for (int oc = 0; oc < target_w; oc++) {
        int sc = (oc * 30) / target_w;            /* 0..29 source col */
        for (int r = 0; r < 30; r++) {
            int dr = flip_v ? (29 - r) : r;
            uint32_t line = big_arrow_up[dr];
            if ((line >> (31 - sc)) & 1)
                _fb_set_pixel(ox + oc, oy + r, true);
        }
    }
}

/* 동시작동용 가변높이 화살표(72×40 프레임버퍼). big_arrow_up 30px 소스를 tw×th 로 스케일
 *  → 한 화면에 ↑/↓ 두 개를 위·아래로 동시에 그릴 수 있다(_draw_big_arrow 는 높이 30 고정). */
static void _draw_arrow_wh(int cx, int cy, bool flip_v, int tw, int th)
{
    int ox = cx - tw / 2, oy = cy - th / 2;
    for (int oc = 0; oc < tw; oc++) {
        int sc = (oc * 30) / tw;
        for (int orow = 0; orow < th; orow++) {
            int sr = (orow * 30) / th;
            int dr = flip_v ? (29 - sr) : sr;
            if ((big_arrow_up[dr] >> (31 - sc)) & 1) _fb_set_pixel(ox + oc, oy + orow, true);
        }
    }
}
/* 작은 STOP 박스(가운데 음각) — combo 의 MY 표시용. (x,y)=좌상단, w×h. */
static void _draw_stop_small(int x, int y, int w, int h)
{
    _fb_fill_rect(x, y, w, h);
    int hx = x + w / 2, hy = y + h / 2;
    for (int yy = hy - 2; yy <= hy + 1; yy++)
        for (int xx = hx - 4; xx <= hx + 3; xx++) {
            bool c = _fb_get_pixel(xx, yy);
            _fb_set_pixel(xx, yy, !c);
        }
}

static void _draw_big_stop_box(int cx, int cy, int pulse)
{
    /* 가로로 크게(폭 ±pulse), 세로는 모션 밴드(y<=30) 한계로 제한.
     *  hw=가로 반폭(36 → 사용자 요청에 따라 축소), hh=세로 반높이. */
    int hw = 18 + pulse;
    int hh = 14 + pulse;
    if (hh > 15) hh = 15;
    _fb_fill_rect(cx - hw, cy - hh, hw * 2, hh * 2);
    /* 음각 가운데 사각형 — visual 강조 */
    int hole = 5;
    for (int y = cy - hole; y < cy + hole; y++) {
        for (int x = cx - hole; x < cx + hole; x++) {
            int px = x, py = y;
            bool cur = _fb_get_pixel(px, py);
            _fb_set_pixel(px, py, !cur);
        }
    }
}

/* 큰 PROG(◎) — 동심원 펄싱 */
static void _draw_big_prog(int cx, int cy, uint8_t frame)
{
    int r_out = 14 + (frame / 4) % 4;
    int r_mid = 8 + (frame / 5) % 3;
    _fb_circle(cx, cy, r_out, false);
    _fb_circle(cx, cy, r_mid, false);
    _fb_circle(cx, cy, 3, true);
}

/* ─── 일반 픽셀 라인 (Bresenham) ─── */
static void _fb_draw_line(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    while (1) {
        _fb_set_pixel(x0, y0, true);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* ─── 로터리 회전 모션 (↻ / ↺ 스타일) ─────────────────────────
 * 거의 한 바퀴(≈285°) 두꺼운 호 + 호 끝의 큰 화살표 head.
 * 호의 gap(빈 구간 ~75°)이 frame 마다 회전 → 전체 ↻/↺ 가 도는 느낌.
 * ───────────────────────────────────────────────────────── */
static void _draw_big_rotate(int cx, int cy, uint8_t frame, bool cw)
{
    /* gap이 한 바퀴 도는 데 ≈3초 (frame 60 = 3s @20fps) */
    float gap_start = (float)frame * 0.105f;   // ~6 rad / 60 frame
    if (!cw) gap_start = -gap_start;

    const float GAP_SIZE = 1.30f;          // 약 75° 빈 구간
    const int   R         = 14;
    const int   THICK     = 3;

    /* 호를 0..2π 중 gap 영역만 빼고 두껍게 그림 */
    for (int i = 0; i < 240; i++) {
        float a = (float)i * (6.28318f / 240.0f);
        /* a 가 gap 안인지 검사 (wrap-around) */
        float d = a - gap_start;
        while (d < 0)         d += 6.28318f;
        while (d > 6.28318f)  d -= 6.28318f;
        if (d < GAP_SIZE) continue;

        for (int p = 0; p < THICK; p++) {
            int x = cx + (int)((R - p) * cosf(a));
            int y = cy + (int)((R - p) * sinf(a));
            _fb_set_pixel(x, y, true);
        }
    }

    /* 화살표 head — 호의 leading edge (CW: gap_start 직전, CCW: gap_end 직후) */
    float head_a;
    if (cw) {
        head_a = gap_start - 0.05f;
    } else {
        head_a = gap_start + GAP_SIZE + 0.05f;
    }

    /* 삼각형 화살표: 외곽 tip, 내부 tip, 전방 tip(tangent) */
    float ca = cosf(head_a), sa = sinf(head_a);
    int outer_x = cx + (int)((R + 4) * ca);
    int outer_y = cy + (int)((R + 4) * sa);
    int inner_x = cx + (int)((R - 6) * ca);
    int inner_y = cy + (int)((R - 6) * sa);
    /* forward tip — tangent 방향(회전 진행 방향) */
    float fwd_a = head_a + (cw ? -0.40f : 0.40f);
    int fwd_x = cx + (int)((R + 1) * cosf(fwd_a));
    int fwd_y = cy + (int)((R + 1) * sinf(fwd_a));

    /* 삼각형 그리기 — 3 line + 중심 채움 */
    _fb_draw_line(outer_x, outer_y, fwd_x,   fwd_y);
    _fb_draw_line(inner_x, inner_y, fwd_x,   fwd_y);
    _fb_draw_line(outer_x, outer_y, inner_x, inner_y);
    /* fill: 중점 → fwd 라인 추가 + 외곽/내부 중점에서 fwd로 */
    int mid_x = (outer_x + inner_x) / 2;
    int mid_y = (outer_y + inner_y) / 2;
    _fb_draw_line(mid_x, mid_y, fwd_x, fwd_y);
    int q1_x = (outer_x + mid_x) / 2, q1_y = (outer_y + mid_y) / 2;
    int q2_x = (inner_x + mid_x) / 2, q2_y = (inner_y + mid_y) / 2;
    _fb_draw_line(q1_x, q1_y, fwd_x, fwd_y);
    _fb_draw_line(q2_x, q2_y, fwd_x, fwd_y);

    /* 가운데 작은 점 (회전 축) — 가독성 보조 */
    _fb_set_pixel(cx, cy, true);
}

/* 모션 대상 블라인드 라벨 계산 ("B3" / "ALL" / "1 3 4") — 네이티브/논리 공용 */
static void _action_blind_label(const oled_ui_ctx_t *ctx, char *lbl, size_t n)
{
    uint8_t mask = ctx->action_blind_mask;
    const uint8_t ALLM = (uint8_t)((1u << BLIND_MAX_COUNT) - 1);   /* 전체 채널 비트 */
    if (mask == 0)
        mask = (ctx->selected_blind >= BLIND_MAX_COUNT) ? ALLM : (1u << ctx->selected_blind);
    int cnt = 0;
    for (int i = 0; i < BLIND_MAX_COUNT; i++) if (mask & (1u << i)) cnt++;
    if ((mask & ALLM) == ALLM) {
        snprintf(lbl, n, "ALL");
    } else if (cnt == 1) {
        int b = 0; while (!(mask & (1u << b))) b++;
        snprintf(lbl, n, "B%d", b + 1);
    } else {
        int p = 0;
        for (int i = 0; i < BLIND_MAX_COUNT && p < (int)n - 2; i++)
            if (mask & (1u << i)) { if (p) lbl[p++] = ' '; lbl[p++] = (char)('1' + i); }
        lbl[p] = '\0';
    }
}

#if OLED_RENDER_128X64
/* ══ 128×64 네이티브 모션 프리미티브 (블록스케일 없이 또렷) ══ */
static void _pline(int x0, int y0, int x1, int y1)   /* Bresenham, 물리 1:1 */
{
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx - dy;
    while (1) {
        _px(x0, y0, true);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}
static void _pcircle(int cx, int cy, int r)          /* 중점원 외곽선 */
{
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        _px(cx + x, cy + y, true); _px(cx + y, cy + x, true);
        _px(cx - y, cy + x, true); _px(cx - x, cy + y, true);
        _px(cx - x, cy - y, true); _px(cx - y, cy - x, true);
        _px(cx + y, cy - x, true); _px(cx + x, cy - y, true);
        y++;
        if (err < 0) err += 2 * y + 1;
        else { x--; err += 2 * (y - x) + 1; }
    }
}
/* 네이티브 큰 화살표: big_arrow_up(30×30 소스)를 tw×th 로 선형 매핑(또렷) */
static void _pbig_arrow(int cx, int cy, bool flip_v, int slide_off, int tw, int th)
{
    int ox = cx - tw / 2, oy = cy - th / 2 + slide_off;
    for (int oc = 0; oc < tw; oc++) {
        int sc = (oc * 30) / tw;
        for (int orow = 0; orow < th; orow++) {
            int sr = (orow * 30) / th;
            int dr = flip_v ? (29 - sr) : sr;
            if ((big_arrow_up[dr] >> (31 - sc)) & 1) _px(ox + oc, oy + orow, true);
        }
    }
}
/* 네이티브 로터리 회전(↻/↺) — _draw_big_rotate 의 물리해상도 버전(R 확대) */
static void _pbig_rotate(int cx, int cy, uint8_t frame, bool cw)
{
    float gap_start = (float)frame * 0.105f;
    if (!cw) gap_start = -gap_start;
    const float GAP = 1.30f;
    const int R = 20, THICK = 4;
    for (int i = 0; i < 320; i++) {
        float a = (float)i * (6.28318f / 320.0f);
        float d = a - gap_start;
        while (d < 0)        d += 6.28318f;
        while (d > 6.28318f) d -= 6.28318f;
        if (d < GAP) continue;
        for (int p = 0; p < THICK; p++)
            _px(cx + (int)((R - p) * cosf(a)), cy + (int)((R - p) * sinf(a)), true);
    }
    float head_a = cw ? (gap_start - 0.05f) : (gap_start + GAP + 0.05f);
    float ca = cosf(head_a), sa = sinf(head_a);
    int ox = cx + (int)((R + 5) * ca), oy = cy + (int)((R + 5) * sa);
    int ix = cx + (int)((R - 8) * ca), iy = cy + (int)((R - 8) * sa);
    float fa = head_a + (cw ? -0.40f : 0.40f);
    int fx = cx + (int)((R + 1) * cosf(fa)), fy = cy + (int)((R + 1) * sinf(fa));
    _pline(ox, oy, fx, fy); _pline(ix, iy, fx, fy); _pline(ox, oy, ix, iy);
    int mx = (ox + ix) / 2, my = (oy + iy) / 2;
    _pline(mx, my, fx, fy);
    _pline((ox + mx) / 2, (oy + my) / 2, fx, fy);
    _pline((ix + mx) / 2, (iy + my) / 2, fx, fy);
    _px(cx, cy, true);
}
#endif /* OLED_RENDER_128X64 */

static void _render_action(oled_ui_ctx_t *ctx)
{
    _fb_clear();

#if OLED_RENDER_128X64
    {   /* ══ 128×64 네이티브 모션 (또렷한 1:1, 블록스케일 없음) ══ */
        uint8_t f = ctx->anim_frame;
        const int cx = OLED_PANEL_W / 2;   /* 64 */
        const int cy = 23;                  /* 모션 중심(하단 라벨 위) */
        switch (ctx->action) {
            case OLED_ACTION_UP:
                _pbig_arrow(cx, cy, false, 7 - (int)(f % 15), 56, 34);
                break;
            case OLED_ACTION_DOWN:
                _pbig_arrow(cx, cy, true, -7 + (int)(f % 15), 56, 34);
                break;
            case OLED_ACTION_TILT_UP:
            case OLED_ACTION_TILT_DN: {
                bool up = (ctx->action == OLED_ACTION_TILT_UP);
                const int px0 = 32, ax = 44, len = 54;
                const int sb[3] = {12, 24, 36};
                const int phase[8] = {0, 3, 6, 9, 12, 9, 6, 3};
                int t = phase[(f / 3) % 8], dir = up ? -1 : +1;
                for (int y = 4; y <= 44; y++) {
                    _px(px0, y, true); _px(px0 + 1, y, true); _px(px0 + 2, y, true);
                }
                for (int s = 0; s < 3; s++)
                    for (int x = 0; x <= len; x++) {
                        int rel = x - len / 2;
                        int py = sb[s] + (dir * t * rel * 2) / len;
                        if (py < 4)  py = 4;
                        if (py > 44) py = 44;
                        _px(ax + x, py, true);
                        if (py + 1 <= 44) _px(ax + x, py + 1, true);
                    }
                break;
            }
            case OLED_ACTION_STOP: {
                int pulse = (int)(3.0f * sinf(f * 0.55f));
                int hw = 30 + pulse, hh = 20 + pulse;
                _pfill(cx - hw, cy - hh, hw * 2, hh * 2, true);
                _pfill(cx - 8, cy - 8, 16, 16, false);   /* 음각 가운데 */
                break;
            }
            case OLED_ACTION_PROG:
                _pcircle(cx, cy, 20 + (f / 4) % 5);
                _pcircle(cx, cy, 12 + (f / 5) % 3);
                _pfill(cx - 3, cy - 3, 6, 6, true);
                break;
            case OLED_ACTION_ROT_CW:  _pbig_rotate(cx, cy, f, true);  break;
            case OLED_ACTION_ROT_CCW: _pbig_rotate(cx, cy, f, false); break;
            case OLED_ACTION_UP_DOWN: {           /* ↑왼쪽 + ↓오른쪽 (가로 배치) */
                int up = 7 - (int)(f % 15), dn = -7 + (int)(f % 15);
                _pbig_arrow(34, cy, false, up, 52, 38);
                _pbig_arrow(94, cy, true,  dn, 52, 38);
                break;
            }
            case OLED_ACTION_MY_UP: {             /* ■STOP왼쪽 + ↑오른쪽 */
                _pfill(12, cy - 18, 44, 36, true);
                _pfill(27, cy - 5,  14, 10, false);
                _pbig_arrow(94, cy, false, 7 - (int)(f % 15), 52, 38);
                break;
            }
            case OLED_ACTION_MY_DOWN: {           /* ■STOP왼쪽 + ↓오른쪽 */
                _pfill(12, cy - 18, 44, 36, true);
                _pfill(27, cy - 5,  14, 10, false);
                _pbig_arrow(94, cy, true, -7 + (int)(f % 15), 52, 38);
                break;
            }
            default: break;
        }
        /* 하단 라벨(고딕) — 모션 밴드와 분리(밴드 강제 클리어 후 그림) */
        char lbl[16];
        _action_blind_label(ctx, lbl, sizeof(lbl));
        _pfill(0, 49, OLED_PANEL_W, OLED_PANEL_H - 49, false);
        _pstr8_center(52, lbl);
        _fb_flush();
        return;
    }
#endif

    uint8_t f = ctx->anim_frame;
    const int cx = OLED_WIDTH / 2;   // 36
    /* ★ 모션 영역과 하단 블라인드 라벨이 겹치지 않도록 모션 중심을 위로.
     *  라벨은 y=32~38(7px). 모션은 y<=30 에 들어오도록 cy=16 + 슬라이드
     *  진폭 축소(±6). 추가로 하단 밴드(y>=31)는 라벨 직전 강제 클리어. */
    const int cy = 15;

    switch (ctx->action) {
        case OLED_ACTION_UP: {
            /* 시작 = 모션 반대쪽 끝(아래, +11) → 위로(-6) 흐름. 사이클
             *  18프레임(~0.9s, 기존 24프레임 대비 33% 빠름). 가로 폭 45
             *  (기존 60 대비 축소). */
            int slide = 11 - (int)(f % 18);
            _draw_big_arrow(cx, cy, false, slide, 45);
            break;
        }
        case OLED_ACTION_DOWN: {
            /* 시작 = 모션 반대쪽 끝(위, -11) → 아래로(+6) 흐름. */
            int slide = -11 + (int)(f % 18);
            _draw_big_arrow(cx, cy, true, slide, 45);
            break;
        }
        case OLED_ACTION_TILT_UP:
        case OLED_ACTION_TILT_DN: {
            /* 좌측 고정 '기둥' + 슬랫 3장(블라인드).
             *   - 기둥과 슬랫 사이 간격을 넉넉히(5px) 띄움
             *   - 슬랫은 '중앙 피벗' — 좌/우 끝이 대칭으로 ±t 만큼
             *     상/하로 움직임(실제 베네치안 블라인드 회전과 동일)
             *   - TILT_UP : 우측 끝 ↑, 좌측 끝 ↓
             *   - TILT_DN : 우측 끝 ↓, 좌측 끝 ↑
             *  영역은 y<=30 으로 제한(하단 라벨 비침범). */
            bool up = (ctx->action == OLED_ACTION_TILT_UP);
            /* 화면 중앙 정렬: 기둥(2px) + 간격(5px) + 슬랫(30px) = 37px,
             *  px0=18 → 기둥 x=18..19, 간격 20..24, 슬랫 x=25..55. 중심≈37. */
            const int px0 = 18;           /* 기둥 좌측 x (2px 폭) */
            const int ax  = 25;           /* 슬랫 시작 x (간격 5px) */
            const int len = 30;           /* 슬랫 길이(가로) */
            const int sb[3] = {5, 13, 21};/* 슬랫 기준 y (중심선) */
            const int phase[8] = {0, 2, 4, 6, 8, 6, 4, 2};
            int t = phase[(f / 3) % 8];
            int dir = up ? -1 : +1;       /* 우측 끝 이동 방향 */
            /* 고정 기둥 (실선 수직) */
            for (int y = 2; y <= 30; y++) {
                _fb_set_pixel(px0,     y, true);
                _fb_set_pixel(px0 + 1, y, true);
            }
            /* 슬랫: 중앙 피벗(좌/우 끝이 대칭 이동).
             *  rel = (x - len/2) ∈ [-len/2, +len/2]
             *  py  = sb + dir * t * rel * 2 / len
             *  → x=0     : sb - dir*t  (좌측 끝)
             *  → x=len/2 : sb         (중앙 = 피벗)
             *  → x=len   : sb + dir*t  (우측 끝) */
            for (int s = 0; s < 3; s++) {
                for (int x = 0; x <= len; x++) {
                    int rel = x - len / 2;
                    int py  = sb[s] + (dir * t * rel * 2) / len;
                    if (py < 2)  py = 2;
                    if (py > 30) py = 30;
                    _fb_set_pixel(ax + x, py, true);
                    if (py + 1 <= 30) _fb_set_pixel(ax + x, py + 1, true);
                }
            }
            break;
        }
        case OLED_ACTION_STOP: {
            /* 펄스 주파수 ↑(0.4 → 0.55), 가로 ↓(box hw 22→18) */
            int pulse = (int)(2.0f * sinf(f * 0.55f));
            _draw_big_stop_box(cx, cy, pulse);
            break;
        }
        case OLED_ACTION_PROG: {
            _draw_big_prog(cx, cy, f);
            break;
        }
        case OLED_ACTION_ROT_CW: {
            _draw_big_rotate(cx, cy, f, true);
            break;
        }
        case OLED_ACTION_ROT_CCW: {
            _draw_big_rotate(cx, cy, f, false);
            break;
        }
        case OLED_ACTION_UP_DOWN: {           /* ↑왼쪽 + ↓오른쪽 (가로 배치) */
            int d = (5 - (int)(f % 10)) / 2;
            _draw_arrow_wh(18, cy + d,  false, 30, 26);
            _draw_arrow_wh(54, cy - d,  true,  30, 26);
            break;
        }
        case OLED_ACTION_MY_UP: {             /* ■STOP왼쪽 + ↑오른쪽 */
            _draw_stop_small(5, cy - 12, 26, 24);
            _draw_arrow_wh(54, cy, false, 30, 26);
            break;
        }
        case OLED_ACTION_MY_DOWN: {           /* ■STOP왼쪽 + ↓오른쪽 */
            _draw_stop_small(5, cy - 12, 26, 24);
            _draw_arrow_wh(54, cy, true, 30, 26);
            break;
        }
        default: break;
    }

    /* ── 하단: 모션 대상 블라인드 표시 (사용자 요청) ──────────────
     *  action_blind_mask: bit0..N-1=채널, (1<<BLIND_MAX_COUNT)-1=ALL, 0=selected_blind 폴백.
     *   - 단일      : "B3"
     *   - 전체      : "ALL"
     *   - 다중      : "1 3 4" (명령 전달된 번호 나열) */
    {
        char lbl[16];
        _action_blind_label(ctx, lbl, sizeof(lbl));
        /* ★ 겹침 원천 차단: 라벨 영역(하단 밴드 y=31~39)을 먼저 강제
         *  클리어 → 모션 그래픽이 어떤 경우에도 라벨을 침범하지 못함. */
        for (int yy = 31; yy < OLED_HEIGHT; yy++)
            for (int xx = 0; xx < OLED_WIDTH; xx++)
                _fb_set_pixel(xx, yy, false);

        int len = (int)strlen(lbl);
        int x = (OLED_WIDTH - len * FONT_W) / 2;
        if (x < 0) x = 0;
        _fb_draw_string(x, 32, lbl);   /* y=32 (라벨, 모션과 분리) */
    }

    _fb_flush();
}

/* ═══════════════════════════════════════════════
   Thread 커미셔닝 화면 (구 WiFi 프로비저닝)
   BLE commissioner 가 Thread 자격증명을 자동 주입 — 별도 SSID/PW 없음.
   사용자에게 Matter pair-code 와 SmartThings 추가 절차만 안내.
═══════════════════════════════════════════════ */
static void _render_thread_prov(oled_ui_ctx_t *ctx)
{
    _fb_clear();

    uint8_t f = ctx->anim_frame;

    /* 타이틀 */
    _fb_draw_string(0, 0, "Thread");
    _fb_hline(0, OLED_WIDTH - 1, 9);

    /* 안내 메시지 (슬라이드 애니메이션) */
    static uint8_t s_slide = 0;
    if (f % 80 == 0) s_slide = (s_slide + 1) % 3;

    switch (s_slide) {
        case 0:
            _fb_draw_string(0, 12, "1.Add via");
            _fb_draw_string(0, 21, "SmartTh.");
            _fb_draw_string(0, 30, "Matter+");
            break;
        case 1:
            _fb_draw_string(0, 12, "2.PIN:");
            _fb_draw_string(0, 21, ctx->thread_prov_qr[0] ? ctx->thread_prov_qr : "20202021");
            _fb_draw_string(0, 30, "BLE pair");
            break;
        case 2:
            _fb_draw_string(0, 12, "3.Need");
            _fb_draw_string(0, 21, "Thread BR");
            _fb_draw_string(0, 30, "(hub/HP)");
            break;
    }

    /* 진행 점 (3개 돌아가며 표시) */
    for (int i = 0; i < 3; i++) {
        if (s_slide == i) {
            _fb_fill_rect(60 + i * 5, 37, 4, 4);
        } else {
            _fb_rect(60 + i * 5, 37, 4, 4);
        }
    }

    /* 깜박이는 WiFi 아이콘 */
    if ((f / 15) % 2 == 0) {
        _fb_draw_icon(0, 29, icon_wifi);
    }

    _fb_flush();
}

/* ═══════════════════════════════════════════════
   화면 보호기 - 2단계 구현
   1단계 (0~1분): 시계 + 점 플로팅
   2단계 (1분~): 시계 크게 + 점 빠르게 이동 (번인 방지 강화)
═══════════════════════════════════════════════ */
/* 5×7 폰트를 2배 스케일로 출력 (10×14 px) — 화면 보호기 큰 시계용 */
static void _fb_draw_char_2x(int x, int y, char c)
{
    if (c < 0x20 || c > 0x7E) return;
    const uint8_t *g = font5x7_basic[(uint8_t)c - 0x20];
    for (int col = 0; col < 5; col++) {
        uint8_t line = g[col];
        for (int row = 0; row < 7; row++) {
            if ((line >> row) & 1) {
                int px = x + col * 2;
                int py = y + row * 2;
                _fb_set_pixel(px,     py,     true);
                _fb_set_pixel(px + 1, py,     true);
                _fb_set_pixel(px,     py + 1, true);
                _fb_set_pixel(px + 1, py + 1, true);
            }
        }
    }
}

static void _fb_draw_string_2x(int x, int y, const char *s)
{
    while (*s) {
        _fb_draw_char_2x(x, y, *s++);
        x += 12;   // 10px char + 2px spacing
        if (x + 10 > OLED_WIDTH) break;
    }
}


/* ═══════════════════════════════════════════════
   주파수 편집 화면
═══════════════════════════════════════════════ */
static void _render_freq_edit(oled_ui_ctx_t *ctx)
{
    _fb_clear();

    _fb_draw_string(0, 0, "FREQ EDIT");
    _fb_hline(0, OLED_WIDTH - 1, 9);

    /* 주파수 큰 글씨 */
    char freq_str[12];
    snprintf(freq_str, sizeof(freq_str), "%.2f", ctx->freq_mhz);
    _fb_draw_string(4, 14, freq_str);
    _fb_draw_string(52, 14, "M");  // MHz
#if BOARD_HAS_LR_BUTTONS
    /* 디지트 커서 밑줄 — 0='447.[7]2' tenths(x=28), 1='447.7[2]' hundredths(x=34) */
    {
        int curx = (ctx->freq_edit_cursor == 0) ? 28 : 34;
        _fb_hline(curx, curx + FONT_W - 2, 22);
    }
#endif

    /* 편집 여부 + 조작 힌트 (SET=저장, STOP=취소→메뉴) */
    if (ctx->freq_edit_dirty)
        _fb_draw_string(0, 24, "*edited*");
    _fb_draw_string(0, 32, "SET:ok X:esc");

    _fb_flush();
}

/* ═══════════════════════════════════════════════
   설정 메뉴 화면 (v3.1+)
     72 × 40 — 5×7 narrow 폰트, 한 줄 6px 높이 × 5줄
     >  1. Freq Edit
        2. Matter Pair
        3. Thread Rst
        4. Cancel
═══════════════════════════════════════════════ */
/* v3.5: 숫자 prefix 제거 (가독성 ↑), Cancel 을 첫 항목으로 이동.
 *       72×40 OLED 폭 — 5×7 narrow font 6px → cursor 1ch + 1 space + 10ch 본문 = 12ch.
 *       문자 잘림 방지 위해 본문 최대 10 chars 유지. */
static const char * const SETUP_MENU_ITEMS[] = {
    "Cancel",        /* 메인 화면 복귀 */
    "Freq Edit",     /* 주파수 편집 */
#if !BOARD_DISABLE_TIME
    "Time Set",      /* 날짜/시간 수동 설정 (v3.9+) */
#endif
    "Matter Pair",   /* Matter 페어링 */
    "Thread Rst",    /* Thread 리셋 */
#if !BOARD_DISABLE_OTA
    "FW Update",     /* 펌웨어 업데이트 (Matter OTA over Thread) */
#endif
    "Reboot",        /* 시스템 재부팅 (HW 변경 재검출 — 예: CC1101 탈착) */
};
#define SETUP_MENU_COUNT (sizeof(SETUP_MENU_ITEMS) / sizeof(SETUP_MENU_ITEMS[0]))

static void _render_setup_menu(oled_ui_ctx_t *ctx)
{
    _fb_clear();

#if OLED_RENDER_128X64
    {   /* ══ 128×64 네이티브 설정 메뉴 (고딕, 4행 스크롤) ══ */
        _pstr8_center(2, "SETUP MENU");
        _fb_hline_phys(13);

        uint8_t cur = ctx->setup_cursor;
        if (cur >= SETUP_MENU_COUNT) cur = 0;
        const uint8_t VIS = 4;
        uint8_t top = (cur >= VIS) ? (uint8_t)(cur - VIS + 1) : 0;
        for (uint8_t row = 0; row < VIS && (top + row) < SETUP_MENU_COUNT; row++) {
            uint8_t i = (uint8_t)(top + row);
            int y = 18 + row * 11;
            if (i == cur) {                       /* 선택행: 반전 막대 */
                _pfill(0, y - 1, OLED_PANEL_W, 11, true);
                _pstr8(2, y, ">", false);
                _pstr8(12, y, SETUP_MENU_ITEMS[i], false);
            } else {
                _pstr8(12, y, SETUP_MENU_ITEMS[i], true);
            }
        }
        _fb_flush();
        return;
    }
#elif OLED_RENDER_64X128
    {   /* ══ 64×128 세로 설정 메뉴 (고딕, 다행 스크롤) ══ */
        _pstr8_center(2, "SETUP");          /* 64px 폭 → 짧은 제목 */
        _fb_hline_phys(13);

        uint8_t cur = ctx->setup_cursor;
        if (cur >= SETUP_MENU_COUNT) cur = 0;
        const uint8_t VIS = 9;              /* 128px 높이 → 최대 9행 */
        uint8_t top = (cur >= VIS) ? (uint8_t)(cur - VIS + 1) : 0;
        for (uint8_t row = 0; row < VIS && (top + row) < SETUP_MENU_COUNT; row++) {
            uint8_t i = (uint8_t)(top + row);
            int y = 18 + row * 12;
            if (i == cur) {                  /* 선택행: 반전 막대 */
                _pfill(0, y - 1, OLED_PANEL_W, 11, true);
                _pstr8(1, y, ">", false);
                _pstr8(9, y, SETUP_MENU_ITEMS[i], false);
            } else {
                _pstr8(9, y, SETUP_MENU_ITEMS[i], true);
            }
        }
        _fb_flush();
        return;
    }
#endif

    /* 헤더 */
    _fb_draw_string(0, 0, "SETUP MENU");
    _fb_hline(0, OLED_WIDTH - 1, 8);

    uint8_t cur = ctx->setup_cursor;
    if (cur >= SETUP_MENU_COUNT) cur = 0;

    /* 페이지당 3행 스크롤 뷰포트. 행 간격 10px 로 넓혀 가독성 확보
     *  (72×40 OLED: y = 11/21/31, 7px 폰트 → 38px 이내). 커서가 항상
     *  보이도록 top 계산. */
    const uint8_t VIS = 3;
    uint8_t top = 0;
    if (cur >= VIS) top = (uint8_t)(cur - VIS + 1);
    for (uint8_t row = 0; row < VIS && (top + row) < SETUP_MENU_COUNT; row++) {
        uint8_t i = (uint8_t)(top + row);
        int y = 11 + row * 10;
        _fb_draw_string(0, y, (i == cur) ? ">" : " ");
        _fb_draw_string(6, y, SETUP_MENU_ITEMS[i]);
    }

    _fb_flush();
}

/* ═══════════════════════════════════════════════
   날짜/시간 수동 설정 화면 (v3.9+)
     72×40 — "YYYY-MM-DD" / "HH:MM"
     UP/DOWN = 자리 이동, 틸트UP/DN = 값 ±, SET=저장, STOP=취소
     선택된 자리 아래에 밑줄 표시.
═══════════════════════════════════════════════ */
static void _render_time_edit(oled_ui_ctx_t *ctx)
{
    _fb_clear();
    _fb_draw_string(0, 0, "TIME SET");
    _fb_hline(0, OLED_WIDTH - 1, 9);

    const int *v = ctx->time_edit_val;
    char dbuf[12], tbuf[8];
    snprintf(dbuf, sizeof(dbuf), "%04d-%02d-%02d", v[0], v[1], v[2]);
    snprintf(tbuf, sizeof(tbuf), "%02d:%02d",      v[3], v[4]);

    const int DY = 13;            /* 날짜 줄 y */
    const int TY = 24;            /* 시간 줄 y */
    const int TX = 21;            /* 시간 줄 x (가운데 정렬: 5ch) */
    _fb_draw_string(0,  DY, dbuf);
    _fb_draw_string(TX, TY, tbuf);

    /* 활성 자리 밑줄. 자리: 0=년 1=월 2=일 3=시 4=분 */
    int ux0, ux1, uy;
    switch (ctx->time_edit_field) {
        case 0: ux0 = 0;        ux1 = 0  + 4*FONT_W - 1; uy = DY + 8; break; /* YYYY */
        case 1: ux0 = 5*FONT_W; ux1 = 5*FONT_W + 2*FONT_W - 1; uy = DY + 8; break; /* MM */
        case 2: ux0 = 8*FONT_W; ux1 = 8*FONT_W + 2*FONT_W - 1; uy = DY + 8; break; /* DD */
        case 3: ux0 = TX;       ux1 = TX + 2*FONT_W - 1; uy = TY + 8; break; /* HH */
        default:ux0 = TX+3*FONT_W; ux1 = TX+3*FONT_W + 2*FONT_W - 1; uy = TY + 8; break; /* MM */
    }
    _fb_hline(ux0, ux1, uy);

    _fb_draw_string(0, 33, "SET:ok X:esc");
    _fb_flush();
}

/* Thread 리셋 확인 화면 — SETUP 2s 길게 → 실행 */
static void _render_thread_reset(oled_ui_ctx_t *ctx)
{
    (void)ctx;
    _fb_clear();
    _fb_draw_string(0, 0, "THREAD RST");
    _fb_hline(0, OLED_WIDTH - 1, 8);
    _fb_draw_string(0, 12, "Hold SETUP");
    _fb_draw_string(0, 21, "2s=execute");
    _fb_draw_string(0, 30, "STOP:cancel");
    _fb_flush();
}

/* 펌웨어 업데이트(Matter OTA) 화면.
 *  fw_ota_state(matter_ota_state_t): 0=Idle 1=Querying 2=Downloading
 *  3=Applying 4=Delayed 5=Unknown. somfy_app 가 매 tick 갱신. */
static void _render_fw_update(oled_ui_ctx_t *ctx)
{
    _fb_clear();
    _fb_draw_string(0, 0, "FW UPDATE");
    _fb_hline(0, OLED_WIDTH - 1, 8);

    /* 현재 버전 */
    char ver[16];
    snprintf(ver, sizeof(ver), "v%s", ctx->fw_version[0] ? ctx->fw_version : "?");
    _fb_draw_string(0, 12, ver);

    /* OTA 상태 라인 */
    char st[14];
    switch (ctx->fw_ota_state) {
        case 1:  snprintf(st, sizeof(st), "Checking..");           break;
        case 2:  snprintf(st, sizeof(st), "DL %u%%",
                          (unsigned)ctx->fw_ota_progress);          break;
        case 3:  snprintf(st, sizeof(st), "Applying..");           break;
        case 4:  snprintf(st, sizeof(st), "Delayed");              break;
        case 5:  snprintf(st, sizeof(st), "N/A");                  break;
        default: snprintf(st, sizeof(st), "Idle");                 break;  /* 0 */
    }
    _fb_draw_string(0, 22, st);

    /* 하단 힌트: 진행 중이 아니면 SET=Check / STOP=back */
    if (ctx->fw_ota_state == 0 || ctx->fw_ota_state == 5) {
        _fb_draw_string(0, 32, "SET:check");
    } else {
        _fb_draw_string(0, 32, "STOP:back");
    }
    _fb_flush();
}

void oled_ui_show_fw_update(oled_ui_ctx_t *ctx)
{
    s_ctx = ctx;
    ctx->state           = OLED_STATE_FW_UPDATE;
    ctx->fw_ota_state    = 0;
    ctx->fw_ota_progress = 0;
}

/* ═══════════════════════════════════════════════
   Matter 페어링 화면
═══════════════════════════════════════════════ */
/* ═══════════════════════════════════════════════
   충전 애니메이션 — USB 케이블 연결 중 1분 마다 호출
   72×40 OLED:
     y= 0..7  : "CHARGING" 헤더 (좌우 sliding 효과)
     y=10..28 : 배터리 본체 (48×18) + 단자 (4×8) + fill bar
     y=30..39 : 퍼센트 + 동적 lightning
═══════════════════════════════════════════════ */
static void _render_charging(oled_ui_ctx_t *ctx)
{
    _fb_clear();

    uint32_t now = _ms_now();
    uint32_t elapsed = now - ctx->chg_anim_start_ms;
    uint8_t  f       = ctx->anim_frame;

    /* ── 헤더: "CHARGING" 좌우로 미끄러지는 효과 ── */
    /* 첫 1초: 왼쪽에서 슬라이드 인 */
    int header_x;
    if (elapsed < 1000) {
        /* 0..1000ms → x=-50..6 */
        header_x = -50 + (int)((elapsed * 56) / 1000);
    } else if (elapsed > OLED_CHG_ANIM_DISPLAY_MS - 1000) {
        /* 마지막 1초: 오른쪽으로 슬라이드 아웃 */
        uint32_t out = elapsed - (OLED_CHG_ANIM_DISPLAY_MS - 1000);
        header_x = 6 + (int)((out * 70) / 1000);
    } else {
        header_x = 6;
    }
    _fb_draw_string(header_x, 0, "CHARGING");
    _fb_hline(0, OLED_WIDTH - 1, 9);

    /* ── 배터리 외곽 (48×18, 단자 4×8) ── */
    const int bx = 10, by = 12, bw = 48, bh = 18;
    _fb_rect(bx, by, bw, bh);
    _fb_fill_rect(bx + bw, by + 5, 4, 8);  // 우측 단자

    /* ── Fill bar 애니메이션
       애니메이션 시작 후 0~2.5초간 0%→target% 까지 채워 올림.
       이후엔 target% 유지하되 wave 효과 추가.
    ── */
    int target_w = (bw - 4) * ctx->chg_percent / 100;  // 내부 영역 (44px) × percent
    int fill_w;
    if (elapsed < 2500) {
        fill_w = target_w * (int)elapsed / 2500;
    } else {
        fill_w = target_w;
    }
    if (fill_w < 0) fill_w = 0;
    if (fill_w > bw - 4) fill_w = bw - 4;

    /* 채워진 영역: 점선/줄무늬 패턴으로 dynamic 느낌 */
    for (int dx = 0; dx < fill_w; dx++) {
        for (int dy = 0; dy < bh - 4; dy++) {
            /* 대각선 줄무늬: anim_frame에 따라 흐르는 효과 */
            int stripe = (dx + dy + (f / 2)) % 4;
            if (stripe < 3) {
                _fb_set_pixel(bx + 2 + dx, by + 2 + dy, true);
            }
        }
    }

    /* ── Lightning bolt 깜빡임 ──
       8×16 픽셀 번개 모양, 0.5초 주기로 표시/숨김 */
    bool lightning_visible = (f / 10) % 2 == 0;
    if (lightning_visible && elapsed < OLED_CHG_ANIM_DISPLAY_MS - 1000) {
        /* 단자 위에 작은 lightning 모양 그리기 */
        const int lx = bx + bw / 2 - 3;
        const int ly = by + bh / 2 - 4;
        /* 간단 번개 패턴 (8x9) */
        const uint8_t bolt[9] = {
            0b00000110,
            0b00001100,
            0b00011000,
            0b00110000,
            0b01111110,
            0b00001100,
            0b00011000,
            0b00110000,
            0b01100000,
        };
        for (int row = 0; row < 9; row++) {
            for (int col = 0; col < 8; col++) {
                if (bolt[row] & (0x80 >> col)) {
                    _fb_set_pixel(lx + col, ly + row, true);
                }
            }
        }
    }

    /* ── 하단: 퍼센트 텍스트 (큰 8×8 폰트, 가운데 정렬) ─
     * 예: " 75%" (4글자 × 8 = 32px) → x = (72-32)/2 = 20 */
    char pct_str[8];
    snprintf(pct_str, sizeof(pct_str), "%d%%", ctx->chg_percent);
    int plen = (int)strlen(pct_str);
    int text_x = (OLED_WIDTH - plen * 8) / 2;
    /* 텍스트도 매 0.25초마다 살짝 진동 (1px up/down) */
    int text_y = 30 + ((f / 5) % 2);
    for (int i = 0; i < plen; i++) {
        _fb_draw_char_8x8(text_x + i * 8, text_y, pct_str[i]);
    }

    _fb_flush();
}

/* 코드를 화면 중앙 영역에 표시 (11자리 1줄 / 21자리 2줄). */
static void _draw_pair_code(const oled_ui_ctx_t *ctx)
{
    if (!ctx->pair_code[0]) { _fb_draw_string(0, 14, "Wait..."); return; }
    int len = (int)strlen(ctx->pair_code);
    if (len <= 12) {
        int x = (OLED_WIDTH - len * FONT_W) / 2; if (x < 0) x = 0;
        _fb_draw_string(x, 13, ctx->pair_code);
    } else {
        char buf1[13], buf2[13];
        memcpy(buf1, ctx->pair_code, 12); buf1[12] = '\0';
        snprintf(buf2, sizeof(buf2), "%s", ctx->pair_code + 12);
        int x1 = (OLED_WIDTH - 12 * FONT_W) / 2;          if (x1 < 0) x1 = 0;
        int x2 = (OLED_WIDTH - (int)strlen(buf2)*FONT_W)/2; if (x2 < 0) x2 = 0;
        _fb_draw_string(x1, 11, buf1);
        _fb_draw_string(x2, 20, buf2);
    }
}

/* 가운데 정렬 문자열 (5×7 narrow). */
static void _draw_center(const char *s, int y)
{
    int x = (OLED_WIDTH - (int)strlen(s) * FONT_W) / 2;
    if (x < 0) x = 0;
    _fb_draw_string(x, y, s);
}

/* forward decl: 정의가 디스패처보다 뒤에 있어 묵시적 비-static 선언 충돌 방지. */

#if OLED_RENDER_NATIVE
/* ── 큰 패널(128×64/64×128) 페어링 QR 코드 렌더 ─────────────────────────
 *  Matter QR payload("MT:...")를 esp_qrcode 로 2D 모듈 비트맵 인코딩 후
 *  물리 픽셀(_px/_pfill)로 그린다. 모듈 비트맵은 static 캐시(같은 payload 면
 *  재생성 안 함) — esp_qrcode 핸들/heap 은 generate 내부에서만 일시 사용. */
#include "qrcode.h"

static uint8_t s_qr_mod[(33 * 33 + 7) / 8];   /* 최대 version4(33×33) 비트맵 */
static int     s_qr_size = 0;
static char    s_qr_cached[40] = {0};

static void _qr_capture_cb(esp_qrcode_handle_t qr)
{
    int sz = esp_qrcode_get_size(qr);
    if (sz < 0)  sz = 0;
    if (sz > 33) sz = 33;
    s_qr_size = sz;
    memset(s_qr_mod, 0, sizeof(s_qr_mod));
    for (int y = 0; y < sz; y++)
        for (int x = 0; x < sz; x++)
            if (esp_qrcode_get_module(qr, x, y)) {
                int idx = y * sz + x;
                s_qr_mod[idx >> 3] |= (uint8_t)(1u << (idx & 7));
            }
}

static void _qr_gen_if_needed(const char *payload)
{
    if (s_qr_size > 0 && strncmp(s_qr_cached, payload, sizeof(s_qr_cached)) == 0)
        return;
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func       = _qr_capture_cb;
    cfg.max_qrcode_version = 4;                /* Matter QR ≤ v4(33×33) */
    s_qr_size = 0;
    if (esp_qrcode_generate(&cfg, payload) == ESP_OK && s_qr_size > 0) {
        strncpy(s_qr_cached, payload, sizeof(s_qr_cached) - 1);
        s_qr_cached[sizeof(s_qr_cached) - 1] = '\0';
    } else {
        s_qr_size = 0;                         /* 실패 → 호출측 PIN 폴백 */
    }
}

/* QR 을 화면 우측에 세로중앙으로. quiet zone 1모듈 + 밝은 배경(스캐너 대비).
 *  반환값 = QR 좌측 끝 x (= 좌측 PIN 병기 영역 폭). 실패 시 -1. */
static int _draw_qr_native(const char *payload)
{
    _qr_gen_if_needed(payload);
    if (s_qr_size <= 0) return -1;
    int sz    = s_qr_size;
    int scale = OLED_PANEL_H / (sz + 2);       /* 세로 기준 최대 배율(+2 quiet) */
    if (scale < 1) scale = 1;
    int total = (sz + 2) * scale;
    int x0 = OLED_PANEL_W - total;             /* 우측 정렬 */
    int y0 = (OLED_PANEL_H - total) / 2;
    _pfill(x0, y0, total, total, true);        /* 밝은 배경 + quiet zone */
    int qx = x0 + scale, qy = y0 + scale;
    for (int my = 0; my < sz; my++)
        for (int mx = 0; mx < sz; mx++) {
            int idx = my * sz + mx;
            if (s_qr_mod[idx >> 3] & (1u << (idx & 7)))
                _pfill(qx + mx * scale, qy + my * scale, scale, scale, false);
        }
    return x0;                                  /* 좌측 영역 = [0, x0) */
}
#endif /* OLED_RENDER_NATIVE */

static void _render_pairing(oled_ui_ctx_t *ctx)
{
    _fb_clear();
    uint8_t f = ctx->anim_frame;
    bool blink = (f % 20 < 12);   /* ~0.6s 주기 점멸 */

#if OLED_RENDER_NATIVE
    /* 큰 패널 + QR payload: 대기/준비 단계는 우측 QR + 좌측 PIN 병기.
     * ACTIVE/DONE/FAIL 은 기존 PIN/메시지 유지(스캔 끝났거나 결과 표시). */
    if (ctx->qr_payload[0] &&
        (ctx->pair_phase == OLED_PAIR_WAIT || ctx->pair_phase == OLED_PAIR_READY)) {
        int qr_x = _draw_qr_native(ctx->qr_payload);   /* 우측 QR, 좌측 끝 x */
        if (qr_x > 0) {
            /* 좌측: 상단 "SCAN/PIN" 한 줄 + manual code 를 4-3-4 그룹 3줄로
             *   XXXX- / XXX- / XXXX (11자리 manual pairing code 기준). */
            _pstr8(2, 1, "SCAN/PIN", true);
            int len = (int)strlen(ctx->pair_code);
            if (len >= 11) {
                char l[8];
                snprintf(l, sizeof(l), "%.4s-", ctx->pair_code);       /* XXXX- */
                _pstr8(2, 20, l, true);
                snprintf(l, sizeof(l), "%.3s-", ctx->pair_code + 4);   /* XXX-  */
                _pstr8(2, 35, l, true);
                snprintf(l, sizeof(l), "%.4s",  ctx->pair_code + 7);   /* XXXX  */
                _pstr8(2, 50, l, true);
            } else if (ctx->pair_code[0]) {
                /* 비표준 길이 폴백: 좌측 폭에 맞춰 줄바꿈 */
                int maxc = (qr_x - 3) / 8;  if (maxc < 1) maxc = 1;
                int yy = 20;
                for (int i = 0; i < len && yy <= OLED_PANEL_H - 8; i += maxc) {
                    char line[20];
                    int n = (len - i < maxc) ? (len - i) : maxc;
                    if (n > (int)sizeof(line) - 1) n = (int)sizeof(line) - 1;
                    memcpy(line, ctx->pair_code + i, (size_t)n);
                    line[n] = '\0';
                    _pstr8(2, yy, line, true);
                    yy += 11;
                }
            }
            _fb_flush();
            return;
        }
        /* QR 생성 실패 → 아래 PIN 폴백 진행 */
    }
#endif

    switch (ctx->pair_phase) {

    case OLED_PAIR_DONE:
        /* 완료 — 큰 메시지. main.c 가 3초 후 메인 복귀시킴. */
        _draw_center("MATTER", 4);
        if (blink) _draw_center("SUCCESS", 16);
        _draw_center("Connected", 30);
        break;

    case OLED_PAIR_FAIL:
        /* 실패 — STOP 복귀 전까지 유지. 오류 코드를 점멸 표시. */
        _draw_center("MATTER PAIR", 0);
        _fb_hline(0, OLED_WIDTH - 1, 8);
        _draw_center("FAILED", 12);
        if (ctx->pair_err[0] && blink) {
            _draw_center(ctx->pair_err, 22);   /* 예: "ERR FS-NOC" 점멸 */
        }
        _draw_center("STOP=back", 32);
        break;

    case OLED_PAIR_ACTIVE: {
        /* 페어링 진행 중 — "PAIRING" + 회전 로딩 스피너 + 코드 유지. */
        _draw_center("MATTER PAIR", 0);
        _fb_hline(0, OLED_WIDTH - 1, 8);
        _draw_pair_code(ctx);
        static const char SP[4] = { '|', '/', '-', '\\' };
        char line[12];
        snprintf(line, sizeof(line), "PAIRING %c", SP[(f / 3) % 4]);
        _draw_center(line, 31);
        break;
    }

    case OLED_PAIR_READY:
        /* 사용자가 SETUP 으로 페어링 준비 확정 — "READY" 점멸 + 코드. */
        _draw_center("MATTER PAIR", 0);
        _fb_hline(0, OLED_WIDTH - 1, 8);
        _draw_pair_code(ctx);
        if (blink) _draw_center("READY", 31);
        break;

    case OLED_PAIR_WAIT:
    default:
        /* 페어링 대기 — 코드 표시 + "WAITING" 점멸. */
        _draw_center("MATTER PAIR", 0);
        _fb_hline(0, OLED_WIDTH - 1, 8);
        _draw_pair_code(ctx);
        if (blink) _draw_center("WAITING", 31);
        break;
    }

    _fb_flush();
}

/* ═══════════════════════════════════════════════
   UI 업데이트 태스크 (20fps)
═══════════════════════════════════════════════ */
static void _ui_task(void *pvParam)
{
    oled_ui_ctx_t *ctx = (oled_ui_ctx_t *)pvParam;

    while (1) {
        uint32_t now = _ms_now();
        ctx->anim_frame++;  // ~20fps, 0~255 순환

        uint32_t idle_ms = now - ctx->last_activity_ms;

        /* 상태 전환 로직 */
        if (ctx->state == OLED_STATE_PAIRING) {
            /* 페어링 중 → 다른 상태로 전환 없음 */

        } else if (ctx->state == OLED_STATE_ACTION) {
            /* 버튼 떼고 5초 후 일반 화면 복귀 */
            if (!ctx->action_active) {
                uint32_t action_elapsed = now - ctx->action_start_ms;
                if (action_elapsed >= OLED_ACTION_DISPLAY_MS) {
                    ctx->state  = OLED_STATE_NORMAL;
                    ctx->action = OLED_ACTION_NONE;
                }
            }

        } else if (ctx->state == OLED_STATE_FREQ_EDIT) {
            /* 20초 비활동 시 일반 화면 복귀 */
            if (idle_ms > 20000) {
                ctx->state = OLED_STATE_NORMAL;
            }

        } else if (ctx->state == OLED_STATE_TIME_EDIT) {
            /* 날짜/시간 편집 중 — 화면보호기/자동복귀 없음(사용자 종료까지
             *  유지). s_setup_screen 과 desync 방지 위해 auto-revert 안 함. */

        } else if (ctx->state == OLED_STATE_CHARGING) {
            /* 충전 애니메이션: OLED_CHG_ANIM_DISPLAY_MS 후 자동 복귀 */
            uint32_t anim_elapsed = now - ctx->chg_anim_start_ms;
            if (anim_elapsed >= OLED_CHG_ANIM_DISPLAY_MS) {
                ctx->state = ctx->chg_resume_state ?
                             ctx->chg_resume_state : OLED_STATE_NORMAL;
            }

        }
        /* ★2026-07-24 화면보호기 제거(사용자 요청) — oled_ui 자체의 유휴 타이머로
         *  OLED_STATE_SCREENSAVER 로 넘어가던 경로를 삭제했다.
         *  (somfy_app 쪽 화면보호기 단계는 먼저 지웠는데 이쪽이 남아 있어
         *   "분명히 제거했는데 화면보호기가 나타난다"는 증상이 계속됐다.)
         *  화면 OFF 는 somfy_app 의 유휴 정책(CFG_SCREEN_OFF_SEC)이 단독으로 담당한다. */

        /* 렌더링 */
        switch (ctx->state) {
            case OLED_STATE_NORMAL:      _render_normal(ctx);      break;
            case OLED_STATE_ACTION:      _render_action(ctx);      break;
            case OLED_STATE_SCREENSAVER:
                /* ★2026-07-24 화면보호기 **완전 삭제**(사용자 요청).
                 *  이 상태는 이제 "패널 OFF" 표시용 마커일 뿐이며 아무것도 그리지 않는다.
                 *  (렌더 함수 _render_screensaver 와 페이드 헬퍼는 코드에서 제거함) */
                break;
            case OLED_STATE_FREQ_EDIT:   _render_freq_edit(ctx);   break;
            case OLED_STATE_PAIRING:     _render_pairing(ctx);     break;
            case OLED_STATE_THREAD_PROV: _render_thread_prov(ctx); break;
            case OLED_STATE_CHARGING:    _render_charging(ctx);    break;
            case OLED_STATE_SETUP_MENU:  _render_setup_menu(ctx);  break;
            case OLED_STATE_THREAD_RESET:_render_thread_reset(ctx);break;
            case OLED_STATE_FW_UPDATE:   _render_fw_update(ctx);   break;
            case OLED_STATE_TIME_EDIT:   _render_time_edit(ctx);   break;
        }

        vTaskDelay(pdMS_TO_TICKS(OLED_TASK_INTERVAL_MS));
    }
}

#ifdef OLED_SIM
/* ═══ 웹 시뮬레이터 진입점 — _ui_task 1틱(action 타이머 + 렌더)을 FreeRTOS 없이 직접 호출.
 *  glue.c(emscripten)가 매 프레임 oled_sim_render 를 부른다. 렌더 결과 프레임버퍼 s_fb 를
 *  oled_sim_fb 로 그대로 노출 → 웹 canvas 가 그린다. 펌웨어 렌더 코드를 그대로 실행하므로
 *  oled_ui.c 를 고치면 WASM 재빌드만으로 웹에 반영된다. */
const uint8_t* oled_sim_fb(void){ return &s_fb[0][0]; }
int oled_sim_panel_w(void){ return OLED_PANEL_W; }
int oled_sim_panel_h(void){ return OLED_PANEL_H; }
void oled_sim_render(oled_ui_ctx_t *ctx){
    ctx->anim_frame++;
    uint32_t now = _ms_now();
    if (ctx->state == OLED_STATE_ACTION && !ctx->action_active &&
        (now - ctx->action_start_ms) >= OLED_ACTION_DISPLAY_MS) {
        ctx->state  = OLED_STATE_NORMAL;
        ctx->action = OLED_ACTION_NONE;
    }
    switch (ctx->state) {
        case OLED_STATE_ACTION:     _render_action(ctx);     break;
        case OLED_STATE_SETUP_MENU: _render_setup_menu(ctx); break;
        default:                    _render_normal(ctx);     break;
    }
}
#endif

/* ═══════════════════════════════════════════════
   공개 API 구현
═══════════════════════════════════════════════ */

/* 패널 초기화 시퀀스 (부팅 검출 시 또는 hot-plug 재검출 시 호출).
 *  해상도/오프셋/회전/72×40 보정은 BOARD_OLED_* 가 결정. */
static void _oled_panel_init(void)
{
#if BOARD_OLED_BITBANG
    /* ★비트뱅 모드: 라이브러리 ssd1306_init/clear/contrast 는 내부가 HW I2C 라 못 쓴다.
     *  표준 SSD1306 init 시퀀스를 _oled_send_cmds(=비트뱅)로 직접 보낸다.
     *  (라이브러리 i2c_init() 이 보내던 것과 동일 구성, MUX/COM 만 패널 높이로 분기.) */
    {
        const uint8_t h = OLED_PHYS_H;
        uint8_t seq1[] = { 0xAE,                       /* display off        */
                           0xA8, (uint8_t)(h - 1),     /* mux ratio          */
                           0xD3, 0x00,                 /* display offset     */
                           0x40,                       /* start line 0       */
                           0xA1,                       /* seg remap          */
                           0xC8 };                     /* com scan dec       */
        uint8_t seq2[] = { 0xD5, 0x80,                 /* clk div            */
                           0xDA, (uint8_t)(h == 32 ? 0x02 : 0x12), /* com pins */
                           0x81, 0xFF,                 /* contrast           */
                           0xA4,                       /* display from RAM   */
                           0xDB, 0x40 };               /* vcomh              */
        uint8_t seq3[] = { 0x20, 0x02,                 /* page addressing    */
                           0x00, 0x10,                 /* col start          */
                           0x8D, 0x14,                 /* charge pump on     */
                           0x2E,                       /* scroll off         */
                           0xA6 };                     /* normal (not inv)   */
        /* ★display ON(0xAF)은 여기서 보내지 않는다 — GDDRAM 을 지우기 전에 켜면
         *  전원인가 직후의 **초기화 안 된 랜덤 RAM**이 그대로 보인다(실기에서
         *  "화면에 점이 무수히" 증상으로 확인). 아래 clear 이후에 켠다. */
        _oled_send_cmds(seq1, sizeof(seq1));
        _oled_send_cmds(seq2, sizeof(seq2));
        _oled_send_cmds(seq3, sizeof(seq3));
        s_dev._width = OLED_PHYS_W;
        s_dev._height = OLED_PHYS_H;
    }
#else
    ssd1306_init(&s_dev, OLED_PHYS_W, OLED_PHYS_H);   /* 물리 패널 크기(회전 무관 원본) */
#endif
    s_dev._pages = OLED_PHYS_H / 8;   /* 라이브러리 i2c_init 의 _pages=8 하드코딩 회피(세로 패널 16페이지 대비; 128×64 가로는 8 로 동일) */
#if OLED_PANEL_FIXUP_72X40
    /* 0.42" SSD1315 72×40 전용 보정(멀티플렉스/COM/IREF). 표준 128×64 는 불필요. */
    _ssd1315_apply_72x40_fixup();
#endif
#if BOARD_OLED_BITBANG
    {   /* clear: 전 페이지에 0x00 채움 (라이브러리 대체) — 이 함수는 비트뱅 경로를 탄다 */
        static uint8_t zero[128];
        memset(zero, 0, sizeof(zero));
        for (int p = 0; p < OLED_PHYS_H / 8; p++)
            _oled_write_page_locked(&s_dev, p, 0, zero, OLED_PHYS_W > 128 ? 128 : OLED_PHYS_W);
        uint8_t ct[2] = { 0x81, 0xCF };
        _oled_send_cmds(ct, 2);
        /* ★RAM 을 다 지운 **뒤에** 디스플레이 ON — 랜덤 픽셀 표시 방지(위 주석 참조) */
        uint8_t on[1] = { 0xAF };
        _oled_send_cmds(on, 1);
    }
#else
    ssd1306_clear_screen(&s_dev, false);
    ssd1306_contrast(&s_dev, 0xCF);
#endif
}

/* ── 2026-07-19: 저속 I2C 버스 생성(라이브러리 400k 우회) — 금요일 04:38 버전 복원 ──
 *  라이브러리 i2c_master_init 은 400kHz 를 .c 에 하드코딩하므로, 동일 시퀀스를 직접
 *  구성해 SCL 속도를 OLED_I2C_HZ(100k)로 낮춘다. 낮은 속도 = 글리치 마진↑ → 폭주/멈춤 억제.
 *  (버스+device 생성만. 패널 init 은 검출 성공 후 _oled_panel_init 담당.) */
static void _oled_i2c_init_at(SSD1306_t *dev, int sda, int scl, uint32_t hz, uint16_t addr)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = hz,
    };
    i2c_master_dev_handle_t devh = NULL;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &devh));
    dev->_address        = addr;
    dev->_flip           = false;
    dev->_i2c_num        = I2C_NUM_0;
    dev->_i2c_bus_handle = bus;
    dev->_i2c_dev_handle = devh;
    ESP_LOGI(TAG, "[OLED] I2C 저속 초기화 SDA=IO%d SCL=IO%d %lukHz addr=0x%02X (글리치 마진 확보)",
             sda, scl, (unsigned long)(hz / 1000), addr);
}

void oled_ui_init(oled_ui_ctx_t *ctx)
{
    s_ctx = ctx;

    /* ★2026-08-11 뮤텍스를 **함수 맨 앞**에서 만든다.
     *  전에는 패널 검출 직전(아래)에 만들었는데, 그 사이의 `_bbo_write` 호출과
     *  다른 태스크(btn_handler)의 oled_ui_i2c_lock() 이 s_i2c_mutex==NULL 을 만나
     *  **무보호로 통과**했다. 보호 구멍을 없애려고 앞으로 당긴다. */
    if (!s_i2c_mutex) s_i2c_mutex = xSemaphoreCreateRecursiveMutex();

    /* esp-idf-ssd1306 라이브러리 초기화 — I2C 모드.
     * ESP32-C6-0.42 보드의 0.42" OLED는 IO1(SDA) / IO0(SCL)에
     * 하드와이어 연결됨. RST 핀 없음 (SSD1306 internal POR 사용). */
#if BOARD_OLED_BITBANG
    /* ★비트뱅 모드: HW I2C 버스를 아예 만들지 않는다(페리페럴 고착 회피).
     *  핀만 GPIO 로 확보하고 s_dev 의 주소/크기 필드만 채운다. */
    _bbo_init_pins();
    s_dev._address = BOARD_OLED_ADDR;
    s_dev._i2c_num = I2C_NUM_0;
    s_dev._i2c_bus_handle = NULL;
    s_dev._i2c_dev_handle = NULL;
    ESP_LOGW(TAG, "[OLED] ★비트뱅 I2C 모드 (SDA=IO%d SCL=IO%d, HW 페리페럴 미사용)",
             BOARD_PIN_OLED_SDA, BOARD_PIN_OLED_SCL);
#else
    _oled_i2c_init_at(&s_dev, OLED_PIN_SDA, OLED_PIN_SCL, OLED_I2C_HZ, 0x3C);  /* 400k→100k 저속 */
#endif

    /* (뮤텍스 생성은 이 함수 맨 앞으로 이동 — 위 주석 참조) */

    /* ── OLED 존재 여부 검출(0x3C/0x3D) ────────────────────────────
     *  미연결 보드(예: 배선 전 XIAO)에서 ssd1306 init/flush 의 I2C NACK
     *  로그 스팸(50ms마다)을 막는다. 미검출이면 I2C 스캔으로 진단 로그를
     *  남기고 표시 비활성 + 5초마다 재검출(hot-plug). 검출되면 패널 init. */
    if (_oled_try_detect()) {
        ESP_LOGI(TAG, "OLED init OK (캔버스 %d×%d, 회전=%d)",
                 OLED_WIDTH, OLED_HEIGHT, OLED_ROTATE_180);
    } else {
        s_oled_present = false; g_oled_present_mon = false; _shadow_invalidate();
        s_oled_last_probe_ms = _ms_now();
        ESP_LOGW(TAG, "OLED(0x3C/0x3D) 미검출 — 표시 비활성(5s마다 자동 재검출)");
        _oled_i2c_scan();     /* 진단: 버스에 응답하는 주소 목록 */
        /* _oled_bitbang_diag() 비활성화(2026-07-19): 내부에서 라이브러리 i2c_master_init(400k)로
         * 버스를 재생성하는데, 위 저속 init 이 이미 포트0에 버스를 만들어 둬서 중복 생성 시
         * ESP_ERROR_CHECK 패닉이 난다. 진단 전용이므로 호출만 제거(함수는 보존). */
    }

    /* 기본값 초기화 */
    ctx->last_activity_ms  = _ms_now();
    ctx->state             = OLED_STATE_NORMAL;
    ctx->action            = OLED_ACTION_NONE;
    ctx->action_active     = false;
    ctx->thread_connected  = false;
    ctx->matter_state      = OLED_MT_UNPAIRED;
    ctx->parent_rssi       = OLED_RSSI_INVALID;
    ctx->pair_phase        = OLED_PAIR_WAIT;
    ctx->pair_err[0]       = '\0';
    ctx->anim_frame        = 0;
    strncpy(ctx->time_str, "00:00", sizeof(ctx->time_str));

    /* ── 시작 스플래시 (OLED 검출된 경우만 — 미연결 시 무의미한 지연 회피) ── */
    if (s_oled_present) {
#if OLED_RENDER_128X64
        /* 128×64 네이티브 부팅 (고딕 + 로딩바) — 해상도 기준 선택 */
        _fb_clear();
        _pstr8_center(8,  "SOMFY");
        _pstr8_center(22, "BLIND CTRL");
        _pstr8_center(36, "V1.0");
        for (int w = 0; w <= 100; w += 5) {
            _pfill(14, 52, w, 4, true);
            _fb_flush();
            vTaskDelay(pdMS_TO_TICKS(15));
        }
        vTaskDelay(pdMS_TO_TICKS(600));
#elif OLED_RENDER_64X128
        /* 64×128 세로 부팅 (고딕 로고 세로 적층 + 로딩바) */
        _fb_clear();
        _pstr8_center(24, "SOMFY");
        _pstr8_center(44, "BLIND");
        _pstr8_center(56, "CTRL");
        _pstr8_center(78, "V1.0");
        for (int w = 0; w <= 100; w += 5) {
            _pfill(7, 100, w / 2, 4, true);   /* 64px 폭 → bar 최대 50px */
            _fb_flush();
            vTaskDelay(pdMS_TO_TICKS(15));
        }
        vTaskDelay(pdMS_TO_TICKS(600));
#else
        _fb_clear();
        /* 중앙 정렬 로고 */
        _fb_draw_string(4,  1,  "SOMFY");
        _fb_draw_string(4,  11, "BLIND");
        _fb_draw_string(4,  21, "CTRL");
        /* 버전 표시 (작은 숫자로) */
        _fb_draw_string(48, 31, "v1.0");
        /* 하단 로딩바 애니메이션 */
        for (int i = 0; i <= OLED_WIDTH; i += 4) {
            _fb_hline(0, i, 38);
            _fb_hline(0, i, 39);
            _fb_flush();
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        vTaskDelay(pdMS_TO_TICKS(800));
#endif
    }
}

/* 공유 I2C 버스 핸들 노출 — BOARD_I2C_SHARED 보드에서 PCF8574 가 같은 버스 사용.
 * oled_ui_init() 이후 유효(ssd1306 new-driver 가 s_dev._i2c_bus_handle 채움). */
i2c_master_bus_handle_t oled_ui_get_i2c_bus(void)
{
    return s_dev._i2c_bus_handle;
}

/* 공유 HW I2C 버스 직렬화 — btn_handler 의 PCF8574 read 가 OLED flush(_fb_flush)와
 * 겹치지 않도록 lock/unlock. 뮤텍스 미생성(oled_ui_init 전) 시 무동작. */
/* ★2026-08-11 재귀 뮤텍스로 바뀌었다(s_i2c_mutex 주석 참조) → Take/Give 도
 *  반드시 Recursive 판을 써야 한다. 일반 판을 섞어 쓰면 소유권 검사가 어긋나
 *  해제가 안 되거나(=영구 점유) assert 로 죽는다. */
void oled_ui_i2c_lock(void)
{
    if (s_i2c_mutex) xSemaphoreTakeRecursive(s_i2c_mutex, portMAX_DELAY);
}
void oled_ui_i2c_unlock(void)
{
    if (s_i2c_mutex) xSemaphoreGiveRecursive(s_i2c_mutex);
}

/* ── 2026-07-17 추가: 대기 없는(유한 대기) 버전 ─────────────────────────────
 *  oled_ui_i2c_lock() 은 portMAX_DELAY 라 **영원히** 막힐 수 있다. 실제로
 *  somfy_app 이 BAT ADC 를 읽으려고 이 락을 잡다가 무한 대기에 빠졌다:
 *    - flush 중 NACK 1회 → IDF i2c_master.c 의 NACK 처리기가
 *        while (i2c_ll_is_bus_busy(hal->dev)) { asm("nop"); }   ← 타임아웃 없음
 *      로 스핀 → oled_ui(prio 3) 가 뮤텍스를 **쥔 채** CPU 를 놓지 않음
 *    - somfy_app(prio 4) 은 락 대기로 정지(120초에 BAT 읽기 2회뿐)
 *    - IDLE(prio 0) 이 굶어 5초마다 워치독(17회 × 5초 = 35.7초~120초 전 구간)
 *  → 급하지 않은 호출자(5초 주기 배터리 측정)는 **못 잡으면 그냥 건너뛴다**.
 *    다음 주기에 다시 읽으면 되므로 손해가 없고, 무한 대기·우선순위 역전이 사라진다.
 *  ※ oled_ui_i2c_lock() 자체는 삭제하지 않는다(다른 호출자 유지). */
bool oled_ui_i2c_trylock(uint32_t timeout_ms)
{
    if (!s_i2c_mutex) return true;   /* 뮤텍스 없으면 직렬화 대상 없음 */
    return xSemaphoreTakeRecursive(s_i2c_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void oled_ui_start_task(oled_ui_ctx_t *ctx)
{
    /* ★우선순위 3 = somfy_app(4) **아래**. 절대 5(=somfy_app 위)로 올리지 말 것.
     *
     *  이유(2026-07-17 실측): OLED 가 NACK 하면 ESP-IDF I2C 드라이버가
     *    esp_driver_i2c/i2c_master.c 의
     *        else if (event == I2C_EVENT_NACK) {
     *            while (i2c_ll_is_bus_busy(hal->dev)) { __asm__ __volatile__("nop"); }
     *        }
     *  에서 **타임아웃 없이 무한 바쁜대기**한다(C6 는 SOC_I2C_SUPPORT_HW_FSM_RST=1 이라
     *  i2c_master_bus_reset 의 9클럭 복구 분기도 건너뛴다). 우선순위가 somfy_app 보다
     *  높으면 메인루프를 통째로 굶겨 Task Watchdog 폭주 → RF·버튼·Matter 까지 전부 정지.
     *  실측: prio 5 = 워치독 180~385건·기기 정지 / prio 3 = 워치독 0·정상 동작.
     *  드라이버에 타임아웃이 없어 호출자가 빠져나올 방법이 없으므로 우선순위로 격리한다
     *  → 화면이 죽어도 본체는 산다. (평상시 somfy_app 은 대부분 자므로 프레임률 영향 없음.) */
    xTaskCreate(_ui_task, "oled_ui", 4096, ctx, 3, NULL);
}

void oled_ui_set_display_on(bool on)
{
    /* SSD1315/SSD1306 명령:
     *   0xAE = Display OFF (sleep mode, panel 차단, ~5μA)
     *   0xAF = Display ON
     * I2C 패널이 응답 못해도(전원 OFF 상태 등) 호출은 안전 — _oled_send_cmds
     * 가 i2c_master_transmit 의 timeout 으로 fail 후 반환. */
    if (!s_oled_present) return;   /* 미검출 시 NACK 로그 방지 */
    uint8_t cmd = on ? 0xAF : 0xAE;
    _oled_send_cmds(&cmd, 1);
}

void oled_ui_set_action_blinds(oled_ui_ctx_t *ctx, uint8_t mask)
{
    ctx->action_blind_mask = mask & (uint8_t)((1u << BLIND_MAX_COUNT) - 1);
}

void oled_ui_notify_action_start(oled_ui_ctx_t *ctx, oled_action_t action)
{
    ctx->action         = action;
    ctx->action_active  = true;
    ctx->action_start_ms = _ms_now();
    ctx->last_activity_ms = _ms_now();
    ctx->state          = OLED_STATE_ACTION;
}

void oled_ui_notify_action_end(oled_ui_ctx_t *ctx)
{
    ctx->action_active   = false;
    ctx->last_activity_ms = _ms_now();
    /* action_start_ms는 그대로 유지 → 5초 후 자동 복귀 */
}

void oled_ui_set_blind(oled_ui_ctx_t *ctx, uint8_t idx)
{
    ctx->selected_blind   = idx;
    ctx->last_activity_ms = _ms_now();
    oled_ui_wake(ctx);
}

void oled_ui_set_freq(oled_ui_ctx_t *ctx, float freq_mhz)
{
    ctx->freq_mhz         = freq_mhz;
    ctx->last_activity_ms = _ms_now();
}

void oled_ui_set_time(oled_ui_ctx_t *ctx, const char *time_str)
{
    strncpy(ctx->time_str, time_str, sizeof(ctx->time_str) - 1);
}

void oled_ui_set_qr(oled_ui_ctx_t *ctx, const char *qr_payload)
{
    if (!ctx) return;
    if (qr_payload && qr_payload[0]) {
        strncpy(ctx->qr_payload, qr_payload, sizeof(ctx->qr_payload) - 1);
        ctx->qr_payload[sizeof(ctx->qr_payload) - 1] = '\0';
    } else {
        ctx->qr_payload[0] = '\0';
    }
}

void oled_ui_show_pairing(oled_ui_ctx_t *ctx, const char *pair_code)
{
    strncpy(ctx->pair_code, pair_code, sizeof(ctx->pair_code) - 1);
    ctx->state = OLED_STATE_PAIRING;
    /* 페어링 메뉴(재)진입 시 항상 대기 상태부터 시작.
     * 실패 후 메뉴 복귀 → 재진입해도 WAITING 화면 유지 보장. */
    ctx->pair_phase = OLED_PAIR_WAIT;
}

void oled_ui_set_pair_phase(oled_ui_ctx_t *ctx, oled_pair_phase_t ph)
{
    ctx->pair_phase = ph;
}

/* 커미셔닝 중에는 연속 OLED 태스크를 띄우지 않는다(검증 베이스와 동일하게
 * 802.15.4 operational/SRP 타이밍 보호). 그래도 사용자가 페어링 코드를
 * 봐야 하므로, 여기서 페어링 화면을 *1회만* 그린다(단발 I2C 버스트 —
 * Phase1/2 에서 단발 init 은 페어링에 무해함이 검증됨). */
void oled_ui_render_pairing_once(oled_ui_ctx_t *ctx, const char *pair_code,
                                 oled_pair_phase_t ph)
{
    s_ctx = ctx;
    strncpy(ctx->pair_code, pair_code, sizeof(ctx->pair_code) - 1);
    ctx->pair_code[sizeof(ctx->pair_code) - 1] = '\0';
    ctx->state      = OLED_STATE_PAIRING;
    ctx->pair_phase = ph;
    _render_pairing(ctx);   /* 단 1회 렌더 + flush */
}

void oled_ui_render_main_once(oled_ui_ctx_t *ctx)
{
    s_ctx = ctx;
    ctx->state = OLED_STATE_NORMAL;
    /* ★ OLED 태스크의 자체 idle 타이머도 리셋 — 안 하면 다음 tick 에서
     *  idle_ms ≥ SCREENSAVER 임계값이라 state 가 즉시 SCREENSAVER 로 되돌아가
     *  화면 보호기가 다시 그려진다. */
    ctx->last_activity_ms = _ms_now();
    _render_normal(ctx);    /* 단 1회 렌더 + flush */
}

void oled_ui_set_thread(oled_ui_ctx_t *ctx, bool attached)
{
    ctx->thread_connected = attached;
}

void oled_ui_set_matter_status(oled_ui_ctx_t *ctx,
                               oled_matter_state_t st, int8_t rssi)
{
    ctx->matter_state = st;
    ctx->parent_rssi  = rssi;
    /* thread_connected 는 다른 곳(화면 보호기 등) 호환을 위해 동기화 */
    ctx->thread_connected = (st == OLED_MT_CONNECTED);
}

void oled_ui_show_thread_prov(oled_ui_ctx_t *ctx,
                              const char *net_name, const char *pair_code)
{
    if (net_name) {
        strncpy(ctx->thread_prov_name, net_name, sizeof(ctx->thread_prov_name) - 1);
    }
    if (pair_code) {
        strncpy(ctx->thread_prov_qr, pair_code, sizeof(ctx->thread_prov_qr) - 1);
    }
    ctx->state = OLED_STATE_THREAD_PROV;
    ctx->last_activity_ms = _ms_now();
}

void oled_ui_notify_blind_select(oled_ui_ctx_t *ctx, uint8_t idx)
{
    if (!ctx) return;
    ctx->selected_blind = idx;
    ctx->state = OLED_STATE_NORMAL;
    ctx->last_activity_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

void oled_ui_wake(oled_ui_ctx_t *ctx)
{
    ctx->last_activity_ms = _ms_now();
    if (ctx->state == OLED_STATE_SCREENSAVER) {
        ctx->state = OLED_STATE_NORMAL;
    }
}

void oled_ui_show_setup_menu(oled_ui_ctx_t *ctx, uint8_t cursor)
{
    if (!ctx) return;
    ctx->setup_cursor = (cursor < SETUP_MENU_COUNT) ? cursor : 0;
    ctx->state = OLED_STATE_SETUP_MENU;
    ctx->last_activity_ms = _ms_now();
}

void oled_ui_set_setup_cursor(oled_ui_ctx_t *ctx, uint8_t cursor)
{
    if (!ctx) return;
    if (cursor >= SETUP_MENU_COUNT) cursor = SETUP_MENU_COUNT - 1;
    ctx->setup_cursor = cursor;
    ctx->last_activity_ms = _ms_now();
    /* state 가 다른 화면이면 자동 전환하지 않음 — 호출자가 제어 */
}

void oled_ui_show_thread_reset(oled_ui_ctx_t *ctx)
{
    if (!ctx) return;
    ctx->state = OLED_STATE_THREAD_RESET;
    ctx->last_activity_ms = _ms_now();
}

void oled_ui_show_time_edit(oled_ui_ctx_t *ctx, const int v[5], uint8_t field)
{
    if (!ctx || !v) return;
    for (int i = 0; i < 5; i++) ctx->time_edit_val[i] = v[i];
    ctx->time_edit_field = (field < 5) ? field : 0;
    ctx->state = OLED_STATE_TIME_EDIT;
    ctx->last_activity_ms = _ms_now();
}

void oled_ui_set_time_edit(oled_ui_ctx_t *ctx, const int v[5], uint8_t field)
{
    if (!ctx || !v) return;
    for (int i = 0; i < 5; i++) ctx->time_edit_val[i] = v[i];
    ctx->time_edit_field = (field < 5) ? field : 0;
    ctx->last_activity_ms = _ms_now();
}

void oled_ui_show_charging(oled_ui_ctx_t *ctx, uint8_t percent)
{
    if (!ctx) return;
    if (percent > 100) percent = 100;
    /* 현재 상태 저장 (애니메이션 종료 후 복귀) — 단 충전 애니메이션
     * 자체나 페어링/프로비저닝 화면이라면 NORMAL로 복귀. */
    if (ctx->state != OLED_STATE_CHARGING &&
        ctx->state != OLED_STATE_PAIRING &&
        ctx->state != OLED_STATE_THREAD_PROV) {
        ctx->chg_resume_state = ctx->state;
    } else if (ctx->state == OLED_STATE_PAIRING ||
               ctx->state == OLED_STATE_THREAD_PROV) {
        /* 페어링/프로비저닝 중에는 충전 애니메이션 인터럽트 금지 */
        return;
    }
    ctx->chg_percent       = percent;
    ctx->chg_anim_start_ms = _ms_now();
    ctx->state             = OLED_STATE_CHARGING;
    ctx->last_activity_ms  = _ms_now();   // screensaver 진입 방지
}
