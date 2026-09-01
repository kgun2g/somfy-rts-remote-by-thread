/*
 * pcf_poll.c — LP 코어 프로그램: PCF8574 폴링 + 로터리 쿼드러처 디코딩
 * ═════════════════════════════════════════════════════════════════════
 * 이 파일은 **LP 코어(ULP-RISC-V)에서 실행**된다. HP(메인 CPU)와는 공유 RAM
 * 변수로만 통신한다. HP 쪽에서는 `ulp_<변수명>` 으로 접근한다.
 *
 * 왜 존재하나 (③)
 * ----------------
 * 실측 6.64시간 평균 57mA / 사용시간 12.3h. 원인은 Matter 가 아니라 HP 가
 * **10ms 마다 깨어나 PCF8574 를 비트뱅 I2C 로 읽는 것**이었다(초당 100회).
 * PCF 의 `~INT` 로 이벤트 구동하려던 시도(②)는 실측으로 불가 판정됐다
 * (버튼을 눌러도 선이 안 움직임 — 표본 290,023 / 전이 0회).
 * → 남은 방법은 **폴링 주체를 LP 코어로 옮기는 것**뿐이다.
 *
 * ★로터리를 여기서 디코딩하는 이유 (②가 깨진 바로 그 지점)
 * -----------------------------------------------------------
 * 디텐트 1개는 A/B 전이 2회가 **수 ms 안에** 끝난다. LP 가 "변화 있음"만 알리고
 * HP 가 깨어나서 읽으면 그 전이는 이미 지나가 **방향을 알 수 없다**.
 * 그래서 쿼드러처 누산을 여기서 끝내고 **디텐트 개수(부호 포함)** 만 넘긴다.
 * 버튼은 눌림이 100ms 이상 유지되므로 HP 가 나중에 읽어도 안전하다.
 *
 * 통신 규약 (공유 RAM)
 * --------------------
 *   pcf_state   최신 PCF 1바이트 원본 (HP 가 버튼 디바운스에 그대로 사용)
 *   rot_delta   누적 디텐트. HP 가 읽고 **뺀 만큼 차감**한다(소비형).
 *   seq         상태가 바뀔 때마다 증가 — HP 는 이 값으로 변화 유무를 판단
 *   poll_cnt    폴링 누적(살아있음 확인)
 *   i2c_err     I2C 실패 누적 — 늘어나면 LP 경로를 접고 HP 폴링으로 되돌릴 근거
 *   enabled     HP 가 0 으로 두면 LP 는 I2C 를 건드리지 않는다(HP 가 버스를 쓸 때)
 */

#include <stdint.h>
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_i2c.h"
#include "pcf_lp_config.h"   /* ★LP·HP 공용 — 읽기 폭/비트 위치 */

/* C6 는 LP_I2C 포트가 하나뿐(SOC_LP_I2C_NUM=1) */
#define PCF_I2C_PORT   LP_I2C_NUM_0
#define PCF_I2C_ADDR   0x20        /* PCF8574/8575 (A0~A2 = GND) — 부팅 로그와 동일 */

#define I2C_TIMEOUT_CYCLES 5000    /* 예제와 동일. 무한 대기 금지(멈추면 복구 불가) */
/* 폴 주기는 공유 헤더로 옮겼다 — HP 가 LP 타이머 주기로 같은 값을 써야 한다. */
#define POLL_US LP_POLL_US

/* ── HP 와 공유하는 변수 (HP 에서는 ulp_* 로 접근) ─────────────────────── */
volatile uint32_t pcf_state  = 0xFFFFFFFFu; /* 최신 원본(8/16비트). 0xFFFFFFFF = 아직 읽기 전 */
volatile int32_t  rot_delta  = 0;        /* 누적 디텐트(+CW/-CCW). HP 가 차감 */
volatile uint32_t seq        = 0;        /* 변화 카운터 */
/* ★★★2026-08-23 **눌림 래치** — HP 폴 주기를 늘려도 짧은 누름을 잃지 않게 한다.
 *
 *  왜 필요한가: HP 는 25ms 마다 `pcf_state`(=현재 상태)를 읽는다. 폴과 폴 사이에
 *  눌렀다 뗀 누름은 **통째로 사라진다**. 그래서 주기를 못 늘렸고, 그게 전체
 *  깨어남의 53%(46.8회/초)를 차지한다.
 *  → LP 가 2ms 마다 보면서 **한 번이라도 눌렸던 비트를 OR 로 모아 둔다**.
 *    HP 는 "지금 눌림 OR 그동안 눌렸음" 으로 판정하므로 누락이 없다.
 *
 *  버튼은 active-low(풀업, 눌림=0)라 `~rx` 의 1 비트가 '눌렸음' 이다.
 *  소비 규약은 rot_delta 와 동일하게 **HP 가 본 것만 차감**한다:
 *      uint32_t d = ulp_press_latch;  ...사용...  ulp_press_latch &= ~d;
 *  (HP 가 통째로 0 을 쓰면, 읽기와 쓰기 사이에 LP 가 OR 한 비트를 잃는다.) */
volatile uint32_t press_latch = 0;
volatile uint32_t poll_cnt   = 0;        /* 폴링 누적(생존 확인) */
volatile uint32_t i2c_err    = 0;        /* I2C 실패 누적 */
volatile uint32_t enabled    = 1;        /* HP 가 0 으로 내리면 I2C 정지 */

/* ── 쿼드러처 LUT — button_handler.c 의 kQuad 와 **같은 표** ────────────
 *  인덱스 = (이전 AB << 2) | 현재 AB. ±1 이 모여 ±2 가 되면 1 디텐트.
 *  바운스(왕복)는 +1/−1 로 상쇄돼 자동 제거된다. */
static const int8_t kQuad[16] = {
    0, +1, -1,  0,
   -1,  0,  0, +1,
   +1,  0,  0, -1,
    0, -1, +1,  0,
};

/* PCF8574 비트 배치 — button_handler.h 와 동일해야 한다.
 * 여기서 헤더를 include 하지 않는 이유: LP 코어 툴체인은 HP 쪽 헤더 체인
 * (FreeRTOS/esp_log 등)을 못 따라간다. 값만 최소로 복제하고, 어긋나면
 * 로터리가 안 도는 형태로 즉시 드러난다. */
#define BIT_ROT_A   LP_BIT_ROT_A
#define BIT_ROT_B   LP_BIT_ROT_B

/* ★★★2026-09-01 상태를 **파일 스코프**로 올렸다 — halt/타이머 방식의 전제다.
 *  main() 이 폴 1회마다 새로 호출되므로 자동 변수로 두면 매번 초기화돼
 *  쿼드러처 누산(accum/prev_ab)·변화 감지(last)·2연속 래치(prev_low)가 전부 깨진다.
 *  파일 스코프 static 은 LP RAM 에 남아 폴 사이에 보존된다. */
static uint8_t  prev_ab  = 0x03;         /* 디텐트 기본 위치(11) */
static int8_t   accum    = 0;
static uint32_t last     = 0xFFFFFFFFu;  /* '아직 안 읽음' 표식 — 16비트 값과 겹치면 안 된다 */
static uint32_t prev_low = 0;            /* 2연속 눌림 판정용 직전 표본 */

/* ★★★2026-09-01 **busy-wait 무한루프 → 1회 실행 후 halt** 로 바꿨다.
 *
 *  왜 (사용자 지적: "LP 코어로 도는데 어떻게 H2 보다 더 먹냐"):
 *    기존 구조는 `while(1) { ...; ulp_lp_core_delay_us(POLL_US); }` 였다.
 *    `ulp_lp_core_delay_us()` 는 IDF 문서상 **busy-wait** 다("Makes the
 *    co-processor busy-wait"). 즉 LP CPU 가 **100% 듀티로 계속 돌고** 있었고,
 *    LP_I2C·LP 페리페럴 전원 도메인도 내내 켜져 있었다.
 *    LP 코어를 넣은 목적이 정확히 반대로 뒤집혀 있던 셈이다.
 *
 *  IDF 가 의도한 관용구(examples/system/ulp/lp_core/lp_adc):
 *      HP:  cfg.wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER;
 *           cfg.lp_timer_sleep_duration_us = <주기>;
 *      LP:  int main(void) { ...1회 작업...; return 0; }
 *    main() 이 반환하면 lp_core_startup() 이 LP 타이머를 걸고
 *    `ulp_lp_core_halt()` 를 부른다(components/ulp/lp_core/lp_core/lp_core_startup.c).
 *
 *  → LP CPU 활성 듀티가 100% → **1회 읽기 시간 / 주기** 로 떨어진다.
 *    폴 주기(LP_POLL_US)는 **바꾸지 않았다** — 동작·응답성은 그대로다.
 *
 *  ※`do { } while (0)` 로 감싼 이유: 본문의 `continue` 가 "이번 폴 종료"로
 *    그대로 동작해 들여쓰기·로직을 하나도 안 건드리고 옮길 수 있다. */
int main(void)
{
    do {
        if (!enabled) {
            continue;
        }

        /* PCF8574/8575 는 레지스터 주소가 없다 — 그냥 read 하면 현재 입력이 나온다.
         * PCF8575 는 **저바이트(P00~P07) 먼저, 고바이트(P10~P17) 나중**에 나온다. */
        uint8_t buf[2] = { 0xFF, 0xFF };
        esp_err_t err = lp_core_i2c_master_read_from_device(
                            PCF_I2C_PORT, PCF_I2C_ADDR, buf, LP_PCF_NBYTES,
                            I2C_TIMEOUT_CYCLES);
        uint16_t rx = (LP_PCF_NBYTES == 2)
                        ? (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8))
                        : (uint16_t)buf[0];
        if (err != ESP_OK) {
            i2c_err++;
            continue;
        }

        poll_cnt++;

        /* ── 로터리: 전이가 생길 때마다 즉시 누산 (여기가 핵심) ──
         *  ★비트 순서는 HP 쪽(button_handler.c)과 **반드시 동일**해야 한다:
         *      ab = (A << 1) | B
         *  같은 kQuad 표를 쓰는데 순서가 다르면 방향이 뒤집히거나 디텐트를 놓친다. */
        uint8_t ab = (uint8_t)((((rx >> BIT_ROT_A) & 1) << 1) | ((rx >> BIT_ROT_B) & 1));
        if (ab != prev_ab) {
            accum = (int8_t)(accum + kQuad[((prev_ab & 3) << 2) | (ab & 3)]);
            prev_ab = ab;
            if (accum >= 2)       { rot_delta++; accum = 0; seq++; }
            else if (accum <= -2) { rot_delta--; accum = 0; seq++; }
        }

        /* ★2026-08-23 눌림 래치 누적 (선언부 주석 참조).
         *  ① 상위 8비트 마스크 — LP_PCF_NBYTES 가 1 이면 의미 없다.
         *  ② **로터리(A/B) 제외** — 회전 중 계속 토글하므로 래치하면 영구히 눌림으로
         *     굳는다. 실제로 첫 시험에서 `래치 0x0003`(= A/B 비트)이 지워지지 않고
         *     남았다. 로터리는 rot_delta 로 이미 누적되므로 래치가 필요 없다.
         *  ③ **2연속(4ms) 눌림만 인정** — 이게 없으면 2ms 짜리 글리치 하나가 그대로
         *     '누름'으로 래치된다. 전에는 HP 가 25ms 로 띄엄띄엄 봐서 그런 글리치를
         *     대부분 놓쳤는데, 래치는 **전부 잡아 버린다**. PROG 자동 송신 사고
         *     (2026-08-17)의 원인이 PCF 읽기 흔들림으로 의심됐던 만큼, 여기서
         *     한 번 걸러야 안전하다. */
        {
            const uint32_t mask = ((LP_PCF_NBYTES == 2) ? 0xFFFFu : 0xFFu)
                                & ~((1u << BIT_ROT_A) | (1u << BIT_ROT_B));
            const uint32_t low = ((uint32_t)(~rx)) & mask;
            press_latch |= (low & prev_low);
            prev_low = low;
        }

        /* ── 버튼/전체 상태 변화 → HP 에게 알림 ── */
        if (rx != last) {
            last = rx;
            pcf_state = rx;
            seq++;
            /* HP 가 light sleep 중이면 깨운다. 깨어 있으면 무해(무시된다). */
            ulp_lp_core_wakeup_main_processor();
        }
    } while (0);
    return 0;      /* → lp_core_startup() 이 LP 타이머 세팅 후 halt (위 주석) */
}
