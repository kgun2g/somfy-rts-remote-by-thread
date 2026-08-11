#pragma once
/*
 * somfy_config.h
 * ──────────────────────────────────────────────────────────
 * 프로젝트 전체 설정 중앙 관리 파일
 *
 * 값 우선순위:
 *   1. idf.py menuconfig (Kconfig.projbuild)
 *   2. sdkconfig.defaults
 *   3. 이 파일의 기본값 (fallback)
 *
 * 핀 번호: boards/board_select.h 의 BOARD_PIN_* 매크로에서 참조.
 *         (보드별 핀맵은 boards/<board>.h 에 정의. 빌드 시 -DBOARD_* 로 선택.)
 * ──────────────────────────────────────────────────────────
 */

#include "sdkconfig.h"
#include "boards/board_select.h"

/* CC1101 SPI 핀의 단일 진실원천은 cc1101.h 의 CC1101_PIN_* 이다
 * (boards/<board>.h 의 BOARD_PIN_CC1101_* 에서 파생). */

/* ════════════════════════════════════════════════════════
   PCF8574 I2C 핀 — bit-bang I2C (소프트웨어 구동, 외부 4.7kΩ pull-up 필수)
   값은 boards/<board>.h 의 BOARD_PIN_PCF_SDA/SCL 에서 가져온다.
   ════════════════════════════════════════════════════════ */
#define CFG_PCF8574_SDA  BOARD_PIN_PCF_SDA
#define CFG_PCF8574_SCL  BOARD_PIN_PCF_SCL

/* ════════════════════════════════════════════════════════
   OLED I2C 핀 (보드별 변경 가능)
   ════════════════════════════════════════════════════════ */
#define CFG_OLED_SDA   BOARD_PIN_OLED_SDA
#define CFG_OLED_SCL   BOARD_PIN_OLED_SCL
/* OLED 규격은 보드별 — boards/<board>.h 의 BOARD_OLED_* 가 단일 진실원천.
 * (실제 렌더링 치수/회전/보정은 oled_ui.h/oled_ui.c 가 BOARD_OLED_* 로 결정.) */
#define CFG_OLED_ADDR   BOARD_OLED_ADDR     // I2C 주소 (보드별, 보통 0x3C)
#define CFG_OLED_WIDTH  BOARD_OLED_WIDTH    // 물리 패널 픽셀 너비 (GNPE 72 / XIAO 128)
#define CFG_OLED_HEIGHT BOARD_OLED_HEIGHT   // 물리 패널 픽셀 높이 (GNPE 40 / XIAO 64)

/* ════════════════════════════════════════════════════════
   버튼/센서 핀 (Active-LOW, 내부 Pull-UP) ─ PCB v2.0
   ════════════════════════════════════════════════════════
   ★ 모든 버튼은 PCF8574 (I2C 0x20) 로 이관됨.
     light sleep 중 PCF8574 ~INT (Pin13) 가 IO17 을 LOW 로 끌어
     GPIO wake 가능 → 모든 P 변화로 sleep 해제.

   PCF8574 P 핀 (active-LOW, 내부 pull-up):
     P0 = ROT_A           (로터리 엔코더 A상)
     P1 = ROT_B           (로터리 엔코더 B상)
     P2 = ROT_BTN         (로터리 푸시 버튼)
     P3 = SW6 SETUP       (설정/페어링)
     P4 = SW1 BTN_UP
     P5 = SW2 BTN_DOWN
     P6 = SW3 BTN_SELECT
     P7 = SW4 BTN_PROG

   GPIO 직결 (light sleep wake source) — v2.0 핀 재배치:
     IO17 = PCF8574 ~INT      (구 IO2 → IO17)
     IO16 = VS1 VIBRATION     (active-LOW, wake 가능)
     IO3  = MCP73831 CHG_STAT (active-LOW, wake 가능)
     IO2  = 예약 (미사용 ← v2.0 에서 freed up)
   ════════════════════════════════════════════════════════ */
#define CFG_PCF8574_INT  BOARD_PIN_PCF_INT
#define CFG_CHG_STAT_PIN BOARD_PIN_CHG_STAT
#define CFG_VIBE_PIN     BOARD_PIN_VIBE

/* ════════════════════════════════════════════════════════
   RF 주파수 설정 (Kconfig 또는 기본값)
   ════════════════════════════════════════════════════════ */
#define CFG_FREQ_MIN_MHZ 447.20f
#define CFG_FREQ_MAX_MHZ 447.79f
#define CFG_FREQ_STEP_MHZ 0.01f

/* menuconfig 값으로 기본 주파수 계산 */
#ifdef CONFIG_SOMFY_FREQ_DEFAULT_MHZ_INT
#define CFG_FREQ_DEF_MHZ (CONFIG_SOMFY_FREQ_DEFAULT_MHZ_INT / 100.0f)
#else
/* RX OOK 디코더로 정밀 확정: 정품 한국 Somfy RTS 리모컨이 447.60MHz
 *  에서만 깨끗이 디코드됨(HW싱크/심볼 다수). RX·TX 동일 CC1101 이므로
 *  447.60 그대로 송신에 사용. (RSSI 스캐너의 447.65 는 넓은 플래토 오해) */
#define CFG_FREQ_DEF_MHZ 447.60f
#endif

/* ════════════════════════════════════════════════════════
   Somfy RTS 타이밍 (μs) - 프로토콜 표준값, 변경 주의
   ════════════════════════════════════════════════════════ */
#define CFG_SOMFY_T_SYMBOL 640        // Manchester 심볼 반주기
#define CFG_SOMFY_T_HWSYNC_ON 2416    // 하드웨어 동기 HIGH
#define CFG_SOMFY_T_HWSYNC_OFF 2416   // 하드웨어 동기 LOW
#define CFG_SOMFY_T_SWSYNC_ON 4832    // 소프트웨어 동기 HIGH
#define CFG_SOMFY_T_SWSYNC_OFF 640    // 소프트웨어 동기 LOW
#define CFG_SOMFY_T_INTER_FRAME 30400 // 프레임 간격

/* 반복 횟수는 somfy_rts.h 의 SOMFY_REPEAT_COUNT 하나로 일원화한다.
 *  (과거 CFG_SOMFY_REPEAT_COUNT 는 송신 경로에서 미사용·CONFIG_ 심볼도 없어
 *   menuconfig 로 도달 불가한 사표 매크로였다.) */

/* ════════════════════════════════════════════════════════
   버튼 타이밍 (Kconfig 또는 기본값)
   ════════════════════════════════════════════════════════ */
#ifdef CONFIG_BTN_DEBOUNCE_MS
#define CFG_BTN_DEBOUNCE_MS CONFIG_BTN_DEBOUNCE_MS
#else
#define CFG_BTN_DEBOUNCE_MS 20
#endif

#ifdef CONFIG_BTN_LONG_PRESS_MS
#define CFG_BTN_LONG_PRESS_MS CONFIG_BTN_LONG_PRESS_MS
#else
#define CFG_BTN_LONG_PRESS_MS 2000
#endif

#ifdef CONFIG_BTN_HOLD_REPEAT_MS
#define CFG_BTN_HOLD_REPEAT_MS CONFIG_BTN_HOLD_REPEAT_MS
#else
#define CFG_BTN_HOLD_REPEAT_MS 500
#endif

#define CFG_BTN_MIN_HOLD_MS 100
#define CFG_BTN_MAX_HOLD_MS 15000

/* ════════════════════════════════════════════════════════
   OLED 타이밍 (Kconfig 또는 기본값)
   ════════════════════════════════════════════════════════ */
#define CFG_OLED_FPS 20 // 프레임레이트 (FreeRTOS 태스크 50ms 주기)

#ifdef CONFIG_OLED_SCREENSAVER_IDLE_SEC
#define CFG_OLED_IDLE_SS_MS (CONFIG_OLED_SCREENSAVER_IDLE_SEC * 1000)
#else
#define CFG_OLED_IDLE_SS_MS 120000
#endif

#ifdef CONFIG_OLED_ACTION_DISPLAY_SEC
#define CFG_OLED_ACTION_MS (CONFIG_OLED_ACTION_DISPLAY_SEC * 1000)
#else
#define CFG_OLED_ACTION_MS 5000
#endif

#ifdef CONFIG_OLED_COL_OFFSET
#define CFG_OLED_COL_OFFSET CONFIG_OLED_COL_OFFSET
#else
#define CFG_OLED_COL_OFFSET BOARD_OLED_COL_OFFSET  // 보드별 (GNPE 28 / XIAO 0)
#endif

/* ════════════════════════════════════════════════════════
   블라인드 설정
   ════════════════════════════════════════════════════════ */
/* 채널 수의 단일 진실원천은 BLIND_MAX_COUNT(boards/<board>.h — H2=3, C6=8).
 *  과거의 독립 기본값 5 는 BLIND_MAX_COUNT 와 어긋나 _blind_cycle 버그를 유발했으므로
 *  별칭으로 고정해 절대 발산하지 않게 한다. */
#define CFG_BLIND_MAX_COUNT BLIND_MAX_COUNT

#define CFG_BLIND_NAME_LEN 16

/* ════════════════════════════════════════════════════════
   Matter 설정
   ════════════════════════════════════════════════════════ */
#ifdef CONFIG_MATTER_VENDOR_ID
#define CFG_MATTER_VENDOR_ID CONFIG_MATTER_VENDOR_ID
#else
#define CFG_MATTER_VENDOR_ID 0xFFF1
#endif

#ifdef CONFIG_MATTER_PRODUCT_ID
#define CFG_MATTER_PRODUCT_ID CONFIG_MATTER_PRODUCT_ID
#else
#define CFG_MATTER_PRODUCT_ID 0x8001
#endif

#ifdef CONFIG_MATTER_DEVICE_NAME
#define CFG_MATTER_DEVICE_NAME CONFIG_MATTER_DEVICE_NAME
#else
#define CFG_MATTER_DEVICE_NAME "Somfy Blind Ctrl"
#endif

/* ════════════════════════════════════════════════════════
   NVS 키
   ════════════════════════════════════════════════════════ */
#ifdef CONFIG_SOMFY_NVS_NAMESPACE
#define CFG_NVS_NAMESPACE CONFIG_SOMFY_NVS_NAMESPACE
#else
#define CFG_NVS_NAMESPACE "somfy_blinds"
#endif

#define CFG_NVS_KEY_BLINDS "blinds_cfg"

/* ════════════════════════════════════════════════════════
   SNTP / 시간
   ════════════════════════════════════════════════════════ */
#define CFG_SNTP_SERVER "pool.ntp.org"

#ifdef CONFIG_SOMFY_TIMEZONE
#define CFG_TIMEZONE CONFIG_SOMFY_TIMEZONE
#else
#define CFG_TIMEZONE "KST-9"
#endif

#define CFG_SNTP_UPDATE_MS 10000 // 10초마다 시간 갱신

/* ── 화면 자동 OFF 시간 (2026-07-23) ────────────────────────────────
 *   유휴(버튼·진동·Matter 명령 없음) 이 시간(초)을 넘기면 OLED 패널을 끈다.
 *   버튼을 누르거나 진동이 감지되면 즉시 다시 켜진다.
 *   ※구 "화면보호기(중간 애니메이션)" 단계는 삭제됨 — 이 값 하나로만 제어한다.
 *   보드별로 다르게 하려면 boards/<board>.h 에서 먼저 정의하면 된다. */
#ifndef CFG_SCREEN_OFF_SEC
#define CFG_SCREEN_OFF_SEC   10   /* 초. 예: 10=10초, 60=1분, 180=3분 */
#endif
/* ★2026-08-11 USB 연결 시에는 더 길게 (사용자 요청).
 *  왜 나누나: 배터리 구동은 화면이 소비의 큰 몫이라 짧게 꺼야 하지만, USB 는 전원이
 *  무제한이므로 작업 중 화면이 10초마다 꺼지면 불편하다. `usb_mode`(_is_usb_powered)
 *  로 분기해 같은 유휴 로직에 서로 다른 문턱만 적용한다.
 *  ※배터리값(CFG_SCREEN_OFF_SEC)은 그대로 10초 유지. */
#ifndef CFG_SCREEN_OFF_USB_SEC
#define CFG_SCREEN_OFF_USB_SEC  300   /* 초. USB 연결 시 5분 */
#endif
