#pragma once
/*
 * boards/xiao-c6-test.h
 * ──────────────────────────────────────────────────────────
 * XIAO ESP32-C6 **테스트 보드** (COM6, 2026-08-16 사용자 제공 사양)
 *
 * 실기와 다른 점만 여기서 덮는다. 핀맵·나머지 설정은 xiao-c6.h 를 그대로
 * 상속하므로, 실기 보드 설정이 바뀌면 이 보드도 자동으로 따라간다
 * (복사본을 두면 두 파일이 조용히 어긋난다 — 그래서 include 방식).
 *
 *   · PCF8574 (8비트)      — 실기는 PCF8575(좌/우 버튼 P10/P11)
 *   · LP 코어 미적용        — PCF 를 공유 HW I2C(22/23)로만 읽는다
 *   · 기본 주파수 447.72MHz — 실기 기본은 447.70
 *   · OLED 128×64 / 180도   — 실기와 동일(참고용으로 명시)
 *   · 로터리 EC05          — 실기와 동일(BOARD_ROT_HALF_STEP=1)
 *   · 배터리 없음, 충전/배터리 측정 회로 없음
 *
 * ★왜 별도 보드 키인가: build.ps1 은 보드마다 별도 빌드 디렉터리
 *   (build-<board>/)와 sdkconfig 를 쓴다. 같은 키로 변형 플래그만 바꿔 쓰면
 *   실기(COM7)와 테스트(COM6)를 오갈 때마다 **전체 재빌드**가 난다.
 * ──────────────────────────────────────────────────────────
 */

#include "boards/xiao-c6.h"

/* ── PCF8574(8비트) — 좌/우 버튼 없음 ──────────────────────────────────
 *  실기(xiao-c6.h)는 PCF8575 라 1 이다. 여기서 0 으로 되돌린다.
 *  ※이 값이 0 이면 PCF_NBYTES=1 이 되는데, LP 코어 프로그램은
 *    lp_core/pcf_lp_config.h 의 LP_PCF_NBYTES=2 **고정**이라 그대로 두면
 *    button_handler.c 의 _Static_assert 가 빌드를 깬다. 아래에서 LP 를 끄는
 *    이유가 이것이기도 하다(사용자 사양 "lp 미적용"과도 일치). */
#undef  BOARD_HAS_LR_BUTTONS
#define BOARD_HAS_LR_BUTTONS   0

/* ── LP 코어 미적용 — PCF 를 공유 HW I2C 로만 읽는다 ────────────────────
 *  LP 프로그램은 main/CMakeLists.txt 가 `BOARD STREQUAL "xiao-c6"` 일 때만
 *  빌드하므로, 보드 키가 다른 이 보드에서는 **애초에 빌드되지 않는다**.
 *  여기 0 은 button_handler.c 의 LP 경로(#if BOARD_PCF_LP_CORE)를 함께 닫는다. */
#undef  BOARD_PCF_LP_CORE
#define BOARD_PCF_LP_CORE      0

/* LP_I2C 프로브→공유 폴백도 불필요(처음부터 공유 경로). */
#undef  BOARD_I2C_LP_FALLBACK
#define BOARD_I2C_LP_FALLBACK  0

/* ── ★★★OLED 를 HW I2C 로 (비트뱅 끔) — 공유 버스에서는 필수 ────────────
 *  실기(xiao-c6.h)는 BOARD_OLED_BITBANG=1 이다. HW I2C0 가 "bus busy" 로
 *  고착되는 개체가 있어 페리페럴을 통째로 우회하려고 넣은 값이고, 실기는
 *  **PCF 가 LP 전용핀(6/7)** 에 있어 22/23 을 비트뱅해도 충돌이 없다.
 *
 *  그런데 이 테스트 보드는 PCF 가 **같은 22/23 공유 HW I2C** 다. 비트뱅 모드는
 *  board_select.h 설명대로 "HW I2C 버스를 만들지 않는다" → PCF 가 쓸 버스
 *  핸들이 없고, 같은 핀을 두 드라이버가 만지게 된다.
 *  실측 증상(2026-08-16): 화면은 한 번 그려진 뒤 **멈추고**, 버튼도 함께 죽음.
 *      E OLED_UI: [BBFAIL] #121 len=0
 *      W OLED_UI: 라인레벨 검출실패: SDA(IO22)=1 SCL(IO23)=1 → 9클럭 복구 반복
 *  (풀업은 정상 장착돼 있음을 사용자가 확인 — 배선 문제가 아니었다.)
 *  → 공유 버스 보드는 H2 와 같은 구성, 즉 **HW I2C + 뮤텍스 공유**가 정답이다. */
#undef  BOARD_OLED_BITBANG
#define BOARD_OLED_BITBANG     0

/* ── 기본 송신 주파수 447.72MHz ────────────────────────────────────────
 *  실기 기본은 447.70. 빌드 시 -Freq 로 덮을 수 있다(BOARD_OVR_FREQ). */
#undef  BOARD_DEFAULT_FREQ_MHZ
#define BOARD_DEFAULT_FREQ_MHZ 447.72f

/* ── OLED 회전 — 이 기판은 **정방향 장착** ──────────────────────────────
 *  실기(xiao-c6.h)는 OLED 가 180° 뒤집혀 장착돼 ROTATE_180=1 이지만,
 *  이 테스트 기판은 정방향이라 상속값을 그대로 두면 화면이 상하 반전된다
 *  (2026-08-17 사용자 확인). → 0 으로 되돌린다.
 *  로터리는 실기와 동일(ROT_HALF_STEP=1 = EC05), A/B 스왑도 없음(기본 0). */
#undef  BOARD_OLED_ROTATE_180
#define BOARD_OLED_ROTATE_180  0

/* ── 배터리/충전 회로 없음 ─────────────────────────────────────────────
 *  ★2026-08-16 가드를 정리해 이제 끌 수 있다.
 *    예전에는 BOARD_HAS_BAT_ADC=0 이 **컴파일조차 안 됐다** — 배터리 로그·평활
 *    기계가 통째로 `#if BOARD_HAS_BAT_ADC` 안인데 호출부는 가드 밖이었다
 *    (기본값이 0인 gnpe-c6 도 같이 깨져 있었다).
 *    → somfy_app.c 에 `#else` 스텁을 넣고, 방전 세션 블록을 가드 안으로 옮겼다.
 *  실측(끄기 전): `[BAT?] raw=590 vadc=596mV -> vbat=1192mV 산포=255카운트`
 *    — 플로팅 핀을 읽어 잔량%가 무의미했다. 이제 관련 코드가 통째로 빠진다. */
#undef  BOARD_HAS_BAT_ADC
#define BOARD_HAS_BAT_ADC      0

/* ── 전원: 항상 USB ────────────────────────────────────────────────────
 *  배터리도 충전 감지 회로도 없다. 이게 없으면 CHG_STAT 플로팅을 읽어
 *  '배터리 구동'으로 오판하고 유휴 10초에 화면이 꺼진다 —
 *  실측(2026-08-16): 부팅 19초 시점에 `절전 모드 진입 — OLED off`,
 *  [PWR] VBUS=0 화면=OFF. OLED 는 정상 검출됐는데 화면만 안 보였다. */
#undef  BOARD_ALWAYS_USB_POWERED
#define BOARD_ALWAYS_USB_POWERED 1
