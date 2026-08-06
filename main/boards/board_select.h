#pragma once
/*
 * boards/board_select.h
 * ──────────────────────────────────────────────────────────
 * 빌드 시점에 -DBOARD_<NAME>=1 매크로로 보드별 핀맵 선택.
 *
 * 사용 (브랜드-SoC 단위 키):
 *   build.ps1 -Board gnpe-c6  → -DBOARD_GNPE_C6=1  (GNPE ESP32-C6-0.42, 검증)
 *   build.ps1 -Board xiao-c6  → -DBOARD_XIAO_C6=1  (Seeed XIAO ESP32-C6)
 *   build.ps1 -Board esp32-h2 → -DBOARD_ESP32_H2=1
 *   (구 -Board esp32-c6 = gnpe-c6 별칭 — 역호환)
 *
 * ★ 같은 SoC(esp32c6)라도 브랜드(GNPE/XIAO)마다 핀 배열이 다르므로 board 파일
 *   을 브랜드별로 분리한다. OTA 구분도 chip-ID 가 같아(esp32c6) 안 되므로
 *   브랜드별 Product ID 로만 구분 — 각 board 의 PID 가 달라야 한다.
 *
 * 새 보드 추가:
 *   1. boards/<brand>-<soc>.h 작성 (BOARD_PIN_* 모두 정의)
 *   2. 아래 분기에 #elif 추가
 *   3. build.ps1 의 $BoardMap (idf target/sdkconfig/PID) 갱신
 *   4. (선택) sdkconfig.defaults.<name> 추가 (브랜드별 PID 등)
 * ──────────────────────────────────────────────────────────
 */

#if defined(BOARD_GNPE_C6) || defined(BOARD_ESP32_C6)
   /* GNPE ESP32-C6-0.42 (현 검증 보드). BOARD_ESP32_C6 은 구 키 backward 별칭. */
#  include "boards/gnpe-c6.h"
#elif defined(BOARD_XIAO_C6)
#  include "boards/xiao-c6.h"
#elif defined(BOARD_ESP32_H2)
#  include "boards/esp32-h2.h"
#else
#  error "No board selected. Build with -DBOARD_ESP32_C6=1 (or another supported board). \
See main/boards/ for available boards."
#endif

/* ── OLED 규격 기본값 (보드가 BOARD_OLED_* 를 안 주면 GNPE 0.42" 72×40 가정) ──
 *   보드별 디스플레이가 다르므로 boards/<board>.h 에서 정의하는 게 원칙.
 *   여기 fallback 은 구 보드 파일 호환용 안전망. */
#ifndef BOARD_OLED_WIDTH
#  define BOARD_OLED_WIDTH       72
#endif
#ifndef BOARD_OLED_HEIGHT
#  define BOARD_OLED_HEIGHT      40
#endif
#ifndef BOARD_OLED_COL_OFFSET
#  define BOARD_OLED_COL_OFFSET  28
#endif
#ifndef BOARD_OLED_ROTATE_180
#  define BOARD_OLED_ROTATE_180   1
#endif
#ifndef BOARD_OLED_FIXUP_72X40
#  define BOARD_OLED_FIXUP_72X40  1
#endif
#ifndef BOARD_OLED_FLIP_X
#  define BOARD_OLED_FLIP_X       0   /* 좌우(가로) 반전 — 기본 꺼짐(역호환) */
#endif
#ifndef BOARD_OLED_ROTATE_90
#  define BOARD_OLED_ROTATE_90    0   /* 0 / 90(시계) / 270(반시계) — 기본 회전 없음 */
#endif
#ifndef BOARD_OLED_ADDR
#  define BOARD_OLED_ADDR        0x3C
#endif

/* ── OLED 렌더러는 "해상도"로 자동 선택된다 (보드 무관) ───────────────────
 *   oled_ui.c 는 위 BOARD_OLED_WIDTH×HEIGHT 값만 보고 렌더러를 고른다
 *   (oled_ui.h 의 OLED_RENDER_* 매크로). 즉 디스플레이는 보드마다 자유롭게
 *   교체 가능하며, 보드는 패널 규격(이 6개 매크로)만 선언하면 된다.
 *
 *     128×64 (가로)  → 풀스크린 네이티브 렌더러 (고딕 6×9 + 7세그, 가로 배치)
 *     64×128 (세로)  → 풀스크린 네이티브 렌더러 (고딕 6×9 + 7세그, 세로 적층)
 *     그 외          → 72×40 논리 캔버스 렌더러 (어떤 패널에도 중앙 배치/블록 스케일)
 *
 *   예1) GNPE 에 0.96" 128×64 패널 → gnpe-c6.h 의 BOARD_OLED_* 를
 *        128×64/COL_OFFSET=0/FIXUP=0 으로 (코드 수정 불필요, CONFIG_OFFSETX=0).
 *   예2) 어느 보드든 64×128 세로 패널(SH1107 등) → BOARD_OLED_* 를
 *        64×128 으로 선언하면 세로 레이아웃 렌더러가 자동 적용된다. */

/* ── I2C 버스 공유 플래그 ──────────────────────────────────────────────
 *   0 (기본): OLED=하드웨어 I2C, PCF8574=소프트웨어 비트뱅 (서로 다른 핀).
 *   1       : OLED 와 PCF8574 가 하드웨어 I2C 한 버스를 공유(SDA/SCL 동일 핀).
 *             LP_I2C 가 없거나 핀이 부족한 보드(ESP32-H2 등)에서 사용.
 *             이 경우 BOARD_PIN_OLED_SDA==BOARD_PIN_PCF_SDA,
 *                      BOARD_PIN_OLED_SCL==BOARD_PIN_PCF_SCL 이어야 한다. */
#ifndef BOARD_I2C_SHARED
#  define BOARD_I2C_SHARED       0
#endif

#if BOARD_I2C_SHARED
#  if (BOARD_PIN_OLED_SDA != BOARD_PIN_PCF_SDA) || (BOARD_PIN_OLED_SCL != BOARD_PIN_PCF_SCL)
#    error "BOARD_I2C_SHARED=1 인데 OLED 와 PCF8574 의 SDA/SCL 핀이 다릅니다 (같은 버스여야 함)."
#  endif
#endif

/* ── LP_I2C → 공유 HW I2C 런타임 자동 폴백 (XIAO 등) ──────────────────────
 *   0 (기본): 폴백 없음 — 위 BOARD_I2C_SHARED 단일 경로만 사용.
 *   1       : 부팅 시 LP_I2C 전용핀(BOARD_PIN_PCF_LP_SDA/SCL) 비트뱅으로 PCF8574
 *             프로브 → 응답 없으면 공유 HW I2C(OLED 버스)로 자동 전환.
 *             → 같은 펌웨어가 "뒷면 LP_I2C 배선" / "앞면 공유 I2C 배선" 모두 지원.
 *   조건: BOARD_I2C_SHARED=1(폴백 대상) + BOARD_PIN_PCF_LP_SDA/SCL 정의 필수. */
#ifndef BOARD_I2C_LP_FALLBACK
#  define BOARD_I2C_LP_FALLBACK  0
#endif
#if BOARD_I2C_LP_FALLBACK
#  if !BOARD_I2C_SHARED || !defined(BOARD_PIN_PCF_LP_SDA) || !defined(BOARD_PIN_PCF_LP_SCL)
#    error "BOARD_I2C_LP_FALLBACK=1 requires BOARD_I2C_SHARED=1 and BOARD_PIN_PCF_LP_SDA/SCL."
#  endif
#endif

/* ── 배터리/충전 사양 기본값 ──────────────────────────────────────────
 *   충전량 시간추정(somfy_app.c::_estimate_battery_percent)의 완충시간을
 *   보드별 (용량 ÷ 충전전류)로 파생하기 위한 값. 보드가 안 주면 GNPE 기준
 *   600 mAh / 300 mA(= 완충 2.5h) 로 폴백 → 기존 동작 그대로 유지(역호환). */
#ifndef BOARD_BATT_MAH
#  define BOARD_BATT_MAH         600   /* 단셀 LiPo 용량(mAh) */
#endif
#ifndef BOARD_CHG_MA
#  define BOARD_CHG_MA           300   /* 충전 IC 정전류(mA) */
#endif

/* ── 충전 감지 극성 / 배터리 ADC 기본값 ───────────────────────────────
 *   BOARD_CHG_STAT_ACTIVE_HIGH : 0 = active-LOW(충전 IC STAT 직결, GNPE)
 *                                1 = active-HIGH(VBUS 분압 → "USB 연결" 감지)
 *   BOARD_HAS_BAT_ADC=1 이면 BAT 분압을 ADC 로 읽어 실측 % 사용
 *     (BOARD_PIN_BAT_ADC = ADC 핀. Vbat = Vadc×(DIV_TOP+DIV_BOT)/DIV_BOT). */
#ifndef BOARD_CHG_STAT_ACTIVE_HIGH
#  define BOARD_CHG_STAT_ACTIVE_HIGH  0
#endif
/*   BOARD_CHG_STAT_EXT_PULLDOWN : 1 = VBUS 분압의 하단 저항이 이미 풀다운 역할을
 *     하므로 CHG_STAT 에 **내부 풀다운을 켜지 말 것**. 내부 풀다운(~45kΩ)은 분압
 *     (100k/150k, 출력임피던스 60kΩ)을 3.0V→1.29V 로 끌어내려 USB 를 LOW 로 오독
 *     시킨다 → _is_usb_powered() 항상 false → (a) USB 인데 배터리 모드로 1분 유휴
 *     절전(OLED off), (b) 배터리 미연결 판정(_nobat_track)이 영영 성립 못 함.
 *     0 = 기존 동작(내부 풀다운 ON) 유지. */
#ifndef BOARD_CHG_STAT_EXT_PULLDOWN
#  define BOARD_CHG_STAT_EXT_PULLDOWN 0
#endif
#ifndef BOARD_HAS_BAT_ADC
#  define BOARD_HAS_BAT_ADC           0
#endif
#ifndef BOARD_PIN_BAT_ADC
#  define BOARD_PIN_BAT_ADC          (-1)
#endif
#ifndef BOARD_BAT_DIV_TOP
#  define BOARD_BAT_DIV_TOP          100  /* 분압 상단 저항(kΩ) */
#endif
#ifndef BOARD_BAT_DIV_BOT
#  define BOARD_BAT_DIV_BOT          100  /* 분압 하단 저항(kΩ) */
#endif

/* ── 배터리/USB 분압 핀 스왑 (기판 리비전) ────────────────────────────────
 *   0 (기본·정상 배선): BAT_ADC 핀=BAT 분압(ADC 로 잔량 % 실측), CHG_STAT 핀=VBUS 분압.
 *                       → 기존 _estimate_battery_percent/chg_percent 충전률 % 표시.
 *   1 (현 기판 오배선): BAT_ADC 핀=VBUS 분압, CHG_STAT 핀=BAT 분압.
 *                       CHG_STAT 핀이 ADC 불가라 잔량 % 측정 불가 → BAT_ADC(ADC)로 USB
 *                       감지 + CHG_STAT 풀업 1임계로 저전압만 판별("USB/BAT/LOW" 상태).
 *   ※ 두 경로 모두 코드에 보존(somfy_app·oled_ui 의 #if 분기). 기판을 정상 배선으로
 *     고치면 보드 헤더에서 0 으로(또는 줄 삭제) → 충전률 % 로직으로 자동 복귀. */
#ifndef BOARD_BAT_SWAPPED
#  define BOARD_BAT_SWAPPED          0
#endif

/* ── 기본 송신 주파수 (register 값, MHz) ───────────────────────────────
 *   CC1101 register 에 쓰는 목표 주파수(=OLED·Freq Edit 표시값). 크리스털 오차로
 *   실제 on-air 는 다르며, RF 모듈마다 오차가 달라 권장 register 값이 다르다:
 *     · 테스트 보드(E07-M1101D-SMA): 447.72 → on-air 447.675
 *     · 제작 보드(E07-400MM10S)    : 447.70 → on-air 447.673
 *   기본 447.70(제작 보드). build.ps1 -Freq 447.72|447.70 로 빌드 시 변경(BOARD_OVR_FREQ). */
#ifndef BOARD_DEFAULT_FREQ_MHZ
#  define BOARD_DEFAULT_FREQ_MHZ     447.70f
#endif

/* ── 좌/우 버튼 확장 — PCF8574(8핀) ↔ PCF8575(16핀) 분기 ───────────────
 *   0 (기본): PCF8574 (8핀, P0~P7) — 좌/우 버튼 없음 (기존 동작).
 *   1       : PCF8575 (16핀) — P10=LEFT, P11=RIGHT 추가. read/write 2바이트.
 *             주소(0x20)·~INT·I2C 배선 동일, 칩만 16비트 형제로 교체. */
#ifndef BOARD_HAS_LR_BUTTONS
#  define BOARD_HAS_LR_BUTTONS       0
#endif

/* ── 로터리 엔코더 디텐트 타입 ─────────────────────────────────────────
 *   0 (기본): full-step (EC11 등) — 1디텐트=1 그레이사이클, rest@11 만.
 *             기존 "rest 이탈 첫엣지" 디코더 사용.
 *   1       : half-step (EC05 등) — 1디텐트=2 그레이스텝, rest@11·00 양쪽.
 *             그레이코드 LUT 누산 디코더(바운스 상쇄)로 양방향 대칭 처리. */
#ifndef BOARD_ROT_HALF_STEP
#  define BOARD_ROT_HALF_STEP        0
#endif

/* ── 블라인드 최대 개수 (= Matter endpoint 수에 직결, RAM 소비 좌우) ───────────
 *   보드 프로파일(boards/<board>.h)에서 정의 가능. 미정의 시 기본 8(C6 = HP SRAM
 *   512KB · free ~92KB 실측 여유). H2 는 RAM 한계로 esp32-h2.h 에서 3 으로 낮춤
 *   (시간/날짜·OTA 제거분으로 확보; 커미셔닝 PASE peak 까지 고려한 안정 상한은 2). */
#ifndef BLIND_MAX_COUNT
#  define BLIND_MAX_COUNT            8
#endif

/* ── Matter 다중 블라인드 노출 구조 ─────────────────────────────────────
 *   0 (기본): Bridge — Aggregator + bridged_node. 각 블라인드를 *개별 기기 타일*로 노출
 *             (SmartThings/Apple/Google). bridged endpoint RAM 이 노드당 수십 KB.
 *   1       : composed — root 직속 WindowCovering endpoint(단일 기기 + 다중 ep). RAM 대폭
 *             절약. free 빠듯한 보드(H2 등)용. 보드 헤더에서 1 로 설정. */
#ifndef BOARD_MATTER_COMPOSED
#  define BOARD_MATTER_COMPOSED      0
#endif

/* ── 시간/날짜 표시 (RTC 폴링 + SNTP + 시계 렌더) ─────────────────────────
 *   0 (기본): 활성 — 시계 화면 + 화면보호기 시계 + SNTP 동기.
 *   1       : 비활성 — time_update/time_persist 태스크(스택 ~5KB)·SNTP 를 만들지
 *             않아 heap 확보. H2 처럼 BLE 커미셔닝 heap 이 빠듯한 보드용
 *             (시계 화면 → 블라인드 표시, 화면보호기 → 시간 없는 형태로 oled_ui.c 분기). */
#ifndef BOARD_DISABLE_TIME
#  define BOARD_DISABLE_TIME         0
#endif

/* OTA(Matter OTA over Thread) 비활성 — H2 처럼 OTA 미지원 보드용.
 *  설정 메뉴에서 FW Update 항목을 컴파일 제외한다. */
#ifndef BOARD_DISABLE_OTA
#  define BOARD_DISABLE_OTA          0
#endif

/* ── 빌드타임 변형 오버라이드 (build.ps1 -Pcf/-Rotary/-Oled/-Rotate → CMake -D) ──
 *  보드 헤더를 안 고치고도 한 보드의 PCF/로터리/OLED 변형을 빌드 시 골라 만든다.
 *  main/CMakeLists.txt 가 -D BOARD_OVR_* 컴파일 정의로 주입 → 여기서 보드 기본값을 덮는다.
 *  오버라이드를 안 주면(기본) 아래 블록은 전부 비활성 → 보드 기본값 그대로(바이너리 동일).
 *  ※ OLED 해상도 override 는 렌더러·패널크기(ssd1306_init)를 바꾸지만, 물리 컬럼
 *    오프셋은 SSD1306 라이브러리 Kconfig(CONFIG_OFFSETX, sdkconfig)라 같이 맞춰야
 *    한다(72x40→28 / 128x64·64x128→0). build.ps1 가 경고를 출력한다. */
#ifdef BOARD_OVR_HAS_LR_BUTTONS
#  undef  BOARD_HAS_LR_BUTTONS
#  define BOARD_HAS_LR_BUTTONS    BOARD_OVR_HAS_LR_BUTTONS
#endif
#ifdef BOARD_OVR_ROT_HALF_STEP
#  undef  BOARD_ROT_HALF_STEP
#  define BOARD_ROT_HALF_STEP     BOARD_OVR_ROT_HALF_STEP
#endif
#ifdef BOARD_OVR_OLED_WIDTH
#  undef  BOARD_OLED_WIDTH
#  define BOARD_OLED_WIDTH        BOARD_OVR_OLED_WIDTH
#endif
#ifdef BOARD_OVR_OLED_HEIGHT
#  undef  BOARD_OLED_HEIGHT
#  define BOARD_OLED_HEIGHT       BOARD_OVR_OLED_HEIGHT
#endif
#ifdef BOARD_OVR_OLED_COL_OFFSET
#  undef  BOARD_OLED_COL_OFFSET
#  define BOARD_OLED_COL_OFFSET   BOARD_OVR_OLED_COL_OFFSET
#endif
#ifdef BOARD_OVR_OLED_FIXUP
#  undef  BOARD_OLED_FIXUP_72X40
#  define BOARD_OLED_FIXUP_72X40  BOARD_OVR_OLED_FIXUP
#endif
#ifdef BOARD_OVR_OLED_ROTATE_180
#  undef  BOARD_OLED_ROTATE_180
#  define BOARD_OLED_ROTATE_180   BOARD_OVR_OLED_ROTATE_180
#endif
#ifdef BOARD_OVR_OLED_FLIP_X
#  undef  BOARD_OLED_FLIP_X
#  define BOARD_OLED_FLIP_X       BOARD_OVR_OLED_FLIP_X
#endif
#ifdef BOARD_OVR_OLED_ROTATE_90
#  undef  BOARD_OLED_ROTATE_90
#  define BOARD_OLED_ROTATE_90    BOARD_OVR_OLED_ROTATE_90
#endif
#ifdef BOARD_OVR_FREQ
#  undef  BOARD_DEFAULT_FREQ_MHZ
#  define BOARD_DEFAULT_FREQ_MHZ  BOARD_OVR_FREQ
#endif

/* ── OTA 하드웨어 변형(variant) 식별자 ─────────────────────────────────
 *  같은 보드(=같은 Matter Product ID)라도 주변장치 빌드 변형이 다르면 펌웨어가
 *  호환되지 않는다:
 *    · PCF8574(8핀) ↔ PCF8575(16핀)      — 버튼 read/write 폭(1↔2 byte)이 다름
 *    · EC11(full-step) ↔ EC05(half-step) — 로터리 디코더가 다름
 *    · OLED 해상도+회전(72x40r180·128x64r0 등) — 렌더러·표시 방향이 다름
 *      (BOARD_OLED_WIDTH×HEIGHT + BOARD_OLED_ROTATE_180)
 *  Matter OTA 는 VID/PID + SoftwareVersion(숫자)만 매칭하므로 이 변형을 스스로
 *  구분하지 못한다. → 변형을 태그로 인코딩해 펌웨어 식별자(SoftwareVersionString ·
 *  .ota 파일명)에 실어 교차 설치를 막는다.
 *
 *    BOARD_HW_VARIANT_CODE : PCF·EC 2비트 — 0=8574.ec11(기본) · 1=8575.ec11 ·
 *                            2=8574.ec05 · 3=8575.ec05
 *    BOARD_HW_OLED_STR     : "<W>x<H>r<0|180>" 해상도+회전 토큰 (예 "128x64r0")
 *    BOARD_HW_VARIANT_STR  : "<pcf>.<enc>.<oled>" 전체 태그 (예 "8574.ec05.128x64r0")
 *
 *  ※ 자동감지/자동롤백 안 함: EC11/EC05 는 기계적 디텐트만 달라 전기적 구분 불가,
 *    PCF8574/8575 는 0x20 공유, OLED 도 신뢰성 있는 자동감지가 어렵다 → "이미지
 *    식별자 + 파일명 + 운영자 확인"이 구분 수단. build.ps1 ota-image 가 자동 반영. */
#define BOARD_HW_VARIANT_CODE  (((BOARD_ROT_HALF_STEP) ? 2 : 0) | ((BOARD_HAS_LR_BUTTONS) ? 1 : 0))
#if BOARD_HAS_LR_BUTTONS
#  define BOARD_HW_PCF_STR  "8575"
#else
#  define BOARD_HW_PCF_STR  "8574"
#endif
#if BOARD_ROT_HALF_STEP
#  define BOARD_HW_ROT_STR  "ec05"
#else
#  define BOARD_HW_ROT_STR  "ec11"
#endif
/* OLED 해상도+회전 토큰 "<W>x<H>r<0|180>" — 렌더러·표시 방향을 결정하는 축.
   BOARD_OLED_WIDTH/HEIGHT/ROTATE_180(보드 헤더 또는 위 fallback)을 문자열화. */
#define BOARD_HW_STRINGIFY2(x) #x
#define BOARD_HW_STRINGIFY(x)  BOARD_HW_STRINGIFY2(x)
#if (BOARD_OLED_ROTATE_90 == 90)
#  define BOARD_HW_OROT_STR  "r90"       /* 시계 90° */
#elif (BOARD_OLED_ROTATE_90 == 270)
#  define BOARD_HW_OROT_STR  "r270"      /* 반시계 90° */
#elif BOARD_OLED_ROTATE_180
#  if BOARD_OLED_FLIP_X
#    define BOARD_HW_OROT_STR  "r180m"   /* 180° + 좌우반전(=상하반전) */
#  else
#    define BOARD_HW_OROT_STR  "r180"
#  endif
#else
#  if BOARD_OLED_FLIP_X
#    define BOARD_HW_OROT_STR  "r0m"     /* 좌우(가로) 반전 */
#  else
#    define BOARD_HW_OROT_STR  "r0"
#  endif
#endif
#define BOARD_HW_OLED_STR  BOARD_HW_STRINGIFY(BOARD_OLED_WIDTH) "x" BOARD_HW_STRINGIFY(BOARD_OLED_HEIGHT) BOARD_HW_OROT_STR
#define BOARD_HW_VARIANT_STR  BOARD_HW_PCF_STR "." BOARD_HW_ROT_STR "." BOARD_HW_OLED_STR

/* 필수 매크로 누락 시 컴파일 단계에서 catch */
#if !defined(BOARD_PIN_CC1101_SCK)  || !defined(BOARD_PIN_CC1101_MISO) || \
    !defined(BOARD_PIN_CC1101_MOSI) || !defined(BOARD_PIN_CC1101_CS)   || \
    !defined(BOARD_PIN_CC1101_GD0)  || \
    !defined(BOARD_PIN_OLED_SDA)    || !defined(BOARD_PIN_OLED_SCL)    || \
    !defined(BOARD_PIN_PCF_SDA)     || !defined(BOARD_PIN_PCF_SCL)     || \
    !defined(BOARD_PIN_PCF_INT)     || \
    !defined(BOARD_PIN_CHG_STAT)    || !defined(BOARD_PIN_VIBE)
#  error "Selected board is missing required BOARD_PIN_* macros (see boards/esp32-c6.h template)."
#endif
