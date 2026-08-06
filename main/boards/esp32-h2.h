#pragma once
/*
 * boards/esp32-h2.h
 * ──────────────────────────────────────────────────────────
 * 보드: ESP32-H2 SuperMini (저가 호환 보드) — 미검증
 *   • SoC          : ESP32-H2 (RISC-V, single core, 802.15.4 + BLE, WiFi 없음)
 *   • IDF target   : esp32h2
 *   • 핀 출처      : doc/esp32/SuperMini/ (핀맵 esp32-h2-supermini0.jpg,
 *                    esp32_h2_superMini1.jpg + 회로도 esp32-h2-superMini2.png)
 *
 * ★ ESP32-H2 특성:
 *   • 802.15.4(Thread/Zigbee) + BLE → C6 와 동일하게 **Matter over Thread**
 *     (WiFi 미지원이므로 Thread 가 유일 트랜스포트). Thread Border Router 필요.
 *
 * ★ SuperMini 노출 핀 (실측 핀맵):
 *   [좌] GP24(TX) GP23(RX) GP0 GP1 GP2 GP3 GP4 GP5 GP8 GP26 GP27
 *   [우] GP14 GP13 GP12 GP11 GP10 GP9 GP22 GP25
 *   - FSPI 네이티브: FSPICLK=GP4, FSPID=GP5, FSPIQ=GP0, FSPICS0=GP1
 *     → CC1101 SPI 를 여기에 배치(IO-MUX 최적).
 *   - **미노출/금지**: GP6·GP7(미노출), GP8(RGB LED/LOG·strapping),
 *     GP9(BOOT strapping), GP26/GP27(USB D-/D+), GP15~21(in-package flash).
 *
 * ★★ I2C 공유 (이 보드의 핵심):
 *   ESP32-H2 에는 LP_I2C 가 없고 핀도 빠듯하므로, **OLED 와 PCF8574 를
 *   하드웨어 I2C 한 버스에 공유**한다(SDA/SCL 동일 핀, 주소만 다름:
 *   OLED 0x3C / PCF8574 0x20). → BOARD_I2C_SHARED=1.
 *   (GNPE 는 PCF=비트뱅으로 버스 분리. XIAO 는 LP_I2C(6/7) 미연결 시 이 공유 버스로
 *    런타임 자동 폴백(BOARD_I2C_LP_FALLBACK). 각 보드 헤더 참고.)
 *   공유 시 펌웨어는 PCF8574 도 OLED 와 같은 HW I2C 버스로 폴링한다
 *   (button_handler.c 의 BOARD_I2C_SHARED 분기).
 *
 *   첫 빌드 전: ./build.ps1 -Board esp32-h2 -Action set-target 1회 수행.
 * ──────────────────────────────────────────────────────────
 */

#define BOARD_NAME "esp32-h2"
#define BOARD_IDF_TARGET "esp32h2"

/* ════════════════════════════════════════════════════════
   CC1101 SPI  (FSPI IO-MUX 네이티브 핀)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_CC1101_SCK 4  // GP4  FSPICLK
#define BOARD_PIN_CC1101_MISO 0 // GP0  FSPIQ
#define BOARD_PIN_CC1101_MOSI 5 // GP5  FSPID
#define BOARD_PIN_CC1101_CS 1   // GP1  (FSPICS0)
#define BOARD_PIN_CC1101_GD0 10 // GP10 Async TX (RMT)

/* ════════════════════════════════════════════════════════
   I2C — OLED + PCF8574 공유 버스 (HW I2C, SDA/SCL 동일 핀)
     외부 4.7kΩ pull-up 필수(SDA/SCL 각 1). 주소: OLED 0x3C / PCF8574 0x20.
     핀은 고정 아님(GPIO 매트릭스) — 현재 GP13/GP14.
       ※ GP13/GP14 는 32.768kHz xtal 겸용핀. SuperMini 는 외부 32K 크리스털을
         실장하지 않으므로 일반 GPIO 로 사용 OK(외부 32K 를 달 거면 다른 핀으로
   옮길 것). ════════════════════════════════════════════════════════ */
#define BOARD_I2C_SHARED 1
#define BOARD_PIN_OLED_SDA 13 // GP13  ─┐ 공유 SDA
#define BOARD_PIN_OLED_SCL 14 // GP14  ─┤ 공유 SCL
#define BOARD_PIN_PCF_SDA 13  // GP13  ─┘ (OLED 와 동일)
#define BOARD_PIN_PCF_SCL 14  // GP14     (OLED 와 동일)
#define BOARD_PIN_PCF_INT 11  // GP11  PCF8574 ~INT (active-LOW, wake)

/* ── OLED 디스플레이 규격 (외부 모듈 — 실제 구성: 0.96" 128×64) ──
 *   128×64 표준 패널이라 COL_OFFSET=0 / FIXUP=0. sdkconfig.esp32-h2 의
 *   CONFIG_OFFSETX=0 · CONFIG_SSD1306_128x64=y 와 일치.
 *   (72×40 패널로 바꾸려면 WIDTH=72/HEIGHT=40/COL_OFFSET=28/FIXUP=1 +
 *    CONFIG_OFFSETX=28.)                                                */
#define BOARD_OLED_WIDTH 128
#define BOARD_OLED_HEIGHT 64
#define BOARD_OLED_COL_OFFSET 0
#define BOARD_OLED_ROTATE_180 0 // 외부 모듈은 보통 정방향
#define BOARD_OLED_FIXUP_72X40 0
#define BOARD_OLED_ADDR 0x3C

/* 로터리: 실제 구성 EC05 (하프스텝 — 그레이코드 LUT 누산 디코더) */
#define BOARD_ROT_HALF_STEP 1

/* ── Matter 구조 = composed (RAM 절약) ─────────────────────────────────
 *   H2 는 free 가 빠듯하므로 Bridge(Aggregator+bridged_node, 노드당 ~수십KB) 대신
 *   composed(root 직속 WindowCovering)를 쓴다. app_main.cpp 가 이 매크로로 분기.
 *   효과: Bridge 땐 free 2.5KB(블라인드 2개 한계) → composed 로 free 72KB(블라인드 5개).
 *   단 BLE(CHIPoBLE 커미셔닝)까지 켜면 5개는 CHIP PacketBuffer 가 소진돼 커미셔닝(BLE
 *   연결)이 NO_MEMORY(err=b)로 실패한다(SmartThings 39-100). 3개로도 커미셔닝 중
 *   NimBLE mbuf(heap)·CHIP PacketBuffer 가 부족(ble_hs_mbuf_from_flat failed / pool EMPTY,
 *   39-104)해 PASE 가 실패 → 2개로 더 낮춰 heap 을 확보한다. (CHIP PacketBuffer 풀은
 *   main/CHIPProjectConfig.h 에서 24 로, CHIP task 우선순위는 sdkconfig 에서 15 로 올림.
 *   C6 는 RAM 여유로 board_select 기본 5 유지.) */
#define BOARD_MATTER_COMPOSED 1
/* 사용자 요청: 블라인드 3개. heap 확보책(BOARD_DISABLE_TIME 시간제거 + OT/MDNS/EVENT 축소 +
 *  CHIP PacketBuffer 풀 main/CHIPProjectConfig.h + CHIP task priority 15)으로 2개에선 페어링
 *  통과 확인. 3개는 endpoint 가 늘어 heap 이 더 빠듯 — 재커미셔닝(PASE peak) 시 부족하면 2개로. */
#define BLIND_MAX_COUNT 3

/* 시간/날짜 비활성 — BLE 커미셔닝 heap 확보(time 태스크·SNTP 제거). 측정상 BLE 켠
 *  H2 의 free heap 이 ~2.5KB 까지 떨어져 PASE peak(20~30KB)를 못 버티므로, 시간 표시를
 *  희생해 heap 을 회복한다. (시계 화면→블라인드, 화면보호기→시간 없는 형태: oled_ui.c) */
#define BOARD_DISABLE_TIME 1

/* OTA 미지원 — 설정 메뉴에서 FW Update 항목 제거(사용자 요청). */
#define BOARD_DISABLE_OTA 1

/* ════════════════════════════════════════════════════════
   GPIO 직결 센서 (light sleep wake 가능)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_CHG_STAT                                                     \
  12 // GP12 — TP4054 CHRG(녹색 LED 전용, GPIO 미노출)는 못 읽지만, GP12 를
     //   VBUS(5V) 분압(active-HIGH)으로 재사용해 USB 연결 감지(A+B, 아래 매크로).
#define BOARD_PIN_VIBE 2 // GP2 VS1 진동 스위치 (비-strapping 가용핀, light-sleep wake)

/* 배터리/충전 — SuperMini 온보드 충전회로 (스키매틱 doc/esp32/SuperMini/
 *   esp32-h2-superMini2.png 확인):
 *   • 충전 IC : TP4054 (SOT23-5, 단셀 Li-ion 리니어 차저)
 *   • 충전전류: R_PROG=10kΩ → I ≈ 1000/10 = 100 mA
 *   • 파워패스: BAT → Schottky(PD1) → VCC.  LDO: ME6217C33 (3.3V 출력)
 *   • 충전 status(CHRG)는 녹색 LED 전용 → GPIO 로 안 빠짐(XIAO NCHG 와 동일
 * 이슈) ⚠ BATT_MAH 는 BAT 패드에 직결하는 실제 셀 용량으로 맞출 것(아래는
 * 대표값). 완충시간 = (BATT_MAH/CHG_MA) h × 1.25. 예: 400mAh → 4.0h×1.25
 * = 5.0h. */
#define BOARD_BATT_MAH                                                         \
  400                    // 대표값 — 실제 셀로 조정(100mA 충전이라 소형 셀 적합)
#define BOARD_CHG_MA 100 // TP4054, R_PROG=10kΩ → ~100mA

/* 충전 감지(A+B) — VBUS 분압(active-HIGH) + BAT 분압 ADC(실측 %) */
#define BOARD_CHG_STAT_ACTIVE_HIGH 1 // CHG_STAT(GP12) ← VBUS(5V) 100k/150k 분압(~3.0V)
#define BOARD_HAS_BAT_ADC 1
/* ★ 현 기판은 분압이 오배선: BAT_ADC핀(GP3)=VBUS, CHG_STAT핀(GP12)=BAT.
 *   GP12 가 ADC 불가핀이라 잔량 % 측정 불가 → "USB/BAT/LOW" 상태 표시로 동작.
 *   기판을 정상 배선(GP3=BAT, GP12=VBUS)으로 고치면 0(또는 삭제) → 충전률 % 복귀. */
#define BOARD_BAT_SWAPPED 1
#define BOARD_PIN_BAT_ADC 3   // GP3 (ADC1) ← BAT 100k/100k 분압
#define BOARD_BAT_DIV_TOP 100 // 100k/100k → Vbat = Vadc×2
#define BOARD_BAT_DIV_BOT 100
