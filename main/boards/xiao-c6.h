#pragma once
/*
 * boards/xiao-c6.h
 * ──────────────────────────────────────────────────────────
 * 보드: Seeed Studio XIAO ESP32-C6
 *   • 브랜드/제품  : Seeed "XIAO ESP32-C6"
 *   • SoC          : ESP32-C6 (RISC-V, 802.15.4 + BLE) — GNPE 와 동일 SoC
 *   • IDF target   : esp32c6
 *   • OLED         : 외부 모듈 (XIAO 에 내장 OLED 없음)
 *   • Matter PID   : 0x8003 (sdkconfig.defaults.xiao_c6 — GNPE 0x8000 과 구분)
 *
 * ★ 핀맵 출처: doc/esp32/XIAO/XIAO_ESP32C6_Pinout.xlsx (Seeed 공식, "QFN32 Pin
 *   Summary") + Xiao esp32-c6.png(앞/뒷면) + XIAO_ESP32_C6_v1.0_SCH_260114.pdf.
 *
 * XIAO ESP32-C6 실제 노출 핀 — 앞면 11 castellated edge + 일부 뒷면 패드:
 *  [앞면 edge] D0=GPIO0(A0,LP)   D1=GPIO1(A1,LP)   D2=GPIO2(A2,FSPIQ,LP)
 *             D3=GPIO21         D4=GPIO22(SDA)    D5=GPIO23(SCL)
 *             D6=GPIO16(TX)     D7=GPIO17(RX)     D8=GPIO19(SCK)
 *             D9=GPIO20(MISO)   D10=GPIO18(MOSI)
 *  [뒷면 패드] MTCK=GPIO6 (A6, LP_GPIO6, **LP_I2C_SDA**, FSPICLK)  ← 하드웨어 LP_I2C
 *             MTDO=GPIO7 (    LP_GPIO7, **LP_I2C_SCL**, FSPID)    ← 하드웨어 LP_I2C
 *
 * ★ 미노출 핀(보드 실물 확인):
 *   • **D11=GPIO3(A3) 은 브레이크아웃되지 않음** — 사용 불가. (데이터시트상 칩
 *     핀에는 존재하나 XIAO 보드가 패드로 빼지 않음.) → VIBE 를 GPIO3→GPIO0(D0) 로 이동.
 *   • GPIO8 도 미노출. MTMS=GPIO4·MTDI=GPIO5 뒷면 노출 여부는 미확인 → 사용하지 않음.
 *   • 따라서 확정 가용핀 = 앞면 D0~D10(11) + 뒷면 MTCK/MTDO(2) = 13. (프로젝트 12핀)
 *
 * ★ MTCK=GPIO6=LP_I2C_SDA, MTDO=GPIO7=LP_I2C_SCL 는 ESP32-C6 하드웨어 LP_I2C
 *   전용핀이다 → PCF8574 I2C 를 이 핀쌍에 배치(앞면 D0 는 VIBE, D1 은 spare).
 *   (JTAG(MTxx) 점유 → JTAG 디버그는 불가.)
 *
 * ★★ 사용 금지/주의 핀:
 *   • GPIO14 = 온보드 **RF 스위치(ANT1/ANT2 안테나 선택)** — 절대 사용 금지.
 *     (ESP32-C6 자체 라디오 안테나 선택용; Thread/BLE RF 에 영향, CC1101 무관.)
 *   • GPIO15 = USER LED,  GPIO9 = BOOT(strapping),  GPIO12/13 = USB D-/D+.
 *   • GPIO3(D11)·GPIO8 = 미노출.
 *   • ★ Seeed 스키매틱 명시 경고: "Avoid using GPIO4, 5, 8, 9, 15".
 *     → MTMS=GPIO4·MTDI=GPIO5(VDDA3P3 인접) 는 사용하지 말 것.
 *       (단 MTCK=GPIO6·MTDO=GPIO7=LP_I2C 는 회피목록에 없음 → 사용 OK.)
 *   • CC1101 SPI 는 IO-MUX 네이티브(FSPICLK/D=GPIO6/7) 대신 D8/D9/D10(GPIO-matrix)
 *     사용 — GPIO6/7 은 PCF8574 LP_I2C 에 양보. CC1101 은 저속이라 matrix 로 충분.
 *
 * ★ 배터리/충전 (XIAO 온보드 완비 — 외부 충전회로 불필요. 공식 스키매틱 확인):
 *   • 충전 IC: SGM40567-4.2 (단셀 LiPo, 4.2V 컷오프), **충전전류 120 mA**
 *     (R10=200K, ICharge=24000/200K). → BAT+/BAT- 패드에 PCM 단셀 LiPo 직결만 하면 됨.
 *   • 파워패스 자동: USB 연결=USB 로 동작+충전 / USB 분리=배터리로 동작
 *     (Q1 P-MOS + D1 Schottky). DC-DC: SGM6029C(5V→3.3V).
 *   • → GNPE 의 MCP73831/SS14/충전저항/충전 LED 전부 불필요(이미 XIAO 에 있음).
 *   • ⚠ 충전 status(NCHG)는 온보드 빨강 LED(CHG1) 전용 — **castellated/TP 어디에도
 *     안 나옴**. 즉 충전상태 GPIO 읽기는 불가 → 아래 CHG_STAT 는 그대로는 못 씀.
 *   • USB 연결 감지가 필요하면 노출된 **VBUS 패드(5V)** 를 분압(예 100K/100K)해
 *     GPIO 로 읽는다(극성: GNPE STAT=active-LOW 와 반대인 **active-HIGH**). 배터리
 *     전압은 BAT 패드를 분압해 ADC 로. 둘 다 선택사항(없어도 RTS 동작엔 무관).
 *
 * 실제 배선이 아래와 다르면 BOARD_PIN_* 를 조정할 것. PCF8574 를 뒷면 LP_I2C
 * (MTCK/MTDO) 로 옮기고 VIBE 를 D0(GPIO0) 로 옮겨, 앞면 **D1(GPIO1)** 만 spare 다.
 * 안테나(GPIO14)·LED(GPIO15)·USB(GPIO12/13)·D11(GPIO3,미노출)은 건드리지 말 것.
 * 첫 빌드 전: ./build.ps1 -Board xiao-c6 -Action set-target 1회.
 * ──────────────────────────────────────────────────────────
 */

#define BOARD_NAME            "xiao-c6"
#define BOARD_IDF_TARGET      "esp32c6"

/* ════════════════════════════════════════════════════════
   CC1101 SPI  (XIAO 기본 SPI 패드 D8/D9/D10, GPIO-matrix 라우팅)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_CC1101_SCK   19  // D8  (GPIO19) SPI SCK
#define BOARD_PIN_CC1101_MISO  20  // D9  (GPIO20) SPI MISO
#define BOARD_PIN_CC1101_MOSI  18  // D10 (GPIO18) SPI MOSI
#define BOARD_PIN_CC1101_CS    21  // D3  (GPIO21)
#define BOARD_PIN_CC1101_GD0   16  // D6  (GPIO16) Async TX (RMT)

/* ════════════════════════════════════════════════════════
   OLED I2C  (외부 모듈; XIAO 기본 I2C 패드 D4=SDA D5=SCL)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_OLED_SDA     22  // D4 (GPIO22) SDA
#define BOARD_PIN_OLED_SCL     23  // D5 (GPIO23) SCL

/* ── OLED 디스플레이 규격 — XIAO 외부 0.96" SSD1306 128×64, 정방향 ──
 *   • 표준 128×64 패널 → COL_OFFSET=0 (sdkconfig.defaults.xiao_c6 의
 *     CONFIG_OFFSETX=0 과 일치). 72×40 보정 시퀀스 불필요.
 *   • 정방향 장착 → 회전 없음.
 *   • 해상도 기준 렌더러 자동 선택 → 128×64 풀스크린 네이티브(고딕+7세그).
 *     (oled_ui.h 의 OLED_RENDER_128X64. 다른 규격 패널로 바꾸면 그 해상도
 *      렌더러가 자동 적용 — 예: 64×128 세로 패널 → OLED_RENDER_64X128.)     */
#define BOARD_OLED_WIDTH       128
#define BOARD_OLED_HEIGHT       64
#define BOARD_OLED_COL_OFFSET    0   // 표준 128×64 → CONFIG_OFFSETX=0
/* ★★★2026-08-16 0 → 1. **기판 실물이 180° 장착이다.**
 *  doc/wiring/wiring_xiao-c6.md:12 — "현 시제품 기판은 OLED 가 180° 장착이라
 *  `-Rotate 180` 필수(flash·ota-image 도 동일)".
 *  여기가 0 이면 `-Rotate 180` 을 매번 손으로 붙여야 하고, 한 번만 빠뜨려도
 *  화면이 상하 뒤집힌다(실제로 그렇게 플래시해 사용자를 화나게 했다).
 *  → 기본값을 기판에 맞춘다. 정방향 패널로 바꾸면 `-Rotate 0` 으로 덮으면 된다. */
#define BOARD_OLED_ROTATE_180    1   // 기판이 180° 장착 (wiring_xiao-c6.md)
#define BOARD_OLED_FIXUP_72X40   0   // 표준 SSD1306 → 보정 불필요
#define BOARD_OLED_ADDR         0x3C

/* ════════════════════════════════════════════════════════
   PCF8574 I2C — 버튼/로터리/SETUP.  ※ XIAO 는 두 배선을 런타임 자동 지원(LP_I2C 우선→공유 폴백):

   [옵션 A — 현재 설정·검증됨] OLED 와 HW I2C 버스 공유 (앞면 D4/D5 = GPIO22/23)
     · H2 와 동일 방식. 앞면 표준 I2C 패드라 납땜 쉬움(뒷면 작업 불요).
     · BOARD_I2C_SHARED=1 → 비트뱅 대신 OLED 가 만든 i2c_master 버스에 device 로
       붙어 폴링(주소 OLED 0x3C / PCF 0x20). 외부 4.7kΩ pull-up 은 SDA/SCL 각
       1개씩(OLED·PCF 공통)이면 됨.

   [옵션 B — 자동 폴백] 뒷면 LP_I2C 전용핀 (MTCK=GPIO6 / MTDO=GPIO7) 비트뱅
     · BOARD_I2C_LP_FALLBACK=1 → 부팅 시 LP_I2C(6/7)에서 PCF8574 를 먼저 프로브하고,
       응답이 없으면 [A] 공유 HW I2C(22/23)로 런타임 자동 전환한다.
       → 한 펌웨어가 "뒷면 LP_I2C 배선" 과 "앞면 공유 I2C 배선" 을 모두 지원(택1 불필요).
     · LP_I2C 로 배선 시 그 버스에 외부 4.7kΩ 풀업 1쌍 필요. (MTCK/MTDO=JTAG → JTAG 디버그 불가)

     wake 가 필요한 ~INT 는 두 배선 모두 LP_GPIO(D2=GPIO2) 에 둔다.
   ════════════════════════════════════════════════════════ */
/* ★★★2026-08-16 이 기판은 **PCF8575**(16비트) 다 — 좌/우 버튼이 P10/P11(bit8/9).
 *  여기에 정의가 없으면 board_select.h 기본값 0 → PCF_NBYTES=1(PCF8574) 이 되어
 *  (a) 좌/우 버튼이 통째로 사라지고
 *  (b) LP 코어 프로그램은 LP_PCF_NBYTES 2 고정이라 button_handler.c:148 의
 *      _Static_assert 가 "LP/HP PCF read width mismatch" 로 **빌드를 깬다**.
 *  그래서 지금까지 `-Pcf 8575` 를 매번 손으로 붙여야 했다(빠뜨려 빌드 실패함).
 *  → 기본값을 기판에 맞춘다. PCF8574 개체는 `-Pcf 8574` 로 덮으면 된다.
 *  ※dist/ 의 옛 파일명 `..._8574_...` 와 doc/OTA.md 예시는 실기와 다르다. */
#define BOARD_HAS_LR_BUTTONS   1   // PCF8575 (좌/우 버튼 P10/P11)

#define BOARD_I2C_SHARED       1   // 공유 HW I2C(22/23) — LP_I2C 미연결 시 폴백 대상
#define BOARD_PIN_PCF_SDA      22  // D4(GPIO22)=OLED SDA 공유 (공유 모드)
#define BOARD_PIN_PCF_SCL      23  // D5(GPIO23)=OLED SCL 공유 (공유 모드)
#define BOARD_I2C_LP_FALLBACK  1   // LP_I2C(아래 핀) 우선 프로브 → 무응답 시 공유로 자동 전환
/* ★2026-07-23: 이 보드의 HW I2C0 가 "bus busy" 로 고착돼 화면이 멈추는 현상이 실측 재현됨
 *  (라인 idle 1/1 인데 clear bus failed/reset hardware failed, 버스 재생성도 무효.
 *   같은 순간 비트뱅은 0x3C ACK 정상). → OLED 전송을 비트뱅으로 전환해 페리페럴 우회.
 *  되돌리려면 0. (PCF 는 LP 비트뱅이라 공유버스 영향 없음) */
#define BOARD_OLED_BITBANG     1
#define BOARD_PIN_PCF_LP_SDA   6   // MTCK / LP_I2C_SDA (뒷면 패드, 비트뱅)
#define BOARD_PIN_PCF_LP_SCL   7   // MTDO / LP_I2C_SCL (뒷면 패드, 비트뱅)

/* ★2026-08-13 (③) LP 코어 폴링 위임 허용. GPIO6/7 = C6 LP_I2C 고정핀과 일치하므로
 *  이 배선(뒷면 LP_I2C)으로 붙은 개체에서는 LP 코어가 PCF 폴링+로터리 디코딩을
 *  맡을 수 있다. 앞면 공유 I2C(22/23)로 폴백된 개체는 런타임 판정에서 걸러져
 *  **현행 HP 폴링 그대로** 동작한다(같은 펌웨어가 두 배선 모두 지원). */
#define BOARD_PCF_LP_CORE      1
#define BOARD_PIN_PCF_INT      2   // D2 (GPIO2, LP) ~INT (active-LOW, wake) — 공통

/* ════════════════════════════════════════════════════════
   GPIO 직결 센서 (light sleep wake)
     VIBE 는 LP_GPIO(D0=GPIO0) 에 배치해 light-sleep wake 안정성↑.
     (구 GPIO3/D11 은 XIAO 에서 미노출 → D0 으로 이동.)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_CHG_STAT     17  // D7 (GPIO17) — XIAO 온보드 NCHG 미노출이라 충전감지
                                   //   불가. 선택: VBUS(5V) 분압→USB 연결감지(active-HIGH,
                                   //   GNPE 와 극성 반대). 미사용이면 GND 고정(floating 금지).
#define BOARD_PIN_VIBE         0   // D0  (GPIO0, LP) VS1 진동 스위치 (구 GPIO3/D11 미노출)

/* 배터리/충전 — SGM40567-4.2 충전전류 120 mA(R10=200K, ICharge=24000/200K).
 *   ⚠ BATT_MAH 는 BAT 패드에 직결하는 실제 셀 용량으로 맞출 것(아래는 대표값).
 *   완충시간 = (BATT_MAH/CHG_MA) h × 1.25. 예: 500mAh → 4.17h×1.25 ≈ 5.2h. */
#define BOARD_BATT_MAH        500   // XIAO 직결 LiPo 대표값 — 실제 셀로 조정
#define BOARD_CHG_MA          120   // SGM40567 충전전류(mA)

/* 로터리: XIAO 에 장착한 EC05 는 하프스텝(11·00 양쪽 디텐트) → LUT 누산 디코더 */
#define BOARD_ROT_HALF_STEP        1

/* 충전 감지(A+B) — VBUS 분압(active-HIGH) + BAT 분압 ADC(실측 %) */
#define BOARD_CHG_STAT_ACTIVE_HIGH 1   // CHG_STAT(D7/GPIO17) ← VBUS(5V) 100k/150k 분압(~3.0V)
/* ★ 분압 하단 150k 가 이미 풀다운이므로 내부 풀다운을 켜면 안 된다: 내부 ~45kΩ 가
 *   병렬로 물리면 3.0V→1.29V 로 끌려내려가 USB 를 LOW 로 오독한다(실측 확인).
 *   → _is_usb_powered() 항상 false → USB 인데 1분 유휴에 절전(화면 꺼짐) +
 *     배터리 미연결 판정이 성립 불가(USB=0 로 조건 탈락). (2026-07-17) */
#define BOARD_CHG_STAT_EXT_PULLDOWN 1
#define BOARD_HAS_BAT_ADC          1
/* 구 기판은 분압이 오배선이었다: BAT_ADC핀(GP1)=VBUS, CHG_STAT핀(GP17)=BAT.
 *   GP17 이 ADC 불가핀이라 잔량 % 측정 불가 → "USB/BAT/LOW" 상태 표시로 동작(=1).
 * ★2026-07-17: BAT_ADC/CHG_STAT 회로를 **정상 배선으로 개선한 신 PCB** 로 교체됨
 *   (GP1=BAT 분압 → ADC 실측 %, GP17=VBUS 분압 → USB 감지) → **0 으로 전환**.
 *   =1 경로(USB/BAT/LOW 텍스트)는 somfy_app·oled_ui 의 #if 로 그대로 보존(삭제 금지). */
#define BOARD_BAT_SWAPPED          0
#define BOARD_PIN_BAT_ADC          1   // D1 (GPIO1=A1, spare) ← BAT 100k/100k 분압
#define BOARD_BAT_DIV_TOP          100 // 100k/100k → Vbat = Vadc×2
#define BOARD_BAT_DIV_BOT          100
