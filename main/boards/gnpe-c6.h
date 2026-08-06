#pragma once
/*
 * boards/gnpe-c6.h
 * ──────────────────────────────────────────────────────────
 * 보드: GNPE ESP32-C6-0.42  (현 검증·배포 보드)
 *   • 브랜드/제품  : GNPE "ESP32-C6-0.42"
 *   • SoC          : ESP32-C6 (RISC-V, single core, 802.15.4 + BLE)
 *   • IDF target   : esp32c6
 *   • OLED         : SSD1315/SSD1306 72×40, 내부 배선 (IO0/IO1)
 *   • 외부 모듈   : CC1101 (SPI), PCF8574 (I2C bit-bang), VS1 진동스위치,
 *                   MCP73831 충전 모니터
 *   • Matter PID   : 0x8000 (OTA 보드 매칭; sdkconfig)
 *
 * ★ 같은 ESP32-C6 라도 XIAO 등 다른 브랜드는 핀 배열이 다르다 → 별도
 *   boards/<brand>-c6.h 로 분리. (chip-ID 는 동일 esp32c6 라 OTA 구분은
 *   Product ID 로만 됨 — boards 별 PID 분리 필수.)
 * ──────────────────────────────────────────────────────────
 */

#define BOARD_NAME            "gnpe-c6"
#define BOARD_IDF_TARGET      "esp32c6"

/* ════════════════════════════════════════════════════════
   CC1101 SPI (FSPI controller, IO-MUX 라우팅 가능 핀)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_CC1101_SCK   6   // FSPICLK (IO-MUX)
#define BOARD_PIN_CC1101_MISO  2   // FSPIQ   (IO-MUX)
#define BOARD_PIN_CC1101_MOSI  7   // FSPID   (IO-MUX)
#define BOARD_PIN_CC1101_CS    4
#define BOARD_PIN_CC1101_GD0   8   // Async TX 데이터 입력 핀

/* ════════════════════════════════════════════════════════
   OLED I2C (GNPE ESP32-C6-0.42 내부 배선 — 변경 불가)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_OLED_SDA     1   // silkscreen "SDA-1"
#define BOARD_PIN_OLED_SCL     0   // silkscreen "SCL-0"

/* ── OLED 디스플레이 규격 — GNPE 내장 0.42" SSD1315 72×40 ──
 *   • 패널이 컨트롤러 SEG28~99 구간에 매핑 → COL_OFFSET=28
 *     (sdkconfig CONFIG_OFFSETX=28 과 일치해야 함).
 *   • 라이브러리가 height=40 을 모름 → 72×40 보정 시퀀스(FIXUP) 필요.
 *   • 보드를 거꾸로 장착해 사용 → 소프트웨어 180° 회전.            */
#define BOARD_OLED_WIDTH        72
#define BOARD_OLED_HEIGHT       40
#define BOARD_OLED_COL_OFFSET   28   // CONFIG_OFFSETX 와 일치
#define BOARD_OLED_ROTATE_180    1   // 거꾸로 장착 → 180° 회전
#define BOARD_OLED_FIXUP_72X40   1   // SSD1315 72×40 멀티플렉스/COM/IREF 보정
#define BOARD_OLED_ADDR         0x3C

/* ════════════════════════════════════════════════════════
   PCF8574 I2C (bit-bang) — 모든 버튼 + 로터리 + SETUP
     8 P-핀 매핑은 회로 고정 (P0~P7):
        P0=ROT_A P1=ROT_B P2=ROT_BTN P3=SETUP
        P4=UP   P5=DOWN  P6=SELECT  P7=PROG
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_PCF_SDA      19  // PCF8574 SDA Pin15 (구 IO6)
#define BOARD_PIN_PCF_SCL      18  // PCF8574 SCL Pin14 (구 IO7)
#define BOARD_PIN_PCF_INT      17  // PCF8574 ~INT (active-LOW)

/* ════════════════════════════════════════════════════════
   GPIO 직결 센서 (light sleep wake 가능)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_CHG_STAT     3   // MCP73831 STAT (active-LOW)
#define BOARD_PIN_VIBE         16  // VS1 JYX-1210-X160 진동 스위치

/* 배터리/충전 — 충전량 시간추정(_estimate_battery_percent)에 사용.
 *   MCP73831 + 단셀 600 mAh, 충전전류 300 mA(0.5C).
 *   완충시간 = (BATT_MAH/CHG_MA) h × 1.25(CV 오버헤드) = 2.0h×1.25 = 2.5h. */
#define BOARD_BATT_MAH        600   // 단셀 LiPo 용량(mAh)
#define BOARD_CHG_MA          300   // MCP73831 충전전류(mA, 0.5C)
