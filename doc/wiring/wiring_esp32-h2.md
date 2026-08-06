# 배선도 — ESP32-H2 SuperMini (검증)

> **보드 핀맵 단일 진실원천: [`main/boards/esp32-h2.h`](../../main/boards/esp32-h2.h)**
> 핀 출처: `doc/esp32/SuperMini/` (핀맵 `esp32-h2-supermini0.jpg`·`esp32_h2_superMini1.jpg`
> + 회로도 `esp32-h2-superMini2.png`). 실물 동작 검증 완료.

- **SoC / IDF target**: ESP32-H2 (RISC-V, 802.15.4 + BLE, **WiFi 없음**) / `esp32h2`
- **트랜스포트**: Thread 만 가능 → **Matter over Thread** (Thread Border Router 필요)
- **OLED**: 외부 모듈 (기본 72×40 가정 — 실제 패널에 맞춰 `BOARD_OLED_*` 조정).
  렌더러는 **해상도로 자동 선택** — 128×64 모듈을 달면 풀스크린 네이티브 렌더러가
  자동 적용된다(`esp32-h2.h` 를 128×64/OFFSET 0/FIXUP 0 으로, `sdkconfig` `CONFIG_OFFSETX=0`)
- **첫 빌드 전 1회**: `./build.ps1 -Board esp32-h2 -Action set-target`
- **빌드**: `./build.ps1 -Board esp32-h2 -Action build -Rotate 180` — 현 시제품 기판은 OLED 가 **180° 장착**이라 `-Rotate 180` 필수(flash·ota-image 도 동일)

## 핀맵 이미지 (SuperMini)

![ESP32-H2 SuperMini 핀맵](<../esp32/SuperMini/esp32-h2-supermini0.jpg>)

> 회로도: [`esp32-h2-superMini2.png`](<../esp32/SuperMini/esp32-h2-superMini2.png>).
> 핀 번호 최종 기준은 아래 표(=`boards/esp32-h2.h`). **GP6/GP7 미노출**, GP8(LED)·
> GP9(BOOT)·GP26/27(USB) 회피.

## CC1101 모듈 (E07-M1101D-SMA) 핀맵

![E07-M1101D-SMA 핀맵](<../CC1101/E07 (M1101D-SMA)/M07-M1101D_MAPPING.jpg>)

> ※ **제작 보드는 E07-400MM10S 모듈** 사용 — 같은 CC1101 칩이라 핀맵은 동일하나, 크리스털 오차가
>   달라 register **447.70** → on-air 447.673 MHz(실제 블라인드 인식 확인). 테스트용 E07-M1101D-SMA 는
>   447.72 → 447.675. 주파수 상세는 README "RF / Somfy RTS 447" 섹션 참고.

## ★ I2C 공유 (이 보드의 핵심 차이)

ESP32-H2 에는 **LP_I2C 가 없고** SuperMini 의 가용 핀도 빠듯하다. 그래서
**OLED 와 PCF8574 를 하드웨어 I2C 한 버스에 공유**한다(SDA/SCL 동일 핀, 주소만
다름: OLED `0x3C` / PCF8574 `0x20`).

- 보드 파일에서 `BOARD_I2C_SHARED = 1`, 그리고 `BOARD_PIN_OLED_SDA == BOARD_PIN_PCF_SDA`,
  `BOARD_PIN_OLED_SCL == BOARD_PIN_PCF_SCL` (board_select.h 가 컴파일타임에 검증).
- 펌웨어: 공유 시 PCF8574 는 비트뱅이 아니라 **OLED 가 만든 i2c_master 버스에
  device 로 붙어** 폴링한다(`button_handler.c` 의 `BOARD_I2C_SHARED` 분기 →
  `i2c_master_bus_add_device` + `i2c_master_receive`). 버스 접근은 새 I2C 드라이버가
  내부적으로 직렬화하므로 OLED 갱신과 PCF 폴링이 안전하게 공존한다.
- **GNPE 는 공유 안 함**(OLED=HW I2C + PCF8574=비트뱅, 서로 다른 핀, `BOARD_I2C_SHARED=0`).
  **XIAO 는 H2 와 동일하게 공유**(`BOARD_I2C_SHARED=1`, OLED 버스에 PCF device) — XIAO
  실기에서 이 공유 경로가 처음 검증됐고, 그 과정에서 읽기 타임아웃 버그(5ms→50ms)도 수정됨.

> 공유 버스이므로 **외부 4.7 kΩ pull-up 은 SDA/SCL 각 1개씩만** 필요(OLED·PCF 공통).

## 핀 매핑 (SuperMini 실측 기반)

| 기능 | 신호 | GPIO | 비고 |
|---|---|---|---|
| CC1101 | SCK  | **GP4**  | FSPICLK (IO-MUX 네이티브) |
| CC1101 | MISO | **GP0**  | FSPIQ |
| CC1101 | MOSI | **GP5**  | FSPID |
| CC1101 | CS   | **GP1**  | (FSPICS0) active-LOW |
| CC1101 | GD0  | **GP10** | RMT 비동기 TX 데이터 |
| **I2C 공유** | SDA | **GP13** | OLED+PCF8574 공통, 4.7 kΩ pull-up (구 GP12) |
| **I2C 공유** | SCL | **GP14** | OLED+PCF8574 공통, 4.7 kΩ pull-up (구 GP22) |
| PCF8574 | ~INT | **GP11** | active-LOW wake, 10 kΩ pull-up |
| 센서   | CHG_STAT | **GP12** | **USB 감지** — VBUS 분압 active-HIGH (충전 섹션) |
| 센서   | BAT ADC | **GP3** | 배터리 전압 분압 ADC → 실측 % (충전 섹션) |
| 센서   | VIBE | **GP2** | VS1 진동 스위치 (비-strapping 가용핀, 구 GP25) |

## PCF8574 (I2C addr 0x20, bit-bang)

`A0/A1/A2 → GND` (주소 0x20). 8개 P-핀은 회로 고정:

| P-핀 | 연결 | 기능 |
|---|---|---|
| P0 | ROT_A | 로터리 CW/CCW |
| P1 | ROT_B | 로터리 CW/CCW |
| P2 | ROT_BTN | 클릭=STOP/MY (주파수 편집은 설정 메뉴 `Freq Edit` — v3.5 이후) |
| P3 | SW6 SETUP | 짧게=설정 메뉴, 2초=메뉴 항목 저장/실행 |
| P4 | SW1 UP | 블라인드 ↑ |
| P5 | SW2 DOWN | 블라인드 ↓ |
| P6 | SW3 SELECT | 블라인드 선택 순환 |
| P7 | SW4 PROG | Somfy PROG (2초 롱프레스=모터 등록) |

## ⚠️ SuperMini 미노출/금지 핀

| GPIO | 사유 |
|---|---|
| **GP6 / GP7** | SuperMini 에 **미노출** |
| GP8 | RGB LED / LOG (+ strapping) — 사용 금지 |
| GP9 | BOOT strapping |
| GP26 / GP27 | USB D-/D+ |
| GPIO15~21 | in-package SPI flash |

**노출 핀**: GP0-5 · GP10-14 · GP22-25. **사용 중**: CC1101(GP0/1/4/5/10) · I2C(GP13/14) ·
~INT(GP11) · CHG_STAT(GP12) · BAT ADC(GP3) · VIBE(GP2). → **현재 여유: GP22·GP23·GP24·GP25.**
(GP13/GP14 는 32.768kHz xtal 겸용 — SuperMini 는 외부 32K 미실장이라 일반 GPIO 로 사용
가능하며 **현재 I2C(SDA=GP13/SCL=GP14)에 배정**. 외부 32K 크리스털을 달 거면 I2C 를 다른 핀으로.)

## 🔋 충전 / 배터리

**온보드 충전회로** (스키매틱 [`esp32-h2-superMini2.png`](<../esp32/SuperMini/esp32-h2-superMini2.png>)):
- 충전 IC **TP4054**(SOT23-5, 단셀 Li-ion), **R_PROG=10 kΩ → ~100 mA**.
- 파워패스 BAT→Schottky(PD1)→VCC, **LDO ME6217C33**(3.3V). BAT 패드에 단셀 LiPo 직결.
- 충전 status(CHRG)는 **녹색 LED 전용**이라 GPIO 로 안 빠짐(XIAO NCHG 와 동일).

**충전 감지 — A+B** (펌웨어 구현됨 · `BOARD_CHG_STAT_ACTIVE_HIGH=1` · `BOARD_HAS_BAT_ADC=1`).
**배터리 쓸 때만** 분압 2개를 납땜한다:

```
A. USB 감지 :  VBUS(5V) ──100k──◉──150k──GND      ◉ = CHG_STAT(GP12), active-HIGH
B. 배터리   :  BAT ──────100k──◉──100k──GND       ◉ = GP3 (ADC) → OCV-SoC → 실측 %
```

- **충전중** = A(USB)=HIGH AND Vbat < ~4.15V / **만충** = A AND Vbat ≈ 4.2V.
- 배터리 미사용 시 **생략 가능**(펌웨어 정상 — 충전 애니메이션만 비활성). 폴백 추정시간은
  `BOARD_BATT_MAH` · `BOARD_CHG_MA=100` 에서 파생.
- ⚠ B 분압 기생소모 ~21 µA(100k/100k). 절전 중요하면 **1M/1M + ADC 노드 100 nF**.
- 단셀은 **PCM(보호회로) 내장 셀** 권장 — TP4054 는 충전만, 과방전 보호는 셀 PCM.

> ⚠️ **현 시제품 기판은 위 A·B 분압이 핀에서 뒤바뀌어 있다**(BAT_ADC핀 **GP3 ← VBUS** 100k/150k,
>   CHG_STAT핀 **GP12 ← BAT** 100k/100k). GP12 는 ADC·아날로그 comparator 불가핀(H2 ADC=GP1~5,
>   ana_cmpr SRC=GP11)이라 **배터리 잔량 % 측정이 물리적으로 불가** → `boards/esp32-h2.h` 의
>   **`BOARD_BAT_SWAPPED=1`** 로 화면 우상단에 **`USB`/`BAT`/`LOW` 상태**를 표시한다(GP3 의 VBUS 를
>   ADC 로 읽어 USB 감지 + GP12 내부풀업 1임계≈3V 로 저전압 판별). 위 **정상 배선(BAT→GP3, VBUS→GP12)
>   으로 기판을 고치면 `BOARD_BAT_SWAPPED=0`**(기본값) → 기존 충전률 % 표시로 자동 복귀(두 경로 코드 보존).

## 비고

- CC1101 SPI 는 H2 의 **GPSPI2(FSPI) IO-MUX 네이티브 핀**(CLK=GP4/D=GP5/Q=GP0)에 배치.
- CC1101 은 **3.3V 전용** — 5V 금지.
- Thread Border Router 필요 (SmartThings Hub v3+, Apple TV 4K 등).
