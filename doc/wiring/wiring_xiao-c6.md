# 배선도 — Seeed XIAO ESP32-C6

> **보드 핀맵 단일 진실원천: [`main/boards/xiao-c6.h`](../../main/boards/xiao-c6.h)**
> 핀 출처: `doc/esp32/XIAO/XIAO_ESP32C6_Pinout.xlsx`("QFN32 Pin Summary") +
> `Xiao esp32-c6.png`(앞/뒷면) + `XIAO_ESP32_C6_v1.0_SCH_260114.pdf` (Seeed 공식).

- **브랜드/제품**: Seeed Studio "XIAO ESP32-C6"
- **SoC / IDF target**: ESP32-C6 (GNPE 와 동일 SoC) / `esp32c6`
- **Matter Product ID**: `0x8003` (GNPE 0x8000 과 구분 — 같은 SoC 라 OTA 는 PID 로만 구분)
- **OLED**: XIAO 에 내장 OLED 없음 → **외부 0.96" SSD1306 128×64** (I2C 0x3C). 현 시제품 기판은 **180° 회전 장착** → 빌드 시 `-Rotate 180`
- **첫 빌드 전 1회**: `./build.ps1 -Board xiao-c6 -Action set-target`
- **빌드**: `./build.ps1 -Board xiao-c6 -Action build -Rotate 180` — 현 시제품 기판은 OLED 가 **180° 장착**이라 `-Rotate 180` 필수(flash·ota-image 도 동일)

## 핀맵 이미지 (앞/뒷면)

![Seeed XIAO ESP32-C6 핀맵 (앞/뒷면)](<../esp32/XIAO/Xiao esp32-c6.png>)

> 앞면 11 castellated + 뒷면 패드(MTCK/MTDO=LP_I2C 등). 최종 기준은 아래 표
> (=`boards/xiao-c6.h`). **D11(GPIO3)은 미노출** 주의(이미지 라벨과 별개로 보드 미연결).

## CC1101 모듈 (E07-M1101D-SMA) 핀맵

![E07-M1101D-SMA 핀맵](<../CC1101/E07 (M1101D-SMA)/M07-M1101D_MAPPING.jpg>)

> ※ **제작 보드는 E07-400MM10S 모듈** 사용 — 같은 CC1101 칩이라 핀맵은 동일하나, 크리스털 오차가
>   달라 register **447.70** → on-air 447.673 MHz(실제 블라인드 인식 확인). 테스트용 E07-M1101D-SMA 는
>   447.72 → 447.675. 주파수 상세는 README "RF / Somfy RTS 447" 섹션 참고.

## XIAO 패드 ↔ GPIO (앞면 11 edge + 뒷면 패드)

```
[앞면 edge — 11 castellated]
D0=GPIO0(A0,LP)  D1=GPIO1(A1,LP)  D2=GPIO2(A2,FSPIQ,LP)  D3=GPIO21
D4=GPIO22(SDA)   D5=GPIO23(SCL)   D6=GPIO16(TX)          D7=GPIO17(RX)
D8=GPIO19(SCK)   D9=GPIO20(MISO)  D10=GPIO18(MOSI)

[뒷면 패드 — 노출 확인된 것]
MTCK=GPIO6 (A6, LP_GPIO6, LP_I2C_SDA, FSPICLK)   ← 하드웨어 LP_I2C
MTDO=GPIO7 (    LP_GPIO7, LP_I2C_SCL, FSPID)     ← 하드웨어 LP_I2C
```

> 📌 **노출 정정(보드 실물 기준)**
> - **D11=GPIO3(A3) 은 브레이크아웃되지 않음 → 사용 불가.** (칩 핀에는 존재하나
>   XIAO 보드가 패드로 빼지 않음.) 이전에 VIBE 를 GPIO3 에 뒀던 것을 **D0(GPIO0)** 로 옮김.
> - 뒷면 **MTCK=GPIO6=`LP_I2C_SDA`, MTDO=GPIO7=`LP_I2C_SCL`** 는 하드웨어 LP_I2C
>   전용핀 → PCF8574 를 여기 두면 부팅 자동 폴백이 우선 사용(미연결 시 공유로 전환).
> - GPIO8 미노출. MTMS=GPIO4·MTDI=GPIO5 의 뒷면 노출 여부는 미확인 → **사용하지 않음**.
> - **확정 가용핀 = 앞면 D0~D10(11) + 뒷면 MTCK/MTDO(2) = 13** (프로젝트 12핀 사용).

## 핀 매핑 (요약)

| 기능 | 신호 | 패드 | GPIO | 비고 |
|---|---|---|---|---|
| CC1101 | SCK  | D8  | **GPIO19** | FSPICLK, GPIO-matrix 라우팅 |
| CC1101 | MISO | D9  | **GPIO20** | FSPIQ |
| CC1101 | MOSI | D10 | **GPIO18** | FSPID |
| CC1101 | CS   | D3  | **GPIO21** | (FSPICS0) active-LOW |
| CC1101 | GD0  | D6  | **GPIO16** | RMT 비동기 TX 데이터 |
| OLED   | SDA  | D4  | **GPIO22** | XIAO 기본 I2C — 0.96" SSD1306 128×64 |
| OLED   | SCL  | D5  | **GPIO23** | I2C 0x3C |
| PCF8574 | SDA | D4(공유)/MTCK | **GPIO22** / 6 | 자동 폴백 — LP_I2C(6) 우선 프로브 → 무응답 시 공유 OLED 버스(22) |
| PCF8574 | SCL | D5(공유)/MTDO | **GPIO23** / 7 | 〃 pull-up: 공유=OLED 공통 1쌍 / LP=PCF 전용 1쌍 |
| PCF8574 | ~INT | D2 | **GPIO2** (LP) | active-LOW wake, 10 kΩ pull-up |
| 센서   | CHG_STAT | D7 | **GPIO17** | **USB 감지** — VBUS 분압 active-HIGH (충전 섹션) |
| 센서   | BAT ADC | D1 | **GPIO1** (A1) | 배터리 전압 분압 ADC → 실측 % (충전 섹션) |
| 센서   | VIBE | D0 | **GPIO0** (LP) | VS1 진동, light-sleep wake (구 GPIO3/D11 미노출) |

> **PCF8574 배선 — 런타임 자동 폴백 (`BOARD_I2C_LP_FALLBACK=1`):** 부팅 시 펌웨어가
> **LP_I2C(뒷면 MTCK=GPIO6 / MTDO=GPIO7, bit-bang)** 에서 PCF8574(0x20)를 먼저 프로브하고,
> **무응답이면 공유 HW I2C(D4/D5=GPIO22/23, OLED 버스)** 로 자동 전환한다. → 어느 쪽에
> 배선해도 같은 펌웨어로 동작(택1·재빌드 불필요).
> - **공유(앞면 D4/D5)**: 납땜 쉬움, 외부 4.7 kΩ pull-up 은 OLED·PCF **공통 1쌍(저항 2개)**. 뒷면 작업 불요.
> - **LP_I2C(뒷면 MTCK/MTDO)**: ⚠️ **이 구성은 풀업이 총 2쌍(저항 4개) 필요하다.**
>   OLED 는 **언제나** 앞면 D4/D5(GPIO22/23)에 붙으므로 "D4/D5 를 비운다"는 성립하지 않고,
>   PCF 를 LP 로 빼는 순간 **I2C 버스가 물리적으로 둘로 분리**된다 → **버스마다 1쌍씩** 필요:
>     · **OLED 버스(D4/D5=GPIO22/23) 전용 4.7 kΩ 1쌍 → +3V3**
>     · **PCF 버스(MTCK/MTDO=GPIO6/7) 전용 4.7 kΩ 1쌍 → +3V3**
>   ※ 그리고 **OLED VCC 옆 100 nF 디커플링 1개**도 같이 둘 것.
>
> 🔴 **실기 사고 기록(2026-07-16)**: 이 문서가 위 "OLED 버스 전용 1쌍"을 명시하지 않아,
>   PCF 쪽에만 풀업을 단 기판이 제작됐다. 결과 — OLED 버스에 **ESP32 내부 풀업(~45 kΩ)만**
>   남았고, 배선 용량 ~200 pF 기준 τ≈9 µs·VIH(2.3 V) 도달 ≈10.7 µs 라 **400 kHz(비트주기
>   2.5 µs)는 물론 100 kHz(10 µs)에서도 상승시간이 모자란다.** 증상: OLED 가 I2C 프로브에
>   **응답조차 못 함(화면 안 켜짐)** / 어쩌다 붙어도 **INVALID_STATE 폭주 → 찌그러짐·멈춤**.
>   → **내부 풀업으로는 어떤 속도로도 못 쓴다. 외부 4.7 kΩ 는 선택이 아니라 필수.**
>
> ⚠️ **JTAG**: 자동 폴백이 부팅 프로브로 GPIO6/7 을 GPIO 로 잡으므로, 공유 배선이라도
> **MTCK/MTDO JTAG 디버그는 불가**. JTAG 가 꼭 필요하면 `BOARD_I2C_LP_FALLBACK=0` +
> 공유 단독(`BOARD_I2C_SHARED=1`)으로 빌드. 앞면 **D1(GPIO1)=BAT ADC** 는 그대로.

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

> 🔁 **로터리 = EC05 (하프스텝)** — 디텐트가 `11`·`00` **양쪽**(2디텐트/사이클)이라
> `boards/xiao-c6.h` 에 **`BOARD_ROT_HALF_STEP=1`** 설정 → 그레이코드 LUT 누산 디코더
> (바운스 상쇄·양방향 대칭). GNPE 의 **EC11(full-step, rest@11)** 과 디코더가 다르다.
> 방향이 반대로 느껴지면 `button_handler.h` 의 `_ROT_CW_ON_AB1` 를 뒤집는다.

## ⚠️ 사용 금지/주의 핀

| GPIO | 용도 | 주의 |
|---|---|---|
| **GPIO14** | 온보드 **RF 스위치(ANT1/ANT2)** | **절대 사용 금지** — C6 내장 라디오 안테나 선택, Thread/BLE RF 에 영향 |
| GPIO15 | USER LED | 사용 시 LED 와 충돌 |
| GPIO9  | BOOT (strapping) | 부팅 영향 |
| GPIO12/13 | USB D-/D+ | USB 사용 시 금지 |
| **GPIO3 (D11)** | 미노출 | XIAO 보드가 패드로 빼지 않음 → **사용 불가** |
| GPIO8 | 미노출 | 사용 불가 |
| **GPIO4/5 (MTMS/MTDI)** | 뒷면 TP 노출되나 **회피** | Seeed 스키매틱 명시 *"Avoid using GPIO4, 5, 8, 9, 15"* (VDDA3P3 인접) |

> ✅ MTCK=GPIO6·MTDO=GPIO7(=LP_I2C)은 위 회피목록에 **없음** → PCF8574 LP_I2C(비트뱅) 용
> OK. 자동 폴백이 부팅 시 이 핀을 프로브하므로 JTAG 와는 양립 불가(위 PCF 배선 주석 참고).

> CC1101 SPI 는 FSPI 네이티브(FSPICLK/D=GPIO6/7) 대신 D8/D9/D10(GPIO-matrix) 사용 —
> GPIO6/7 은 PCF8574 LP_I2C(자동 폴백 프로브) 전용. CC1101 은 저속이라 matrix 라우팅으로 충분.

## 🔋 배터리 / 충전 (XIAO 온보드 — 외부 충전회로 불필요)

> 출처: XIAO ESP32-C6 공식 스키매틱 `XIAO_ESP32_C6_v1.0_SCH_260114.pdf` (03 Power).

**GNPE 와 가장 큰 차이**: XIAO 는 충전·파워패스가 **이미 보드에 내장**되어 있어,
**BAT+/BAT- 패드에 단셀 LiPo 를 직결**하기만 하면 된다. GNPE 처럼 외장 충전회로
(MCP73831 + SS14 + 충전저항 + 충전 LED)를 추가할 필요가 **전혀 없다**.

| 항목 | XIAO 온보드 | 비고 |
|---|---|---|
| 충전 IC | **SGM40567-4.2** | 단셀 LiPo, 4.2V 컷오프 |
| 충전 전류 | **120 mA** | R10=200K, `ICharge=24000/200K` (GNPE MCP73831 ~300mA 보다 느림) |
| 파워패스 | USB=USB동작+충전 / 분리=배터리동작 | Q1 P-MOS + D1 Schottky 자동 전환 |
| DC-DC | **SGM6029C** 5V→3.3V | 3V3_OUT 패드로 출력 |
| 노출 패드 | **BAT+/BAT-**, **VBUS(5V)**, 3V3_OUT, GND | 배터리·USB·전원 |

### 충전 감지 — A+B 방식 (펌웨어 구현됨)

충전 IC status `NCHG` 는 **온보드 빨강 LED(CHG1) 전용**(GPIO 미노출)이라 직접 못 읽는다.
대신 분압 2개로 **USB 연결(A) + 배터리 전압(B)** 를 읽어 충전중/만충을 판별한다
(`boards/xiao-c6.h`: `BOARD_CHG_STAT_ACTIVE_HIGH=1` · `BOARD_HAS_BAT_ADC=1`).

| | 분압 배선 | 핀 | 펌웨어 |
|---|---|---|---|
| **A. USB 감지** | VBUS(5V)─100k─◉─150k─GND | ◉ = **D7 (GPIO17)** (~3.0V) | active-HIGH (HIGH=USB) |
| **B. 배터리 전압** | BAT+─100k─◉─100k─GND | ◉ = **D1 (GPIO1=A1)** ADC | ADC→OCV-SoC→실측 % |

- **충전중 vs 만충**: A(USB)=HIGH AND Vbat<~4.15V → **충전중** / A AND Vbat≈4.2V → **만충**.
- 분압비·핀은 `boards/xiao-c6.h` (`BOARD_PIN_BAT_ADC` · `BOARD_BAT_DIV_*`)에서 조정.
- 미사용 시 D7 은 **GND 고정**(floating 금지). 실측 % 는 충전 중 단자전압이라 약간 높게 읽힘(표시용).

> ⚠️ 단셀은 **PCM(보호회로) 내장 셀** 사용 권장 — SGM40567 은 충전만 담당하고
> 과방전 보호는 셀 PCM 에 의존한다.

> ⚠️ **현 시제품 기판은 위 A·B 분압이 핀에서 뒤바뀌어 있다**(BAT_ADC핀 **D1/GPIO1 ← VBUS** 100k/150k,
>   CHG_STAT핀 **D7/GPIO17 ← BAT** 100k/100k). GPIO17 은 ADC·아날로그 comparator 불가핀(C6 ADC=GPIO0~6,
>   ana_cmpr 미지원)이라 **배터리 잔량 % 측정이 물리적으로 불가** → `boards/xiao-c6.h` 의
>   **`BOARD_BAT_SWAPPED=1`** 로 화면 우상단에 **`USB`/`BAT`/`LOW` 상태**를 표시한다(GPIO1 의 VBUS 를
>   ADC 로 읽어 USB 감지 + GPIO17 내부풀업 1임계≈3V 로 저전압 판별). 위 **정상 배선(BAT→GPIO1, VBUS→GPIO17)
>   으로 기판을 고치면 `BOARD_BAT_SWAPPED=0`**(기본값) → 기존 충전률 % 표시로 자동 복귀(두 경로 코드 보존).

## 🖥 디스플레이 (GNPE 와 다름)

| 항목 | XIAO | GNPE(참고) |
|---|---|---|
| 패널 | **외부 0.96" SSD1306 128×64** | 내장 0.42" SSD1315 72×40 |
| 방향 | **정방향**(회전 없음) | 거꾸로 장착 → SW 180° 회전 |
| 컬럼 오프셋 | **0** (`CONFIG_OFFSETX=0`) | 28 (SEG28~99) |
| 72×40 보정 | **불필요** | 필요(멀티플렉스/COM/IREF) |
| I2C 주소 | 0x3C | 0x3C |

- 규격은 **`boards/xiao-c6.h` 의 `BOARD_OLED_*`** 가 결정한다(WIDTH=128, HEIGHT=64,
  COL_OFFSET=0, ROTATE_180=0, FIXUP_72X40=0). 오프셋은 `sdkconfig.defaults.xiao_c6`
  의 `CONFIG_OFFSETX=0` 과 짝을 이룬다.
- **렌더러는 해상도로 자동 선택**된다 — 128×64 이므로 **풀스크린 네이티브
  렌더러**(고딕 6×9 폰트 + 7세그먼트 시계)가 적용된다(`oled_ui.h` 의
  `OLED_RENDER_128X64`). 72×40 논리 캔버스가 아니라 128×64 물리 픽셀에 1:1 로
  그린다. 다른 규격 패널을 달면 그 해상도에 맞는 렌더러가 자동으로 선택된다.

## 조립 주의

- CC1101 은 **3.3V 전용** — 5V 금지.
- PCF8574 SDA/SCL: **자동 폴백** — 뒷면 **MTCK/MTDO(GPIO6/7) LP_I2C** 우선, 미연결 시
  앞면 **D4/D5(GPIO22/23)** OLED 공유로 전환. 어느 쪽이든 같은 펌웨어로 동작.
  GPIO14(안테나)·GPIO15(LED)·GPIO12/13(USB)은 건드리지 말 것.
- 외부 풀업: 공유 배선=OLED·PCF 공통 SDA/SCL 각 **4.7 kΩ** 1쌍 / LP_I2C 배선=PCF 전용 별도 1쌍.
  ~INT 는 **10 kΩ** → +3V3.
- CC1101 은 저속이라 GPIO-matrix 라우팅으로 충분.
