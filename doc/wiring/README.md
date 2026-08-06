# 배선도 (보드별)

보드마다 핀 배열이 다르므로 배선 문서를 **보드별로 분리**한다. 각 문서의 핀맵은
대응하는 `main/boards/<board>.h` 헤더(= 단일 진실원천)에서 파생한다.

| 보드 | 문서 | SoC | OLED | Matter PID | 상태 |
|---|---|---|---|---|---|
| **GNPE ESP32-C6-0.42** | [`wiring_gnpe-c6.md`](wiring_gnpe-c6.md) · [상세 HTML](wiring_gnpe-c6.html) | ESP32-C6 | 내장 0.42" 72×40, **180° 회전** | `0x8000` | ✅ 검증 |
| **Seeed XIAO ESP32-C6** | [`wiring_xiao-c6.md`](wiring_xiao-c6.md) | ESP32-C6 | 외부 0.96" 128×64, **정방향** | `0x8003` | ✅ 검증 (**I2C 공유**) |
| **ESP32-H2 SuperMini** | [`wiring_esp32-h2.md`](wiring_esp32-h2.md) | ESP32-H2 | 외부 모듈 | `0x8001` | ✅ 검증 (SuperMini 핀 반영, **I2C 공유**) |

> 📺 **디스플레이 규격은 보드별로 다르다** — `boards/<board>.h` 의 `BOARD_OLED_*`
> (해상도/오프셋/회전/72×40 보정/주소)가 단일 진실원천. 컬럼 오프셋만 라이브러리
> 제약상 `sdkconfig` 의 `CONFIG_OFFSETX` 와 짝을 맞춘다(GNPE 28 / XIAO 0).
>
> 🔀 **렌더러는 "해상도"로 자동 선택**(보드 무관) — 펌웨어(`oled_ui.c`)는
> `BOARD_OLED_WIDTH×HEIGHT` 만 보고 렌더러를 고른다(`oled_ui.h` 의 `OLED_RENDER_*`).
>
> | 해상도 | 렌더러 |
> |---|---|
> | **128×64 (가로)** | 풀스크린 네이티브 — 고딕 폰트 + 7세그 시계 (가로 배치) |
> | **64×128 (세로)** | 풀스크린 네이티브 — 고딕 폰트 + 7세그 시계 (세로 적층) |
> | **그 외** | 72×40 논리 캔버스 (어떤 패널에도 중앙 배치) |
>
> 따라서 **어느 보드든 OLED 를 다른 규격으로 교체 가능** — 보드 헤더의 `BOARD_OLED_*`
> 만 바꾸면 맞는 렌더러가 자동 적용된다(코드 수정 불필요). 예: 128×64 → `BOARD_OLED_*`
> 를 128×64/OFFSET 0/FIXUP 0, 세로 64×128(SH1107 등) → 64×128 으로 선언.

> ⚠️ **핀맵의 단일 진실원천은 `main/boards/<board>.h`** 다. 문서가 헤더와 상충하면
> 헤더(+ 루트 `README.md`)가 우선이다. `wiring_gnpe-c6.html` 의 일부 버전 이력
> 섹션은 과거 표기를 보존하므로 현행과 다를 수 있다.

## 공통 사항 (모든 보드)

- **CC1101 은 3.3V 전용** — 5V 연결 시 즉시 파손.
- **PCF8574 / PCF8575** I2C 확장칩(주소 `0x20`, A0/A1/A2 → GND) — 버튼·로터리를 한 칩에
  모아 읽는다. 외부 풀업 SDA/SCL **4.7 kΩ** · ~INT **10 kΩ** → +3V3. 좌/우 버튼 유무로
  칩 분기(`BOARD_HAS_LR_BUTTONS`). **상세 핀맵·배선표 ↓ [§PCF8574 / PCF8575 핀맵·배선](#pcf8574--pcf8575-핀맵배선).**
- **I2C 버스 구성 (`BOARD_I2C_SHARED`)**:
  - **분리(GNPE)**: OLED=하드웨어 I2C, PCF8574=소프트웨어 비트뱅 — 서로 다른 핀.
  - **공유(XIAO·H2)**: OLED 와 PCF8574 가 **하드웨어 I2C 한 버스** (SDA/SCL 동일 핀,
    주소만 0x3C/0x20). 펌웨어가 PCF8574 를 OLED 버스의 device 로 폴링.
  - **XIAO 는 런타임 자동 폴백**(`BOARD_I2C_LP_FALLBACK=1`): 부팅 시 **LP_I2C(뒷면 MTCK=6/
    MTDO=7) 비트뱅**으로 PCF8574 를 먼저 프로브 → 무응답이면 **공유 HW I2C(D4/D5=GPIO22/23)**
    로 자동 전환. 한 펌웨어가 두 배선(뒷면 LP / 앞면 공유)을 모두 지원(택1 불필요).
- Light-sleep wake 소스: PCF8574 ~INT(버튼/로터리) · VIBE(진동) · CHG_STAT(충전).
- 보드 추가/교체 방법: `main/boards/` 에 `<brand>-<soc>.h` 추가 → `board_select.h`
  분기 → `build.ps1` `$BoardMap` 등록. (자세히는 루트 `README.md`)

---

### PCF8574 / PCF8575 핀맵·배선

버튼·로터리 입력을 한 I2C 칩으로 모은다. 좌/우 버튼이 없으면 **PCF8574**(8비트),
있으면 **PCF8575**(16비트)를 `BOARD_HAS_LR_BUTTONS` 로 분기한다. 핀↔신호 매핑의
단일 진실원천은 `main/button_handler.h`, 보드별 SDA/SCL/INT 핀은 `main/boards/<board>.h`.

**① 핀 ↔ 신호 매핑**

| 핀 | 비트 | 신호 | 펌웨어 매크로 | 칩 | 스위치 |
|---|---|---|---|---|---|
| P0 | 0 | ROT_A | `PCF8574_BIT_ROT_A` | 공통 | 로터리 A상 |
| P1 | 1 | ROT_B | `PCF8574_BIT_ROT_B` | 공통 | 로터리 B상 |
| P2 | 2 | ROT_BTN | `PCF8574_BIT_ROT_BTN` | 공통 | 로터리 푸시 |
| P3 | 3 | SETUP | `PCF8574_BIT_SETUP_BTN` | 공통 | SW6 |
| P4 | 4 | UP | `PCF8574_BIT_BTN_UP` | 공통 | SW1 |
| P5 | 5 | DOWN | `PCF8574_BIT_BTN_DOWN` | 공통 | SW2 |
| P6 | 6 | SELECT | `PCF8574_BIT_BTN_SEL` | 공통 | SW3 |
| P7 | 7 | PROG | `PCF8574_BIT_BTN_PROG` | 공통 | SW4 |
| P10 | 8 | LEFT | `PCF8574_BIT_BTN_LEFT` | **8575 전용** | 좌 버튼 |
| P11 | 9 | RIGHT | `PCF8574_BIT_BTN_RIGHT` | **8575 전용** | 우 버튼 |
| P12~P17 | 10~15 | — | (미사용) | 8575 여유 | — |

> 모든 입력 **active-LOW**(눌림=0). 펌웨어가 `0xFF` 를 써 입력으로 두고 칩 내부 약풀업
> 으로 평상시 HIGH. 좌/우(P10·P11)는 `BOARD_HAS_LR_BUTTONS=1` 일 때만 유효.

**② 버스측 배선 (PCF8574·PCF8575 동일)**

| 칩 핀 | 연결 | 풀업 |
|---|---|---|
| VCC | +3V3 (CC1101 과 동일 3.3V) | — |
| GND | GND | — |
| SDA | MCU SDA (보드별 ③) | 4.7 kΩ → +3V3 |
| SCL | MCU SCL (보드별 ③) | 4.7 kΩ → +3V3 |
| ~INT | MCU wake 핀 (보드별 ③) | 10 kΩ → +3V3 |
| A0·A1·A2 | GND | — (→ 주소 `0x20`) |

**③ 보드별 SDA / SCL / ~INT** (출처: `boards/<board>.h`)

| 보드 | SDA | SCL | ~INT | I2C 버스 |
|---|---|---|---|---|
| GNPE C6 | `19` | `18` | `17` | 분리 (OLED=HW · PCF=SW 비트뱅) |
| XIAO C6 — LP_I2C(우선) | `6` (MTCK) | `7` (MTDO) | `2` (D2) | 비트뱅 — 부팅 프로브 시 응답하면 사용 |
| XIAO C6 — 공유(폴백) | `22` (D4) | `23` (D5) | `2` (D2) | OLED HW I2C — LP 무응답 시 자동 전환 |
| H2 SuperMini | `13` (GP13) | `14` (GP14) | `11` (GP11) | OLED 와 HW I2C 공유 |

> 현재 세 보드 모두 **PCF8574** 사용(좌/우 버튼 미장착). 좌/우 버튼을 달면 같은
> 배선·주소 그대로 **PCF8575** 로 교체하고 `BOARD_HAS_LR_BUTTONS=1` 만 켠다.

**④ PCF8574 vs PCF8575**

| 항목 | PCF8574 | PCF8575 |
|---|---|---|
| I/O | 8 (P0~P7) | 16 (P00~P07 · P10~P17) |
| 주소 | `0x20` | `0x20` (동일) |
| read | 1 byte | 2 byte (`PCF_NBYTES`) |
| 좌/우 버튼 | ✗ | ✓ P10·P11 |
| 펌웨어 플래그 | `BOARD_HAS_LR_BUTTONS=0` | `=1` |
| 패키지 | 16핀 (DIP/SO/TSSOP) | 24핀 (SSOP/TSSOP) |

> 두 칩은 P0~P7 핀배치·주소·~INT 동작이 같은 **16비트 형제**. 보드 배선·풀업은
> 그대로 두고 칩만 교체하면 되며, 펌웨어는 플래그로 read 바이트 수를 1↔2 로 바꾼다.

**⑤ 로터리 디텐트 타입 (`BOARD_ROT_HALF_STEP`)**

| 값 | 인코더 | rest 위치 | 디텐트/사이클 | 디코더 (보드) |
|---|---|---|---|---|
| `0` | EC11 (full-step) | `11` | 1 | 단순 상태천이 (GNPE·H2) |
| `1` | EC05 (half-step) | `11`·`00` | 2 | 그레이코드 LUT 누산·바운스 상쇄 (XIAO) |

> 방향이 반대로 느껴지면 `button_handler.c` 의 `_ROT_CW_ON_AB1` 를 뒤집는다.
> 좌/우 버튼 동작 정의는 루트 `README.md` §좌/우 버튼.
