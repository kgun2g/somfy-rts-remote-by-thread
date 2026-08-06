# 조립 및 설치 가이드 (v3.9 / Matter over Thread)

> 📋 PCB 변형의 자세한 비교 + 발주 옵션은 [`kicad/FINAL_REPORT.md`](../kicad/FINAL_REPORT.md) 참조.
>
> **v3.0 (2026-05-13) 부터** SmartThings 연동이 WiFi → **Matter over Thread (802.15.4)** 로 변경되었습니다.
> WiFi SoftAP 프로비저닝은 더 이상 사용되지 않으며, **Thread Border Router** 가 필요합니다 (SmartThings Hub v3+, Apple TV 4K, Google Nest Hub 2nd gen 등).

## 1단계: PCB 변형 선택

현재 4종: **base / _y / _v2 / _h2** (구 `_v`, `_h` 는 삭제됨).

| 변형 | 형태 | 특징 | 추천 사용처 |
|------|------|------|-----|
| **base** | USB-C 직결 | 배터리 없음, 가장 단순 | 개발/데모, 책상용 |
| **_y** | **Jung(융) 스위치 스타일** | 벽 매입 스위치 폼팩터 | 벽 고정형 리모컨 |
| **_v2** | sandwich 분리형 | 배터리, EC05 + STOP, **진동 wake** | 양산형 (추천) |
| **_h2** | sandwich 통합형 | 배터리, EC11 SMD, **진동 wake** | 양산형 컴팩트 (추천) |

> ℹ️ `_y`(Jung 스위치 스타일)의 상세 BOM·배터리/로터리 구성은 `kicad/` 의
> 해당 변형 파일 기준 — 본 표는 폼팩터만 표기.

발주 시: `kicad/gerber/<variant>.zip` 파일을 PCB 제조사에 업로드.

---

## 2단계: 준비물

| 부품 | y | y2 | v2 | v3 | v4 | h2 | h3 | h4 | 규격 / 비고 |
|-------|------|------|------|------|------|------|------|------|------------|
| ESP32-C6 or ESP32-H2 | GNPE-C6 | XIAO-C6 | GNPE-C6 | H2 | XIAO-C6 | GNPE-C6 | H2 | XIAO-C6 | GNPE ESP32-C6-0.42, XIAO ESP32-C6, ESP32-H2 SuperMini |
| CC1101 모듈 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 433/447 MHz, 안테나 분리형 추천 |
| PCF8574 or PCF8575 IC | 8574 | 8574 | 8574 | 8574 | 8574 | 8574 | **8575** | **8575** | SOIC-16W, I2C GPIO 확장. **좌/우 버튼 추가 시 → PCF8575**(SSOP-24, 핀호환)로 교체 |
| EC11 or EC05 로터리 엔코더 | EC11 | EC11 | EC05 | EC05 | EC05 | EC11 | **ANO** | **ANO** | 클릭 버튼 포함, 스위치 대체 가능 |
| 택트 스위치 | 5 | 5 | 5 | 5 | 5 | 5 | 3 | 3 | UP / DOWN / SELECT / PROG / SETUP (+ **LEFT / RIGHT** — PCF8575 시) |
| 3.5mm 스위치 |  |  | 1 | 1 | 1 |  |  |  | STOP(MY) |
| 디커플링 캐패시터 100 nF | 3 | 4 | 3 | 3 | 4 | 3 | 3 | 4 | 1206 SMD, U1/U2/U3 VCC 근처 |
| 전해 캐패시터 10 µF | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 0805 SMD, +3V3 rail 벌크 |
| I2C 풀업 저항 4.7kΩ | 2 | 4 | 2 | 2 | 4 | 2 | 2 | 4 | 0805 SMD, I2C 근처 SDA/SCL 각각. ⚠️**수량은 "버스 1개" 기준** — **I2C 버스가 2개면 2쌍(4개)**. 예: xiao-c6 에서 PCF8574 를 **LP_I2C(GP6/7)** 로 빼면 OLED(GP22/23)와 버스가 분리되므로 **각 버스에 1쌍씩 필수**(OLED 쪽 누락 시 내부 풀업 45kΩ만 남아 400k·100k 모두 동작 불가 → 화면 안 켜짐). [wiring_xiao-c6.md](wiring/wiring_xiao-c6.md) 참조 |
| INT 풀업 저항 10kΩ | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 0805 SMD, INT 근처 |
| 402560 LiPo 700 mAh |  |  | 1 | 1 | 1 | 1 | 1 | 1 | PCM 내장 셀, JST PH 2-pin 커넥터 |
| **[충전]** MCP73831T-2ACI/OT |  |  | 1 |  |  | 1 |  |  | SOT-23-5 충전 IC |
| **[충전]** SS14 |  |  | 1 |  |  | 1 |  |  | SMA 패키지, 1A 40V Schottky 전원경로 |
| **[충전]** 저항 3.3kΩ, 1kΩ |  |  | 각 1 |  |  | 각 1 |  |  | 0805 SMD (PROG / LED 제한) |
| **[충전]** 캐패시터 4.7µF |  |  | 2 |  |  | 2 |  |  | 0805 SMD (입력/출력 디커플링) |
| **[충전]** Red LED |  |  | 1 |  |  | 1 |  |  | 0805 SMD 충전 인디케이터 |
| **[충전확인]** 저항 100kΩ |  |  |  | 4 | 4 |  | 4 | 4 | 0805 SMD, CHG_STAT, BAT_ADC |
| SGDBM + SGDBF BTB 터미널 |  |  | 각 2 |  |  | 각 2 |  |  | 2×5 0.8mm pitch |
| 5핀,6핀 커낵터 |  |  |  | 각 1 | 각 1 |  |  |  | 1.24mm pitch, H : 4mm (HEADER 포함) |
| C08 or C02 진동 센서 |  |  | C02 | C02 | C02 | C02 | C02 | C02 | 1210 or 0805 SMD ball type |

---

## 2단계: CC1101 배선

> ⚠️ **핀맵 단일 진실원천은 `main/boards/<board>.h` + `README.md` 다.**
> 아래는 **기본 보드 GNPE ESP32-C6-0.42**(`boards/gnpe-c6.h`) 기준이며, 검증 핀은
> **SCK=IO6, MOSI=IO7, MISO=IO2, CSN=IO4, GDO0=IO8** 이다(FSPI IO-MUX 네이티브).
> 다른 보드(XIAO ESP32-C6 / ESP32-H2 SuperMini 등)는 핀·디스플레이·I2C 구성이
> 다르므로 [`doc/wiring/`](wiring/) 의 보드별 배선도를 따를 것. 상충 시 보드 헤더/README 우선.

### 보드별 핀 맵핑 (요약)

세 보드는 같은 SoC(C6) 라도 핀이 모두 다르다. 단일 진실원천은 각 `boards/<board>.h`.
(`‡` = OLED·PCF8574 가 **같은 I2C 핀 공유**.)

| 신호 | **GNPE C6-0.42** | **XIAO ESP32-C6** | **ESP32-H2 SuperMini** |
|---|---|---|---|
| CC1101 SCK  | IO6 | D8 (GPIO19) | GP4 |
| CC1101 MISO | IO2 | D9 (GPIO20) | GP0 |
| CC1101 MOSI | IO7 | D10 (GPIO18) | GP5 |
| CC1101 CSN  | IO4 | D3 (GPIO21) | GP1 |
| CC1101 GDO0 | IO8 | D6 (GPIO16) | GP10 |
| OLED SDA | IO1 | D4 (GPIO22) | GP13 ‡ |
| OLED SCL | IO0 | D5 (GPIO23) | GP14 ‡ |
| PCF8574 SDA | IO19 (비트뱅) | LP 6 ↔ 공유 22 (자동폴백) | GP13 ‡ (공유) |
| PCF8574 SCL | IO18 (비트뱅) | LP 7 ↔ 공유 23 (자동폴백) | GP14 ‡ (공유) |
| PCF8574 ~INT | IO17 | D2 (GPIO2) | GP11 |
| CHG_STAT | IO3 (active-LOW) | D7 (GPIO17, active-HIGH) | GP12 (active-HIGH) |
| BAT ADC | — | D1 (GPIO1) | GP3 |
| VIBE | IO16 | D0 (GPIO0) | GP2 |

| 항목 | GNPE C6-0.42 | XIAO ESP32-C6 | ESP32-H2 SuperMini |
|---|---|---|---|
| OLED 규격 | 72×40 (180° 회전) | 128×64 (정방향) | 72×40 (외부 모듈, 교체 가능) |
| I2C 버스 | 분리 (OLED HW + PCF 비트뱅) | LP_I2C(6/7)→공유(22/23) **자동폴백** | 공유 (OLED+PCF 한 HW I2C) |
| 로터리 | EC11 (full-step) | EC05 (half-step) | EC11 (기본) |
| 충전 | MCP73831(외부) 600mAh/300mA | 온보드 SGM40567 500mAh/120mA | 온보드 TP4054 400mAh/100mA |
| Matter PID | `0x8000` | `0x8003` | `0x8001` |
| 상태 | ✅ 검증 | ✅ 검증 | ✅ 검증 |

> 핀은 `boards/<board>.h` 의 `BOARD_PIN_*`/`BOARD_OLED_*` 가 기준. 보드별 상세 배선도는
> [`doc/wiring/`](wiring/) 참고. **아래 3~5단계의 상세 배선/이미지는 GNPE 기준**이다.

### 보드 GPIO 핀맵 — GNPE ESP32-C6-0.42

![GNPE ESP32-C6-0.42 핀맵](<./esp32/GNPE/ESP32-C6-0.42/GNPE_ESP32-C6-0.42_MAPPING.png>)

### CC1101 모듈 핀맵 — E07-M1101D-SMA

![E07-M1101D-SMA 핀맵](<./CC1101/E07 (M1101D-SMA)/M07-M1101D_MAPPING.jpg>)

| CC1101 핀 | 신호명 | ESP32-C6 GPIO | 비고 |
|---|---|---|---|
| Pin 1 GND  | GND      | GND  | |
| Pin 2 VCC  | +3.3V    | 3V3  | ⚠ 반드시 3.3V! |
| Pin 3 GDO0 | TX Data  | IO8  | RMT 비동기 TX 출력 |
| Pin 4 CSN  | SPI CS   | IO4  | |
| Pin 5 SCK  | SPI CLK  | **IO6** | FSPICLK (IO-MUX) |
| Pin 6 MOSI | SPI MOSI | **IO7** | FSPID (IO-MUX) |
| Pin 7 MISO | SPI MISO | **IO2** | FSPIQ (IO-MUX) |
| Pin 8 GDO2 | NC       | —    | 연결 불필요 |

⚠ **CC1101은 3.3V 전용입니다. 5V 연결 시 즉시 파손됩니다.**

> 📄 CC1101 모듈 상세: [`E07-M1101D-SMA Usermanual EN v1.30`](<./CC1101/E07 (M1101D-SMA)/E07-M1101D-SMA_Usermanual_EN_v1.30.pdf>)
> 보드 상세: [`0.42-ESP32C6`](<./esp32/GNPE/ESP32-C6-0.42/0.42-ESP32C6.pdf>)

---

## 3단계: 버튼/센서 배선 (v2.0 — 모두 PCF8574 이관)

```
컴포넌트         연결                                     비고
──────────────  ──────────────────────────────────────  ───────────
SW1 UP        → PCF8574 P4 (Pin 9)                      ★ v2.0 이관
SW2 DOWN      → PCF8574 P5 (Pin 10)                     ★ v2.0 이관
SW3 SELECT    → PCF8574 P6 (Pin 11)                     ★ v2.0 이관
SW4 PROG      → PCF8574 P7 (Pin 12)                     ★ v2.0 이관 (Somfy PROG)
SW6 SETUP     → PCF8574 P3 (Pin 7)                      ★ v2.0 이관 (짧게 = 설정 메뉴, v3.9)
EC11 ROT_CLICK→ PCF8574 P2                               STOP/MY 커맨드
EC11 ROT_LONG → PCF8574 P2 (2초)                          주파수 편집 모드
VS1 VIBRATION → ESP32 IO16 직결                          wake source
MCP73831 STAT → ESP32 IO3 직결                           wake source
PCF8574 ~INT  → ESP32 IO17 직결 (10kΩ pull-up)           모든 P 변화 wake
```

---

## 4단계: PCF8574 배선 (v2.0 — bit-bang I2C, IO18/IO19)

```
PCF8574 핀    연결 대상             비고
──────────    ──────────────────    ─────────────────────────────────
Pin 1  A0   → GND                  I2C 주소 bit0 = 0
Pin 2  A1   → GND                  I2C 주소 bit1 = 0
Pin 3  A2   → GND                  addr 0x20
Pin 4  P0   → EC11 A상 (ROT_A)
Pin 5  P1   → EC11 B상 (ROT_B)
Pin 6  P2   → EC11 BTN (ROT_BTN)
Pin 7  P3   → SW6 SETUP 버튼       ★ v2.0 변경 (구: CHG_STAT)
Pin 8  GND  → GND
Pin 9  P4   → SW1 BTN_UP           ★ v2.0 변경 (구: VIBRATION)
Pin 10 P5   → SW2 BTN_DOWN         ★ v2.0 변경 (구: SETUP)
Pin 11 P6   → SW3 BTN_SELECT       ★ v2.0 신규
Pin 12 P7   → SW4 BTN_PROG         ★ v2.0 신규
Pin 13 ~INT → IO17 (모든 P 변화 wake) ★ v2.0 신규, 10kΩ pull-up 필수
Pin 14 SCL  → IO18 (bit-bang)      ★ v2.0 변경 (구: LP_GPIO7 IO7)
Pin 15 SDA  → IO19 (bit-bang)      ★ v2.0 변경 (구: LP_GPIO6 IO6)
Pin 16 VCC  → +3.3V
```

> **외부 4.7kΩ pull-up 저항 필수** (SDA/SCL 각 1개씩 to +3.3V).
> ESP32-C6 HP_I2C0 는 OLED 전용, LP_I2C0 는 LP_GPIO6/7 고정 — 따라서 IO18/IO19 는
> 소프트웨어 bit-bang I2C 로 구동 (50kHz, ~10ms 폴링).

**EC11 로터리 엔코더 연결:**
```
EC11 핀    연결
────────   ──────────────────────
A상        PCF8574 P0 (Pin4)
COM/GND    GND
B상        PCF8574 P1 (Pin5)
SW (버튼)  PCF8574 P2 (Pin6)
SWC (COM)  GND
```

### (선택) 좌/우 버튼 — PCF8575 (16핀)

기본 5버튼(UP/DOWN/SELECT/PROG/SETUP) + 로터리(A/B/BTN)가 PCF8574 의 **P0~P7 을 모두**
점유한다. **좌/우 버튼을 추가**하려면 여유 핀이 없으므로 **PCF8574 의 16비트 핀호환
형제 PCF8575(16핀)** 로 교체한다 — 주소(`0x20`) · `~INT` · SDA/SCL · 풀업 · **P0~P7 매핑이
전부 동일**하고 칩만 바꾸면 된다. 추가된 상위 비트 중 2개만 쓴다:

```
PCF8575 추가 핀   연결 대상        비고
─────────────    ────────────     ──────────────────
P10 (bit 8)    → 좌(LEFT) 버튼    active-LOW
P11 (bit 9)    → 우(RIGHT) 버튼   active-LOW
P12~P17        → (미사용)         여유
```

**빌드 플래그**: `boards/<board>.h` 에 `#define BOARD_HAS_LR_BUTTONS 1` → 펌웨어가
PCF8575 로 인식해 **2바이트 read**(P0~7 + P10~17). `0`(기본)이면 PCF8574(1바이트 read).

> **좌/우 버튼 동작** (자세히는 10단계 표):
> - **메인 화면**: 블라인드 선택 **이전 / 다음** (SELECT 순환의 좌우 버전).
> - **시간/날짜 편집**: 자리(필드) 이동.
> - **주파수 편집**: 디지트 **커서 좌/우** 이동(0.1↔0.01 자리) — UP/DOWN(또는 로터리)이 그 자리 값 ±.

---

## 5단계: 빌드 환경 설정

### 5-1. 시스템 환경 변수 설정 (최초 1회)

빌드 스크립트(`build.ps1`)·`CMakeLists.txt`·`factory_nvs_gen.py` 는 개인 경로를
**하드코딩하지 않고 시스템 환경 변수에서 읽는다**. 아래 5개를 **시스템(Machine) 범위**
환경 변수로 등록한다(개인 폴더 구조 비노출). 보통 나머지 4개는 `WORKSPACES_PATH` 아래에 둔다.

| 환경 변수 | 가리키는 곳 | 경로 (WORKSPACES_PATH 하위 기준) |
|---|---|---|
| `WORKSPACES_PATH` | 작업공간 루트(아래 항목들의 부모) | (작업공간 루트 — 직접 지정) |
| `RTS_BLINDS_THREAD_PATH` | **이 프로젝트** 루트 | `%WORKSPACES_PATH%\somfy-blinds-things-by-claude` |
| `IDF_PATH` | ESP-IDF v5.4.1 루트 | `%WORKSPACES_PATH%\esp-idf` |
| `ESP_MATTER_PATH` | esp-matter(CHIP SDK 포함) 루트 | `%WORKSPACES_PATH%\esp-matter` |
| `ESP_SSD1306_PATH` | esp-idf-ssd1306(SSD1306 드라이버) 루트 | `%WORKSPACES_PATH%\esp-idf-ssd1306` |

**관리자 PowerShell 에서 1회 등록** (`$ws` 를 실제 작업공간 경로로 바꿀 것):

```powershell
$ws = "C:\path\to\workspaces"   # esp-idf / esp-matter / 이 프로젝트의 공통 부모
[Environment]::SetEnvironmentVariable('WORKSPACES_PATH',        $ws,                                 'Machine')
[Environment]::SetEnvironmentVariable('RTS_BLINDS_THREAD_PATH', "$ws\somfy-blinds-things-by-claude", 'Machine')
[Environment]::SetEnvironmentVariable('IDF_PATH',               "$ws\esp-idf",                       'Machine')
[Environment]::SetEnvironmentVariable('ESP_MATTER_PATH',        "$ws\esp-matter",                    'Machine')
[Environment]::SetEnvironmentVariable('ESP_SSD1306_PATH',       "$ws\esp-idf-ssd1306",               'Machine')
# 등록 후 새 PowerShell 창을 열어야 적용된다. (또는 시스템 속성 → 환경 변수 GUI 로 등록)
```

> ℹ️ 설치 경로가 위 예와 다르면 각 변수를 실제 경로로 지정하면 된다(서로 다른 폴더에
> 흩어져 있어도 무방 — 각자 독립 변수). 값은 백슬래시(`C:\...`)든 forward slash 든 허용
> (`build.ps1`·`CMakeLists.txt` 가 CMake 용으로 자동 정규화).

**등록 확인** (새 셸에서):

```powershell
'WORKSPACES_PATH','RTS_BLINDS_THREAD_PATH','IDF_PATH','ESP_MATTER_PATH','ESP_SSD1306_PATH' |
  ForEach-Object { "{0,-24} = {1}" -f $_, [Environment]::GetEnvironmentVariable($_) }
```

### 5-2. esp-idf-ssd1306 설치

```bash
git clone https://github.com/nopnop2002/esp-idf-ssd1306 ${WORKSPACES_PATH}\esp-idf-ssd1306
```

### 5-3. esp-matter 설치 (최초 1회)

```bash
git clone --recursive https://github.com/espressif/esp-matter ${WORKSPACES_PATH}\esp-matter
cd ${WORKSPACES_PATH}\esp-matter
# Linux/Mac
./install.sh
# Windows: esp-idf 환경에서 bootstrap 수행
```

### 5-4. 빌드 스크립트 동작 (`build.ps1`)

`build.ps1` 은 경로를 **하드코딩하지 않는다** — 위 5-1 의 시스템 환경 변수
(`RTS_BLINDS_THREAD_PATH`/`IDF_PATH`/`ESP_MATTER_PATH`/`ESP_SSD1306_PATH`)에서 읽고,
pigweed·OTA 도구 경로는 `ESP_MATTER_PATH` 에서 파생한다. 추가로:

- **자동 보충**: 시스템 변수 등록 직후의 기존 셸이면 프로세스에 값이 없을 수 있어
  Machine 스코프에서 자동 로드한다(없으면 `[env] 필수 시스템 환경변수 … 미설정` 에러로 중단).
- **forward slash 정규화**: 환경 변수가 `C:\...`(백슬래시)여도 CMake 용으로 자동 변환.

따라서 5-1 만 끝내면 `build.ps1` 내부를 손댈 필요가 없다.

---

## 6단계: 빌드 & 플래시

> ⚠ **반드시 PowerShell 사용** — Git Bash 는 MSYSTEM 환경변수 충돌로 빌드 실패.
> 사전에 **[§5-1 시스템 환경 변수](#5-1-시스템-환경-변수-설정-최초-1회)** 등록 필수.

모든 빌드/플래시/OTA 는 프로젝트 루트의 **`build.ps1`** 한 스크립트로 한다(보드 선택·로그·
OTA 까지 일원화). 산출물·로그는 보드별로 분리된다.

### 6-1. 보드 선택 (`-Board`)

| `-Board` | 보드 | 빌드 디렉토리 / sdkconfig | 상태 |
|---|---|---|---|
| `gnpe-c6` (기본) | GNPE ESP32-C6-0.42 | `build/` · `sdkconfig` | ✅ 검증 |
| `xiao-c6` | Seeed XIAO ESP32-C6 | `build-xiao-c6/` · `sdkconfig.xiao-c6` | ✅ 검증 |
| `esp32-h2` | ESP32-H2 SuperMini | `build-esp32-h2/` · `sdkconfig.esp32-h2` | ✅ 검증 |

보드마다 **별도 빌드 디렉토리·sdkconfig** 라 전환 시 전체 재컴파일이 불필요하다.
`-Board` 생략 시 기본 `gnpe-c6`.

### 6-2. 표준 작업 흐름

```powershell
# (최초 1회·보드별) IDF target 설정 — 검증 sdkconfig 를 .verified.bak 로 자동 백업
.\build.ps1 -Board gnpe-c6 -Action set-target

# 빌드 → 산출물 build/somfy_blinds.bin
.\build.ps1 -Board gnpe-c6 -Action build

# 플래시 (COM 포트는 6-4 로 확인)
.\build.ps1 -Board gnpe-c6 -Action flash -Port COM3

# 시리얼 모니터 (Ctrl+] 로 종료)
.\build.ps1 -Board gnpe-c6 -Action monitor -Port COM3
```

> XIAO/H2 는 `-Board xiao-c6` / `-Board esp32-h2` 로 바꾸기만 하면 된다(나머지 동일).

### 6-3. 전체 액션 (`-Action`)

| `-Action` | 동작 |
|---|---|
| `build` | 빌드 (산출물 `<builddir>/somfy_blinds.bin`) |
| `flash` | 빌드 후 플래시 (`-Port` 필요) |
| `monitor` | 시리얼 모니터 (네이티브 USB 리셋·재열거 처리, **Ctrl+]** 종료) |
| `erase` | **전체 flash 삭제** (⚠ `rollcode` 파티션까지 → 블라인드 **재-PROG** 필요) |
| `clean` | 빌드 클린 (`fullclean`) |
| `menuconfig` | Kconfig 편집기 |
| `set-target` | IDF target 설정 (검증 sdkconfig 를 `.verified.bak` 로 백업) |
| `restore-sdkconfig` | set-target 후 링크 깨지면 검증 sdkconfig 복원 |
| `ota-image` | `.bin` → 보드별 Matter OTA `.ota` 생성 ([§6-7](#6-7-ota-무선-업데이트)) |

> 모든 출력은 **`logs/<board>-<action>.log`** 로 자동 기록(매 실행 덮어쓰기).
> ⚠ `set-target` 은 sdkconfig 를 defaults 에서 재생성 → WindowCovering 클러스터 등 구성
> 누락으로 **링크 실패**할 수 있다. 그럴 땐 `-Action restore-sdkconfig` 로 복원.

### 6-4. COM 포트 찾기

```powershell
# 연결된 시리얼 포트 나열
Get-PnpDevice -Class Ports -Status OK | Select-Object FriendlyName, InstanceId
# 또는 (간단)
[System.IO.Ports.SerialPort]::GetPortNames()
```

USB 연결 전/후로 비교해 새로 뜨는 포트가 대상(예: `COM6`). XIAO/H2 는 네이티브
USB(USB-Serial-JTAG) 라 `flash`/`monitor` 가 리셋·재열거를 자동 처리한다.

### 6-5. VSCode 태스크

`Ctrl+Shift+B` → 태스크 선택: **Build / Flash / Monitor / Clean** (각 idf.py 대응).

### 6-6. esptool 직접 (참고)

> 💡 **`build.ps1 -Action build` 출력 끝에 정확한 esptool 명령이 그대로 찍힌다 — 이걸
> 복사하는 게 가장 안전하다.** 아래는 GNPE 수동 예(보드별 `--chip`=`esp32c6`/`esp32h2`,
> 빌드 디렉토리=`build` 또는 `build-<board>`):

```powershell
python -m esptool --chip esp32c6 -b 460800 `
  --before default_reset --after hard_reset `
  write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m `
  0x0      build\bootloader\bootloader.bin `
  0xc000   build\partition_table\partition-table.bin `
  0x1d000  build\ota_data_initial.bin `
  0x20000  build\somfy_blinds.bin
```

> ⚠ 오프셋은 **이 프로젝트 파티션 기준**(`partitions.csv`): partition-table `0xc000`,
> otadata `0x1d000`, 앱(`ota_0`) `0x20000`. ESP-IDF 기본값(0x8000/0xf000)과 **다르니**
> 반드시 build 출력 명령을 따를 것.

### 6-7. OTA 무선 업데이트

빌드 후 **무선 배포**는 `-Action ota-image` 로 보드별 `.ota` 를 만들어 Provider/허브로 보낸다.
버전 증가·HW 변형 태그(`<pcf>.<enc>.<oled>`)·배포 절차는 **[`doc/OTA.md`](OTA.md)** 참고.

```powershell
.\build.ps1 -Board gnpe-c6 -Action ota-image   # → dist/somfy_blinds_gnpe-c6_..._v0035.ota
```

### 6-8. 변형 선택 빌드 (OTA 처럼 PCF / 로터리 / OLED)

보드 헤더를 **안 고치고도** 한 보드에서 PCF·로터리·OLED 변형을 빌드 시 고를 수 있다
(`board_select.h` 가 빌드타임 `-D` 오버라이드를 적용). 안 주면 보드 기본값(바이너리 동일).

| 옵션 | 값 | 덮는 매크로 / 효과 |
|---|---|---|
| `-Pcf` | `8574` / `8575` | `BOARD_HAS_LR_BUTTONS` — `8575`=좌/우 버튼 추가 |
| `-Rotary` | `ec11` / `ec05` | `BOARD_ROT_HALF_STEP` — `ec05`=half-step 디코더 |
| `-Oled` | `72x40` / `128x64` / `64x128` | `BOARD_OLED_WIDTH/HEIGHT/FIXUP` — 해상도→렌더러 |
| `-Rotate` | `0` / `180` / `m0` / `m180` | `BOARD_OLED_ROTATE_180` + `BOARD_OLED_FLIP_X` — OLED 회전·좌우반전 (`m` 접두=좌우반전, 아래 표) |
| `-Freq` | `447.70` / `447.72` (미지정=**447.70**) | `BOARD_DEFAULT_FREQ_MHZ` — 기본 송신 주파수 register. 제작보드(E07-400MM10S)=447.70 · 테스트보드(E07-M1101D-SMA)=447.72 (모듈 크리스털 오차 차이) |

**`-Rotate` 방향 값** (`m`=mirror 접두 = 좌우(가로)반전 추가; 회전과 독립 조합):

| 값 | 표시 방향 | 변환 |
|---|---|---|
| `0` | 정방향 | 없음 |
| `180` | 180° 회전 | 좌우+상하 |
| `m0` | **좌우(가로) 반전** | 열 역순 |
| `m180` | 상하(세로) 반전 | `m0`+180° |

```powershell
# GNPE 보드에 좌/우 버튼(PCF8575) + EC05 로 빌드
.\build.ps1 -Board gnpe-c6 -Action build -Pcf 8575 -Rotary ec05

# XIAO 를 EC11 로 (기본 EC05 덮어쓰기)
.\build.ps1 -Board xiao-c6 -Action build -Rotary ec11

# OLED 좌우(가로) 반전 — 거울처럼 뒤집혀 장착된 패널 보정
.\build.ps1 -Board xiao-c6 -Action build -Rotate m0

# ESP32-H2 시제품 기판: OLED 가 180° 장착이라 -Rotate 180 필수 (안 주면 화면 상하 뒤집힘)
.\build.ps1 -Board esp32-h2 -Action build -Rotate 180 -Freq 447.72
```

> 🏷️ **같은 변형 옵션을 `ota-image` 에도 동일하게** 줘야 `.ota` 태그가 빌드와 일치한다:
> `.\build.ps1 -Board gnpe-c6 -Action ota-image -Pcf 8575 -Rotary ec05` → `..._8575_ec05_..._v0035.ota`.

> ⚠ **`-Oled` 해상도만 주의**: 렌더러·패널크기(`ssd1306_init`)는 `-D` 로 바뀌지만, 물리
> **컬럼 오프셋**은 SSD1306 라이브러리 Kconfig(`CONFIG_OFFSETX`, sdkconfig)라 따로
> 맞춰야 정상 표시된다(72x40=28 / 128x64·64x128=0 — build 시 경고 출력).
> PCF·로터리·회전(`-Rotate`)은 sdkconfig 무관, 완전 자동.

### 6-9. 시리얼 콘솔 (진단 / 자동 RF 테스트)

USB Serial JTAG 콘솔에 명령을 보내 **버튼 없이** RF 송신·블라인드 선택·주파수 설정·재부팅을
무인 제어한다 — PC 가 COM 포트로 기기를 구동해 RF 를 쏘고 SDR(`somfy_cli`)로 캡처·분석하는
자동 루프에 쓴다(`read_serial.py` 패턴, 하네스는 `sim/tools/`).

| 명령 | 동작 |
|---|---|
| `tx up\|down\|updown\|myup\|mydown\|my\|prog [hold_ms]` | Somfy RF 송신(session-gate 우회 → combo 직접). `tx up`=cmd2 · `tx updown`=cmd6 |
| `sel <0..N \| N=ALL>` | 블라인드 선택 (N=`BLIND_MAX_COUNT`; H2 `3`·C6 `8`=ALL) |
| `cyc <-1 \| 1>` | 블라인드 선택 순환(`_blind_cycle` — PCF8575 LEFT/RIGHT 의 콘솔 버전) |
| `freq [idx mhz]` | 주파수 조회 / 설정(+NVS 저장, 447.20~447.79 클램프) |
| `reboot` | 재부팅(`esp_restart`) |

> ★ **USB Serial JTAG 를 primary 콘솔**로 둬야 명령 입력이 먹는다(UART default 면 COM 에
> stdin 이 없어 write 가 hang). pyserial 로 **포트를 여러 번 여닫으면 DTR 토글로 리셋**되니
> 한 세션(단일 open) 안에서 연속 전송할 것.

---

## 7단계: Thread Border Router 준비 (v3.0+)

Matter over Thread 는 **Thread Border Router** 가 mesh ↔ Internet 게이트웨이 역할을 합니다.
아래 중 **하나 이상의 장비** 가 Wi-Fi 라우터와 같은 네트워크에 있어야 합니다:

| 장비 | 비고 |
|------|------|
| **SmartThings Hub v3 / Aeotec Smart Home Hub** | 가장 추천 — SmartThings 앱 자동 인식 |
| **SmartThings Station** | 신형, USB-C 전원 |
| Apple TV 4K (2021+) / HomePod mini | Apple Home 사용 시 |
| Google Nest Hub 2nd gen / Nest Hub Max | Google Home 사용 시 |
| Amazon Echo 4th gen / Echo Hub | Alexa 사용 시 |
| OpenThread Border Router (RPi + nRF52840 dongle) | DIY |

> ℹ️ "Thread Border Router 필요" 는 **물리 PCB 배선이 아니라 네트워크 게이트웨이 장비** 의미입니다.
> ESP32-C6 자체가 SED (Sleepy End Device) 로 동작하며 border router 의 Thread mesh 에 join 합니다.

---

## 8단계: Matter over Thread 페어링 (SmartThings)

### 최초 페어링 (v3.5+ 설정 메뉴 사용)

1. ESP32 전원 인가 — Thread network credential 이 없으면 **BLE advertising** 시작 (자동)
2. **SETUP 버튼 (SW6) 짧게 클릭** → 설정 메뉴 진입 (`> Cancel / Freq Edit / Time Set / Matter Pair / Thread Rst / FW Update`). **ESP32-H2 는 메모리 절약으로 `Time Set`·`FW Update` 가 없는 4항목**(Cancel / Freq Edit / Matter Pair / Thread Rst).
3. 로터리 CW/CCW 로 커서를 `> Matter Pair` 까지 이동
4. **SETUP 버튼을 짧게 한 번 클릭** → Matter Pair 화면 진입. ⚠ SELECT 는 설정 모드에서 무시됨 — 메뉴 진입·항목 활성화는 모두 **SETUP 짧게**
5. OLED 에 **기기 고유 8자리 페어링 코드 + QR** 표시 — discriminator/passcode 를 **eFuse MAC 에서 산출**하므로 **기기마다 다르다**(`efuse_commissionable.cpp`, fctry 플래시 불필요). **128×64 / 64×128 패널은 좌측 PIN(`XXXX-XXX-XXXX`) + 우측 QR 병기**, 72×40 은 PIN 만 표시.
6. **SmartThings 앱** → **+** → **디바이스** → **Matter 디바이스 추가** → **OLED 의 QR 스캔**(또는 화면의 수동 코드 입력). 코드가 기기마다 고유라 **여러 대를 동시에 페어링해도 충돌 없음**.
7. SmartThings 가 BLE 로 ESP32 연결 → **Thread network credential 전송** → 802.15.4 mesh 가입
8. Matter Fabric join → 블라인드 엔드포인트 자동 등록 (C6 = 8개 EP1~EP8, **ESP32-H2 = 3개** EP1~EP3 — H2 는 메모리 한계로 축소)

> 🔹 **ESP32-H2(composed) — SmartThings custom 드라이버 필요**: H2 는 RAM 절약을 위해
> composed 구조(root 직속 WindowCovering)라, SmartThings **스톡 드라이버로는 블라인드가 1개만**
> 보인다. `smartthings-driver/` custom Edge driver 를 설치해야 **1카드 3-component**(블라인드 3개)로
> 노출된다. 설치 절차(`edge:drivers:package`→`channels:create`→`assign`→`enroll`→`install`)는
> 루트 [`README.md` §보드별 펌웨어 기능 차이](../README.md#esp32-h2--smartthings-custom-edge-driver-설치)
> 또는 [`smartthings-driver/README.md`](../smartthings-driver/README.md) 참고. (C6 는 Bridge 라 스톡
> 드라이버로 자동 노출 — 설치 불필요. Apple/Google Home 은 H2 도 추가 작업 없이 3개 노출.)

> 페어링 화면 나가기: **로터리 클릭(STOP) 짧게 = 메뉴 / 2초 = 메인**. (Matter Pair 에서 SETUP 짧게 = 페어링 준비 READY 확정)

### Thread credential 재설정 (v3.1 신규)

설정 메뉴에서 `> Thread Rst` 선택 → Yes 확인 → NVS 의 Thread credential 삭제 후 재부팅 → 다시 BLE advertising 시작.

### 재페어링 / 전체 NVS 초기화

```powershell
# 전체 flash 삭제 후 재플래시 (Port 는 보드별: COM3 등)
.\build.ps1 -Board gnpe-c6 -Action erase -Port COM3
.\build.ps1 -Board gnpe-c6 -Action flash -Port COM3
```

> ⚠️ `erase` 는 **전체 flash 삭제** → `rollcode` 파티션(롤링코드 영속)까지
> 지워져 블라인드 **재-PROG** 가 필요하다. 단순 네트워크 초기화는 위 `Thread Rst`
> (Matter factory reset, 기본 NVS 만 삭제 → 롤링코드는 `rollcode` 파티션에 보존)
> 를 쓰는 게 낫다.
> 블라인드 **주소**는 eFuse MAC 산출이라 erase 후에도 동일(기기 고유) — 자세한
> 내용은 루트 `README.md`의 "기기 고유 ID & 롤링코드 영속" 참고.

---

## 9단계: SmartThings 앱에서 블라인드 제어

| SmartThings 명령 | Somfy RTS 커맨드 | 동작 |
|-----------------|-----------------|------|
| Open (100%) | UP (0x2) | 블라인드 완전히 올리기 |
| Close (0%) | DOWN (0x4) | 블라인드 완전히 내리기 |
| Pause / Stop | MY (0x1) | 현재 위치에서 정지 |
| Lift % 설정 | UP/DOWN | 직접 위치 제어 (Somfy는 절대위치 미지원) |
| 루틴 설정 | SmartThings 자동화 | 시간/센서 트리거 → 각 엔드포인트 개별 설정 가능 |

---

## 10단계: 물리 버튼 / 로터리 / 진동 사용법

| 조작 | 동작 |
|------|------|
| UP 버튼 (SW1) | 선택된 블라인드 올리기 (누르는 동안 반복 전송) |
| DOWN 버튼 (SW2) | 선택된 블라인드 내리기 |
| **SELECT 버튼 (SW3)** | **블라인드 선택 순환** (C6 1→…→5→ALL / **H2 1→2→3→ALL**) / 주파수 편집 종료 |
| **LEFT / RIGHT 버튼** (PCF8575, 선택) | 메인: 블라인드 **이전 / 다음** 선택 · 시간/날짜 편집: 자리 이동 · 주파수 편집: 디지트 커서 좌/우 (UP/DOWN=그 자리 값 ±) |
| PROG 버튼 (SW4) 클릭 | Somfy PROG 커맨드 |
| PROG 버튼 (SW4) 2초 롱프레스 | Somfy 리모컨 PROG 명령 (블라인드 등록) |
| **SETUP 버튼 (SW6) 짧게 클릭** (메인 화면, v3.9) | **설정 메뉴 진입** (`> Cancel / Freq Edit / Time Set / Matter Pair / Thread Rst / FW Update`) |
| **SETUP 버튼 짧게 클릭 (설정 메뉴 안에서)** | 커서 위치의 항목 활성화 (예: Matter Pair 시작). SELECT 는 설정 모드에서 무시됨 — 항목 활성화는 **반드시 SETUP 짧게** |
| **SETUP 짧게 또는 2초 (Freq/Time 편집 중)** | 변경 **저장** 후 복귀 (v3.9: 짧게도 저장) |
| **로터리 클릭(STOP) (편집/페어링/리셋 화면)** | 변경 **취소**·화면 종료 → 짧게=메뉴 / 길게=메인 (v3.9) |
| SETUP 버튼 2초 롱프레스 (Thread Rst 확인) | Thread credential 삭제 + 재커미셔닝 |
| 보조 STOP 버튼 (SW5, `_v2`만) | ROT_BTN과 wired-OR (로터리 클릭과 동일 동작) |
| 로터리 CW 회전 | 틸팅 UP (틸트 모션 — 3장 블라인드 슬랫 상하 모션, v3.6) (주파수 편집 중: +0.01 MHz) |
| 로터리 CCW 회전 | 틸팅 DOWN (주파수 편집 중: −0.01 MHz) |
| **로터리 클릭** | **STOP / MY 커맨드** (정지) |
| **모션 중 SELECT (v3.6)** | **모션 중단 + 블라인드 선택 순환 동시 실행** |
| **진동 감지 (`_v2`/`_h2`)** | **흔들면 sleep에서 wake** (0.5초 디바운스) |
| 디바이스 거꾸로 들기 | OLED 화면이 180° 회전 모드로 표시되어 정상 가독 |

> **2026-04-26 변경**: 기존 SW3 = STOP은 SELECT로, STOP은 로터리 클릭으로 이동.

---

## 11단계: 주파수 미세조정 (v3.5 변경 — 설정 메뉴 경유)

1. 블라인드가 반응하지 않을 때 **SETUP 버튼 (SW6) 짧게 클릭** → 설정 메뉴
2. OLED 에 설정 메뉴 표시 (`> Cancel / Freq Edit / Time Set / Matter Pair / Thread Rst / FW Update`)
3. 로터리로 `> Freq Edit` 선택 후 **SETUP 짧게 클릭** → 주파수 편집 화면 진입
4. 로터리 CW/CCW 로 ±0.01 MHz 단위 조정
5. 범위: 447.20 ~ 447.79 MHz (저장값은 **재부팅 후 유지** — 부팅 시 이 범위 밖 손상값만 기본으로 보정)
6. **SETUP 짧게**(또는 2초 길게) → 저장 후 메뉴 복귀 (v3.9). SELECT 는 설정 모드에서 무시됨

> v3.4 이전의 "로터리 2초 롱프레스 = 주파수 편집" 동작은 v3.5 에서 제거되었습니다 (설정 메뉴로 일원화).

---

## 12단계: 배터리 사용 (배터리 변형만)

### USB 충전
1. USB-C 케이블 연결 → MCP73831이 자동으로 300 mA 충전 시작
2. LED1 점등 → 충전 중 표시
3. **OLED 충전 애니메이션**: 매 1분마다 6초간 다이내믹 배터리 차징 영상 표시
   > ⚠ 현 시제품 **H2·xiao 기판**은 BAT_ADC↔CHG_STAT 핀 swap 으로 USB(충전) 감지가 GP12/GPIO17 에서
   > 안 잡혀 충전 애니메이션이 비활성 — 화면 우상단 **`USB`/`BAT`/`LOW` 상태**로 대체된다(`BOARD_BAT_SWAPPED=1`).
4. 풀 충전: 약 2.5시간 (600 mAh, 0.5C)
5. 케이블 분리 시 자동으로 배터리 모드 전환 (D1 Schottky 전원경로)

### 절전 모드 (v3.2 / v3.3)

**전원에 따른 정책 분기 (v3.2):**
- **USB 연결 (충전 중)**: 3 분 무조작 → 화면 보호기 (sleep 진입 안 함)
- **배터리 단독**: 1 분 무조작 → light sleep 진입

**Thread SED + ICD + PM auto-light-sleep (v3.3):**
- `esp_pm_configure()` + `CONFIG_FREERTOS_USE_TICKLESS_IDLE` + `CONFIG_ENABLE_ICD_SERVER` 조합으로
  **라디오를 끄지 않은 채** CPU 만 자동 light sleep.
- Thread mesh 의 ICD poll (SlowPoll 5 s / FastPoll 200 ms) 주기 내 SmartThings 명령 수신 가능 → 즉시 wake + 모션 표시 (v3.6).
- 주변장치 power-down 비활성: I²C/SPI/RMT 상태 보존 (peripheral light-sleep 시 깨어나면 재초기화 불필요).

**Wake 소스:**
- 4 개 버튼 GPIO (PCF8574 ~INT IO17)
- 로터리/충전/진동 변화 (PCF8574 P0~P3 + IO3 CHG_STAT + IO16 VIBE)
- **SmartThings Matter 명령 (v3.6)** — Thread mesh ICD wake 후 해당 모션 즉시 화면 표시

### 진동 wake (`_v2`/`_h2`만)
- 디바이스를 흔들면 VS1 진동 센서 동작 → PCF8574 INT pulse → wake
- 0.5초 디바운스 후 진동으로 분류 (실제 버튼 누름 = >100ms 지속)
- **지속 진동 시** (계속 흔드는 경우): 마지막 진동 후 5초간 sleep 차단

---

## 문제 해결

| 증상 | 원인 | 해결 방법 |
|------|------|---------|
| CC1101 인식 안됨 | 배선 오류 / 5V 연결 / 구 핀 사용 | **GNPE 검증 핀: SCK=IO6, MOSI=IO7, MISO=IO2, CSN=IO4, GDO0=IO8**(`gnpe-c6.h`). 반드시 3.3V. 다른 보드는 해당 배선도 |
| PCF8574 NACK 실패 | SDA/SCL 배선 / pull-up 미장착 | GNPE: **SDA=IO19, SCL=IO18**(bit-bang) + 4.7kΩ pull-up to 3V3 필수. (H2 는 OLED 와 공유 HW I2C — `BOARD_I2C_SHARED`) |
| `CONFLICT! driver_ng vs old driver` abort | legacy/new I2C 동시 링크 | `CONFIG_LEGACY_DRIVER` 비활성화 (모두 new 드라이버) |
| OLED 절반만 표시 / 글자 너무 큼 | (GNPE 72×40) SSD1315 보정 누락 | `BOARD_OLED_FIXUP_72X40=1` + `CONFIG_OFFSETX=28` + multiplex(0xA8 0x27)/COM(0xDA 0x12). **128×64 패널은 보정 끔**(`FIXUP=0`, `OFFSETX=0`) |
| OLED 내용이 한쪽으로 쏠림 | 패널 크기 ≠ `BOARD_OLED_*` | 보드 헤더의 `BOARD_OLED_WIDTH/HEIGHT` 와 `sdkconfig` 의 `CONFIG_OFFSETX` 를 실제 패널에 맞출 것 |
| OLED 상하 반전(뒤집힘) | H2 시제품 패널 180° 장착 + `-Rotate` 미지정(기본 `ROTATE_180=0`) | `-Rotate 180` 로 빌드/플래시 (`.\build.ps1 -Board esp32-h2 -Rotate 180 -Action flash`). 좌우(거울) 반전이면 `-Rotate m0` |
| 로터리 미반응 | PCF8574 I2C 미연결 | GNPE: SDA=IO19, SCL=IO18 bit-bang + 4.7kΩ pull-up 확인 |
| 블라인드 반응 없음 | 주파수 불일치 | 로터리 롱프레스 → 주파수 미세조정 |
| OLED 안 켜짐 | I2C 주소 불일치 | `CFG_OLED_ADDR=0x3C` 확인 |
| Matter 페어링 실패 | Thread Border Router 부재 / 거리 | SmartThings Hub v3+ / Apple TV 4K / Nest Hub 2nd gen 등이 같은 네트워크에 있는지 확인 |
| Matter 페어링 실패 (코드 인식) | NVS 의 이전 fabric 잔존 | 설정 메뉴 → Thread Rst, 또는 `.\build.ps1 -Action erase` |
| SmartThings 명령에 응답 지연 (≤5초) | Thread SED SlowPoll 주기 (5 s) | 정상 동작 — ICD 응답성 (FastPoll 200 ms) 는 active 트랜잭션 중에만 발동 |
| 리셋 직후 시간 초기화 | NVS 시간 영속화 미동작 | v3.6 이상 펌웨어 사용 + border router 통한 SNTP 동기화 1시간 대기 |
| 빌드 실패 (MSYSTEM) | Git Bash 사용 | PowerShell에서 `build.ps1` 실행 |
| 롤링코드 오류 (모터 무응답) | 전체 `erase-flash` 로 `rollcode` 파티션까지 삭제 | 블라인드 재-PROG. (Matter factory reset/OTA 는 `rollcode` 파티션 보존 → 재등록 불필요) |
| 여러 기기가 같은 블라인드 충돌 | (구 펌웨어) 하드코딩 주소 동일 | 현 펌웨어는 eFuse MAC 산출로 기기별 고유 — 각 기기 재플래시 후 PROG 재등록 |
| 펌웨어 업데이트 안 됨 | OTA 이미지 PID/버전 불일치 | 대상 보드 PID + 현재보다 높은 버전으로 `.ota` 생성(`doc/OTA.md`) |
| 바이너리 크기 초과 | 파티션 너무 작음 | `partitions.csv`에서 파티션 크기 확인 |

---

## 참조 링크

| 항목 | URL |
|------|-----|
| ESP-IDF **v5.4.1** | https://docs.espressif.com/projects/esp-idf |
| esp-matter (`7706cfbd` · main) | https://github.com/espressif/esp-matter |
| esp-idf-ssd1306 (`554df45` · master) | https://github.com/nopnop2002/esp-idf-ssd1306 |
| esp_qrcode **0.2.0** (관리 컴포넌트) | https://components.espressif.com/components/espressif/qrcode |
| Somfy RTS 프로토콜 | https://pushstack.wordpress.com/somfy-rts-protocol/ |
| CC1101 모듈 매뉴얼 (E07-M1101D-SMA) | [`doc/CC1101/E07 (M1101D-SMA)/E07-M1101D-SMA_Usermanual_EN_v1.30.pdf`](<./CC1101/E07 (M1101D-SMA)/E07-M1101D-SMA_Usermanual_EN_v1.30.pdf>) |
| CC1101 모듈 핀맵 이미지 | [`doc/CC1101/E07 (M1101D-SMA)/M07-M1101D_MAPPING.jpg`](<./CC1101/E07 (M1101D-SMA)/M07-M1101D_MAPPING.jpg>) |
| PCF8574 데이터시트 | https://www.ti.com/lit/ds/symlink/pcf8574.pdf |
| ESP32-C6-0.42 보드 문서 (GNPE) | [`doc/esp32/GNPE/ESP32-C6-0.42/0.42-ESP32C6.pdf`](<./esp32/GNPE/ESP32-C6-0.42/0.42-ESP32C6.pdf>) |
| ESP32-C6-0.42 핀맵 이미지 | [`doc/esp32/GNPE/ESP32-C6-0.42/GNPE_ESP32-C6-0.42_MAPPING.png`](<./esp32/GNPE/ESP32-C6-0.42/GNPE_ESP32-C6-0.42_MAPPING.png>) |
