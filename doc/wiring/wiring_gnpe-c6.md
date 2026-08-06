# 배선도 — GNPE ESP32-C6-0.42

> **보드 핀맵 단일 진실원천: [`main/boards/gnpe-c6.h`](../../main/boards/gnpe-c6.h)**
> 이 표는 그 헤더에서 파생했다. 헤더가 바뀌면 이 문서도 갱신할 것.
> 상세 다이어그램(전원 정책·절전·UX·PCB 변형 등) 은 [`wiring_gnpe-c6.html`](wiring_gnpe-c6.html).

- **브랜드/제품**: GNPE "ESP32-C6-0.42" (현 검증 보드 · 문서 핀맵 기준)
- **SoC / IDF target**: ESP32-C6 (RISC-V, 802.15.4 + BLE) / `esp32c6`
- **Matter Product ID**: `0x8000` (OTA 보드 매칭, `sdkconfig.defaults.c6_thread`)
- **OLED**: SSD1315 **72×40**, 거꾸로 장착(SW 180° 회전), 컬럼 오프셋 28 —
  **보드 내장 배선(IO0/IO1, 변경 불가)**. 규격은 `boards/gnpe-c6.h` 의 `BOARD_OLED_*`
  (+ `sdkconfig` `CONFIG_OFFSETX=28`).
  - 🔀 **렌더러는 해상도로 자동 선택**(보드에 고정 아님). 72×40 이므로 논리 캔버스
    렌더러가 적용된다. **다른 규격 OLED 를 연결**하려면(I2C 핀이 노출된 변형/외부 모듈)
    `gnpe-c6.h` 의 `BOARD_OLED_*` 만 그 패널 규격으로 바꾸면 된다 — 예: 128×64 로 바꾸면
    풀스크린 네이티브 렌더러(고딕+7세그)가 자동 적용된다(`OFFSET 0`, `FIXUP 0`,
    `sdkconfig` `CONFIG_OFFSETX=0` 동반). 코드 수정 불필요.
- **빌드**: `./build.ps1 -Board gnpe-c6 -Action build` (기본 보드)

## 핀맵 이미지

### GNPE ESP32-C6-0.42 핀맵

![GNPE ESP32-C6-0.42 핀맵](<../esp32/GNPE/ESP32-C6-0.42/GNPE_ESP32-C6-0.42_MAPPING.png>)

### CC1101 모듈 (E07-M1101D-SMA) 핀맵

![E07-M1101D-SMA 핀맵](<../CC1101/E07 (M1101D-SMA)/M07-M1101D_MAPPING.jpg>)

> ⚠️ 핀 번호의 최종 기준은 아래 표(=`boards/gnpe-c6.h`). 이미지는 보드/모듈
> 물리 위치 파악용 참고.

## 핀 매핑 (요약)

| 기능 | 신호 | ESP32-C6 GPIO | 비고 |
|---|---|---|---|
| CC1101 | SCK  | **IO6**  | FSPICLK (IO-MUX) |
| CC1101 | MISO | **IO2**  | FSPIQ (IO-MUX) |
| CC1101 | MOSI | **IO7**  | FSPID (IO-MUX) |
| CC1101 | CS   | **IO4**  | (FSPICS0) active-LOW |
| CC1101 | GD0  | **IO8**  | RMT 비동기 TX 데이터 |
| OLED   | SDA  | **IO1**  | 실크 `SDA-1`, 내장 배선 |
| OLED   | SCL  | **IO0**  | 실크 `SCL-0`, 내장 배선 |
| PCF8574 | SDA | **IO19** | bit-bang I2C, 외부 4.7 kΩ pull-up |
| PCF8574 | SCL | **IO18** | bit-bang I2C, 외부 4.7 kΩ pull-up |
| PCF8574 | ~INT | **IO17** | active-LOW wake, 외부 10 kΩ pull-up |
| 센서   | CHG_STAT | **IO3** | MCP73831 STAT (active-LOW), sleep wake |
| 센서   | VIBE | **IO16** | VS1 진동 스위치, sleep wake |

> ℹ️ CC1101 SPI 핀 단일 진실원천은 `gnpe-c6.h`(위 표: SCK=IO6 / MOSI=IO7 / MISO=IO2 / CS=IO4).
> `wiring_gnpe-c6.html` 다이어그램도 현행 값으로 갱신됨(옛 v2.0 표기는 변경이력 섹션에만 보존).

## CC1101 ↔ ESP32-C6

| CC1101 핀 | 신호 | GPIO | 비고 |
|---|---|---|---|
| Pin1 GND  | GND  | GND  | 공통 GND |
| Pin2 VCC  | +3.3V | 3V3 | ⚠ **3.3V 전용** — 5V 연결 시 파손 |
| Pin3 GDO0 | TX Data | IO8 | RMT 비동기 TX |
| Pin4 CSN  | SPI CS | IO4 | active-LOW |
| Pin5 SCK  | SPI Clock | IO6 | |
| Pin6 MOSI | SPI Data OUT | IO7 | |
| Pin7 MISO | SPI Data IN | IO2 | |
| Pin8 GDO2 | NC | — | 연결 불필요 |

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

## Light-sleep Wake 소스

| GPIO | 소스 | 트리거 |
|---|---|---|
| IO17 | PCF8574 ~INT | P0~P7 중 변화 (모든 버튼/로터리) |
| IO16 | VS1 진동 | 접점 닫힘 (active-LOW) |
| IO3  | MCP73831 STAT | 충전 시작 (open-drain LOW) |

## 조립 주의

- CC1101 은 **3.3V 전용** — 5V 금지.
- 외부 풀업: PCF8574 SDA/SCL 각 **4.7 kΩ**, ~INT **10 kΩ** → +3V3.
- OLED(HP I2C0, IO0/IO1, 내장) ↔ PCF8574(bit-bang, IO18/IO19) — 핀 충돌 없음.
- 각 VCC 핀 근처 **100 nF 디커플링** 권장.
