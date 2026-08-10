# 구현 체크리스트 및 빌드 전 주의사항

## ✅ 구현 완료 항목

| 요구사항 | 파일 | 상태 |
|---------|------|------|
| H/W: ESP32-C6 + CC1101 (보드 교체 가능) | `boards/<board>.h`, `cc1101.c` | ✅ |
| RF 447.72 MHz 2-FSK (한국 베네치아), 0.01 MHz 단위 | `somfy_rts.h`, `blind_manager.c` | ✅ |
| Somfy RTS 447 80비트 프로토콜 (RMT + Manchester) | `somfy_rts.c` | ✅ |
| SmartThings Matter 엔드포인트 (C6 8개 / H2 3개) | `app_main.cpp` + `matter_blinds_shim.cpp` | ✅ |
| NVS 블라인드 저장 (C6 8 / H2 3, `BLIND_MAX_COUNT`) | `blind_manager.c` | ✅ |
| **기기별 고유 블라인드 주소** (eFuse MAC → `F0`+등차+ID, 블록 ALL) | `blind_manager.c` `_derive_addresses()` | ✅ |
| **롤링코드 factory-reset 영속** (채널 + 블록 ALL 전용 파티션) | `blind_manager.c` `rollcode` 파티션(`rolling[]`+`all_rolling[]`) | ✅ |
| 블라인드 1개 또는 **블록별 전체(ALL)** 조작 — 채널↑→블록·ALL 자동↑ | `blind_manager_get_targets()` (`all_blocks[]`) | ✅ |
| UP/DOWN/SELECT/PROG/SETUP 버튼 | `button_handler.c` PCF8574 P3~P7 | ✅ |
| 로터리 엔코더 (PCF8574 P0/P1/P2) — full-step(EC11) / half-step(EC05) 분기 | `button_handler.c` + `BOARD_ROT_HALF_STEP`(EC05=그레이코드 LUT 누산) | ✅ |
| **좌/우 버튼 (선택, PCF8575 16핀)** — 메인=블라인드 선택±, 편집화면=자리이동/디지트 커서 | `BOARD_HAS_LR_BUTTONS` + `button_handler.c`(16비트 read) + `somfy_app.c` | ✅ |
| 로터리 클릭 = STOP/MY 커맨드 | `somfy_app.c` `BTN_EVT_ROT_*` 핸들러 | ✅ |
| 버튼 0.1초~15초 조작 범위 | `somfy_app.c` `_clamp_hold()` | ✅ |
| 주파수 0.01 MHz 미세조정 + 저장 | `somfy_app.c` Freq Edit → `blind_manager_set_freq()` | ✅ |
| SmartThings 앱 개별 루틴 설정 | Matter Window Covering Cluster | ✅ |
| Tilt 슬라이더 7단계 step burst | `app_main.cpp` WC delegate + `somfy_rts_send_steps()` | ✅ |
| Lift/Tilt 미러 companion 억제 (400ms) | `app_main.cpp` WC delegate | ✅ |
| 일시정지(Stop) → MY (auto-stop 500ms 가드) | `app_main.cpp` `HandleStopMotion()` | ✅ |
| 진동 wake (X160, IO16) | `button_handler.c` 진동 ISR + duty-cycle | ✅ |
| 보드별 핀맵 config + 빌드 (`-Board`) | `boards/`, `build.ps1` | ✅ |
| 빌드/플래시 로그 logs/ 자동 기록 | `build.ps1` Tee → `logs/<board>-<action>.log` | ✅ |
| 안전망 (Task WDT + crash breadcrumb) | `somfy_app.c`, `button_handler.c` | ✅ |
| **시리얼 진단 콘솔** (tx/sel/cyc/freq/reboot — 버튼 없이 RF 송신·선택·순환·주파수·재부팅 무인 제어) | `app_main.cpp` esp_console + `somfy_app.c` `somfy_app_console_*` | ✅ |
| OLED 일반 화면 (주파수/블라인드/버튼) | `oled_ui.c` `_render_normal()` | ✅ |
| OLED 애니메이션 20fps | `oled_ui.c` FreeRTOS 태스크 50ms | ✅ |
| 버튼 동작 중 5초 애니메이션 | `oled_ui.c` `_render_action()` | ✅ |
| ~~화면보호기~~ **삭제됨**(2026-07) — 대신 유휴 후 패널 OFF | `somfy_config.h` `CFG_SCREEN_OFF_SEC`(기본 10초) | ✅ |
| 버튼·**진동** 조작 시 즉시 화면 복귀 | `oled_ui_wake()` / `_exit_screensaver()` | ✅ |
| **OLED 비트뱅 I2C**(HW 페리페럴 고착 우회, GPIO 레지스터 직접 접근) | `oled_ui.c` `_bbo_write()` + `BOARD_OLED_BITBANG` | ✅ |
| **dirty-page 전송 감축**(변경된 페이지만 전송, 실측 86% 절감) | `oled_ui.c` `s_shadow`/`_page_dirty()` | ✅ |
| **채널변경 잠금**(RF 버튼·애니메이션 중 SELECT·좌·우 무시) | `somfy_app.c` `_ch_locked()` + `CFG_CH_LOCK_MS` | ✅ |
| **OLED 안정성 계측** `[OLEDMON]`(60초)·`[BBFAIL]` | `oled_ui.c` `g_bbo_tx_cnt`/`g_bbo_fail_cnt` | ✅ |
| 충전률 측정 (5초 주기·8표본) — 직렬화 구멍 수정 후 **재활성** | `somfy_app.c` `TEMP_NO_CHARGE=0` / `_read_bat_mv()` | ✅ |
| OLED 전송 직렬화 — 락을 `_bbo_write()` 로 이동 + **재귀 뮤텍스** | `oled_ui.c` `s_i2c_mutex` · `[OLEDMON] lockTO` | ✅ |
| **OLED 180° 회전 표시** (보드별) | `oled_ui.c` `_fb_flush()` + `BOARD_OLED_ROTATE_180` | ✅ |
| **보드별 디스플레이 규격** (해상도/오프셋/회전/72×40 보정) | `boards/<board>.h` `BOARD_OLED_*` (단일 진실원천) | ✅ |
| **해상도별 렌더러 자동 선택** (보드 무관) | `oled_ui.h` `OLED_RENDER_128X64`/`OLED_RENDER_64X128`/`OLED_RENDER_NATIVE` | ✅ |
| **풀스크린 네이티브 렌더러** (고딕 6×9 폰트 + 7세그 시계) | `oled_ui.c` 128×64 가로 / 64×128 세로 분기 | ✅ |
| **72×40 논리 캔버스 렌더러** (그 외 패널 중앙 배치/블록 스케일) | `oled_ui.c` `_fb_set_pixel()` 폴백 | ✅ |
| 시각 동기화 (Matter SetUTCTime + SNTP 폴백, KST-9) | `somfy_app.c`, Time Sync 클러스터 | ✅ |
| Matter 커미셔닝 (BLE+Thread) | `app_main.cpp` (Matter-over-Thread) | ✅ |
| **펌웨어 업데이트 (Matter OTA over Thread)** | `matter_blinds_shim.cpp` OTA 브리지 + FW Update 메뉴 | ✅ |
| **보드별 OTA 이미지** (브랜드별 PID) | `build.ps1 -Action ota-image` → `dist/*.ota` | ✅ |
| **브랜드별 보드 분리** (gnpe-c6/xiao-c6/esp32-h2) | `boards/<brand>-<soc>.h` | ✅ |
| **XIAO ESP32-C6 핀맵** (LP_I2C MTCK/MTDO, D11 미노출 회피, 온보드 충전) | `boards/xiao-c6.h` (Seeed 핀아웃 검증) | ✅ |
| **ESP32-H2 SuperMini 핀맵** | `boards/esp32-h2.h` (SuperMini 핀아웃 반영) | ✅ 검증 |
| **I2C 버스 공유** (OLED+PCF8574 한 HW I2C — XIAO·H2) | `BOARD_I2C_SHARED` + `button_handler.c` 분기 + `oled_ui_get_i2c_bus()` | ✅ XIAO·H2 검증 |
| **XIAO LP_I2C → 공유 HW I2C 자동 폴백** (부팅 프로브) | `BOARD_I2C_LP_FALLBACK` + `button_handler.c` `_pcf_lp_probe()`/런타임 디스패처 | ✅ 실기 검증(LP 미연결→공유 전환) |
| **ESP32-H2 메모리 최적화** (3채널 · 시각/OTA 메뉴 제외) | `BLIND_MAX_COUNT=3` + `BOARD_DISABLE_TIME`·`BOARD_DISABLE_OTA` | ✅ |
| **페어링 QR 코드** (128×64/64×128 — 좌 PIN + 우 QR 병기) | `oled_ui.c` `_draw_qr_native()` + `espressif/qrcode` | ✅ |
| **Matter Vendor/Product 이름** (`NLB` / `Somfy RTS Thread`) | `CHIPProjectConfig.h` | ✅ |
| SmartThings에서 개별 제어 | Matter 독립 엔드포인트 (C6 = 8 / **H2 = 3**) | ✅ |
| VSCode ESP-IDF v5.4.x 빌드 | `build.ps1`, `.vscode/tasks.json` | ✅ |
| **KiCad 10 4종 변형** (base/_y/_v2/_h2) — 구 _v·_h 삭제 | `kicad/` 폴더 | ✅ |
| **거버 패키지 ZIP + MANIFEST** | `kicad/gerber/` | ✅ |
| **PCB 0 ERROR DRC** | `_electrical_check.py` 0 issues | ✅ |
| 빌드 성공 (경고 없음) | `somfy_blinds.bin` ≈1.7 MB | ✅ |

### 배터리 변형 (`_v2` / `_h2`)

| 요구사항 | 파일 | 상태 |
|---------|------|------|
| 851640 600 mAh LiPo (PCM 내장) + JST PH | `kicad/_add_battery.py` BAT1 | ✅ |
| MCP73831 단일 셀 충전 IC (300 mA) | `kicad/_add_battery.py` U4 | ✅ |
| SS14 Schottky 전원경로 | `kicad/_add_battery.py` D1 | ✅ |
| 1분 무입력 light sleep (~750 µA) | `somfy_app.c` `_enter_sleep()` | ✅ |
| 충전/USB 감지 — GNPE: CHG_STAT(IO3 active-LOW) · XIAO/H2: VBUS 분압(active-HIGH) | `button_handler.c` `btn_handler_is_charging()` (`BOARD_CHG_STAT_ACTIVE_HIGH`) | ✅ |
| 배터리 표시: **정상 배선=실측 %**(BAT 분압 ADC) / **현 시제품 H2·xiao 기판=`USB`/`BAT`/`LOW` 상태**(BAT_ADC↔CHG_STAT 핀 swap → CHG_STAT핀 ADC 불가, `BOARD_BAT_SWAPPED=1`) | `somfy_app.c` `_estimate_battery_percent()` / `usb_pwr`·`bat_low` | ✅ |
| 충전 중 sleep 차단 + 1분 OLED 애니메이션 | `somfy_app.c` + `oled_ui_show_charging()` | ✅ |

### Sandwich 변형 (`_v2` / `_h2`)

| 요구사항 | 파일 | 상태 |
|---------|------|------|
| 2-board 분리 (BOTTOM 메인 + TOP UI) | KiCad PCB 양면 + V-cut | ✅ |
| BTB1-4 board-to-board (10-pin × 2 pair) | `_create_v2_h2.py` BTB symbols | ✅ |
| BREAKAWAY_TAB (V-cut 함께 제조) | PCB 풋프린트 | ✅ |
| **VS1 진동 센서** (JYX-1210-X160, 1210 SMD) | `_add_vibration2.py` VS1 | ✅ |
| **진동 → IO16 GPIO 직결** (light sleep wake) | `boards/gnpe-c6.h` `BOARD_PIN_VIBE=16` | ✅ |
| **진동 wake** (IO16 HIGH-level, duty-cycle 검출) | `button_handler.c` 진동 ISR + 30폴 윈도우 | ✅ |
| **지속 진동 5초 holdoff** | `somfy_app.c` `VIBRATION_HOLD_MS=5000` | ✅ |

---

## ⚠️ 빌드 전 필수 확인사항

### 1. PCF8574 I2C 주소 확인

PCF8574의 A0/A1/A2 핀이 모두 GND에 연결되어 있어야 I2C 주소 `0x20`이 됩니다.

```
PCF8574 핀    연결
──────────    ────
A0 (Pin1)  → GND  (주소 bit0 = 0)
A1 (Pin2)  → GND  (주소 bit1 = 0)
A2 (Pin3)  → GND  (주소 bit2 = 0)
결과 I2C 주소: 0x20
```

다른 주소 필요 시 `button_handler.h`의 `PCF8574_I2C_ADDR` 수정.

### 2. I2C 버스 공유 (OLED + PCF8574)

OLED(SSD1315)와 PCF8574가 동일 I2C 버스(IO18/IO19)를 공유합니다.
`button_handler.c`의 I2C 뮤텍스(`s_i2c_mutex`)가 충돌을 방지합니다.

> `btn_handler_init()`은 반드시 `oled_ui_init()` **이후**에 호출해야 합니다.
> (I2C 버스가 OLED 초기화에서 먼저 열림)

### 3. COM 포트 확인

`build.ps1`의 기본 포트는 `COM3`입니다. 실제 포트로 변경:

```powershell
.\build.ps1 -Action flash -Port COM5   # 장치 관리자에서 확인
```

### 4. PowerShell만 사용 (Git Bash 사용 불가)

```
⚠ Git Bash에서 idf.py 실행 시 MSYSTEM=MINGW64 충돌로 빌드 실패
   반드시 PowerShell 또는 CMD에서 build.ps1 실행
```

### 5. 디스플레이 규격 / 해상도별 렌더러

디스플레이는 **보드별 `BOARD_OLED_*`** 가 단일 진실원천이고, 렌더러는 **해상도로
자동 선택**된다(`oled_ui.h` 의 `OLED_RENDER_*`):

| 해상도 | 렌더러 | 예 |
|---|---|---|
| **128×64 (가로)** | 풀스크린 네이티브 (고딕 6×9 + 7세그) | XIAO 0.96″ SSD1306 |
| **64×128 (세로)** | 풀스크린 네이티브 (세로 적층) | SH1107 계열 세로 패널 |
| **그 외** | 72×40 논리 캔버스 (중앙 배치) | GNPE 0.42″ SSD1315 |

- 프레임버퍼는 **물리 패널 크기**로 잡힌다(72×40=360 B, 128×64=1024 B …).
- GNPE SSD1315(72×40)는 SSD1306(128×64)과 해상도가 달라 **컬럼 오프셋
  `BOARD_OLED_COL_OFFSET=28`**(+ `sdkconfig` `CONFIG_OFFSETX=28`) + 72×40 보정 필요.
- 다른 규격 패널로 바꾸려면 `boards/<board>.h` 의 `BOARD_OLED_*` 만 수정(코드 무수정).

### 6. ESP_MATTER_DEVICE_PATH

`build.ps1`에서 자동 설정됩니다:
```
ESP_MATTER_DEVICE_PATH = esp-matter\device_hal\device\esp32c6_devkit_c
```

---

## 🔧 알려진 제한사항

### Matter 페어링 코드 (기기 고유 — eFuse 산출)
- 128×64 / 64×128 패널 = QR + PIN, 72×40 = PIN(8자리) 텍스트 표시
- discriminator/passcode 를 **eFuse MAC 에서 결정적 산출**(`efuse_commissionable.cpp` custom CommissionableDataProvider) → **기기마다 다른 코드**, fctry 플래시 없이 자동 고유. 여러 대 동시 페어링도 충돌 없음
- (선택) `factory_nvs_gen.py` 로 fctry 에 고정 코드 지정도 가능

### 네트워크 (Matter over Thread)
- v3.0+ WiFi SoftAP 프로비저닝 폐기 → **Matter over Thread (802.15.4)**.
- Thread Border Router 필요(SmartThings Hub v3+, Apple TV 4K, Nest Hub 2nd 등).
- 커미셔닝은 BLE + Thread. OLED 에 8자리 PIN 표시.

### Somfy RTS 주파수
- 기본 register 447.72 MHz(2-FSK) → on-air ~447.678 MHz(정품 ±3 kHz).
- 편집 범위 447.20~447.79 MHz, 0.01 단위(설정 메뉴 → Freq Edit).
- 편집값은 NVS 저장 + **재부팅 후 유지** — 부팅 클램프가 편집 범위 밖 구버전/손상값만 기본으로 보정(`_ensure_blinds`).

### 블라인드 주소 & Rolling Code
- **주소**: ESP32 eFuse 팩토리 MAC 에서 산출(기기별 고유, OTA·reset 불변).
  주소를 바꾼 기기(구 펌웨어 → 신 펌웨어)는 모터에 PROG 재등록 필요.
- **롤링코드 영속**: 전용 `rollcode` 파티션 → Matter factory reset 후에도 보존.
  전체 `erase-flash` 시에만 리셋 → 재페어링(아래) 필요.
- 재페어링 절차: ① 기존 Somfy 리모컨 PROG 6초 → ② ESP32 PROG 단클릭.

### OTA 업데이트 (Matter over Thread)
- 파티션: **듀얼 OTA**(`ota_0`/`ota_1` 각 0x1E0000) + `otadata`. (구 "factory 단일"
  설명은 오래된 것 — 이미 듀얼 OTA.)
- `rollcode` 파티션 추가(0x3E6000, 16KB) — 롤링코드 영속.
- 보드별 PID 로 OTA 이미지 매칭: `build.ps1 -Action ota-image`. 상세: `doc/OTA.md`.

---

## 📐 메모리 현황 (현행)

| 항목 | 값 |
|------|-----|
| 바이너리 크기 | ≈0x1A6800 (1.65 MB) |
| ota_0 / ota_1 | 각 0x1E0000 (1.875 MB) — 12% 여유 |
| otadata | 0x2000 |
| nvs 파티션 | 0xC000 (48 KB) |
| fctry 파티션 | 0x6000 (Matter factory NVS) |
| **rollcode 파티션** | 0x4000 (16 KB, 롤링코드 영속) |
| Flash 총 크기 | 4 MB |

---

## 🗂 소스 파일 구조

```
main/
├── boards/             ← 보드별 핀맵 (board_select.h + <board>.h)
├── app_main.cpp        ← Matter 코어/노드/엔드포인트/WC delegate/OpenThread
├── app_driver.cpp      ← Matter attribute/identify 글루
├── matter_blinds_shim.cpp/h ← matter_blinds_* C API (상태조회/재커미셔닝)
├── somfy_app.c         ← 앱 메인: OLED/버튼/메뉴/시계/절전/안전망
├── cc1101.c/h          ← CC1101 SPI 드라이버 (핀은 boards/ 에서 파생)
├── somfy_rts.c/h       ← Somfy RTS 447 프레임 생성/송신 (RMT)
├── blind_manager.c/h   ← 블라인드 NVS 저장/로드, 선택 관리
├── oled_ui.c/h         ← SSD1306 드라이버 래퍼, 화면 상태 머신
├── button_handler.c/h  ← PCF8574 버튼/로터리 + 진동 ISR
├── thread_provision.c/h ← Thread 네트워크 부착
├── somfy_config.h      ← 설정(BOARD_PIN_* 참조)/타임존/주파수
└── gen_build_epoch.cmake ← 빌드시각 헤더 생성
```
