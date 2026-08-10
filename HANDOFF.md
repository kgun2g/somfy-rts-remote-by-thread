# 인수인계 — 다음 세션에서 이어갈 것

> 새 세션을 열고 **"HANDOFF.md 읽고 이어서 작업해줘"** 라고만 하면 된다.

기준 커밋: `24bbe12` (원격 `github.com/kgun2g/somfy-rts-remote-by-thread`, push 완료)

---

## 최우선 — 충전률 측정 재활성

### 현재 상태
`main/somfy_app.c` 의 **`TEMP_NO_CHARGE = 1`** 로 충전률 측정이 **꺼져 있다.**
이 상태에서만 OLED 가 안정적이다(60초에 전송 3,001건 / 실패 0건).

### 확정된 원인
`_read_bat_mv()` 가 부르는 `adc_oneshot_read()` 는 IDF 내부에서 변환마다
`portENTER_CRITICAL(&rtc_spinlock)` 으로 **인터럽트를 끈다**. 이를 **8회 연속** 돌리는 동안
**CPU 가 곧 클럭인 비트뱅 I2C 전송이 얼어붙어** 프레임이 깨진다.
(HW I2C 는 페리페럴이 자체 클럭으로 보내 영향이 적었다 — 비트뱅 전환이 취약성을 키움.)

실기 이분 탐색으로 확정:

| 조건 | 결과 |
|---|---|
| 충전측정 ON + 버튼 RF ON | 수 분 내 화면 멈춤 |
| 충전측정 OFF + 버튼 RF OFF | 멈춤 없음 |
| **충전측정 OFF + 버튼 RF ON** | **멈춤 없음** → RF 는 무죄 |

### 핵심 결함 — 뮤텍스가 전송 지점을 못 덮는다

| 비트뱅 전송 지점 | 호출 경로 | 보호 |
|---|---|---|
| `oled_ui.c:1054` 페이지 주소 cmd | `_oled_write_page_locked` ← `_fb_flush` | ✅ (1133~1210 락 안) |
| `oled_ui.c:1065` 페이지 데이터 | 〃 | ✅ |
| **`oled_ui.c:204`** `_oled_send_cmds` | 패널 init·contrast·display ON/OFF | ❌ **무방비** |
| **`oled_ui.c:192`** `_bbo_probe` | `_oled_try_detect` ← 5초 주기 재검출 | ❌ **무방비** |

`_read_bat_mv()` 는 `oled_ui_i2c_trylock(30)` 으로 **flush 만** 피하므로 위 두 경로와 그대로 충돌한다.

### 수정안 (우선순위)

1. **`_bbo_write()` 자체를 뮤텍스로 감싼다** ← 가장 확실. 호출 지점이 어디든 자동 보호.
   ★설계 판단 필요: `_fb_flush` 가 이미 락을 쥔 채 들어오므로 **재귀 획득** 처리가 필요하다.
   → `xSemaphoreCreateRecursiveMutex()` 로 바꾸거나, **`_fb_flush` 의 바깥 락을 없애고
   `_bbo_write` 단위로만 잠그는 편**이 깔끔하다(후자 권장 — 락 구간이 짧아져 ADC 가 굶지 않음).
2. `adc_oneshot_read()` **8회 → 1~2회** 축소 + 회차 사이 `vTaskDelay(1)` (크리티컬 구간 분할)
3. 배터리 측정 주기 **5초 → 30초**
4. 적용 후 `TEMP_NO_CHARGE = 0` 원복 → `[OLEDMON]` 실패 카운터로 검증

관련 이슈 초안: `ISSUE_charge_adc_breaks_oled.md` (아직 GitHub 에 등록 안 함, `gh` CLI 미설치)

---

## 그 외 남은 과제

### 1. 화면이 저절로 켜짐 — 진동센서 오검출 (미해결)
```
[VIBE-stat] 진동=1 ISR누적=70338 (3초+100) HIGH=300/300
```
핀이 **300/300 전부 HIGH 고정**인데 ISR 은 초당 33회. `vibration_active` 가 계속 true →
`_mark_activity()` 반복 → 유휴 10초에 영영 도달 못 함.

- 시도: `button_handler.c` `_vibration_track()` 에 200샘플 stuck 판별 추가 → **발동 안 함**
- **먼저 할 일**: `_vibration_track()` 호출 지점과 `[VIBE-stat]` 로그 생성 코드를 **대조**해
  실제 데이터 흐름 확인(내 stuck 카운터가 안 채워지는 이유).
- 대안: `btn_handler_is_vibrating()` 또는 `somfy_app` 의 `vibration_active` 지점에서 직접 차단,
  혹은 ISR 발생률 비정상(초당 30회↑)으로 판정.
- HW: VIBE 핀(GPIO0)·VS1 진동 스위치 배선 점검 필요.

### 2. ESP32-H2 메모리
블라인드 3개에서 **free heap 4,712B / 최대블록 2,432B**(C6 는 171KB). BLE 커미셔닝 불가.
→ H2 는 `BLIND_MAX_COUNT` 를 **2 이하**로.

### 3. COM4 하드웨어
SDA·SCL 둘 다 `0/0/0`. v4 는 PCB 풀업이 없으므로 이는 **모듈 VCC 가 0V** 라는 뜻.
→ 멀티미터로 OLED 모듈 VCC 측정. 0V 면 전원 경로(냉납·단선), 3.3V 면 모듈 손상.

### 4. OLED 장시간 고착 관찰
충전측정을 꺼도 **7시간 뒤 고착**된 이력이 있다(실패율 0.02%→12.5%).
dirty-page 감축(전송 75%↓) 이후 재발 주기가 늘었는지 `[OLEDMON]` 으로 확인.
근본 대책은 **모듈 VCC 를 GPIO+MOSFET 으로 제어**해 자동 전원 리셋(4핀 SSD1306 은 RES 핀이
없어 전원 차단만이 복구 수단).

### 5. WASM 시뮬레이터 복구 (선택)
`sim/wasm/build.ps1` 이 실제 `oled_ui.c` 를 컴파일하는데 stub 헤더가 없다
(`driver/gpio.h`, `soc/gpio_reg.h`, `soc/io_mux_reg.h`, `soc/lp_aon_reg.h`, `esp_rom_sys.h`).
이 include 들은 2026-07-17 부터 있던 것으로 **이번 변경 때문이 아니다.**
`emcc` 미설치 + 빌드 산출물 없음 → 한동안 미사용.
되살리려면 emcc 설치 → 빌드 → **컴파일러가 알려주는 에러대로** stub 추가(추측 금지).

---

## 보드 현황

| 포트 | 보드 | 옵션 | 상태 |
|---|---|---|---|
| COM7 | xiao-c6 (h4 신PCB) | `-Pcf 8575 -Rotary ec05 -Oled 128x64 -Rotate 180 -Freq 447.70` | 최신 |
| COM4 | xiao-c6 (v4 구PCB) | `-Pcf 8574` (좌/우 버튼 없음) | 커밋 `a3694d3` 시점 / OLED HW 문제 |
| COM3 | esp32-h2 | `-Pcf 8574` | 최신 / 메모리 한계 |

빌드 예:
```bash
./build.ps1 -Board xiao-c6 -Action flash -Port COM7 -Pcf 8575 -Rotary ec05 -Oled 128x64 -Rotate 180 -Freq 447.70
```

---

## ★반드시 알아야 할 함정

- **시리얼 로그는 CP949 인코딩.** UTF-8 로 grep 하면 한글 에러가 **0건으로 오독**된다.
  → `open(...,'rb').read().decode('cp949')`. 이것 때문에 "에러 0, 해결됨"이라고 잘못 보고한 적 있음.
- **부팅 ~100ms 구간 로그는 USB-JTAG 이 버린다.** 진단값은 static 에 저장해 루프에서 재출력.
- **긴 한글/기호 줄도 유실**된다. 중요 측정값은 짧은 ASCII 한 줄로(`XCONN sda=%d>%d ...`).
- **"에러 0"은 정상의 증거가 아니다.** OLED 미검출이면 flush 를 건너뛰어 에러도 안 난다.
  `present=1` 과 전송 카운터 증가를 함께 볼 것.
- **`#if BOARD_OLED_BITBANG` 밖에서 쓰는 전역은 가드 밖에 선언**할 것.
  가드 안에 두면 `BOARD_OLED_BITBANG=0` 보드(H2)에서 컴파일/링크 에러(실제로 두 번 발생).
- 빌드/플래시 후 **반드시 `error:` 카운트를 확인**할 것. 빌드 실패인데 "플래시 OK"로
  보고한 적 있음.

---

## 배경 문서

- `README.md` 「OLED 구동 방식 & 화면 정책」 — 비트뱅 전환 이유, dirty-page, 화면 정책, 진단 로그
- `sim/tools/README_oled_sim.md` — 진단 시뮬레이터 3종 사용법과 한계
- `doc/CHECKLIST.md` — 기능 구현 현황(화면보호기 삭제·충전측정 ⚠ 반영됨)
- 커밋 메시지 — 각 변경의 증상→원인→조치가 상세히 기록돼 있다(`git log`)
