# 인수인계 — 다음 세션에서 이어갈 것

> 새 세션을 열고 **"HANDOFF.md 읽고 이어서 작업해줘"** 라고만 하면 된다.

기준: `main` 최신 커밋 (원격 `github.com/kgun2g/somfy-rts-remote-by-thread`)
> 최근 변경 요약은 `git log` 의 커밋 메시지가 가장 상세하다.

---

## ✅ 완료 — 충전률 측정 재활성 (2026-08-11)

`TEMP_NO_CHARGE = 0`. 상세는 `ISSUE_charge_adc_breaks_oled.md` 와
README 「OLED 구동 방식 & 화면 정책」 참조. 요약만 남긴다.

**진짜 원인은 "ADC 가 인터럽트를 끈다" 가 아니라 직렬화 구멍이었다.**
락(`oled_ui_i2c_lock`)이 `_fb_flush` 만 덮어, `_oled_send_cmds`(화면 자동 OFF/ON,
10초마다)와 `_bbo_probe`(미검출 시 5초마다)가 무방비였다. 우선순위가
`btn_handler 10 > somfy_app 4 > oled_ui 3` 이라 somfy_app 이 oled_ui 를 **선점**하고,
비트뱅은 CPU 가 곧 클럭이라 선점당한 전송이 SSD1306 을 고착시켰다(래치).

**수정**: 락 범위를 `_bbo_write()` 자체로 이동 + **재귀 뮤텍스**(flush 가 바깥 락을
쥔 채 부르므로 일반 뮤텍스면 즉시 데드락) + 뮤텍스 생성을 `oled_ui_init` 맨 앞으로.
락 타임아웃 200 ms 시엔 전송을 진행하고 `[OLEDMON] lockTO` 로 계수(정상 = 0).

**측정 주기 5초·표본 8회는 그대로 둘 것** — 줄이면 `_nobat_track` 의
"5분 창 / 반쪽당 30표본" 가정이 깨져 배터리 미연결 오판이 난다.

검증: `sim/tools/adc_oled_mutex_sim.py`(8시드×10분) 수정 전 6/8회 고착(최빠른 80초)
→ 수정 후 0/8.

**★실기 장시간 검증 (2026-08-11 02:15~07:00, COM7)** — 콘솔 `cyc` 로 화면을 계속
갱신시켜 OLED 를 100% 부하로 돌리고 ADC 는 5초 주기로 병행:

| 항목 | 결과 |
|---|---|
| 가동 | **4시간 45분** (17,047초) |
| 전송 | **257,562건** (907건/분) |
| **실패** | **0** (0.000000%) |
| **lockTO** | **0** |
| present | 284개 샘플 **전부 1** (미검출 0회) |
| 패닉·재부팅·워치독·abort·스택오버플로 | **각 0건** |
| OLED 검출실패 / 버스복구 호출 | **0건 / 0건** |
| free heap | 169,152B (첫 샘플 대비 **-80B**) |
| dirty-page 감축 | 98.1% (보냄 51,503 / 건너뜀 2,672,313) |

되돌리려면 `TEMP_NO_CHARGE` 를 1 로.

---

## ✅ 완료 — 배터리 "78%" 오표시 (2026-08-11)

**증상**: 배터리를 안 꽂았는데 충전률이 78% 로 표시됨.

**원인**: 배터리를 빼도 충전 IC 가 BAT+ 를 **3,970 mV** 로 띄우는데, 이 값이 OCV 곡선상
정확히 78% 다. 전압만으론 실제 배터리 78% 와 구분 불가. 구분 단서인 "전압이 오르는가"
판정(`_nobat_track`)은 **5분 창**이 차야 나오므로, 부팅 후 **5분 23초** 동안 78% 가 보였다.
(테스트로 시리얼을 열 때마다 DTR/RTS 로 리셋돼 그 5분이 계속 리셋된 것도 겹쳤다.)

**수정**: 첫 판정 전에는 % 대신 `--%` 표시(`BAT_PCT_UNKNOWN`). 단 **애매할 때만** 숨긴다 —
float 창(3940~4010 mV) 밖이거나 USB 미연결이면 배터리 존재가 자명하므로 즉시 % 표시.

**★창(5분)을 줄이지 말 것.** 충전 상승률 ≈9.3 mV/5분 vs 문턱 4 mV →
3분 창은 2.8 mV 라 **충전 중인 배터리를 "미연결"로 오판**한다.
검증표는 `sim/tools/bat_pct_display_sim.py` 및 `sim/tools/README_oled_sim.md` 부록.

---

## ✅ 완료 — 배터리 단독 부팅 멈춤 (2026-08-11)

**증상**: USB 를 빼고 배터리만 연결하면 부팅 화면에서 멈추고 40초~1분마다 재부팅.
USB 를 꽂으면 정상 — 그래서 시리얼 로그를 볼 수 없었다.

**원인**: `esp_matter::console::init()` 이 CHIP shell 을 **우선순위 5** 태스크로 띄운다
(`somfy_app`=4, `oled_ui`=3 보다 높다). 프롬프트 출력이 `esp_rom_usb_serial_putc` →
`usb_serial_device_tx_flush()` 로 **USB 호스트를 기다리는데**, 호스트가 없으면 이 대기가
길어져 prio 5 가 아래 태스크를 전부 굶긴다.

**수정**: **VBUS(GPIO17)가 있을 때만** CHIP shell 을 시작(`app_main.cpp`
`_usb_vbus_present()`). 콘솔은 개발용이고 `tx`/`sel`/`cyc`/`bd` 도 USB 전용이다.
(`usb_serial_jtag_is_connected()` 는 SOF 기반이라 부팅 1초 시점에 false 가 나올 수 있어
USB 개발 중에도 콘솔이 사라진다 — 그래서 VBUS 핀을 쓴다.)

**진단 도구 — `boot_diag`** (`main/boot_diag.c`, 이번에 새로 만듦):
전원이 끊겨도 남는 **NVS 부팅 단계 기록**. RTC RAM 은 배터리를 빼면 지워져 못 쓴다.
- app_main 경로(`stage`)와 somfy_app 경로(`stage2`)를 **따로** 센다 — 두 태스크가 동시
  진행하므로 단일 카운터로는 어느 쪽이 멈췄는지 구분 불가
- 실패한 부팅은 별도 키(`fail`)에 **영구 보관** — 정상 부팅이 덮어쓰면 증거가 날아간다
  (실제로 한 번 날렸다)
- 콘솔 **`bd`** 로 언제든 조회, `bd clear` 로 삭제
- 끄려면 `BOOT_DIAG_ENABLE 0` (부팅당 NVS 쓰기 ~17회)

**탈락시킨 가설들**: 전원/브라운아웃(배터리 4.02 V, 최저치 동일 — 사그 0), Task WDT
(5초라 주기 불일치), BAT_ADC 로직(해당 코드 미실행), I2C 스캔 NULL 핸들(`sub=10` 이라 미실행).

> ⚠ 진단 중 브라운아웃 임계값을 2.51 V → **2.92 V**(`sdkconfig.xiao-c6`
> `ESP_BROWNOUT_DET_LVL=4`)로 올렸고 **그대로 두었다**. 원인은 전원이 아니었지만,
> 더 민감한 검출은 보호 측면에서 유리하다. 되돌리려면 그 두 줄을 `SEL_7`/`7` 로.

---

## ✅ 완료 — 배터리 % 흔들림 (2026-08-11)

배터리 구동에서 81→82→**74**→84→81 % 로 튀었다. BLE 광고·RF 송신 순간 **실제로**
전압이 떨어지는데(USB 는 레일이 단단해 안 보였다), OCV 곡선상 이 구간은 1 % ≈ 10 mV 다.

표시 경로에만 **중앙값 5주기 + EMA(α=1/4)** 적용(`_bat_smooth_mv`). `_nobat_track` 에는
**원본**을 준다(5분 창 30표본 통계라 평활하면 가정이 깨짐). 측정 주기·표본 수 불변.

★**EMA 는 1/16 mV 단위로 누적**한다 — C 의 정수 나눗셈이 0 쪽으로 절단해 1 mV 단위로
쓰면 차이 1~3 mV 에서 몫이 0 이 되어 EMA 가 고착된다.
검증: `sim/tools/bat_pct_smooth_sim.py` (진폭 19→9 %p, 주기간 변동 4.09→0.38 %p).

---

## ✅ 완료 — 화면 OFF 시간 전원별 분리 (2026-08-11)

| 전원 | 매크로 | 값 |
|---|---|---|
| USB | `CFG_SCREEN_OFF_USB_SEC` | **300초(5분)** |
| 배터리 | `CFG_SCREEN_OFF_SEC` | 10초 |

---

## 남은 과제

### 1. 진동센서 — 소프트는 해결, **하드웨어 고장은 남음**

★2026-08-11 정정: "stuck 판별이 발동 안 한다"는 이전 기록은 **틀렸다**. 실기에서 발동한다.

```
W (21051) BTN: [VIBE] 센서 고장 판정(핀 고정) (최근 200샘플 HIGH=200) — 진동 무시함
```

덕분에 화면도 정상적으로 꺼진다(유휴 캡처에서 60초 이후 전송이 멈춘 채 200초 이상 유지).
→ **"화면이 저절로 켜져 안 꺼진다"는 해결됨.**

다만 근본은 하드웨어다. VIBE 핀이 **300/300 전부 HIGH 고정**이라 진동을 아예 못 읽는다.

- 현재 영향: **진동으로 화면을 깨울 수 없다**(버튼으로만 깨어남). 소프트로는 더 할 게 없다.
- 할 일: VIBE 핀(GPIO0)·VS1 진동 스위치 배선/납땜 점검. 핀이 계속 HIGH 면
  스위치 단선 또는 GND 미연결이 유력.
- 판정 로직은 `button_handler.c` 의 `VIBE_STUCK_WIN`(200샘플) — 건드릴 필요 없음.

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
| COM3 | esp32-h2 | `-Pcf 8574 -Rotate 180` | 최신 / 메모리 한계 |

> ★COM3 의 **`-Rotate 180` 은 필수**다(2026-08-13 표 보완). H2 시제품은 OLED 가 180°
> 뒤집혀 장착돼 있고 `boards/esp32-h2.h` 기본값은 `BOARD_OLED_ROTATE_180 0` 이라,
> 옵션을 빼고 구우면 **화면이 상하반전**된다. `doc/wiring/wiring_esp32-h2.md:13` 에는
> 원래 적혀 있었는데 이 표에서 누락돼 실제로 한 번 당했다(2026-07-02).

빌드 예:
```bash
./build.ps1 -Board xiao-c6 -Action flash -Port COM7 -Pcf 8575 -Rotary ec05 -Oled 128x64 -Rotate 180 -Freq 447.70
```

---

## ★절전 측정 현황 (2026-08-14)

**비교는 반드시 "동일 전압 구간의 기울기"로 한다.** % 는 1%p 양자화 + OCV 곡선
비선형이고, 저 SoC 에서는 부하 강하가 커져 전체 평균 % 로 비교하면 왜곡된다.
세션 조건(`radio` / `screen` / `pm`)이 같은지도 같이 확인할 것 — 배터리 기록의
각 행에 들어 있다(`bl` 콘솔 명령).

| 세션 | 구성 | 길이 | 3790~4000mV 구간 | 환산 |
|---|---|---|---|---|
| #22 | 개선 전 (tick **100Hz**, LP 없음) | 6.64h | −0.67 mV/분 | **47.4mA** |
| #32 | ①tick **1000Hz** + ③LP 코어 | 7.70h | −0.75 mV/분 | **52.7mA** |

세부 4구간 중 3개 악화 + 1개 동일, 개선된 구간 **없음**. 두 세션 모두
`radio=1 / screen=OFF / pm=3` 로 조건 동일(#22 는 화면 ON 11% 포함 = 오히려 불리).

⇒ **①(tick 1000Hz)은 절전이 아니라 소폭 악화.** 2026-08-14 **100Hz 로 되돌렸다.**

> ★되돌리기 전에 반드시 했어야 하는 일: `pdMS_TO_TICKS` 는 정수 내림이라
> 100Hz 에서 **10ms 미만 대기가 0 tick = 소멸**한다. `vTaskDelay(0)` 은 yield 라
> 대기가 통째로 사라진다. 해당 4곳을 `esp_rom_delay_us()` 로 교체 완료:
> `somfy_rts.c:339/415`(5ms VCO/PA settle — SDR 실측으로 넣은 것),
> `cc1101.c:365`(2ms SCAL), `somfy_app.c:3399`(3ms CHG_STAT 풀업 안정).
> **tick 을 다시 건드릴 때는 `vTaskDelay(pdMS_TO_TICKS(x))` 전수 재검사할 것.**

100Hz 실기 확인: 부팅 정상, `tx up 3000` = 3.29초/23프레임(1000Hz 는 3.44초/24).
프레임이 오히려 촘촘해진다(1000Hz 는 프레임당 +4.81ms 여분 틈이 생겼다).

### ★★★진짜 원인 발견 (2026-08-15) — light sleep 은 한 번도 진입한 적이 없었다

`CONFIG_PM_PROFILING=y` + 콘솔 `pm`(`esp_pm_dump_locks`)으로 **계측**했다:

```
Lock stats                     Active  Total_count  Time(%)
  bt        APB_FREQ_MAX          1        1          100%   ★
  rmt_0_0   CPU_FREQ_MAX          1        1          100%   ★
  ot_sleep  APB_FREQ_MAX          0       59            1%
Mode stats:  CPU_MAX 160M  99%      ← light sleep 항목 자체가 없음
```

락 두 개가 부팅부터 100% 잡혀 있어 **automatic light sleep 이 진입 자체를 못 했다.**
①(tick)·③(LP 코어)이 3회 측정에서 전부 눈금에 안 잡힌 이유가 이것이다 —
CPU 가 CPU_MAX 에서 내려온 적이 없으니 **CPU 쪽 조치는 효과가 있을 수 없었다.**

| 조치 | 내용 | 결과 |
|---|---|---|
| `rmt_0_0` | `rmt_enable()` 이 CPU_FREQ_MAX 락을 잡는다. 부팅 때 켜고 계속 뒀다 → **burst 동안만** 켜도록(`_rmt_acquire`/`_rmt_release`, 경계는 `cc1101_enter_tx_mode`…`cc1101_idle`). `rmt_disable` 은 진행 중 전송을 자르므로 `rmt_tx_wait_all_done()` 로 먼저 비운다 | **CPU_MAX 99% → 5%** |
| `bt` | `CONFIG_USE_BLE_ONLY_FOR_COMMISSIONING` 이 CHIP 기본값 y 인데 우리는 꺼져 있었다(esp-matter 예제 기본값이 이유 없이 따라옴). 켜면 등록 후 BLE deinit + 이미 provisioned 면 부팅 시 초기화조차 안 함 | **`bt` 락 소멸, 힙 164→184KB** |

수정 후: `APB_MIN 90% / APB_MAX 3% / CPU_MAX 6%`, 락 목록 깨끗.
※USB 에서는 설계상 light sleep 을 끄므로(USB-JTAG 보호) **진입 여부는 배터리에서만** 보인다.

### ★계측을 NVS 로 남긴다 (2026-08-15)

이 보드는 **포트를 여는 순간 리셋**된다(`리셋사유=USB리셋(플래시/포트열기)`).
그래서 `esp_pm_dump_locks` 의 RAM 통계는 **읽으러 가는 행위가 지워버린다** —
실제로 배터리 구간 통계를 그렇게 날렸다.
→ `CONFIG_PM_LIGHT_SLEEP_CALLBACKS=y` + `esp_pm_light_sleep_register_cbs` 의 exit
콜백이 주는 **실제 `sleep_time_us`** 를 누적해 배터리 로그 행(`bat_sample_t.ls`)에
싣는다. `bl` 한 번으로 방전 기울기와 sleep% 를 같은 표에서 본다.

```
BL   12  + 1436s  3869mV   62%  avg  70mA  radio=1 screen=0 pm=3  sleep 87%  산포  2( 3mV)  주기
                                                                  ^^^^^^^^^
```

### 남은 후보 (기대값 순)

| 순서 | 조치 | 상태 |
|---|---|---|
| 0 | **락 해제 후 방전 측정** — `bt`/`rmt` 락을 푼 뒤 배터리 방전을 아직 못 받았다 | **미측정. 최우선** |
| 1 | `CONFIG_MAC_BB_PD` 활성화 — light sleep 중 라디오 MAC/BB 전원차단 | 미시도. 현재 not set |
| 2 | CC1101 `SPWD` (송신 안 할 때 절전) | 미시도 (데이터시트 ~1.7mA **추정**) |
| 3 | btn_task 유휴 주기 25ms → 30ms 초과 | 미시도 — 아래 ★ |
| 4 | 메인 루프 유휴 깨움(10회/초), hold_repeat(2회/초) | 미시도 |

> ★**light sleep 은 아직 한 번도 진입한 적이 없을 가능성이 크다.** 문턱은
> `FREERTOS_IDLE_TIME_BEFORE_SLEEP=3` tick 이고(FreeRTOS 최소값 2라 더 못 낮춘다),
> 100Hz 에서 3 tick = **30ms**. 그런데 btn_task 유휴가 25ms → `pdMS_TO_TICKS(25)`
> = 2 tick = 20ms 로 깨어난다 → 문턱을 못 넘는다.
> `pm=3` 은 "esp_pm_configure 가 light sleep 을 **허용**했다"는 뜻일 뿐
> **실제로 자고 있다는 증거가 아니다.** 라디오(~60mA)가 지배 항목이므로
> 1번(MAC_BB_PD)부터 가는 게 기대값이 크다.

## ✅ 완료 — PROG 긴 누름 무응답 (2026-08-15)

**증상**: PROG 를 2초 이상 눌러도 블라인드가 반응하지 않음(짧게는 정상, 정품은 동작).
훨씬 이전부터 있던 문제.

**원인**: `_build_frame` 이 **byte7 을 전 프레임 `0x84` 로 고정**하고 있었다.
정품(그리고 ESPSomfy)은 **재전송마다 byte7 을 올린다** — 첫 프레임 `0x84`,
재전송 `196 + rep*4` (= `0xC4, 0xC8, 0xCC, …`).

**수정**: `somfy_rts.c` 송신 루프에서 프레임마다
`frame[7] = first ? 0x84 : _encode80_byte7(196, i-1)` 로 갱신하고,
체크섬이 byte7 을 입력으로 받으므로 `frame[9]` 도 함께 재계산한다.
(조합 명령 UP+DOWN/MY± 은 byte9 가 실측 고정값이라 제외.)
**타이밍은 건드리지 않았다** — 과거 "byte7 가변 → 모터 무응답" 기록은 타이밍
표준값과 묶어서 바꾼 것이라 byte7 이 누명을 썼던 것으로 보인다.

### ★★★이 건에서 이틀 돈 이유 — rtl_433 표시값 ≠ wire 바이트

송신은 b[1..6]만 체인 XOR 하지만 **rtl_433 풀 디코더는 b[1..9] 전체를 디스크램블**한다
(`ESPSomfy-RTS/SOMFY_RTS_447.md` 에 명시돼 있다). 따라서

```
표시 b8 = wire b8 ^ wire b7        표시 b9 = wire b9 ^ wire b8
```

콘솔 `tx8` 로 raw byte8 을 스윕해 8/8 실증했다(`H4_02.json`):

| raw | 0x00 | 0x20 | 0x40 | 0x44 | 0x48 | 0x4C | 0xC4 | 0x60 |
|---|---|---|---|---|---|---|---|---|
| 표시 | 84 | A4 | C4 | C0 | CC | C8 | 40 | E4 |

> **오독 경위(반복 금지)** — 정품이 짧게 `84/A4/A8`, 길게 `C4/E4/E8` 로 보여
> "bit 0x40 = 누르고 있음(hold 코드)" 이라고 판단하고 그걸 구현했다가 **멀쩡하던
> 긴 누름까지 깼다**. 실제로는 wire b7(재전송 인덱스)이 XOR 되어 그렇게 보인 것이고
> **hold 코드는 존재하지 않는다**. 정품 81건 역산에서 75건(93%)이
> `0x84`(첫 프레임)/`0xC4`(재전송0)/`0xC8`(1)/`0xE8`(9)/`0xFC`(14) 로 설명된다.
> 나머지 6건은 `0xFD` 로 계열 밖 + byte9 불일치 = 디코드 오류.
> **표시값을 wire 값으로 착각하지 말 것.**

### 참조 구현을 먼저 볼 것

`D:\dev\workspaces\ESPSomfy-RTS` — 사용자가 447MHz 2-FSK 대응을 추가한 fork.
`SOMFY_RTS_447.md`(설계 근거) + `Somfy.cpp` 의 `encode80BitFrame` /
`encode80Byte7` / `sendFrame`. 우리 `somfy_rts.c` 와 구조가 사실상 같다.
전 항목 대조 결과 프레임 내용·프레임 수는 일치하고, 타이밍은 우리가 실측에 더 가깝다
(SYMBOL 644 vs 640, SW sync 4840 vs 4850, interFrame 3916 vs 4000).
**프로토콜 의문이 생기면 SDR 분석 전에 여기부터 볼 것.**

## ★RF 실측 데이터 — 추측하지 말고 여기서 확인할 것

| | 경로 |
|---|---|
| 정품 리모컨 SDR 캡처 | `D:\RTL_SDR\sdrsharp-x64\somfy_rts_447` |
| 분석 스크립트 + 절차 문서 | `D:\dev\workspaces\plugin-Rtl433-for-SdrSharp-master\scripts` (README.md §0, §7) |

캡처는 `1. up` / `2. down` / `3. my` / `4. prog` / `5. tilt up` / `6. tilt down` 로
버튼별 폴더, 그 아래 게인별 하위폴더(`3_42.1dB` 등). **한 wav = 버튼 한 번 누름.**

```bash
python diag_iq.py <정품.wav> <우리.wav>   # burst 위치/길이 (인자 2개 필수)
python dump_burst.py <wav>                # 펄스열 — HW/SW sync 개수
python measure_rf.py <wav>                # deviation / carrier offset / 타이밍
```
※경로에 공백이 있어 셸에서 `$(ls ...)` 로 넘기면 인자가 쪼개진다. Python 으로 돌릴 것.

> ★**캡처에는 "일부러 길게 누른 것"이 섞여 있다.** 사용자가 짧게/길게를 구분하라고
> 넣어둔 것이다. 한두 개만 보고 "이 버튼은 원래 길다"고 결론내면 틀린다 —
> 반드시 **전수 중앙값**으로 볼 것. 실제로 그 오독으로 PROG 특수처리가 들어갔었다
> (2026-08-13 제거). 전수 측정 결과 송신 길이는 **전 버튼 350~400ms 동일**이고,
> HW sync 도 prog 29/34/29 vs down 36/36/36/36 으로 **PROG 가 오히려 적다**.
> 정품 구조는 분석 README 에 "HW sync 12회(첫 프레임)/6회(재전송), 버튼 구분 없음"
> 으로 이미 적혀 있었다.

### 길게 누름 = **끊김 없는 한 덩어리** (2026-08-13 확정)

같은 캡처를 10ms 창으로 다시 본 결과(`scratchpad/hold_pattern.py`), ON 길이 분포는

```
300~399ms 765개 · 400~499 41 · 600~699 4 · 700~799 7 · 900~999 3 · 1300~1399 4
1. up  g066  910          ← 910ms 가 통째로 하나의 ON
3. my  g107  1360         ← 1360ms 도 하나
1. up  g070  340 [90] 670 ← 뗐다 다시 누른 것
```

즉 **정품은 누르고 있는 동안 조각내지 않고 계속 쏜다.** 간격이 보이는 캡처는
전부 별개 누름이다. 우리 펌웨어도 이 구조로 맞췄다(아래 함정 절 참조).

## 진단 콘솔 명령 (USB 시리얼)

| 명령 | 용도 |
|---|---|
| `bl` / `bl clear` | 배터리 방전 기록(NVS) — 전압·%·평균전류·**sleep%**·ADC 산포 |
| `pm` | `esp_pm_dump_locks` — PM 락 보유 + 절전모드 체류(진단 빌드 전용) |
| `tx <cmd> [hold_ms]` | RF 송신 (up/down/updown/myup/mydown/my/prog) |
| `tx8 <hex> [cmd] [hold]` | **raw byte8 강제 지정** 송신 — rtl_433 표시값과의 대응표 작성용 |
| `intpd` | `~INT` 풀다운 진단 — 배선단선 vs PCF측 고장 판별 |
| `intdiag` | `~INT` 선 관찰(15초, 버튼 조작 필요) |
| `vl` / `bd` | 진동센서 기록 / 부팅 진단 |

> ★시리얼 접속은 `write_timeout` 을 반드시 설정할 것. 안 그러면 write 가 hang 한다.
> 그리고 **포트를 여는 것만으로 기기가 리셋된다** — RAM 통계는 그때 날아간다.

## ★반드시 알아야 할 함정

- **`somfy_rts_abort` 는 프레임 경계(≈143ms)에서 딱 1번만 검사된다.**
  그래서 `abort=true; delay(20); abort=false;` 같은 **짧은 펄스는 그냥 놓친다** →
  끊으려던 job 이 안 죽고 `max_loops`(hold_ms 15s ⇒ 15.3초)까지 살아버리고,
  새 job 은 직렬 worker 뒤에 줄서서 **버튼 뗀 뒤에 나간다**.
  반드시 **레벨로 세워둔 채** `s_rf_tx_busy=false && 큐 빔` 을 확인하고 해제할 것.
  (`_hold_repeat_task` 의 `s_combo_pending` 상태머신이 이 방식이다.)
- **버튼 PRESS 는 같은 cmd 가 송신 중이면 새 job 을 만들면 안 된다**
  (`_send_command_press` → `_rf_same_cmd_inflight`). PRESS 가 `somfy_rts_abort=false`
  로 중단요청을 해제하므로, 연타 시 job 을 하나 더 넣으면 이전 job 이 abort 를
  못 만나 15초 폭주한다. `sim/tools/hold_gap_sim.py` 케이스 ⑦ 참조.
- **`_hold_repeat_task` 는 누름과 위상이 맞지 않는 자유주행 태스크다.**
  "500ms 주기니까 최대 500ms 지연"이 아니라, 게이트(500ms)+위상까지 겹쳐
  위상 스윕 실측 **최악 561ms 공백**이 나왔다. 주기 기반 재송신으로 연속성을
  만들려 하지 말 것 — 한 job 이 스스로 길게 쏘게 해야 한다.
- **PCF8574 `~INT`(GPIO2) 는 쓸 수 없다 — 2026-08-13 실측 확정.** 절전을 위해
  10ms 폴링을 인터럽트 기반으로 바꾸려는 시도(②)는 **이 하드웨어에서 불가능**하다.
  콘솔 `intdiag` 로 폴링을 멈춘 채 관찰한 결과:

  | 조건 | 표본 | 결과 |
  |---|---|---|
  | 정지 상태 | 38,765 | HIGH 100% / **전이 0회** |
  | **버튼 조작 중 15초** | **290,023** | HIGH 100% / **전이 0회** |
  | 폴링 동작 중 | 33,198 | HIGH 100% / 전이 0회 |

  회로도상 풀업(`R3 10k → +3V3`)은 **정상적으로 있다**(kicad/somfy_blinds_h4). 그런데
  버튼을 눌러도 선이 안 움직인다 → PCF8574 가 `~INT` 를 구동하지 못하는 상태
  (배선 미연결 또는 IC INT 출력 불량). **풀업 추가는 해결책이 아니다** — 이미 있다.
  → ② 를 다시 시도하기 전에 **반드시 `intdiag` 로 전이가 잡히는지 먼저 확인**할 것.

  **데이터시트 대조(`doc/parts/pcf8575.pdf`)로 대안 가설이 전부 배제됐다** —
  INT sink `IOL 1.6mA` vs 우리 부하 0.33mA(풀업 무관) · `tiv 4µs`(샘플링 무관) ·
  *"reading/writing **another device** does not affect the interrupt circuit"*(OLED 무관).
  게다가 *"resetting … when data on the port is **changed to the original setting**"* 이라
  **버튼을 누르고 있는 동안 INT 는 LOW 로 유지**된다(누름당 100ms+). 290,023 표본에서
  전이 0회는 "짧은 펄스를 놓쳤다" 로 설명 불가 = **선이 정말 안 움직인다.**
  고장 위치는 `intpd`(GPIO2 내부 풀다운+ADC, 풀다운 시 2508mV → 상단 풀업 ≈14k)로
  **R3 ~ U3 pad1 구간**으로 좁혀졌다 → **pad1 재납땜 먼저**. 자세한 건
  `doc/wiring/wiring_xiao-c6.md` 의 `~INT` 절.

  ★★**`intdiag` 는 LP 코어 폴링을 멈춰야 한다**(2026-08-15 수정). PCF 읽기가 INT 를
  해제하는데 LP 가 2ms 마다 읽으므로, 안 멈추면 관찰 구간이 통째로 HIGH 로 보인다.
  최초 측정(13:46, `1bbcbc5`)은 LP 도입(18:54, `f45ac86`)보다 5시간 앞서 이 문제가
  없었지만, **재검증 시에는 이 수정 없이는 무조건 "여전히 죽음" 이 나온다.**
  ※폴링과 INT 는 원리적으로 공존이 어렵다 —
  *"Interrupts that occur during the ACK clock pulse **can be lost**"*.
  ※`~INT` ISR 을 등록해두면 **초당 약 4,500회 폭주**해 prio 10 버튼 태스크를 짓밟아
    **버튼이 통째로 죽는다**(실측 540,522회/120초). 선은 조용하므로 전기 신호가 아니라
    `_vibe_isr_handler` 의 `gpio_ll_intr_disable`/`gpio_intr_enable` 반복과의 간섭으로
    본다. 이걸 "진단용이라 무해"하다고 남겨뒀다가 버튼을 두 번 죽였다.
- **진동센서(X160)는 배선이 아니라 센서 자체가 죽을 수 있다.** 2026-08-13 에 세 번
  재납땜하며 배선을 의심했는데, 증상이 `GND 고정(0/300)` → `HIGH 고정(300/300)` 으로
  바뀌기만 했다. **센서를 교체하니 즉시 정상**(평상시 LOW, 흔들 때 HIGH 1~75/273).
  → 판정은 콘솔 `vl`(NVS 진동 기록)로. `섞인 창`이 0 이면 센서가 한 번도 동작 안 한 것.
  정상 기준은 H2(COM3)와 같은 패턴이다.

- **로그 인코딩은 캡처 경로마다 다르다.** pyserial 직접 캡처 = **UTF-8**,
  빌드 로그(`logs/*.log`) = **CP949**. 틀리게 디코드하면 한글 에러가 **0건으로 오독**된다.
  → `try utf-8 → except cp949`. 이것 때문에 "에러 0, 해결됨"이라고 잘못 보고한 적 있음.
- **화면이 꺼지면 OLED 부하가 0 이라 검증이 안 된다.** 콘솔 `cyc 1` / `cyc -1` 을 2초마다
  보내면 화면 내용이 바뀌어 dirty-page 가 계속 생긴다(스트레스 스크립트로 활용).
- **부팅 ~100ms 구간 로그는 USB-JTAG 이 버린다.** 진단값은 static 에 저장해 루프에서 재출력.
- **긴 한글/기호 줄도 유실**된다. 중요 측정값은 짧은 ASCII 한 줄로(`XCONN sda=%d>%d ...`).
- **"에러 0"은 정상의 증거가 아니다.** OLED 미검출이면 flush 를 건너뛰어 에러도 안 난다.
  `present=1` 과 전송 카운터 증가를 함께 볼 것.
- **`#if BOARD_OLED_BITBANG` 밖에서 쓰는 전역은 가드 밖에 선언**할 것.
  가드 안에 두면 `BOARD_OLED_BITBANG=0` 보드(H2)에서 컴파일/링크 에러(실제로 두 번 발생).
- 빌드/플래시 후 **반드시 `error:` 카운트를 확인**할 것. 빌드 실패인데 "플래시 OK"로
  보고한 적 있음.
- **`sdkconfig.defaults.*` 를 고쳐도 안 먹는다.** `build.ps1` 은 보드별 `sdkconfig.<board>`
  를 실제 설정 파일로 쓰고, defaults 는 **최초 생성 때만** 씨앗으로 쓴다. 이미 만들어진
  뒤에는 `sdkconfig.xiao-c6` 를 직접 고쳐야 한다(실제로 한 번 헛빌드했다).
- **리셋 사유는 "그 부팅이 시작된 이유"** 지 "끝난 이유"가 아니다. 헷갈리면 오진한다.

---

## 배경 문서

- `README.md` 「OLED 구동 방식 & 화면 정책」 — 비트뱅 전환 이유, dirty-page, 화면 정책, 진단 로그
- `sim/tools/README_oled_sim.md` — 진단 시뮬레이터 6종 사용법과 한계
  (OLED I2C 3종 + 직렬화·채널잠금·배터리% 3종)
- `doc/CHECKLIST.md` — 기능 구현 현황
- `ISSUE_charge_adc_breaks_oled.md` — 배터리/충전 이슈 4건의 원인·근거·수정·검증 전문
- 커밋 메시지 — 각 변경의 증상→원인→조치가 상세히 기록돼 있다(`git log`)
