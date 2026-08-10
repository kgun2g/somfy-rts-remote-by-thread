# [해결됨] 충전률 측정을 켜면 OLED 비트뱅 I2C 전송이 깨짐

> **상태: 2026-08-11 해결.** `TEMP_NO_CHARGE = 0` 으로 되돌렸다.
> 원인은 처음 추정했던 "ADC 가 인터럽트를 끈다" 가 아니라 **직렬화 구멍**이었다.

## 증상

배터리 충전률 측정을 켜면 **수 분 내에 OLED 화면이 멈췄다**. SSD1306 은 4핀 모듈이라
RES 핀이 없어, 한 번 고착되면 ESP32 리셋·재플래시로도 풀리지 않았다.

## 근거 (실기 이분 탐색, COM7 / xiao-c6)

| 조건 | 결과 |
|---|---|
| 충전측정 ON + 버튼 RF ON | 수 분 내 화면 멈춤 |
| 충전측정 OFF + 버튼 RF OFF | 멈춤 없음 |
| **충전측정 OFF + 버튼 RF ON** | **멈춤 없음** → RF 는 무죄 |

## 진짜 원인 — 락이 전송 경로를 다 덮지 못했다

ESP32-C6 는 싱글코어라 "동시 실행"이 아니라 **선점**이 파괴 기전이다.

    btn_handler 10  >  somfy_app 4  >  oled_ui 3
                          ↑ ADC + 화면OFF/ON      ↑ OLED 비트뱅 flush

`somfy_app`(prio 4)이 `oled_ui`(prio 3)를 선점한다. 비트뱅은 **CPU 가 곧 클럭**이라
선점당한 전송은 SCL/SDA 가 중간 상태로 수백 µs 멈춰 SSD1306 상태머신이 고착된다.
`adc_oneshot_read()` 의 `portENTER_CRITICAL` 은 이 구간을 인터럽트로도 되돌릴 수 없게
만드는 **악화 요인**이지 근본이 아니었다.

`oled_ui_i2c_lock()` 이 `_fb_flush` 만 감싸서 아래 두 경로가 **무방비**였다:

| 무방비 경로 | 호출자 | 빈도 |
|---|---|---|
| `_oled_send_cmds()` ← `oled_ui_set_display_on()` | somfy_app | 화면 자동 OFF/ON **10초마다** |
| `_bbo_probe()` ← `_oled_try_detect()` | flush 의 early-return 경로(락 잡기 **전**) | 미검출 시 5초마다 |

`_read_bat_mv()` 의 `oled_ui_i2c_trylock(30)` 은 flush 하고만 직렬화돼 위를 못 막았다.

## 수정 (적용됨)

1. **락 범위를 `_bbo_write()` 전송 함수 자체로 이동** — 호출 지점이 늘어도 자동 보호
2. **재귀 뮤텍스로 변경**(`xSemaphoreCreateRecursiveMutex` + `Take/GiveRecursive`) —
   `_fb_flush` 가 바깥 락을 쥔 채 `_bbo_write` 를 부르므로 일반 뮤텍스면 **즉시 데드락**
3. 뮤텍스 생성을 `oled_ui_init()` **맨 앞**으로 — NULL 이면 무보호로 통과하던 구간 제거
4. 락 타임아웃(200 ms) 시 **전송은 진행**하고 `g_bbo_lock_to_cnt` 로만 계수
   (영구 정지보다 한 프레임 깨짐이 낫다). `[OLEDMON]` 의 `lockTO` 로 관찰
5. `TEMP_NO_CHARGE` 1 → **0**

**측정 주기(5초)·표본 수(8회)는 바꾸지 않았다.** 줄이면 `_nobat_track` 의
"5분 창 / 반쪽당 30표본" 노이즈 상쇄 가정이 깨져 배터리 미연결 오판이 생긴다.

## 검증

**시뮬레이션** — `sim/tools/adc_oled_mutex_sim.py` (FreeRTOS 선점 모델, 8시드 × 10분):

| 구성 | 손상 | 고착 | 고착된 회차 | 최빠른 고착 |
|---|---|---|---|---|
| 수정 전 (flush 만 락) | 7 | 7 | **6/8** | **80초** |
| 수정 후 (`_bbo_write` 가 락) | **0** | **0** | **0/8** | 없음 |
| 보호 없음(참고 상한) | 972 | 968 | 8/8 | 0초 |

배터리 측정은 두 구성 모두 952/952 성공 → 주기·표본을 줄이지 않아도 기아가 없다.

**실기(COM7)** — 콘솔 `cyc` 로 화면을 계속 갱신시켜 부하를 건 상태:

```
[OLEDMON]  60s  전송   837 / 실패 0  lockTO 0  present=1
[OLEDMON] 120s  전송 1,767 / 실패 0  lockTO 0  present=1
```

## 참고

- 배경 문서: README 「OLED 구동 방식 & 화면 정책」, `sim/tools/README_oled_sim.md`
- ★**로그 인코딩은 캡처 경로마다 다르다** — pyserial 직접 캡처 = UTF-8,
  빌드 로그(`logs/*.log`) = CP949. 틀리게 디코드하면 한글 에러가 0건으로 오독된다.
- ★**"실패 0" 은 정상의 증거가 아니다** — OLED 미검출이면 flush 를 건너뛰어 에러도
  안 생긴다. `present=1` 과 전송 카운터 증가를 함께 볼 것.
- 별개로, 충전측정을 껐던 시기에도 **장시간(7시간) 뒤 고착**된 사례가 있다.
  근본 대책은 모듈 VCC 를 GPIO+MOSFET 으로 제어해 자동 전원 리셋하는 것.
