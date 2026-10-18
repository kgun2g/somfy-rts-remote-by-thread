# 전체 펌웨어 절전 감사 — 2026-10-16

## 1. 결론과 이전 분석의 범위 정정

**15일을 못 넘기는 이유를 ‘버튼·진동 때문에 100ms마다 깨워야 한다’ 하나로 설명할 수 없다. 현재 코드에는 불필요한 반복 기상, 입력 전달의 지연·보존 문제, ADC가 강제로 유지하는 전원 도메인, 정상 운용 중 진단 저장이 함께 남아 있다.**

이전 답변은 절전의 주요 경로를 검토한 결과였다. 이번에는 main의 13개 C/C++ 구현 파일 전체와 25개 헤더, 보드 선택, 빌드·설정 파일을 검토하고, 호출되는 SDK의 전원·통신 경로까지 추적했다. 이번 보고서가 전체 펌웨어 범위의 종합 판단이다.

핵심 판정:

1. **HP 기상 최적화는 남아 있다.** LP 입력 루프에는 통지 대기 뒤 추가 10ms 지연이 있고 main/OLED OFF/hold도 주기적으로 깨어난다.
2. **100ms만 늘려서는 안 된다.** 디바운스 미확정 상태의 100ms 재대기, HP가 이미 기상한 동안의 LP 알림 빈틈, 짧은 탭 래치 소비, 로터리 누적량 소실이 먼저 해결돼야 한다.
3. **ADC는 별도 전력 문제다.** 기본 상주 ADC는 로컬 IDF 5.5.5에서 MODEM과 TOP을 ON으로 요청한다. 측정 주기를 늘려도 유닛이 상주하면 이 유지 요청은 없어지지 않는다.
4. **진단을 제품 경로에서 분리해야 한다.** 로그의 플래시 저장은 정상 대기 중에도 실행된다. 그러나 로그 기능을 통째로 끄면 LP 입력 전달 콜백까지 빠지는 결합이 있다.
5. **현재 자료로 ‘이 항목만 고치면 +몇 일’은 확정할 수 없다.** 기존 전류는 전압 기반 추정이며 새 WAKE 기록은 모의 실행이다. 확정 가능한 주기·횟수와 가정 기반 배터리 계산을 구분했다.

조건은 700mAh, 하루 20~50회 사용, OLED 10초 유지, 현재 SmartThings 지연 유지다. 펌웨어·설정·기존 도구를 수정하지 않았고 빌드, 플래시, 시리얼 연결, 실기 실험도 하지 않았다.

## 2. 분석 범위와 증거 수준

### 2.1 전체 펌웨어 파일 검토

구현 소스 13개, 합계 15,588행이다. 주석을 제외한 실행 코드, 함수 간 호출, 조건부 컴파일과 자원 수명을 검토했다. main의 텍스트 42개는 헤더·CMake·Kconfig·의존 manifest·빌드 시각 생성 스크립트를 포함한다. 글꼴 2개는 정적 데이터이며 태스크·타이머를 만들지 않는다.

| 구현 파일 | 행 수 | 확인한 전체 기능과 절전 관련 결론 |
|---|---:|---|
| app_main.cpp | 1,278 | 부팅, RF 초기화, Matter 엔드포인트·delegate, 콘솔, 공통 대기. 정상 앱을 만들고 남는 app_main은 10초마다 재개한다 |
| app_driver.cpp | 183 | 예제 조명 API. 정상 부팅에서 LED 초기화 호출이 없으며 WC 속성 처리와 구분됨. 파일 존재를 LED 상시 구동 증거로 볼 수 없음 |
| blind_manager.c | 420 | 주소·주파수·채널, 설정·롤링코드 저장. 송신 후 설정/전용 rollcode 두 파티션에 저장 |
| boot_diag.c | 279 | 부팅 단계·리셋·전압 진단. 단계 변경과 전압 최저값 50mV 갱신 때 저장, 고정 고속 타이머 없음 |
| button_handler.c | 1,713 | PCF 전체 입력, 로터리, 래치, 디바운스, 진동, LP 기동·폴백, 태스크·ISR. 기상·응답성 결합의 주요 위치 |
| cc1101.c | 567 | SPI, reset, 주파수, TX, SPWD, 핀 설정, 탐침. 정상 송신 끝에 SPWD 경로 있음 |
| efuse_commissionable.cpp | 99 | 커미셔닝 데이터 생성. 요청 시 계산·캐시, 상시 주기 작업 없음 |
| matter_blinds_shim.cpp | 289 | 앱 콜백, fabric·BLE 창, Thread on/off, OTA 조회. 무선 켜짐 요청 상태와 실제 적용 성공은 구분 필요 |
| oled_ui.c | 3,455 | 모든 화면, 비트뱅/HW I2C, 페이지 비교, 복구, ON/OFF, UI 태스크. OFF 주기와 실제 OFF 성공 확인 문제 |
| somfy_app.c | 6,315 | 모든 앱 상태·RF 큐·PM·배터리·시각·로그·딥슬립/LSF 진단·메인 루프. 가장 많은 기능이 100ms 루프에 결합됨 |
| somfy_rts.c | 580 | 프레임 생성, 반복·중단, 다단 tilt, RMT. 초기화 후 disable, 송신 후 disable 경로 있음 |
| thread_provision.c | 216 | dataset·부착·role·RSSI·삭제. 일부 함수가 중첩 잠금을 사용하지만 로컬 SDK 잠금은 recursive라 중첩 자체를 데드락으로 판정하지 않음 |
| lp_core/pcf_poll.c | 194 | LP 전체 프로그램. 5ms 설정, 1회 읽기 뒤 halt, 버튼 래치·회전 누적, raw 변화 때 HP wake 요청 |

전체 파일별 SHA-256, 행 수, 전원·태스크·타이머 제어 지점과 계산값은 [분석 범위 및 계산 원자료](D:/dev/workspaces/somfy-blinds-things-by-claude/doc/assets/power_saving_full_audit_2026-10-16/coverage_and_budget.json)에 보존했다.

### 2.2 빌드·라이브러리·시험 코드

- build-xiao-c6의 project_description과 compile_commands는 sdkconfig.xiao-c6, BOARD_XIAO_C6, LP 프로그램 포함, PCF8575, 외장 안테나 override를 가리킨다. 소스 기본 안테나는 내장이므로 빌드 override를 빼면 해석이 달라진다.
- sdkconfig와 생성된 sdkconfig.h를 대조했다. CPU 최대 80MHz, tick 1000Hz, tickless ON, CPU/TOP 차단 허용 ON, IEEE802154 sleep ON, Thread MTD, RX_ON_WHEN_IDLE OFF, ICD slow 7000ms / fast 200ms다.
- SDK 5.5.5의 ADC, SPI/RMT 보존·PM lock, 자동 light-sleep, LP 기동, Thread poll, ICD, BLE 종료, OpenThread 잠금, lwIP SNTP/DNS와 SSD1306 경로를 확인했다.
- SmartThings Lua 코드는 명령·속성 구독·수동 refresh 경로를 검토했다. 사용자 코드에 반복 polling timer는 없다. 허브 플랫폼이 실제로 협상한 구독·트래픽은 이 코드만으로 알 수 없다. 이 드라이버는 주석상 H2 3채널 composed용이라 C6에 실제 적용된 드라이버와 동일하다고 전제하지 않는다.
- test의 시험 진입점과 태스크·대기·송수신 경로를 전수 검색하고 정상 빌드 포함 여부를 확인했다. 현재 C6 compile_commands에는 시험 C 소스가 없다. 시험 파일의 무한 수신이나 CW 송신을 정상 대기 전류 원인에 합산하지 않는다.
- sim/도구는 실행 위치와 목적을 분류하고 절전·배터리·진동·ADC·태스크 모델 및 NVS 해석기를 검토했다. 전압 해석 오류의 재계산은 이전 감사 결과와 대조했다.
- CAD 생성 도구, 과거 백업 폴더, 생성 바이너리는 기기에서 실행되는 펌웨어가 아니다. SDK 전체·제3자 라이브러리 전체의 모든 행을 감사했다고 주장하지 않는다. 이번 범위는 **전체 자체 펌웨어와 실제 호출되는 의존 경로의 절전 감사**다.

현재 HEAD는 bafb638이며 제품 소스·설정 11개 파일의 기존 미커밋 변경이 있다. 소스 지문과 빌드 설정을 확인한 것이며 현재 실기에 올라간 바이너리가 동일하다는 검증은 아니다.

증거 수준은 ‘코드로 확인’, ‘기존 저장 기록’, ‘조건부 계산’, ‘실기 확인 필요’로 구분한다.

## 3. 현재 전력 구조

~~~mermaid
flowchart TD
    B[배터리 입력의 전체 소비] --> F[절전 중 유지되는 바닥 부하]
    B --> W[HP 절전 진입·복귀 및 작업]
    B --> A[실사용: OLED·RF·Matter 통신]
    F --> ADC[상주 ADC가 MODEM·TOP ON 요청]
    F --> LP[LP / LP I2C / PCF 주기 감지]
    F --> HW[분압·풀업·보드·주변 모듈]
    W --> BT[버튼: 100ms 통지 대기 + 10ms 추가 대기]
    W --> MAIN[main: 100ms]
    W --> UI[OLED OFF: 500ms]
    W --> HOLD[hold: 유휴 500ms]
    W --> NET[Thread·ICD·SNTP 및 SDK 시한]
    MAIN --> NVS[진단·시각의 주기 플래시 저장]
~~~

중요한 이름 구분: 앱의 _enter_sleep은 패널을 끄고 상태 플래그를 바꾼다. 그 함수에서 CPU를 직접 재우는 것은 아니다. 실제 CPU 절전은 _enable_pm_light_sleep의 PM 설정과 태스크·타이머 상태에 의해 자동으로 일어난다. 반대로 OLED가 켜진 10초 동안에도 조건이 맞는 짧은 틈에는 자동 light-sleep이 가능하다.

즉 ‘OLED OFF’, ‘PM 허용’, ‘CPU 실제 절전’, ‘TOP/MODEM 실제 차단’은 각각 확인해야 하는 다른 상태다. [main/somfy_app.c:1867](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:1867), [main/somfy_app.c:3912](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:3912).

### 3.1 전체 자체 태스크의 정상 대기

| 태스크 | 우선순위 | 입력 없는 동안의 대기 | 실제 역할 / 검토 방향 |
|---|---:|---|---|
| btn_handler | 10 | LP 유휴 100ms 통지 대기 뒤 공통 10ms | 단일 대기 지점과 디바운스 시한 필요 |
| rf_worker | 9 | RF 큐, 60초 timeout | timeout 때 CC1101을 깨워 탐침 후 다시 SPWD. 양산 진단 축소 후보 |
| hold_repeat | 8 | 기본 500ms | 아무 버튼이 없어도 반복. 눌림·조합·RF 완료 이벤트로 대기 가능 |
| somfy_app | 4 | 루프 끝 100ms | Thread 상태, PM 판정, 전원, 진동, 화면 시한, 배터리·로그·시각을 함께 처리 |
| oled_ui | 3 | ON 50ms / OFF 500ms | OFF 상태도 태스크 재개. UI 변경 통지와 상태별 시한으로 대체 후보 |
| app_main 잔여 루프 | 기본 main task | 10초 | 메모리 profiling OFF라 실질 내용은 없음. 낮은 우선순위 정리 후보 |
| defer_start | 4 | 부팅 중 500ms | 미등록 부팅·커미셔닝 감시 후 스스로 삭제. 정상 장기 대기 항목 아님 |

SDK 소유의 Thread, CHIP, lwIP, esp_timer, NimBLE/콘솔 태스크는 별도다. 코드가 라이브러리에 포함됐다는 것만으로 태스크가 계속 실행되거나 라디오가 켜져 있다고 계산하지 않는다.

유휴 버튼 반복은 약 110ms당 한 번이며 대기에서 돌아오는 지점은 두 곳이다. 작업·통지 없이 계산하면 버튼만 약 18.18회/초, main 10회/초, OLED와 hold 각각 2회/초의 **대기 복귀 지점**이 생긴다. 합계 약 32.18은 기존 정상 기록의 31~32회/초와 규모가 비슷하다. 다만 이 합이 CPU 기상 횟수의 정확한 분해는 아니다. 같은 기상에 여러 태스크가 함께 실행될 수 있다.

기존 저장 기록의 최신 정상 앱 비교 창은 COM7 약 32.10회/초·98.29%, COM8 약 31.75회/초·98.39%였다. 현재 코드와 실기 바이너리의 일치 및 새 전류는 이번에 측정하지 않았다. [기존 기록 재검증](D:/dev/workspaces/somfy-blinds-things-by-claude/doc/POWER_SAVING_AUDIT_2026-10-16.md).

## 4. 수정 검토 항목 — 우선순위와 근거

### F01. 상주 ADC가 MODEM과 TOP을 유지한다 — 최우선 전력 경로

**코드로 확인.** 기본 diag/adcfix=0이면 ADC 유닛을 생성해 유지한다. 로컬 IDF의 adc_oneshot_new_unit은 현재 TOP 차단 옵션에서 MODEM과 TOP을 모두 ON으로 요청한다. del_unit이 반대 요청을 한다.

- 5초마다 측정한다는 사실보다 **측정 사이에도 유닛이 존재한다는 사실**이 중요하다.
- 배터리 측정을 5초→60초로 바꿔도 상주 정책이면 도메인 유지 요청은 그대로다. 측정 작업량만 줄어든다.
- PM lock 덤프만 보면 놓칠 수 있다. 이 경로는 명시적인 도메인 전원 설정이기도 하다.
- 기존 adcfix=1 실험은 Thread가 멈춘 이력이 있어 기본 적용을 권할 수 없다. ADC 수명, 무선 보존, GPIO 상태와 정상 Thread를 한 묶음으로 해결해야 한다.
- SPI/RMT allow_pd 추가는 준비 조건이지 이 차단을 해제하는 변경은 아니다.

근거: [main/somfy_app.c:2433](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:2433), [main/somfy_app.c:2465](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:2465), [실제 ADC SDK](D:/dev/workspaces/esp-idf/components/esp_adc/adc_oneshot.c:171).

**효과량:** 상주 유지 요청의 존재는 확정. 몇 mA인지와 해결 후 추가 사용 일수는 미측정.

### F02. TOP 차단 뒤에는 핀 상태 보존도 같이 해결해야 한다

**코드·SDK 조건 확인, 정상 동작은 추가 검증 필요.** 정상 CC1101 핀 보정은 sleep 방향·pull 설정이다. 별도 진단은 TOP 실험에서 gpio_hold를 사용한다. 안테나 핀도 정상 경로에서는 sleep 선택 해제로 유지한다. TOP이 실제로 꺼질 경우 기존 GPIO 동작을 그대로 가정하면 안 된다.

ADC를 없앤 뒤 무조건 절전이 좋아진다고 보면 CC1101 CSn이 다시 떠서 칩이 깨어나거나 안테나·OLED 상태가 달라지는 역효과를 놓칠 수 있다. 송신 전에는 hold를 해제하고 정상 출력으로 되돌리는 수명 규칙도 필요하다.

근거: [main/cc1101.c:246](D:/dev/workspaces/somfy-blinds-things-by-claude/main/cc1101.c:246), [main/app_main.cpp:919](D:/dev/workspaces/somfy-blinds-things-by-claude/main/app_main.cpp:919), [main/somfy_app.c:4565](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:4565). [Espressif 전원 관리 — 주변 장치 차단과 GPIO 보존](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32c6/api-reference/system/power_management.html).

### F03. 디바운스 중에도 100ms 유휴 대기를 선택할 수 있다

**코드로 확인.** raw_state가 달라지면 디바운스 시각을 저장하고 기본 20ms 뒤 재확인해야 한다. 그런데 any_held는 확정된 last_state만 확인한다. 첫 변화를 봤지만 눌림이 미확정이면 또 100ms 대기를 고를 수 있다.

권장: ‘미확정 변화 있음’도 활성 처리로 보고 다음 디바운스 확인 시한을 예약한다. 입력 없는 동안의 안전망과 입력이 발생한 뒤의 판정 시한을 분리한다. ‘LP가 빠르게 봄’과 ‘버튼 이벤트 확정’은 같은 지연이 아니다.

근거: [main/button_handler.c:778](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:778), [main/button_handler.c:1016](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:1016).

### F04. LP 통지 대기 뒤 공통 10ms 대기가 남아 있다

**코드로 확인.** C6의 BOARD_PCF_INT_ISR=0 경로에서는 통지 대기에서 돌아온 뒤 10ms를 추가로 기다린다.

권장: 디바운스·누름·진동 시한을 계산해 한 곳에서 대기한다. 단순 삭제 후 진동 표본 시간을 그대로 두는 변경은 피한다.

**정량적 변화:** 조용한 유휴에서 명목 대기 복귀 지점은 약 18.18→10회/초로 줄어든다. 이는 CPU 기상·소비 전하의 실측 절감량이 아니다. [main/button_handler.c:1025](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:1025), [main/button_handler.c:1068](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:1068).

### F05. LP wake 요청은 모든 경우의 태스크 통지가 아니다

**코드로 확인.** 현재 연결은 LP raw 변화 → CPU wake 요청 → ULP 원인의 절전 복귀 콜백 → 버튼 태스크 통지다. HP가 이미 기상한 동안 같은 입력이 발생하면 이 콜백만으로는 태스크 통지를 보장하지 않는다.

권장: 공유 pending/seq, 생산·소비 확인, 잠들기 직전 변화와 이미 HP가 기상한 경우를 포함한 전달 규약을 설계한다. 실제 SDK 문맥에 맞는 방법을 검증한 뒤 안전망 timeout을 조정한다. 현재의 100ms를 무한 대기로 치환하는 것은 해결책이 아니다.

근거: [main/lp_core/pcf_poll.c:178](D:/dev/workspaces/somfy-blinds-things-by-claude/main/lp_core/pcf_poll.c:178), [main/somfy_app.c:2948](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:2948), [main/button_handler.c:1688](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:1688).

### F06. 짧은 탭 래치가 HP 디바운스 완료까지 입력을 보존하지 않는다

**코드상 누락 가능 경로 확인. 실제 빈도 미측정.** LP는 2연속 표본의 눌림을 OR로 기록하지만 HP는 읽는 순간 이를 지우고 한 번의 눌림 상태로 합친다. HP가 그때 raw_state를 바꾸고 다음 확인 때 이미 해제 상태를 읽으면 눌림 확정 전에 raw_state가 돌아갈 수 있다.

따라서 ‘래치가 있으니 HP 주기와 무관하게 짧은 탭 누락 0’은 현재 전체 경로에 대한 보장이 아니다. 눌림과 뗌의 순서·시각·소비 확인을 보존하고 판정 완료 뒤 소비해야 한다. PROG 글리치를 새 명령으로 만들지 않는 정책도 함께 필요하다.

근거: [main/lp_core/pcf_poll.c:170](D:/dev/workspaces/somfy-blinds-things-by-claude/main/lp_core/pcf_poll.c:170), [main/button_handler.c:618](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:618), [main/button_handler.c:778](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:778).

### F07. 로터리 누적값 여러 개를 읽어도 한 개만 전달될 수 있다

**코드로 확인.** HP는 rot_delta 전체를 차감한 뒤 최대 8회 이벤트 루프를 돈다. 그러나 루프 내 now는 같은 값이다. 첫 이벤트가 s_rot_last_ms=now를 기록하면 나머지는 30ms 제한을 통과하지 못한다. n이 2 이상일 때 모두 전달하는 루프가 아니다. 별도로 RF tilt는 전송 중 추가 요청을 버리는 정책도 있다.

권장: 디코딩 누적량, UI 조작량, RF 처리 가능한 조작량을 구분하고 잔여량을 남기거나 명시적으로 병합한다. 안전망 대기를 늘릴수록 여러 이동이 한 번에 쌓일 수 있어 먼저 해결해야 한다.

근거: [main/button_handler.c:838](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:838), [main/somfy_app.c:999](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:999).

### F08. LP/HP 공유값의 읽고-고쳐-쓰기는 상호 배제가 아니다

**경쟁 가능성 확인, 실기 손실 재현은 미실시.** HP의 press_latch &= ~lat와 rot_delta -= d, LP의 OR/증가 모두 공유값을 읽고 다시 쓰는 형태다. volatile과 ‘본 것만 차감’이라는 주석만으로 두 코어의 갱신을 원자적으로 보존하지 않는다.

권장: 단일 생산자가 쓰는 누적 카운터와 소비 위치, 또는 지원이 검증된 원자 연산·버퍼 규약으로 정리한다. HP 인터럽트만 막아도 LP가 멈추는 것은 아니다. 반대 방향 회전이 순누적으로 상쇄되는 정책과 overflow도 정의한다. 이는 절전 대기를 늘리기 위한 입력 안정성 전제다.

근거: [main/button_handler.c:620](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:620), [main/button_handler.c:841](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:841), [main/lp_core/pcf_poll.c:156](D:/dev/workspaces/somfy-blinds-things-by-claude/main/lp_core/pcf_poll.c:156).

### F09. 진동은 인터럽트 기반 감지와 폴링 기반 처리 전달이 섞여 있다

**코드로 확인.** ISR은 마스킹·계수만 수행한다. 버튼 루프가 계수 차분을 판정하고 메인이 다시 확인하여 화면을 켠다. ISR 재활성 3회, HIGH 창 30회, 고장 판정 200회는 실제 시간 대신 반복 수를 기준으로 한다.

HP가 유휴 약 110ms 간격이면 3/30/200표본은 대략 0.33/3.3/22초다. 통지와 부하에 따라 달라지므로 과거 주석의 30ms/300ms/2초를 현재 시간 보장으로 볼 수 없다. 특히 200표본 고장 판정이 늦어지면 GPIO HIGH 고정 wake 폭주를 끊기까지의 시간도 달라진다.

권장: 정상 이벤트 통지, 시간 기준 마스킹·재활성, 짧은 관찰, HIGH 고정 고장 차단·복구를 함께 설계한다. HP 기상 중 ISR 경로와 절전 복귀의 GPIO wake 원인 경로를 모두 확인한다. ISR 통지만 추가하면 잡음에서 기상이 늘 수 있다.

근거: [main/button_handler.c:928](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:928), [main/button_handler.c:965](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:965), [main/button_handler.c:1413](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:1413), [main/button_handler.c:1510](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:1510), [main/somfy_app.c:5869](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:5869).

### F10. main·OLED·hold의 유휴 반복이 서로 독립적으로 남아 있다

**코드로 확인.** main은 100ms마다 Thread 부착 상태를 읽고 전원·진동·시한을 확인한다. 화면 OFF UI는 500ms, hold는 눌림이 없어도 500ms다.

권장: main은 입력·전원·네트워크 상태 변경과 다음 처리 시한, UI OFF는 갱신 통지, hold는 눌림·조합·송신 완료를 기다린다. ADC·기록·RTC는 필요한 시한에 묶는다. RF와 hold를 하나로 합치면 진행 중 송신 중단·조합 전환 감시를 막을 수 있어 무조건적인 태스크 병합은 권하지 않는다.

근거: [main/somfy_app.c:1061](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:1061), [main/somfy_app.c:5438](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:5438), [main/somfy_app.c:6312](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:6312), [main/oled_ui.c:2913](D:/dev/workspaces/somfy-blinds-things-by-claude/main/oled_ui.c:2913).

### F11. 배터리 측정·진단 저장이 정상 대기 중에도 실행된다

**주기는 코드로 확인, 전류 기여는 미측정.**

| 작업 | 정상 유휴 주기 | 하루 명목 횟수 | 검토할 변경 |
|---|---:|---:|---|
| 배터리 ADC | 5초, 회당 8회 변환 | 측정 17,280회 / 변환 138,240회 | 화면 OFF에는 수십 초~수분, 입력·전원 전환에는 즉시 측정. ADC 상주 문제는 별도 해결 |
| batlog 주기 표본 | 시작 2분 뒤 120초 | 약 720회 | 개발용 상세 로그와 제품용 요약 분리 |
| vibelog 정지 heartbeat | 60초 | 약 1,440회 | 정상 정지 기록 축소, 고장·변화 중심 기록 |
| PM dump NVS 저장 | 배터리에서 60초 | 약 1,440회 | 개발 진단에서만 활성화하거나 간격 확대 |
| 시각 NVS 저장 | 300초 | 약 288회 | 허용 가능한 재부팅 후 시각 복원 오차에 따라 주기 조정 |
| CC1101 수면 탐침 | 큐 timeout 60초 | 최대 약 1,440회 | 초기 검증 후 개발용으로 축소 |

batlog는 11바이트×300칸=3,300바이트의 전체 버퍼, vibelog는 8바이트×128칸=1,024바이트 전체 버퍼를 저장 요청한다. 조용한 정상 대기의 명목 주기만으로 두 버퍼의 요청 payload는 약 3.85MB/일이다. PM 문자열·메타데이터·실제 flash erase/write 동작은 별도이며 요청 바이트와 물리 쓰기량은 같지 않다. 주기별 commit 경로 합은 약 3,888회/일로, 버튼·RF·부팅 기록은 제외했다.

이 횟수만으로 절감 mA를 정할 수 없다. 다만 실제 제품 대기 경로에 개발 진단 비용이 남아 있음은 확인된다. 모든 로그를 끄는 변경 대신 다음 F12의 기능 결합부터 해소해야 한다.

근거: [main/somfy_app.c:3070](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:3070), [main/somfy_app.c:3331](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:3331), [main/somfy_app.c:3367](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:3367), [main/somfy_app.c:5620](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:5620), [main/somfy_app.c:6203](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:6203).

### F12. 로그 옵션과 LP 입력 wake 전달이 결합돼 있다

**조건부 컴파일로 확인.** _ls_exit_cb와 등록 함수 _ls_stats_init은 BOARD_BATLOG_ENABLE 블록 내부에 있다. 이 콜백이 ULP 원인을 보고 버튼 태스크에 통지한다. 로그 OFF 분기에서는 _ls_stats_init이 빈 함수다. 일부 진단 함수 참조 조건도 함께 정리해야 하므로 단순 플래그 하나로 독립적인 ‘진단 없는 제품 빌드’가 완성되는 구조가 아니다.

권장: 입력 전달에 필요한 절전 콜백을 진단 계측과 분리하고, 로그·PM profiling을 꺼도 버튼 기능이 같도록 한다. 이전 보고서의 ‘진단 줄이기’는 이 분리를 전제로 해석해야 한다.

근거: [main/somfy_app.c:2815](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:2815), [main/somfy_app.c:2928](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:2928), [main/somfy_app.c:3283](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:3283).

### F13. 입력·Matter 콜백이 OLED 렌더를 직접 수행한다

**코드로 확인.** 첫 버튼/Matter 명령은 _exit_sleep을 거쳐 동기 화면 렌더를 먼저 하고 이후 RF 요청을 진행한다. 버튼 태스크는 우선순위 10이며 Matter 콜백은 SDK 실행 문맥이다. I2C 잠금·전송·복구 시간이 명령 경로에 들어간다.

권장: 명령·상태 이벤트를 짧게 기록하고 RF 요청과 UI 갱신을 각 작업에 전달한다. UI 태스크의 낮은 우선순위 격리와 버스 직렬화를 유지한다. 화면 첫 갱신이 늦어지지 않도록 UI 태스크를 통지로 즉시 깨워야 한다. 표시 상태·프레임버퍼의 소유권도 함께 정리한다.

근거: [main/somfy_app.c:1279](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:1279), [main/somfy_app.c:1772](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:1772), [main/somfy_app.c:3927](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:3927), [main/oled_ui.c:3345](D:/dev/workspaces/somfy-blinds-things-by-claude/main/oled_ui.c:3345).

### F14. OLED OFF 표시가 실제 OFF 성공의 증거는 아니다

**코드로 확인.** oled_ui_set_display_on은 소프트웨어 플래그를 먼저 바꾸고 0xAE/0xAF 전송 결과를 무시한다. 전송이 실패해도 코드와 로그에는 OFF로 남을 수 있다. 0xAE는 외부 모듈 전원 스위치 차단도 아니다.

또한 _bbo_write는 뮤텍스 확보 실패를 기록한 뒤에도 전송을 진행한다. 정상 ADC 경로는 잠금 실패 시 포기하는데, OLED 쪽 일부 경로는 같은 보호를 보장하지 않는다. 이 경합이 현재 실제로 발생한다는 측정은 없지만 상태 신뢰성과 복구에 영향을 줄 수 있다.

권장: ON/OFF 요청과 실제 전송 성공 상태를 분리하고 제한된 재시도·복구를 설계한다. 실패한 잠금에서 버스를 동시에 구동하지 않는 정책과 UI 단일 소유를 검토한다. OLED 10초 자체를 먼저 줄일 이유는 부족하다.

근거: [main/oled_ui.c:214](D:/dev/workspaces/somfy-blinds-things-by-claude/main/oled_ui.c:214), [main/oled_ui.c:3253](D:/dev/workspaces/somfy-blinds-things-by-claude/main/oled_ui.c:3253).

### F15. 설정 화면·페어링 실패 상태는 10초 화면 OFF의 예외다

**코드로 확인.** _in_setup_mode이면 화면 OFF를 억제하고 오래 유휴이면 활동 시각도 갱신한다. 메뉴·시각 설정·Thread reset 확인·FW 화면을 켠 채 두면 일반 10초 정책이 적용되지 않는다. FREQ 화면은 UI 상태만 20초 후 NORMAL로 바뀌어도 s_setup_screen이 그대로면 메인의 억제 조건이 남을 수 있다.

페어링 보호 CPU lock은 페어링 화면을 떠날 때 해제된다. 실패 화면에 계속 머물면 보호 상태가 유지되는 경로가 있다.

권장: 배터리에서 설정 세션 전체에 명시적 유휴 종료를 두고 변경 저장·취소 정책을 정의한다. 페어링 성공/실패/취소/시간 초과에서 보호 lock을 해제한다. 실제 OTA 다운로드·설치 중에는 별도 보호 정책이 필요하다.

근거: [main/somfy_app.c:2039](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:2039), [main/somfy_app.c:5558](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:5558), [main/somfy_app.c:5904](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:5904), [main/oled_ui.c:2878](D:/dev/workspaces/somfy-blinds-things-by-claude/main/oled_ui.c:2878).

### F16. USB 물리 판정과 앱의 안정화 판정이 일치하지 않는다

**코드로 확인.** PM 설정은 raw GPIO 기반 btn_handler_is_charging을 사용한다. 로그 세션은 별도의 2초 USB 분리 확인을 한다. VBUS가 흔들리면 기록만 안정화돼도 PM·화면 정책은 순간 판독의 영향을 받을 수 있다. 반면 XIAO의 CHG_STAT 레벨 wake를 끈 기존 수정은 유지돼 있다.

권장: 실제 전원 변화를 판정하는 한 곳에 시간 기준 안정화와 이벤트 전달을 두고 PM·UI·로그가 그 결과를 사용한다. 과거 폭주를 일으킨 GPIO level wake를 단순 재활성화하지 않는다. 실제 GPIO17 신호가 안정적인지는 하드웨어 측정 영역이다.

근거: [main/somfy_app.c:1942](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:1942), [main/somfy_app.c:5996](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:5996), [main/button_handler.c:1382](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:1382).

### F17. Thread 상태 조회·SNTP는 기능을 유지하며 정리할 여지가 있다

main은 100ms마다 Thread 부착 상태를 조회하지만 일반적인 UI 갱신은 7초/60초 등 별도 시한을 쓴다. 상태 변경 통지나 다음 처리 시한으로 충분한 조회를 10회/초 할 필요는 없다. 이 getter 자체가 매번 무선 패킷을 보낸다는 의미는 아니다.

_start_app_tasks_once는 시각 기능이 켜져 있으면 미등록 상태에서도 SNTP를 시작한다. 25초 유예 뒤 미등록 기기의 Thread/BLE를 끄는 정책은 별도로 있다. SNTP가 도달 불가 상태에서 계속 DNS/재시도 시한을 가질 수 있으므로 네트워크 준비·무선 상태에 맞춰 시작·정지·재개해야 한다. 연결 재부착마다 불필요한 재시작도 줄일 후보다.

기존 slow 7초 / fast 200ms는 그대로 유지한다. 부모 재탐색, 실제 구독, 재전송으로 fast 상태가 길어지는지는 실기 트래픽을 봐야 한다. 무선 신호가 약하다고 무조건 TX power를 낮추면 재시도로 에너지가 늘 수 있다.

근거: [main/somfy_app.c:2007](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:2007), [main/somfy_app.c:2139](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:2139), [main/somfy_app.c:5438](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:5438), [main/somfy_app.c:6151](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:6151).

### F18. 주석·통계·기존 시뮬레이션을 현재 동작 증거로 사용하면 안 된다

- btn_handler_poll_ms는 LP 활성 시 여전히 25를 반환한다. 실제 유휴 timeout은 100ms다.
- LPWAKE의 ‘HP 깨우기’ 출력은 seq 증가량인데 seq는 로터리 누적과 raw 변화 양쪽에서 증가한다. 실제 CPU wake 횟수와 같지 않다.
- btn_poll_period_sim은 래치·현재 디바운스·콜백을 통합한 모델이 아니다. 마지막 ‘래치면 누락 0’ 문구는 F06을 증명하지 못한다.
- vibe_wake_sim은 10ms 폴링을 전제하여 현재 C6 LP 유휴 경로와 다르다. 고장 후 기상을 세는 guard의 latched는 false로 초기화되고 true로 갱신되지 않으므로 woke_after가 0인 결과 자체가 고장 후 차단을 검증하지 못한다.
- 기존 lsfloor 해석의 신뢰 상한 시간축 오류, 모든 wake에 703us 적용, 세션·PM 계수의 다른 분모는 이전 감사에서 확인했다. 실제 표본 시각 재계산을 참조한다.
- 배터리 %는 전압표·평활·하한·속도 제한을 거친 UI 값이다. 이를 그대로 전류계처럼 사용하면 표시 알고리즘 변경을 소비 개선으로 오인할 수 있다. dsleep의 ‘전압 기울기/0.915’와 그 결과의 기판/칩 자동 범인 판정도 직접 전류 측정에 해당하지 않는다.

근거: [main/button_handler.c:1705](D:/dev/workspaces/somfy-blinds-things-by-claude/main/button_handler.c:1705), [main/somfy_app.c:6253](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:6253), [sim/tools/vibe_wake_sim.py:185](D:/dev/workspaces/somfy-blinds-things-by-claude/sim/tools/vibe_wake_sim.py:185), [main/somfy_app.c:4324](D:/dev/workspaces/somfy-blinds-things-by-claude/main/somfy_app.c:4324), [기존 해석 검증](D:/dev/workspaces/somfy-blinds-things-by-claude/doc/POWER_SAVING_AUDIT_2026-10-16.md).

## 5. 원인으로 단정하지 않은 항목과 하드웨어 경계

| 항목 | 전체 검토 후 판단 |
|---|---|
| BLE가 항상 켜짐 | CONFIG_USE_BLE_ONLY_FOR_COMMISSIONING=y, SDK에 커미셔닝 후 Shutdown 경로가 있다. BT sleep OFF만 보고 정상 대기의 상시 BLE 소비를 확정하면 틀림 |
| CC1101이 항상 IDLE | 초기화·송신 뒤 SPWD 경로와 CSn 보정이 있다. 옛 문제를 현재 코드에도 그대로 있다고 주장할 수 없음. 물리 핀·실제 SPWD는 별도 측정 |
| RMT가 계속 enable | 초기화 마지막과 송신 마지막에 disable. 정상 유휴의 상시 RMT lock으로 확정할 수 없음 |
| LED 예제 파일 존재 | 정상 부팅은 app_driver_light_init을 호출하지 않는다. 파일이나 led_strip 의존성이 있다는 이유만으로 LED 전류를 계산하지 않음 |
| Insights 라이브러리 존재 | 실제 CONFIG_ESP_INSIGHTS_ENABLED는 OFF. 정상 전송을 시작한 증거 없음 |
| LP 코어가 매 5ms마다 HP도 깨움 | LP는 1회 처리 후 halt, raw 변화 때 명시적 HP wake 요청. LP 타이머 자체가 같은 속도로 HP를 깨운다는 해석은 맞지 않음 |
| 미등록 상태에서 무선 영구 ON | 부팅 유예 25초 뒤 미등록·페어링 화면 아님이면 무선 OFF 정책이 실제로 있다 |
| task 수만 줄이면 저전력 | 독립 UI·RF·hold는 응답성과 중단 처리 이유가 있음. 우선 유휴 예약을 줄여야 함 |
| battery 설정 500mAh | 보드의 500은 시간 기반 충전 추정식에 쓰인다. 로그 쪽 BAT_CAPACITY_MAH는 700. 이 불일치를 고쳐도 실제 셀 용량이나 전류가 자동 개선되는 것은 아님 |

배터리 분압 100k+100k는 회로가 정의대로 연결됐을 때 3.7V에서 약 18.5uA, 하루 약 0.444mAh다. 이 경로 하나를 완전히 없애도 15일 동안 약 6.66mAh로, 가용 560mAh의 약 1.19%다. 수 mA 격차의 주원인으로 삼기는 어렵다.

진동 입력에는 내부 pull-up이 있고 센서는 정상 정지 LOW로 처리한다. 따라서 정상 상태에서도 pull-up 전류 경로가 있다. 실제 저항·공급 전압·다른 회로를 측정하지 않았으므로 임의의 mA로 확정하지 않는다. 외부 pull-up·RC·센서 교체는 하드웨어 선택지이며 당장 제거하면 감지 기능을 잃을 수 있다.

OLED 모듈·PCF·CC1101 모듈의 보조 IC·레귤레이터·LED, XIAO 전원 회로와 셀 유효 용량은 소스만으로 측정할 수 없다. OLED는 모듈 전원 차단 핀이 정의돼 있지 않고 명령으로만 OFF한다. 밝기 0xCF 조정은 10초 표시를 유지하면서 활성 전하를 줄일 후보지만 선형 비율의 절감을 보장하지 않는다.

## 6. 15일 예산과 수정별 확보 가능량

### 6.1 먼저 충족해야 할 전체 전류

700mAh × 사용 가능 비율 80% = 560mAh 가정이다. 배터리 입력 기준 전체 평균으로 비교한다. 3.3V 레일 전류를 배터리 mAh에 그대로 대입하면 전원 변환 손실·전압 차이를 놓친다.

| 전체 평균 | 하루 사용량 | 560mAh 기준 예상 시간 |
|---:|---:|---:|
| 1.0mA | 24.0mAh | 23.3일 |
| 1.3mA | 31.2mAh | 17.9일 |
| 1.5mA | 36.0mAh | 15.6일 |
| **1.556mA** | **37.33mAh** | **15.0일** |
| 2.0mA | 48.0mAh | 11.7일 |
| 3.0mA | 72.0mAh | 7.8일 |
| 4.56mA | 109.44mAh | 5.12일 |

4.56mA는 과거 기록의 전압 기반 모델 기준이며 현재 빌드의 새 직접 측정값이 아니다. 이를 참고 기준으로 삼으면 하루 약 72.11mAh, 평균 약 3.00mA를 줄여야 한다. 단지 OLED 타이머를 조금 줄이는 규모와는 차이가 크다.

### 6.2 OLED 10초를 유지할 때의 예시 예산

‘화면 사용 중 장치 전체가 유휴보다 배터리 입력에서 50mA 더 사용’한다고 **가정**한다. OLED 단독 사양이나 이 기기의 실측값이 아니다.

| 하루 화면 사용 | ON 시간 | 추가 소비/일 | 평균 추가 전류 | 15일 예산 중 비중 | 유휴·나머지 활동의 잔여 평균 예산 |
|---:|---:|---:|---:|---:|---:|
| 20회 | 200초 | 2.78mAh | 0.116mA | 7.4% | 1.440mA |
| 50회 | 500초 | 6.94mAh | 0.289mA | 18.6% | 1.266mA |

RF·통신 활동이 이 가정에 포함되지 않으면 그 전하는 따로 차감해야 한다. 화면을 10초→5초로 줄일 때 확보량도 이 가정에서 하루 약 1.39~3.47mAh다. 과거 기준과 목표의 하루 72.11mAh 격차 전체를 해결하지 못한다.

### 6.3 기상 횟수 감소가 며칠로 이어지는가 — 민감도

절전 기상 1회가 없애 주는 ‘추가 전하’를 q라고 할 때, 기상 30회/초 감소의 절감 평균은 30×q/1000 mA다. 바닥 전류·통신·실사용은 같다고 가정한다.

| 가정 q | 가정 절감 | 과거 4.56mA 모델 대비 감소 | 조건부 잔여 평균 | 조건부 사용 시간 |
|---:|---:|---:|---:|---:|
| 1uC/회 | 0.03mA | 0.66% | 4.53mA | 5.15일 |
| 10uC/회 | 0.30mA | 6.58% | 4.26mA | 5.48일 |
| 50uC/회 | 1.50mA | 32.89% | 3.06mA | 7.63일 |
| 100uC/회 | 3.00mA | 65.79% | 1.56mA | 14.96일 |

**표의 어느 q도 현재 기기의 실측값으로 채택하지 않았다.** 기상 정리만으로 15일 가까이 가려면 대략 100uC/회 수준의 추가 비용을 30회/초 줄이는 규모가 필요하다는 뜻이다. 이 표는 기대 일수 예측이 아니라 결론이 미지의 기상 전하에 얼마나 민감한지 보여 준다. 이전 보고서의 50uC 가정도 같은 한계를 가진다.

### 6.4 수정 후보별 지금 말할 수 있는 확보량

| 수정 후보 | 확정 가능한 변화 | 전류·시간 확보량을 정하는 추가 값 |
|---|---|---|
| F03/F04 입력 시한·이중 대기 | 불필요 10ms 대기 제거, 조용한 유휴의 명목 복귀 지점 약 8.18회/초 감소 | 실제 CPU 기상 감소 × 조건별 추가 전하 |
| F10 main 이벤트 대기 | 유휴 100ms 확인을 줄일 수 있음 | 새 필수 시한과 실제 기상 차분 |
| F10 OLED OFF 통지 대기 | OFF 상태 2회/초 예약을 줄일 수 있음 | 다른 태스크와 동시 실행된 비율, 추가 전하 |
| F10 hold 유휴 대기 | 미눌림 2회/초 예약을 줄일 수 있음 | 송신 완료 통지와 조합 전환 검증 |
| F11 OFF ADC 5→60초 예시 | 측정 회수 17,280→1,440/일, **91.7% 감소** | 회당 측정 전하. 상주 유지 전류에는 변화 없음 |
| F11 기록 60→600초 예시 | 해당 기록 주기 횟수 **90% 감소** | flash 저장·erase·부수 작업의 실제 전하 |
| F01 ADC/전원 수명 개선 | MODEM/TOP ON 요청을 해소할 설계 가능성 | 정상 Thread와 GPIO를 유지한 유휴 평균 차이 |
| OLED 10→5초 | 위 50mA 가정에서 1.39~3.47mAh/일 | 실제 활성 추가 전류. 사용자 요구상 우선 변경하지 않음 |
| OLED 밝기 조정 | 같은 10초 동안 활성 소비를 줄일 후보 | 가독성과 실제 활성 전하 |
| LP 주기 확대 | 거래 횟수 감소 가능 | 로터리 전이 손실·에너지 검증 필요. 우선 제외 |

태스크 기상 절감, ADC 수명, 도메인 차단은 서로 영향을 주므로 가정 효과를 단순 합산하면 안 된다. ‘측정 횟수 91.7% 감소’는 ‘장치 배터리 사용량 91.7% 감소’가 아니다.

## 7. 제가 정리할 구조와 순서

~~~mermaid
flowchart LR
    LP[LP: 5ms 입력 감지 유지] --> MB[입력 보존·생산/소비 규약]
    MB --> IN[입력 처리: 확정 시한에만 실행]
    VG[진동 ISR·GPIO wake] --> VF[시간 기준 필터·고장 차단]
    VF --> APP[앱 상태·화면 종료 시한]
    IN --> APP
    IN --> RF[RF 큐·독립 송신 작업]
    RF --> DONE[송신 완료 이벤트]
    DONE --> HOLD[눌림·조합 상태 작업]
    IN --> HOLD
    APP --> UI[통지로 즉시 깨어나는 독립 UI]
    NET[현재 Thread·Matter 수신] --> APP
    NET --> RF
    APP --> BG[배터리·시각·요약 기록: 필요한 시한]
~~~

| 단계 | 작업 묶음 | 먼저 통과해야 할 조건 |
|---|---|---|
| 1 | 통계 이름·시간 창·시뮬레이션 오류 정정, 소스·바이너리·NVS 상태 식별 | 같은 조건에서 전후 비교 가능 |
| 2 | 입력 디바운스 시한, 단일 대기, 짧은 탭·회전 보존 규약 | 화면 OFF/ON·HP 기상/수면·빠른 탭·회전·PROG·롱프레스 모두 유지 |
| 3 | 진동을 시간 기준 이벤트 처리로 전환 | 첫 두드림, 정지 오검출, HIGH 고장, 복구와 잡음 폭주 차단 |
| 4 | 앱 상태·UI·hold의 통지/시한 대기, 기능과 진단 분리 | RF 프레임 경계 중단·조합 전환·화면 10초·전원 연결/분리·5초 WDT 정책 유지 |
| 5 | 진단 저장·OFF ADC 측정 축소 | 개발 진단을 줄여도 입력 wake와 복구가 동일 |
| 6 | ADC·MODEM/TOP·GPIO 보존을 정상 Thread에서 함께 해결 | 지속 수신, 재부착·재전송, RF 재송신, OLED wake, LP 동작 및 전원 플래그 확인 |
| 7 | 하루 20~50회 실제 사용을 포함한 배터리 평가 | 전체 평균 1.556mA 이하 또는 유효 방전 시험 15일 이상 |

스마트싱스 slow/fast poll과 OLED 10초, LP 5ms는 우선 유지한다. 단계 2~5는 반응성을 유지한 HP 기상 정리이고 단계 6은 절전 중 바닥 전류 정리다. 두 축 모두 확인한 뒤 15일 가능성을 판단한다.

반드시 기록할 것은 물리 입력 시각, LP 관측·이벤트 보존, HP 소비, 디바운스 확정, RF 요청·실제 출력, 화면 첫 갱신이다. 평균만 아니라 누락과 느린 경우도 확인한다. 직접 전류 측정은 배터리 경로의 전하를 적분하고, 전압법만 사용하면 셀 곡선·충전 후 완화·부하 변화·온도·세션 경계를 함께 한계로 남긴다.

## 8. 최종 판단

현재의 병목은 ‘깨워야 해서 절전할 수 없다’가 아니라 **입력 감지, 이벤트 전달, 판정, 화면·통신 처리와 진단이 주기 확인에 결합돼 있고, 동시에 ADC가 절전 중 전원 영역을 유지하는 구조**다.

따라서 100ms를 500ms로 늘리는 식의 변경은 권하지 않는다. 입력 전달·보존·판정과 진동 시간 기준부터 바로잡고 유휴 반복을 줄인 뒤, 정상 Thread를 유지하면서 ADC·도메인·핀 수명을 해결하는 순서가 적절하다. 그 결과의 mA가 아직 없으므로 15일 불가능도, 특정 변경만으로 15일 달성도 현재 코드만으로 확정하지 않는다.

기존 기록·수치 검증은 [10월 16일 실측 해석 감사](D:/dev/workspaces/somfy-blinds-things-by-claude/doc/POWER_SAVING_AUDIT_2026-10-16.md), 입력 경로 중심의 이전 설계 메모는 [이벤트 설계 분석](D:/dev/workspaces/somfy-blinds-things-by-claude/doc/POWER_SAVING_EVENT_DESIGN_2026-10-16.md)에서 확인할 수 있다. 이번 문서는 그보다 넓은 전체 자체 펌웨어 범위를 다룬다.

공식 근거: [ESP-IDF 5.5.5 전원 관리](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32c6/api-reference/system/power_management.html), [ESP-IDF LP 코어](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32c6/api-reference/system/ulp-lp-core.html), [TI CC1101 데이터시트](https://www.ti.com/lit/ds/symlink/cc1101.pdf). 실제 프로젝트 동작 판단에는 위에서 링크한 로컬 SDK와 현재 소스를 우선했다.
