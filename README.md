# Somfy RTS 우드 블라인드 컨트롤러 — ESP32 + CC1101

ESP32(C6 기본) + CC1101(447 MHz 2-FSK / 433 MHz OOK) + 0.42" OLED +
PCF8574 버튼/로터리로 **Somfy RTS** 블라인드(C6 8개 / H2 3개)를 제어하고,
**Matter-over-Thread**(SmartThings)로 노출하는 펌웨어.

한국 베네치아(Venetian) 블라인드의 **447 MHz 2-FSK 변종**을 정품 리모컨 IQ
녹음으로 역분석·검증해 송신한다(UP/DOWN/MY/PROG/TILT). 보드는 `boards/<board>.h`
핀맵·디스플레이·I2C 구성까지 보드별 교체 가능 — 브랜드-SoC 단위:
**3보드 모두 검증** — gnpe-c6(GNPE ESP32-C6-0.42)·xiao-c6(Seeed XIAO
ESP32-C6)·esp32-h2(ESP32-H2 SuperMini, I2C 공유).

레포: <https://github.com/kgun2g/somfy-rts-remote-by-thread>

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
&nbsp; 무료/오픈소스 — [MIT 라이선스](LICENSE).

## 주요 기능

- **Matter (보드별 구조)**: C6 = Bridge(Aggregator + 8× Bridged WindowCovering) →
  SmartThings 에 블라인드 8개 독립 타일 / H2 = composed(root 직속 3× WindowCovering,
  RAM 절약) → custom Edge driver 로 1카드 3블라인드. 둘 다 lift/tilt.
- **Somfy RTS 447 송신**: CC1101 2-FSK + RMT 비트뱅, 80비트(10바이트) 프레임,
  롤링코드, 블라인드별 주파수. Manchester 극성·HW/SW sync·byte 7~9·tilt 커맨드
  모두 정품 IQ 검증값. 한 누름 = **2 프레임**(첫 프레임 HW sync 12 + 재전송 6),
  **byte7 은 재전송 인덱스**(첫 프레임 `0x84`, 재전송 `196+rep*4`) — 정품·ESPSomfy 동일.
  > ★**rtl_433 이 보여주는 byte 값은 wire 바이트가 아니다.** 송신은 b[1..6]만
  > 체인 XOR 하지만 rtl_433 은 **b[1..9] 전체를 디스크램블**한다
  > (`표시 b8 = wire b8 ^ wire b7`, `표시 b9 = wire b9 ^ wire b8`).
  > 이걸 모르면 재전송 프레임의 byte7 을 "hold 코드" 로 오독한다 — 실제로 그렇게
  > 오독해 멀쩡하던 긴 누름을 깬 적이 있다. 자세한 건 `HANDOFF.md`.
- **OLED UI**: 동작 모션(UP/DOWN/STOP/TILT/PROG/ROT), 시계, 대상 블라인드 표시,
  설정 메뉴. **렌더러는 해상도로 자동 선택**(보드 무관, `BOARD_OLED_*`):
  **128×64 가로** / **64×128 세로** → 풀스크린 네이티브(고딕 폰트 + 7세그 시계),
  **그 외**(GNPE 72×40 등) → 72×40 논리 캔버스(어떤 패널에도 중앙 배치).
- **로터리 인코더**: detent 잠금 + 이탈 첫엣지 방향결정 + 2-연속표본
  디바운스(노이즈 내성). 클릭당 1동작(틸트 업/다운), 클릭=STOP/MY.
- **PCF8574 버튼**: UP/DOWN/SELECT/PROG/SETUP.
- **진동 wake**: JYX-1210-X160 진동 스위치(IO16). 절전/화면보호기 중 흔들면
  즉시 메인 화면 복귀.
- **설정 메뉴**: Cancel / Freq Edit / Time Set / Matter Pair / Thread Rst /
  FW Update (C6 6항목; **H2 는 Time Set·FW Update 제외 4항목**). SET 짧게=저장/실행, STOP=취소→메뉴.
- **기기별 고유 블라인드 ID + 블록 ALL**: 블라인드 주소를 ESP32 **eFuse 팩토리 MAC**
  (칩 고유·불변)에서 `F0+등차+ID` 형식으로 산출 → 여러 대를 만들어도 충돌 없음,
  OTA·factory reset 에도 불변. **4채널 = 1블록**, 블록마다 전용 ALL(채널 등차의 다음
  항)을 자동 산출 → ALL 명령 시 블록별 ALL 을 차례로 송신(채널↑ → 블록·ALL 자동↑).
  ([기기 고유 ID & 롤링코드 영속](#기기-고유-id--롤링코드-영속))
- **기기별 고유 커미셔닝 코드**: Matter 페어링 코드(discriminator/passcode)도 같은
  **eFuse MAC** 에서 산출(`efuse_commissionable.cpp`, custom CommissionableDataProvider)
  → 기기마다 다른 QR/PIN, `fctry` 플래시 없이 자동 고유. 여러 대 동시 페어링도 충돌 없음.
- **롤링코드 factory-reset 영속**: 롤링코드를 기본 NVS 와 분리된 전용
  `rollcode` 파티션에도 보존 → Matter factory reset(기본 NVS 삭제) 후에도
  모터가 "롤링코드 역행"으로 거부하지 않음.
- **3단계 화면보호기/절전**: 정상 → (유휴) 화면보호기 애니메이션(패널 ON)
  → 패널 OFF/절전. 버튼/진동/Matter/USB 로 즉시 복귀.
- **배터리 절전(2026-08-25 실측)** — 두 보드 모두 **light sleep 96 %대**:

  | | XIAO-C6 | ESP32-H2 |
  |---|---|---|
  | 잠 | **96.2 %** | **96.6 %** |
  | 깨어남 | **23.7 회/초** | **33.2 회/초** |
  | 깨우기 방식 | LP 코어 눌림 래치 + ULP 깨움 | PCF8574 `~INT` 레벨 트리거 |

  C6 는 깨어있는 시간 **13.4 % → 3.6 %**, 방전 **14.1 → 8.7 mV/시간**
  (700 mAh 로 **약 5일**). H2 는 **0.0 % → 96.6 %** — light sleep 이 한 번도 안 걸리던
  상태였다(`bt` PM 락).
  버튼 폴링을 늦췄는데도 **반응은 오히려 빨라졌다** — 실제 입력은 LP 코어(C6) 또는
  `~INT`(H2) 가 즉시 깨우고, 폴링은 안전망 역할만 한다.
  자세한 근거·측정법은 [`HANDOFF.md`](HANDOFF.md) 「절전 측정 현황」.
- **안전망**: 메인 루프 + 버튼 태스크 Task WDT 감시(hang 시 자동 리부트),
  RTC 메모리 crash breadcrumb(비정상 재부팅 시 부팅 직후 진단 로그).
- **펌웨어 업데이트(Matter OTA over Thread)**: 듀얼 OTA 파티션 + Matter OTA
  Requestor. Provider 가 `.ota` 이미지(버전↑) 배포 → 자동 다운로드/적용.
  설정 메뉴 `FW Update` 에서 버전·진행 확인. 절차: [`doc/OTA.md`](doc/OTA.md).

## SmartThings 제어 동작

| 조작 | RF 동작 |
|---|---|
| **Lift 슬라이더** | 방향 기반 — target>current=DOWN(닫힘), <=UP(열림). 모터가 그 방향대로 끝까지 주행(위치 피드백 없음). |
| **Tilt 슬라이더** | 정품 Tilt 커맨드(cmd nibble 0xB). delta 를 **7단계**로 환산해 N step burst(슬랫 각도). |
| **일시정지(Stop)** | Somfy MY 송신. 단, 직전 movement 500 ms 이내 auto-stop 은 무시. |
| **Open/Close** | Lift 먼저 송신, 동반 Tilt 는 companion 으로 skip(400 ms). |

> SmartThings WC UI 는 Lift/Tilt 슬라이더가 미러링돼 한쪽만 만져도 양쪽
> 명령이 온다 → 400 ms companion 가드로 먼저 들어온 쪽만 RF 송신(짧게만
> 움직이던 문제 해결). 두 슬라이더를 의도적으로 따로 조작하려면 400 ms 이상
> 간격을 둔다.

## 하드웨어 핀 배선

핀맵·디스플레이·I2C 구성의 단일 진실원천은 **`main/boards/<board>.h`**
(`BOARD_PIN_*` / `BOARD_OLED_*` / `BOARD_I2C_SHARED`). 다른 보드는
`boards/<board>.h` 추가 후 `-Board <name>` 으로 빌드.

### 지원 보드

| 보드 | SoC | 디스플레이 | I2C 버스 | PID | 배선문서 | 상태 |
|---|---|---|---|---|---|---|
| **GNPE ESP32-C6-0.42** | C6 | 내장 0.42″ 72×40 (180° 회전) | 분리(OLED HW + PCF 비트뱅) | `0x8000` | [gnpe-c6](doc/wiring/wiring_gnpe-c6.md) | ✅ 검증 |
| **Seeed XIAO ESP32-C6** | C6 | 외부 0.96″ 128×64 (정방향) | **LP_I2C→공유 자동폴백** | `0x8003` | [xiao-c6](doc/wiring/wiring_xiao-c6.md) | ✅ 검증 |
| **ESP32-H2 SuperMini** | H2 | 외부 모듈 | **공유**(OLED+PCF 한 HW I2C) | `0x8001` | [esp32-h2](doc/wiring/wiring_esp32-h2.md) | ✅ 검증 |

- **디스플레이 규격**은 보드별 `BOARD_OLED_*`(해상도/오프셋/회전/72×40 보정/주소).
  **렌더러는 해상도로 자동 선택**(`oled_ui.h` 의 `OLED_RENDER_*`) — 128×64 가로·64×128
  세로는 풀스크린 네이티브, 그 외는 72×40 논리 캔버스를 중앙 배치. 어느 보드든
  `BOARD_OLED_*` 만 바꾸면 OLED 교체 가능(코드 수정 불필요).
- **I2C 버스**: GNPE 는 OLED=HW I2C + PCF8574=비트뱅(서로 다른 핀). **H2 는
  `BOARD_I2C_SHARED=1` 로 OLED·PCF8574 가 HW I2C 한 버스 공유**. **XIAO 는 런타임 자동
  폴백**(`BOARD_I2C_LP_FALLBACK=1`): LP_I2C(뒷면 6/7) 우선 프로브 → 무응답 시 공유 HW
  I2C(22/23)로 전환 — 두 배선 모두 한 펌웨어로 지원.
- 아래 두 표는 **제품(보드)별** 핀 배치다. 단일 진실원천은 각 `boards/<board>.h`,
  상세 패드·실크 매핑은 [배선문서](doc/wiring/)를 따른다. 표기는 제품 실크 기준
  (GNPE `IOx` · XIAO 패드 `Dx`/뒷면 `MTxx` · H2 `GPx`), 괄호는 GPIO 번호.
  **★ = 기준(대표) 보드 — 상세 배선·이미지 기준(GNPE)**.

### 보드별 펌웨어 기능 차이

같은 펌웨어라도 보드 헤더(`boards/<board>.h`)의 매크로로 SoC·메모리·디스플레이에
맞춰 기능이 분기한다.

> **⚠️ H2 가 지금 형태로 구현된 근본 이유 = RAM 부족.** ESP32-H2 는 가용 RAM 이
> **~145 KB** 뿐이라 OpenThread **FTD** + Matter 스택만으로 free heap 이 거의 소진된다.
> 실측: Bridge(Aggregator+bridged) 구조로 블라인드 5개를 노출하면 `bad_alloc`/CHIP task
> 생성 실패, 3개여도 `somfy_app` 태스크가 free<5 KB 로 못 뜨고, BLE 커미셔닝 중에는 free 가
> **~2.5 KB** 까지 떨어져 PASE peak(20~30 KB)를 못 버틴다. 그래서 H2 펌웨어는 **기능을
> 빼서가 아니라 메모리가 허락하는 범위에 맞춰** 다음과 같이 축소·단순화한 형태로 구현됐다
> (C6 두 보드는 RAM 여유로 이런 제약이 전혀 없다):
>
> - **composed 구조**(Bridge → root 직속 WindowCovering): free 2.5 KB → **~72 KB** 회복
> - **채널 3개 / 블라인드 2개 안정 상한**: endpoint 가 늘수록 heap 압박 — 3개는 재커미셔닝(PASE peak) 때 부족할 수 있어 2개가 안전
> - **시각·OTA 기능 제거**: time 태스크·SNTP·OTA Requestor 를 빼 커미셔닝용 heap 확보 (`BOARD_DISABLE_TIME`/`BOARD_DISABLE_OTA`)
> - **CHIP PacketBuffer 풀 24 + CHIP task priority 15**: NimBLE mbuf·PacketBuffer 부족으로 PASE 가 실패하지 않도록 상향 (`CHIPProjectConfig.h`/sdkconfig)

위 메모리 대책의 결과로 **ESP32-H2** 는 아래가 C6 와 다르다(C6 두 보드는 공통):

| 항목 | GNPE / XIAO (C6) | **ESP32-H2** | 근거 매크로 |
|---|---|---|---|
| 블라인드 채널 | **8채널** (2블록·ALL2) | **3채널** (1블록·ALL1) | `BLIND_MAX_COUNT` |
| 설정 메뉴 | 6항목(Time Set·FW Update 포함) | **4항목**(Time/FW 제외) | `BOARD_DISABLE_TIME`·`BOARD_DISABLE_OTA` |
| 시각 동기/표시 | SetUTCTime + SNTP + 시계 | **없음**(메모리 절약) | `BOARD_DISABLE_TIME` |
| 펌웨어 OTA | Matter OTA over Thread | **없음**(메뉴에서 숨김) | `BOARD_DISABLE_OTA` |
| 페어링 화면 | PIN (72×40 은 PIN 만) | PIN + **QR**(128×64/64×128 패널) | 해상도 자동 |
| 동작 애니메이션 | 20fps 모션 | **동일 복구**(H2 분기 제거) | `oled_ui.c` |
| Matter 구조 | Bridge(Aggregator + Bridged EP) | **composed**(root 직속 WindowCovering) | `BOARD_MATTER_COMPOSED` |
| SmartThings 노출 | 블라인드별 독립 타일(스톡 드라이버) | **1카드 3블라인드**(custom Edge driver) | `smartthings-driver/` |

> 위 표의 모든 H2 차이는 별개 결정이 아니라 **하나의 원인(RAM 부족, 섹션 상단 박스)에서
> 파생**된 것이다 — 메모리가 더 컸다면 C6 와 동일했을 항목들. Vendor/Product 이름만은
> 메모리와 무관하게 전 보드 공통 **`NLB` / `Somfy RTS Thread`**(`CHIPProjectConfig.h`).

#### ESP32-H2 — SmartThings custom Edge driver 설치

H2 는 RAM 절약을 위해 **composed**(root 직속 WindowCovering EP1·EP2·EP3) 구조라, SmartThings
**스톡 `matter-window-covering` 드라이버는 다중 동일-타입 endpoint 를 1개만 매핑**해 블라인드가
하나만 보인다. `smartthings-driver/`(custom Edge driver)를 설치하면 **카드 1개 안에 블라인드 3개**
(component `main`/`blind2`/`blind3`)로 노출된다. (C6 두 보드는 Bridge 라 스톡 드라이버로 블라인드별
독립 타일 — **설치 불필요**. Apple/Google Home 은 H2 도 추가 작업 없이 3개가 보인다 — SmartThings 한정 보완.)

```bash
npm install -g @smartthings/cli                          # 0) CLI(Node 필요)
smartthings edge:drivers:package ./smartthings-driver    # 1) 패키지 → Driver Id 출력
smartthings edge:channels:create                         # 2) 배포 채널 생성 → Channel Id
smartthings edge:channels:assign                         # 3) 채널에 driver 할당 (인자=driverId)
smartthings edge:channels:enroll                         # 4) 내 허브를 채널에 등록 (인자=hubId)
smartthings edge:drivers:install                         # 5) 허브에 driver 설치 (인자=driverId)
```

> ⚠️ 명령마다 인자가 다르다 — `assign`=**driverId**, `enroll`=**hubId**, `install`=**driverId**.
> `channelId` 를 인자로 주면 `404 Missing driver`/`500` — **인자 없이** 실행해 대화형으로 고르는 게 안전.
> **적용**: 앱 → 블라인드 기기 → ⋮ → **Driver** → `Somfy Blinds 3-Shade (composed)` 선택(또는 삭제
> 후 재페어링 시 fingerprint VID `0xFFF1`/PID `0x8001` 로 자동 적용). 코드 수정 후 갱신이 안 먹으면
> `edge:drivers:uninstall`+`install` 또는 허브 재부팅(Edge 는 install 후 자동 갱신 최대 12h).
> 전체 절차·component 매핑·주의: [`smartthings-driver/README.md`](smartthings-driver/README.md).

### CC1101 SPI (제품별)

FSPI IO-MUX 네이티브 핀 우선(저속이라 GPIO-matrix 라우팅도 무방). 모듈 제조사마다
헤더 순서가 다르니 **모듈 실크스크린 라벨 기준**으로 배선할 것.

| CC1101 | ★ GNPE C6-0.42 | Seeed XIAO C6 | ESP32-H2 SuperMini |
|---|---|---|---|
| SCK  | IO6 | D8 (19)  | GP4  |
| MOSI | IO7 | D10 (18) | GP5  |
| MISO | IO2 | D9 (20)  | GP0  |
| CSN  | IO4 | D3 (21)  | GP1  |
| GDO0 | IO8 | D6 (16)  | GP10 |

> 공통: **VCC = 3.3 V / GND, 5 V 금지.** GDO0 = RMT 비동기 TX 데이터.

### 버튼/센서 (제품별)

| 신호 | ★ GNPE C6-0.42 | Seeed XIAO C6 | ESP32-H2 SuperMini |
|---|---|---|---|
| PCF8574 SDA | IO19 | D4 (22) ‡ | GP13 ‡ |
| PCF8574 SCL | IO18 | D5 (23) ‡ | GP14 ‡ |
| PCF8574 ~INT | IO17 | D2 (2) | GP11 |
| 진동 VIBE | IO16 | D0 (0) | GP2 |
| 충전 CHG_STAT | IO3 | D7 (17) † | GP12 † |
| OLED SDA | IO1 | D4 (22) ‡ | GP13 ‡ |
| OLED SCL | IO0 | D5 (23) ‡ | GP14 ‡ |

> - **‡ I2C 공유**: **H2 = OLED·PCF8574 가 HW I2C 한 버스 공유**(같은 핀, `BOARD_I2C_SHARED=1`).
>   **XIAO 는 LP_I2C(6/7) 미연결 시 이 공유 버스로 자동 폴백**. GNPE 는 PCF8574 소프트 비트뱅.
> - **XIAO PCF8574 = 자동 폴백**(`BOARD_I2C_LP_FALLBACK=1`): 부팅 시 LP_I2C(뒷면 MTCK=6/
>   MTDO=7) 우선 프로브 → 무응답 시 OLED 공유(D4/D5=GPIO22/23)로 전환(택1 불필요).
>   **† CHG_STAT**: GNPE = MCP73831 STAT(IO3, active-LOW) 직결. **XIAO·H2 는 충전 status 가
>   LED 전용**이라, 대신 **VBUS 분압(active-HIGH)=USB 감지 + BAT 분압 ADC=실측 %** 조합(A+B,
>   `BOARD_CHG_STAT_ACTIVE_HIGH`·`BOARD_HAS_BAT_ADC`)으로 충전중/만충 판별. ADC: XIAO=D1·H2=GP3.
>   ⚠ **현 시제품 H2·xiao 기판은 이 두 분압이 핀에서 swap 오배선**(BAT_ADC핀=VBUS · CHG_STAT핀=BAT).
>   CHG_STAT 핀(H2 GP12·xiao GPIO17)이 ADC·comparator 불가라 잔량 % 측정 불가 → `BOARD_BAT_SWAPPED=1`
>   로 **`USB`/`BAT`/`LOW` 상태** 표시(배선도 참고). 정상 배선으로 고치면 `0` → 실측 % 자동 복귀.
> - **PCF8574**(주소 0x20) 비트 매핑(모든 보드 공통): UP=P4 · DOWN=P5 · SEL=P6 ·
>   PROG=P7 · SETUP=P3 · 로터리 A/B/BTN = P0/P1/P2. **OLED** 주소 0x3C(SSD1306/1315).
> - **좌/우 버튼(선택)**: `BOARD_HAS_LR_BUTTONS=1` 로 **PCF8575**(16핀) 사용 시 **P10=LEFT·
>   P11=RIGHT** 추가(주소·배선 동일, 칩만 교체). 동작은 [설정 메뉴 §좌/우 버튼](#좌우-버튼-선택--pcf8575) 참고.
> - **로터리 디텐트 타입**: `BOARD_ROT_HALF_STEP` — 0=full-step(**EC11**) / 1=half-step(**EC05**,
>   rest@11·00). 펌웨어 디코더 자동 분기(half-step=그레이코드 LUT 누산). **XIAO=EC05 → 1**.
> - 진동 스위치 · CHG_STAT · PCF8574 ~INT 은 모두 light-sleep **wake 소스**.

## 빌드 & 플래시

`esp-idf`·`esp-matter`·SSD1306 SDK 와 프로젝트 경로는 **시스템 환경 변수**에서 읽는다
(`WORKSPACES_PATH`·`RTS_BLINDS_THREAD_PATH`·`IDF_PATH`·`ESP_MATTER_PATH`·`ESP_SSD1306_PATH`).
**최초 1회 등록**은 [`doc/SETUP_GUIDE.md` §5-1](doc/SETUP_GUIDE.md#5-1-시스템-환경-변수-설정-최초-1회) 참고.
프로젝트 **루트가 ESP-IDF 프로젝트**다. PowerShell 사용(Git Bash 는 MSYSTEM 충돌).

```powershell
./build.ps1 -Board esp32-c6 -Action build               # 빌드 (기본 보드)
./build.ps1 -Board esp32-c6 -Action flash -Port COM3    # 플래시
./build.ps1 -Board esp32-c6 -Action erase -Port COM3    # 전체 지움(페어링 삭제)
./build.ps1 -Board esp32-c6 -Action menuconfig          # Kconfig 편집
```

- `-Board` 기본값은 `esp32-c6`(생략 가능). 기본 보드는 기존 `build/` +
  `sdkconfig` 를 그대로 사용(역호환). 다른 보드는 `build-<board>/`,
  `sdkconfig.<board>` 별도 디렉토리/파일.
- 모든 build/flash 출력은 **`logs/<board>-<action>.log`** 로 자동 기록
  (deterministic 이름 → 덮어쓰기, 무한증식 방지). menuconfig 만 제외.
- 빌드 시작 시 보드 자산(`boards/<board>.h` + sdkconfig defaults 조합)이
  없으면 **어떤 파일이 빠졌는지** 명시하고 중단.

산출물: `build/somfy_blinds.bin` (≈1.7 MB, 앱 파티션 ~12% 여유).

### 새 보드 추가

1. `main/boards/<name>.h` — 모든 `BOARD_PIN_*`·`BOARD_OLED_*`(필요시 `BOARD_I2C_SHARED`) 정의(`gnpe-c6.h` 참고).
2. `main/boards/board_select.h` — `#elif defined(BOARD_<NAME>)` 분기 추가.
3. `build.ps1` 의 `$BoardMap` — idf target / sdkconfig defaults / esp-matter
   device 매핑 추가.
4. `sdkconfig.defaults.<name>` 작성(필요 시).

### ★ sdkconfig 주의

검증된 `sdkconfig` 가 **단일 진실원천**이다. `set-target` 으로
`sdkconfig.defaults` 에서 재생성하면 WindowCovering 클러스터 서버가
링크에서 빠져(`emberAfWindowCovering*Callback` undefined) 빌드가 깨진다.

- 평상시 `-Action build` 만 사용(기존 sdkconfig 유지).
- 부득이 `set-target` 시 스크립트가 `sdkconfig.verified.bak` 자동 백업 →
  복구: `./build.ps1 -Board <name> -Action restore-sdkconfig`.

플래시 후 재플래시가 안 되면(절전 시 USB-Serial 정지: 저전력 Matter
기기 정상 동작) 보드 **RESET** 또는 **BOOT+RESET**(다운로드 모드) 후 진행.

## 설정 메뉴 (OLED)

메인 화면에서 **SETUP 버튼 짧게** → 설정 메뉴 진입. 메뉴는 6항목 스크롤(**H2 는 4항목**)이며,
**UP/DOWN 버튼 또는 로터리 회전**으로 커서 이동, **SETUP 짧게**로 선택 항목 진입,
**STOP(로터리 클릭)** 으로 메인 복귀.

> 용어: **SETUP** = 설정 버튼(PCF8574 P3), **STOP** = 로터리 푸시 클릭,
> **틸트** = 로터리 좌/우 회전, **UP/DOWN** = 상/하 버튼.
>
> ⏻ **SETUP 15초 이상 길게 누름** = 화면 무관 **강제 재부팅**(기기 멈춤·응답불가 대비).

| 항목 | 기능 | 사용법 |
|---|---|---|
| **Cancel** | 메인 화면으로 복귀 | SETUP 짧게 = 메인 복귀 |
| **Freq Edit** | 선택된 블라인드의 송신 주파수 미세조정 (0.01 MHz 단위, 447.20~447.79) | UP / 틸트 = +, DOWN / 틸트 = − (커서 자리값) · **좌/우 = 디지트 커서**(0.1↔0.01 자리, PCF8575) · **SETUP = 저장** · STOP = 취소(변경 폐기)→메뉴 |
| **Time Set** | 날짜·시간 수동 설정 (SmartThings 미연동 시) | UP/DOWN **또는 좌/우(PCF8575)** = 편집 자리 이동(년·월·일·시·분) · 틸트 = 값 증감 · **SETUP = 저장**(시계+NVS) · STOP = 취소→메뉴 |
| **Matter Pair** | BLE 커미셔닝 윈도우 재오픈 (기존 fabric 유지, 추가 페어링) | 진입 시 자동 오픈, OLED 에 PIN 표시(128×64/64×128 패널은 좌측 PIN + 우측 **QR** 병기) · SETUP 짧게 = "준비(READY)" 확정 · STOP 짧게 = 메뉴 / 길게 = 메인 |
| **Thread Rst** | Thread 네트워크 자격증명 삭제 후 재페어링 (네트워크/허브 교체·연결 꼬임 복구용) | **SETUP 길게(2s) = 실행** · SETUP/STOP 짧게 = 취소→메뉴 · STOP 길게 = 메인 |
| **FW Update** | 펌웨어 업데이트 (Matter OTA over Thread) — 현재 버전·진행 상태 표시 | **SET = 업데이트 확인**(QueryImage) · STOP = 메뉴/메인. 상세: [`doc/OTA.md`](doc/OTA.md) |

> 🔹 **ESP32-H2 는 `Time Set`·`FW Update` 가 없는 4항목**(Cancel / Freq Edit / Matter
> Pair / Thread Rst) — 메모리 절약으로 시각·OTA 기능을 컴파일 단계에서 제외했다
> (`BOARD_DISABLE_TIME` / `BOARD_DISABLE_OTA`). [§보드별 펌웨어 기능 차이](#보드별-펌웨어-기능-차이) 참고.

### 좌/우 버튼 (선택 — PCF8575)

`BOARD_HAS_LR_BUTTONS=1` 일 때만 활성. **PCF8574(8핀) → PCF8575(16핀)** 로 칩만 교체하고
**P10=LEFT · P11=RIGHT** 를 추가한다(주소 0x20·~INT·I2C 배선 동일, read 2바이트). 동작:

| 화면 | LEFT | RIGHT |
|---|---|---|
| **메인** | 블라인드 선택 **이전** | 블라인드 선택 **다음** (SELECT 의 좌우 버전) |
| **Time Set** | 편집 자리 **이전** | 편집 자리 **다음** |
| **Freq Edit** | 디지트 커서 **0.1 자리** | 디지트 커서 **0.01 자리** (OLED 밑줄 표시 · UP/DOWN 이 그 자리값만큼 증감) |

> 좌/우 버튼 없으면(기본) 위 동작은 비활성이고 8핀 PCF8574 그대로 — `boards/<board>.h` 에서 택일.

### Freq Edit — 주파수 미세조정

블라인드별 송신 주파수를 0.01 MHz 단위로 조정한다. 표시·편집값은 **register
설정값**(기본 447.72)이며, 보드 크리스털 오차로 실제 on-air 는 다르다(아래
**RF / Somfy RTS 447** 섹션의 "register ≠ on-air" 설명 참고). 모터가 무응답이면
정품 주파수에 맞도록 여기서 미세조정. **SETUP 으로 저장**해야 NVS 에 반영되며,
저장한 값은 **재부팅 후에도 유지**된다(부팅 시 편집 범위 447.20~447.79 밖의 손상값만 기본으로 보정).

### Thread Rst — Thread 리셋

`Thread Rst` 는 **Thread 네트워크 연결을 초기화**하는 기능이다(전체 공장초기화인
`erase-flash` 보다 가벼움 — Thread/네트워크만 리셋). SETUP 길게 누르면:

1. `thread_prov_erase()` — Thread 중단 + 활성 dataset(네트워크키 등) 비움 +
   저장된 Thread 자격증명 영구 삭제.
2. BLE 커미셔닝 윈도우 재오픈 → OLED Thread 프로비저닝 화면(재페어링 대기).

**언제**: Thread 허브/Border Router 교체, Thread 연결이 꼬여 복구 안 될 때,
다른 SmartThings 계정/허브로 이전 시.

## SmartThings (Matter over Thread)

1. SmartThings 앱 → 기기 추가 → Matter → 디바이스 OLED 의 PIN/QR.
2. 커미셔닝 완료 후 블라인드 노출 — **C6** 8개 독립 타일 / **H2** 1카드 3블라인드(composed, custom Edge driver 필요).
3. 추가 페어링: 설정 메뉴 → `Matter Pair`(BLE 커미셔닝 윈도우 재개,
   기존 fabric 유지).
4. 시각: SmartThings 가 자동 동기(SetUTCTime). 미연동 시 설정 메뉴
   → `Time Set` 으로 수동 설정. (**H2 는 시각 기능 없음** — `BOARD_DISABLE_TIME`)

> 커미셔닝 중에는 OLED/버튼/RF 등 모든 부가 태스크가 정지한다(802.15.4/
> SRP/CASE 타이밍 보호 — 페어링 39-517 방지). 완료 후 자동 시작.

## RF / Somfy RTS 447 (한국 베네치아)

정품 리모컨1(addr 0xC91BF0) 의 12-bit IQ 녹음을 rtl_433 fork 로 150+ frame
디코드해 확정한 프로토콜:

- **주파수**: 기본 register 447.72 MHz → 크리스털 오차 보정 후 on-air
  ~447.678 MHz(정품과 ±3 kHz). 블라인드별 설정(설정 메뉴 → `Freq Edit`).

> #### ★ register 값(447.72) ≠ 실제 송출(447.678) — 왜?
>
> CC1101 은 **크리스털(nominal 26 MHz)** 을 기준으로 캐리어를 합성한다.
> 드라이버가 `FREQ레지스터 = 목표주파수 × 2^16 / 크리스털주파수` 로 값을
> 써넣으면, 칩은 그 값 × (실제 크리스털 / 2^16) 으로 출력한다.
>
> 문제는 **이 보드의 크리스털이 정확히 26.000000 MHz 가 아니라는 것**이다
> (제조 공차·부하 커패시터·온도로 ±수십 ppm). 측정 결과 이 모듈은
> 계산값보다 약 **-41.5 kHz**(≈-93 ppm) 낮게 송신한다. 그래서:
>
> ```
> register 447.72 입력 → 칩 합성 → 안테나 실제 출력 447.678
>                         (크리스털 오차가 -41.5 kHz 끌어내림)
> ```
>
> 정품 모터가 듣는 진짜 주파수는 **447.678** 이므로, 거기에 실제로 도달하려면
> register 에 **447.72**(= 447.678 + 0.042)를 넣어야 한다. 즉 별도 보정 코드가
> 아니라 **이 보정이 기본값 선택에 녹아 있는** 상태다.
>
> | 항목 | 의미 | 값 |
> |---|---|---|
> | register (코드→칩) | "이 주파수로 쏴" 지정값 = OLED·Freq Edit 표시값 | 447.72 |
> | on-air (안테나 출력) | 모터가 실제 받는 주파수 | ~447.678 |
> | 정품 리모컨 실측 | 목표 | 447.675~447.678 |
>
> **주의**: register 를 447.675 로 바꾸면 실제 출력이 ~447.633 으로 떨어져
> 모터가 무응답할 수 있다. OLED 상단에 보이는 주파수도 이 **register 값**이다.
> 다른 보드로 교체하면 크리스털 오차가 달라 이 보정 상수(기본 447.72)를
> 재측정·재설정해야 할 수 있다.
>
> **RF 모듈별 실측** (같은 447 대역이라도 모듈 크리스털 오차가 달라 register 권장값이 다름):
>
> | RF 모듈 | 보드 | register(설정·표시) | on-air(실측) |
> |---|---|---|---|
> | **E07-M1101D-SMA** | 테스트 보드 | 447.72 | 447.675 MHz |
> | **E07-400MM10S** | 제작 보드 | **447.70** | 447.673 MHz (실제 블라인드 인식 확인) |
>
> → 제작 보드(**E07-400MM10S**)는 `Freq Edit` 로 **447.70** 으로 맞춰야 정품 on-air(~447.67)에 도달한다.
> 새 모듈로 교체 시 SDR 등으로 on-air 를 실측해 register 를 재조정할 것.
- **변조**: 2-FSK(±2.6~2.8 kHz). (표준 433.42 MHz Somfy 는 OOK.)
- **프레임**: 80비트(10바이트). byte0=key, byte1=cmd|cks, byte2-3=rolling(BE),
  byte4-6=address(BE), byte7=0x84, byte8/9=cmd별 + calc80Checksum.
- **Manchester 극성**: '1'=LOW→HIGH, '0'=HIGH→LOW(표준 RTS/ESPSomfy 동일).
- **HW sync**: 첫 프레임 12 cycle / 반복 6 cycle, ON/OFF 2560 µs. SW sync
  4850 µs HIGH + 640 µs LOW. inter-frame gap 4000 µs.
- **주소(address)**: 24비트. 기기별로 eFuse MAC 에서 산출(아래 섹션). 주파수는
  블라인드별 NVS 저장, 롤링코드는 NVS + `rollcode` 파티션 이중 저장.
- **커맨드**: UP/DOWN/MY/PROG + **Tilt Up/Down**(cmd nibble 0xB, byte8 로 방향).
- 신규 블라인드 등록: 블라인드 선택 → `PROG` 송신(리모컨 페어링 절차).

> ESPSomfy-RTS fork(`${WORKSPACES_PATH}/ESPSomfy-RTS`)에도 동일한 447 분기를
> 반영(`modulation=0` 일 때 한국 wire-byte/timing 적용). 상세는 그 레포의
> `SOMFY_RTS_447.md`.

## 기기 고유 ID & 롤링코드 영속

여러 대를 만들어 같은 블라인드를 제어할 때 주소(=리모컨 ID)가 겹치면 안 되고,
펌웨어 업데이트·리셋에도 ID 가 바뀌면 안 된다.

### 기기별 고유 블라인드 주소 (eFuse MAC → 블록 ALL 체계)

블라인드 주소(=리모컨 ID)를 ESP32 의 **eFuse 팩토리 MAC**(칩 고유·불변)에서 FNV-1a
해시로 결정적 산출한다 (`blind_manager.c` `_derive_addresses`). 실제 Somfy 호환 리모컨
실측 규칙대로 **`F0 [중간:채널 등차] [하위:보드 ID]`** 24비트 형식이다:

```
esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);      // 칩 고유 6바이트 (불변)
h = FNV-1a(mac);  h ^= (블록+1)*0x9E3779B1;     // 블록별 salt → 블록마다 다른 base
mid  = (h>>8) % 0x27;                           // 중간바이트 = 등차 1번 최소항(0~0x26)
low  =  h     & 0xFF;                           // 하위바이트 = 보드 고유 ID
base = 0xF00000 | (mid<<8) | low;               // 채널1 주소
채널 i = base + i*0x2700;   ALL = base + 4*0x2700   // 중간바이트 +0x27 등차
```

**4채널 = 1 블록**(채널 4개 + 전용 ALL 1개). 채널이 늘면 블록(=ALL)이 자동 증가한다
(`BLIND_BLOCK_COUNT = ceil(채널수/4)`). ALL 명령은 **모든 블록의 ALL 을 차례로 송신**한다:

| 보드(채널) | 블록 | ALL | ALL 송신 |
|---|---|---|---|
| ESP32-H2 (3채널) | 1 | base+4×0x2700 | ALL 1회 |
| GNPE/XIAO C6 (8채널) | 2 | 블록0·블록1 각 ALL | ALL1, ALL2 |
| (가정) 9채널 | 3 | 블록0~2 각 ALL | ALL1, ALL2, ALL3 |

| 속성 | 보장 | 이유 |
|---|---|---|
| 기기 간 충돌 없음 | ✅ | 칩마다 다른 MAC → 다른 해시 → 다른 하위 ID |
| OTA 후 불변 | ✅ | eFuse 불변 |
| **factory reset(erase-flash) 후 불변** | ✅ | eFuse 는 flash 가 아님 + 매 부팅 enforce |
| ALL = 등차 다음 항 | ✅ | 실측: 채널 4개 뒤 ALL = base+4×0x2700(중간바이트 +0x27 규칙) |

> ⚠️ `esp_efuse_mac_get_default()` 는 C6 에서 EUI-64 형태(중간 `FF:FE` 패딩)를 줘
> 고유 바이트가 거의 없다 → 반드시 `ESP_MAC_EFUSE_FACTORY` 를 쓸 것.
> ※ **ALL 은 PROG 등록 절차가 없다** — 모터는 채널 주소만 PROG 로 받고 ALL 은 내부
> 인식한다. 펌웨어는 ALL 주소를 등차로 산출해 **송신만** 한다. 주소를 바꾼 기기는
> 채널마다 모터에 **PROG 재등록** 필요(새 주소 = 새 리모컨 ID).

### 롤링코드 factory-reset 영속 (채널 + 블록 ALL)

롤링코드는 매 송신마다 증가하고, 모터는 직전보다 작은 롤링코드를 replay 로
거부한다. 기본 NVS 에만 두면 Matter factory reset(기본 NVS 전체 삭제) 시
START 로 리셋 → 모터 무응답. 그래서 **전용 `rollcode` NVS 파티션**(partitions.csv,
0x3E6000/16KB)에 **채널과 블록 ALL 롤링코드를 모두** 보존하고(`rollcode_blob_t`
= `rolling[채널]` + `all_rolling[블록]`), 부팅 시 **더 큰 값을 채택**(역행 방지)한다.
특히 **ALL 은 PROG 재등록 절차가 없어** 롤링코드가 한 번 어긋나면 복구가 불가능하므로
ALL 롤링코드의 영속이 채널 못지않게 중요하다.

| 이벤트 | 롤링코드(채널 + 블록 ALL) |
|---|---|
| 일반 송신 | NVS + `rollcode` 파티션 동시 갱신 |
| OTA | 둘 다 보존 |
| **Matter factory reset**(기본 NVS 삭제) | `rollcode` 파티션 보존 → 채널·ALL 모두 복원 ✅ |
| 전체 `erase-flash` | 모두 삭제 → 재-PROG 필요(개발자 행위) |

### 기기별 고유 커미셔닝 코드 (eFuse MAC → discriminator/passcode)

Matter 페어링 코드(discriminator 12bit + passcode + Spake2p verifier)도 블라인드 주소와
같은 **eFuse 팩토리 MAC** 에서 결정적 산출한다(`efuse_commissionable.cpp`, custom
`CommissionableDataProvider`). `fctry`(factory_nvs_gen.py) 플래시 없이도 **기기마다 다른
페어링 코드**를 가져, 여러 대를 만들어 동시에 페어링해도 BLE discriminator 충돌이 없다.

```
mac = eFuse FACTORY MAC;   h = FNV-1a(mac) ^ "COMM" salt
discriminator = (h ^ h>>12 ^ h>>23) & 0xFFF   // 비트 폴딩 → short(상위 4bit)도 분산
passcode      = (h % 99999998) + 1            // 유효 PIN 보정
verifier      = Spake2pVerifier.Generate(iter=1000, salt(eFuse), passcode)  // PASE
```

| 속성 | 보장 |
|---|---|
| 기기 간 코드 충돌 없음 | ✅ eFuse 고유 — 실측 gnpe `3327`/12 · xiao `4032`/15 · h2 `2687`/10 (long·short 모두 다름) |
| fctry 플래시 불필요 | ✅ 펌웨어가 부팅 시 자동 산출 |
| DAC(인증서)는 별개 유지 | ✅ commissionable provider 만 교체(DAC provider 무관) |

> `sdkconfig`: **`CONFIG_CUSTOM_COMMISSIONABLE_DATA_PROVIDER=y`** (C6·H2 적용). 미설정 보드는
> 기존 경로(EXAMPLE 기본 `3840`/`20202021`). discriminator 12bit(4096)·short 4bit(16) 공간이라
> 수십 대 규모면 충돌 가능(생일 문제) — 그땐 `factory_nvs_gen.py` 로 fctry 고유성 부여.

## OLED 구동 방식 & 화면 정책 (2026-08-11 갱신)

### 비트뱅 I2C (`BOARD_OLED_BITBANG`, xiao-c6 = 1)

xiao-c6(h4)에서 **ESP32 하드웨어 I2C0 페리페럴이 "bus busy" 로 고착**되는 현상이 재현됐다.
라인은 idle(1/1)인데 `i2c.master: clear bus failed` / `reset hardware failed` 가 뜨고 버스를
지웠다 다시 만들어도 풀리지 않는다. 같은 순간 **순수 GPIO 비트뱅은 정상 ACK** 를 받는다.

→ OLED 전송을 비트뱅으로 옮겨 페리페럴을 통째로 우회한다.

- `_bbo_write()` : GPIO **레지스터 직접 접근**(`GPIO_OUT_W1TS/W1TC`, `GPIO_IN`). HAL 호출
  (`gpio_set_direction`)은 비용이 크고 불규칙해 400 kHz 에서 파형이 깨진다(화면에 랜덤 점).
- 핀은 `GPIO_MODE_INPUT_OUTPUT_OD` 로 **한 번만** 설정하고 이후 출력 레지스터만 토글한다.
  `OUTPUT_OD` 로 하면 입력 버퍼가 꺼져 읽기가 항상 0 이 되므로 반드시 `INPUT_OUTPUT_OD`.
- 속도 `BBO_HALF_US` (1 ≈ 400 kHz). 점/깨짐이 보이면 2~3 으로 올릴 것.
- HW I2C 버스를 **아예 만들지 않으므로**, PCF8574 를 OLED 와 공유 HW I2C 로 쓰는 보드
  (`BOARD_I2C_SHARED` 경로, 예: esp32-h2)는 이 옵션과 함께 쓰지 말 것.
- **이식성 주의**: 비트뱅 계측 전역(`g_bbo_tx_cnt` 등)은 `#if BOARD_OLED_BITBANG` **밖**에
  선언해야 한다. 가드 안에 두면 `BOARD_OLED_BITBANG=0` 보드(H2)에서 링크 에러가 난다.

### dirty-page 전송 감축

화면 전체를 매번 재전송하면 초당 약 450 건의 I2C 트랜잭션이 발생한다(전송 1건 = 고착 기회).
마지막으로 성공 전송한 페이지를 `s_shadow` 에 보관하고 **내용이 바뀐 페이지만** 보낸다.
전송 실패 시 shadow 를 갱신하지 않아 다음 주기에 자동 재시도되고, 검출 실패/재검출 시에는
무효화해 전량 재전송한다. 실측: 전송 **75 % 감축**, 페이지 **86 % 건너뜀**.

### 화면 정책 — 화면보호기 없음

- 화면보호기(중간 애니메이션)는 **코드째 삭제**됐다. 유휴 시간이 지나면 곧바로 패널 OFF.
- 꺼지는 시간은 **전원에 따라 다르다**(`main/somfy_config.h`):

  | 전원 | 매크로 | 기본값 | 이유 |
  |---|---|---|---|
  | USB | `CFG_SCREEN_OFF_USB_SEC` | **300초(5분)** | 전원이 무제한 — 작업 중 자꾸 꺼지면 불편 |
  | 배터리 | `CFG_SCREEN_OFF_SEC` | **10초** | 화면이 소비의 큰 몫 |

  보드별로 바꾸려면 `boards/<board>.h` 에서 먼저 정의하면 된다.
- 복귀: **버튼 또는 진동**. 스택이 자동 진입하는 상태(PAIRING/THREAD_PROV/CHARGING)는
  화면 유지 조건에서 제외했다 — 포함하면 유휴 타이머가 계속 리셋돼 화면이 영영 안 꺼진다.

### 채널변경 잠금 (`CFG_CH_LOCK_MS`, 기본 700 ms)

상/하 등을 **동시에** 누를 때 좌/우가 스쳐 채널이 바뀌는 것을 막는다. 아래 중 하나라도
해당하면 SELECT·좌·우를 무시한다.

1. RF 발생 버튼(상·하·정지(로터리)·PROG)이 눌려 있는 동안
2. 동작 애니메이션이 재생되는 동안 (`OLED_STATE_ACTION`)
3. 마지막 RF 버튼 눌림 후 `CFG_CH_LOCK_MS` 이내 (동시 누름의 "텀" 대응)

### I2C 직렬화 — 락은 `_bbo_write()` 한 곳에 (2026-08-11)

**모든 비트뱅 전송은 `_bbo_write()` 안에서 재귀 뮤텍스로 보호된다.** 락을 여기 한 곳에만
두면 호출 지점이 늘어도 자동으로 보호된다.

왜 이렇게 바뀌었나 — 전에는 `_fb_flush` 만 락을 잡아 아래 두 경로가 **무방비**였다.

| 무방비였던 경로 | 호출자 | 빈도 |
|---|---|---|
| `_oled_send_cmds()` ← `oled_ui_set_display_on()` | somfy_app | 화면 자동 OFF/ON **10초마다** |
| `_bbo_probe()` ← `_oled_try_detect()` | flush 의 early-return 경로(락 잡기 **전**) | 미검출 시 5초마다 |

태스크 우선순위가 **`btn_handler` 10 > `somfy_app` 4 > `oled_ui` 3** 이라 somfy_app 이
oled_ui 를 **선점**한다. 비트뱅은 CPU 가 곧 클럭이므로, 선점당한 전송은 SCL/SDA 가 중간
상태로 수백 µs 멈춰 SSD1306 상태머신이 고착된다(한 번 고착되면 저절로 안 풀리는 래치).

- **재귀 뮤텍스여야 한다.** `_fb_flush` 가 바깥 락을 쥔 채 `_bbo_write` 를 부르므로,
  일반 뮤텍스면 자기 자신을 기다려 **즉시 데드락**(화면 영구 정지)이다.
  `xSemaphoreCreateRecursiveMutex` + `xSemaphoreTakeRecursive`/`GiveRecursive` 를 쓸 것.
- 뮤텍스는 `oled_ui_init()` **맨 앞**에서 만든다. 늦게 만들면 그 사이 호출이
  `s_i2c_mutex == NULL` 을 만나 무보호로 통과한다.
- 락 획득 타임아웃(200 ms)이 나면 **그냥 전송한다**. 락은 겹침 방지용이지 전송의
  전제조건이 아니며, 화면을 영구히 멈추는 것보다 한 프레임 깨지는 편이 낫다.
  발생 횟수는 `[OLEDMON]` 의 `lockTO` 로 관찰한다(정상 = 0).

검증: `sim/tools/adc_oled_mutex_sim.py` (8시드 × 10분) — 수정 전 **6/8회 고착(최빠른 80초)**,
수정 후 손상 0 / 고착 0 / 배터리 측정 952/952.

**실기 장시간(2026-08-11, COM7)**: 콘솔 `cyc` 로 화면을 계속 갱신시킨 100 % 부하 +
ADC 5초 주기 병행으로 **4시간 45분 / 전송 257,562건 / 실패 0 / lockTO 0**,
`present` 284개 샘플 전부 1, 패닉·재부팅·워치독·버스복구 0건, free heap Δ-80 B.

### 충전률 측정 (재활성됨, 2026-08-11)

`TEMP_NO_CHARGE = 0`. 위 직렬화 수정으로 되살렸다.

- **측정 주기 5초 · 표본 8회는 그대로 둘 것.** 줄이면 `_nobat_track` 의
  "5분 창 / 반쪽당 30표본" 노이즈 상쇄 가정이 깨져 **배터리 미연결 오판**이 생긴다.
  (30초 주기면 반쪽당 5표본 → 노이즈가 판정 문턱 4 mV 에 근접한다.)
- 문제가 재발하면 `TEMP_NO_CHARGE` 를 1 로 되돌리고 `[OLEDMON]` 의 실패·`lockTO` 를 볼 것.

### 배터리 미연결 판정과 "78%" (2026-08-11)

배터리를 빼도 충전 IC(MCP73831)가 BAT+ 를 무부하로 **약 3,970 mV** 로 띄운다.
이 값은 OCV-SoC 곡선상 정확히 **78 %** 라, 예전에는 "배터리 없는데 78 %" 로 보였다.
**전압만으로는 실제 배터리 78 % 와 구분할 수 없다.**

유일한 단서는 **전압이 오르는가**다(무배터리 float = 안 오름 / 충전 중인 셀 = 오름).
`_nobat_track()` 이 5분 창의 전반·후반 평균을 비교해 판정한다.

- **첫 판정 전에는 % 를 표시하지 않는다** (`BAT_PCT_UNKNOWN` → 화면에 `--%`).
  단 **애매할 때만** 숨긴다: float 창(3940~4010 mV) **밖**이면 충전 IC 가 그 값을
  만들 수 없어 배터리 존재가 전압만으로 확정되므로 **대기 없이 즉시** % 를 보여준다.
  USB 가 없을 때(=배터리로 구동 중)도 배터리가 있는 게 자명하므로 즉시 표시한다.
- **창(5분)을 줄이면 안 된다.** 충전 상승률이 ≈9.3 mV/5분 이라 창이 짧으면
  전반/후반 평균차가 판정 문턱(4 mV)에 못 미쳐 **진짜 충전 중인 배터리를 "미연결"로
  오판**해 0 % 로 표시한다. 3분 창 = 2.8 mV 로 이미 위험하다.
- 실기 로그: `[BAT] 배터리 판정: 미연결 → 0% 표시 — 전반3970mV(n30) 후반3969mV(n30)
  상승-1mV (창O, 안오름O, USB=1)` 가 부팅 **5분 23초**에 나온다.
- 검증: `sim/tools/bat_pct_display_sim.py` (4개 시나리오 + 창 단축 위험 표)

### 배터리 % 표시 평활 (2026-08-11)

배터리 구동에서는 BLE 광고·RF 송신 순간 **실제로** 전압이 떨어진다. 5초 주기 측정이
그 순간에 걸리면 값이 크게 낮게 찍히고, OCV 곡선상 이 구간은 **1 % ≈ 10 mV** 라
100 mV 만 흔들려도 10 %p 가 움직인다(실측: 81→82→**74**→84→81 %).

`_read_bat_mv()` 의 8회 평균은 수십 µs 안에 끝나 **같은 순간을 8번 재는 것**이라
ADC 노이즈만 줄일 뿐 부하 변동은 못 거른다. 그래서 **표시 경로에만** 후처리를 넣었다:

1. **중앙값 5주기**(`_bat_smooth_mv`) — 송신 순간에 걸린 점을 통째로 버린다
2. **EMA(α=1/4)** — 남은 흔들림을 시간축으로 누른다

`_nobat_track` 에는 **원본 값**을 준다 — 그쪽은 5분 창 30표본 통계라 평활하면
"전압이 오르는가" 판정 가정이 깨진다. 측정 주기·표본 수도 그대로 둔다.

★**EMA 는 1/16 mV 단위로 누적**한다. C 의 정수 나눗셈은 0 쪽으로 절단하므로 1 mV
단위로 쓰면 차이가 1~3 mV 일 때 몫이 0 이 되어 **EMA 가 고착**된다.

검증(`sim/tools/bat_pct_smooth_sim.py`): 진폭 19→9 %p, 주기간 변동 4.09→0.38 %p,
송신 강하 이상치 미노출, 방전 추종 지연 2 %p 이내.

### 배터리 단독 부팅 (USB 없이) — 2026-08-11

**증상**: USB 를 빼고 배터리만 연결하면 부팅 화면에서 멈추고 40초~1분마다 재부팅.

**원인**: `esp_matter::console::init()` 이 CHIP shell 을 **우선순위 5** 태스크로 띄운다
(`somfy_app`=4, `oled_ui`=3 보다 높다). 이 태스크가 프롬프트를 출력하는데, 보조 콘솔이
USB-Serial-JTAG 이라 `esp_rom_usb_serial_putc` 가 줄바꿈마다
`usb_serial_device_tx_flush()` 로 **호스트를 기다린다**. USB 가 없으면 이 대기가 길어지고
prio 5 가 아래 태스크를 굶겨 부팅이 끝나지 않는다.

**수정**: **VBUS 가 실제로 있을 때만** CHIP shell 을 시작한다(`app_main.cpp`
`_usb_vbus_present()` — GPIO17 VBUS 분압). 콘솔은 개발용이고 `tx`/`sel`/`cyc`/`bd` 도
USB 전용이라 배터리 단독 동작에는 필요 없다.

> `usb_serial_jtag_is_connected()` 를 쓰지 않은 이유: SOF 패킷 기반이라 부팅 ~1초
> 시점엔 열거가 안 끝나 false 가 나올 수 있고, 그러면 **USB 개발 중에도 콘솔이 사라진다**.

**진단 방법 — `boot_diag`** (`main/boot_diag.c`): USB 를 꽂는 순간 증상이 사라져 시리얼
로그를 볼 수 없으므로, 전원이 끊겨도 남는 **NVS 부팅 단계 기록**을 만들었다(RTC RAM 은
배터리를 빼면 지워져 못 쓴다). app_main 경로와 somfy_app 경로를 **따로** 세고, 실패한
부팅은 별도 키에 **영구 보관**한다(정상 부팅이 덮어쓰지 않게).

- 콘솔 `bd` — 언제든 조회 / `bd clear` — 보관 기록 삭제
- 끄려면 `BOOT_DIAG_ENABLE 0`(부팅당 NVS 쓰기 ~17회)

### 진단 로그

- `[OLEDMON]` (60초 주기) — 가동시간 · 전송/실패 · 페이지 보냄/건너뜀 · **`lockTO`** · `present` · free heap
- `[BBFAIL]` — 비트뱅 전송 실패(20회마다 1줄)
- ★**로그 인코딩은 캡처 경로마다 다르다.** pyserial 직접 캡처 = **UTF-8**,
  빌드 로그(`logs/*.log`) = **CP949**. 틀리게 디코드하면 한글 에러가 0건으로 오독된다.
  `try utf-8 → except cp949` 로 판별할 것.
- ★**"실패 0"은 정상의 증거가 아니다.** OLED 미검출이면 flush 를 건너뛰어 전송도 에러도
  안 생긴다. **`present=1` 과 전송 카운터 증가를 함께** 봐야 한다.

### 알려진 미해결 / 임시 상태

- **진동센서 오검출** — 핀이 고정 HIGH 인데 ISR 이 계속 발생해 화면이 저절로 켜지는 사례.
  `[VIBE-stat] 진동=1 HIGH=300/300` 로 확인. HW 로는 VIBE 핀·VS1 배선 점검 필요.
- **ESP32-H2 메모리** — 블라인드 3개면 free heap 이 5 KB 미만까지 떨어진다(C6 는 171 KB).
  H2 는 `BLIND_MAX_COUNT` 를 2 이하로 둘 것.
  ※**2026-08-24 이후 사정이 크게 달라졌다.** `CONFIG_USE_BLE_ONLY_FOR_COMMISSIONING=y`
  로 등록 후 BLE 를 내리자 **free 11.3 KB → 40.5 KB (+29.2 KB)**. 여기에 2026-08-22
  태스크 통합(`time_update`·`time_persist` → 메인 루프)으로 5 KB 이상을 더 회수했다.
  ※단 벽은 **총량이 아니라 연속 블록**이다 — free 5.9 KB 가 남았는데도 최대 연속
  블록이 2,112 B 라 NOC 인증서 검증이 실패한 적이 있다(2026-08-19).
  ※대가: 이미 등록된 기기는 **BLE 광고를 하지 않는다.** 재페어링은 공장초기화 후.

## 안전 / 진단

- **Task WDT**: 메인 루프 + 버튼 태스크가 5 s 이상 멈추면 자동 panic→리부트.
- **Crash breadcrumb(RTC 메모리)**: 비정상 재부팅 시 부팅 직후 직전 tick
  카운터/타임스탬프/상태(screensaver·sleep·setup)를 ★ 로그로 덤프 →
  어느 태스크가 어디서 멈췄는지 추정.
- **화면보호기 stuck 복구**: 플래그가 false 여도 OLED state 가 SCREENSAVER 면
  버튼/진동 입력 시 강제 재해제.

시리얼 캡처(진단용):
```powershell
python read_serial.py COM3 115200 > logs/serial_latest.log
```

### 시리얼 콘솔 (진단 / 자동 테스트)

USB Serial JTAG 콘솔로 **버튼 없이** RF 송신·블라인드 선택·순환·주파수 설정·재부팅을
무인 제어한다 — PC 가 COM 포트로 기기를 직접 구동해 RF 를 쏘고 SDR(`somfy_cli`)로
캡처·분석하는 자동 루프에 쓴다(`read_serial.py` 패턴, `sim/tools/`).

| 명령 | 동작 |
|---|---|
| `tx up\|down\|updown\|myup\|mydown\|my\|prog [hold_ms]` | Somfy RF 송신(session-gate 우회 → combo 직접 지정). 실측 `tx up`=cmd2 · `tx updown`=cmd6 |
| `sel <0..N \| N=ALL>` | 블라인드 선택 (N = `BLIND_MAX_COUNT`; H2 `3`·C6 `8` = ALL) |
| `cyc <-1 \| 1>` | 블라인드 선택 순환(`_blind_cycle`) — PCF8575 LEFT/RIGHT 의 콘솔 버전 |
| `freq [idx mhz]` | 주파수 조회 / 설정(+NVS 저장, 447.20~447.79 클램프) |
| `reboot` | `esp_restart` 재부팅 |
| `bd` / `bl` / `vl` | 부팅 진단 / 배터리 방전 기록 / 진동센서 기록 (NVS) |
| **`tmr`** | **등록된 `esp_timer` 덤프** — 깨어남 출처 추적. 이 한 줄이 "깨어남의 78 %" 범인(쓰지도 않는 `iot_button` 20 ms 타이머)을 찾아냈다 |
| **`pm`** | **`esp_pm_dump_locks`** — 누가 light sleep 을 막는지 이름으로 나온다(`CONFIG_PM_PROFILING=y` 면 보유 시간 %까지) |
| `rt` | 태스크별 CPU 점유(`CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS` 필요) |

### 절전 진단 — 추정하지 말고 찍을 것

절전 원인 추적에서 가설이 **세 번 연속 틀렸고**(LP 코어 정지 / PCF 지터 /
Matter 내부 타이머), `esp_timer_dump()` 한 줄이 그걸 끝냈다. 순서는 이렇게 한다:

1. **`tmr`** — 12~20 ms 같은 짧은 주기 타이머가 있으면 그게 천장이다.
   *신호*: 수면 길이가 **특정 값을 절대 안 넘으면** 그 주기의 타이머가 있다는 뜻.
2. **`pm`** — `Active=1` 로 오래 잡힌 락이 있으면 light sleep 은 시도조차 못 한다
   (`light_sleep_counts:0`). `batlog` 의 `pm=3` 은 "설정이 허용됐다"는 뜻일 뿐
   **자고 있다는 증거가 아니다.**
3. 배터리 구간은 USB 콘솔이 죽으므로 **NVS 로** 받는다 —
   `sim/tools/batlog_decode.py`(수면 길이 히스토그램·깨어남 원인·태스크별 반복),
   `sim/tools/pmdump_decode.py`(PM 락 보유자),
   `sim/tools/bat_current_estimate.py`(방전 → 실효 전류).
4. **비교는 같은 출발·도착 전압 구간으로.** OCV 기울기가 구간마다 2.3 배 다르다
   (3850~3950 = 5.0 mV/%p vs 4080~4150 = 11.7 mV/%p). 어기면 수면 전류가
   음수로 나오는 식의 헛계산이 된다.

> ⚠️ **멀티미터 직렬 전류 측정은 이 기판에서 불가**(배터리 선을 끊을 수 없다).
> 배터리 +/− 에 **전류계 모드로 직결하면 셀 단락** — 절대 금지. 전압만 잰다.

> ★ **USB Serial JTAG 를 primary 콘솔**로 둬야 명령 입력이 먹는다(UART default 면 COM 에
> stdin 이 없어 write 가 hang). 또 pyserial 로 포트를 **여러 번 여닫으면 DTR 토글로 리셋**되니
> 한 세션(단일 open) 안에서 연속 전송할 것.

## 프로젝트 구조

```
somfy-blinds-things-by-claude/        ← ESP-IDF 프로젝트 루트
├─ CMakeLists.txt   project(somfy_blinds)
├─ build.ps1        보드별 빌드/플래시/ota-image(-Board), logs/ 자동 기록
├─ sdkconfig        검증된 설정(단일 진실원천)  + sdkconfig.verified.bak
├─ sdkconfig.defaults, .c6_thread, .h2_thread, .xiao_c6
├─ partitions.csv   ota_0/ota_1(듀얼 OTA) + rollcode(롤링코드 영속) + fctry
├─ logs/            빌드/플래시/시리얼 로그(<board>-<action>.log) — git ignore
├─ dist/            보드별 Matter OTA 이미지(.ota) — git ignore
├─ main/
│   ├─ boards/             브랜드-SoC 단위 핀맵 (OTA PID 도 보드별 분리)
│   │   ├─ board_select.h   -DBOARD_<NAME> → 핀맵 분기 + 필수 매크로 검증
│   │   ├─ gnpe-c6.h        GNPE ESP32-C6-0.42 (검증, PID 0x8000)
│   │   ├─ xiao-c6.h        Seeed XIAO ESP32-C6 (검증, PID 0x8003)
│   │   ├─ esp32-c6.h       구 키 → gnpe-c6.h 별칭(역호환)
│   │   └─ esp32-h2.h       ESP32-H2 SuperMini(Thread, 검증, I2C 공유)
│   ├─ app_main.cpp        Matter 코어/노드/엔드포인트/WC delegate/OpenThread
│   ├─ app_driver.cpp      Matter attribute/identify 글루
│   ├─ matter_blinds_shim.cpp  matter_blinds_* C API (상태조회/재커미셔닝)
│   ├─ somfy_app.c         OLED/버튼/메뉴/시계/절전/안전망 애플리케이션
│   ├─ cc1101.c/.h         CC1101 드라이버 (핀은 boards/<board>.h 에서 파생)
│   ├─ somfy_rts.c/.h      Somfy RTS 447 프로토콜(프레임/타이밍/tilt step)
│   ├─ blind_manager.c/.h  블라인드 상태/NVS
│   ├─ oled_ui.c/.h        OLED UI/모션
│   ├─ button_handler.c/.h PCF8574 버튼 + 로터리 + 진동 ISR
│   ├─ thread_provision.c/.h
│   ├─ somfy_config.h      핀(BOARD_PIN_* 참조)/타임존/주파수 등 설정
│   └─ gen_build_epoch.cmake  빌드시각 헤더 생성
├─ doc/             문서(README 제외 전부) — 아래 "문서" 참고
│   ├─ SETUP_GUIDE.md   배선/빌드/플래시/페어링 단계별 가이드
│   ├─ CHECKLIST.md     기능 체크리스트 + 메모리/파티션 현황
│   ├─ OTA.md           Matter OTA(over Thread) 워크플로
│   ├─ CLEANUP_v3.5.md  정리 노트
│   ├─ architecture.html 아키텍처 다이어그램
│   ├─ wiring/          보드별 배선도 (gnpe-c6 / xiao-c6 / esp32-h2)
│   └─ CC1101/ esp32/ parts/  데이터시트·핀맵 이미지·부품 자료
└─ test/            가상/온에어/디코더 테스트 하네스(build_test.ps1)
```

## 문서

루트에는 본 `README.md` 만 두고, 나머지 문서는 모두 [`doc/`](doc/) 아래에 있다.

| 문서 | 내용 |
|---|---|
| [`doc/SETUP_GUIDE.md`](doc/SETUP_GUIDE.md) | 배선·빌드·플래시·Matter 페어링·트러블슈팅 단계별 가이드 |
| [`doc/CHECKLIST.md`](doc/CHECKLIST.md) | 구현 기능 체크리스트, 메모리/파티션 현황 |
| [`doc/OTA.md`](doc/OTA.md) | Matter OTA(over Thread) 이미지 생성·배포·보드별 PID |
| [`smartthings-driver/README.md`](smartthings-driver/README.md) | **ESP32-H2(composed) 전용** SmartThings custom Edge driver — 1카드 3블라인드 노출 + CLI 배포 절차 |
| [`doc/wiring/`](doc/wiring/) | **보드별 배선도** — [gnpe-c6](doc/wiring/wiring_gnpe-c6.md) · [xiao-c6](doc/wiring/wiring_xiao-c6.md) · [esp32-h2](doc/wiring/wiring_esp32-h2.md) |
| [`doc/architecture.html`](doc/architecture.html) | 아키텍처 다이어그램 |

> `CLAUDE.md`(루트) 는 에디터 툴 지침이라 문서가 아니며, Claude Code 가 루트에서
> 읽어야 하므로 이동하지 않는다.

## 외부 의존성 (SDK, 별도 위치 유지)

대형 SDK 는 저장소에 포함하지 않고 환경변수 경로에서 참조한다. **검증에 사용한 정확한 리비전**:

| 의존성 | 버전 / 리비전 | 경로 |
|---|---|---|
| **ESP-IDF** | **v5.4.1** (tag · 커밋 `4c2820d377`) | `${WORKSPACES_PATH}/esp-idf` |
| **esp-matter** | 커밋 `7706cfbd` (main · 2026-03-31 · 태그 없음) | `${WORKSPACES_PATH}/esp-matter` |
| **esp-idf-ssd1306** | 커밋 `554df45` (master · 2026-02-20) | `${WORKSPACES_PATH}/esp-idf-ssd1306` |

> esp-matter 는 connectedhomeip(Matter 코어)를 submodule 로 동봉 — esp-matter 리비전에 고정된다.

### 관리 컴포넌트 (IDF Component Manager · 자동 다운로드)

`main/idf_component.yml` 이 직접 선언하고 나머지는 esp-matter 가 끌어온다. 잠금은
`dependencies.lock`(매니페스트 `2.0.0` · target `esp32c6`):

| 컴포넌트 | 버전 | 용도 |
|---|---|---|
| `espressif/qrcode` | **0.2.0** | OLED 페어링 QR 인코더 (직접 선언 `*` → 잠금 0.2.0) |
| `espressif/cmake_utilities` | 1.1.1 (`^1`) | 빌드 유틸 (직접 선언) |
| ~~`espressif/button`~~ | ~~4.1.6~~ | **2026-08-24 제거** — 20 ms 주기 `esp_timer` 를 상시 돌려 깨어남의 78 %를 차지했다. 공장초기화 BOOT 버튼 하나에만 쓰였고(사용 안 함), 실제 조작 버튼은 전부 PCF8575 → `button_handler` 가 처리한다 |
| `espressif/mdns` | 1.11.1 | Matter mDNS / SRP |
| `espressif/esp_secure_cert_mgr` | 2.9.2 | DAC 인증서 파티션 |
| `espressif/esp_delta_ota` · `esp_encrypted_img` | 1.1.4 · 2.3.0 | OTA 델타 · 암호화 |

## 참조 프로젝트

본 펌웨어가 의존하거나 참조한 프로젝트와 git 주소.

| 프로젝트 | 버전 / 리비전 | 용도 | Git |
|---|---|---|---|
| **이 프로젝트** | v3.x (Matter over Thread) | Somfy RTS · Matter-over-Thread 펌웨어 | <https://github.com/kgun2g/somfy-rts-remote-by-thread> |
| **ESP-IDF** | **v5.4.1** | ESP32 빌드 SDK/RTOS (Apache-2.0) | <https://github.com/espressif/esp-idf> |
| **esp-matter** | `7706cfbd` (main) | Matter SDK (Apache-2.0) | <https://github.com/espressif/esp-matter> |
| **connectedhomeip** | esp-matter 동봉 submodule | Matter 코어 SDK (esp-matter 가 래핑, Apache-2.0) | <https://github.com/project-chip/connectedhomeip> |
| **esp-idf-ssd1306** | `554df45` (master) | SSD1306/SSD1315 OLED 컴포넌트 (MIT) | <https://github.com/nopnop2002/esp-idf-ssd1306> |
| **ESPSomfy-RTS** | fork (447 분기 · `modulation==0`) | Somfy RTS 프로토콜 참조 구현 (447 분기 작업의 기준) | <https://github.com/kgun2g/ESPSomfy-RTS> |
| **rtl_433** | fork (`somfy-RTS-447` device) | RF 디코딩·검증(송신 프레임 역검증) | <https://github.com/kgun2g/rtl_433> |

> Somfy RTS 447 분기는 본 저장소와 더불어 `ESPSomfy-RTS`·`rtl_433` 의 로컬 fork(`modulation==0`
> / `somfy-RTS-447`)에도 반영했다. 각 의존성은 자체 라이선스(대부분 Apache-2.0/MIT)를 따른다.


| 역할 | Fork (kgun2g) | Upstream |
|---|---|---|
| ESPSomfy-RTS (447 MHz 지원 추가) | https://github.com/kgun2g/ESPSomfy-RTS | https://github.com/rstrouse/ESPSomfy-RTS |
| rtl_433 (447 MHz 지원 추가) | https://github.com/kgun2g/rtl_433 | https://github.com/merbanan/rtl_433 |
| SDR# plugin (447 MHz 지원 추가, SDR# 측 UI) | https://github.com/kgun2g/plugin-Rtl433-for-SdrSharp | https://github.com/marco402/plugin-Rtl433-for-SdrSharp |
| SDR# plugin DLL (447 MHz 지원 추가) | https://github.com/kgun2g/Rtl_433_dll-for-plugin-SdrSharp | https://github.com/marco402/Rtl_433_dll-for-plugin-SdrSharp |
| GraphLibSpecific (.NET 9 재빌드) | https://github.com/kgun2g/GraphLibSpecific | https://github.com/marco402/GraphLibSpecific |


## 라이선스

[MIT 라이선스](LICENSE) — 자유롭게 사용·수정·배포 가능(무료/오픈소스). © 2026 kgun2g.
저작권 고지와 라이선스 전문을 포함하면 된다. 외부 의존성(ESP-IDF·esp-matter 등)은
각 프로젝트의 라이선스를 따른다.

## 참고

- 핀맵·빌드·버전의 현행 기준은 본 README 와 `main/boards/<board>.h` 다.
  다른 문서와 상충 시 본 문서 우선.
