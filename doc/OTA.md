# 펌웨어 업데이트 — Matter OTA over Thread

이 디바이스는 **Thread 위 Matter OTA** 로 무선 펌웨어 업데이트를 받는다.
WiFi·USB 불필요(USB 는 개발용 플래시로만). 설정 메뉴 **"FW Update"** 에서 현재
버전·진행 상태를 보고 수동 업데이트 확인을 트리거할 수 있다.

## 동작 개요

```
[빌드] somfy_blinds.bin (버전 N+1)
   │  image_tool.py 로 Matter OTA 이미지 생성 (.ota)
   ▼
[OTA Provider]  ← chip-tool / ota-provider-app / 허브
   │  AnnounceOTAProvider  또는  디바이스의 수동 QueryImage
   ▼
[디바이스 OTA Requestor]  버전 비교(N+1 > N) → BDX 다운로드 → ota_1 기록 → 재부팅
```

- 디바이스: **듀얼 OTA 파티션**(`ota_0`/`ota_1` 각 1.875 MB) + `CONFIG_ENABLE_OTA_REQUESTOR=y`.
- 현재 보고 버전: `CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER`(= **35**, `sdkconfig`/
  `sdkconfig.defaults`). OTA 이미지는 반드시 **이보다 큰** SoftwareVersion 이어야
  Provider 가 "업데이트 있음" 으로 응답한다.
- 암호화 OTA 비활성(`CONFIG_ENABLE_ENCRYPTED_OTA` 미설정) → **평문 .ota 이미지**.

## 1. 새 펌웨어 빌드 (버전 올리기)

업데이트를 내보내려면 **버전을 반드시 증가**시킨다 (현재 35 → 36).

| 위치 | 항목 | 예 (35 → 36) |
|---|---|---|
| `CMakeLists.txt` | `PROJECT_VER` / `PROJECT_VER_NUMBER` | "3.6" / 36 |
| `sdkconfig` + `sdkconfig.defaults` | `CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER` | 36 |

```powershell
# 버전 수정 후
./build.ps1 -Board gnpe-c6 -Action build
# 산출물: build/somfy_blinds.bin  (SoftwareVersion = 36)
```

> ⚠️ `DEVICE_SOFTWARE_VERSION_NUMBER` 와 `PROJECT_VER_NUMBER` 를 함께 올릴 것.
> 전자는 OTA 비교 기준, 후자는 OLED·`esp_app_desc` 표시 버전.
>
> 🔢 `.ota` 파일명·SoftwareVersionString 의 VERSION 은 **4자리 zero-pad**(예 36 →
> `v0036` / `0036+…`). 숫자 비교용 `vn`(=`DEVICE_SOFTWARE_VERSION_NUMBER`)은 원본
> 값 그대로라 OTA 비교에는 영향 없다.

## 2. Matter OTA 이미지 생성 (.ota) — 보드별

★ **보드(C6/H2)마다 SoC·바이너리가 다르므로 OTA 이미지도 보드별로 따로
만들어야 한다.** 잘못된 보드 이미지를 받지 않도록 **보드별 고유 Product ID** 로
구분한다 (Matter Provider 가 PID 로 매칭).

| 보드 | 브랜드/제품 | SoC | Product ID | sdkconfig |
|---|---|---|---|---|
| `gnpe-c6` | GNPE ESP32-C6-0.42 (검증) | esp32c6 | **0x8000** | `sdkconfig` / `.c6_thread` |
| `xiao-c6` | Seeed XIAO ESP32-C6 (검증) | esp32c6 | **0x8003** | `.c6_thread` + `.xiao_c6` 오버레이 |
| `esp32-h2` | ESP32-H2 SuperMini (검증, I2C 공유) | esp32h2 | **0x8001** | `.h2_thread` |

(VendorID 는 공통 0xFFF1.)

> ★★ **같은 SoC·다른 브랜드 주의** (`gnpe-c6` vs `xiao-c6`): 둘 다 esp32c6 라
> **chip-ID 안전망이 둘을 구분하지 못한다**(이미지가 칩에선 부팅됨). 핀맵만
> 달라 잘못 적용되면 CC1101/버튼이 엉뚱한 GPIO 에 붙어 오동작한다. 따라서
> **Product ID(0x8000 vs 0x8003)가 유일한 보드 식별자** — `.ota` 생성 시
> 반드시 대상 브랜드 PID 와 일치시킬 것. `build.ps1 -Action ota-image` 는
> 보드 sdkconfig 의 PID 를 자동 사용하므로 안전하다.

### build.ps1 로 자동 생성 (권장)

`-Action ota-image` 가 빌드된 `.bin`, **그 보드 sdkconfig 의 VID/PID/버전**,
**`boards/<board>.h` 의 HW 변형(PCF·EC 플래그 + OLED 해상도·회전)**을 읽어
`dist/somfy_blinds_<board>_<pcf>_<enc>_<oled>_v<NNNN>.ota` 를 만든다
(`<oled>`=`<W>x<H>r<0|180>[m]`, `m`=좌우반전, VERSION 4자리) — PID·변형 자동 반영.

```powershell
./build.ps1 -Board gnpe-c6 -Action build        # 버전 올린 뒤 빌드
./build.ps1 -Board gnpe-c6 -Action ota-image     # → dist/somfy_blinds_gnpe-c6_8574_ec11_72x40r180_v0036.ota (PID 0x8000)

./build.ps1 -Board xiao-c6 -Action build
./build.ps1 -Board xiao-c6 -Action ota-image     # → dist/somfy_blinds_xiao-c6_8574_ec05_128x64r0_v0036.ota (PID 0x8003)
```

### 수동 (CHIP 도구 직접)

```bash
python connectedhomeip/src/app/ota_image_tool.py create \
  -v 0xFFF1 -p 0x8000 \                        # ← PID 는 대상 보드 값 (C6=8000 H2=8001)
  -vn 36 -vs "0036+8574.ec11.72x40r180" \      # ← vs=<NNNN>+<pcf>.<enc>.<oled> (VERSION 4자리)
  -da sha256 \
  build/somfy_blinds.bin  dist/somfy_blinds_gnpe-c6_8574_ec11_72x40r180_v0036.ota
```

| 옵션 | 의미 |
|---|---|
| `-v` / `-p` | Vendor / **Product ID (대상 보드값과 일치 필수)** |
| `-vn` | SoftwareVersion **숫자**(비교 기준, 디바이스 현재값보다 커야 함, 예 36) — 파일명/vs 는 4자리 표기 |
| `-vs` | SoftwareVersion **문자열** — `<NNNN>+<pcf>.<enc>.<oled>` 로 HW 변형 태그 부착(식별용) |
| `-da` | digest 알고리즘 (sha256) |

> 암호화 OTA 를 쓰려면 `CONFIG_ENABLE_ENCRYPTED_OTA=y` + 키 지정 필요. 현재
> 프로젝트는 평문이므로 위 명령 그대로.

### 보드 안전망 — chip-ID 검증 (이중 방어)

설령 잘못된 보드 이미지가 전달돼도, **ESP-IDF 부트로더가 이미지 헤더의 chip_id
(esp32c6/esp32h2)를 실행 칩과 대조**해 불일치면 부팅을 거부하고 이전
슬롯으로 롤백한다.

| 경우 | 1차 PID 매칭 | 2차 chip-ID |
|---|---|---|
| 다른 SoC (c6 ↔ h2) | 막음 | **추가로 막음**(부팅 거부+롤백) |
| 같은 SoC·다른 브랜드 (`gnpe-c6` ↔ `xiao-c6`) | 막음 | **구분 못 함**(둘 다 esp32c6) |

→ 같은 SoC 브랜드 구분은 **PID 가 유일 방어선**이므로 보드별 PID 분리가 필수.

### 하드웨어 변형(variant) 구분 — PCF8574/8575 · EC11/EC05 · OLED

PID 는 **보드(브랜드)** 까지만 구분한다. 같은 보드라도 주변장치 빌드 변형이
다르면 펌웨어가 호환되지 않는다:

- **PCF8574(8핀) ↔ PCF8575(16핀)** — 버튼 read/write 폭이 1↔2 byte 로 다르다
  (`BOARD_HAS_LR_BUTTONS`). 잘못 받으면 좌/우 버튼 미동작·입력 오독.
- **EC11(full-step) ↔ EC05(half-step)** — 로터리 디코더가 다르다
  (`BOARD_ROT_HALF_STEP`). 잘못 받으면 한쪽 회전이 안 먹거나 방향이 뒤집힌다.
- **OLED 해상도(72x40 · 128x64 · 64x128) + 회전(0°·180°) + 좌우반전** — 렌더러·표시
  방향이 다르다 (`BOARD_OLED_WIDTH×HEIGHT` · `BOARD_OLED_ROTATE_180` · `BOARD_OLED_FLIP_X`).
  잘못 받으면 화면이 깨지거나 상하/좌우 반전된다.

변형 태그는 **`<pcf>.<enc>.<oled>`** 형식이다(예 `8574.ec05.128x64r0`).
PCF·EC 는 2비트 코드로도 인코딩한다(`board_select.h` `BOARD_HW_VARIANT_*`):

| 코드 | PCF | 엔코더 | `HAS_LR_BUTTONS` | `ROT_HALF_STEP` | PCF·EC 태그 |
|---|---|---|---|---|---|
| 0 | PCF8574 | EC11 | 0 | 0 | `8574.ec11` (GNPE·H2 기본) |
| 1 | PCF8575 | EC11 | 1 | 0 | `8575.ec11` |
| 2 | PCF8574 | EC05 | 0 | 1 | `8574.ec05` (**XIAO**) |
| 3 | PCF8575 | EC05 | 1 | 1 | `8575.ec05` |

> OLED 는 위 코드와 **직교(orthogonal)** 축이라 코드에 넣지 않고 해상도+회전+반전 토큰
> `<W>x<H>r<0|180>[m]`(`BOARD_HW_OLED_STR`, `m`=좌우반전 `BOARD_OLED_FLIP_X`)로 태그 끝에
> 붙인다 — 예 코드2 + 128x64r0 → `8574.ec05.128x64r0`. 현재: **GNPE `72x40r180`**,
> H2 `72x40r0`, XIAO `128x64r0`. 회전·반전 4종 토큰: `r0` 정방향 / `r180` 180° /
> `r0m` 좌우반전 / `r180m` 상하반전 (GNPE/H2 는 같은 72×40 라도 토큰이 갈린다 — PID 와 별개 2차 표식).

> ⚠️ **Matter OTA 는 VID/PID + SoftwareVersion(숫자)만 매칭** → 같은 PID·버전이면
> 변형을 가리지 못한다. 따라서 변형은 **이미지 식별자**로 실어 운영자가 맞는
> 파일을 서빙하게 한다(프로토콜이 자동 차단하지는 않음):
>
> - **`.ota` 파일명**: `somfy_blinds_<board>_<pcf>_<enc>_<oled>_v<NNNN>.ota`
>   (예 `somfy_blinds_xiao-c6_8574_ec05_128x64r0_v0035.ota`) — 운영자 선택 시 1차 방어선.
> - **SoftwareVersionString**(`-vs`): `<NNNN>+<pcf>.<enc>.<oled>`
>   (예 `0035+8574.ec05.128x64r0`) — Provider/허브 로그·이미지 헤더에 남는 식별 문자열.
> - **인코딩 단일 진실원천**: `board_select.h` 의 `BOARD_HW_VARIANT_*` 매크로.
>   `build.ps1 ota-image` 가 같은 규칙으로 `boards/<board>.h` 플래그를 읽어
>   파일명·`vs` 를 만든다 — **OTA 식별은 런타임이 아닌 빌드/패키징 책임**이라
>   펌웨어 동작/바이너리는 바뀌지 않는다(GNPE 코드 불변 검증 완료).

> 🖥️ **왜 화면 자기표시는 안 두나**: FW Update 화면은 72×40 논리 캔버스(128×64
> 패널에서도 중앙 72px)라 `v3.5 8574.ec05` 가 한 줄에 안 들어가 **ec05/ec11 구분
> 글자가 잘린다**. 디바이스측 식별이 필요하면 부팅 시 시리얼 로그로
> `BOARD_HW_VARIANT_STR` 를 남기는 방식이 가볍다(추후).

> ❓ **왜 자동감지·자동롤백이 아닌가**: EC11/EC05 는 기계적 디텐트만 달라 전기적
> 으로 구분 불가(사용자가 돌려봐야 앎), PCF8574/8575 도 0x20 을 공유해 신뢰성
> 있는 자동감지가 어렵다. 잘못된 자동롤백은 부팅 루프 위험 → **식별자 + 운영자
> 확인**을 구분 수단으로 둔다.

> 🔒 **하드 차단이 필요하면**: 변형마다 **별도 Product ID** 를 할당하면 Matter 가
> 프로토콜 레벨에서 막는다(대신 변형마다 커미셔닝 정체성·인증서가 갈라져 무겁다).
> 기본 권장은 위 파일명/vs 태그 방식.

## 3. OTA Provider 실행 + 디바이스에 알림

가장 단순한 검증 경로 = CHIP `ota-provider-app` + `chip-tool`.

```bash
# (a) Provider 가 .ota 파일 서빙
./chip-ota-provider-app --discriminator 22 --secured-device-port 5565 \
    --KVS /tmp/provider.kvs --filepath somfy_blinds_v36.ota

# (b) Provider 를 같은 fabric 에 커미셔닝 (node 0x1)
./chip-tool pairing onnetwork 0x1 20202021

# (c) Provider ACL 에 디바이스 접근 허용
./chip-tool accesscontrol write acl '[{...Operate, subjects:[<deviceNodeId>]...}]' 0x1 0

# (d) 디바이스(node 0xDEV)에 Provider 위치 알림 → 자동 다운로드/적용
./chip-tool otasoftwareupdaterequestor announce-otaprovider 0x1 0 0 0 0xDEV 0
```

- 알림을 받으면 디바이스 OTA Requestor 가 QueryImage → 버전 비교 → BDX 다운로드
  → `ota_1` 기록 → 자동 재부팅 → 새 슬롯 부팅.
- **설정 메뉴 "FW Update"** 화면에서 진행을 볼 수 있다:
  `Idle → Checking.. → DL nn% → Applying.. → (재부팅)`.

> SmartThings/Apple/Google 허브가 OTA Provider 역할을 지원하면 위 chip-tool
> 과정 없이 허브를 통해 배포 가능(허브별 지원 편차 있음).

## 4. 설정 메뉴 "FW Update" 사용법

| 조작 | 동작 |
|---|---|
| 메뉴 → `FW Update` (SET) | 화면 진입 — 현재 버전(`v3.5`) + OTA 상태 표시 |
| **SET** | 수동 업데이트 확인 — 기본 Provider 에 즉시 QueryImage |
| **STOP** | 메뉴 복귀(짧게) / 메인(길게) |

> 🏷️ HW 변형(PCF8574/8575·EC11/EC05)은 화면이 아니라 **`.ota` 파일명·
> SoftwareVersionString** 에 실린다([§하드웨어 변형 구분](#하드웨어-변형variant-구분--pcf85748575--ec11ec05)).
> 디바이스 버전 화면은 PROJECT_VER(`v3.5`)만 표시.

상태 라인: `Idle`(대기) / `Checking..`(질의) / `DL nn%`(다운로드) /
`Applying..`(적용) / `Delayed`(지연) / `N/A`(requestor 미준비).

> "SET=Check" 는 **기본 OTA Provider 가 설정돼 있을 때만** 의미가 있다(없으면
> 무동작). 일반적으로는 Provider/허브가 `AnnounceOTAProvider` 로 밀어주는 흐름.

## 구현 위치 (코드)

| 파일 | 역할 |
|---|---|
| `sdkconfig` / `sdkconfig.defaults` | `CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER=35`, OTA Requestor/듀얼 파티션 |
| `partitions.csv` | `ota_0` / `ota_1` / `otadata` + `fctry`(Matter) + `rollcode`(롤링코드 영속) |
| `main/app_main.cpp` | OTA Requestor 초기화(esp_matter::start 자동 + 암호화 분기) |
| `main/boards/board_select.h` | **HW 변형 인코딩 정의** `BOARD_HW_VARIANT_CODE/_STR/_OLED_STR`(PCF8574/8575·EC11/EC05·OLED 해상도/회전, canonical) + 보드별 PID 분기 |
| `main/matter_blinds_shim.cpp` | C 브리지: `matter_ota_version_str` / `_get_state` / `_trigger_check` |
| `main/somfy_app.c` | 설정 메뉴 `SETUP_FW_UPDATE` + 매-tick 상태 갱신 |
| `main/oled_ui.c` | `_render_fw_update` (버전·상태·진행률) |
| `build.ps1` (`ota-image`) | `.bin`→`.ota` 생성, PID(sdkconfig)+**HW 변형**(board 헤더)을 파일명·`-vs` 에 반영 |

## 알려진 제약

- **앱 크기 vs 슬롯**: 현재 1.65 MB / 슬롯 1.875 MB(여유 ~150 KB). 기능 추가로
  앱이 커지면 슬롯 한계에 주의(초과 시 파티션 재설계 또는 8 MB 플래시 필요).
- **버전 단조 증가 필수**: OTA 이미지 SoftwareVersion 이 현재값 이하이면 무시됨.
- **롤백**: 새 이미지가 부팅 자가검증 실패 시 ESP-IDF app rollback 으로 이전 슬롯
  복귀(앱이 `esp_ota_mark_app_valid_cancel_rollback()` 를 호출해야 확정).
- **롤링코드/블라인드 주소 보존**: OTA 는 앱 슬롯(`ota_0`/`ota_1`)만 갱신 → `rollcode`·
  `nvs`·`fctry` 데이터 파티션은 그대로. OTA 후에도 롤링코드 동기 유지, 재페어링 불필요.
  블라인드 주소는 eFuse MAC 산출이라 펌웨어와 무관하게 항상 동일.
