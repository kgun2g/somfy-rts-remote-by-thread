# 테스트 (재사용 가능)

ESP32-C6 Somfy RTS 컨트롤러 v3.5 의 **가상(on-target, 온에어 불필요)**
테스트 모음. 케이스별로 정리되어 있고, 빌드 스크립트를 포함하여 나중에
다시 활용할 수 있다.

## 구조

```
test/
├─ README.md
├─ build_test.ps1            테스트 펌웨어 빌드/플래시 (build_test/ 에 산출)
└─ somfy_cases/
   ├─ somfy_selftest.h
   ├─ somfy_selftest.c       [virtual] 블라인드 1~5 × 케이스별 가상 검증
   ├─ somfy_onair_test.h
   └─ somfy_onair_test.c     [onair]   실제 CC1101 온에어 RF 송신 검증
```

## 두 가지 테스트 모드 (`-Mode`, 택일·별도 빌드)

| 모드 | 빌드 플래그 | 내용 | RF |
|---|---|---|---|
| `virtual` (기본) | `-DSOMFY_SELFTEST=1` | 7바이트 프레임 정합성 + **ALL 블록 라우팅**(가상) | 불필요 |
| `onair` | `-DSOMFY_ONAIR_TEST=1` | 실제 CC1101 로 진짜 신호 송신 | **필요** |

### onair 케이스 (실제 신호 발생)

| Phase | 내용 |
|---|---|
| A | CC1101 SPI 통신/칩 확인 (PARTNUM/VERSION) |
| B | 6개 주파수 + 기기 설정 주파수로 변경 → FREQ 레지스터 라이트백 + 실제 캐리어(TX state) 송신 검증 |
| C | 가상 버튼(UP/DOWN/MY/PROG) × 블라인드 1~5 번갈아 실제 변조 송신 |
| D | ALL 선택 시 5개 블라인드 동시(순차) 실제 송신 + PROG 차단 정책 검증 |

> onair 는 실제 RF 를 공중에 송신한다. 공유 인스턴스
> `g_cc1101/g_somfy/g_mgr` (app_main.cpp Phase2 초기화분)을 그대로 사용하며
> NVS·실제 롤링코드는 건드리지 않는다(로컬 복제). CC1101 하드웨어가
> 미응답이면 명확한 진단 로그와 함께 중단(`결과: 0/0, 하드웨어 미준비`).

- 정상 펌웨어 빌드(`build/`)와 **분리된 `build_test/`** 에 산출 → 테스트
  펌웨어가 보존되어 재활용 가능. 검증된 루트 `sdkconfig` 를 공유한다.
- `-DSOMFY_SELFTEST=1` 일 때만 테스트 코드가 컴파일/실행된다. 일반
  빌드(`build.ps1`)에는 전혀 포함되지 않는다.

## 케이스 (블라인드 1~5 각각)

| 케이스 | 모델링한 RTS 절차 |
|---|---|
| 신규 등록 | PROG(long) |
| 상한 설정 | PROG → UP → MY |
| 하한 설정 | PROG → DOWN → MY |
| 복사 | 원격 식별자(주소/주파수/롤링) 복제 후 동일 주소 전송 검증 |
| 위치 기억 | MY(long, favorite 기억) |

검증 항목(프레임별): 커맨드 nibble, 24bit 주소, 롤링코드(스텝마다 +1),
체크섬 nibble. CC1101/RF/NVS 를 건드리지 않는 순수 가상 검증
(`somfy_rts_test_build_frame()`).

### ALL(블록) 라우팅 케이스 (`_all_route_tests`)

`blind_manager_get_targets()`(실제 함수) + `somfy_app.c` `_do_rf_send` 라우팅을
재현해 **실제 산출 주소(eFuse)** 로 검증한다. 채널 수에 따라 자동(H2 3채널=1블록,
C6 8채널=2블록):

| 검증 | 내용 |
|---|---|
| 주소 규칙 | `F0` prefix(carry 없음) · 채널 등차 `0x2700` · `ALL=base+4×0x2700` · base mid<0x27 |
| ALL + UP/DOWN/MY | 블록별 ALL 주소로 송신(블록 수만큼) + **모든 채널 position 정확히 1회** 갱신(100/0/50) |
| ALL + PROG | **차단**(송신 0 — ALL 은 PROG 없음) |
| 개별 채널 | 선택 채널만 송신/갱신, 나머지 미변경. 개별+PROG 는 정상 송신 |
| 포인터 판별 | 채널(`off<MAX`) vs ALL 블록(`off≥MAX`) 구분 |

매니저는 `blind_manager_test_populate()`(테스트 전용, **NVS 미접근**)로 채운다.
이 훅과 라우팅 케이스는 `SOMFY_SELFTEST`/`SOMFY_ONAIR_TEST` 빌드에서만 컴파일되어
**프로덕션 `.bin` 은 불변**이다(GNPE byte-identical 보존).

## 실행

```powershell
# 가상 프레임 검증 (기본)
./test/build_test.ps1 -Action build -Mode virtual
./test/build_test.ps1 -Action flash -Mode virtual -Port COM3

# 실제 CC1101 온에어 RF 송신 검증
./test/build_test.ps1 -Action build -Mode onair
./test/build_test.ps1 -Action flash -Mode onair -Port COM3
```

부팅 직후 표준 레벨 로깅(esp_log)으로 출력:
- `I (SELFTEST) [PASS] 블라인드N <케이스>`
- `E (SELFTEST) [FAIL] 블라인드N <케이스>`
- `W (SELFTEST) ===== 결과: X/Y PASS, Z FAIL =====`

전체 통과 시 `Z FAIL` = 0.

## 일반 펌웨어로 복귀

테스트는 별도 `build_test/` 라 일반 빌드에 영향 없음. 평소 운영
펌웨어는 그대로:

```powershell
./build.ps1 -Action build
./build.ps1 -Action flash -Port COM3
```
