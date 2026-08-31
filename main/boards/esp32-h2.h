#pragma once
/*
 * boards/esp32-h2.h
 * ──────────────────────────────────────────────────────────
 * 보드: ESP32-H2 SuperMini (저가 호환 보드) — 미검증
 *   • SoC          : ESP32-H2 (RISC-V, single core, 802.15.4 + BLE, WiFi 없음)
 *   • IDF target   : esp32h2
 *   • 핀 출처      : doc/esp32/SuperMini/ (핀맵 esp32-h2-supermini0.jpg,
 *                    esp32_h2_superMini1.jpg + 회로도 esp32-h2-superMini2.png)
 *
 * ★ ESP32-H2 특성:
 *   • 802.15.4(Thread/Zigbee) + BLE → C6 와 동일하게 **Matter over Thread**
 *     (WiFi 미지원이므로 Thread 가 유일 트랜스포트). Thread Border Router 필요.
 *
 * ★ SuperMini 노출 핀 (실측 핀맵):
 *   [좌] GP24(TX) GP23(RX) GP0 GP1 GP2 GP3 GP4 GP5 GP8 GP26 GP27
 *   [우] GP14 GP13 GP12 GP11 GP10 GP9 GP22 GP25
 *   - FSPI 네이티브: FSPICLK=GP4, FSPID=GP5, FSPIQ=GP0, FSPICS0=GP1
 *     → CC1101 SPI 를 여기에 배치(IO-MUX 최적).
 *   - **미노출/금지**: GP6·GP7(미노출), GP8(RGB LED/LOG·strapping),
 *     GP9(BOOT strapping), GP26/GP27(USB D-/D+), GP15~21(in-package flash).
 *
 * ★★ I2C 공유 (이 보드의 핵심):
 *   ESP32-H2 에는 LP_I2C 가 없고 핀도 빠듯하므로, **OLED 와 PCF8574 를
 *   하드웨어 I2C 한 버스에 공유**한다(SDA/SCL 동일 핀, 주소만 다름:
 *   OLED 0x3C / PCF8574 0x20). → BOARD_I2C_SHARED=1.
 *   (GNPE 는 PCF=비트뱅으로 버스 분리. XIAO 는 LP_I2C(6/7) 미연결 시 이 공유 버스로
 *    런타임 자동 폴백(BOARD_I2C_LP_FALLBACK). 각 보드 헤더 참고.)
 *   공유 시 펌웨어는 PCF8574 도 OLED 와 같은 HW I2C 버스로 폴링한다
 *   (button_handler.c 의 BOARD_I2C_SHARED 분기).
 *
 *   첫 빌드 전: ./build.ps1 -Board esp32-h2 -Action set-target 1회 수행.
 * ──────────────────────────────────────────────────────────
 */

#define BOARD_NAME "esp32-h2"
#define BOARD_IDF_TARGET "esp32h2"

/* ════════════════════════════════════════════════════════
   CC1101 SPI  (FSPI IO-MUX 네이티브 핀)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_CC1101_SCK 4  // GP4  FSPICLK
#define BOARD_PIN_CC1101_MISO 0 // GP0  FSPIQ
#define BOARD_PIN_CC1101_MOSI 5 // GP5  FSPID
#define BOARD_PIN_CC1101_CS 1   // GP1  (FSPICS0)
#define BOARD_PIN_CC1101_GD0 10 // GP10 Async TX (RMT)

/* ════════════════════════════════════════════════════════
   I2C — OLED + PCF8574 공유 버스 (HW I2C, SDA/SCL 동일 핀)
     외부 4.7kΩ pull-up 필수(SDA/SCL 각 1). 주소: OLED 0x3C / PCF8574 0x20.
     핀은 고정 아님(GPIO 매트릭스) — 현재 GP13/GP14.
       ※ GP13/GP14 는 32.768kHz xtal 겸용핀. SuperMini 는 외부 32K 크리스털을
         실장하지 않으므로 일반 GPIO 로 사용 OK(외부 32K 를 달 거면 다른 핀으로
   옮길 것). ════════════════════════════════════════════════════════ */
#define BOARD_I2C_SHARED 1
#define BOARD_PIN_OLED_SDA 13 // GP13  ─┐ 공유 SDA
#define BOARD_PIN_OLED_SCL 14 // GP14  ─┤ 공유 SCL
#define BOARD_PIN_PCF_SDA 13  // GP13  ─┘ (OLED 와 동일)
#define BOARD_PIN_PCF_SCL 14  // GP14     (OLED 와 동일)
#define BOARD_PIN_PCF_INT 11
/* ★★2026-08-25 `~INT` ISR **켠다(1)** — H2 에서는 검증한 적이 없다.
 *  H2 는 LP 코어가 없어 버튼을 10ms 마다 HW I2C 로 읽고, 그 트랜잭션마다
 *  NO_LIGHT_SLEEP 락을 잡았다 놓는다(PM 락 덤프: I2C_0 748,352회). 그래서
 *  깨어남이 **113.9회/초**다(C6 는 23.7). C6 에서 쓴 LP 눌림 래치는 LP 코어가
 *  전제라 H2 엔 못 쓰므로, `~INT` 가 유일한 대안이다.
 *  C6 에서 4,500회/초로 실패했던 근거는 **그 보드의 비트뱅 I2C 크로스토크**였다
 *  (board_select.h 주석 참조) — H2 는 HW I2C·다른 핀이라 조건이 다르다. */
/* ★★★2026-08-25 **되돌림(0)** — 실사용 판정: "버튼이 가끔 눌려지고, 계속 재부팅한다".
 *  정지 상태 계측은 깨끗했다([BTNWAKE] ~INT 0건 / 3분, C6 의 4,500회/초와 정반대).
 *  즉 **크로스토크 가설은 맞았고**, 깨진 것은 다른 부분이다 — 원인 규명 전까지 끈다. */
#define BOARD_PCF_INT_ISR 1
  // GP11  PCF8574 ~INT (active-LOW, wake)

/* ── OLED 디스플레이 규격 (외부 모듈 — 실제 구성: 0.96" 128×64) ──
 *   128×64 표준 패널이라 COL_OFFSET=0 / FIXUP=0. sdkconfig.esp32-h2 의
 *   CONFIG_OFFSETX=0 · CONFIG_SSD1306_128x64=y 와 일치.
 *   (72×40 패널로 바꾸려면 WIDTH=72/HEIGHT=40/COL_OFFSET=28/FIXUP=1 +
 *    CONFIG_OFFSETX=28.)                                                */
#define BOARD_OLED_WIDTH 128
#define BOARD_OLED_HEIGHT 64
#define BOARD_OLED_COL_OFFSET 0
/* ★★★2026-08-16 0 → 1. 이 시제품은 OLED 가 **뒤집혀 장착**돼 있다
 *  (doc/wiring/README.md 및 SETUP_GUIDE 의 `-Rotate 180 필수` 주석).
 *  기본값이 0 이면 빌드할 때마다 `-Rotate 180` 을 손으로 붙여야 하는데,
 *  한 번만 빠뜨려도 화면이 상하 뒤집힌다. → 기본값을 기판에 맞춘다. */
#define BOARD_OLED_ROTATE_180 1 // 시제품 기판이 180° 장착

/* ★★2026-08-27 OLED I2C **200kHz** — 화면 갱신이 느린 원인이었다.
 *  100kHz 에서는 프레임 전송이 92.9ms 라 UI 태스크 주기(50ms=20fps)를 못 따라가
 *  실질 약 **10.8fps** 였다. 200kHz 면 46.4ms 로 **20fps 를 정확히 채운다.**
 *  400kHz 로 더 올려도 태스크가 병목이라 실익이 없고, 100kHz 를 도입한 원인이던
 *  글리치 마진을 절반 남긴다.
 *  ※그 글리치는 **C6 실측**이고 H2 에서 400k 를 검증한 적은 없다(보드가 다르면
 *    결과도 다르다 — `~INT` 가 그랬다). 이상 시 100000 으로 되돌릴 것.
 *  관찰 지표: [OLEDMON] 의 `실패`·`lockTO` 가 늘면 되돌린다. */
#define BOARD_OLED_I2C_HZ 200000
#define BOARD_OLED_FIXUP_72X40 0
#define BOARD_OLED_ADDR 0x3C

/* 로터리: 실제 구성 EC05 (하프스텝 — 그레이코드 LUT 누산 디코더) */
#define BOARD_ROT_HALF_STEP 1

/* ── Matter 구조 = composed (RAM 절약) ─────────────────────────────────
 *   H2 는 free 가 빠듯하므로 Bridge(Aggregator+bridged_node, 노드당 ~수십KB) 대신
 *   composed(root 직속 WindowCovering)를 쓴다. app_main.cpp 가 이 매크로로 분기.
 *   효과: Bridge 땐 free 2.5KB(블라인드 2개 한계) → composed 로 free 72KB(블라인드 5개).
 *   단 BLE(CHIPoBLE 커미셔닝)까지 켜면 5개는 CHIP PacketBuffer 가 소진돼 커미셔닝(BLE
 *   연결)이 NO_MEMORY(err=b)로 실패한다(SmartThings 39-100). 3개로도 커미셔닝 중
 *   NimBLE mbuf(heap)·CHIP PacketBuffer 가 부족(ble_hs_mbuf_from_flat failed / pool EMPTY,
 *   39-104)해 PASE 가 실패 → 2개로 더 낮춰 heap 을 확보한다. (CHIP PacketBuffer 풀은
 *   main/CHIPProjectConfig.h 에서 24 로, CHIP task 우선순위는 sdkconfig 에서 15 로 올림.
 *   C6 는 RAM 여유로 board_select 기본 5 유지.) */
#define BOARD_MATTER_COMPOSED 1
/* 사용자 요청: 블라인드 3개. heap 확보책(BOARD_DISABLE_TIME 시간제거 + OT/MDNS/EVENT 축소 +
 *  CHIP PacketBuffer 풀 main/CHIPProjectConfig.h + CHIP task priority 15)으로 2개에선 페어링
 *  통과 확인. 3개는 endpoint 가 늘어 heap 이 더 빠듯 — 재커미셔닝(PASE peak) 시 부족하면 2개로. */
#define BLIND_MAX_COUNT 3

/* 시간/날짜 비활성 — BLE 커미셔닝 heap 확보(time 태스크·SNTP 제거). 측정상 BLE 켠
 *  H2 의 free heap 이 ~2.5KB 까지 떨어져 PASE peak(20~30KB)를 못 버티므로, 시간 표시를
 *  희생해 heap 을 회복한다. (시계 화면→블라인드, 화면보호기→시간 없는 형태: oled_ui.c) */
#define BOARD_DISABLE_TIME 1

/* OTA 미지원 — 설정 메뉴에서 FW Update 항목 제거(사용자 요청). */
#define BOARD_DISABLE_OTA 1

/* ★★★2026-08-15 진단 로그 링버퍼 축소 — heap 확보(사용자 지시).
 *  두 링은 정적 배열(.bss)이라 그대로 heap 을 깎는다. H2 는 BLE 커미셔닝
 *  PASE peak 때문에 heap 이 빠듯해, 배터리 ADC 실측(BOARD_BAT_SWAPPED=0)을 켜자
 *  CHIP PacketBuffer pool EMPTY + linenoise malloc 실패로 죽었다.
 *    진동 로그 : 센서 고장 원인(센서 불량)이 이미 규명돼 상시 기록 불필요 → **OFF** (-1,024B)
 *    방전 로그 : % 실측 검증에 필요 → 유지하되 300 → **128 건**으로 축소 (-1,376B)
 *  합계 약 2.4KB 확보. C6 는 기본값(300 / 128) 그대로 — 절전 측정에 긴 기록이 필요하다. */
#define BOARD_VIBELOG_ENABLE 0
/* ★★★2026-08-16 light sleep 임시 차단 — 배터리 구동 중 패닉 반복.
 *  근거: 부팅진단 리셋사유가 절전빌드 적용 전 `USB리셋` → 적용 후 `★패닉/예외`,
 *  실패 누적 29 → 39. 원인(추정: 공유 I2C ↔ light sleep 복귀) 확정 전까지 끈다.
 *  ①MTD/SED(가장 큰 절감)와 DFS 는 유지된다. 원인 해결되면 0 으로 되돌릴 것. */
/* ★★★2026-08-16 (3) **0 으로 되돌린다** — 아래 ①②의 진범이 따로 밝혀졌다.
 *
 *  코어덤프로 확인한 패닉의 실제 원인은 light sleep 이 아니라 **CC1101 SPI** 였다:
 *      #1 uninstall_priv_desc  spi_master.c:1181   #2 setup_priv_desc  :1260
 *    DMA + 스택버퍼 → 전송마다 heap 바운스 할당 → 실패 시 IDF 가 초기화 안 된
 *    포인터로 memcpy → 크래시. cc1101.c 를 SPI_DMA_DISABLED 로 고쳐 경로가 소멸했다.
 *  ②의 버튼 연타도 light sleep 을 끈 빌드에서 그대로 재현됐고(SEL 70건),
 *    이후 하드웨어 조치로 해결됐다 — 역시 light sleep 과 무관했다.
 *  → 아래 기록은 **오진의 이력**으로 남겨둔다. 판단 근거였던 두 증상 모두
 *    다른 원인으로 설명·해결됐으므로 light sleep 을 다시 켠다.
 *
 * ── 이하 2026-08-16 당시의 (틀린) 판단 기록 ─────────────────────────────
 * ★2026-08-16 다시 1 — light sleep 이 **두 가지 오작동의 공통분모**로 확인됐다.
 *
 *  ① 패닉: 절전빌드 후 배터리에서 반복(부팅진단 리셋사유 `USB리셋`→`★패닉/예외`,
 *          실패누적 29→39). 전압은 5004mV 정상이라 브라운아웃이 아니다.
 *  ② 버튼 연타: SEL 이 같은 1초에 4번씩 찍힌다(방전로그 실측). 디바운스는
 *          esp_timer 기반(20ms)이라 tick 변경과 무관하므로, **PCF 읽기 값 자체가
 *          토글**되고 있다는 뜻이다 → 사용자 신고 "채널 변경 버튼이 안 된다".
 *
 *  둘 다 H2 가 **OLED 와 PCF 로 I2C 를 공유**하는 구조에서 light sleep 복귀와
 *  충돌하는 것으로 보인다(C6 는 PCF 를 LP_I2C/LP코어로 분리해 해당 없음).
 *  ★backtrace 는 아직 못 봤다 — coredump 파티션은 넣어뒀으니 다음 패닉 때 잡힌다.
 *  ①(FTD→MTD/SED, 최대 절감)과 DFS 는 그대로 유지된다. */
#define BOARD_DISABLE_LIGHT_SLEEP 0

/* ★★★2026-08-17 방전 로그 **끔** — 페어링 heap 확보(사용자 지시).
 *  링버퍼 128건 + NVS 코드가 통째로 빠진다.
 *  ~~H2 는 배터리로 살아남지 못해(USB 빼면 꺼짐) 방전 기록의 실익이 없다.~~
 *
 * ★★★2026-08-23 **다시 켠다(1)** — 위 근거가 **사실과 달랐다**.
 *  실측: 사용자가 장시간 방치 후 연결했을 때 **uptime 341,637초(94.9시간=3.95일)**,
 *  배터리 3,423mV(4%)까지 방전. H2 는 배터리로 **나흘 가까이 산다**.
 *  그런데 batlog 가 꺼져 있어 "어떻게 줄었는지" 를 통째로 못 봤다 —
 *  4일 뒤에 "지금 4%" 라는 한 점만 남는다.
 *  heap 사정도 달라졌다: 2026-08-22 태스크 통합(time_update·time_persist 를
 *  메인 루프로 흡수)으로 **5KB 이상 회수**했다(C6 실측 free +7.4KB).
 *  비용은 128건 × 11B = 1,408B(.bss). 재페어링 여유를 보고 줄이려면 64 로.
 *  ※샘플이 11B 인 이유: 2026-08-17 t_s 를 uint32 로 넓혔다(18.2시간 포화 해소). */
#define BOARD_BATLOG_ENABLE 1
#define BOARD_BATLOG_MAX 128 // × 11B = 1,408B

/* ════════════════════════════════════════════════════════
   GPIO 직결 센서 (light sleep wake 가능)
   ════════════════════════════════════════════════════════ */
#define BOARD_PIN_CHG_STAT                                                     \
  12 // GP12 — TP4054 CHRG(녹색 LED 전용, GPIO 미노출)는 못 읽지만, GP12 를
     //   VBUS(5V) 분압(active-HIGH)으로 재사용해 USB 연결 감지(A+B, 아래 매크로).
#define BOARD_PIN_VIBE 2 // GP2 VS1 진동 스위치 (비-strapping 가용핀, light-sleep wake)

/* 배터리/충전 — SuperMini 온보드 충전회로 (스키매틱 doc/esp32/SuperMini/
 *   esp32-h2-superMini2.png 확인):
 *   • 충전 IC : TP4054 (SOT23-5, 단셀 Li-ion 리니어 차저)
 *   • 충전전류: R_PROG=10kΩ → I ≈ 1000/10 = 100 mA
 *   • 파워패스: BAT → Schottky(PD1) → VCC.  LDO: ME6217C33 (3.3V 출력)
 *   • 충전 status(CHRG)는 녹색 LED 전용 → GPIO 로 안 빠짐(XIAO NCHG 와 동일
 * 이슈) ⚠ BATT_MAH 는 BAT 패드에 직결하는 실제 셀 용량으로 맞출 것(아래는
 * 대표값). 완충시간 = (BATT_MAH/CHG_MA) h × 1.25. 예: 400mAh → 4.0h×1.25
 * = 5.0h. */
#define BOARD_BATT_MAH                                                         \
  400                    // 대표값 — 실제 셀로 조정(100mA 충전이라 소형 셀 적합)
#define BOARD_CHG_MA 100 // TP4054, R_PROG=10kΩ → ~100mA

/* 충전 감지(A+B) — VBUS 분압(active-HIGH) + BAT 분압 ADC(실측 %) */
#define BOARD_CHG_STAT_ACTIVE_HIGH 1 // CHG_STAT(GP12) ← VBUS(5V) 분압
/* ★★2026-08-15 분압 100k/150k → **3.0V** (설계값 복구 확인).
 *   ESP32-H2 의 VIH 는 약 2.475V 라 3.0V 면 여유가 충분하다.
 *   ※한때 100k/100k(=2.5V)로 잘못 실장돼 있었고, 그때는 어느 쪽도 못 썼다:
 *       내부 풀다운 ON  → 100k∥45k=31k → 1.18V → 항상 LOW(USB 영영 미감지)
 *       내부 풀다운 OFF → 2.5V, 문턱 바로 위 → 불안정. 콘솔 REPL 이 스핀해
 *                         somfy_app 을 굶기고 폭주했다(task_wdt: CPU 0 = console).
 *   → 3.0V 로 복구됐으므로 아래 EXT_PULLDOWN 을 켠다(하단 150k 가 이미 풀다운이라
 *     내부 풀다운을 병렬로 물리면 3.0V→1.29V 로 눌려 USB 를 LOW 로 오독한다). */
/* ★★★2026-08-15 **보류(0)** — 배선을 설계값(3.0V)으로 복구한 뒤에도 이 값을 켜면
 *   H2 가 죽는다. 원인은 분압이 아니라 **공유 I2C 버스 물림**이다:
 *     task_wdt 덤프 = somfy_app 굶음 / 현재 실행 = btn_handler,
 *     레지스터 A5=0x600c5090(주변장치) → IDF i2c 의
 *      (**타임아웃 없음**) 안에서 스핀.
 *     같은 시간에 CHIP 은 정상(CASE 수립·구독 협상 완료) = 시스템은 살아 있다.
 *   EXT_PULLDOWN 을 켜면 USB 가 감지되어 충전 애니메이션 등 OLED 트래픽이 늘고,
 *   그 잠재 결함이 드러난다. trylock 수정은 **뮤텍스 대기**만 막을 뿐
 *   드라이버가 **락을 쥔 채 스핀**하는 건 못 막는다.
 *   → 공유 I2C 물림을 먼저 고친 뒤 켤 것. 지금은 USB 미감지를 감수하고 안정 우선.
 */
/* ★★★2026-08-16 **켠다(1)** — 멀티미터 실측 근거 + 사용자 결정.
 *   USB 연결 시 CHG_STAT = **0.69V** 로 측정됐다(설계상 5V×150/250 = 3.0V 여야 함).
 *   역산하면 내부 풀다운이 ~18kΩ 로 붙어 하단 150k 를 짓누르고 있다
 *     5 × (150k∥18k)/(100k + 150k∥18k) = 0.69V
 *   → 항상 LOW → USB 를 영영 못 잡는다. 하단 150k 가 이미 풀다운 역할을 하므로
 *     내부 풀다운은 해로울 뿐이다. 끄면 3.0V 로 올라가 정상 판정된다.
 *
 *   ※위 2026-08-15 의 위험 기록(켜면 공유 I2C 가 물려 somfy_app 이 굶음)은
 *     **여전히 유효하다**. 다만 그 뒤로 조건이 바뀌었다:
 *       · CC1101 SPI 가 DMA→비DMA (전송마다 나던 heap 할당이 사라짐, cc1101.c 주석)
 *       · 버튼 하드웨어 정상화
 *       · Thread FTD→MTD/SED (RAM·트래픽 감소)
 *     재발(task_wdt: somfy_app 굶음 / 현재 실행 btn_handler)하면 **즉시 0 으로 되돌릴 것**. */
#define BOARD_CHG_STAT_EXT_PULLDOWN 1
#define BOARD_HAS_BAT_ADC 1
/* 구 기판은 분압이 오배선이었다: BAT_ADC핀(GP3)=VBUS, CHG_STAT핀(GP12)=BAT.
 *   GP12 가 ADC 불가핀이라 잔량 % 측정 불가 → "USB/BAT/LOW" 상태 표시로 동작(=1).
 * ★2026-08-15: BAT_ADC/CHG_STAT 회로를 **정상 배선으로 수정**(GP3=BAT 분압 → ADC
 *   실측 %, GP12=VBUS 분압 → USB 감지) → **0 으로 전환**.
 *   =1 경로(USB/BAT/LOW 텍스트)는 somfy_app·oled_ui 의 #if 로 그대로 보존(삭제 금지).
 *   ※되돌리기: 배선이 옛 상태인 기판을 쓰면 다시 1 로. */
/* ★★★2026-08-15 처음 0 으로 켰을 때 H2 가 죽었다(task_wdt 23회: somfy_app + IDLE,
 *   로그 폭주, somfy_app=7 부팅 미완료). 원인은 배선이 아니라 **락**이었다 —
 *   공유 I2C 경로 `PCF_RD_SHARED()` 가 `oled_ui_i2c_lock()`(portMAX_DELAY) 을
 *   **버튼 태스크(prio 10)에서 10ms 마다** 잡고 있었고, 같은 뮤텍스를 쥔 쪽이 IDF I2C
 *   NACK 무한 스핀에 걸리면 prio 10 이 영원히 막혀 prio 4/0 이 굶는다.
 *   ADC 읽기(=이 뮤텍스의 두 번째 경쟁자)를 켜자 그 경로가 드러난 것이다.
 *   → button_handler.c 의 해당 락을 **유한 대기(trylock 50ms) + 실패 시 이번 폴
 *     건너뜀** 으로 바꾼 뒤 0 으로 복귀. (그쪽 주석 참조) */
#define BOARD_BAT_SWAPPED 0
#define BOARD_PIN_BAT_ADC 3   // GP3 (ADC1) ← BAT 분압
/* ★2026-08-15 저항 실장이 한때 CHG_STAT 과 서로 바뀌어 있었다(150k 가 BAT_ADC 하단에).
 *   그 상태에선 BOT=150 으로 맞춰야 했고, 안 고치면 전압을 1.2배 과대 계산했다
 *   (vadc 2085mV → 잘못 4170mV / 올바르게 3475mV). **사용자가 원래 회로로 복구**하여
 *   설계값 100k/100k(×2) 로 되돌린다. */
/* ★★★2026-08-16 BOT 100 → 150. **멀티미터 실측으로 확정**했다.
 *    USB 분리(VBUS=0.02V) 상태에서  GP3 = 2.500V,  배터리 = 4.187V
 *    → 분압비 2.500/4.187 = 0.597 ≈ 150/(100+150)  즉 상단 100k / 하단 150k
 *  USB 를 빼도 GP3 가 2.5V 를 유지하므로 **GP3 는 배터리를 본다**(VBUS 아님).
 *  → 이전에 "GP3 가 VBUS 를 본다"고 판단해 BAT_SWAPPED 를 의심했던 것은 **오진**이었다.
 *    2516mV 가 우연히 VBUS/2(2.5V)와 겹쳐 그렇게 보였을 뿐이다.
 *  검산: 2516mV × 250/150 = 4193mV  vs 실측 4187mV (6mV 차) — 일치. */
#define BOARD_BAT_DIV_TOP 100 // 상단(BAT→노드)
#define BOARD_BAT_DIV_BOT 150 // 하단(노드→GND) — 실측 분압비 0.597 로 확정

/* ── 로터리 A/B 배선이 뒤바뀐 기판 (2026-08-17 사용자 확인) ──────────────
 *  이 시제품은 엔코더 A/B 가 반대로 물려 있어 회전 방향이 뒤집혔다.
 *  배선을 고치는 대신 소프트에서 비트 위치를 맞바꾼다(P0=B, P1=A).
 *  ※H2 는 LP 코어가 없어 lp_core/pcf_lp_config.h 와의 정합성 문제는 없다. */
#define BOARD_ROT_AB_SWAP      1
