#pragma once
/*
 * CHIPProjectConfig.h — CHIP 컴파일타임 설정 override.
 *   sdkconfig 의 CONFIG_CHIP_PROJECT_CONFIG="main/CHIPProjectConfig.h" 로 연결되며,
 *   esp-matter chip 컴포넌트가 chip_project_config_include / chip_system_project_config_include
 *   양쪽으로 이 헤더를 include 한다(config/esp32/components/chip/CMakeLists.txt).
 *
 * ── H2 BLE 커미셔닝 PacketBuffer 풀 확대 ──────────────────────────────
 *   증상: SmartThings Matter 페어링이 PASE 단계에서 "39-104" 로 실패. 시리얼:
 *     E chip[CSL]: PacketBuffer: pool EMPTY
 *     E chip[SC]:  Failed to allocate status report message
 *     E chip[SC]:  Failed during PASE session setup: b      (b = NO_MEMORY)
 *   원인: 기본 풀(CHIP_SYSTEM_CONFIG_PACKETBUFFER_POOL_SIZE=15, SystemConfig.h)이
 *     BLE 전송계층(BTP)이 버퍼를 점유한 상태에서 PASE_Pake 응답을 만들 때 소진된다.
 *     CHIP task 우선순위를 올려도(우선순위 역전 아님) 동일 지점에서 발생 → 풀 절대 부족.
 *   대안 비교: CHIP_SYSTEM_CONFIG_POOL_USE_HEAP=y 는 H2 에서 Load access fault
 *     boot loop 를 유발(Kconfig 경고대로 "코드가 고정 풀 크기를 가정") → 사용 불가.
 *   해결: 고정 풀 크기를 키워 BTP+PASE 동시 수용.
 */
/* ★★★2026-08-17 18 → 22.
 *   18 로도 여전히 같은 지점에서 막혔다(실측: BLE 연결 3회 → 3회 모두
 *     E chip[CSL]: PacketBuffer: pool EMPTY  → err = b(NO_MEMORY) → 연결 종료,
 *     PASE 는 한 번도 시작 못 함. SmartThings 39-100).
 *   그때 free heap 은 4.6KB 로 여유가 있었다 → **heap 이 아니라 이 고정 풀** 이
 *   병목임이 확정. 풀은 BSS 라 늘리려면 다른 정적 메모리를 내줘야 한다.
 *   → map 실측으로 찾은 미사용 Matter 클러스터(color-control/level-control/
 *     on-off 등 12개, 약 5.8KB+)를 끄고 그 자리를 여기에 돌린다.
 *   22 × 1,583B ≈ 34.8KB (18개 대비 +6.3KB).
 *   ※그래도 부족하면 남는 길은 블라인드 채널을 3 → 2 로 줄이는 것뿐이다. */
#define CHIP_SYSTEM_CONFIG_PACKETBUFFER_POOL_SIZE 22

/* ── Device Instance Info: Vendor/Product Name ──────────────────────
 *   sdkconfig 의 CONFIG_EXAMPLE_DEVICE_INSTANCE_INFO_PROVIDER=y 가 이 두 상수를
 *   Basic Information 클러스터의 VendorName/ProductName 으로 노출한다. 기본값
 *   "TEST_VENDOR"/"TEST_PRODUCT" 를 override → 커미셔너(SmartThings/Apple/Google)가
 *   표시하는 기기 식별 이름이 바뀐다. (Matter 스펙상 각 최대 32자)
 *   ※ VID/PID(0xFFF1/0x8001)는 sdkconfig CONFIG_DEVICE_VENDOR_ID/PRODUCT_ID 에서 별도 관리.
 */
#define CHIP_DEVICE_CONFIG_DEVICE_VENDOR_NAME  "NLB"
#define CHIP_DEVICE_CONFIG_DEVICE_PRODUCT_NAME "Somfy RTS Thread"

/* ── ★★2026-08-25 Thread SSED (CSL) — 실험 ────────────────────────────────
 *  지금은 SED 로 **7초마다 부모에게 폴링**한다(CONFIG_ICD_SLOW_POLL_INTERVAL_MS).
 *  CSL(Coordinated Sampled Listening)은 부모와 시각을 맞춰 **정해진 순간에만**
 *  수신기를 켜므로 무선 대기 시간이 크게 준다.
 *
 *  ★이 매크로가 실제 스위치다. CHIP 소스
 *    (GenericThreadStackManagerImpl_OpenThread.hpp, SetPollingInterval)를 보면
 *        #if CHIP_DEVICE_CONFIG_THREAD_SSED
 *            otLinkSetCslPeriod(...)      ← CSL 주기 설정 = SSED 로 동작
 *        #else
 *            otLinkSetPollPeriod(...)     ← 지금 경로(SED 폴링)
 *        #endif
 *    즉 `CONFIG_OPENTHREAD_CSL_ENABLE=y` 만 켜면 **컴파일 지원만 들어가고
 *    동작은 안 바뀐다** — 둘 다 필요하다.
 *
 *  ★★위험: CSL 은 **부모(Thread 라우터)가 CSL 송신을 지원해야** 성립한다.
 *    SmartThings 허브가 지원하지 않으면 명령 수신이 늦거나 끊길 수 있다.
 *    → 적용 후 **SmartThings 명령 전달을 반드시 확인**할 것. 이상하면 이 매크로와
 *      CONFIG_OPENTHREAD_CSL_ENABLE 을 함께 되돌린다.
 *  ※CSL_ACCURACY(±50ppm)·CSL_UNCERTAIN(50 = 500us)은 이미 sdkconfig 에 있다. */
/* ★★★2026-08-25 **되돌림(0)** — 실기에서 통신이 깨졌다.
 *      E chip[DL]: SRP update error: timed out waiting on server response  (반복)
 *      Matter 메시지(chip[EM]) 3분간 **0건** (이전엔 수십 건)
 *      free heap 40,084 → 15,004 B (**-25KB**, CSL 타이밍 버퍼)
 *  SmartThings 허브(부모 라우터)가 CSL 송신을 지원하지 않는 것으로 보인다.
 *  위에 적어둔 위험이 그대로 실현됐다. heap 대가도 H2 에는 치명적이다.
 *  ※다시 시도하려면 **부모가 Thread 1.2 CSL 을 지원하는지 먼저 확인**할 것. */
#define CHIP_DEVICE_CONFIG_THREAD_SSED 0
