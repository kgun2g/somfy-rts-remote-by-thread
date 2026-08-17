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
