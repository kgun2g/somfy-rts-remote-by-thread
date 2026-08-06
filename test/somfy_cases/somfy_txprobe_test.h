/*
 * somfy_txprobe_test.h — TX 자가진단(자기 GD0 출력 캡처/검증).
 *  부팅 시 실제 Somfy PROG 프레임을 RMT/CC1101 로 송신하면서, 동시에
 *  자기 IO8(GD0) 핀 전이를 µs 단위로 캡처해 RMT 출력이 표준 Somfy RTS
 *  파형(HW싱크 ~2416µs, 심볼 ~640µs)을 정확히 내는지 검증한다.
 *  → 디지털 OK 면 의심은 RF/PA/안테나, 디지털 BAD 면 RMT/프레임 버그.
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void somfy_txprobe_test_run(void);   /* 1회 실행 후 결과 출력하고 대기 */
#ifdef __cplusplus
}
#endif
