/*
 * somfy_stress_test.h — 실제 CC1101 온에어 스트레스 테스트.
 *  라운드마다 송신 횟수를 늘려 가며 무한 반복(중단 지시 전까지).
 *  공유 인스턴스(g_cc1101/g_somfy/g_mgr/g_rf_ready) 사용, NVS 불변.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 실제 CC1101 로 무한 증가 스트레스 송신. 반환하지 않는다.
 *        라운드 r = r*STRESS_STEP 회 실제 RF 송신(블라인드/주파수/커맨드
 *        회전 + ALL 순차). 누적 PASS/FAIL/경과시간을 표준 레벨 로그로 출력.
 */
void somfy_stress_test_run(void);

#ifdef __cplusplus
}
#endif
