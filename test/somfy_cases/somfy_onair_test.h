/*
 * somfy_onair_test.h — 실제 CC1101 온에어 RF 테스트.
 *  app_main.cpp 가 Phase2 에서 초기화한 공유 인스턴스
 *  (g_cc1101/g_somfy/g_mgr/g_rf_ready) 를 그대로 사용해 실제 신호를 송신한다.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 실제 CC1101 로 온에어 RF 송신 테스트 1회 수행.
 *        - CC1101 SPI 통신/칩 확인
 *        - 여러 주파수로 변경하며 레지스터 라이트백·실제 캐리어 송신 검증
 *        - 가상 버튼(UP/DOWN/MY/PROG) × 블라인드 1~5 실제 변조 송신
 *        - ALL 선택 시 5개 블라인드 동시(순차) 제어 + PROG 차단 정책 검증
 * @return 0=전부 PASS, >0=FAIL 케이스 수, <0=CC1101 미준비(하드웨어 없음)
 */
int somfy_onair_test_run(void);

#ifdef __cplusplus
}
#endif
