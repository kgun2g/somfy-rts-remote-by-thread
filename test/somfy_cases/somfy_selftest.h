/*
 * somfy_selftest.h — Somfy RTS 가상 테스트 하네스 (재사용).
 * Somfy RTS 블라인드 컨트롤러 (ESP32-C6) v3.5.
 *
 * 온에어/CC1101 없이 RTS 7바이트 프레임을 생성·검증하는 순수 가상
 * 테스트. 블라인드 1~5 각각에 대해 다음 케이스를 수행:
 *   - 신규 등록(register)   : PROG 길게
 *   - 상한 설정(upper limit): PROG → UP → MY
 *   - 하한 설정(lower limit): PROG → DOWN → MY
 *   - 복사(copy)            : 원격 식별자 복제 후 동일 주소 전송 검증
 *   - 위치 기억(my position): MY 길게
 *
 * 빌드 시 -DSOMFY_SELFTEST=1 일 때만 컴파일/실행 (test/build_test.ps1).
 * 표준 레벨 로깅(esp_log): 케이스별 PASS=ESP_LOGI, FAIL=ESP_LOGE,
 * 요약=ESP_LOGW.
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/* 전체 가상 테스트 실행. 반환: 실패 케이스 수(0=전부 통과). */
int somfy_selftest_run(void);

#ifdef __cplusplus
}
#endif
