/*
 * somfy_cwtest_test.h — CW(순수 carrier) 송신 테스트.
 *  OOK 변조 없이 carrier 만 연속 송신 → SDR 로 carrier 자체의 주파수
 *  안정성 확인. 번짐 원인이 crystal/FS chirp(하드웨어)인지 OOK 변조
 *  splatter(설정)인지 최종 판별.
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void somfy_cwtest_test_run(void);   /* 부팅 후 CW 송신 무한 루프 */
#ifdef __cplusplus
}
#endif
