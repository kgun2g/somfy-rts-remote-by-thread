/*
 * somfy_txdecode_test.h — 우리 TX 송신을 GD0 핀에서 직접 캡처해 7바이트로
 *  디코드. 정품 rxbyte 결과와 byte-level 비교용. CC1101 RX 불사용 — RMT
 *  가 직접 구동하는 GD0 의 raw 펄스를 캡처.
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void somfy_txdecode_test_run(void);   /* 부팅 직후 PROG 송신 → 디코드. 무한 루프. */
#ifdef __cplusplus
}
#endif
