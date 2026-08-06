/*
 * somfy_rxbyte_test.h — 정품 Somfy RTS 리모컨의 7바이트 프레임 디코더.
 *  OOK 펄스 → Manchester → 56비트 → 7바이트 → 역난독화 → 의미 해석.
 *  우리 코드가 생성하는 프레임과 1바이트씩 비교 가능.
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void somfy_rxbyte_test_run(void);   /* 무한 루프 — 캡처/디코드/출력 반복 */
#ifdef __cplusplus
}
#endif
