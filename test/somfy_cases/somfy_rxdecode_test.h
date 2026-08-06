/*
 * somfy_rxdecode_test.h — CC1101 RX OOK 펄스 디코더.
 *  정품 Somfy RTS 리모컨의 복조 펄스 타이밍을 캡처해, 어느 "프로그램
 *  주파수" 에서 표준 RTS 파형(HW싱크 2416µs / 심볼 640µs)이 깨끗이
 *  잡히는지 확정한다(= TX 에 써야 할 진짜 값). 송신 안 함.
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void somfy_rxdecode_test_run(void);   /* 무한 루프(반환 안 함) */
#ifdef __cplusplus
}
#endif
