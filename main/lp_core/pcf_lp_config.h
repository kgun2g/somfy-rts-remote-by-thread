/*
 * pcf_lp_config.h — LP 코어 프로그램과 HP 가 **함께 보는** 최소 설정
 * ═══════════════════════════════════════════════════════════════════
 * LP 코어 툴체인은 HP 쪽 헤더 체인(button_handler.h → FreeRTOS/esp_log …)을
 * 따라갈 수 없다. 그렇다고 값을 양쪽에 따로 적으면 **조용히 어긋난다** —
 * 실제로 LP 가 1바이트만 읽어 좌/우 버튼(PCF8575 P10/P11 = bit8/9)이 통째로
 * 사라졌다(2026-08-13 실사용 신고).
 *
 * ★그래서 이 파일 하나만 **양쪽이 같이 include** 하고, HP 쪽에서
 *   `_Static_assert(LP_PCF_NBYTES == PCF_NBYTES, ...)` 로 어긋남을 **빌드에서**
 *   잡는다. 런타임에 조용히 틀리는 것보다 컴파일이 깨지는 게 낫다.
 *
 * ※빌드 정의(-D)로 넘기는 방법은 못 쓴다: ulp_embed_binary 가 만드는 타깃은
 *   ExternalProject 라 target_compile_options 가 통하지 않는다
 *   ("target_compile_options called with non-compilable target type").
 */
#pragma once

/* PCF 확장기 read 폭(바이트).
 *   2 = PCF8575(16비트, 좌/우 버튼 있음) ← xiao-c6 기본
 *   1 = PCF8574(8비트)
 * button_handler.h 의 PCF_NBYTES 와 반드시 같아야 한다(위 static assert 가 검증). */
#define LP_PCF_NBYTES 2

/* 로터리 A/B 비트 위치 — **물리 배치 고정**(P0=A, P1=B).
 * ★2026-08-31 배선이 뒤바뀐 개체(BOARD_ROT_AB_SWAP=1, 예 COM8)라도 **여기는
 *   바꾸지 않는다.** LP 코어는 빌드 -D 를 받을 수 없어(위 머리말 참조) 이 값이
 *   전 보드 공통이기 때문이다. 스왑 보정은 HP 쪽에서 방향을 뒤집어 처리한다
 *   (button_handler.c 의 `#if BOARD_ROT_AB_SWAP  cw = !cw;`).
 *   A/B 교환 == 쿼드러처 delta 부호 반전이므로 결과는 동일하다. */
#define LP_BIT_ROT_A  0
#define LP_BIT_ROT_B  1

/* ★★2026-09-01 LP 폴 주기(us) — **LP·HP 가 함께 쓴다.**
 *  HP 는 이 값을 `ulp_lp_core_cfg_t.lp_timer_sleep_duration_us` 에 넣어
 *  LP 코어를 이 주기로 깨운다(1회 실행 후 halt). LP 는 같은 값으로 동작한다.
 *  두 곳에 따로 적으면 조용히 어긋나므로 — 이 헤더의 존재 이유대로 — 여기 하나만 둔다.
 *
 *  값 근거(2026-08-27 시뮬 sim/tools/power_lever_sim.py): 로터리 20디텐트 CW 를
 *  폴 주기별로 디코딩하면 5000us 까지는 전부 +20, 8000us 부터 빠른 회전에서
 *  손실이 난다. **버튼은 press_latch 가 모아주지만 로터리는 래치가 없어**
 *  로터리가 진짜 제약이다. 늘리지 말 것. */
#define LP_POLL_US  5000
