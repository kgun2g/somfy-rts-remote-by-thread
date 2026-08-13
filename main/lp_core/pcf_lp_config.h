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

/* 로터리 A/B 비트 위치 — button_handler.h 의 PCF8574_BIT_ROT_A/B 와 동일해야 한다. */
#define LP_BIT_ROT_A  0
#define LP_BIT_ROT_B  1
