#pragma once
#include "somfy_rts.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BLIND_MAX_COUNT 는 보드 프로파일(boards/<board>.h)에서 정의 — 미정의 시 board_select.h 가 기본 8(H2 는 esp32-h2.h 에서 3).
 * include 체인: somfy_rts.h → cc1101.h → boards/board_select.h → boards/<board>.h 로 여기서 보인다. */
#define BLIND_NVS_NAMESPACE "somfy_blinds"
#define BLIND_NVS_KEY       "blinds_cfg"

/* ── 4채널 = 1 블록(4채널 + 전용 ALL). 채널이 늘면 블록(=ALL)이 자동 증가 ──
 *  H2(3채널)=1블록·ALL 1개, C6(5채널)=2블록·ALL 2개, 9채널=3블록·ALL 3개.
 *  ALL 명령은 모든 블록의 ALL 을 차례로 송신(ALL1, ALL2, …). */
#define BLINDS_PER_BLOCK   4
#define BLIND_BLOCK_COUNT  ((BLIND_MAX_COUNT + BLINDS_PER_BLOCK - 1) / BLINDS_PER_BLOCK)
/* 선택 인덱스: 0..BLIND_MAX_COUNT-1 = 개별 채널, BLIND_MAX_COUNT = ALL(전체).
 * (구 하드코딩 5 를 일반화 — 채널 수가 늘어도 ALL 선택이 항상 마지막 인덱스) */
#define BLIND_SEL_ALL      BLIND_MAX_COUNT

/* ─── 블라인드 전체 설정 ──────────────────────── */
typedef struct {
    somfy_blind_t blinds[BLIND_MAX_COUNT];
    somfy_blind_t all_blocks[BLIND_BLOCK_COUNT];  // 블록별 ALL(주소 + 전용 rolling)
    uint8_t       count;       // 등록된 블라인드 수
    uint8_t       block_count; // ALL 블록 수 (= BLIND_BLOCK_COUNT)
    uint8_t       selected;    // 현재 선택 (0~N-1 개별, N(=BLIND_SEL_ALL)=ALL)
    uint32_t      cfg_tag;     // 설정 세대 태그(BLIND_CFG_TAG) — 불일치 시 롤링 재기준
} blind_manager_t;

/* ─── API ────────────────────────────────────── */

/**
 * @brief 블라인드 매니저 초기화 (NVS 로드)
 */
void blind_manager_init(blind_manager_t *mgr);

/**
 * @brief 전체 설정을 NVS에 저장
 */
void blind_manager_save(const blind_manager_t *mgr);

/**
 * @brief NVS에서 로드
 */
bool blind_manager_load(blind_manager_t *mgr);

/**
 * @brief 블라인드 추가
 * @param mgr    매니저
 * @param name   블라인드 이름 (최대 15자)
 * @param freq   주파수 (447.20 ~ 447.79 MHz)
 * @return 추가된 인덱스 (0..BLIND_MAX_COUNT-1), 실패 시 -1
 */
int blind_manager_add(blind_manager_t *mgr, const char *name, float freq);

/**
 * @brief 블라인드 삭제
 * @param mgr  매니저
 * @param idx  삭제할 인덱스
 */
void blind_manager_remove(blind_manager_t *mgr, uint8_t idx);

/**
 * @brief 블라인드 주파수 변경 (NVS 저장 포함)
 */
void blind_manager_set_freq(blind_manager_t *mgr, uint8_t idx, float freq_mhz);

/**
 * @brief 블라인드 선택 변경 (로컬)
 */
void blind_manager_select(blind_manager_t *mgr, uint8_t idx);

/**
 * @brief 선택된 블라인드 또는 ALL에 대해 포인터 배열 반환
 * @param mgr      매니저
 * @param out      출력 포인터 배열 (최대 BLIND_MAX_COUNT 개)
 * @param out_count 출력 개수
 */
void blind_manager_get_targets(blind_manager_t *mgr,
                                somfy_blind_t **out, uint8_t *out_count);

/**
 * @brief 롤링 코드 NVS 저장 (전송 후 즉시 호출)
 */
void blind_manager_save_rolling(blind_manager_t *mgr, uint8_t idx);

#if defined(SOMFY_SELFTEST) || defined(SOMFY_ONAIR_TEST)
/**
 * @brief [테스트 전용] NVS 없이 결정적 주소(eFuse 기반)로 매니저를 채운다.
 *        실제 _ensure_blinds(주소 산출 + ALL 블록 + active)만 수행하고
 *        NVS 저장/로드는 하지 않아 기기 상태를 건드리지 않는다.
 *        SOMFY_SELFTEST / SOMFY_ONAIR_TEST 빌드에서만 컴파일된다.
 */
void blind_manager_test_populate(blind_manager_t *mgr);
#endif

#ifdef __cplusplus
}
#endif
