/* boot_diag — 전원이 끊겨도 남는 부팅 단계 기록 (2026-08-11)
 * ───────────────────────────────────────────────────────────────────────────
 * 왜 필요한가:
 *   "USB 없이 배터리만 연결하면 부팅 중 멈춘다"를 진단해야 하는데, USB 를 꽂는
 *   순간 증상이 사라진다(전원이 바뀜). 시리얼 로그를 볼 수 없으므로 **다음 부팅에
 *   읽을 수 있는 흔적**을 남겨야 한다.
 *
 * 왜 RTC 메모리가 아니라 NVS 인가:
 *   RTC RAM 은 리셋은 견디지만 **전원 차단은 못 견딘다**. 배터리를 빼서 재시도하는
 *   순간 지워진다. NVS(플래시)만 살아남는다.
 *
 * ★왜 단계를 둘로 나누는가 (stage / stage2):
 *   `xTaskCreate(somfy_app_run, ..., prio 4)` 순간 somfy_app 이 app_main(prio 1)을
 *   **즉시 선점**한다. 이후 두 경로가 번갈아 진행되므로 단일 카운터로는 "어느 쪽이
 *   멈췄는지"를 구분할 수 없다(단조증가 규칙 때문에 늦은 쪽 기록이 삼켜진다).
 *   → app_main 진행도(stage)와 somfy_app 진행도(stage2)를 **따로** 센다.
 *
 * 쓰는 법:
 *   부팅 초반에 boot_diag_begin() 1회 → 직전/보관된 실패 부팅이 로그로 찍힌다.
 *   app_main 은 boot_diag_stage(), somfy_app 은 boot_diag_stage2() 를 부른다.
 *   메인 루프가 30초 생존하면 boot_diag_stage2(BOOT_S2_RUNNING) 로 마무리.
 *
 * 조회:
 *   시리얼 콘솔에서 `bd` → 언제든 다시 출력(부팅 직후 몇 초짜리 창을 놓쳐도 됨).
 *   `bd clear` → 보관된 실패 기록 삭제.
 *
 * 플래시 마모:
 *   부팅당 쓰기 ~17회. 진단용이라 허용치이며, 끝나면 BOOT_DIAG_ENABLE 0 으로
 *   꺼서 코드를 지우지 않고 비활성화할 수 있다.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BOOT_DIAG_ENABLE
#define BOOT_DIAG_ENABLE 1
#endif

/* app_main 경로 진행도. 값이 커질수록 뒤 단계(뒤에만 추가할 것). */
typedef enum {
    BOOT_S1_NONE          = 0,
    BOOT_S1_APP_MAIN      = 1,   /* app_main 진입 / NVS init 완료 */
    BOOT_S1_ENDPOINTS     = 2,   /* Matter 엔드포인트 생성 완료 */
    BOOT_S1_BLIND_MGR     = 3,   /* blind_manager_init 완료 */
    BOOT_S1_MATTER_START  = 4,   /* esp_matter::start() 호출 직전 */
    BOOT_S1_MATTER_OK     = 5,   /* esp_matter::start() 반환 */
    BOOT_S1_APP_TASK      = 6,   /* somfy_app 태스크 생성 직후 */
    BOOT_S1_CONSOLE_ENTER = 7,   /* ★콘솔 등록/init 섹션 진입 */
    BOOT_S1_CONSOLE_CMDS  = 8,   /* 명령 등록 완료(console::init 직전) */
    BOOT_S1_DONE          = 9,   /* console::init 반환 = app_main 끝 */
} boot_s1_t;

/* somfy_app 경로 진행도. */
typedef enum {
    BOOT_S2_NONE          = 0,
    BOOT_S2_RUN_ENTRY     = 1,   /* somfy_app_run 진입 */
    BOOT_S2_RTC_INIT      = 2,   /* 빌드시각 RTC 초기화 완료 */
    BOOT_S2_OLED_ENTER    = 3,   /* ★oled_ui_init 호출 직전 */
    BOOT_S2_OLED_DONE     = 4,   /* oled_ui_init 반환 */
    BOOT_S2_BTN_DONE      = 5,   /* btn_handler_init 반환 */
    BOOT_S2_RF_DONE       = 6,   /* RF 큐 생성 완료 */
    BOOT_S2_MAIN_LOOP     = 7,   /* 메인 루프 진입 */
    BOOT_S2_RUNNING       = 8,   /* 메인 루프 30초 생존 = 부팅 성공 */
} boot_s2_t;

void boot_diag_begin(void);
void boot_diag_stage(boot_s1_t stage);    /* app_main 경로 */
void boot_diag_stage2(boot_s2_t stage);   /* somfy_app 경로 */

/* ★현재 단계 **안쪽**의 세부 진행도(0~255). oled_ui_init 처럼 한 함수가 길고
 *  그 안에서 멈출 때, 어느 줄까지 갔는지 좁히는 용도. 값이 커질 때만 기록한다. */
void boot_diag_sub(uint8_t step);

/* ★직전/보관된 실패 부팅 요약을 **다시** 출력한다.
 *  begin() 은 부팅 ~100ms 안에 실행되는데 그 구간 로그는 USB-JTAG 이 버린다
 *  (실제로 첫 시험에서 이 줄만 통째로 유실됐다). 메인 루프와 `bd` 명령에서 재출력. */
void boot_diag_log_prev(void);

/* 보관된 '실패한 부팅' 기록을 지운다(원인 해결 후 정리용). */
void boot_diag_clear_fail(void);

/* 배터리 전압을 기록에 남긴다. 0 이하면 무시.
 *  · 첫 값은 bat_mv 로 보존(부팅 초기 전압)
 *  · 이후 값은 **최저치(bat_min_mv)** 를 추적한다 — 부하가 걸릴 때 얼마나 처지는지가
 *    "전원 붕괴" 판정의 핵심 증거다. 플래시 마모를 줄이려 50mV 이상 떨어질 때만 쓴다. */
void boot_diag_set_bat_mv(int mv);

const char *boot_diag_s1_name(boot_s1_t s);
const char *boot_diag_s2_name(boot_s2_t s);

#ifdef __cplusplus
}
#endif
