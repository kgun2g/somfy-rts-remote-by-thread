#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "cc1101.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Somfy RTS 커맨드 ─────────────────────────
 *  enum 값의 하위 4비트 = wire frame[1] 의 cmd nibble.
 *  Tilt 는 cmd nibble 0xB 를 공유하고 방향은 byte 8/9 로 구분(정품 동일).
 *  Tilt UP/DOWN 의 상위 nibble(1/2)은 enum 식별자일 뿐 wire 송신값과 무관 —
 *  _build_frame 에서 (cmd & 0x0F) 마스킹 후 byte 1 high nibble 로 사용된다. */
typedef enum {
    SOMFY_CMD_MY        = 0x01,  // 정지 / My 포지션
    SOMFY_CMD_UP        = 0x02,  // 올리기
    SOMFY_CMD_MY_UP     = 0x03,  // My + Up
    SOMFY_CMD_DOWN      = 0x04,  // 내리기
    SOMFY_CMD_MY_DOWN   = 0x05,  // My + Down
    SOMFY_CMD_UP_DOWN   = 0x06,  // Up + Down
    SOMFY_CMD_PROG      = 0x08,  // 프로그램
    SOMFY_CMD_SUN_FLAG  = 0x09,
    SOMFY_CMD_FLAG      = 0x0A,
    SOMFY_CMD_TILT_UP   = 0x1B,  // 틸트 업   (cmd nibble=0xB, byte8=0x30, b9 base=0x20)
    SOMFY_CMD_TILT_DOWN = 0x2B,  // 틸트 다운 (cmd nibble=0xB, byte8=0x38, b9 base=0x10)
} somfy_command_t;

/* ─── Somfy RTS 타이밍 (μs) ──────────────────
   표준 Somfy RTS 프로토콜 타이밍
   Symbol = 640μs (Manchester half-bit)
─────────────────────────────────────────────── */
/* ★ 타이밍 — 하이브리드 (정품 SDR 실측 + ESPSomfy 표준값).
 *    HW sync : 2560µs HIGH + 2560µs LOW (= 4 × SYMBOL, ESPSomfy 표준; 정품 실측 ~2520)
 *    SW sync : 4850µs HIGH + 640µs LOW (ESPSomfy 표준; LOW = 1 SYMBOL start-bit)
 *    SYMBOL  : 640µs (Manchester chip)
 *    Inter-frame gap : 4000µs LOW (정품 실측 ~4080; ESPSomfy 의 0 으로 두면 한국 모터 무응답)
 *  HW sync ON/OFF 와 SW sync ON 만 ESPSomfy 표준값으로 정렬(2026-05-25). */
/* ★ 시추오4ch리모컨1 hold 캡처(SDR#) 실측값으로 정렬(2026-07-03).
 *  이전 값(604/2416/4550/4000)에서 정품 hold 캡처 측정치로 갱신:
 *   bitChip 632 · hwSync 2520 · swSyncHi 4752 · interFrame 5448.
 *  ※ 모터는 정품(632) 파형을 그대로 디코드하므로 정품값이 오히려 안전.
 *    만약 이 변경 후 블라인드 무응답이면 timing 만 이전 값으로 되돌릴 것. */
#define SOMFY_T_SYMBOL      632     // Manchester chip (μs) — 정품 hold 실측(bitChip 632)
#define SOMFY_T_HWSYNC_ON   2520    // HW sync HIGH (μs) — 정품 hold 실측(hwSync 2520)
#define SOMFY_T_HWSYNC_OFF  2520    // HW sync LOW  (μs) — 정품 hold 실측
#define SOMFY_T_SWSYNC_ON   4752    // SW sync HIGH (μs) — 정품 hold 실측(swSyncHi 4752)
#define SOMFY_T_SWSYNC_OFF  632     // SW sync LOW  (μs) — 1 SYMBOL start-bit
#define SOMFY_T_INTER_FRAME 5448    // inter-frame gap (μs) — 정품 hold 실측(interFrame 5448, <10000 이라
                                    //  burst 가 한 패킷으로 묶여 디코더 그룹핑 유지).
/* ★ Wake-up 펄스 — ESPSomfy-RTS 가 실제 Telis 리모컨 측정으로 검증한 값.
 *  표준 문서의 9415+89565 는 실측과 다르다(실제 wake 뒤 긴 silence 없음). */
#define SOMFY_T_WAKE_HI     10920       // (preamble 사용으로 미사용)
#define SOMFY_T_WAKE_LO     7357        // (미사용)
#define SOMFY_T_PRE_HI      90          // 정품 preamble 짧은 HIGH 펄스(μs) — LOW 1700/3744/5460/5688 와 짝

/* 반복 횟수 — ★ 정품 짧은 누름 burst(SDR 실측 ~0.5초)에 맞춤.
 *  wake 프레임(~217ms) + repeat 2회(~148ms×2) ≈ 0.5초.
 *  Somfy 모터는 frame 1회만 받아도 인식하므로 3 frames 로 충분.
 *  (이전 7회는 ~1초로 과길어 정품과 burst 길이가 달랐음.) */
#define SOMFY_REPEAT_COUNT  3           // 1 wake + 2 repeats = 3 frames ≈ 0.5초

/* ─── 블라인드 구성 ──────────────────────────── */
typedef struct {
    uint32_t address;       // 24비트 원격 주소 (고유값)
    uint16_t rolling_code;  // 롤링 코드 (전송마다 증가)
    float    freq_mhz;      // 전용 주파수 (447.20 ~ 447.79 MHz)
    char     name[16];      // 블라인드 이름
    bool     active;        // 활성 여부
} somfy_blind_t;

/* ─── Somfy RTS 컨텍스트 ──────────────────────── */
typedef struct {
    cc1101_t *cc1101;
} somfy_rts_t;

/* ─── hold 반복 송신 중단 ──────────────────────────
 *  UP/DOWN 을 길게 눌러 반복 송신하는 도중 버튼을 떼면 즉시 멈추기 위함.
 *  somfy_rts_send 호출 직전 somfy_rts_abortable 로 이 송신의 중단 가능
 *  여부를 지정한다(일반 누름 burst=false → 끝까지 송신, hold 반복=true).
 *  중단하려면 somfy_rts_abort 를 true 로 둔다. */
extern volatile bool somfy_rts_abort;
extern volatile bool somfy_rts_abortable;
/* true 면 다음 somfy_rts_send 1회가 롤링코드를 증가시키지 않고 현재 코드를 재사용한다
 * (hold 반복 — 정품은 버튼 누르는 동안 같은 코드 반복). 송신 후 자동으로 false 로 리셋(one-shot). */
extern volatile bool somfy_rts_keep_rolling;

/* ─── API ────────────────────────────────────── */

/**
 * @brief Somfy RTS 초기화
 * @param ctx  Somfy 컨텍스트
 * @param dev  초기화된 CC1101 장치
 */
bool somfy_rts_init(somfy_rts_t *ctx, cc1101_t *dev);

/**
 * @brief Somfy RTS 커맨드 전송
 *        지정된 블라인드에 커맨드를 전송하고 rolling code를 증가시킵니다.
 *
 * @param ctx    Somfy 컨텍스트
 * @param blind  대상 블라인드 설정
 * @param cmd    전송할 커맨드
 * @param hold_ms 버튼 누름 시간 (ms), 0=단순 클릭
 *               0.1초~15초 = 100ms~15000ms
 */
void somfy_rts_send(somfy_rts_t *ctx, somfy_blind_t *blind, somfy_command_t cmd, uint32_t hold_ms);

/**
 * @brief Tilt 다단 step 송신 — 연속 TX 모드 유지로 끊어짐 최소화.
 *
 *  정품 tilt 리모컨의 7-detent 회전을 N 회 빠르게 누른 것과 등가:
 *   - cc1101_enter_tx_mode 는 burst 시작 시 1회만 (매 step 마다 안 함)
 *   - step 당 2 frames (wake + 1 repeat) ≈ 330 ms
 *   - 매 step 마다 rolling_code 증가 → 모터가 각 step 을 독립 detent 로 인식
 *   - 일반 송신(3 frames + cc1101 enter/idle) 대비 step 당 ~170ms 단축
 *
 * @param ctx        Somfy RTS 컨텍스트
 * @param blind      대상 블라인드
 * @param cmd        SOMFY_CMD_TILT_UP / SOMFY_CMD_TILT_DOWN (TILT 권장)
 * @param step_count 송신할 step 수 (1~7 권장, 내부적으로 1~10 clamp)
 */
void somfy_rts_send_steps(somfy_rts_t *ctx, somfy_blind_t *blind,
                           somfy_command_t cmd, uint8_t step_count);

/**
 * @brief 틸팅 전송 (휠 회전 1단계)
 *        UP 또는 DOWN 커맨드를 짧게(100ms) 반복 전송
 *
 * @param ctx   Somfy 컨텍스트
 * @param blind 대상 블라인드
 * @param up    true=틸트 업, false=틸트 다운
 */
void somfy_rts_tilt(somfy_rts_t *ctx, somfy_blind_t *blind, bool up);

/**
 * @brief 테스트 전용 — CC1101/RF 없이 7바이트 RTS 프레임만 생성.
 *        가상 테스트(온에어 불필요)에서 프레임 정합성 검증에 사용.
 * @param out      9바이트 출력 버퍼 (한국 베네치아 9바이트 frame)
 * @param cmd      커맨드
 * @param rolling  롤링 코드
 * @param address  24비트 주소
 */
void somfy_rts_test_build_frame(uint8_t out[10], somfy_command_t cmd,
                                uint16_t rolling, uint32_t address);

#ifdef __cplusplus
}
#endif
