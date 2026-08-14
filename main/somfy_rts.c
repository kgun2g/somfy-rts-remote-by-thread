#include "somfy_rts.h"
#include <inttypes.h>
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include <string.h>

static const char *TAG = "SOMFY_RTS";

/* ─── RMT 채널 핸들 ──────────────────────────── */
static rmt_channel_handle_t  s_tx_chan   = NULL;
static rmt_encoder_handle_t  s_copy_enc  = NULL;

/* ─── RMT 최대 심볼 수 계산 ──────────────────
   Wake frame:    1(wake) + 12(HW) + 1(SW) + 80(data) + 1(gap) = 95 심볼
   Repeat frame:  6(HW) + 1(SW) + 80(data) + 1(gap) = 88 심볼
   여유 포함 256.
────────────────────────────────────────────── */
#define MAX_RMT_SYMBOLS   256

static rmt_symbol_word_t s_rmt_buf[MAX_RMT_SYMBOLS];

/* ─── 헬퍼: RMT 심볼 추가 ───────────────────── */
static inline int _add_symbol(rmt_symbol_word_t *buf, int idx,
                               uint32_t dur0_us, uint8_t lvl0,
                               uint32_t dur1_us, uint8_t lvl1)
{
    buf[idx].level0    = lvl0;
    buf[idx].duration0 = dur0_us;
    buf[idx].level1    = lvl1;
    buf[idx].duration1 = dur1_us;
    return idx + 1;
}

/* cmd → byte 8/9 매핑 (한국 베네치아 10바이트 frame — 정품 SDR 디코드 일치).
 *  Tilt UP/DOWN 은 cmd nibble 0xB(Tilt) 공유, byte 8 로 방향 구분:
 *    TILT_UP   : b[8]=0x30, b[9] base=0x20  (정품 wire ac→1E…84 30 2D)
 *    TILT_DOWN : b[8]=0x38, b[9] base=0x10  (정품 wire ae→12…84 38 16) */
static uint8_t _byte8_for_cmd(somfy_command_t cmd)
{
    switch (cmd) {
        case SOMFY_CMD_MY:        return 0x00;
        case SOMFY_CMD_UP:        return 0x20;
        case SOMFY_CMD_DOWN:      return 0x2C;
        case SOMFY_CMD_PROG:      return 0x00;
        case SOMFY_CMD_TILT_UP:   return 0x30;
        case SOMFY_CMD_TILT_DOWN: return 0x38;
        case SOMFY_CMD_UP_DOWN:   return 0x6C;  /* ★동시작동(limit/등록) 정품 실측(h2_15) */
        case SOMFY_CMD_MY_UP:     return 0x40;
        case SOMFY_CMD_MY_DOWN:   return 0x40;
        default:                  return 0x00;
    }
}
static uint8_t _byte9_base_for_cmd(somfy_command_t cmd)
{
    switch (cmd) {
        case SOMFY_CMD_UP:        return 0x00;
        case SOMFY_CMD_DOWN:      return 0x80;
        case SOMFY_CMD_MY:        return 0x10;
        case SOMFY_CMD_PROG:      return 0x10;
        case SOMFY_CMD_TILT_UP:   return 0x20;
        case SOMFY_CMD_TILT_DOWN: return 0x10;
        default:                  return 0x10;
    }
}

/* 80비트 프레임 체크섬 (ESPSomfy-RTS calc80Checksum 과 동일).
 *  byte9 하위 4비트 = byte7/8/9 상위·하위 니블 XOR(byte9 하위 제외). */
static uint8_t _calc80_checksum(uint8_t b7, uint8_t b8, uint8_t b9)
{
    uint8_t cs = ((b7 & 0xF0) >> 4) ^ ((b8 & 0xF0) >> 4);
    cs ^= ((b9 & 0xF0) >> 4);
    cs ^= (b7 & 0x0F);
    cs ^= (b8 & 0x0F);
    return cs & 0x0F;
}

/* rolling code + cmd → byte0(key) — 한국 정품 리모컨1 IQ 디코드 검증값.
 *  공식: key = 0xA0 | ((2*rolling + offset[cmd]) & 0x0F)
 *  cmd 별 offset (150+ frame 분석 — 6 버튼 IQ rtl_433 디코드):
 *    MY(1)=5, UP(2)=8, DOWN(4)=7, PROG(8)=0, TILT(B)=0
 *  cmd nibble 별로 다른 offset 을 두면 같은 rolling 도 cmd 마다 다른 key 가
 *  나와 정품 시퀀스를 완전 모사한다(rtl_433 의 seed 출력까지 정품 동일).
 *  ★ key 는 scrambler seed 일 뿐 모터가 검증하지 않음 — 정품 모사는 SDR
 *    옵저버 관점의 완성도 목적. ESPSomfy 식 단순 공식(rolling & 0xF)도
 *    동등하게 작동했음. */
static uint8_t _key_offset_for_cmd(somfy_command_t cmd)
{
    switch (cmd) {
        case SOMFY_CMD_MY:        return 5;
        case SOMFY_CMD_UP:        return 8;
        case SOMFY_CMD_DOWN:      return 7;
        case SOMFY_CMD_PROG:      return 0;
        case SOMFY_CMD_TILT_UP:   return 0;
        case SOMFY_CMD_TILT_DOWN: return 0;
        default:                  return 0;  /* MY_UP/MY_DOWN/UP_DOWN/SUN/FLAG 미측정 */
    }
}
static uint8_t _key_for_rolling(uint16_t rolling, somfy_command_t cmd)
{
    uint8_t offset = _key_offset_for_cmd(cmd);
    return (uint8_t)(0xA0u | (((2u * (uint32_t)rolling) + offset) & 0x0Fu));
}

/* ─── 프레임 빌드 ─────────────────────────────
 *  한국 베네치아 Somfy RTS 80비트 frame:
 *    byte 0~6 : key / (cmd<<4)|cks / rolling BE / address LE(byte6=0xF0) + chain-XOR scramble
 *    byte 7   : 0x84 고정 (XOR chain 밖, raw)
 *    byte 8   : cmd 별 (UP=0x20 DOWN=0x2C MY/PROG=0x00) raw
 *    byte 9   : (cmd 별 base) | calc80Checksum 하위 4비트 raw
 *  ※ ESPSomfy 의 byte 7 가변(196+i*4)·timing 표준값을 적용해본 결과 모터가
 *    응답하지 않아 byte 7 = 0x84 고정 + SDR 실측 timing 으로 원복. */
static void _build_frame(uint8_t *frame,
                          somfy_command_t cmd,
                          uint16_t rolling, uint32_t address)
{
    frame[0] = _key_for_rolling(rolling, cmd);
    /* cmd nibble = enum 값의 하위 4비트 (Tilt UP=0x1B, DOWN=0x2B 의 경우 0xB).
     *  byte 1 high nibble = cmd nibble, low nibble = checksum (아래서 OR). */
    frame[1] = (uint8_t)((cmd & 0x0F) << 4);
    frame[2] = (rolling >> 8) & 0xFF;
    frame[3] =  rolling       & 0xFF;
    /* 시추오 실측: frame byte4=ID(LSB) · byte5=중간(등차) · byte6=0xF0(MSB).
     *  주소값은 0xF0[중간][ID] 이므로 LE 로 풀어야 byte6=0xF0 이 된다.
     *  byte6=0xF0 은 시추오 전 프레임(채널/ALL/PROG/2리모컨) 공통 = 모터의 ALL 내부 인식 키.
     *  ※ 이전 BE 분해는 byte6 에 ID 가 들어가 1번은 정확매칭으로 동작했으나, ALL 은
     *    모터가 byte6=0xF0 기준으로 인식하는데 우리 byte6≠F0 라 무응답이었음. */
    frame[4] =  address        & 0xFF;   /* LSB = 보드 ID */
    frame[5] = (address >>  8) & 0xFF;   /* 중간 = 채널 등차 */
    frame[6] = (address >> 16) & 0xFF;   /* MSB = 0xF0 */

    /* checksum = XOR of all nibbles of byte 0~6 */
    uint8_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum ^= frame[i] ^ (frame[i] >> 4);
    }
    frame[1] |= (checksum & 0x0F);

    /* obfuscation: byte 1~6 chain-XOR */
    for (int i = 1; i < 7; i++) {
        frame[i] ^= frame[i - 1];
    }

    /* byte 7~9 (chain 밖) */
    frame[7] = 0x84;
    frame[8] = _byte8_for_cmd(cmd);
    /* ★동시작동(UP+DOWN/MY+UP/MY+DOWN)은 정품 byte9 가 calc80 공식과 불일치(단일은 일치)
     *  → 정품 실측값 직접 지정(h2_15: UP+DOWN=0xC0, MY+UP/DOWN=0x59). 구현 후 캡처로 frame 일치 검증. */
    if (cmd == SOMFY_CMD_UP_DOWN) {
        frame[9] = 0xC0;
    } else if (cmd == SOMFY_CMD_MY_UP || cmd == SOMFY_CMD_MY_DOWN) {
        frame[9] = 0x59;
    } else {
        uint8_t b9_base = _byte9_base_for_cmd(cmd);
        frame[9] = b9_base | _calc80_checksum(frame[7], frame[8], b9_base);
    }

    ESP_LOGD(TAG, "Frame10: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             frame[0], frame[1], frame[2], frame[3], frame[4],
             frame[5], frame[6], frame[7], frame[8], frame[9]);
}

/* ─── RMT 심볼 변환 ──────────────────────────
 *  wake(첫 frame) → HW sync × N → SW sync → 80비트 Manchester → inter-frame gap.
 *  Manchester 극성: '1' = LOW→HIGH, '0' = HIGH→LOW (mid-bit rising/falling).
────────────────────────────────────────────── */
static int _frame_to_rmt(const uint8_t *frame, rmt_symbol_word_t *buf,
                           int hw_sync_count, bool wake)
{
    int idx = 0;

    /* 0. 정품 preamble (burst 첫 frame 에만) — 짧은 HIGH + 증가하는 LOW
     *  (1700/3744/5460/5688μs). 정품 리모컨1·2 공통 파형. period(P1)가 첫 LOW 1700
     *  과 일치 → 신호 형태를 정품과 맞춤(기존 단일 wake 9660+6644 대체). */
    if (wake) {
        static const uint16_t pre_lo[4] = { 1700, 3744, 5460, 5688 };
        for (int p = 0; p < 4; p++)
            idx = _add_symbol(buf, idx, SOMFY_T_PRE_HI, 1, pre_lo[p], 0);
    }

    /* 1. HW sync */
    for (int i = 0; i < hw_sync_count; i++) {
        idx = _add_symbol(buf, idx,
                          SOMFY_T_HWSYNC_ON,  1,
                          SOMFY_T_HWSYNC_OFF, 0);
    }

    /* 2. SW sync */
    idx = _add_symbol(buf, idx,
                      SOMFY_T_SWSYNC_ON,  1,
                      SOMFY_T_SWSYNC_OFF, 0);

    /* 3. Manchester 80비트 ('1' = LH, '0' = HL) */
    for (int byte_idx = 0; byte_idx < 10; byte_idx++) {
        for (int bit_idx = 7; bit_idx >= 0; bit_idx--) {  // MSB first
            uint8_t bit = (frame[byte_idx] >> bit_idx) & 0x01;
            if (bit) {
                idx = _add_symbol(buf, idx,
                                  SOMFY_T_SYMBOL, 0,
                                  SOMFY_T_SYMBOL, 1);
            } else {
                idx = _add_symbol(buf, idx,
                                  SOMFY_T_SYMBOL, 1,
                                  SOMFY_T_SYMBOL, 0);
            }
        }
    }

    /* 4. inter-frame gap (LOW) */
    if (SOMFY_T_INTER_FRAME > 0) {
        idx = _add_symbol(buf, idx,
                          SOMFY_T_INTER_FRAME / 2,             0,
                          SOMFY_T_INTER_FRAME - SOMFY_T_INTER_FRAME / 2, 0);
    }
    return idx;
}

/* 단일 frame RMT 송신. 송신 소요시간만큼 vTaskDelay (trans_done 미사용). */
static void _transmit_frame(somfy_rts_t *ctx, const uint8_t *frame,
                             int hw_sync_count, bool wake)
{
    int sym_count = _frame_to_rmt(frame, s_rmt_buf, hw_sync_count, wake);
    (void)ctx;

    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    esp_err_t e = rmt_transmit(s_tx_chan, s_copy_enc, s_rmt_buf,
                               sym_count * sizeof(rmt_symbol_word_t),
                               &tx_config);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "rmt_transmit 실패(%s) — 프레임 skip",
                 esp_err_to_name(e));
        return;
    }

    uint32_t us = (wake ? (uint32_t)(SOMFY_T_WAKE_HI + SOMFY_T_WAKE_LO) : 0u)
                + (uint32_t)hw_sync_count *
                  (uint32_t)(SOMFY_T_HWSYNC_ON + SOMFY_T_HWSYNC_OFF)
                + (uint32_t)(SOMFY_T_SWSYNC_ON + SOMFY_T_SWSYNC_OFF)
                + 80u * 2u * (uint32_t)SOMFY_T_SYMBOL
                + (uint32_t)SOMFY_T_INTER_FRAME;
    uint32_t ms = (us + 999u) / 1000u + 4u;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ─── 공개 API ───────────────────────────────── */

bool somfy_rts_init(somfy_rts_t *ctx, cc1101_t *dev)
{
    ctx->cc1101 = dev;

    /* RMT TX 채널 설정 (CC1101_GD0 핀에 연결). 한 frame 최대 95 심볼이
     *  mem_block_symbols=128 안에 단발 적재되므로 refill ISR 불필요 → 라디오
     *  ISR 폭주 환경에서도 송신이 안정적이다. */
    rmt_tx_channel_config_t tx_chan_cfg = {
        .clk_src         = RMT_CLK_SRC_DEFAULT,
        .gpio_num        = CC1101_PIN_GD0,
        .mem_block_symbols = 128,
        .resolution_hz   = 1000000,   // 1μs 분해능
        .intr_priority   = 3,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma   = false,
    };
    esp_err_t e = rmt_new_tx_channel(&tx_chan_cfg, &s_tx_chan);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel 실패: %s — RF 비활성", esp_err_to_name(e));
        return false;
    }
    rmt_copy_encoder_config_t copy_enc_cfg = {};
    if (rmt_new_copy_encoder(&copy_enc_cfg, &s_copy_enc) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder 실패 — RF 비활성");
        return false;
    }
    if (rmt_enable(s_tx_chan) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable 실패 — RF 비활성");
        return false;
    }
    ESP_LOGI(TAG, "Somfy RTS 초기화 완료 (GD0=IO%d)", CC1101_PIN_GD0);
    return true;
}

/* 테스트 전용: CC1101/RF 없이 9바이트 프레임만 생성(순수 가상 검증). */
void somfy_rts_test_build_frame(uint8_t out[10], somfy_command_t cmd,
                                uint16_t rolling, uint32_t address)
{
    if (out) _build_frame(out, cmd, rolling, address);
}

/* ─── hold 반복 송신 중단 플래그 ────────────────────
 *  somfy_rts_abortable : 다음 somfy_rts_send 가 중단 가능한가
 *                        (일반 누름 burst=false → 끝까지, hold 반복=true)
 *  somfy_rts_abort     : true 면 abortable 송신을 즉시 중단 */
volatile bool somfy_rts_abort       = false;
volatile bool somfy_rts_abortable   = false;
volatile bool somfy_rts_keep_rolling = false;   /* true=이번 송신은 롤링코드 증가 안 함(hold 반복) */

void somfy_rts_send(somfy_rts_t *ctx, somfy_blind_t *blind,
                    somfy_command_t cmd, uint32_t hold_ms)
{
    /* 이 송신의 중단 허용 여부를 진입 시점에 캡처(송신 중 바뀌어도 고정) */
    bool abortable = somfy_rts_abortable;

    /* 주파수 설정 */
    cc1101_set_frequency(ctx->cc1101, blind->freq_mhz);

    /* ★ Somfy RTS 표준: 롤링코드는 버튼 누름 1회당 1번만 증가.
     *  같은 누름의 모든 반복 프레임(wake + repeats + hold_ms sustained)은
     *  반드시 동일한 롤링코드/프레임을 사용해야 모터가 한 번의 누름으로
     *  인식한다. (Nickduino/Tasmota/ESPHome 등 모든 표준 구현 동일.)
     *  이전엔 매 반복마다 증가 → 모터가 한 누름을 여러 누름으로 혼동
     *  → PROG 등록·UP/DOWN 인식 실패의 근본 원인이었음. */
    /* ★정품 매칭(hold): somfy_rts_keep_rolling=true 면 이번 송신은 증가하지 않고
     *  현재 롤링코드를 재사용한다 — 정품은 버튼을 누르는 동안 같은 코드를 반복한다.
     *  (hold_repeat_task 가 같은 cmd 를 유지하는 동안 이 플래그를 세워 보낸다.) */
    if (!somfy_rts_keep_rolling) blind->rolling_code++;
    somfy_rts_keep_rolling = false;   /* one-shot — 다음 송신 기본은 증가 */

    /* frame 1회 빌드 후 burst 전체에 재사용 (rolling code 동일 유지). */
    uint8_t frame[10];
    _build_frame(frame, cmd, blind->rolling_code, blind->address);

    ESP_LOGI(TAG, "전송: 블라인드[%s] cmd=%d rolling=%d freq=%.2fMHz hold=%" PRIu32 "ms",
             blind->name, cmd, blind->rolling_code, blind->freq_mhz, hold_ms);

    /* ★★ burst 당 STX 1회 (SDR 워터폴 번짐 수정):
     *  매 프레임 SIDLE→STX 하면 FS_AUTOCAL 로 frame 마다 재calibration →
     *  carrier 주파수가 frame 마다 흩어져 스펙트럼이 넓게 번진다.
     *  burst 시작 시 cc1101_enter_tx_mode 를 1회만 호출(calibration 1회),
     *  이후 모든 frame 은 TX state 를 유지한 채 RMT 송신만 한다. */
    cc1101_enter_tx_mode(ctx->cc1101);
    /* ★ TX 진입(MARCSTATE=TX) 후에도 VCO/PA 가 완전히 안정되기까지 잠깐
     *  여유 — SDR 실측상 burst 첫 frame 이 settling 중 깨져 나갔다.
     *  5ms 지연 후 첫 frame 송신 → 모든 frame 이 안정 상태에서 나간다. */
    vTaskDelay(pdMS_TO_TICKS(5));

    /* ★★ 한국 베네치아 정품 구조로 송신 (rxbyte 펄스 dump 확정):
     *  - HW sync 13 cycles (정품 frame period 141ms 역산: HW 63ms)
     *  - inter-frame gap(silence) 없음 — frame 끝나면 바로 다음 frame HW sync
     *  - wake-up 펄스 없음 (정품 미사용, AC 모터 항상 RX)
     *  frame 마다 개별 rmt_transmit, 송신시간만큼 vTaskDelay 로 연속 송신. */
    /* ★ 정품 리모컨 동작: 버튼을 누르고 있는 동안 동일 frame(동일 롤링코드)을
     *  계속 반복 송신하고, 버튼을 떼면 즉시 멈춘다.
     *   min_loops : 짧은 탭이라도 보장하는 최소 burst(SOMFY_REPEAT_COUNT).
     *   max_loops : hold_ms 상한 — 버튼 PRESS 시 큰 값을 넘겨 길게 누름 허용.
     *  abortable 송신은 min_loops 이후 somfy_rts_abort(버튼 뗌)가 서면 한
     *  frame(~150ms) 안에 종료된다. 일반 누름(abortable=false, hold_ms=0)은
     *  정확히 min_loops 만 송신. 롤링코드는 이 호출 1회당 1번만 증가하므로
     *  길게 눌러도 같은 롤링코드의 frame 이 반복된다(정품과 동일). */
    /* ★ ESPSomfy-RTS 80비트 패턴 적용:
     *   첫 프레임 : wake 펄스 + HW sync 9 cycle
     *   반복 프레임: wake 없음 + HW sync 6 cycle (~143ms / frame)
     *  hold_ms 상한은 반복 frame 시간(143ms) 기준으로 환산. */
    /* ★★2026-08-13 PROG 특수처리 **제거**. 근거 없는 추측이었다.
     *
     *  제거한 것: `min_loops = SOMFY_REPEAT_COUNT * 2` 와 첫 프레임 `hw_sync = 17`.
     *  근거로 적혀 있던 "시추오 실측상 PROG 는 일반보다 2배 길고 HW sync 많음
     *  (span 307k vs 174k, HWsync 34 vs 23)" 은 **실측 데이터를 잘못 읽은 것**이다.
     *
     *  실제 캡처 전수 측정 (D:\RTL_SDR\sdrsharp-x64\somfy_rts_447,
     *  분석 스크립트 plugin-Rtl433-for-SdrSharp-master/scripts):
     *    송신 길이 중앙값 — up/down/my/prog/tilt 전부 **350~400ms 동일**
     *    HW sync 개수(3_42.1dB, 첫 burst)
     *        up   31,28,29,24   down 36,36,36,36
     *        my   28,30,28      prog 29,34,29      ← PROG 가 down 보다 오히려 적다
     *  즉 PROG 는 다른 버튼과 다르지 않다. 위 "2배" 는 **일부러 길게 누른 캡처**를
     *  PROG 고유 특성으로 오독하고, HW sync 도 PROG 최대값(34)과 타 버튼 최소값(23)을
     *  골라 비교한 결과였다. (사용자가 짧게/길게 구분용으로 긴 누름을 섞어 넣어둔 것)
     *  분석 문서에도 정품 구조가 "HW sync 12회(첫 프레임)/6회(재전송), 버튼 구분 없음"
     *  으로 이미 적혀 있었다.
     *
     *  길게 누르면 길게 나가는 동작은 그대로다 — 그건 hold_ms(PROG=15000ms)가 한다. */
    int min_loops = SOMFY_REPEAT_COUNT;
    int max_loops = min_loops + (int)(hold_ms / 143);
    for (int i = 0; i < max_loops; i++) {
        if (i >= min_loops && abortable && somfy_rts_abort) {
            ESP_LOGI(TAG, "버튼 뗌 — 송신 종료 (%d frame)", i);
            break;
        }
        bool first = (i == 0);
        /* 첫 frame 12 cycles → cpt_synchro_hw==12 → 80-bit 식별 (ESPSomfy 수신
         *  분석값). 반복은 6 cycles. **버튼 종류와 무관** — 정품 캡처 분석 문서의
         *  "HW sync 12회(첫 프레임)/6회(재전송)" 와 일치한다.
         *  (PROG 만 17 로 올렸던 특수처리는 오독이라 제거 — 위 주석 참조) */
        int hw_sync = first ? 12 : 6;
        _transmit_frame(ctx, frame, hw_sync, first);
    }

    /* CC1101 IDLE 복귀 (burst 끝 — TX state 종료) */
    cc1101_idle(ctx->cc1101);
}

void somfy_rts_tilt(somfy_rts_t *ctx, somfy_blind_t *blind, bool up)
{
    /* 정품 Tilt 커맨드 (cmd nibble 0xB) — byte 8 로 방향 구분. */
    somfy_rts_send(ctx, blind, up ? SOMFY_CMD_TILT_UP : SOMFY_CMD_TILT_DOWN, 0);
}

void somfy_rts_send_steps(somfy_rts_t *ctx, somfy_blind_t *blind,
                           somfy_command_t cmd, uint8_t step_count)
{
    if (step_count == 0) return;
    if (step_count > 10) step_count = 10;   /* 안전 cap */

    cc1101_set_frequency(ctx->cc1101, blind->freq_mhz);

    /* ★ 다단 step 핵심: cc1101_enter_tx_mode 를 burst 시작 시 1회만 호출.
     *  매 step 마다 enter/idle 안 함 → step 간 ~20ms CC1101 오버헤드 제거
     *  → 슬랫이 끊어짐 없이 연속 회전. */
    cc1101_enter_tx_mode(ctx->cc1101);
    vTaskDelay(pdMS_TO_TICKS(5));   /* VCO/PA settle */

    ESP_LOGI(TAG, "Tilt steps: 블라인드[%s] cmd=%d count=%u",
             blind->name, cmd, step_count);

    for (uint8_t s = 0; s < step_count; s++) {
        blind->rolling_code++;

        uint8_t frame[10];
        _build_frame(frame, cmd, blind->rolling_code, blind->address);

        ESP_LOGI(TAG, "  step %u/%u: rolling=%d",
                 (unsigned)(s + 1), (unsigned)step_count, blind->rolling_code);

        /* step 당 2 frames: wake frame(12 HW sync) + repeat(6 HW sync).
         *  정품 detent 1회 누름과 동일한 frame 구조 (3 frames 대신 2 로 줄여
         *  step 당 ~143ms 단축, 모터는 1+ frame 만 받아도 인식). */
        _transmit_frame(ctx, frame, /*hw_sync=*/12, /*wake=*/true);
        _transmit_frame(ctx, frame, /*hw_sync=*/6,  /*wake=*/false);
    }

    cc1101_idle(ctx->cc1101);
}
