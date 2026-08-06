/*
 * somfy_txdecode_test.c — 우리 TX 송신을 자기 GD0 핀에서 (lv,dur) 펄스로
 *  캡처해 7바이트로 디코드. 정품 rxbyte 결과와 byte-level 비교 가능.
 *  CC1101 RX 거치지 않으므로 OOK demod jitter 영향 없음 — RMT 가 만든
 *  깨끗한 펄스만 본다.
 *
 * 방법(txprobe 의 캡처 메커니즘 + rxbyte 의 Manchester 디코더 결합):
 *  1) 폴링 태스크 prio=2 가 세마포 대기
 *  2) somfy_rts_send(PROG) 호출 직전 세마포 give → 폴링 시작
 *  3) RMT 가 GD0 구동하는 동안(rmt_tx_wait_all_done 가 호출 태스크 블록)
 *     폴링 태스크가 IO8 의 (lv,dur) 펄스 캡처
 *  4) 끝나면 rxbyte 동일 알고리즘으로 디코드:
 *     SW sync 찾기 → 56 Manchester 비트 → 7바이트 MSB-first → 역난독화
 *  5) 동시에 somfy_rts_test_build_frame() 으로 기대값 계산, byte-by-byte
 *     비교해 일치/불일치 보고
 *
 * 사용: ./test/build_test.ps1 -Action build -Mode txdecode 후 flash.
 *  부팅 후 1회 송신·캡처·디코드·비교 → 로그 후 유지.
 */
#include "somfy_txdecode_test.h"
#include "cc1101.h"
#include "somfy_rts.h"
#include "blind_manager.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "TXDEC";

extern cc1101_t        g_cc1101;
extern somfy_rts_t     g_somfy;
extern blind_manager_t g_mgr;
extern bool            g_rf_ready;

#define GD0_PIN        CC1101_PIN_GD0
#define MAX_EDGES      8192             /* 64KB static — Matter/Thread heap 침범 안 함 */
#define CAPTURE_MS     2800             /* 처음 ~2.8초 = ~18 frames burst (검증 충분) */
#define SYMBOL_US      640

typedef struct { uint8_t lv; uint32_t dur; } pulse_t;

static pulse_t s_pulses[MAX_EDGES];
static volatile int  s_n = 0;
static volatile bool s_capture_done = false;
static SemaphoreHandle_t s_start_sem = NULL;

static void _poll_task(void *pv)
{
    (void)pv;
    xSemaphoreTake(s_start_sem, portMAX_DELAY);

    s_n = 0;
    int last = gpio_get_level(GD0_PIN);
    int64_t t0 = esp_timer_get_time();
    int64_t tprev = t0;
    int64_t until = t0 + (int64_t)CAPTURE_MS * 1000;
    while (esp_timer_get_time() < until && s_n < MAX_EDGES) {
        int lv = gpio_get_level(GD0_PIN);
        if (lv != last) {
            int64_t now = esp_timer_get_time();
            uint32_t w = (uint32_t)(now - tprev);
            tprev = now;
            if (w >= 50 && w <= 100000) {
                s_pulses[s_n].lv  = (uint8_t)last;
                s_pulses[s_n].dur = w;
                s_n++;
            }
            last = lv;
        }
    }
    s_capture_done = true;
    vTaskDelete(NULL);
}

/* SW sync 찾기 — active_lv 펄스 ~4836µs + idle 펄스 ~1280µs */
static int _find_sw_sync_end(int active_lv, int *out_data_start)
{
    int idle_lv = active_lv ? 0 : 1;
    for (int i = 0; i < s_n - 1; i++) {
        if (s_pulses[i].lv != active_lv) continue;
        uint32_t a = s_pulses[i].dur;
        if (a < 3500 || a > 6300) continue;
        if (s_pulses[i + 1].lv != idle_lv) continue;
        uint32_t b = s_pulses[i + 1].dur;
        if (b < 800 || b > 1900) continue;
        *out_data_start = i + 2;
        return 0;
    }
    return -1;
}

static int _halves(uint32_t dur)
{
    if (dur < SYMBOL_US / 2) return 0;
    return (int)((dur + SYMBOL_US / 2) / SYMBOL_US);
}

/* 한 프레임 디코드 시도. 시작 인덱스부터 SW sync 검색 → 56 비트 → 7 바이트
 *  → 역난독화. 성공=0, 실패=-1. 다음 검색 시작 인덱스를 *next_start 로. */
static int _decode_one_frame(int from, int active_lv,
                             uint8_t raw[7], uint8_t deob[7], int *next_start)
{
    int data_start = -1;
    /* SW sync 검색 범위를 from 부터로 한정 */
    for (int i = from; i < s_n - 1; i++) {
        if (s_pulses[i].lv != active_lv) continue;
        uint32_t a = s_pulses[i].dur;
        if (a < 3500 || a > 6300) continue;
        if (s_pulses[i + 1].lv != (active_lv ? 0 : 1)) continue;
        uint32_t b = s_pulses[i + 1].dur;
        if (b < 800 || b > 1900) continue;
        data_start = i + 2;
        break;
    }
    if (data_start < 0) return -1;

    static uint8_t halves[256];
    int h = 0;
    for (int i = data_start; i < s_n && h < (int)sizeof(halves); i++) {
        int nh = _halves(s_pulses[i].dur);
        uint8_t norm_lv = (s_pulses[i].lv == active_lv) ? 1 : 0;
        for (int k = 0; k < nh && h < (int)sizeof(halves); k++) {
            halves[h++] = norm_lv;
        }
        if (h >= 112) { *next_start = i + 1; break; }
    }
    if (h < 112) return -1;

    uint8_t bits[56];
    for (int b = 0; b < 56; b++) bits[b] = halves[b * 2];

    memset(raw, 0, 7);
    for (int B = 0; B < 7; B++)
        for (int b = 0; b < 8; b++)
            if (bits[B * 8 + b]) raw[B] |= 1 << (7 - b);

    memcpy(deob, raw, 7);
    for (int i = 6; i >= 1; i--) deob[i] ^= deob[i - 1];
    return 0;
}

void somfy_txdecode_test_run(void)
{
    ESP_LOGW(TAG, "===== TX 송신 자가 디코더 시작 =====");
    if (!g_rf_ready) {
        ESP_LOGE(TAG, "CC1101 미준비");
        return;
    }

    /* 대상 블라인드 복사(NVS 보호). 없으면 합성. */
    somfy_blind_t tb;
    if (g_mgr.count >= 1 && g_mgr.blinds[0].active) {
        tb = g_mgr.blinds[0];
    } else {
        memset(&tb, 0, sizeof(tb));
        tb.address      = 0x100001u;
        tb.rolling_code = 0x0010;
        tb.freq_mhz     = 447.60f;
        tb.active       = true;
    }

    /* 기대값(우리 코드가 만들 7바이트) 미리 계산 — 비교용.
     *  somfy_rts_send 가 ++rolling 후 송신하므로 rolling+1 사용. */
    uint8_t expected[7];
    somfy_rts_test_build_frame(expected, SOMFY_CMD_PROG,
                               tb.rolling_code + 1, tb.address);
    ESP_LOGW(TAG, "[기대] _build_frame(PROG, roll=%u, addr=0x%06lX):",
             (unsigned)(tb.rolling_code + 1), (unsigned long)tb.address);
    ESP_LOGW(TAG, "  난독화 7바이트: %02X %02X %02X %02X %02X %02X %02X",
             expected[0], expected[1], expected[2], expected[3],
             expected[4], expected[5], expected[6]);

    /* GD0 입력 enable (RMT 출력 매트릭스는 그대로) */
    gpio_input_enable(GD0_PIN);

    s_start_sem    = xSemaphoreCreateBinary();
    s_capture_done = false;
    s_n            = 0;

    xTaskCreate(_poll_task, "txdec_poll", 4096, NULL, 2, NULL);

    ESP_LOGW(TAG, "[송신] PROG (freq=%.2fMHz addr=0x%06lX roll=%u→%u)",
             tb.freq_mhz, (unsigned long)tb.address,
             (unsigned)tb.rolling_code, (unsigned)(tb.rolling_code + 1));
    xSemaphoreGive(s_start_sem);
    somfy_rts_send(&g_somfy, &tb, SOMFY_CMD_PROG, 0);

    int waited = 0;
    while (!s_capture_done && waited < 3000) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited += 50;
    }
    ESP_LOGW(TAG, "[캡처] %d 펄스 수집", s_n);

    /* ── 펄스 통계 (Somfy 표준 bins) ── */
    int b_sym = 0, b_dbl = 0, b_hw = 0, b_sw = 0, b_other = 0;
    for (int i = 0; i < s_n; i++) {
        uint32_t w = s_pulses[i].dur;
        if      (w >= 420  && w <= 880)  b_sym++;
        else if (w >= 900  && w <= 1700) b_dbl++;
        else if (w >= 1700 && w <= 3200) b_hw++;
        else if (w >= 3600 && w <= 6200) b_sw++;
        else b_other++;
    }
    ESP_LOGW(TAG, "[통계] sym=%d dbl=%d HW싱크=%d SW싱크=%d 기타=%d",
             b_sym, b_dbl, b_hw, b_sw, b_other);

    /* ── SW sync 위치 모아 frame 간격 측정 (active_lv=1, ~4836µs HIGH) ── */
    int sw_idx[64]; int sw_cnt = 0;
    for (int i = 0; i + 1 < s_n && sw_cnt < 64; i++) {
        if (s_pulses[i].lv == 1 && s_pulses[i].dur >= 3800 && s_pulses[i].dur <= 6000
            && s_pulses[i + 1].lv == 0 && s_pulses[i + 1].dur >= 800 && s_pulses[i + 1].dur <= 1900) {
            sw_idx[sw_cnt++] = i;
        }
    }
    /* SW sync 시각 (캡처 시작 = 0us 기준) 계산 + 간격 출력 */
    uint64_t sw_t[64];
    uint64_t acc = 0;
    int sk = 0;
    for (int i = 0; i < s_n && sk < sw_cnt; i++) {
        if (i == sw_idx[sk]) { sw_t[sk++] = acc; }
        acc += s_pulses[i].dur;
    }
    char gaps[400]; int p = 0;
    for (int k = 1; k < sw_cnt && p < 380; k++) {
        p += snprintf(gaps + p, sizeof(gaps) - p, "%llums ",
                      (unsigned long long)((sw_t[k] - sw_t[k - 1]) / 1000));
    }
    ESP_LOGW(TAG, "[burst] 총 frame수=%d, frame 간격=[%s]", sw_cnt, gaps);

    /* ── 프레임 디코드 (모든 SW sync 마다) ── */
    int frame_idx = 0;
    int from = 0;
    int match_count = 0;
    while (from < s_n - 10 && frame_idx < 40) {
        uint8_t raw[7], deob[7];
        int next = from;
        if (_decode_one_frame(from, 1, raw, deob, &next) != 0) break;
        frame_idx++;
        ESP_LOGW(TAG, "[프레임%d] 원시  : %02X %02X %02X %02X %02X %02X %02X",
                 frame_idx, raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]);
        ESP_LOGW(TAG, "[프레임%d] 역난독: %02X %02X %02X %02X %02X %02X %02X",
                 frame_idx, deob[0], deob[1], deob[2], deob[3], deob[4], deob[5], deob[6]);

        bool match = (memcmp(raw, expected, 7) == 0);
        if (match) match_count++;
        ESP_LOGW(TAG, "[프레임%d] 기대값과 %s",
                 frame_idx, match ? "★일치★" : "≠ 불일치");
        if (!match) {
            for (int i = 0; i < 7; i++) {
                if (raw[i] != expected[i]) {
                    ESP_LOGW(TAG, "  byte[%d]: 캡처=0x%02X  기대=0x%02X  XOR=0x%02X",
                             i, raw[i], expected[i], raw[i] ^ expected[i]);
                }
            }
        }
        from = (next > from) ? next : from + 10;
    }

    ESP_LOGW(TAG, "===== 결과 요약: %d/%d 프레임이 기대값과 일치 =====",
             match_count, frame_idx);
    if (frame_idx == 0) {
        ESP_LOGE(TAG, "★ 디코드 실패 — SW sync 패턴조차 못 찾음. RMT 출력 결함 의심.");
    } else if (match_count == frame_idx) {
        ESP_LOGW(TAG, "★ 우리 송신 frame = 우리 _build_frame() 출력 100%% 일치.");
        ESP_LOGW(TAG, "★ 정품 rxbyte 결과(cmd=8, addr=0xC91BF0)와 cmd·구조 모두 같음.");
        ESP_LOGW(TAG, "★ 결론: 디지털 송신 OK → 잔여 원인은 RF 출력·timing 또는 주소 mismatch.");
    } else {
        ESP_LOGE(TAG, "★ 일부 frame 깨짐(%d/%d) — RMT/캡처 jitter 또는 frame builder 버그.",
                 frame_idx - match_count, frame_idx);
    }

    /* 첫 30개 (lv, dur) 덤프 — 시각 점검 */
    ESP_LOGW(TAG, "[펄스 덤프] 첫 30개 (lv,µs):");
    for (int i = 0; i < s_n && i < 30; i++) {
        ESP_LOGW(TAG, "  [%2d] lv=%d  %5luµs", i,
                 s_pulses[i].lv, (unsigned long)s_pulses[i].dur);
    }

    ESP_LOGW(TAG, "===== TX 자가 디코더 종료 (대기) =====");
    while (1) vTaskDelay(pdMS_TO_TICKS(60000));
}
