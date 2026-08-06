/*
 * somfy_stress_test.c — 실제 CC1101 온에어 스트레스 테스트. v3.5.
 *
 * 목적: RF 경로(RMT 인터럽트 우선순위/단발 적재, cc1101 TX 진입 타임아웃,
 *  실패 시 무재부팅 채널 복구) 수정이 지속 부하에서도 견고한지 검증.
 *
 * 동작: 라운드 r 마다 r*STRESS_STEP 회 실제 RF 송신을 수행하며 라운드를
 *  무한 증가(1,2,3,...)시킨다. 각 송신은 블라인드(1~5)·주파수(대역
 *  6분할)·커맨드(UP/DOWN/MY)를 회전하고, 6번째마다 ALL(5개 순차)을 섞는다.
 *  누적 총/PASS/FAIL/경과초를 50회마다·라운드마다 표준 레벨 로그로 출력.
 *  반환하지 않는다(사용자가 중단할 때까지). NVS·실제 롤링코드 불변
 *  (g_mgr 로컬 복제).  부트루프 발생 시 로그 타임스탬프가 리셋되고
 *  STRESS 가 R1 부터 다시 시작되므로 외부에서 즉시 식별 가능.
 */
#include "somfy_stress_test.h"
#include "cc1101.h"
#include "somfy_rts.h"
#include "blind_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

static const char *TAG = "STRESS";

extern cc1101_t        g_cc1101;
extern somfy_rts_t     g_somfy;
extern blind_manager_t g_mgr;
extern bool            g_rf_ready;

static const float k_freqs[] = {
    447.20f, 447.30f, 447.43f, 447.55f, 447.66f, 447.79f,
};
#define NUM_FREQS  (sizeof(k_freqs) / sizeof(k_freqs[0]))
#define STRESS_STEP 10u     /* 라운드당 증가 단위 (R1=10, R2=20, ...) */

/* 5개 테스트 블라인드: 실제 등록분은 그대로, 부족분 합성(로컬 복제). */
static void _build_blinds(blind_manager_t *tb)
{
    memcpy(tb, &g_mgr, sizeof(*tb));
    for (int i = 0; i < 5; i++) {
        bool real = (i < g_mgr.count) && g_mgr.blinds[i].active;
        if (!real) {
            somfy_blind_t *b = &tb->blinds[i];
            memset(b, 0, sizeof(*b));
            b->address      = 0x100001u + (uint32_t)i;
            b->rolling_code = (uint16_t)(0x0010 + i);
            float f = 447.20f + 0.12f * (float)i;
            if (f > 447.79f) f = 447.79f;
            b->freq_mhz = f;
            b->active   = true;
            snprintf(b->name, sizeof(b->name), "B%d", i + 1);
        }
    }
    tb->count = 5;
}

/* 1회 실제 송신 + 송신 완료(롤링 증가) 검증. */
static bool _tx(somfy_blind_t *b, somfy_command_t cmd)
{
    uint16_t r0 = b->rolling_code;
    somfy_rts_send(&g_somfy, b, cmd, 0);
    return (b->rolling_code != r0);
}

void somfy_stress_test_run(void)
{
    ESP_LOGW(TAG, "===== Somfy 온에어 스트레스 테스트 시작 "
                  "(실제 송신·무한·라운드마다 +%u회) =====",
             (unsigned)STRESS_STEP);

    if (!g_rf_ready) {
        ESP_LOGE(TAG, "CC1101 미준비(하드웨어 미응답) — 스트레스 테스트 불가");
        return;
    }

    blind_manager_t tb;
    _build_blinds(&tb);

    const somfy_command_t cmds[] = {
        SOMFY_CMD_UP, SOMFY_CMD_DOWN, SOMFY_CMD_MY,
    };

    uint32_t total = 0, pass = 0, fail = 0;
    int64_t  t0 = esp_timer_get_time();

    for (uint32_t round = 1; ; round++) {
        uint32_t iters  = round * STRESS_STEP;
        uint32_t r_pass = 0;
        ESP_LOGW(TAG, "[R%" PRIu32 "] 시작 — 이번 라운드 %" PRIu32
                      "회 (누적 %" PRIu32 "회)", round, iters, total);

        for (uint32_t k = 0; k < iters; k++) {
            int bi = (int)(k % 6);                 /* 0~4=블라인드, 5=ALL */
            somfy_command_t cmd = cmds[k % 3];
            bool ok;

            if (bi < 5) {
                somfy_blind_t *b = &tb.blinds[bi];
                b->freq_mhz = k_freqs[k % NUM_FREQS];   /* 주파수 회전 */
                ok = _tx(b, cmd);
            } else {
                /* ALL — 5개 블라인드 순차 실제 송신(동시 제어 모델) */
                ok = true;
                for (int j = 0; j < 5; j++) {
                    if (!_tx(&tb.blinds[j], cmd)) ok = false;
                }
            }

            total++;
            if (ok) { pass++; r_pass++; }
            else      fail++;

            if (total % 50 == 0) {
                int64_t el = (esp_timer_get_time() - t0) / 1000000;
                ESP_LOGW(TAG, "[누적] 총=%" PRIu32 " PASS=%" PRIu32
                              " FAIL=%" PRIu32 " 경과=%llds",
                         total, pass, fail, (long long)el);
            }
            vTaskDelay(pdMS_TO_TICKS(5));   /* 태스크 양보 */
        }

        int64_t el = (esp_timer_get_time() - t0) / 1000000;
        ESP_LOGW(TAG, "[R%" PRIu32 "] 완료 — 라운드 %" PRIu32 "/%" PRIu32
                      " PASS | 누적 총=%" PRIu32 " PASS=%" PRIu32
                      " FAIL=%" PRIu32 " 경과=%llds",
                 round, r_pass, iters, total, pass, fail, (long long)el);
    }
}
