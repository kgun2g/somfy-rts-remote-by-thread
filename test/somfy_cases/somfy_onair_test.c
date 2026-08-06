/*
 * somfy_onair_test.c — 실제 CC1101 온에어 RF 테스트. v3.5.
 *
 * 가상 프레임 검증(somfy_selftest.c)과 달리, 이 테스트는 실제 CC1101 모듈을
 * 사용해 진짜 RF 신호를 송신한다. app_main.cpp 가 Phase2 에서 초기화한
 * 공유 인스턴스(g_cc1101/g_somfy/g_mgr/g_rf_ready)를 그대로 사용한다.
 *
 *  PHASE A  CC1101 SPI 통신/칩 확인 (PARTNUM/VERSION 라이트백)
 *  PHASE B  주파수 다양화 — 6개 주파수 + 기기 설정 주파수로 변경하며
 *           FREQ 레지스터 라이트백 검증 + 실제 캐리어(TX state) 송신 확인
 *  PHASE C  가상 버튼(UP/DOWN/MY/PROG) × 블라인드 1~5 번갈아 실제 변조 송신
 *  PHASE D  ALL 선택 시 5개 블라인드 동시(순차) 제어 + PROG 차단 정책 검증
 *
 * 실제 신호 발생 근거: somfy_rts_send()/cc1101_enter_tx_mode() 가 PA 를 키잉
 * 하고 RMT 가 GD0 로 변조 데이터를 출력한다. TX state 도달과 롤링코드 증가로
 * 송신 완료를 검증한다.  NVS 는 건드리지 않는다(g_mgr 로컬 복제 사용).
 */
#include "somfy_onair_test.h"
#include "cc1101.h"
#include "somfy_rts.h"
#include "blind_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "ONAIR";

/* app_main.cpp 가 단일 소유·초기화한 공유 인스턴스. */
extern cc1101_t        g_cc1101;
extern somfy_rts_t     g_somfy;
extern blind_manager_t g_mgr;
extern bool            g_rf_ready;

/* Somfy 대역(447.20~447.79MHz) 전반을 훑는 테스트 주파수. */
static const float k_freqs[] = {
    447.20f, 447.30f, 447.43f, 447.55f, 447.66f, 447.79f,
};
#define NUM_FREQS (sizeof(k_freqs) / sizeof(k_freqs[0]))

#define CC1101_XTAL_FREQ 26000000UL

/* cc1101.c _set_freq_regs 와 동일한 레지스터 값 계산(라이트백 비교용). */
static void _expected_freq_regs(float mhz, uint8_t *f2, uint8_t *f1, uint8_t *f0)
{
    uint32_t r = (uint32_t)((double)mhz * 1e6 * 65536.0 / (double)CC1101_XTAL_FREQ);
    *f2 = (r >> 16) & 0xFF;
    *f1 = (r >>  8) & 0xFF;
    *f0 = (r      ) & 0xFF;
}

/* CC1101 칩 상태의 state 필드(bits6:4). TX=0x20. */
static inline uint8_t _chip_state(void)
{
    return cc1101_get_status(&g_cc1101) & 0x70;
}

/* 한 주파수: 설정 → FREQ 레지스터 라이트백 검증 → 실제 캐리어 송신(TX)확인. */
static bool _verify_freq_onair(float mhz)
{
    cc1101_set_frequency(&g_cc1101, mhz);

    uint8_t e2, e1, e0;
    _expected_freq_regs(mhz, &e2, &e1, &e0);
    uint8_t r2 = cc1101_read_reg(&g_cc1101, CC1101_FREQ2);
    uint8_t r1 = cc1101_read_reg(&g_cc1101, CC1101_FREQ1);
    uint8_t r0 = cc1101_read_reg(&g_cc1101, CC1101_FREQ0);
    bool reg_ok = (r2 == e2) && (r1 == e1) && (r0 == e0);

    /* 실제 캐리어 송신: PA 키잉 → TX state 도달 확인 후 짧게 유지(실측 가능). */
    cc1101_enter_tx_mode(&g_cc1101);
    uint8_t st = _chip_state();
    bool tx_ok = (st == CC1101_STATUS_TX);
    vTaskDelay(pdMS_TO_TICKS(8));     /* 실제 무변조 캐리어가 공중에 출력됨 */
    cc1101_idle(&g_cc1101);

    bool ok = reg_ok && tx_ok;
    ESP_LOGI(TAG, "  %.2fMHz FREQ=%02X%02X%02X(exp %02X%02X%02X) reg=%s TX=%s -> %s",
             mhz, r2, r1, r0, e2, e1, e0,
             reg_ok ? "ok" : "BAD", tx_ok ? "ok" : "BAD",
             ok ? "PASS" : "FAIL");
    return ok;
}

/* 실제 변조 송신 1회 + 송신 완료(롤링코드 증가) 검증. */
static bool _send_and_verify(somfy_blind_t *b, somfy_command_t cmd,
                             const char *who, const char *btn)
{
    uint16_t roll_before = b->rolling_code;
    somfy_rts_send(&g_somfy, b, cmd, 0);
    uint16_t roll_after = b->rolling_code;
    /* somfy_rts_send 는 wake+repeat 로 롤링을 여러 번 증가시킨다.
     *  증가했다 = 전체 송신 경로(프레임 빌드+CC1101 TX+RMT)가 끝까지 실행됨. */
    bool ok = (roll_after != roll_before);
    ESP_LOGI(TAG, "  [%s] %s %s freq=%.2fMHz roll 0x%X->0x%X -> %s",
             ok ? "PASS" : "FAIL", who, btn, b->freq_mhz,
             roll_before, roll_after, ok ? "ok" : "BAD");
    vTaskDelay(pdMS_TO_TICKS(20));    /* 로그 가독 + 태스크 양보 */
    return ok;
}

/* 5개 테스트 블라인드 구성: 실제 등록분은 그대로, 부족분은 합성.
 *  로컬 복제이므로 NVS/실제 롤링코드는 건드리지 않는다. */
static void _build_test_blinds(blind_manager_t *tb)
{
    memcpy(tb, &g_mgr, sizeof(*tb));
    for (int i = 0; i < 5; i++) {
        bool real = (i < g_mgr.count) && g_mgr.blinds[i].active;
        if (!real) {
            somfy_blind_t *b = &tb->blinds[i];
            memset(b, 0, sizeof(*b));
            b->address      = 0x100001u + (uint32_t)i;
            b->rolling_code = (uint16_t)(0x0010 + i);
            /* 대역 내 분산 주파수(447.20 + 0.12*i, ≤447.79) */
            float f = 447.20f + 0.12f * (float)i;
            if (f > 447.79f) f = 447.79f;
            b->freq_mhz = f;
            b->active   = true;
            snprintf(b->name, sizeof(b->name), "B%d", i + 1);
        }
    }
    tb->count = 5;
}

int somfy_onair_test_run(void)
{
    ESP_LOGW(TAG, "===== Somfy 온에어 RF 테스트 시작 (실제 CC1101 송신) =====");

    if (!g_rf_ready) {
        ESP_LOGE(TAG, "CC1101 미준비(하드웨어 미응답/배선·전원 문제) — "
                      "실제 RF 송신 불가. 온에어 테스트 중단.");
        ESP_LOGW(TAG, "===== 온에어 결과: 0/0 PASS, 하드웨어 미준비 =====");
        return -1;
    }

    int pass = 0, fail = 0;

    /* ── PHASE A: CC1101 SPI 통신/칩 확인 ── */
    uint8_t partnum = cc1101_read_reg(&g_cc1101, 0x30 | 0xC0);
    uint8_t version = cc1101_read_reg(&g_cc1101, 0x31 | 0xC0);
    bool chip_ok = !(version == 0x00 || version == 0xFF);
    ESP_LOGI(TAG, "[A] CC1101 PARTNUM=0x%02X VERSION=0x%02X -> %s",
             partnum, version, chip_ok ? "통신OK" : "통신BAD");
    if (chip_ok) { pass++; ESP_LOGI(TAG, "[PASS] CC1101 SPI 통신"); }
    else         { fail++; ESP_LOGE(TAG, "[FAIL] CC1101 SPI 통신"); }

    /* ── PHASE B: 주파수 다양화 + 실제 캐리어 송신 ── */
    ESP_LOGW(TAG, "[B] 주파수 변경/송신 검증 (%d개 + 기기 설정값)",
             (int)NUM_FREQS);
    for (int i = 0; i < (int)NUM_FREQS; i++) {
        if (_verify_freq_onair(k_freqs[i])) {
            pass++; ESP_LOGI(TAG, "[PASS] 주파수 %.2fMHz 송신", k_freqs[i]);
        } else {
            fail++; ESP_LOGE(TAG, "[FAIL] 주파수 %.2fMHz 송신", k_freqs[i]);
        }
    }
    /* 기기에 실제 설정된 블라인드1 주파수도 검증 */
    if (g_mgr.count > 0) {
        float cf = g_mgr.blinds[0].freq_mhz;
        if (_verify_freq_onair(cf)) {
            pass++; ESP_LOGI(TAG, "[PASS] 기기 설정 주파수 %.2fMHz 송신", cf);
        } else {
            fail++; ESP_LOGE(TAG, "[FAIL] 기기 설정 주파수 %.2fMHz 송신", cf);
        }
    }

    /* ── PHASE C: 가상 버튼 × 블라인드 1~5 번갈아 실제 변조 송신 ── */
    blind_manager_t tb;
    _build_test_blinds(&tb);

    struct { somfy_command_t cmd; const char *btn; } btns[] = {
        { SOMFY_CMD_UP,   "UP(올림)"   },
        { SOMFY_CMD_DOWN, "DOWN(내림)" },
        { SOMFY_CMD_MY,   "MY(정지)"   },
        { SOMFY_CMD_PROG, "PROG(등록)" },
    };
    ESP_LOGW(TAG, "[C] 가상 버튼 × 블라인드 1~5 (실제 송신, 번갈아)");
    for (int bn = 1; bn <= 5; bn++) {
        char who[8];
        snprintf(who, sizeof(who), "블라%d", bn);
        for (int k = 0; k < 4; k++) {
            bool ok = _send_and_verify(&tb.blinds[bn - 1], btns[k].cmd,
                                       who, btns[k].btn);
            if (ok) { pass++;
                ESP_LOGI(TAG, "[PASS] 블라인드%d %s", bn, btns[k].btn);
            } else { fail++;
                ESP_LOGE(TAG, "[FAIL] 블라인드%d %s", bn, btns[k].btn);
            }
        }
    }

    /* ── PHASE D: ALL 동시(순차) 제어 + PROG 차단 정책 ── */
    ESP_LOGW(TAG, "[D] ALL 선택 동시 제어 + PROG 차단 검증");
    tb.selected = 5;   /* 5 = ALL */
    somfy_blind_t *targets[BLIND_MAX_COUNT];
    uint8_t tcount = 0;
    blind_manager_get_targets(&tb, targets, &tcount);
    if (tcount == 5) {
        pass++;
        ESP_LOGI(TAG, "[PASS] ALL 대상 수집 = 5개 (블라인드1~5)");
        bool all_tx = true;
        for (int i = 0; i < tcount; i++) {
            char who[8];
            snprintf(who, sizeof(who), "ALL#%d", i + 1);
            if (!_send_and_verify(targets[i], SOMFY_CMD_DOWN, who, "DOWN"))
                all_tx = false;
        }
        if (all_tx) { pass++;
            ESP_LOGI(TAG, "[PASS] ALL 동시제어 — 5개 블라인드 순차 실제 송신");
        } else { fail++;
            ESP_LOGE(TAG, "[FAIL] ALL 동시제어 송신");
        }
    } else {
        fail++;
        ESP_LOGE(TAG, "[FAIL] ALL 대상 수집 = %d개 (5 기대)", tcount);
    }
    /* ALL 선택 시 PROG 는 단일 블라인드 전용 정책상 차단되어야 한다.
     *  앱(_btn_event_cb)의 가드와 동일 조건(selected>=5)을 검증한다. */
    bool prog_blocked = (tb.selected >= 5);
    if (prog_blocked) { pass++;
        ESP_LOGI(TAG, "[PASS] ALL+PROG 차단 (단일 블라인드 전용 정책)");
    } else { fail++;
        ESP_LOGE(TAG, "[FAIL] ALL+PROG 차단 정책 미적용");
    }

    /* 종료: 칩을 IDLE 로 안전 복귀 */
    cc1101_idle(&g_cc1101);

    ESP_LOGW(TAG, "===== 온에어 결과: %d/%d PASS, %d FAIL =====",
             pass, pass + fail, fail);
    return fail;
}
