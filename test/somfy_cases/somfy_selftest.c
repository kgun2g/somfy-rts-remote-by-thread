/*
 * somfy_selftest.c — Somfy RTS 가상 테스트 (재사용). v3.5.
 * 온에어/CC1101 불필요: somfy_rts_test_build_frame() 로 프레임만 생성해
 * 커맨드 nibble / 주소 / 롤링 / 체크섬을 검증한다.
 */
#include "somfy_selftest.h"
#include "somfy_rts.h"
#include "blind_manager.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "SELFTEST";

#define NUM_BLINDS 5

/* 블라인드 1~5 가상 구성(고정값 → 결정적 검증). */
static somfy_blind_t _mk_blind(int n)
{
    somfy_blind_t b = {0};
    b.address      = 0x100000u + (uint32_t)n;   /* 1..5 → 0x100001.. */
    b.rolling_code = (uint16_t)(0x0010 + n);
    b.freq_mhz     = 447.20f;
    b.active       = true;
    snprintf(b.name, sizeof(b.name), "B%d", n);
    return b;
}

/* 한 프레임의 정합성 검증. ok 면 true. */
static bool _verify_frame(const uint8_t fobf[7], somfy_command_t cmd,
                          uint16_t rolling, uint32_t address)
{
    /* 빌더는 마지막에 난독화(frame[i]^=frame[i-1], i=1..6)를 적용한다.
     *  역난독화: i=6..1 역순으로 frame[i]^=frame[i-1] → 원본 복원. */
    uint8_t f[7];
    memcpy(f, fobf, 7);
    for (int i = 6; i >= 1; i--) f[i] ^= f[i - 1];

    bool ok = true;
    if ((f[1] >> 4) != ((uint8_t)cmd & 0x0F)) ok = false;        /* cmd nibble */
    if (f[2] != ((rolling >> 8) & 0xFF))      ok = false;        /* rolling hi */
    if (f[3] != (rolling & 0xFF))             ok = false;        /* rolling lo */
    /* 표준 Somfy RTS: MSB at frame[4], LSB at frame[6] */
    if (f[4] != ((address >> 16) & 0xFF))     ok = false;        /* addr MSB */
    if (f[5] != ((address >> 8) & 0xFF))      ok = false;        /* addr mid */
    if (f[6] != (address & 0xFF))             ok = false;        /* addr LSB */
    /* 체크섬 nibble 재계산(빌더와 동일: 난독화 전 프레임 기준) */
    uint8_t tmp[7];
    memcpy(tmp, f, 7);
    uint8_t saved_lo = tmp[1] & 0x0F;
    tmp[1] &= 0xF0;
    uint8_t cs = 0;
    for (int i = 0; i < 7; i++) cs ^= tmp[i] ^ (tmp[i] >> 4);
    if (saved_lo != (cs & 0x0F))              ok = false;
    return ok;
}

/* 커맨드 시퀀스(steps) 를 한 블라인드에 대해 가상 수행·검증.
 *  각 step 마다 롤링 1 증가(실제 송신과 동일 규칙). */
typedef struct { somfy_command_t cmd; const char *label; } step_t;

static bool _run_seq(const char *proc, int bn, const step_t *steps, int n)
{
    somfy_blind_t b = _mk_blind(bn);
    uint16_t roll = b.rolling_code;
    bool all = true;
    for (int i = 0; i < n; i++) {
        uint8_t f[10];   /* 한국 80비트 = 10바이트 프레임 (out[10]) */
        somfy_rts_test_build_frame(f, steps[i].cmd, roll, b.address);
        bool ok = _verify_frame(f, steps[i].cmd, roll, b.address);
        ESP_LOGD(TAG, "  B%d %s step%d %s cmd=0x%X roll=%u -> %s",
                 bn, proc, i, steps[i].label, steps[i].cmd, roll,
                 ok ? "ok" : "BAD");
        if (!ok) all = false;
        roll++;                       /* 송신 1회 = 롤링 +1 */
    }
    return all;
}

/* 복사(copy): src 블라인드 식별자를 dst 로 복제 → dst 가 동일 주소로
 *  전송하는지 검증(같은 모터를 제어). */
static bool _run_copy(int src_n, int dst_n)
{
    somfy_blind_t src = _mk_blind(src_n);
    somfy_blind_t dst = _mk_blind(dst_n);
    /* 복사 동작: 주소/주파수/롤링을 src 로 일치 */
    dst.address      = src.address;
    dst.freq_mhz     = src.freq_mhz;
    dst.rolling_code = src.rolling_code;
    /* 복사 = 식별자 일치(같은 모터 제어). dst 가 src 주소/롤링으로
     *  유효 프레임을 만드는지 _verify_frame(역난독화 포함)으로 검증. */
    bool ok = (dst.address == src.address) &&
              (dst.rolling_code == src.rolling_code) &&
              (dst.freq_mhz == src.freq_mhz);
    uint8_t f[10];   /* 한국 80비트 = 10바이트 프레임 (out[10]) */
    somfy_rts_test_build_frame(f, SOMFY_CMD_MY, dst.rolling_code, dst.address);
    ok = ok && _verify_frame(f, SOMFY_CMD_MY, src.rolling_code, src.address);
    return ok;
}

/* RF 스캔 로직 검증 헬퍼는 제거됨 (RF Scan 기능 제거에 따라). */

/* ─────────────────────────────────────────────────────────────
 * ALL(블록) 라우팅 검증 — blind_manager get_targets + somfy_app
 *  _do_rf_send 의 채널/ALL 포인터 판별 · position 팬아웃 · PROG 차단을
 *  실제 산출 주소(eFuse)로 검증한다. (CC1101/RF/NVS 무관)
 *  채널 수에 따라 자동: H2(3채널)=1블록, C6(5채널)=2블록.
 * ───────────────────────────────────────────────────────────── */
#define ARTS_STEP      0x2700u   /* blind_manager.c ADDR_STEP */
#define ARTS_MIDSTEP   0x27u     /* blind_manager.c ADDR_MIDSTEP */

static int      _ar_pos[BLIND_MAX_COUNT];
static int      _ar_calls[BLIND_MAX_COUNT];
static int      _ar_oob;
static uint32_t _ar_tx[BLIND_MAX_COUNT];
static int      _ar_tx_n;

static void _ar_reset(void)
{
    _ar_tx_n = 0; _ar_oob = 0;
    for (int i = 0; i < BLIND_MAX_COUNT; i++) { _ar_pos[i] = -1; _ar_calls[i] = 0; }
}

/* somfy_app.c _do_rf_send 의 타겟 순회 라우팅을 그대로 재현(송신/갱신은 기록). */
static void _ar_route(blind_manager_t *mgr, somfy_command_t cmd)
{
    somfy_blind_t *targets[BLIND_MAX_COUNT];
    uint8_t count = 0;
    blind_manager_get_targets(mgr, targets, &count);   /* ★ 실제 함수 */
    int pos = 50;
    switch (cmd) {
        case SOMFY_CMD_UP:   pos = 100; break;
        case SOMFY_CMD_DOWN: pos = 0;   break;
        case SOMFY_CMD_MY:   pos = 50;  break;
        default: break;
    }
    for (int i = 0; i < count; i++) {
        if (_ar_tx_n < BLIND_MAX_COUNT) _ar_tx[_ar_tx_n++] = targets[i]->address;
        int off = (int)(targets[i] - mgr->blinds);
        bool is_ch = (off >= 0 && off < BLIND_MAX_COUNT);
        if (is_ch) {
            _ar_pos[off] = pos; _ar_calls[off]++;
        } else {
            int g = (int)(targets[i] - mgr->all_blocks);
            int s = g * BLINDS_PER_BLOCK, e = s + BLINDS_PER_BLOCK;
            if (e > BLIND_MAX_COUNT) e = BLIND_MAX_COUNT;
            for (int k = s; k < e; k++) {
                if (k >= 0 && k < BLIND_MAX_COUNT) { _ar_pos[k] = pos; _ar_calls[k]++; }
                else _ar_oob++;
            }
        }
    }
}

/* 버튼 핸들러의 ALL+PROG 차단(somfy_app.c BTN_EVT_PROG_PRESS, selected>=5) 재현. */
static int _ar_press(blind_manager_t *mgr, somfy_command_t cmd)
{
    if (cmd == SOMFY_CMD_PROG && mgr->selected >= 5) return 0;   /* ALL 에서 PROG 차단 */
    _ar_route(mgr, cmd);
    return 1;
}

static int _all_route_tests(void)
{
    int p = 0, f = 0;
    blind_manager_t mgr;
    blind_manager_test_populate(&mgr);    /* 실제 주소 산출 + ALL 블록 (NVS 무관) */

    ESP_LOGW(TAG, "----- ALL 라우팅 검증 (채널=%d, 블록=%d) -----",
             BLIND_MAX_COUNT, BLIND_BLOCK_COUNT);
    for (int i = 0; i < BLIND_MAX_COUNT; i++)
        ESP_LOGI(TAG, "  채널%d=0x%06lX", i, (unsigned long)mgr.blinds[i].address);
    for (int g = 0; g < BLIND_BLOCK_COUNT; g++)
        ESP_LOGI(TAG, "  ALL%d =0x%06lX", g + 1, (unsigned long)mgr.all_blocks[g].address);

#define ARTS_CHK(c, m) do { if (c) p++; else { f++; ESP_LOGE(TAG, "  [FAIL] %s", (m)); } } while (0)

    /* (1) 주소 규칙: F0 prefix / 등차 0x2700 / ALL=base+4*step / base mid<0x27 */
    for (int i = 0; i < BLIND_MAX_COUNT; i++)
        ARTS_CHK(((mgr.blinds[i].address >> 16) & 0xFF) == 0xF0, "채널 prefix=F0(carry 없음)");
    for (int g = 0; g < BLIND_BLOCK_COUNT; g++) {
        int s = g * BLINDS_PER_BLOCK, e = s + BLINDS_PER_BLOCK;
        if (e > BLIND_MAX_COUNT) e = BLIND_MAX_COUNT;
        ARTS_CHK(((mgr.blinds[s].address >> 8) & 0xFF) < ARTS_MIDSTEP, "블록 base mid<0x27");
        for (int i = s + 1; i < e; i++)
            ARTS_CHK(mgr.blinds[i].address - mgr.blinds[i-1].address == ARTS_STEP, "채널 등차 0x2700");
        ARTS_CHK(mgr.all_blocks[g].address - mgr.blinds[s].address == (uint32_t)BLINDS_PER_BLOCK * ARTS_STEP,
                 "ALL=base+4*0x2700");
        ARTS_CHK(((mgr.all_blocks[g].address >> 16) & 0xFF) == 0xF0, "ALL prefix=F0(carry 없음)");
    }

    /* (2) ALL 선택 + UP/DOWN/MY → 모든 채널 position 정확히 1회 갱신 */
    const somfy_command_t btns[3] = { SOMFY_CMD_UP, SOMFY_CMD_DOWN, SOMFY_CMD_MY };
    const int exp_pos[3] = { 100, 0, 50 };
    for (int b = 0; b < 3; b++) {
        mgr.selected = 5; _ar_reset();
        int routed = _ar_press(&mgr, btns[b]);
        ARTS_CHK(routed == 1, "ALL+버튼 송신경로 진입");
        ARTS_CHK(_ar_tx_n == BLIND_BLOCK_COUNT, "ALL 송신=블록수");
        for (int g = 0; g < BLIND_BLOCK_COUNT && g < _ar_tx_n; g++)
            ARTS_CHK(_ar_tx[g] == mgr.all_blocks[g].address, "각 블록 ALL 주소로 송신");
        for (int k = 0; k < BLIND_MAX_COUNT; k++) {
            ARTS_CHK(_ar_calls[k] == 1, "각 채널 position 정확히 1회(누락/중복 없음)");
            ARTS_CHK(_ar_pos[k] == exp_pos[b], "각 채널 position 값=cmd매핑");
        }
        ARTS_CHK(_ar_oob == 0, "EP 범위초과 없음");
    }

    /* (3) ALL + PROG → 차단(송신 0) */
    mgr.selected = 5; _ar_reset();
    ARTS_CHK(_ar_press(&mgr, SOMFY_CMD_PROG) == 0, "ALL+PROG 차단(미진입)");
    ARTS_CHK(_ar_tx_n == 0, "ALL+PROG RF 송신 0회");

    /* (4) 개별 채널 선택 + 버튼 */
    for (int sel = 0; sel < BLIND_MAX_COUNT; sel++) {
        mgr.selected = (uint8_t)sel; _ar_reset();
        _ar_press(&mgr, SOMFY_CMD_UP);
        ARTS_CHK(_ar_tx_n == 1 && _ar_tx[0] == mgr.blinds[sel].address, "개별 채널 주소로 송신");
        ARTS_CHK(_ar_calls[sel] == 1 && _ar_pos[sel] == 100, "선택 채널만 position 갱신");
        int oth = 0;
        for (int k = 0; k < BLIND_MAX_COUNT; k++) if (k != sel && _ar_calls[k]) oth++;
        ARTS_CHK(oth == 0, "다른 채널 미변경");
    }
    mgr.selected = 0; _ar_reset();
    ARTS_CHK(_ar_press(&mgr, SOMFY_CMD_PROG) == 1 && _ar_tx_n == 1, "개별 채널+PROG 송신 1회");

    /* (5) 채널 vs ALL 포인터 판별 (실제 struct 레이아웃) */
    for (int g = 0; g < BLIND_BLOCK_COUNT; g++)
        ARTS_CHK((int)(&mgr.all_blocks[g] - mgr.blinds) >= BLIND_MAX_COUNT, "ALL off>=MAX → is_ch=false");

#undef ARTS_CHK
    ESP_LOGW(TAG, "----- ALL 라우팅 결과: %d PASS, %d FAIL -----", p, f);
    return f;
}

int somfy_selftest_run(void)
{
    ESP_LOGW(TAG, "===== Somfy 가상 테스트 시작 (블라인드 1~5) =====");
    int pass = 0, fail = 0;

    /* 케이스별 커맨드 시퀀스 (Somfy RTS 표준 절차 모델링) */
    const step_t reg[]   = { {SOMFY_CMD_PROG, "PROG(long)"} };
    const step_t upper[] = { {SOMFY_CMD_PROG, "PROG(prog)"},
                             {SOMFY_CMD_UP,   "UP(to-upper)"},
                             {SOMFY_CMD_MY,   "MY(set-upper)"} };
    const step_t lower[] = { {SOMFY_CMD_PROG, "PROG(prog)"},
                             {SOMFY_CMD_DOWN, "DOWN(to-lower)"},
                             {SOMFY_CMD_MY,   "MY(set-lower)"} };
    const step_t mypos[] = { {SOMFY_CMD_MY,   "MY(long-memorize)"} };

    for (int bn = 1; bn <= NUM_BLINDS; bn++) {
        struct { const char *name; bool ok; } r[5];
        r[0].name = "신규등록";  r[0].ok = _run_seq("register", bn, reg,   1);
        r[1].name = "상한설정";  r[1].ok = _run_seq("upper",    bn, upper, 3);
        r[2].name = "하한설정";  r[2].ok = _run_seq("lower",    bn, lower, 3);
        r[3].name = "복사";      r[3].ok = _run_copy(bn, (bn % NUM_BLINDS) + 1);
        r[4].name = "위치기억";  r[4].ok = _run_seq("mypos",    bn, mypos, 1);
        for (int k = 0; k < 5; k++) {
            if (r[k].ok) { pass++;
                ESP_LOGI(TAG, "[PASS] 블라인드%d %s", bn, r[k].name);
            } else { fail++;
                ESP_LOGE(TAG, "[FAIL] 블라인드%d %s", bn, r[k].name);
            }
        }
    }
    ESP_LOGW(TAG, "===== 프레임 결과: %d/%d PASS, %d FAIL =====",
             pass, pass + fail, fail);

    /* ALL(블록) 라우팅 검증 — 채널/ALL 라우팅 · PROG 차단 · 주소 규칙 */
    int route_fail = _all_route_tests();
    fail += route_fail;

    ESP_LOGW(TAG, "===== 전체 결과: %d FAIL (프레임 + ALL 라우팅) =====", fail);
    return fail;
}
