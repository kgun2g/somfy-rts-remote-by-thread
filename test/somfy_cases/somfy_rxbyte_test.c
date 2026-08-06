/*
 * somfy_rxbyte_test.c — 정품 Somfy RTS 리모컨 프레임 풀 디코더. v3.5.
 *
 * 목적: 정품 Somfy 코리아 리모컨이 실제로 송신하는 7바이트 RTS 프레임을
 *  바이트 단위로 캡처·디코드해, 우리 펌웨어가 생성하는 프레임과 정확히
 *  비교한다. 이로써 PROG 복사 실패 원인이:
 *    (A) 우리 프레임 포맷 자체 오류  →  실제 리모컨 바이트와 다르면 확정
 *    (B) RF 송신 미도달            →  바이트는 같은데도 안 되면 확정
 *  중 어느 쪽인지 결정한다.
 *
 * 방법:
 *  1) CC1101 RX OOK at 447.60 MHz, IOCFG0=0x0D(복조 시리얼 데이터 출력)
 *  2) GD0(IO8) 입력 활성, 타이트 폴링으로 펄스 폭(µs) 캡처
 *  3) SW 싱크 패턴(4836µs HIGH + 1280µs LOW) 검출
 *  4) 이후 56 Manchester 비트 디코드(640µs half-symbol 기준, G.E. Thomas:
 *     '1'=HIGH-LOW, '0'=LOW-HIGH → first-half 의 레벨 = data bit)
 *  5) MSB-first 로 7바이트 조립
 *  6) 역난독화: i=6..1 에 대해 frame[i] ^= frame[i-1]
 *  7) 의미 해석 후 로그:
 *     key=frame[0], cmd=frame[1]>>4, cks=frame[1]&0xF,
 *     roll=frame[2..3] (big-endian), addr=frame[4..6] (MSB-first)
 *
 * 사용: ./test/build_test.ps1 -Action build -Mode rxbyte 후 flash.
 *  부팅 후 무한 루프. 사용자가 정품 리모컨 버튼(UP/DOWN/MY/PROG long)을
 *  기기 옆 10~20cm 에서 짧게 누름. 각 누름마다 한 줄로 디코드 결과 출력.
 */
#include "somfy_rxbyte_test.h"
#include "cc1101.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

static const char *TAG = "RXBYTE";

extern cc1101_t g_cc1101;
extern bool     g_rf_ready;

#define GD0_PIN              CC1101_PIN_GD0   /* IO8 */
#define MAX_EDGES            8192
#define CAP_MS               2500             /* burst 전체 (4~5 frames) 한 캡처에 */
#define CC1101_IOCFG0_REG    0x02
#define FREQ_MHZ             447.60f
#define SYMBOL_US            640
#define HW_SYNC_US           2416
#define SW_SYNC_HI_US        4836
#define SW_SYNC_LO_US        1280
#define TOL_PCT              30               /* 빈 분류 허용 오차 % */

/* 한 펄스 = (level, duration_us). level: 0=LOW, 1=HIGH */
typedef struct { uint8_t lv; uint32_t dur; } pulse_t;

static pulse_t s_pulses[MAX_EDGES];
static int     s_n = 0;

/* GD0 핀 RX 활성:
 *  cc1101_init() 가 GD0 를 GPIO_MODE_OUTPUT 으로 잡고 LOW 드라이브 중 →
 *  CC1101 칩이 GD0 로 출력하는 시리얼 데이터가 ESP32 출력 드라이버에 묻힘.
 *  ① GPIO matrix RMT/기타 출력 결합 해제(SIG_GPIO_OUT_IDX 로 묶음)
 *  ② direction = INPUT (출력 driver off, input enable)
 *  ③ pull 해제 (CC1101 push-pull 출력과 충돌 방지) */
static void _gd0_input_enable(void) {
    esp_rom_gpio_connect_out_signal(GD0_PIN, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction(GD0_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_dis(GD0_PIN);
    gpio_pullup_dis(GD0_PIN);
}

/* 타이트 폴링으로 GD0 펄스 캡처 — RX 가 활성인 동안 호출 */
static int _capture_pulses(int cap_ms)
{
    s_n = 0;
    int last = gpio_get_level(GD0_PIN);
    int64_t t0 = esp_timer_get_time();
    int64_t tprev = t0;
    int64_t until = t0 + (int64_t)cap_ms * 1000;
    while (esp_timer_get_time() < until && s_n < MAX_EDGES) {
        int lv = gpio_get_level(GD0_PIN);
        if (lv != last) {
            int64_t now = esp_timer_get_time();
            uint32_t w = (uint32_t)(now - tprev);
            tprev = now;
            if (w >= 80 && w <= 60000) {
                s_pulses[s_n].lv = (uint8_t)last;   /* 직전 구간의 레벨 */
                s_pulses[s_n].dur = w;
                s_n++;
            }
            last = lv;
        }
    }
    return s_n;
}

/* CC1101 OOK RX 진입 (FREQ_MHZ 로 튜닝, IOCFG0=0x0D) */
static void _cc1101_rx_on(float mhz)
{
    cc1101_set_frequency(&g_cc1101, mhz);
    cc1101_strobe(&g_cc1101, CC1101_SIDLE);
    cc1101_write_reg(&g_cc1101, CC1101_IOCFG0_REG, 0x0D);
    cc1101_strobe(&g_cc1101, CC1101_SRX);
    for (int i = 0; i < 20 &&
         (cc1101_get_status(&g_cc1101) & 0x70) != CC1101_STATUS_RX; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(30));    /* AGC 안정 */
}

/* OOK 출력 극성. CC1101 IOCFG0=0x0D 가 active-high 가 정상이지만
 *  관측된 idle GD0=1 → 일부 모듈/설정에서 반전됨. 처음 디코드 성공한
 *  극성을 기억해 다음 캡처는 그쪽 먼저 시도(빠른 path). */
static int s_polarity = -1;   /* -1=미정, 1=active-high, 0=active-low */

/* SW 싱크 패턴 위치 찾기(주어진 극성에서):
 *  active 레벨로 ~4836µs 다음에 idle 레벨로 ~1280µs.  못 찾으면 -1. */
static int _find_sw_sync_end_pol(int active_lv)
{
    int idle_lv = active_lv ? 0 : 1;
    for (int i = 0; i < s_n - 1; i++) {
        if (s_pulses[i].lv != active_lv) continue;
        uint32_t a = s_pulses[i].dur;
        if (a < 3500 || a > 6300) continue;       /* SW active ~4836 */
        if (s_pulses[i + 1].lv != idle_lv) continue;
        uint32_t b = s_pulses[i + 1].dur;
        if (b < 800 || b > 1900) continue;        /* SW idle ~1280 */
        return i + 2;                              /* 데이터 시작 */
    }
    return -1;
}

/* 정극성과 반전 양쪽 시도. 성공한 극성을 *out_active_lv 로 반환. */
static int _find_sw_sync_end(int *out_active_lv)
{
    int order[2];
    if (s_polarity == 0)      { order[0] = 0; order[1] = 1; }
    else                       { order[0] = 1; order[1] = 0; }
    for (int k = 0; k < 2; k++) {
        int idx = _find_sw_sync_end_pol(order[k]);
        if (idx >= 0) {
            *out_active_lv = order[k];
            s_polarity = order[k];
            return idx;
        }
    }
    return -1;
}

/* 펄스 폭을 640µs 단위 half-symbol 수로 반올림(허용오차 ±SYMBOL/2). */
static int _halves(uint32_t dur)
{
    if (dur < SYMBOL_US / 2) return 0;
    return (int)((dur + SYMBOL_US / 2) / SYMBOL_US);
}

/* 디코드 결과 출력 + 의미 해석 */
static void _print_frame(const uint8_t *raw, const uint8_t *deob)
{
    ESP_LOGW(TAG, "원시 7바이트  : %02X %02X %02X %02X %02X %02X %02X",
             raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]);
    ESP_LOGW(TAG, "역난독 7바이트: %02X %02X %02X %02X %02X %02X %02X",
             deob[0], deob[1], deob[2], deob[3], deob[4], deob[5], deob[6]);

    uint8_t key = deob[0];
    uint8_t cmd = deob[1] >> 4;
    uint16_t roll = ((uint16_t)deob[2] << 8) | deob[3];
    uint32_t addr_msb_first = ((uint32_t)deob[4] << 16) |
                              ((uint32_t)deob[5] << 8)  | deob[6];
    uint32_t addr_lsb_first = ((uint32_t)deob[6] << 16) |
                              ((uint32_t)deob[5] << 8)  | deob[4];

    /* 체크섬 재계산 */
    uint8_t tmp[7]; memcpy(tmp, deob, 7);
    uint8_t saved_lo = tmp[1] & 0x0F;
    tmp[1] &= 0xF0;
    uint8_t cs = 0;
    for (int i = 0; i < 7; i++) cs ^= tmp[i] ^ (tmp[i] >> 4);
    cs &= 0x0F;
    bool cks_ok = (saved_lo == cs);

    const char *cmd_name = "?";
    switch (cmd) {
        case 0x1: cmd_name = "MY/STOP"; break;
        case 0x2: cmd_name = "UP";      break;
        case 0x4: cmd_name = "DOWN";    break;
        case 0x8: cmd_name = "PROG";    break;
        case 0xC: cmd_name = "SUN/FLAG"; break;
    }
    ESP_LOGW(TAG, "  key=0x%02X (표준=0xA7 %s)  cmd=0x%X (%s)  cks=0x%X (재계산=0x%X %s)",
             key, key == 0xA7 ? "OK" : "≠",
             cmd, cmd_name,
             saved_lo, cs, cks_ok ? "OK" : "≠");
    ESP_LOGW(TAG, "  rolling=%u (0x%04X)", roll, roll);
    ESP_LOGW(TAG, "  주소 MSB-first(표준): 0x%06lX",
             (unsigned long)addr_msb_first);
    ESP_LOGW(TAG, "  주소 LSB-first(역해석): 0x%06lX",
             (unsigned long)addr_lsb_first);
}

/* 한 번 캡처해서 디코드 시도. 성공=true. */
static bool _try_decode_once(void)
{
    int n = _capture_pulses(CAP_MS);
    if (n < 80) return false;

    int active_lv = 1;
    int data_start = _find_sw_sync_end(&active_lv);
    if (data_start < 0) return false;

    /* 데이터 영역의 펄스를 half-symbol 시퀀스로 펼침. 한 frame 의
     *  종료(>=3000µs gap) 만나면 중단해, 다음 frame 의 SW sync 와 혼동 X.
     *  레벨은 active_lv 를 '1', 그 반대를 '0' 으로 정규화. */
    static uint8_t halves[256];   /* up to 128 bits */
    int h = 0;
    for (int i = data_start; i < s_n && h < (int)sizeof(halves); i++) {
        if (s_pulses[i].dur > 3000) break;   /* gap → frame 끝 */
        int nh = _halves(s_pulses[i].dur);
        uint8_t norm_lv = (s_pulses[i].lv == active_lv) ? 1 : 0;
        for (int k = 0; k < nh && h < (int)sizeof(halves); k++) {
            halves[h++] = norm_lv;
        }
    }
    int total_bits = h / 2;
    if (total_bits < 56) return false;

    /* G.E. Thomas Manchester: data bit = first half of bit (even index).
     *  최대 128bit 까지 디코드 가능(8바이트=64bit 지원 + 여유). */
    uint8_t bits[128];
    int nbits = (total_bits > 128) ? 128 : total_bits;
    for (int b = 0; b < nbits; b++) bits[b] = halves[b * 2];

    /* nbits → byte 단위(MSB-first per byte). 56bit=7B, 64bit=8B 등 가능. */
    int nbytes = nbits / 8;
    if (nbytes > 16) nbytes = 16;
    uint8_t raw[16] = {0};
    for (int B = 0; B < nbytes; B++) {
        for (int b = 0; b < 8; b++) {
            if (bits[B * 8 + b]) raw[B] |= 1 << (7 - b);
        }
    }
    /* 표준 7B 디코드 */
    uint8_t deob[7]; memcpy(deob, raw, 7);
    for (int i = 6; i >= 1; i--) deob[i] ^= deob[i - 1];

    _print_frame(raw, deob);

    /* ★ 캡처 윈도우 안의 펄스 timing 통계 — wake/HW/SW/symbol 분포.
     *  정품 burst 패턴 분석용: 매 frame 마다 wake 가 있는지, HW sync 개수,
     *  SW sync 개수(=캡처 안의 frame 수), inter-frame gap 평균. */
    int n_wake_hi = 0, n_wake_lo = 0, n_hw = 0, n_sw_hi = 0, n_sw_lo = 0, n_sym = 0;
    int sw_indices[16]; int sw_cnt = 0;
    for (int i = 0; i < s_n; i++) {
        uint32_t w = s_pulses[i].dur;
        uint8_t  lv = s_pulses[i].lv;
        int active_lv2 = (s_polarity == 0) ? 0 : 1;
        if (w >= 8000 && w <= 11000 && lv == active_lv2)  n_wake_hi++;
        if (w >= 80000)                                   n_wake_lo++;
        if (w >= 2100 && w <= 2700)                       n_hw++;
        if (w >= 3800 && w <= 6000 && lv == active_lv2) {
            n_sw_hi++;
            if (sw_cnt < 16) sw_indices[sw_cnt++] = i;
        }
        if (w >= 800 && w <= 1700 && lv != active_lv2)    n_sw_lo++;
        if (w >= 500 && w <= 800)                         n_sym++;
    }
    /* SW sync 들 사이 시간 간격 (inter-frame gap) */
    char gaps[120] = {0}; int p = 0;
    for (int k = 1; k < sw_cnt && p < (int)sizeof(gaps) - 12; k++) {
        uint64_t dt = 0;
        for (int q = sw_indices[k-1]; q < sw_indices[k]; q++) dt += s_pulses[q].dur;
        p += snprintf(gaps + p, sizeof(gaps) - p, "%lums ", (unsigned long)(dt / 1000));
    }
    ESP_LOGW(TAG, "  [펄스] 총=%d wake_hi=%d wake_lo=%d HW=%d SW=%d (sym=%d) frame수=%d 간격=[%s]",
             s_n, n_wake_hi, n_wake_lo, n_hw, n_sw_hi, n_sym, sw_cnt, gaps);

    /* ★★ frame 별 진입 펄스 dump — 각 SW sync 직전 25 펄스. frame 사이가
     *  silence(긴 LOW = inter-frame gap)인지 HW sync 펄스 연속인지 확인.
     *  형식: "lv:dur lv:dur ...". */
    for (int k = 0; k < sw_cnt && k < 5; k++) {
        int prev = (k == 0) ? 0 : sw_indices[k - 1] + 1;
        int start = sw_indices[k] - 25;
        if (start < prev) start = prev;
        char buf[420]; int p = 0;
        uint32_t max_low = 0;
        for (int q = start; q < sw_indices[k] && p < 400; q++) {
            p += snprintf(buf + p, sizeof(buf) - p, "%d:%lu ",
                          s_pulses[q].lv, (unsigned long)s_pulses[q].dur);
            if (s_pulses[q].lv == 0 && s_pulses[q].dur > max_low)
                max_low = s_pulses[q].dur;
        }
        ESP_LOGW(TAG, "    frame%d 진입(최대LOW=%luus): %s",
                 k, (unsigned long)max_low, buf);
    }

    if (nbits > 56) {
        uint8_t cmd_nib = deob[1] >> 4;
        char extra_bits[80];
        int p = 0;
        for (int b = 56; b < nbits && p < (int)sizeof(extra_bits) - 1; b++) {
            extra_bits[p++] = bits[b] ? '1' : '0';
        }
        extra_bits[p] = 0;
        ESP_LOGW(TAG, "  ★확장★ 총 %dbit (=%dB) cmd=0x%X  56bit 이후 %d bits: %s",
                 nbits, nbytes, cmd_nib, nbits - 56, extra_bits);
        /* 8바이트 가설: raw[7] 도 출력 + 7바이트 XOR chain 연속해 deob[7] 계산 */
        if (nbytes >= 8) {
            uint8_t deob7 = raw[7] ^ raw[6];   /* forward XOR chain 연속 */
            ESP_LOGW(TAG, "  ★8B 가설★ raw[7]=0x%02X (deob[7] 추정=0x%02X)",
                     raw[7], deob7);
        }
    }
    return true;
}

void somfy_rxbyte_test_run(void)
{
    ESP_LOGW(TAG, "===== 정품 Somfy 리모컨 7바이트 디코더 (447.60MHz) =====");
    if (!g_rf_ready) {
        ESP_LOGE(TAG, "CC1101 미준비 — 디코더 불가");
        return;
    }
    _cc1101_rx_on(FREQ_MHZ);
    _gd0_input_enable();
    ESP_LOGW(TAG, "사용자: 정품 리모컨 버튼을 기기 옆 10~20cm 에서 짧게 누름");
    ESP_LOGW(TAG, "  → 누름 1회당 한 줄로 7바이트 + 의미 출력");
    ESP_LOGW(TAG, "표준 RTS: key=0xA7, cmd=0x8(PROG)/0x2(UP)/0x4(DOWN)/0x1(MY)");

    uint32_t pass = 0;
    uint32_t loops = 0;
    int64_t  next_diag_us = esp_timer_get_time() + 2000000;  /* 2초마다 진단 */
    while (1) {
        bool ok = _try_decode_once();
        loops++;
        if (ok) {
            pass++;
            ESP_LOGW(TAG, "  ── (누적 디코드 성공: %u회) ──", (unsigned)pass);
        }
        /* 2초마다 진단 한 줄: 캡처 펄스 수 / SW sync 가장 근접 후보 / GD0 raw level */
        if (esp_timer_get_time() >= next_diag_us) {
            next_diag_us = esp_timer_get_time() + 2000000;
            /* 최근 캡처 s_pulses 에서 가장 긴 HIGH/LOW 펄스 검색 */
            uint32_t max_hi = 0, max_lo = 0;
            int sync_h = 0, sync_l = 0;
            for (int i = 0; i < s_n; i++) {
                if (s_pulses[i].lv == 1 && s_pulses[i].dur > max_hi) max_hi = s_pulses[i].dur;
                if (s_pulses[i].lv == 0 && s_pulses[i].dur > max_lo) max_lo = s_pulses[i].dur;
                if (i + 1 < s_n && s_pulses[i].lv == 1 && s_pulses[i].dur > 3500
                    && s_pulses[i].dur < 6300 && s_pulses[i + 1].lv == 0
                    && s_pulses[i + 1].dur > 800 && s_pulses[i + 1].dur < 1900) sync_h++;
                if (i + 1 < s_n && s_pulses[i].lv == 0 && s_pulses[i].dur > 3500
                    && s_pulses[i].dur < 6300 && s_pulses[i + 1].lv == 1
                    && s_pulses[i + 1].dur > 800 && s_pulses[i + 1].dur < 1900) sync_l++;
            }
            int gd0 = gpio_get_level(GD0_PIN);
            uint8_t stat = cc1101_get_status(&g_cc1101);
            ESP_LOGI(TAG, "[진단] loops=%u pulses=%d maxH=%uus maxL=%uus syncH=%d syncL=%d pol=%d GD0=%d st=0x%02X",
                     (unsigned)loops, s_n, (unsigned)max_hi, (unsigned)max_lo,
                     sync_h, sync_l, s_polarity, gd0, stat);
            loops = 0;
            /* RX FIFO overflow 면 재시작 */
            if ((stat & 0x70) == CC1101_STATUS_RXFIFO_OVF) {
                cc1101_strobe(&g_cc1101, CC1101_SIDLE);
                cc1101_strobe(&g_cc1101, CC1101_SRX);
                ESP_LOGW(TAG, "[진단] RXFIFO OVF → RX 재시작");
            }
        }
        if (!ok) vTaskDelay(pdMS_TO_TICKS(20));
    }
}
