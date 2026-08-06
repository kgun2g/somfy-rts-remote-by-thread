/*
 * somfy_rxdecode_test.c — CC1101 RX OOK 펄스 디코더. v3.5.
 *
 * 목적(확정 진단): 정품 Somfy RTS 리모컨의 복조된 OOK 펄스 폭을 직접
 *  측정해, 어느 "프로그램 주파수" 에서 표준 RTS 파형이 깨끗이 잡히는지
 *  결정한다. RX/TX 가 동일 CC1101 이므로, 깨끗이 디코드되는 프로그램
 *  주파수를 그대로 TX 에 쓰면 모터와 일치한다.
 *
 * 방법:
 *  - CC1101 을 후보 주파수마다 OOK 비동기 RX 로 두고 IOCFG0=0x0D(=복조
 *    시리얼 데이터 출력)로 설정 → GD0(IO8)에 슬라이스된 OOK 비트가 나옴.
 *  - GD0 를 GPIO 입력으로 빠르게 폴링(esp_timer µs)하며 엣지 간격(펄스
 *    폭)을 기록.
 *  - 표준 Somfy RTS 타이밍 빈으로 분류:
 *      심볼 ~640µs, 더블 ~1280µs, HW싱크 ~2416µs, SW싱크 ~4836µs.
 *    HW싱크+심볼이 다수 잡히는 주파수 = 정답(프로그램값).
 *
 * 사용: ./test/build_test.ps1 -Action build -Mode rxdecode 후 flash.
 *  부팅 후 사용자에게 리모컨 UP/DOWN(연속 송신) 을 기기 옆에서 길게
 *  눌러달라고 요청. 송신 안 함, NVS/RF 설정 불변.
 */
#include "somfy_rxdecode_test.h"
#include "cc1101.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "RXDEC";

extern cc1101_t g_cc1101;
extern bool     g_rf_ready;

/* 후보 프로그램 주파수(MHz). 433.42=표준 RTS, 447.x=스캐너가 RSSI 로
 *  본 대역(주파수 매핑 어긋남 검증). */
static const float k_freqs[] = {
    433.42f, 433.92f, 447.55f, 447.60f, 447.65f, 447.70f, 447.75f,
};
#define NUM_FREQS (sizeof(k_freqs) / sizeof(k_freqs[0]))

#define CC1101_IOCFG0_REG   0x02
#define GD0_PIN             CC1101_PIN_GD0
#define CAP_US              350000   /* 캡처 창 0.35s (RTS 프레임 다수) */
#define MAX_EDGES           4096

static uint32_t s_w[MAX_EDGES];      /* 펄스 폭(µs) */

/* GD0 를 RMT/peripheral 에서 떼어내 순수 GPIO 입력으로.
 *  gpio_reset_pin 이 출력 매트릭스(RMT) 연결을 끊고 GPIO 입력으로 복귀. */
static void _gd0_as_input(void)
{
    gpio_reset_pin(GD0_PIN);
    gpio_set_direction(GD0_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GD0_PIN, GPIO_FLOATING);
}

/* 한 주파수: OOK RX 진입 → GD0 펄스폭 캡처 → RTS 빈 분류. */
static void _decode_one(float mhz)
{
    cc1101_set_frequency(&g_cc1101, mhz);
    cc1101_strobe(&g_cc1101, CC1101_SIDLE);
    cc1101_write_reg(&g_cc1101, CC1101_IOCFG0_REG, 0x0D); /* 복조 데이터 출력 */
    cc1101_strobe(&g_cc1101, CC1101_SRX);
    for (int i = 0; i < 20 &&
         (cc1101_get_status(&g_cc1101) & 0x70) != CC1101_STATUS_RX; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    _gd0_as_input();
    vTaskDelay(pdMS_TO_TICKS(20));        /* AGC 안정 */

    /* 타이트 폴링 — FreeRTOS 양보 없이 µs 엣지 캡처. */
    int n = 0;
    int last = gpio_get_level(GD0_PIN);
    int64_t t0 = esp_timer_get_time();
    int64_t tprev = t0;
    while ((esp_timer_get_time() - t0) < CAP_US && n < MAX_EDGES) {
        int lv = gpio_get_level(GD0_PIN);
        if (lv != last) {
            int64_t now = esp_timer_get_time();
            uint32_t w = (uint32_t)(now - tprev);
            tprev = now;
            last = lv;
            if (w > 50 && w < 60000) s_w[n++] = w;   /* 50µs~60ms 유효 */
        }
    }
    cc1101_idle(&g_cc1101);

    /* RTS 표준 타이밍 빈 분류(±35% 허용). */
    int b_sym = 0, b_dbl = 0, b_hw = 0, b_sw = 0, b_other = 0;
    uint32_t mn = 0xFFFFFFFF, mx = 0;
    for (int i = 0; i < n; i++) {
        uint32_t w = s_w[i];
        if (w < mn) mn = w;
        if (w > mx) mx = w;
        if      (w >= 420  && w <= 880)  b_sym++;   /* 심볼 640 */
        else if (w >= 900  && w <= 1700) b_dbl++;   /* 더블 1280 */
        else if (w >= 1700 && w <= 3200) b_hw++;    /* HW싱크 2416 */
        else if (w >= 3600 && w <= 6200) b_sw++;    /* SW싱크 4836 */
        else b_other++;
    }
    bool rts_like = (b_hw >= 4 && b_sym >= 20);
    ESP_LOGW(TAG,
        "  %.2fMHz edges=%-4d sym=%-3d dbl=%-3d HW=%-2d SW=%-2d etc=%-3d "
        "min=%lu max=%lu%s",
        mhz, n, b_sym, b_dbl, b_hw, b_sw, b_other,
        (unsigned long)(n ? mn : 0), (unsigned long)mx,
        rts_like ? "  <==== RTS 디코드!" : "");
}

void somfy_rxdecode_test_run(void)
{
    ESP_LOGW(TAG, "===== CC1101 RX OOK Somfy RTS 디코더 =====");
    if (!g_rf_ready) {
        ESP_LOGE(TAG, "CC1101 미준비 — 디코드 불가");
        return;
    }
    ESP_LOGW(TAG, "정품 Somfy 리모컨 UP/DOWN(연속송신) 을 기기 옆 10~20cm");
    ESP_LOGW(TAG, "에서 한 스윕(~%ds) 내내 꾹 누르세요.", (int)(NUM_FREQS));
    ESP_LOGW(TAG, "RTS 정상: HW싱크(~2416µs) 다수 + 심볼(~640µs) 다수.");

    uint32_t sweep = 0;
    while (1) {
        ESP_LOGW(TAG, "----- 스윕 #%u (리모컨 누르는 중?) -----",
                 (unsigned)(++sweep));
        for (int i = 0; i < (int)NUM_FREQS; i++) {
            _decode_one(k_freqs[i]);
        }
    }
}
