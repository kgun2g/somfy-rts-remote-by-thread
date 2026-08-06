/*
 * somfy_txprobe_test.c — TX 디지털 자가진단. v3.5.
 *
 * 목적: "기기로 블라인드를 한 번도 못 움직임" 의 원인이 디지털 송신
 *  체인(프레임 빌드·RMT 출력)인지, RF/PA/안테나인지 가린다. 이미 RX
 *  디코더로 입증된 CC1101 RX 경로와 별개로, 송신 시 GD0(IO8)에서
 *  실제로 어떤 파형이 나오는지 같은 칩에서 직접 캡처해 비교한다.
 *
 * 방법:
 *  1) 고우선 폴링 태스크 생성 → 세마포어 대기
 *  2) somfy_rts_send(PROG) 호출 직전에 세마포어 signal → 폴링 시작
 *  3) RMT 가 HW 로 GD0 를 구동하는 동안(rmt_tx_wait_all_done 가 CPU 를
 *     세마포어로 블록) 폴링 태스크가 IO8 전이 시각을 µs 단위 캡처
 *  4) 캡처 종료 후 펄스폭을 Somfy RTS 표준 빈으로 분류:
 *       심볼 ~640µs, 더블 ~1280µs, HW싱크 ~2416µs, SW싱크 ~4836µs
 *  5) HW싱크 다수 + 심볼 다수 → 디지털 OK (의심: RF/PA/안테나)
 *     그렇지 않으면 → 디지털 BAD (RMT/프레임 버그)
 *
 * 송신 대상은 NVS 의 블라인드[0]. 모터가 PROG 모드가 아니면 그냥 무시되어
 * 안전. NVS·롤링코드는 로컬 복사로 보호.
 */
#include "somfy_txprobe_test.h"
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

static const char *TAG = "TXPROBE";

extern cc1101_t        g_cc1101;
extern somfy_rts_t     g_somfy;
extern blind_manager_t g_mgr;
extern bool            g_rf_ready;

#define GD0_PIN        CC1101_PIN_GD0    /* IO8 */
#define MAX_EDGES      4096
#define CAPTURE_MS     800               /* PROG=3프레임 ≈ 450ms, 여유 800 */

static uint32_t s_widths[MAX_EDGES];
static volatile int  s_n = 0;
static volatile bool s_capture_done = false;
static SemaphoreHandle_t s_start_sem = NULL;

/* 고우선 폴링 태스크 — RMT 가 HW 로 GD0 구동 시 CPU 가 IO8 전이 캡처. */
static void _poll_task(void *pv)
{
    (void)pv;
    xSemaphoreTake(s_start_sem, portMAX_DELAY);

    s_n = 0;
    int last = gpio_get_level(GD0_PIN);
    int64_t t0 = esp_timer_get_time();
    int64_t tprev = t0;
    int64_t until = t0 + (int64_t)CAPTURE_MS * 1000;

    /* FreeRTOS 양보 없는 타이트 폴링 — single core, 본 태스크가 최고
     *  우선순위라 다른 태스크는 깨지 않음(somfy_rts 는 RMT done 세마포
     *  에서 블록 중). RMT IRQ 는 정상 처리됨(IRQ 가 태스크 우선순위 위). */
    while (esp_timer_get_time() < until && s_n < MAX_EDGES) {
        int lv = gpio_get_level(GD0_PIN);
        if (lv != last) {
            int64_t now = esp_timer_get_time();
            uint32_t w = (uint32_t)(now - tprev);
            tprev = now;
            last = lv;
            if (w >= 50 && w <= 60000) s_widths[s_n++] = w;
        }
    }
    s_capture_done = true;
    vTaskDelete(NULL);
}

void somfy_txprobe_test_run(void)
{
    ESP_LOGW(TAG, "===== TX 디지털 자가진단 시작 =====");
    if (!g_rf_ready) {
        ESP_LOGE(TAG, "CC1101 미준비 — 진단 불가");
        return;
    }

    /* 대상 블라인드: g_mgr.blinds[0] 복사(NVS 보호). 없으면 합성. */
    somfy_blind_t tb;
    if (g_mgr.count >= 1 && g_mgr.blinds[0].active) {
        tb = g_mgr.blinds[0];
    } else {
        memset(&tb, 0, sizeof(tb));
        tb.address      = 0x100001u;
        tb.rolling_code = 0x0010;
        tb.freq_mhz     = 447.60f;
        tb.active       = true;
        snprintf(tb.name, sizeof(tb.name), "TXP");
    }

    /* RMT 출력 매트릭스 결합을 유지한 채 입력 기능만 켜기.
     *  gpio_set_direction(INPUT_OUTPUT)은 GPIO 방향/매트릭스를 리셋할
     *  수 있어 RMT 출력이 핀에 도달하지 않게 되는 부작용이 관찰됨
     *  → gpio_input_enable 로 FUN_IE 만 켠다(매트릭스 그대로). */
    gpio_input_enable(GD0_PIN);

    s_start_sem    = xSemaphoreCreateBinary();
    s_capture_done = false;
    s_n            = 0;

    /* 폴링 태스크 prio = 2 (호출 태스크 5 보다 낮음).
     *  호출 태스크가 somfy_rts_send 의 rmt_tx_wait_all_done 세마포에서
     *  블록될 때마다 폴링이 끼어들어 IO8 전이를 캡처(single core C6).
     *  이전 최고 prio 였을 때는 폴링이 600ms 풀가동 후에야 송신이 시작됨. */
    xTaskCreate(_poll_task, "txpoll", 3072, NULL, 2, NULL);

    ESP_LOGW(TAG, "[%s] PROG 송신 + GD0 캡처 시작 (freq=%.2fMHz, addr=0x%06lX)",
             tb.name, tb.freq_mhz, (unsigned long)tb.address);

    /* 송신 시작 직전 폴링 unblock — RMT 가 GD0 를 구동하는 전 구간 캡처. */
    xSemaphoreGive(s_start_sem);
    somfy_rts_send(&g_somfy, &tb, SOMFY_CMD_PROG, 0);

    /* 캡처 완료 대기(최대 2s) */
    int waited = 0;
    while (!s_capture_done && waited < 2000) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited += 50;
    }

    /* ── 분석: Somfy RTS 표준 펄스 빈 분류 ── */
    int b_sym = 0, b_dbl = 0, b_hw = 0, b_sw = 0, b_other = 0;
    uint32_t mn = 0xFFFFFFFFU, mx = 0;
    for (int i = 0; i < s_n; i++) {
        uint32_t w = s_widths[i];
        if (w < mn) mn = w;
        if (w > mx) mx = w;
        if      (w >= 420  && w <= 880)  b_sym++;   /* 심볼 ~640µs */
        else if (w >= 900  && w <= 1700) b_dbl++;   /* 더블 ~1280µs */
        else if (w >= 1700 && w <= 3200) b_hw++;    /* HW싱크 ~2416µs */
        else if (w >= 3600 && w <= 6200) b_sw++;    /* SW싱크 ~4836µs */
        else b_other++;
    }
    bool digital_ok = (b_hw >= 4 && b_sym >= 20);

    ESP_LOGW(TAG, "[결과] edges=%d sym=%d dbl=%d HW=%d SW=%d etc=%d min=%luµs max=%luµs",
             s_n, b_sym, b_dbl, b_hw, b_sw, b_other,
             (unsigned long)(s_n ? mn : 0), (unsigned long)mx);
    if (digital_ok) {
        ESP_LOGW(TAG, "→ ★ 디지털 출력 = Somfy RTS 사양 일치 (RMT/프레임 OK)");
        ESP_LOGW(TAG, "→ 의심 좁혀짐: RF 측(CC1101 PA·OOK 변조·안테나)");
    } else {
        ESP_LOGE(TAG, "→ ★ 디지털 출력 = Somfy 사양 미일치 (RMT/프레임 버그)");
    }

    /* 첫 40개 펄스폭 덤프 — 시각적 점검용(첫 프레임의 wake+HW싱크 패턴) */
    ESP_LOGW(TAG, "[펄스폭 덤프] 첫 40개(µs):");
    for (int i = 0; i < s_n && i < 40; i++) {
        ESP_LOGW(TAG, "  [%2d] %5luµs", i, (unsigned long)s_widths[i]);
    }

    ESP_LOGW(TAG, "===== TX 자가진단 종료 (이후 대기) =====");
    while (1) vTaskDelay(pdMS_TO_TICKS(60000));   /* 로그 읽도록 유지 */
}
