/*
 * somfy_cwtest_test.c — CW(순수 carrier) 송신 테스트.
 *
 * 목적: 우리 송신 신호가 SDR 워터폴에서 가로로 넓게 번지는 원인이
 *  (A) crystal/FS chirp(하드웨어) 인지 (B) OOK 변조 splatter(설정) 인지
 *  최종 판별. OOK 변조를 완전히 빼고 carrier 만 연속 송신한다.
 *
 * 방법: CC1101 을 TX state 로 두고 GD0 핀을 GPIO 출력 HIGH 로 고정 →
 *  async serial 모드에서 GD0=HIGH = carrier 연속 ON (unmodulated CW).
 *  RMT 매트릭스 결합을 끊고 순수 GPIO 로 GD0 를 HIGH 고정.
 *
 * 판정(SDR 로 447.72MHz 부근 관찰):
 *  - carrier 가 좁은 한 점(세로 직선) → carrier 안정. 번짐은 OOK 변조
 *    splatter → DRATE/변조 설정 튜닝으로 개선 가능.
 *  - carrier 가 가로로 번짐/흔들림 → crystal/FS chirp = 하드웨어(E07
 *    모듈 crystal) 문제 → 모듈 교체 또는 calibration 보정 필요.
 *
 * 사용: ./test/build_test.ps1 -Action build -Mode cwtest 후 flash.
 *  ON 8초 / OFF 3초 반복(SDR 관찰 + 과도 송신 방지).
 */
#include "somfy_cwtest_test.h"
#include "cc1101.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

static const char *TAG = "CWTEST";

extern cc1101_t g_cc1101;
extern bool     g_rf_ready;

#define GD0_PIN  CC1101_PIN_GD0

void somfy_cwtest_test_run(void)
{
    ESP_LOGW(TAG, "===== CW(순수 carrier) 송신 테스트 =====");
    if (!g_rf_ready) {
        ESP_LOGE(TAG, "CC1101 미준비 — CW 불가");
        return;
    }

    const float freq = 447.72f;   /* 설정 447.72 → 실측 ~447.678 (crystal 보정) */
    ESP_LOGW(TAG, "447.72MHz CW carrier 연속 송신 — SDR 로 carrier 안정성 확인");
    ESP_LOGW(TAG, "  좁은 한 점 = carrier 안정(번짐은 OOK 변조 탓)");
    ESP_LOGW(TAG, "  가로 번짐  = crystal/FS chirp(하드웨어)");
    ESP_LOGW(TAG, "  ON 8초 / OFF 3초 반복");

    cc1101_set_frequency(&g_cc1101, freq);

    /* GD0 를 RMT 매트릭스에서 분리하고 순수 GPIO 출력으로 — carrier 를
     *  CPU 가 직접 HIGH/LOW 제어. */
    esp_rom_gpio_connect_out_signal(GD0_PIN, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction(GD0_PIN, GPIO_MODE_OUTPUT);

    uint32_t cycle = 0;
    while (1) {
        /* carrier ON: GD0=HIGH 고정 + TX state 진입 */
        gpio_set_level(GD0_PIN, 1);
        cc1101_enter_tx_mode(&g_cc1101);
        gpio_set_level(GD0_PIN, 1);   /* enter_tx_mode 내 strobe 후 재확정 */
        ESP_LOGW(TAG, "[%u] CW ON (8초) — SDR 447.72MHz 관찰", (unsigned)++cycle);
        vTaskDelay(pdMS_TO_TICKS(8000));

        /* carrier OFF */
        gpio_set_level(GD0_PIN, 0);
        cc1101_idle(&g_cc1101);
        ESP_LOGW(TAG, "[%u] CW OFF (3초)", (unsigned)cycle);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
