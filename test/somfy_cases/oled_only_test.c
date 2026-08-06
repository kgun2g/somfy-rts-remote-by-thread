/*
 * oled_only_test.c — OLED 단독 부팅 테스트 (2026-07-22)
 * ─────────────────────────────────────────────────────────────────────
 * 목적: OLED 를 제외한 모든 기능(Matter/Thread/BLE/CC1101 RF/PCF8574 버튼/
 *   배터리 ADC/진동/절전)을 **전혀 초기화하지 않고**, OLED I2C(22/23)만 단독
 *   구동한다. app_main 최상단에서 이 함수로 단락 → 다른 어떤 코드도 실행되지
 *   않으므로 OLED 외 나머지 핀 입력은 자동으로 무시된다.
 *
 * 활성화: build.ps1 -OledTest  → -D OLED_ONLY_TEST=1 (CMakeLists 가 이 소스 추가).
 *
 * 동작:
 *   1) I2CDIAG   : 22/23 핀 pull 특성(float/내부PD/내부PU) → 외부풀업/stuck 판별
 *   2) bit-bang  : IDF 드라이버 우회 순수 GPIO 로 0x08~0x77 스캔 → 응답주소
 *   3) FORCE     : HW I2C 로 0x3C 에 직접 write → ACK/timeout
 *   4) 패널 init + "OLED TEST" 표시
 *   5) 무한 루프 : frame 카운터/반전바 갱신 + 매 5초 write 결과 로그
 *      → OLED 정상이면 화면이 갱신되고 write=ESP_OK, 고장이면 무반응+INVALID_STATE
 * ───────────────────────────────────────────────────────────────────── */
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"
#include "board_select.h"      /* BOARD_PIN_OLED_SDA/SCL, BOARD_OLED_WIDTH/HEIGHT */

#define TAG      "OLED_ONLY"
#define OT_SDA   ((gpio_num_t)BOARD_PIN_OLED_SDA)   /* GPIO22 (D4) */
#define OT_SCL   ((gpio_num_t)BOARD_PIN_OLED_SCL)   /* GPIO23 (D5) */

/* open-drain 에뮬: v=1 릴리즈(입력, 외부풀업 HIGH) / v=0 LOW 구동 */
static inline void _bb(gpio_num_t p, int v) {
    if (v) gpio_set_direction(p, GPIO_MODE_INPUT);
    else { gpio_set_level(p, 0); gpio_set_direction(p, GPIO_MODE_OUTPUT); }
}
/* 순수 GPIO bit-bang write-probe (IDF I2C 완전 우회). true=ACK. */
static bool _bb_probe(uint8_t addr7) {
    gpio_reset_pin(OT_SDA); gpio_reset_pin(OT_SCL);
    gpio_set_level(OT_SDA, 0); gpio_set_level(OT_SCL, 0);
    _bb(OT_SDA, 1); _bb(OT_SCL, 1); esp_rom_delay_us(10);
    _bb(OT_SDA, 0); esp_rom_delay_us(5);         /* START */
    _bb(OT_SCL, 0); esp_rom_delay_us(5);
    uint8_t byte = (uint8_t)(addr7 << 1);        /* write */
    for (int i = 0; i < 8; i++) {
        _bb(OT_SDA, (byte & 0x80) ? 1 : 0); byte <<= 1; esp_rom_delay_us(3);
        _bb(OT_SCL, 1); esp_rom_delay_us(5);
        _bb(OT_SCL, 0); esp_rom_delay_us(3);
    }
    _bb(OT_SDA, 1); esp_rom_delay_us(3);         /* ACK 읽기 */
    _bb(OT_SCL, 1); esp_rom_delay_us(5);
    int ack = gpio_get_level(OT_SDA);
    _bb(OT_SCL, 0); esp_rom_delay_us(5);
    _bb(OT_SDA, 0); esp_rom_delay_us(3);         /* STOP */
    _bb(OT_SCL, 1); esp_rom_delay_us(5);
    _bb(OT_SDA, 1); esp_rom_delay_us(5);
    return ack == 0;
}

void oled_only_test(void)
{
    ESP_LOGW(TAG, "================ OLED ONLY TEST ================");
    ESP_LOGW(TAG, "OLED 단독 부팅 — Matter/Thread/BLE/RF/버튼/배터리 전부 비활성. SDA=IO%d SCL=IO%d",
             (int)OT_SDA, (int)OT_SCL);

    /* 1) 핀 pull 특성 (외부풀업 존재 / GND stuck 판별) */
    for (int i = 0; i < 2; i++) {
        gpio_num_t pin = i ? OT_SCL : OT_SDA;
        const char *nm = i ? "SCL(23)" : "SDA(22)";
        gpio_reset_pin(pin); gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin, GPIO_FLOATING);      vTaskDelay(pdMS_TO_TICKS(2)); int lf = gpio_get_level(pin);
        gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY); vTaskDelay(pdMS_TO_TICKS(2)); int pd = gpio_get_level(pin);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);   vTaskDelay(pdMS_TO_TICKS(2)); int pu = gpio_get_level(pin);
        gpio_set_pull_mode(pin, GPIO_FLOATING);
        ESP_LOGW(TAG, "[I2CDIAG] %s float=%d int-PD=%d(HIGH=외부풀업존재) int-PU=%d(LOW=GND단락)",
                 nm, lf, pd, pu);
    }

    /* 2) bit-bang 전체 스캔 (IDF 드라이버 우회) */
    char buf[100]; int n = 0; buf[0] = '\0';
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (_bb_probe(a)) {
            n += snprintf(buf + n, sizeof(buf) - n, "0x%02X ", a);
            if (n >= (int)sizeof(buf) - 6) break;
        }
    }
    ESP_LOGW(TAG, "[BITBANG] 응답주소(0x08~0x77) = %s", buf[0] ? buf : "(없음)");

    /* 3) HW I2C init + 0x3C 강제 write */
    SSD1306_t dev;
    memset(&dev, 0, sizeof(dev));
    i2c_master_init(&dev, BOARD_PIN_OLED_SDA, BOARD_PIN_OLED_SCL, -1);
    uint8_t cmd_on[2]  = { 0x00, 0xAF };   /* command stream + display ON */
    esp_err_t r = i2c_master_transmit(dev._i2c_dev_handle, cmd_on, 2, 100);
    ESP_LOGW(TAG, "[FORCE] 0x3C 직접 write = %s  (ESP_OK=모듈 ACK/살아있음, 그 외=무응답)",
             esp_err_to_name(r));

    /* 4) 응답 여부로 분기 */
    if (r == ESP_OK) {
        /* ── 모듈 살아있음 → 패널 init + 화면 테스트 루프(애니메이션 표시) ── */
        ssd1306_init(&dev, BOARD_OLED_WIDTH, BOARD_OLED_HEIGHT);
        ssd1306_contrast(&dev, 0xFF);
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, " OLED TEST", 10, false);
        ssd1306_display_text(&dev, 2, " standalone", 11, false);
        int frame = 0;
        while (1) {
            char line[20];
            snprintf(line, sizeof(line), " frame %d", frame);
            ssd1306_display_text(&dev, 4, line, (int)strlen(line), false);
            if (frame & 1) ssd1306_display_text(&dev, 6, "##########", 10, true);
            else           ssd1306_clear_line(&dev, 6, false);
            ESP_LOGW(TAG, "[LOOP] frame=%d — 화면 갱신중 (OLED 정상 동작)", frame);
            frame++;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } else {
        /* ── 모듈 무응답 → SSD1306 라이브러리 호출은 죽은 버스에서 매 write 100ms
         *  타임아웃+에러도배라 무의미. 경량 bit-bang 스캔을 2초마다 반복해 깔끔하게
         *  상태 재확인(정상 OLED 로 연결/교체하면 이 루프가 응답 감지). ── */
        ESP_LOGW(TAG, "0x3C 무응답 → 패널 init 스킵(라이브러리 에러도배 방지). bit-bang 재스캔 반복.");
        int frame = 0;
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            char sb[100]; int m = 0; sb[0] = '\0';
            for (uint8_t a = 0x08; a <= 0x77; a++) {
                if (_bb_probe(a)) {
                    m += snprintf(sb + m, sizeof(sb) - m, "0x%02X ", a);
                    if (m >= (int)sizeof(sb) - 6) break;
                }
            }
            gpio_reset_pin(OT_SDA); gpio_set_direction(OT_SDA, GPIO_MODE_INPUT);
            gpio_set_pull_mode(OT_SDA, GPIO_FLOATING); vTaskDelay(pdMS_TO_TICKS(2));
            int sda = gpio_get_level(OT_SDA);
            ESP_LOGW(TAG, "[LOOP] frame=%d 무응답 (bit-bang=%s, SDA idle=%d) — 정상 OLED 연결/교체 시 화면 시작",
                     frame, sb[0] ? sb : "(없음)", sda);
            frame++;
        }
    }
}
