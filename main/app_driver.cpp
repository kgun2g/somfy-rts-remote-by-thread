/*
 * app_driver.cpp — Matter attribute/identify 드라이버 글루.
 * Somfy RTS 블라인드 컨트롤러 (ESP32-H2/C6) v3.5.
 * Public Domain (CC0). 무보증.
 */

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <app_priv.h>
#include "common_macros.h"

#include <device.h>
#include <led_driver.h>

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t light_endpoint_id;

// Global variables to store current XY color coordinates
static uint16_t current_x = 0;
static uint16_t current_y = 0;

/* Do any conversions/remapping for the actual value here */
static esp_err_t app_driver_light_set_power(led_driver_handle_t handle, esp_matter_attr_val_t *val)
{
    return led_driver_set_power(handle, val->val.b);
}

static esp_err_t app_driver_light_set_brightness(led_driver_handle_t handle, esp_matter_attr_val_t *val)
{
    int value = REMAP_TO_RANGE(val->val.u8, MATTER_BRIGHTNESS, STANDARD_BRIGHTNESS);
    return led_driver_set_brightness(handle, value);
}

static esp_err_t app_driver_light_set_hue(led_driver_handle_t handle, esp_matter_attr_val_t *val)
{
    int value = REMAP_TO_RANGE(val->val.u8, MATTER_HUE, STANDARD_HUE);
    return led_driver_set_hue(handle, value);
}

static esp_err_t app_driver_light_set_saturation(led_driver_handle_t handle, esp_matter_attr_val_t *val)
{
    int value = REMAP_TO_RANGE(val->val.u8, MATTER_SATURATION, STANDARD_SATURATION);
    return led_driver_set_saturation(handle, value);
}

static esp_err_t app_driver_light_set_temperature(led_driver_handle_t handle, esp_matter_attr_val_t *val)
{
    uint32_t value = REMAP_TO_RANGE_INVERSE(val->val.u16, STANDARD_TEMPERATURE_FACTOR);
    return led_driver_set_temperature(handle, value);
}

static esp_err_t app_driver_light_set_xy(led_driver_handle_t handle, uint16_t x, uint16_t y)
{
    return led_driver_set_xy(handle, x, y);
}


esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    esp_err_t err = ESP_OK;
    if (endpoint_id == light_endpoint_id) {
        led_driver_handle_t handle = (led_driver_handle_t)driver_handle;
        if (cluster_id == OnOff::Id) {
            if (attribute_id == OnOff::Attributes::OnOff::Id) {
                err = app_driver_light_set_power(handle, val);
            }
        } else if (cluster_id == LevelControl::Id) {
            if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                err = app_driver_light_set_brightness(handle, val);
            }
        } else if (cluster_id == ColorControl::Id) {
            if (attribute_id == ColorControl::Attributes::CurrentHue::Id) {
                err = app_driver_light_set_hue(handle, val);
            } else if (attribute_id == ColorControl::Attributes::CurrentSaturation::Id) {
                err = app_driver_light_set_saturation(handle, val);
            } else if (attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
                err = app_driver_light_set_temperature(handle, val);
            } else if (attribute_id == ColorControl::Attributes::CurrentX::Id) {
                current_x = val->val.u16;
                err = app_driver_light_set_xy(handle, current_x, current_y);
            } else if (attribute_id == ColorControl::Attributes::CurrentY::Id) {
                current_y = val->val.u16;
                err = app_driver_light_set_xy(handle, current_x, current_y);
            }
        }
    }
    return err;
}

esp_err_t app_driver_light_set_defaults(uint16_t endpoint_id)
{
    esp_err_t err = ESP_OK;
    void *priv_data = endpoint::get_priv_data(endpoint_id);
    led_driver_handle_t handle = (led_driver_handle_t)priv_data;
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);

    /* Setting brightness */
    attribute_t *attribute = attribute::get(endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_brightness(handle, &val);

    /* Setting color */
    attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorMode::Id);
    attribute::get_val(attribute, &val);
    if (val.val.u8 == (uint8_t)ColorControl::ColorMode::kCurrentHueAndCurrentSaturation) {
        /* Setting hue */
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_light_set_hue(handle, &val);
        /* Setting saturation */
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_light_set_saturation(handle, &val);
    } else if (val.val.u8 == (uint8_t)ColorControl::ColorMode::kColorTemperature) {
        /* Setting temperature */
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_light_set_temperature(handle, &val);
    } else if (val.val.u8 == (uint8_t)ColorControl::ColorMode::kCurrentXAndCurrentY) {
        /* Setting XY coordinates */
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id);
        attribute::get_val(attribute, &val);
        current_x = val.val.u16;
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id);
        attribute::get_val(attribute, &val);
        current_y = val.val.u16;
        err |= app_driver_light_set_xy(handle, current_x, current_y);
    } else {
        ESP_LOGE(TAG, "Color mode not supported");
    }

    /* Setting power */
    attribute = attribute::get(endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_power(handle, &val);

    return err;
}

app_driver_handle_t app_driver_light_init()
{
    /* Initialize led */
    led_driver_config_t config = led_driver_get_config();
    led_driver_handle_t handle = led_driver_init(&config);
    return (app_driver_handle_t)handle;
}

/* ★★★2026-08-23 **공장초기화 버튼(iot_button) 경로 통째로 제거** — 사용자 지시.
 *
 *  왜 제거하나: 이게 **깨어남의 최대 단일 원인**이었다.
 *  배터리 유휴 실측에서 125만 회 수면 중 20ms 를 넘은 것이 한 번도 없었다.
 *  버튼 폴을 100ms 로 늘린 뒤에도 그대로였고, 깨어난 원인은 99.97% 가 TIMER 인데
 *  우리 태스크 주기는 100/100/500/500ms 뿐이라 12~20ms 를 만들 게 없었다.
 *  `esp_timer_dump()`(콘솔 `tmr`)로 찍어 보니 등록된 타이머가 딱 하나였다:
 *        Name          Period   Times_trigg   Cb_exec_time
 *        button_timer  20000    10743         60767 us
 *  iot_button(espressif__button)이 CONFIG_BUTTON_PERIOD_TIME_MS(20ms) 주기
 *  esp_timer 를 **상시** 돌리고 있었다 = 초당 50회. 당시 전체 깨어남 63.9회/초 중
 *  **50회가 이것**이었다. 제거 후 `tmr` 출력은 **타이머 0개**로 비었다.
 *
 *  이 버튼은 esp-matter light 예제에서 복사돼 온 잔재로, 공장초기화(BOOT 5초
 *  롱프레스) 하나에만 쓰였다. 실제 조작 버튼은 전부 PCF8575 → button_handler 가
 *  처리한다. 사용자 확인: "공장초기화 버튼은 안 써" → 통째로 들어낸다.
 *  (공장초기화가 필요하면 설정 메뉴의 Thread reset 경로가 있다.)
 *
 *  ※2026-08-23 배터리 오독(2,600mV) 원인 규명 중 이 제거를 잠시 되돌려 A/B 했다.
 *    제거/복원 두 빌드가 2,604 / 2,600mV 로 동일 → **무관**. 원인은 하드웨어
 *    (BAT_ADC 분압 하단 R5 접촉 불량)로 확정됐다. 그래서 제거를 유지한다.
 *  ※iot_button 에 절전 모드(enable_power_save: 유휴 시 타이머 정지 + GPIO 인터럽트
 *    재기동)가 있어 그걸 켜는 선택지도 있었으나, 쓰지 않는 기능이라 제거가 낫다.
 *  ※되돌리려면 app_priv.h 선언과 함께 아래를 복원하면 된다:
 *      app_driver_handle_t handle = ...iot_button_new_gpio_device(...)
 *      iot_button_register_cb(handle, BUTTON_PRESS_DOWN, ..., toggle_cb, NULL);
 *      app_reset_button_register(handle);
 */
