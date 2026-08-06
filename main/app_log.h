/*
 * app_log.h — 표준 레벨별 로깅 (ESP-IDF 내장 esp_log 활용).
 * Somfy RTS 블라인드 컨트롤러 (ESP32-C6) v3.5.
 *
 * 로깅은 ESP-IDF 기본 유틸리티 esp_log 를 사용한다:
 *   ESP_LOGE(TAG,..) 오류 / ESP_LOGW 경고 / ESP_LOGI 정보 /
 *   ESP_LOGD 디버그 / ESP_LOGV 상세.
 *
 * 전역 로그 레벨은 아래 APP_LOG_DEFAULT_LEVEL 한 곳에서 설정하고,
 * app_log_init() 이 부팅 시 esp_log_level_set("*", ...) 로 모든 태그에
 * 일괄 적용한다(빌드 시 CONFIG_LOG_MAXIMUM_LEVEL 이하 범위에서 동작).
 * 런타임에 특정 태그만 바꾸려면 esp_log_level_set("TAG", lvl) 사용.
 */
#pragma once

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ★ 전역 기본 로그 레벨 — 여기 한 줄만 바꾸면 전체에 반영.
 *  ESP_LOG_ERROR / _WARN / _INFO / _DEBUG / _VERBOSE / _NONE */
#ifndef APP_LOG_DEFAULT_LEVEL
#define APP_LOG_DEFAULT_LEVEL  ESP_LOG_INFO
#endif

/* 부팅 시 1회 호출 — 전역 로그 레벨을 모든 태그에 적용. */
static inline void app_log_init(void)
{
    esp_log_level_set("*", APP_LOG_DEFAULT_LEVEL);
    ESP_LOGI("APP_LOG", "전역 로그 레벨=%d 적용 (E0 W1 I2 D3 V4)",
             (int)APP_LOG_DEFAULT_LEVEL);
}

#ifdef __cplusplus
}
#endif
