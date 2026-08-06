#pragma once
/* sim: 로그 no-op */
#define ESP_LOGE(t,...) ((void)0)
#define ESP_LOGW(t,...) ((void)0)
#define ESP_LOGI(t,...) ((void)0)
#define ESP_LOGD(t,...) ((void)0)
#define ESP_LOGV(t,...) ((void)0)
#define esp_log_level_set(t,l) ((void)0)
