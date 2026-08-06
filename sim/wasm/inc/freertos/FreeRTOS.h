#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
typedef int      BaseType_t;
#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
#define portMAX_DELAY        0xffffffffUL
#define configMINIMAL_STACK_SIZE 0
#define pdMS_TO_TICKS(ms)    ((TickType_t)(ms))
#define portTICK_PERIOD_MS   1
#define tskIDLE_PRIORITY     0
