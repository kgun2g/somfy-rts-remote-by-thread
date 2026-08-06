#pragma once
#include "FreeRTOS.h"
typedef void* TaskHandle_t;
void vTaskDelay(TickType_t ticks);
BaseType_t xTaskCreate(void(*fn)(void*), const char* name, unsigned stack,
                       void* arg, unsigned prio, TaskHandle_t* out);
void vTaskDelete(TaskHandle_t t);
