#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <stdint.h>

esp_err_t bsp_ltr390uv_init(void);
void bsp_ltr390uv_start_read_task(uint32_t priority, uint32_t stack_size); 