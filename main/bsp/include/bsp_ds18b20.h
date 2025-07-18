#pragma once
#include "esp_err.h"

esp_err_t bsp_ds18b20_init(void);
void bsp_ds18b20_start_read_task(unsigned int priority, uint32_t stack_size);