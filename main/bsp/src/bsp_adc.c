#include "bsp_adc.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BSP_ADC"

static bsp_adc_config_t s_adc_cfg;
static TaskHandle_t s_adc_task_handle = NULL;
static uint16_t s_adc_values[BSP_ADC_MAX_CHANNELS] = {0};

static void bsp_adc_task(void *arg)
{
    while (1) {
        for (int i = 0; i < s_adc_cfg.channel_num; ++i) {
            int channel = s_adc_cfg.channels[i].channel;
            // 这里只以ADC1为例，实际可根据需求扩展
            s_adc_values[i] = adc1_get_raw(channel);
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / s_adc_cfg.sample_freq_hz));
    }
}

esp_err_t bsp_adc_init(const bsp_adc_config_t *config)
{
    if (!config || config->channel_num > BSP_ADC_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    s_adc_cfg = *config;
    // ADC初始化
    for (int i = 0; i < config->channel_num; ++i) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(config->channels[i].channel, ADC_ATTEN_DB_12);
    }
    return ESP_OK;
}

esp_err_t bsp_adc_start(void)
{
    if (s_adc_task_handle) return ESP_OK;
    return xTaskCreate(bsp_adc_task, "bsp_adc_task", 2048, NULL, 5, &s_adc_task_handle) == pdPASS ? ESP_OK : ESP_FAIL;
}

esp_err_t bsp_adc_stop(void)
{
    if (s_adc_task_handle) {
        vTaskDelete(s_adc_task_handle);
        s_adc_task_handle = NULL;
    }
    return ESP_OK;
}

esp_err_t bsp_adc_get_latest(uint16_t *out_values, int *out_channel_num)
{
    if (!out_values || !out_channel_num) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < s_adc_cfg.channel_num; ++i) {
        out_values[i] = s_adc_values[i];
    }
    *out_channel_num = s_adc_cfg.channel_num;
    return ESP_OK;
}

