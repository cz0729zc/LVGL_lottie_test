#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_ADC_MAX_CHANNELS  4   // 可根据实际需求调整

typedef struct {
    int channel;        // ADC通道号
    int gpio_num;       // ADC对应的GPIO
} bsp_adc_channel_cfg_t;

typedef struct {
    bsp_adc_channel_cfg_t channels[BSP_ADC_MAX_CHANNELS];
    int channel_num;
    int sample_freq_hz; // 采样频率
} bsp_adc_config_t;

// 初始化ADC，配置多通道
esp_err_t bsp_adc_init(const bsp_adc_config_t *config);

// 启动ADC采集任务（自动采集，回调/队列/缓存方式均可）
esp_err_t bsp_adc_start(void);

// 停止ADC采集任务
esp_err_t bsp_adc_stop(void);

// 获取最新采样值（可选：阻塞/非阻塞）
esp_err_t bsp_adc_get_latest(uint16_t *out_values, int *out_channel_num);

#ifdef __cplusplus
}
#endif

#endif // _BSP_ADC_H_

