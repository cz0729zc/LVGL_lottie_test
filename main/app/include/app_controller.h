// app_controller.h
#ifndef _APP_CONTROLLER_H_
#define _APP_CONTROLLER_H_


// 定义事件标志
#define SENSOR_ADC_DATA_READY    BIT0
#define SENSOR_TEMP_DATA_READY   BIT1
#define SENSOR_LIGHT_DATA_READY  BIT2
#define WIFI_CONNECTED           BIT3

void app_controller_init(void);
void app_controller_start_task(uint32_t priority, uint32_t stack_size);

// 供其他模块调用的接口函数
void app_controller_notify_adc_data(float percent_ch0, float percent_ch1);
void app_controller_notify_temp_data(float temperature);
void app_controller_notify_light_data(float uv_index);
void app_controller_notify_wifi_connected(void);

#endif
