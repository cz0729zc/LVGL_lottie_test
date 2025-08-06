#include "bsp_adc.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "app_controller.h"  // 添加控制器头文件
#include "app_filter.h"      // 添加滤波器头文件
#include <math.h>

static adc_cali_handle_t adc_cali_handle_ch0 = NULL;
static adc_cali_handle_t adc_cali_handle_ch1 = NULL;
static adc_cali_handle_t adc_cali_handle_ch4 = NULL; // 新增：震动传感器的校准句柄

static const char *TAG = "adc_app";

void adc_app_init(void)
{
    bsp_adc_config_t adc_cfg = {
        .channels = {
            {ADC1_CHANNEL_0, 1}, // GPIO1
            {ADC1_CHANNEL_1, 2}, // GPIO2
            {ADC1_CHANNEL_4, 5}, // 新增：GPIO5作为震动传感器输入
        },
        .channel_num = 3, // 更新通道数量
        .sample_freq_hz = 10,
    };
    bsp_adc_init(&adc_cfg);

    // 采样衰减配置（只需应用层调用一次即可）
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12); // 0-3300mV
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);  // 25-1600mV
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12); // 新增：震动传感器使用12dB衰减 (0-3300mV)

    // 校准
    adc_cali_curve_fitting_config_t cali_config_ch0 = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_curve_fitting_config_t cali_config_ch1 = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_curve_fitting_config_t cali_config_ch4 = {
       .unit_id = ADC_UNIT_1,
       .atten = ADC_ATTEN_DB_12,
       .bitwidth = ADC_BITWIDTH_12,
   };

    esp_err_t cali_ret_ch0 = adc_cali_create_scheme_curve_fitting(&cali_config_ch0, &adc_cali_handle_ch0);
    esp_err_t cali_ret_ch1 = adc_cali_create_scheme_curve_fitting(&cali_config_ch1, &adc_cali_handle_ch1);
    esp_err_t cali_ret_ch4 = adc_cali_create_scheme_curve_fitting(&cali_config_ch4, &adc_cali_handle_ch4);

    if (cali_ret_ch0 == ESP_OK && cali_ret_ch1 == ESP_OK && cali_ret_ch4 == ESP_OK) {
        ESP_LOGI(TAG, "ADC 校准初始化成功 for all channels");
    } else {
        ESP_LOGW(TAG, "ADC 校准初始化失败，使用原始值");
    }

    bsp_adc_start();
}

void adc_app_task(void *param)
{
    uint16_t values[BSP_ADC_MAX_CHANNELS];
    int ch_num = 0;
    float last_percent[BSP_ADC_MAX_CHANNELS] = {0};  // 记录上一次百分比值
    const float percent_threshold = 5.0f;            // 变化阈值
    float percent_ch0 = 0.0f, percent_ch1 = 0.0f;

    // 为土壤湿度传感器(CH0)创建一个中位值滤波器实例
    static median_filter_t soil_humidity_filter;
    median_filter_init(&soil_humidity_filter);

   // 震动检测相关变量
   static int last_vibration_voltage = 3300;      // 上一次的电压值，用于检测下降沿
   static uint32_t last_notify_time = 0;          // 上次成功通知的时间戳
   static uint32_t last_single_tap_time = 0;      // 上次检测到单击的时间，用于防抖
   const int VIBRATION_HIGH_THRESHOLD = 3000;      // 高电平阈值 (mV)
   const int VIBRATION_LOW_THRESHOLD = 1800;       // 低电平阈值 (mV)
   const uint32_t SINGLE_TAP_DEBOUNCE_MS = 200;    // 单次敲击的防抖时间 (ms)
   const uint32_t TAP_NOTIFY_COOLDOWN_MS = 5000;   // 两次有效通知之间的冷却时间 (ms)

    while (1)
    {
        bsp_adc_get_latest(values, &ch_num);

        for (int i = 0; i < ch_num; ++i) {
            uint16_t adc_val = values[i];
            
            // 如果是土壤湿度传感器(CH0)，则应用滤波器
            if (i == 0) {
                adc_val = median_filter_update(&soil_humidity_filter, adc_val);
            }

            int voltage = 0;

            // 获取校准句柄
            adc_cali_handle_t current_cali_handle = NULL;
            if (i == 0) current_cali_handle = adc_cali_handle_ch0;
            else if (i == 1) current_cali_handle = adc_cali_handle_ch1;
            else if (i == 2) current_cali_handle = adc_cali_handle_ch4; // 震动传感器通道

            if (current_cali_handle) {
                adc_cali_raw_to_voltage(current_cali_handle, adc_val, &voltage);
            } else {
                // Fallback if calibration fails
                if (i == 0 || i == 2) voltage = adc_val * 3300 / 4095;
                else voltage = adc_val * 1750 / 4095;
            }

           if (i == 2) { // 震动传感器逻辑
                uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // 1. 检测一次有效的、经过防抖处理的敲击
                if (last_vibration_voltage > VIBRATION_HIGH_THRESHOLD &&
                    voltage < VIBRATION_LOW_THRESHOLD &&
                    (now_ms - last_single_tap_time > SINGLE_TAP_DEBOUNCE_MS))
                {
                    last_single_tap_time = now_ms; // 记录本次敲击事件的时间

                    // 2. 检查是否满足5秒冷却时间
                    if (now_ms - last_notify_time > TAP_NOTIFY_COOLDOWN_MS) {
                        ESP_LOGI(TAG, "Tap detected! Notifying controller.");
                        app_controller_notify_tap();
                        last_notify_time = now_ms; // 更新成功通知的时间
                    }
                }
                last_vibration_voltage = voltage; // 持续更新电压值
           } else { // 原有的湿度传感器逻辑
                // 计算百分比
               float percent = 0.0f;
               if (i == 0) {
                    // 新的映射逻辑：将 [20, 80] 的内部值映射到 [0, 100] 的显示值
                    // 首先，计算出原始的内部百分比值
                    float raw_percent = 100.0f - (voltage * 100.0f / 3300.0f);

                    // 定义传感器的实际测量范围
                    const float SENSOR_DRY_VALUE = 25.0f;
                    const float SENSOR_WET_VALUE = 90.0f;

                    // 应用线性映射公式
                    percent = (raw_percent - SENSOR_DRY_VALUE) * 100.0f / (SENSOR_WET_VALUE - SENSOR_DRY_VALUE);

               } else if (i == 1) {
                   percent = (voltage - 25.0f) * 100.0f / (1600.0f - 25.0f);
               }
   
               // 限制范围
               if (percent < 0) percent = 0;
               if (percent > 100) percent = 100;
   
               // 更新通知数据
               if (i == 0) percent_ch0 = percent;
               else if (i == 1) percent_ch1 = percent;
           }
        }

        ESP_LOGI(TAG, "Soil Value： %0.1f%%",percent_ch0);
        // 滤波和映射后的数据已经很平滑，直接通知控制器更新
        app_controller_notify_adc_data(percent_ch0, 100.0f - percent_ch1);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void adc_app_task_start_read_task(uint32_t priority, uint32_t stack_size)
{
    xTaskCreate(adc_app_task, "adc_app_task", stack_size, NULL, priority, NULL);
}
