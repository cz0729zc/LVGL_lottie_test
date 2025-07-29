#include "bsp_adc.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
<<<<<<< HEAD

=======
#include "app_controller.h"  // 添加控制器头文件
#include <math.h>
>>>>>>> feature/wifi

static adc_cali_handle_t adc_cali_handle_ch0 = NULL;
static adc_cali_handle_t adc_cali_handle_ch1 = NULL;

<<<<<<< HEAD
=======
static const char *TAG = "adc_app";

>>>>>>> feature/wifi
void adc_app_init(void)
{
    bsp_adc_config_t adc_cfg = {
        .channels = {
            {ADC1_CHANNEL_0, 1}, // GPIO1
            {ADC1_CHANNEL_1, 2}, // GPIO2
        },
        .channel_num = 2,
        .sample_freq_hz = 10,
    };
    bsp_adc_init(&adc_cfg);

    // 采样衰减配置（只需应用层调用一次即可）
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12); // 0-3300mV
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);  // 25-1600mV

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

    esp_err_t cali_ret_ch0 = adc_cali_create_scheme_curve_fitting(&cali_config_ch0, &adc_cali_handle_ch0);
    esp_err_t cali_ret_ch1 = adc_cali_create_scheme_curve_fitting(&cali_config_ch1, &adc_cali_handle_ch1);

    if (cali_ret_ch0 == ESP_OK && cali_ret_ch1 == ESP_OK) {
<<<<<<< HEAD
        ESP_LOGI("ADC", "ADC 校准初始化成功");
    } else {
        ESP_LOGW("ADC", "ADC 校准初始化失败，使用原始值");
=======
        ESP_LOGI(TAG, "ADC 校准初始化成功");
    } else {
        ESP_LOGW(TAG, "ADC 校准初始化失败，使用原始值");
>>>>>>> feature/wifi
    }

    bsp_adc_start();
}

void adc_app_task(void *param)
{
    uint16_t values[BSP_ADC_MAX_CHANNELS];
    int ch_num = 0;
<<<<<<< HEAD
    while (1)
    {
        bsp_adc_get_latest(values, &ch_num);
=======
    float percent_ch0 = 0.0f, percent_ch1 = 0.0f;
    
    while (1)
    {
        bsp_adc_get_latest(values, &ch_num);
        
        // 重置百分比值
        percent_ch0 = 0.0f;
        percent_ch1 = 0.0f;
        
>>>>>>> feature/wifi
        for (int i = 0; i < ch_num; ++i) {
            uint16_t adc_val = values[i];
            int voltage = 0;

            // 根据通道选择对应的校准句柄
            adc_cali_handle_t current_cali_handle = NULL;
            if (i == 0) {
                current_cali_handle = adc_cali_handle_ch0;
            } else if (i == 1) {
                current_cali_handle = adc_cali_handle_ch1;
            }

            if (current_cali_handle) {
                adc_cali_raw_to_voltage(current_cali_handle, adc_val, &voltage);
            } else {
                if (i == 0) {
                    voltage = (int)(adc_val * 3300 / 4095);
                } else if (i == 1) {
                    voltage = (int)(adc_val * 1750 / 4095);
                }
            }

            float percent = 0.0f;
            if (i == 0) {
                percent = (voltage - 0.0f) * 100.0f / 3300.0f;
<<<<<<< HEAD
            } else if (i == 1) {
                percent = (voltage - 25.0f) * 100.0f / (1600.0f - 25.0f);
=======
                percent_ch0 = percent;
            } else if (i == 1) {
                percent = (voltage - 25.0f) * 100.0f / (1600.0f - 25.0f);
                percent_ch1 = percent;
>>>>>>> feature/wifi
            }

            if (percent < 0) percent = 0;
            if (percent > 100) percent = 100;

<<<<<<< HEAD
            printf("ADC[%d]=%d, Voltage=%.2f V, Percent=%.1f%%\n", i, adc_val, voltage / 1000.0f, percent);
        }
=======
            ESP_LOGI(TAG, "ADC[%d]=%d, Voltage=%.2f V, Percent=%.1f%%", i, adc_val, voltage / 1000.0f, percent);
        }
        
        // 通知应用控制器新的ADC数据
        app_controller_notify_adc_data(percent_ch0, percent_ch1);

>>>>>>> feature/wifi
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void adc_app_task_start_read_task(uint32_t priority, uint32_t stack_size)
{
<<<<<<< HEAD
    xTaskCreate(adc_app_task, "adc_app_task", 4096, NULL, 5, NULL);
} 
=======
    xTaskCreate(adc_app_task, "adc_app_task", stack_size, NULL, priority, NULL);
}
>>>>>>> feature/wifi
