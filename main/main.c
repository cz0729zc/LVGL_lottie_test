/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include "esp_err.h"
 #include "esp_log.h"
 #include "esp_check.h"
 #include "driver/gpio.h"
 #include "driver/adc.h"
 #include "bsp_lcd.h"
 #include "bsp_lvgl.h"
 #include "bsp_ltr390uv.h"
 #include "bsp_as7341.h"
 #include "bsp_ds18b20.h"
 #include "bsp_adc.h"
 #include "Catout.h"    
 #include "eye.h"
 #include "esp_adc/adc_cali.h"
 #include "esp_adc/adc_cali_scheme.h"

 #include "lvgl.h"
 #include "esp_lvgl_port.h"
 //#include "lv_rlottie.h"

 #include "ui_custom/gui_guider.h"
 #include "ui_custom/custom.h"
 #include "ui_custom/events_init.h"
 
 lv_ui guider_ui;

 /* LCD size */
 #define EXAMPLE_LCD_H_RES   (128)
 #define EXAMPLE_LCD_V_RES   (160)




 static void app_main_display(void)
 {
     lv_obj_t *scr = lv_scr_act();
 
     /* Task lock */
     lvgl_port_lock(0);
 
     // 检查可用内存
     size_t free_heap = esp_get_free_heap_size();
     ESP_LOGI("MAIN", "Free heap before Lottie: %zu bytes", free_heap);
     
     if (free_heap < 30000) { // 降低内存阈值
         ESP_LOGW("MAIN", "Memory low, showing simple label instead of Lottie");
         lv_obj_t *label = lv_label_create(scr);
         lv_label_set_text(label, "Memory Low");
         lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
     } else {
        lv_obj_t *lottie = lv_rlottie_create_from_raw(scr, 150, 150, (const char *)eye);
        if (lottie != NULL) {
            lv_obj_center(lottie);
            ESP_LOGI("MAIN", "Complex Lottie animation created successfully");
        } else {
            ESP_LOGE("MAIN", "Failed to create complex Lottie animation");
            // 创建失败时显示简单文本
            lv_obj_t *label = lv_label_create(scr);
            lv_label_set_text(label, "Complex Lottie Failed");
            lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
        }
     }
     // 再次检查内存
     free_heap = esp_get_free_heap_size();
     ESP_LOGI("MAIN", "Free heap after display: %zu bytes", free_heap);
 
     lvgl_port_unlock();
 }
 
 void app_main(void)
 {
     /* LCD HW initialization */
     ESP_ERROR_CHECK(app_lcd_init());
     /* LVGL initialization */
     ESP_ERROR_CHECK(app_lvgl_init());
    //  // DS18B20 初始化
    //  ESP_ERROR_CHECK(bsp_ds18b20_init());
     // LTR390UV 检测与初始化
     //ESP_ERROR_CHECK(bsp_ltr390uv_init());

     // ADC 初始化
     bsp_adc_config_t adc_cfg = {
        .channels = {
            {ADC1_CHANNEL_0, 1}, // GPIO1
            {ADC1_CHANNEL_1, 2}, // GPIO2
        },
        .channel_num = 2,
        .sample_freq_hz = 10,
    };
    bsp_adc_init(&adc_cfg);
    
    // 手动配置不同通道的衰减系数
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // 0-3300mV
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);  // 25-1600mV
    
    //ADC 校准初始化 - 为不同衰减系数创建校准句柄
    adc_cali_curve_fitting_config_t cali_config_ch0 = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_curve_fitting_config_t cali_config_ch1 = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_12,
    };
    
    adc_cali_handle_t adc_cali_handle_ch0 = NULL;
    adc_cali_handle_t adc_cali_handle_ch1 = NULL;
    
    esp_err_t cali_ret_ch0 = adc_cali_create_scheme_curve_fitting(&cali_config_ch0, &adc_cali_handle_ch0);
    esp_err_t cali_ret_ch1 = adc_cali_create_scheme_curve_fitting(&cali_config_ch1, &adc_cali_handle_ch1);
    
    if (cali_ret_ch0 == ESP_OK && cali_ret_ch1 == ESP_OK) {
        ESP_LOGI("ADC", "ADC 校准初始化成功");
    } else {
        ESP_LOGW("ADC", "ADC 校准初始化失败，使用原始值");
    }
 
    //  /* 创建传感器任务,需放在屏幕显示前*/
    // bsp_ds18b20_start_read_task(3, 4096);
    //bsp_ltr390uv_start_read_task(4, 4096);
    bsp_adc_start();
     /* Show LVGL objects */
     // app_main_display();
    /*****GUI界面初始化和事件初始化***/
    setup_ui(&guider_ui);
    events_init(&guider_ui);

    uint16_t values[BSP_ADC_MAX_CHANNELS];
    int ch_num = 0;
    while (1)
    {
        bsp_adc_get_latest(values, &ch_num);
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
                // 校准失败时，根据通道使用不同的线性换算
                if (i == 0) {
                    // ADC1_CHANNEL_0: 0-3300mV, ADC_ATTEN_DB_11
                    voltage = (int)(adc_val * 3300 / 4095);
                } else if (i == 1) {
                    // ADC1_CHANNEL_1: 25-1600mV, ADC_ATTEN_DB_6
                    voltage = (int)(adc_val * 1750 / 4095);
                }
            }
            
            // 根据通道使用不同的百分比计算
            float percent = 0.0f;
            if (i == 0) {
                // ADC1_CHANNEL_0: 0-3300mV范围
                percent = (voltage - 0.0f) * 100.0f / 3300.0f;
            } else if (i == 1) {
                // ADC1_CHANNEL_1: 25-1600mV范围
                percent = (voltage - 25.0f) * 100.0f / (1600.0f - 25.0f);
            }
            
            if (percent < 0) percent = 0;
            if (percent > 100) percent = 100;
            
            printf("ADC[%d]=%d, Voltage=%.2f V, Percent=%.1f%%\n", i, adc_val, voltage / 1000.0f, percent);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }


    }