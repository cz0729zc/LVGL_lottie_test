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
 //#include "lv_rlottie.h

 /* LCD size */
 #define EXAMPLE_LCD_H_RES   (128)
 #define EXAMPLE_LCD_V_RES   (160)


 static adc_cali_handle_t adc_cali_handle = NULL;


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
     // DS18B20 初始化
     ESP_ERROR_CHECK(bsp_ds18b20_init());
     // LTR390UV 检测与初始化
     ESP_ERROR_CHECK(bsp_ltr390uv_init());

     // ADC 初始化
     bsp_adc_config_t adc_cfg = {
        .channels = {
            {ADC1_CHANNEL_0, 1}, // GPIO1
            //{ADC1_CHANNEL_3, 4}, // GPIO4
        },
        .channel_num = 1,
        .sample_freq_hz = 10,
    };
    bsp_adc_init(&adc_cfg);
    //ADC 校准初始化
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (cali_ret == ESP_OK) {
        ESP_LOGI("ADC", "ADC 校准初始化成功");
    } else {
        ESP_LOGW("ADC", "ADC 校准初始化失败，使用原始值");
    }
 
    //  /* 创建传感器任务,需放在屏幕显示前*/
    //  bsp_ds18b20_start_read_task(3, 4096);
    //  bsp_ltr390uv_start_read_task(4, 4096);
     bsp_adc_start();
     /* Show LVGL objects */
    //  app_main_display();

    uint16_t values[BSP_ADC_MAX_CHANNELS];
    int ch_num = 0;
    while (1)
    {
        bsp_adc_get_latest(values, &ch_num);
        for (int i = 0; i < ch_num; ++i) {
            uint16_t adc_val = values[i];
            int voltage = 0;
            if (adc_cali_handle) {
                adc_cali_raw_to_voltage(adc_cali_handle, adc_val, &voltage);
            } else {
                // 校准失败时，简单线性换算
                voltage = (int)(adc_val * 3300 / 4095);
            }
            float percent = (voltage - 150.0f) * 100.0f / (2450.0f - 150.0f);
            if (percent < 0) percent = 0;
                    if (percent > 100) percent = 100;
            printf("ADC[%d]=%d, Voltage=%.2f V, Percent=%.1f%%\n", i, adc_val, voltage / 1000.0f, percent);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }


 }