/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include "esp_err.h"
 #include "esp_log.h"
 #include "esp_check.h"
 #include "driver/gpio.h"
 #include "bsp_lcd.h"
 #include "bsp_lvgl.h"
 #include "bsp_ltr390uv.h"
 #include "bsp_as7341.h"
 #include "bsp_ds18b20.h"
 #include "Catout.h"    
 #include "eye.h"


 #include "lvgl.h"
 #include "esp_lvgl_port.h"
 //#include "lv_rlottie.h

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
     // DS18B20 初始化
     ESP_ERROR_CHECK(bsp_ds18b20_init());
     // LTR390UV 检测与初始化
     ESP_ERROR_CHECK(bsp_ltr390uv_init());
 
     /* 创建传感器任务,需放在屏幕显示前*/
     bsp_ds18b20_start_read_task(3, 4096);
     bsp_ltr390uv_start_read_task(4, 4096);
     /* Show LVGL objects */
     app_main_display();
 }