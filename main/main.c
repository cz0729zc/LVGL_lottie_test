/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include "esp_err.h"
 #include "esp_log.h"
 #include "esp_check.h"
 #include "bsp_lcd.h"
 #include "bsp_lvgl.h"
 #include "Catout.h"    
 
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
 
     /* Label */
     lv_obj_t *label = lv_label_create(scr);
     lv_obj_set_width(label, EXAMPLE_LCD_H_RES);
     lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
 #if LVGL_VERSION_MAJOR == 8
     lv_label_set_recolor(label, true);
     lv_label_set_text(label, "#FF0000 "LV_SYMBOL_BELL" Hello world Espressif and LVGL "LV_SYMBOL_BELL"#\n#FF9400 "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING" #");
 #else
     lv_label_set_text(label,"hello LVGL9");
 #endif
 
     /* Lottie 动画 */
     lv_obj_t *lottie = lv_rlottie_create_from_raw(scr, 150, 150, (const char *)cat);
     lv_obj_center(lottie);
 
     lvgl_port_unlock();
 }
 
 void app_main(void)
 {
     /* LCD HW initialization */
     ESP_ERROR_CHECK(app_lcd_init());
 
     /* LVGL initialization */
     ESP_ERROR_CHECK(app_lvgl_init());
 
     /* Show LVGL objects */
     app_main_display();
 }