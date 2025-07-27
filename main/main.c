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
 #include "bsp_ds18b20.h"
 #include "bsp_adc.h"
 #include "esp_adc/adc_cali.h"
 #include "esp_adc/adc_cali_scheme.h"
 #include "app_adc.h"
 #include "app_wifi.h"

 #include "lvgl.h"
 #include "esp_lvgl_port.h"
 //#include "lv_rlottie.h"

 #include "ui_custom/gui_guider.h"
 #include "ui_custom/custom.h"
 #include "ui_custom/events_init.h"
 
 lv_ui guider_ui;

 void app_main(void)
 {
    /*****BSP驱动层代码初始化*********/
    //  /* LCD HW initialization */
    //  ESP_ERROR_CHECK(app_lcd_init());
    //  /* LVGL initialization */
    //  ESP_ERROR_CHECK(app_lvgl_init());
    // /* DS18B20 初始化 */
    //  ESP_ERROR_CHECK(bsp_ds18b20_init());
    // /* LTR390UV 检测与初始化*/
    //  ESP_ERROR_CHECK(bsp_ltr390uv_init());
    /*****BSP应用层代码初始化*********/
    adc_app_init();
    //app_wifi_init();
    /*****创建任务*********/

    // bsp_ds18b20_start_read_task(3, 4096);
    //bsp_ltr390uv_start_read_task(4, 4096);
    adc_app_task_start_read_task(5, 4096);
    //app_wifi_wait_connected();
     /* Show LVGL objects */
     // app_main_display();
    // /*****GUI界面初始化和事件初始化***/
    // setup_ui(&guider_ui);
    // events_init(&guider_ui);
}