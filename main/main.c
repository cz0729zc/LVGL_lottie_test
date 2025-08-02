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

#include "audio_player.h"
#include "audio_example.h"
#include "freertos/task.h"
#include <math.h>
 
 #include "lvgl.h"
 #include "esp_lvgl_port.h"
 //#include "lv_rlottie.h"

 #include "ui_custom/gui_guider.h" // Include the UI header to get the extern declaration
 #include "app_controller.h"
 #include "app_anim.h"
 #include "app_timer_service.h"
 

  void app_main(void)
 {
    // ESP_ERROR_CHECK(app_lcd_init());            // 初始化 LCD
    // ESP_ERROR_CHECK(app_lvgl_init());           // 初始化 LVGL
    // ESP_ERROR_CHECK(bsp_ds18b20_init());        // 初始化 DS18B20 传感器
    // ESP_ERROR_CHECK(bsp_ltr390uv_init());       // 初始化 LTR390UV 传感器
    // ESP_ERROR_CHECK(audio_player_init(44100));  // 初始化音频播放器，采样率为 44100Hz
    // adc_app_init();                             // 初始化 ADC 应用
    // app_controller_init();                      // 初始化应用控制器
    // app_timer_service_init();                   // 初始化定时器服务


    // bsp_ds18b20_start_read_task(3, 4096);    // 启动 DS18B20 读取任务，优先级为 3，栈大小为 4096
    // bsp_ltr390uv_start_read_task(4, 4096);   // 启动 LTR390UV 读取任务，优先级为 4，栈大小为 4096
    // adc_app_task_start_read_task(5, 4096);  // 启动 ADC 读取任务，优先级为 5，栈大小为 4096
    // app_controller_start_task(6, 4096);     // 启动控制器任务，优先级为 6，栈大小为 4096
    /*****BSP驱动层代码初始化*********/
     /* LCD HW initialization */
    //  ESP_ERROR_CHECK(app_lcd_init());
    //  /* LVGL initialization */
    //  ESP_ERROR_CHECK(app_lvgl_init());
    // /* DS18B20 初始化 */
    // ESP_ERROR_CHECK(bsp_ds18b20_init());

    // /* LTR390UV 检测与初始化*/
    //  ESP_ERROR_CHECK(bsp_ltr390uv_init());
    /* I2S audio initialization */
    //ESP_ERROR_CHECK(audio_player_init(44100));
    
   /*****BSP应用层代码初始化*********/
    // adc_app_init();

    // app_controller_init();
    // app_timer_service_init();

    // 选择使用STA模式或SoftAP模式
    // 方式1: 使用STA模式连接到现有WiFi网络
    //app_wifi_init_sta();
    //app_wifi_wait_connected();
    
    // 方式2: 创建SoftAP热点（注释掉上面两行，取消注释下面一行）
    // app_wifi_init_ap();

    /*****创建任务*********/
    // 播放1kHz正弦波，持续1秒 (1000毫秒)
    ESP_ERROR_CHECK(audio_player_play_for(sine_wave_1khz_16bit_16000_mono, sine_wave_1khz_16bit_16000_mono_len, 16000, 1000));
    // bsp_ds18b20_start_read_task(3, 4096);
    // bsp_ltr390uv_start_read_task(4, 4096);
    // adc_app_task_start_read_task(5, 4096);
    // app_controller_start_task(6, 4096);  // 启动控制器任务
    //app_wifi_wait_connected();
}