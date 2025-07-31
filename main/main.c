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

#include "bsp_iis_MAX98357A.h"
#include "freertos/task.h"
#include <math.h>
 
 #include "lvgl.h"
 #include "esp_lvgl_port.h"
 //#include "lv_rlottie.h"

 #include "ui_custom/gui_guider.h"
 #include "ui_custom/custom.h"
 #include "ui_custom/events_init.h"
 
 #include "app_controller.h"
 #include "app_anim.h"
 #include "app_timer_service.h"
 
 lv_ui guider_ui;

#define SAMPLE_RATE     (44100)
#define BUFFER_SIZE     (1024)
#define SINE_FREQ       (440.0) // A4 note

static void audio_play_task(void *args)
{
    int16_t *audio_buffer = malloc(BUFFER_SIZE * sizeof(int16_t) * 2); // Stereo
    if (!audio_buffer) {
        ESP_LOGE("AUDIO_TASK", "Failed to allocate memory for audio buffer");
        vTaskDelete(NULL);
        return;
    }

    double phase = 0.0;
    double phase_step = 2.0 * M_PI * SINE_FREQ / SAMPLE_RATE;

    while (1) {
        for (int i = 0; i < BUFFER_SIZE; i++) {
            int16_t sample = (int16_t)(sin(phase) * INT16_MAX * 0.5);
            audio_buffer[i * 2] = sample; // Left channel
            audio_buffer[i * 2 + 1] = sample; // Right channel
            phase += phase_step;
            if (phase >= 2.0 * M_PI) {
                phase -= 2.0 * M_PI;
            }
        }

        size_t bytes_written = 0;
        esp_err_t err = bsp_iis_max98357a_write(audio_buffer, BUFFER_SIZE * sizeof(int16_t) * 2, &bytes_written, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE("AUDIO_TASK", "I2S write failed: %s", esp_err_to_name(err));
        }
    }

    free(audio_buffer);
    vTaskDelete(NULL);
}
 
  void app_main(void)
 {
    /*****BSP驱动层代码初始化*********/
     /* LCD HW initialization */
     ESP_ERROR_CHECK(app_lcd_init());
     /* LVGL initialization */
     ESP_ERROR_CHECK(app_lvgl_init());
    // /* DS18B20 初始化 */
    // ESP_ERROR_CHECK(bsp_ds18b20_init());

    // /* LTR390UV 检测与初始化*/
    //  ESP_ERROR_CHECK(bsp_ltr390uv_init());
    /* I2S audio initialization */
    //ESP_ERROR_CHECK(bsp_iis_max98357a_init(SAMPLE_RATE));
   /*****BSP应用层代码初始化*********/
    adc_app_init();

    app_controller_init();
    app_timer_service_init();

    // 选择使用STA模式或SoftAP模式
    // 方式1: 使用STA模式连接到现有WiFi网络
    //app_wifi_init_sta();
    //app_wifi_wait_connected();
    
    // 方式2: 创建SoftAP热点（注释掉上面两行，取消注释下面一行）
    // app_wifi_init_ap();

    /*****创建任务*********/
   //xTaskCreate(audio_play_task, "audio_play_task", 4096, NULL, 5, NULL);

    //bsp_ds18b20_start_read_task(3, 4096);
    //bsp_ltr390uv_start_read_task(4, 4096);
     adc_app_task_start_read_task(5, 4096);
     app_controller_start_task(6, 4096);  // 启动控制器任务
    //app_wifi_wait_connected();
    /*****GUI界面初始化和事件初始化***/
    app_anim_init(&guider_ui); // 初始化动画模块
}