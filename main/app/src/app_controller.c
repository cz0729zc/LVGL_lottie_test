// app_controller.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "app_controller.h"
#include "ui_custom/gui_guider.h"
#include "ui_custom/custom.h"
#include "esp_log.h"
#include <math.h>

// 声明外部的guider_ui变量
extern lv_ui guider_ui;

// 事件组句柄
static EventGroupHandle_t controller_event_group = NULL;

// 传感器数据存储
static struct {
    float adc_percent_ch0;
    float adc_percent_ch1;
    float temperature;
    float uv_index;
} sensor_data = {0};

// 当前界面状态
static int current_screen = 0;

static const char *TAG = "app_controller";

void app_controller_init(void)
{
    controller_event_group = xEventGroupCreate();
    if (controller_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
    }
}

// 应用决策函数
static void make_decisions_based_on_sensors(void)
{
    // 复杂的决策逻辑
    // 示例：根据多个传感器数据决定显示哪个界面
    
    // 如果ADC通道0百分比大于80%，显示警告界面
    if (sensor_data.adc_percent_ch0 > 80.0f) {
        // 显示警告界面
        ESP_LOGI(TAG, "High ADC value detected: %.1f%%", sensor_data.adc_percent_ch0);
        // ui_load_scr_animation(&guider_ui, &guider_ui.warning_screen, ...);
    }
    // 如果温度和光照都高，显示另一个界面
    else if (sensor_data.temperature > 30.0f && sensor_data.uv_index > 5.0f) {
        // 显示高温高光照界面
        ESP_LOGI(TAG, "High temp and UV detected: %.1f°C, %.1f UV", 
                 sensor_data.temperature, sensor_data.uv_index);
        // ui_load_scr_animation(&guider_ui, &guider_ui.hot_sunny_screen, ...);
    }
    // 默认情况显示主界面
    else {
        // 显示主界面
        ESP_LOGI(TAG, "Normal conditions");
        // ui_load_scr_animation(&guider_ui, &guider_ui.main_screen, ...);
    }
}

// 控制器主任务
static void app_controller_task(void *param)
{
    EventBits_t events;
    
    while (1) {
        // 等待传感器数据事件
        events = xEventGroupWaitBits(
            controller_event_group,
            SENSOR_ADC_DATA_READY | SENSOR_TEMP_DATA_READY | SENSOR_LIGHT_DATA_READY,
            pdTRUE,    // 清除事件位
            pdFALSE,   // 等待任意一个事件
            pdMS_TO_TICKS(1000)  // 超时时间
        );
        
        // 根据接收到的事件处理数据
        if (events & SENSOR_ADC_DATA_READY) {
            ESP_LOGD(TAG, "Processing ADC data");
        }
        
        if (events & SENSOR_TEMP_DATA_READY) {
            ESP_LOGD(TAG, "Processing temperature data");
        }
        
        if (events & SENSOR_LIGHT_DATA_READY) {
            ESP_LOGD(TAG, "Processing light data");
        }
        
        // 执行决策逻辑
        make_decisions_based_on_sensors();
        
        // 延迟一段时间避免过于频繁的处理
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_controller_start_task(uint32_t priority, uint32_t stack_size)
{
    xTaskCreate(app_controller_task, "app_controller", stack_size, NULL, priority, NULL);
}

// 供其他模块调用的通知函数
void app_controller_notify_adc_data(float percent_ch0, float percent_ch1)
{
    sensor_data.adc_percent_ch0 = percent_ch0;
    sensor_data.adc_percent_ch1 = percent_ch1;
    
    if (controller_event_group != NULL) {
        xEventGroupSetBits(controller_event_group, SENSOR_ADC_DATA_READY);
    }
}

void app_controller_notify_temp_data(float temperature)
{
    sensor_data.temperature = temperature;
    
    if (controller_event_group != NULL) {
        xEventGroupSetBits(controller_event_group, SENSOR_TEMP_DATA_READY);
    }
}

void app_controller_notify_light_data(float uv_index)
{
    sensor_data.uv_index = uv_index;
    
    if (controller_event_group != NULL) {
        xEventGroupSetBits(controller_event_group, SENSOR_LIGHT_DATA_READY);
    }
}