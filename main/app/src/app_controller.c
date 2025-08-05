/**
 * @file app_controller.c
 * @brief 应用核心控制器实现
 *
 * @see app_controller.h
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "app_controller.h"
#include "ui_custom/gui_guider.h"
#include "ui_custom/custom.h"
#include "ui_controller.h" // 包含新的UI控制器头文件
#include "esp_lvgl_port.h" // 引入lvgl_port以使用锁
#include "esp_log.h"
#include <math.h>
#include "freertos/timers.h" // 引入定时器头文件
#include "app_timer_service.h" // 引入新的定时器服务
#
// 声明外部的guider_ui变量
extern lv_ui guider_ui;

/** @brief 应用控制器的事件组句柄 */
static EventGroupHandle_t controller_event_group = NULL;

/** @brief 存储从各个传感器模块收集的最新数据 */
static struct {
    float adc_percent_ch0;      /**< ADC通道0的百分比值（土壤湿度） */
    float adc_percent_ch1;      /**< ADC通道1的百分比值 （光照强度）*/
    float temperature;          /**< 温度值 */
    float uv_index;             /**< 紫外线指数 */
} sensor_data = {0};


/** @brief 管理小精灵的核心状态 */
static struct {
    sprite_state_t current_state;       /**< 小精灵当前的逻辑状态 */
    uint32_t last_state_change_time;    /**< 上次状态变更的系统时间点 (tick) */
} sprite_status = { .current_state = SPRITE_STATE_AWAY_NORMAL, .last_state_change_time = 0 };


static const char *TAG = "app_controller";
// 函数前向声明
static void sprite_state_machine_run(EventBits_t events);
static void update_system_state(EventBits_t events);


void app_controller_init(void)
{
    controller_event_group = xEventGroupCreate();
    if (controller_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
    }
    
    // 状态初始化移到这里，UI初始化移到任务中
    sprite_status.last_state_change_time = xTaskGetTickCount();
}

/**
 * @brief 小精灵状态机核心处理函数
 *
 * 这是应用逻辑的核心。它根据小精灵的当前状态、传感器数据和用户输入
 * 来决定是否需要转换到新的状态，并触发相应的行为。
 *
 * @param events 当前触发的事件标志位集合
 */
static void sprite_state_machine_run(EventBits_t events)
{
    // --- 高优先级检查 (覆盖正常的状态逻辑) ---
    // 1. 检查10分钟离家定时器是否超时
    if (events & EVENT_TIMER_10_MIN_AWAY_EXPIRED) {
        ESP_LOGI(TAG, "10 min away timer expired. Sprite is now going away.");
        sprite_status.current_state = SPRITE_STATE_AWAY_NORMAL;
        sprite_status.last_state_change_time = xTaskGetTickCount();
        // 不需要手动停止定时器，因为它是一次性的
    }
    // 2. 检查湿度是否过低
    else if (sensor_data.adc_percent_ch0 < 20.0f) {
        // 如果角色在外（正常或准备回家），则强制进入迷路状态
        if (sprite_status.current_state == SPRITE_STATE_AWAY_NORMAL || sprite_status.current_state == SPRITE_STATE_PREPARING_TO_GO_HOME) {
            if (sprite_status.current_state != SPRITE_STATE_AWAY_LOST) {
                ESP_LOGW(TAG, "Humidity too low! Sprite is now LOST.");
                sprite_status.current_state = SPRITE_STATE_AWAY_LOST;
                sprite_status.last_state_change_time = xTaskGetTickCount();
                // 如果“准备回家”的定时器正在运行，通过服务停止它
                app_timer_service_stop(TIMER_ID_5_MIN_GO_HOME);
                // 同样停止离家定时器，以防万一
                app_timer_service_stop(TIMER_ID_10_MIN_AWAY);
            }
        }
    }
    // 3. 检查湿度是否过高
    else if (sensor_data.adc_percent_ch0 > 80.0f) { // 湿度过高 -> 淹水
        if (sprite_status.current_state != SPRITE_STATE_EVENT_FLOODED) {
            ESP_LOGE(TAG, "Humidity too high (%.1f%%)! FLOODED.", sensor_data.adc_percent_ch0);
            sprite_status.current_state = SPRITE_STATE_EVENT_FLOODED;
            sprite_status.last_state_change_time = xTaskGetTickCount();
            // 停止所有可能运行的定时器
            app_timer_service_stop(TIMER_ID_5_MIN_GO_HOME);
            app_timer_service_stop(TIMER_ID_10_MIN_AWAY);
        }
    }


    // 根据当前状态处理特定的事件和逻辑
    switch (sprite_status.current_state) {
        case SPRITE_STATE_AT_HOME_AWAKE:
            ESP_LOGI(TAG, "State: AT_HOME_AWAKE");
            app_timer_service_start(TIMER_ID_10_MIN_AWAY); // 启动或重置10分钟离家定时器
            if (events & USER_INTERACTION_TAP) {
                ESP_LOGI(TAG, "Interaction: Pat the sprite, sprite says hello!");
                // UI相关的行为已移至ui_controller
            }
            break;

        case SPRITE_STATE_AT_HOME_SLEEPING:
            ESP_LOGI(TAG, "State: AT_HOME_SLEEPING");
            app_timer_service_start(TIMER_ID_10_MIN_AWAY); // 启动或重置10分钟离家定时器
            if (events & USER_INTERACTION_TAP) {
                ESP_LOGI(TAG, "Interaction: Pat the sprite, waking it up.");
                sprite_status.current_state = SPRITE_STATE_AT_HOME_AWAKE;
                // 状态已改变，新的循环将重置定时器，此处无需操作
            }
            break;

        case SPRITE_STATE_AWAY_NORMAL:
            ESP_LOGI(TAG, "State: AWAY_NORMAL");
            app_timer_service_stop(TIMER_ID_10_MIN_AWAY); // 在外时，确保离家定时器是停止的
            if (events & USER_INTERACTION_TAP) {
                ESP_LOGI(TAG, "Interaction: Pat the sprite, preparing to go home.");
                sprite_status.current_state = SPRITE_STATE_PREPARING_TO_GO_HOME;
                sprite_status.last_state_change_time = xTaskGetTickCount();
                app_timer_service_start(TIMER_ID_5_MIN_GO_HOME); // 通过服务启动5分钟回家倒计时
            }
            break;

        case SPRITE_STATE_AWAY_LOST:
            ESP_LOGI(TAG, "State: AWAY_LOST");
            app_timer_service_stop(TIMER_ID_10_MIN_AWAY); // 确保停止
            // 检查湿度是否恢复
            if (sensor_data.adc_percent_ch0 >= 20.0f) {
                ESP_LOGI(TAG, "Humidity is back to normal. Sprite is no longer lost.");
                sprite_status.current_state = SPRITE_STATE_PREPARING_TO_GO_HOME;
                sprite_status.last_state_change_time = xTaskGetTickCount();
            }
            break;

        case SPRITE_STATE_PREPARING_TO_GO_HOME:
            ESP_LOGI(TAG, "State: PREPARING_TO_GO_HOME");
            app_timer_service_stop(TIMER_ID_10_MIN_AWAY); // 确保停止
            if (events & EVENT_TIMER_5_MIN_EXPIRED) {
                ESP_LOGI(TAG, "Timer event: Sprite decided to come home.");
                if ((rand() % 10) < 3) {
                    sprite_status.current_state = SPRITE_STATE_AT_HOME_SLEEPING;
                } else {
                    sprite_status.current_state = SPRITE_STATE_AT_HOME_AWAKE;
                }
                sprite_status.last_state_change_time = xTaskGetTickCount();
            }
            // 在此状态下，屏蔽新的“拍一拍”交互
            break;
        
        case SPRITE_STATE_EVENT_FLOODED:
            ESP_LOGI(TAG, "State: EVENT_FLOODED");
            app_timer_service_stop(TIMER_ID_10_MIN_AWAY); // 确保停止
            
            // 增加3秒的冷却时间，防止状态快速切换导致UI卡死
            if (sensor_data.adc_percent_ch0 <= 70.0f) {
                if (xTaskGetTickCount() - sprite_status.last_state_change_time > pdMS_TO_TICKS(3000)) {
                    ESP_LOGI(TAG, "Flood is over. Sprite is back to normal.");
                    if ((rand() % 3) == 0) {
                        ESP_LOGI(TAG, "Timer event: Sprite decided to come home.");
                        if ((rand() % 10) < 3) {
                            sprite_status.current_state = SPRITE_STATE_AT_HOME_SLEEPING;
                        } else {
                            sprite_status.current_state = SPRITE_STATE_AT_HOME_AWAKE;
                        }
                    } else {
                        ESP_LOGI(TAG, "Timer event: Sprite decided to stay away.");
                    }
                } else {
                    ESP_LOGD(TAG, "In FLOODED state cooldown...");
                }
            }
            break;

        default:
            ESP_LOGW(TAG, "Unhandled state: %d", sprite_status.current_state);
            sprite_status.current_state = SPRITE_STATE_AWAY_NORMAL;
            app_timer_service_stop(TIMER_ID_10_MIN_AWAY); // 确保停止
            break;
    }
}


/**
 * @brief 系统状态更新与决策函数
 *
 * 在每次事件循环中被调用，负责分发事件并驱动状态机运行。
 *
 * @param events 当前触发的事件标志位集合
 */
static void update_system_state(EventBits_t events)
{
    // 1. 预处理和日志记录 (可选)
    if (events & SENSOR_ADC_DATA_READY) {
        ESP_LOGD(TAG, "Processing ADC data: CH0=%.1f%%", sensor_data.adc_percent_ch0);
    }
    if (events & USER_INTERACTION_TAP) {
        ESP_LOGI(TAG, "User tap detected!");
    }
    if (events & EVENT_TIMER_48_HOUR_EXPIRED) {
        ESP_LOGI(TAG, "48H timer event triggered!");
    }
    if (events & SENSOR_TEMP_DATA_READY)
    {
        ESP_LOGI(TAG, "Temp data: %1.f",sensor_data.temperature);
    }
    // ... 其他事件处理

    // 2. 执行状态机逻辑
    sprite_state_machine_run(events);

    // 3. 统一更新UI
    // 将当前状态和湿度数据传递给UI控制器，由它负责所有UI更新
    // 在调用任何LVGL API之前，获取锁
    if (lvgl_port_lock(0)) {
        ui_controller_update(sprite_status.current_state, sensor_data.adc_percent_ch0);
        // 完成UI操作后，释放锁
        lvgl_port_unlock();
    }
}


/**
 * @brief 应用控制器的主任务函数
 *
 * @param param 任务参数 (未使用)
 */
static void app_controller_task(void *param)
{
    EventBits_t events;

    // -- UI 初始化 --
    // 在调用任何LVGL API之前，获取锁
    lvgl_port_lock(0);

    // 初始化UI布局和屏幕
    setup_ui(&guider_ui);
    
    // 初始化UI控制器模块
    ui_controller_init(&guider_ui);

    // 完成UI操作后，释放锁
    lvgl_port_unlock();
    
    // 初始化随机数种子
    srand(xTaskGetTickCount());

    while (1) {
        // 等待所有定义的事件
        // 注意：这里不需要再获取锁，因为状态机内部的set_ui_screen函数会处理锁
        events = xEventGroupWaitBits(
            controller_event_group,
            SENSOR_ADC_DATA_READY | SENSOR_TEMP_DATA_READY | SENSOR_LIGHT_DATA_READY |
            USER_INTERACTION_TAP |
            EVENT_TIMER_5_MIN_EXPIRED | EVENT_TIMER_48_HOUR_EXPIRED | EVENT_TIMER_10_MIN_AWAY_EXPIRED,
            pdTRUE,    // 清除事件位
            pdFALSE,   // 等待任意一个事件
            portMAX_DELAY // 无限等待
        );
        
        // 更新系统状态
        update_system_state(events);
        
        // 短暂延迟，以防事件风暴
        vTaskDelay(pdMS_TO_TICKS(50));
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

void app_controller_notify_tap(void)
{
    if (controller_event_group != NULL) {
        xEventGroupSetBits(controller_event_group, USER_INTERACTION_TAP);
    }
}

/**
 * @brief 向控制器发送一个通用事件
 *
 * @see app_controller.h
 */
void app_controller_notify_event(EventBits_t event_bit)
{
    if (controller_event_group != NULL) {
        // 此函数可以从任何任务或ISR安全的回调中调用
        xEventGroupSetBits(controller_event_group, event_bit);
    }
}
