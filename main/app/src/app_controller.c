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
#include "app_anim.h" // 包含动画调度器头文件
#include "esp_log.h"
#include <math.h>
#include "freertos/timers.h" // 引入定时器头文件
#include "app_timer_service.h" // 引入新的定时器服务

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

/** @brief 定义UI屏幕的枚举 */
typedef enum {
    UI_SCREEN_UNKNOWN = -1,
    UI_SCREEN_NORMAL,       // 正常状态下的主屏幕 (screen_1)
    UI_SCREEN_FLOODED,      // 淹水状态的屏幕 (screen)
} ui_screen_t;

/** @brief 当前显示的屏幕，用于避免重复加载 */
static ui_screen_t current_screen = UI_SCREEN_UNKNOWN;

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
    // 初始化状态
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
    // 1. 检查湿度是否过低
    if (sensor_data.adc_percent_ch0 < 20.0f) {
        // 如果角色在外（正常或准备回家），则强制进入迷路状态
        if (sprite_status.current_state == SPRITE_STATE_AWAY_NORMAL || sprite_status.current_state == SPRITE_STATE_PREPARING_TO_GO_HOME) {
            if (sprite_status.current_state != SPRITE_STATE_AWAY_LOST) {
                ESP_LOGW(TAG, "Humidity too low! Sprite is now LOST.");
                sprite_status.current_state = SPRITE_STATE_AWAY_LOST;
                sprite_status.last_state_change_time = xTaskGetTickCount();
                // 如果“准备回家”的定时器正在运行，通过服务停止它
                app_timer_service_stop(TIMER_ID_5_MIN_GO_HOME);
            }
        }
    }
    // 2. 检查湿度是否过高
    else if (sensor_data.adc_percent_ch0 > 80.0f) { // 湿度过高 -> 淹水
        if (sprite_status.current_state != SPRITE_STATE_EVENT_FLOODED) {
            ESP_LOGE(TAG, "Humidity too high (%.1f%%)! FLOODED.", sensor_data.adc_percent_ch0);
            sprite_status.current_state = SPRITE_STATE_EVENT_FLOODED;
            sprite_status.last_state_change_time = xTaskGetTickCount();
        }
    }


    // 根据当前状态处理特定的事件和逻辑
    switch (sprite_status.current_state) {
        case SPRITE_STATE_AT_HOME_AWAKE:
            ESP_LOGI(TAG, "State: AT_HOME_AWAKE");
            if (current_screen != UI_SCREEN_NORMAL) {
                ESP_LOGI(TAG, "Switching to NORMAL screen.");
                //ui_load_scr_animation(&guider_ui, &guider_ui.screen_1, guider_ui.screen_1_del, &guider_ui.screen_del, setup_scr_screen_1, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false, true);
                current_screen = UI_SCREEN_NORMAL;
            }
            //app_anim_play(APP_ANIM_IDLE); // 播放空闲动画
            if (events & USER_INTERACTION_TAP) {
                ESP_LOGI(TAG, "Interaction: Pat the sprite, sprite says hello!");
                app_anim_play(APP_ANIM_GREETING); // 播放打招呼动画
            }
            // 5分钟周期定时器的逻辑现已移除。
            // 如果需要，它将被一个更明确的机制所取代。
            break;

        case SPRITE_STATE_AT_HOME_SLEEPING:
            ESP_LOGI(TAG, "State: AT_HOME_SLEEPING");
            if (current_screen != UI_SCREEN_NORMAL) {
                ESP_LOGI(TAG, "Switching to NORMAL screen.");
                // ui_load_scr_animation(&guider_ui, &guider_ui.screen_1, guider_ui.screen_1_del, &guider_ui.screen_del, setup_scr_screen_1, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false, true);
                current_screen = UI_SCREEN_NORMAL;
            }
            app_anim_play(APP_ANIM_SLEEPING); // 播放睡觉动画
            if (events & USER_INTERACTION_TAP) {
                ESP_LOGI(TAG, "Interaction: Pat the sprite, waking it up.");
                app_anim_play(APP_ANIM_WAKING_UP); // 播放被叫醒动画
                sprite_status.current_state = SPRITE_STATE_AT_HOME_AWAKE;
            }
            // 5分钟周期定时器的逻辑现已移除。
            break;

        case SPRITE_STATE_AWAY_NORMAL:
            ESP_LOGI(TAG, "State: AWAY_NORMAL");
            if (current_screen != UI_SCREEN_NORMAL) {
                ESP_LOGI(TAG, "Switching to NORMAL screen.");
                //ui_load_scr_animation(&guider_ui, &guider_ui.screen_1, guider_ui.screen_1_del, &guider_ui.screen_del, setup_scr_screen_1, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false, true);
                current_screen = UI_SCREEN_NORMAL;
            }
            // 外出时可能没有特定动画，或者是一个“空”的动画
            if (events & USER_INTERACTION_TAP) {
                ESP_LOGI(TAG, "Interaction: Pat the sprite, preparing to go home.");
                sprite_status.current_state = SPRITE_STATE_PREPARING_TO_GO_HOME;
                sprite_status.last_state_change_time = xTaskGetTickCount();
                app_timer_service_start(TIMER_ID_5_MIN_GO_HOME); // 通过服务启动5分钟回家倒计时
            }
            break;

        case SPRITE_STATE_AWAY_LOST:
            ESP_LOGI(TAG, "State: AWAY_LOST");
            // 播放迷路动画
            app_anim_play(APP_ANIM_LOST_SAD);
            // 检查湿度是否恢复
            if (sensor_data.adc_percent_ch0 >= 20.0f) {
                ESP_LOGI(TAG, "Humidity is back to normal. Sprite is no longer lost.");
                sprite_status.current_state = SPRITE_STATE_AWAY_NORMAL;
                sprite_status.last_state_change_time = xTaskGetTickCount();
            }
            break;

        case SPRITE_STATE_PREPARING_TO_GO_HOME:
            ESP_LOGI(TAG, "State: PREPARING_TO_GO_HOME");
            // 在此状态播放一个“准备”或“收拾”的动画
            // app_anim_play(APP_ANIM_PREPARING);
            if (events & EVENT_TIMER_5_MIN_EXPIRED) {
                ESP_LOGI(TAG, "Go home timer expired. Starting to go home.");
                app_anim_play(APP_ANIM_COME_HOME); // 播放回家动画

                ESP_LOGI(TAG, "Timer event: Sprite decided to come home.");
                app_anim_play(APP_ANIM_COME_HOME);
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
            if (current_screen != UI_SCREEN_FLOODED) {
                ESP_LOGI(TAG, "Switching to FLOODED screen.");
                ui_load_scr_animation(&guider_ui, &guider_ui.screen, guider_ui.screen_del, &guider_ui.screen_1_del, setup_scr_screen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false, true);
                current_screen = UI_SCREEN_FLOODED;
            }
            app_anim_play(APP_ANIM_FLOODED_PANIC); // 播放淹水动画
            
            // 增加3秒的冷却时间，防止状态快速切换导致UI卡死
            if (sensor_data.adc_percent_ch0 <= 70.0f) {
                if (xTaskGetTickCount() - sprite_status.last_state_change_time > pdMS_TO_TICKS(3000)) {
                    ESP_LOGI(TAG, "Flood is over. Sprite is back to normal.");
                    if ((rand() % 3) == 0) {
                        ESP_LOGI(TAG, "Timer event: Sprite decided to come home.");
                        app_anim_play(APP_ANIM_COME_HOME);
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
}


/**
 * @brief 应用控制器的主任务函数
 *
 * @param param 任务参数 (未使用)
 */
static void app_controller_task(void *param)
{
    EventBits_t events;
    
    // 初始化随机数种子
    srand(xTaskGetTickCount());

    while (1) {
        // 等待所有定义的事件
        events = xEventGroupWaitBits(
            controller_event_group,
            SENSOR_ADC_DATA_READY | SENSOR_TEMP_DATA_READY | SENSOR_LIGHT_DATA_READY |
            USER_INTERACTION_TAP |
            EVENT_TIMER_5_MIN_EXPIRED | EVENT_TIMER_48_HOUR_EXPIRED,
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