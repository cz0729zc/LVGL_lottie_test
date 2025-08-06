/**
 * @file ui_controller.c
 * @brief UI控制模块实现
 *
 * @see ui_controller.h
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h" // 引入FreeRTOS定时器
#include "ui_custom/gui_guider.h"
#include "ui_custom/custom.h"
#include "ui_controller.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_random.h" // 添加ESP随机数生成器头文件
// 定义日志标签
static const char *TAG = "ui_controller";

// 指向全局UI结构体的静态指针
static lv_ui *p_ui = NULL;
// 用于“问候”动画序列的一次性定时器
static TimerHandle_t hello_anim_timer = NULL;
// 用于周期性表情提醒的周期性定时器
static TimerHandle_t expression_timer = NULL;

// 用于在定时器回调中安全访问的当前湿度缓存
static float current_humidity = 0.0f;

// 用于记录当前选择的外出背景 (0: background2, 1: background3)
static uint8_t current_away_background = 0;

// 前向声明定时器回调函数
static void hello_anim_timer_cb(TimerHandle_t xTimer);
static void expression_timer_cb(TimerHandle_t xTimer);

/**
 * @brief 初始化UI控制器
 */
void ui_controller_init(lv_ui *ui)
{
    if (ui == NULL) {
        ESP_LOGE(TAG, "UI object is NULL! Cannot initialize UI controller.");
        return;
    }
    p_ui = ui;

    // 创建一个一次性的软件定时器，用于处理动画序列
    // 定时器在被启动前不会做任何事
    hello_anim_timer = xTimerCreate(
        "hello_anim_timer",         // 定时器名称
        pdMS_TO_TICKS(2000),        // 定时器周期 (2秒)
        pdFALSE,                    // pdFALSE 表示这是一次性定时器
        (void *)0,                  // 定时器ID，这里未使用
        hello_anim_timer_cb         // 定时器到期时调用的回调函数
    );

    if (hello_anim_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create hello_anim_timer");
    }

    // 创建一个周期性的软件定时器，用于处理周期性表情动画
    expression_timer = xTimerCreate(
        "expression_timer",         // 定时器名称
        pdMS_TO_TICKS(10000),       // 定时器周期 (10秒)
        pdTRUE,                     // pdTRUE 表示这是周期性定时器
        (void *)1,                  // 定时器ID，这里未使用
        expression_timer_cb         // 定时器到期时调用的回调函数
    );

    if (expression_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create expression_timer");
    }

    ESP_LOGI(TAG, "UI controller initialized.");
}

/**
 * @brief 更新整个UI的显示状态
 */
void ui_controller_update(sprite_state_t state, float humidity)
{
    if (p_ui == NULL) {
        ESP_LOGE(TAG, "Cannot update UI, controller not initialized.");
        return;
    }

    // 更新当前湿度缓存，以便定时器回调可以访问
    current_humidity = humidity;

    // 静态变量，用于检测状态变化
    static sprite_state_t last_state = SPRITE_STATE_NULL;

    // 仅在主逻辑状态发生变化时，才执行重量级的UI重置和动画切换
    if (state != last_state) {
        ESP_LOGI(TAG, "UI state changed from %d to %d", last_state, state);

        // 根据状态变化，启动或停止周期性表情定时器
        bool is_at_home_now = (state == SPRITE_STATE_AT_HOME_AWAKE || state == SPRITE_STATE_AT_HOME_SLEEPING);
        bool was_at_home_before = (last_state == SPRITE_STATE_AT_HOME_AWAKE || last_state == SPRITE_STATE_AT_HOME_SLEEPING);

        if (is_at_home_now && !was_at_home_before) {
            ESP_LOGI(TAG, "Sprite is at home, starting expression timer.");
            xTimerStart(expression_timer, 0);
        } else if (!is_at_home_now && was_at_home_before) {
            ESP_LOGI(TAG, "Sprite left home, stopping expression timer.");
            xTimerStop(expression_timer, 0);
        }

        // 1. 隐藏所有与主逻辑状态相关的动画，为新状态做准备
        lv_obj_add_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_cominghome, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_idel_sleep, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_idel_awake, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_plant_normal, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_cominghome, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_idle_cominghome, LV_OBJ_FLAG_HIDDEN);

        // 在密集的UI操作后，强制让出CPU，给系统一个喘息之机以重置看门狗
        vTaskDelay(pdMS_TO_TICKS(5));

        // 2. 根据新状态，显示对应的UI元素和动画
        switch (state) {
            case SPRITE_STATE_AT_HOME_AWAKE:
                lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_ground, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_ground);
                lv_obj_clear_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_idel, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_idel);
                // 当进入此状态时，不再直接播放表情动画
                // 这个功能已移交给周期性定时器 expression_timer
                break;

            case SPRITE_STATE_AT_HOME_SLEEPING:
                lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_ground, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_ground);
                lv_obj_clear_flag(p_ui->screen_animimg_idel_sleep, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_idel_sleep, 1);
                lv_animimg_start(p_ui->screen_animimg_idel_sleep);
                lv_obj_clear_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sleep, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_exp_sleep);
                break;

            case SPRITE_STATE_EVENT_FLOODED:
                lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_ground, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_ground);
                lv_obj_clear_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_idel_flood, 1);
                lv_animimg_start(p_ui->screen_animimg_idel_flood);
                lv_obj_clear_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_flood, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_exp_flood);
                lv_obj_clear_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_flood, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_flood);
                break;
            
            case SPRITE_STATE_AWAY_LOST:
                lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_ground, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_ground);
                lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_exp_sad);
                break;

            case SPRITE_STATE_AWAY_NORMAL:
                ESP_LOGI(TAG, "UI State: SPRITE_STATE_AWAY_NORMAL");
                
                // 随机选择背景 (background2 或 background3)
                current_away_background = (esp_random() % 2); // 使用ESP32的随机数生成器
                ESP_LOGI(TAG, "Selected background: %d (0: background2, 1: background3)", current_away_background);
                
                if (current_away_background == 0) {
                    // 显示background2
                    lv_obj_clear_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
                    
                    // 只在background2中播放animimg_ground2动画
                    lv_obj_clear_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_set_repeat_count(p_ui->screen_animimg_ground2, LV_ANIM_PLAYTIME_INFINITE);
                    lv_animimg_start(p_ui->screen_animimg_ground2);
                } else {
                    // 显示background3
                    lv_obj_clear_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
                }
                
                // 显示小精灵idle动画
                lv_obj_clear_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_idel, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_idel);
                
                // 根据湿度显示表情
                if (humidity < 20.0f) {
                    // 湿度低，显示悲伤表情
                    lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, LV_ANIM_PLAYTIME_INFINITE);
                    lv_animimg_start(p_ui->screen_animimg_exp_sad);
                    
                    // 隐藏其他表情和淹水动画
                    lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                } else if (humidity >= 20.0f && humidity <= 80.0f) {
                    // 湿度正常，显示开心表情
                    lv_obj_clear_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_happy, LV_ANIM_PLAYTIME_INFINITE);
                    lv_animimg_start(p_ui->screen_animimg_exp_happy);
                    
                    // 隐藏其他表情和淹水动画
                    lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                } else { // humidity > 80.0f
                    // 湿度过高，显示淹水动画
                    lv_obj_clear_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_set_repeat_count(p_ui->screen_animimg_flood, LV_ANIM_PLAYTIME_INFINITE);
                    lv_animimg_start(p_ui->screen_animimg_flood);
                    
                    lv_obj_clear_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_set_repeat_count(p_ui->screen_animimg_idel_flood, LV_ANIM_PLAYTIME_INFINITE);
                    lv_animimg_start(p_ui->screen_animimg_idel_flood);
                    
                    lv_obj_clear_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_flood, LV_ANIM_PLAYTIME_INFINITE);
                    lv_animimg_start(p_ui->screen_animimg_exp_flood);
                    
                    // 如果湿度过高，隐藏普通idle动画
                    lv_obj_add_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
                    
                    // 隐藏其他表情
                    lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
                }
                break;

            case SPRITE_STATE_PREPARING_TO_GO_HOME:
                ESP_LOGI(TAG, "UI State: SPRITE_STATE_PREPARING_TO_GO_HOME");
                
                // 保持之前的背景选择
                if (current_away_background == 0) {
                    // 保持background2可见
                    lv_obj_clear_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
                    
                    // 保持animimg_ground2动画
                    lv_obj_clear_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_set_repeat_count(p_ui->screen_animimg_ground2, LV_ANIM_PLAYTIME_INFINITE);
                    lv_animimg_start(p_ui->screen_animimg_ground2);
                } else {
                    // 保持background3可见
                    lv_obj_clear_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
                }
                
                // 显示cominghome图层
                lv_obj_clear_flag(p_ui->screen_cominghome, LV_OBJ_FLAG_HIDDEN);
                
                // 隐藏普通idle动画和flood相关动画
                lv_obj_add_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                
                // 显示cominghome相关动画
                lv_obj_clear_flag(p_ui->screen_animimg_idle_cominghome, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_idle_cominghome, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_idle_cominghome);
                
                lv_obj_clear_flag(p_ui->screen_animimg_exp_cominghome, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_cominghome, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_exp_cominghome);
                break;
    
            default:
                ESP_LOGW(TAG, "Unhandled UI state: %d", state);
                break;
        }
        // 更新上一个状态
        last_state = state;
    }

    // 独立于主状态，随时根据湿度更新状态图标
    // 首先隐藏所有湿度图标
    lv_obj_add_flag(p_ui->screen_state1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_ui->screen_state2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_ui->screen_state3, LV_OBJ_FLAG_HIDDEN);
    // 然后根据当前湿度显示正确的那个
    if (humidity < 20.0f) {
        lv_obj_clear_flag(p_ui->screen_state1, LV_OBJ_FLAG_HIDDEN); // 显示干燥状态
    } else if (humidity >= 20.0f && humidity <= 80.0f) {
        lv_obj_clear_flag(p_ui->screen_state2, LV_OBJ_FLAG_HIDDEN); // 显示正常状态
    } else { // humidity > 80.0f
        lv_obj_clear_flag(p_ui->screen_state3, LV_OBJ_FLAG_HIDDEN); // 显示过湿状态
    }
}

/**
 * @brief "问候"动画的定时器回调函数
 * @details
 * 此函数在hello_anim_timer到期后（即“问候”动画播放2秒后）被调用。
 * 它的作用是无缝衔接，开始播放“开心”动画。
 * @param xTimer 指向触发此回调的定时器的句柄 (未使用)
 */
static void hello_anim_timer_cb(TimerHandle_t xTimer)
{
    if (p_ui == NULL) return;
    ESP_LOGI(TAG, "Hello animation timer expired, starting happy animation.");
    
    // 确保在UI操作期间获取锁
    if (lvgl_port_lock(0)) {
        // 隐藏“问候”和“悲伤”（如果正在播放）动画对象
        lv_obj_add_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);

        // 为了提供即时反馈，交互结束后，根据当前湿度立即播放一次表情动画
        if (current_humidity < 20.0f) {
            lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
            lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, 2);
            lv_animimg_start(p_ui->screen_animimg_exp_sad);
        } else {
            lv_obj_clear_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
            lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_happy, 2);
            lv_animimg_start(p_ui->screen_animimg_exp_happy);
        }

        // 释放锁
        lvgl_port_unlock();
    }
}

/**
 * @brief 播放一个“悲伤”动画序列
 */
void ui_controller_play_sad_animation(void)
{
    if (p_ui == NULL || hello_anim_timer == NULL) {
        ESP_LOGE(TAG, "Cannot play sad animation, controller not initialized or timer not created.");
        return;
    }

    ESP_LOGI(TAG, "Starting sad animation sequence.");

    // 确保在UI操作期间获取锁
    if (lvgl_port_lock(0)) {
        // 停止当前可能正在播放的任何表情动画，以“悲伤”为先
        lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);

        // 准备并播放“悲伤”动画
        lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
        lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, 2); // 播放两次以确保动画完整
        lv_animimg_start(p_ui->screen_animimg_exp_sad);

        // 启动2秒的定时器，时间到后它会调用回调函数来切换到“开心”动画
        // 我们复用同一个定时器和回调
        if (xTimerStart(hello_anim_timer, 0) != pdPASS) {
            ESP_LOGE(TAG, "Failed to start timer for sad animation sequence");
        }
        
        // 释放锁
        lvgl_port_unlock();
    }
}

/**
 * @brief 播放一个“问候”动画序列
 */
void ui_controller_play_hello_animation(void)
{
    if (p_ui == NULL || hello_anim_timer == NULL) {
        ESP_LOGE(TAG, "Cannot play hello animation, controller not initialized or timer not created.");
        return;
    }

    ESP_LOGI(TAG, "Starting hello animation sequence.");

    // 确保在UI操作期间获取锁
    if (lvgl_port_lock(0)) {
        // 停止当前可能正在播放的任何表情动画，以“问候”为先
        lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);

        // 准备并播放“问候”动画
        lv_obj_clear_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_hello, 2); // 只播放一次
        lv_animimg_start(p_ui->screen_animimg_exp_hello);

        // 启动2秒的定时器，时间到后它会调用回调函数来切换到“开心”动画
        if (xTimerStart(hello_anim_timer, 0) != pdPASS) {
            ESP_LOGE(TAG, "Failed to start hello_anim_timer");
        }
        
        // 释放锁
        lvgl_port_unlock();
    }
}

/**
 * @brief 周期性表情动画的定时器回调函数
 * @details
 * 此函数由 expression_timer 周期性调用 (例如每10秒)。
 * 它会检查当前的湿度，并播放一次性的“开心”或“悲伤”动画，
 * 以此来主动提醒用户小精灵的当前状态。
 * @param xTimer 指向触发此回调的定时器的句柄 (未使用)
 */
static void expression_timer_cb(TimerHandle_t xTimer)
{
    if (p_ui == NULL) return;
    ESP_LOGI(TAG, "Periodic expression timer expired. Playing an expression animation.");

    if (lvgl_port_lock(0)) {
        // 隐藏所有可能正在播放的表情，为新表情做准备
        lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);

        // 根据缓存的湿度值，播放一次性动画
        if (current_humidity < 20.0f) {
            ESP_LOGD(TAG, "Humidity is low (%.1f%%), playing sad animation.", current_humidity);
            lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
            lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, 3);
            lv_animimg_start(p_ui->screen_animimg_exp_sad);
        } else {
            ESP_LOGD(TAG, "Humidity is normal (%.1f%%), playing happy animation.", current_humidity);
            lv_obj_clear_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
            lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_happy, 3);
            lv_animimg_start(p_ui->screen_animimg_exp_happy);
        }
        lvgl_port_unlock();
    }
}

