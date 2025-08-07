/**
 * @file ui_controller.c
 * @brief UI控制模块实现，负责将应用状态转换为具体的UI展现。
 *
 * @see ui_controller.h
 * @details
 * 此模块作为应用逻辑 (app_controller) 和LVGL UI库之间的桥梁。
 * 它引入了过渡动画逻辑，以在主要状态（在家/外出）之间提供平滑的视觉切换。
 * 通过内部状态管理和定时器，它封装了复杂的UI操作，使主逻辑更清晰。
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "stdbool.h"
#include "ui_custom/gui_guider.h"
#include "ui_custom/custom.h"
#include "ui_controller.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_random.h"

// 日志标签
static const char *TAG = "ui_controller";

// 全局UI结构体指针
static lv_ui *p_ui = NULL;

// --- 定时器句柄 ---
static TimerHandle_t hello_anim_timer = NULL;   // "问候"动画序列定时器
static TimerHandle_t expression_timer = NULL; // 周期性表情动画定时器
static TimerHandle_t transition_timer = NULL; // 状态切换过场动画定时器

// --- 状态缓存 ---
static float current_humidity = 0.0f;       // 当前湿度缓存
static uint8_t current_away_background = 0; // 当前外出背景选择

// --- 过渡动画状态跟踪 ---
static struct {
    bool is_in_transition;           // 是否正处于过场动画中
    sprite_state_t target_state;     // 过渡结束后的目标状态
    float target_humidity;           // 过渡结束时要使用的湿度值
} transition_data = { .is_in_transition = false, .target_state = SPRITE_STATE_NULL, .target_humidity = 0.0f };

// --- 函数前向声明 ---
static void hello_anim_timer_cb(TimerHandle_t xTimer);
static void expression_timer_cb(TimerHandle_t xTimer);
static void transition_timer_cb(TimerHandle_t xTimer);
static void apply_ui_update(sprite_state_t state, float humidity);

/**
 * @brief 初始化UI控制器，创建所有需要的定时器。
 * @param ui 指向全局 guider_ui 结构体的指针。
 */
void ui_controller_init(lv_ui *ui) {
    if (ui == NULL) {
        ESP_LOGE(TAG, "UI object is NULL! Cannot initialize.");
        return;
    }
    p_ui = ui;

    hello_anim_timer = xTimerCreate("hello_anim_timer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, hello_anim_timer_cb);
    expression_timer = xTimerCreate("expression_timer", pdMS_TO_TICKS(10000), pdTRUE, (void *)1, expression_timer_cb);
    transition_timer = xTimerCreate("transition_timer", pdMS_TO_TICKS(3000), pdFALSE, (void *)2, transition_timer_cb);

    if (!hello_anim_timer || !expression_timer || !transition_timer) {
        ESP_LOGE(TAG, "Failed to create one or more timers.");
    }

    ESP_LOGI(TAG, "UI controller initialized.");
}

/**
 * @brief 更新UI的入口函数，包含过场动画逻辑。
 * @param state 新的应用状态。
 * @param humidity 当前湿度。
 */
void ui_controller_update(sprite_state_t state, float humidity) {
    if (p_ui == NULL) return;

    current_humidity = humidity;

    if (transition_data.is_in_transition) {
        ESP_LOGD(TAG, "In transition, ignoring UI update for state: %d", state);
        return;
    }

    static sprite_state_t last_state = SPRITE_STATE_NULL;

    if (state != last_state) {
        ESP_LOGI(TAG, "UI state changed from %d to %d", last_state, state);

        bool is_at_home_now = (state == SPRITE_STATE_AT_HOME_AWAKE || state == SPRITE_STATE_AT_HOME_SLEEPING);
        bool was_at_home_before = (last_state == SPRITE_STATE_AT_HOME_AWAKE || last_state == SPRITE_STATE_AT_HOME_SLEEPING);
        bool is_away_now = (state >= SPRITE_STATE_AWAY_NORMAL && state <= SPRITE_STATE_PREPARING_TO_GO_HOME);
        bool was_away_before = (last_state >= SPRITE_STATE_AWAY_NORMAL && last_state <= SPRITE_STATE_PREPARING_TO_GO_HOME);

        if ((is_at_home_now && was_away_before) || (is_away_now && was_at_home_before)) {
            ESP_LOGI(TAG, "Transitioning between home/away, starting loading animation.");
            
            transition_data.target_state = state;
            transition_data.target_humidity = humidity;
            transition_data.is_in_transition = true;
            
            if (lvgl_port_lock(0)) {
                // 隐藏所有可能显示的图层，为过场动画做准备
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

                // 显示过场动画
                lv_obj_clear_flag(p_ui->screen_animimg_lodding, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_lodding, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_lodding);
                
                lvgl_port_unlock();
            }
            
            if (xTimerStart(transition_timer, 0) != pdPASS) {
                ESP_LOGE(TAG, "Failed to start transition_timer, skipping transition.");
                transition_data.is_in_transition = false;
                apply_ui_update(state, humidity);
            }
        } else {
            apply_ui_update(state, humidity);
        }
        last_state = state;
    } else {
        // 如果主状态未变，只更新需要持续刷新的部分（如湿度状态图标）
        if (lvgl_port_lock(0)) {
            lv_obj_add_flag(p_ui->screen_state1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(p_ui->screen_state2, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(p_ui->screen_state3, LV_OBJ_FLAG_HIDDEN);
            if (humidity < 20.0f) lv_obj_clear_flag(p_ui->screen_state1, LV_OBJ_FLAG_HIDDEN);
            else if (humidity <= 80.0f) lv_obj_clear_flag(p_ui->screen_state2, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(p_ui->screen_state3, LV_OBJ_FLAG_HIDDEN);
            lvgl_port_unlock();
        }
    }
}

/**
 * @brief 应用实际的UI状态更新（不含过场动画逻辑）。
 * @param state 要应用的目标状态。
 * @param humidity 对应的湿度值。
 */
static void apply_ui_update(sprite_state_t state, float humidity) {
    if (p_ui == NULL) return;

    if (lvgl_port_lock(0)) {
        // 隐藏所有图层，为新状态做准备
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
        lv_obj_add_flag(p_ui->screen_animimg_lodding, LV_OBJ_FLAG_HIDDEN);
        // 确保 lost 动画默认隐藏
        lv_obj_add_flag(p_ui->screen_animimg_exp_lost, LV_OBJ_FLAG_HIDDEN);


        // 根据新状态显示对应UI
        switch (state) {
            case SPRITE_STATE_AT_HOME_AWAKE:
            case SPRITE_STATE_AT_HOME_SLEEPING:
                // 优先处理水淹情况，避免UI闪烁
                if (humidity > 80.0f) {
                    lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_idel_flood);
                    lv_obj_clear_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_exp_flood);
                    lv_obj_clear_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_flood);
                    xTimerStop(expression_timer, 0);
                } else if (state == SPRITE_STATE_AT_HOME_AWAKE) {
                    lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_ground);
                    lv_obj_clear_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_idel);
                    xTimerStart(expression_timer, 0);
                } else { // SPRITE_STATE_AT_HOME_SLEEPING
                    lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_ground);
                    lv_obj_clear_flag(p_ui->screen_animimg_idel_sleep, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_idel_sleep);
                    lv_obj_clear_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_exp_sleep);
                    xTimerStart(expression_timer, 0);
                }
                break;

            case SPRITE_STATE_EVENT_FLOODED:
                //lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                if (current_away_background == 0) {
                    lv_obj_clear_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_ground2);
                } else {
                    lv_obj_clear_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
                }
                lv_obj_clear_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_start(p_ui->screen_animimg_idel_flood);
                lv_obj_clear_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_start(p_ui->screen_animimg_exp_flood);
                lv_obj_clear_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_start(p_ui->screen_animimg_flood);
                xTimerStop(expression_timer, 0);
                break;
            
            case SPRITE_STATE_AWAY_LOST:
                // 保持当前的外出背景
                if (current_away_background == 0) {
                    lv_obj_clear_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_ground2);
                } else {
                    lv_obj_clear_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
                }
                // 显示身体和 lost 表情
                lv_obj_clear_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_start(p_ui->screen_animimg_idel);
                lv_obj_clear_flag(p_ui->screen_animimg_exp_lost, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_lost, -1);
                lv_animimg_start(p_ui->screen_animimg_exp_lost);
                xTimerStop(expression_timer, 0);
                break;

            case SPRITE_STATE_AWAY_NORMAL:
                current_away_background = (esp_random() % 2);
                if (current_away_background == 0) {
                    lv_obj_clear_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_ground2);
                } else {
                    lv_obj_clear_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
                }
                if (humidity > 80.0f) {
                    // 湿度过高，播放水淹动画
                    lv_obj_clear_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_idel_flood);
                    lv_obj_clear_flag(p_ui->screen_animimg_exp_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_exp_flood);
                    lv_obj_clear_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_flood);
                } else {
                    // 正常或干燥，播放对应动画
                    lv_obj_clear_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_idel);
                    if (humidity < 20.0f) {
                        lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                        lv_animimg_start(p_ui->screen_animimg_exp_sad);
                    } else { // humidity <= 80.0f
                        lv_obj_clear_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
                        lv_animimg_start(p_ui->screen_animimg_exp_happy);
                    }
                }
                xTimerStop(expression_timer, 0);
                break;

            case SPRITE_STATE_PREPARING_TO_GO_HOME:
                if (current_away_background == 0) {
                    lv_obj_clear_flag(p_ui->screen_background2, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(p_ui->screen_animimg_ground2, LV_OBJ_FLAG_HIDDEN);
                    lv_animimg_start(p_ui->screen_animimg_ground2);
                } else {
                    lv_obj_clear_flag(p_ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
                }
                lv_obj_clear_flag(p_ui->screen_cominghome, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(p_ui->screen_animimg_idle_cominghome, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_start(p_ui->screen_animimg_idle_cominghome);
                lv_obj_clear_flag(p_ui->screen_animimg_exp_cominghome, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_start(p_ui->screen_animimg_exp_cominghome);
                xTimerStop(expression_timer, 0);
                break;
    
            default:
                ESP_LOGW(TAG, "Unhandled UI state: %d", state);
                break;
        }

        // 更新湿度状态图标
        lv_obj_add_flag(p_ui->screen_state1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_state2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_state3, LV_OBJ_FLAG_HIDDEN);
        if (humidity < 20.0f) lv_obj_clear_flag(p_ui->screen_state1, LV_OBJ_FLAG_HIDDEN);
        else if (humidity <= 80.0f) lv_obj_clear_flag(p_ui->screen_state2, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_clear_flag(p_ui->screen_state3, LV_OBJ_FLAG_HIDDEN);

        lvgl_port_unlock();
    }
}

/**
 * @brief 过场动画定时器回调函数。
 */
static void transition_timer_cb(TimerHandle_t xTimer) {
    if (p_ui == NULL) return;
    ESP_LOGI(TAG, "Transition animation completed, applying target state: %d", transition_data.target_state);
    
    transition_data.is_in_transition = false;
    apply_ui_update(transition_data.target_state, transition_data.target_humidity);
}

/**
 * @brief "问候"动画序列的定时器回调。
 */
static void hello_anim_timer_cb(TimerHandle_t xTimer) {
    if (p_ui == NULL) return;
    ESP_LOGI(TAG, "Hello animation timer expired.");
    
    if (lvgl_port_lock(0)) {
        lv_obj_add_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        if (current_humidity < 20.0f) {
            lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
            lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, 2);
            lv_animimg_start(p_ui->screen_animimg_exp_sad);
        } else {
            lv_obj_clear_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
            lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_happy, 2);
            lv_animimg_start(p_ui->screen_animimg_exp_happy);
        }
        lvgl_port_unlock();
    }
}

/**
 * @brief 播放“悲伤”动画序列。
 */
void ui_controller_play_sad_animation(void) {
    if (p_ui == NULL || hello_anim_timer == NULL) return;
    ESP_LOGI(TAG, "Starting sad animation sequence.");
    if (lvgl_port_lock(0)) {
        lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
        lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, 2);
        lv_animimg_start(p_ui->screen_animimg_exp_sad);
        xTimerStart(hello_anim_timer, 0);
        lvgl_port_unlock();
    }
}

/**
 * @brief 播放“问候”动画序列。
 */
void ui_controller_play_hello_animation(void) {
    if (p_ui == NULL || hello_anim_timer == NULL) return;
    ESP_LOGI(TAG, "Starting hello animation sequence.");
    if (lvgl_port_lock(0)) {
        lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_hello, 2);
        lv_animimg_start(p_ui->screen_animimg_exp_hello);
        xTimerStart(hello_anim_timer, 0);
        lvgl_port_unlock();
    }
}

/**
 * @brief 周期性表情动画的定时器回调。
 */
static void expression_timer_cb(TimerHandle_t xTimer) {
    if (p_ui == NULL) return;
    ESP_LOGI(TAG, "Periodic expression timer expired.");
    if (lvgl_port_lock(0)) {
        lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
        if (current_humidity < 20.0f) {
            lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
            lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, 3);
            lv_animimg_start(p_ui->screen_animimg_exp_sad);
        } else {
            lv_obj_clear_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
            lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_happy, 3);
            lv_animimg_start(p_ui->screen_animimg_exp_happy);
        }
        lvgl_port_unlock();
    }
}
