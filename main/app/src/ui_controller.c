/**
 * @file ui_controller.c
 * @brief UI控制模块实现
 *
 * @see ui_controller.h
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "ui_custom/gui_guider.h"
#include "ui_custom/custom.h"
#include "ui_controller.h"
#include "esp_log.h"

// 定义日志标签
static const char *TAG = "ui_controller";

// 指向全局UI结构体的静态指针
static lv_ui *p_ui = NULL;

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

    // 0. 保存上一个状态，用于检测状态变化
    static sprite_state_t last_state = SPRITE_STATE_NULL;

    // 1. 隐藏所有动态UI元素，确保从一个干净的状态开始
    // 这样做可以避免不同状态的UI元素重叠
    lv_obj_add_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
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
    lv_obj_add_flag(p_ui->screen_state1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_ui->screen_state2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_ui->screen_state3, LV_OBJ_FLAG_HIDDEN);

    // 在密集的UI操作后，强制让出CPU，给系统一个喘息之机以重置看门狗
    vTaskDelay(pdMS_TO_TICKS(5));

    // 2. 根据当前状态，决定显示哪些UI元素和动画
    // 仅在状态发生变化时，才执行UI元素的设置和动画启动
    if (state != last_state) {
        ESP_LOGI(TAG, "UI state changed from %d to %d", last_state, state);
        switch (state) {
            case SPRITE_STATE_AT_HOME_AWAKE:
                // 显示背景
                lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                // 循环播放地面动画
                lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_ground, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_ground);
                // 循环播放空闲动画
                lv_obj_clear_flag(p_ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_idel, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_idel);
                // 播放两次开心动画
                lv_obj_clear_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_happy, 2);
                lv_animimg_start(p_ui->screen_animimg_exp_happy);
                break;

            case SPRITE_STATE_AT_HOME_SLEEPING:
                // 显示背景
                lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                // 循环播放地面动画
                lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_ground, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_ground);
                // 播放一次入睡动画
                lv_obj_clear_flag(p_ui->screen_animimg_idel_sleep, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_idel_sleep, 1);
                lv_animimg_start(p_ui->screen_animimg_idel_sleep);
                // 循环播放睡觉表情
                lv_obj_clear_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sleep, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_exp_sleep);
                break;

            case SPRITE_STATE_EVENT_FLOODED:
                // 显示背景
                lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                // 循环播放地面动画
                lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_ground, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_ground);
                // 播放一次淹水空闲动画
                lv_obj_clear_flag(p_ui->screen_animimg_idel_flood, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_idel_flood, 1);
                lv_animimg_start(p_ui->screen_animimg_idel_flood);
                // 循环播放悲伤表情
                lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_exp_sad);
                // 循环播放淹水动画
                lv_obj_clear_flag(p_ui->screen_animimg_flood, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_flood, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_flood);
                break;
            
            case SPRITE_STATE_AWAY_LOST:
                // 显示背景和地面
                lv_obj_clear_flag(p_ui->screen_background, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(p_ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_ground, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_ground);
                // 循环播放悲伤表情
                lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_exp_sad);
                break;

            // 为其他状态保留占位符，确保代码可扩展性
            case SPRITE_STATE_AWAY_NORMAL:
            case SPRITE_STATE_PREPARING_TO_GO_HOME:
                ESP_LOGD(TAG, "UI State: %d (No specific UI implemented yet)", state);
                // 可以在这里显示一个“外出”的静态图片或特定UI
                break;

            default:
                ESP_LOGW(TAG, "Unhandled UI state: %d", state);
                break;
        }
    }

    // 3. 根据湿度更新土壤状态图标 (这部分逻辑独立于上面的状态机)
    if (humidity < 20.0f) {
        lv_obj_clear_flag(p_ui->screen_state1, LV_OBJ_FLAG_HIDDEN); // 显示干燥状态
    } else if (humidity >= 20.0f && humidity <= 80.0f) {
        lv_obj_clear_flag(p_ui->screen_state2, LV_OBJ_FLAG_HIDDEN); // 显示正常状态
    } else { // humidity > 80.0f
        lv_obj_clear_flag(p_ui->screen_state3, LV_OBJ_FLAG_HIDDEN); // 显示过湿状态
    }

    // 4. 更新上一个状态
    last_state = state;
}
