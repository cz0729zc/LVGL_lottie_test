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

    // 静态变量，用于检测状态变化
    static sprite_state_t last_state = SPRITE_STATE_NULL;

    // 仅在主逻辑状态发生变化时，才执行重量级的UI重置和动画切换
    if (state != last_state) {
        ESP_LOGI(TAG, "UI state changed from %d to %d", last_state, state);

        // 1. 隐藏所有与主逻辑状态相关的动画，为新状态做准备
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
                lv_obj_clear_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_happy, 2);
                lv_animimg_start(p_ui->screen_animimg_exp_happy);
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
                lv_obj_clear_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
                lv_animimg_set_repeat_count(p_ui->screen_animimg_exp_sad, LV_ANIM_PLAYTIME_INFINITE);
                lv_animimg_start(p_ui->screen_animimg_exp_sad);
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
            case SPRITE_STATE_PREPARING_TO_GO_HOME:
                ESP_LOGD(TAG, "UI State: %d (No specific UI implemented yet)", state);
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
