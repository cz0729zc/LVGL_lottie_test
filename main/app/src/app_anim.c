/**
 * @file app_anim.c
 * @brief 应用程序动画调度器实现
 *
 * @see app_anim.h
 */

#include "app_anim.h"
#include "ui_custom/gui_guider.h" // 需要访问UI对象
#include "ui_custom/events_init.h"
#include "lvgl.h"         // 需要Lottie动画的API
#include "esp_log.h"

/*********************
 *      DEFINES
 *********************/
static const char *TAG = "app_anim";

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_ui *p_ui = NULL;                  /**< 指向全局 guider_ui 结构体的指针 */
static app_anim_id_t current_anim_id = APP_ANIM_COUNT; /**< 当前正在播放的动画ID */

// 未来这里可以定义一个动画对象的数组
// static lv_anim_t anim_objects[APP_ANIM_COUNT];

/**********************
 *  FORWARD DECLARATIONS
 **********************/
static void anim_player_idle(bool play);
static void anim_player_flooded_panic(bool play);
// 为保持扩展性，未来可在此处添加更多动画播放器的声明
// static void anim_player_greeting(bool play);
// static void anim_player_sleeping(bool play);

/**********************
 *  STATIC FUNCTIONS
 **********************/

/**
 * @brief "正常/空闲" 动画的播放控制器
 * @param play true: 播放, false: 停止
 */
static void anim_player_idle(bool play)
{
    // if (p_ui && p_ui->screen_1_rlottie_1) {
    //     if (play) {
    //         // 确保动画在正确的屏幕上
    //         if (lv_scr_act() == p_ui->screen_1) {
    //             lv_obj_clear_flag(p_ui->screen_1_rlottie_1, LV_OBJ_FLAG_HIDDEN);
    //             lv_rlottie_set_play_mode(p_ui->screen_1_rlottie_1, LV_RLOTTIE_CTRL_LOOP);
    //         }
    //     } else {
    //         lv_obj_add_flag(p_ui->screen_1_rlottie_1, LV_OBJ_FLAG_HIDDEN);
    //     }
    // }
}

/**
 * @brief "淹水/惊慌" 动画的播放控制器
 * @param play true: 播放, false: 停止
 */
static void anim_player_flooded_panic(bool play)
{
    // if (p_ui && p_ui->screen_rlottie_1) {
    //     if (play) {
    //         // 确保动画在正确的屏幕上
    //         if (lv_scr_act() == p_ui->screen) {
    //             lv_obj_clear_flag(p_ui->screen_rlottie_1, LV_OBJ_FLAG_HIDDEN);
    //             lv_rlottie_set_play_mode(p_ui->screen_rlottie_1, LV_RLOTTIE_CTRL_LOOP);
    //         }
    //     } else {
    //         lv_obj_add_flag(p_ui->screen_rlottie_1, LV_OBJ_FLAG_HIDDEN);
    //     }
    // }
}


/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * @brief 初始化动画模块
 */
void app_anim_init(void *ui)
{
    if (ui == NULL) {
        ESP_LOGE(TAG, "UI object is NULL! Cannot initialize animations.");
        return;
    }
    p_ui = (lv_ui *)ui;
    ESP_LOGI(TAG, "Animation module initialized.");
    //setup_ui(&guider_ui);
    //events_init(&guider_ui);
    // TODO: 在这里添加从GUI Guider生成的动画初始化代码
    // 例如: setup_animation_idle(&p_ui->screen_img_1);
    //       setup_animation_greeting(&p_ui->screen_img_1);
    //       ...
}

/**
 * @brief 播放指定的动画
 */
void app_anim_play(app_anim_id_t anim_id)
{
    if (p_ui == NULL) {
        ESP_LOGE(TAG, "Cannot play animation, UI is not initialized.");
        return;
    }

    if (anim_id >= APP_ANIM_COUNT) {
        ESP_LOGW(TAG, "Invalid animation ID: %d", anim_id);
        return;
    }

    // 如果请求的动画已经在播放，则不执行任何操作
    if (current_anim_id == anim_id) {
        ESP_LOGD(TAG, "Animation %d is already playing.", anim_id);
        return;
    }

    // 停止当前动画
    app_anim_stop();

    ESP_LOGI(TAG, "Playing animation ID: %d", anim_id);
    current_anim_id = anim_id;

    // 分发到具体的动画播放器
    switch (anim_id) {
        case APP_ANIM_IDLE:
            ESP_LOGI(TAG, "Placeholder for: Start Normal Animation");
            //anim_player_idle(true);
            break;
        case APP_ANIM_FLOODED_PANIC:
            ESP_LOGI(TAG, "Placeholder for: Start FLOOD Animation");
            //anim_player_flooded_panic(true);
            break;
        
        // --- 以下为占位符，保持扩展性 ---
        case APP_ANIM_GREETING:
            ESP_LOGI(TAG, "Placeholder for: Start Greeting Animation");
            break;
        case APP_ANIM_SLEEPING:
            ESP_LOGI(TAG, "Placeholder for: Start Sleeping Animation");
            break;
        case APP_ANIM_WAKING_UP:
            ESP_LOGI(TAG, "Placeholder for: Start Waking Up Animation");
            break;
        case APP_ANIM_LOST_SAD:
            ESP_LOGI(TAG, "Placeholder for: Start Lost/Sad Animation");
            break;
        // ... 其他动画的case
        default:
            ESP_LOGW(TAG, "No implementation for animation ID: %d", anim_id);
            break;
    }
}

/**
 * @brief 停止当前正在播放的动画
 */
void app_anim_stop(void)
{
    if (current_anim_id >= APP_ANIM_COUNT) {
        // 没有正在播放的动画
        return;
    }

    ESP_LOGI(TAG, "Stopping animation ID: %d", current_anim_id);

    // 调用当前动画的播放器，并传入false来停止它
    switch (current_anim_id) {
        case APP_ANIM_IDLE:
            anim_player_idle(false);
            break;
        case APP_ANIM_FLOODED_PANIC:
            anim_player_flooded_panic(false);
            break;
        // ... 其他动画的case
        default:
            break;
    }

    current_anim_id = APP_ANIM_COUNT; // 重置为无效ID
}