/**
 * @file app_anim.h
 * @brief 应用程序动画调度器
 *
 * @details
 * 该模块负责管理和控制所有基于LVGL的动画。
 * 它提供了一个统一的接口，供上层应用逻辑（如app_controller）调用，
 * 以播放、停止或切换不同的动画效果，从而将核心逻辑与UI动画实现解耦。
 *
 * @see app_anim.c
 */

#ifndef APP_ANIM_H
#define APP_ANIM_H

#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**
 * @brief 定义了所有应用程序支持的动画类型
 */
typedef enum {
    APP_ANIM_IDLE,              /**< 0: 空闲/正常 (循环) */
    APP_ANIM_GREETING,          /**< 1: 打招呼 (单次) */
    APP_ANIM_SLEEPING,          /**< 2: 睡觉 (循环) */
    APP_ANIM_WAKING_UP,         /**< 3: 被叫醒 (单次) */
    APP_ANIM_LOST_SAD,          /**< 4: 迷路/缺水 (循环) */
    APP_ANIM_WATERING_GUIDE,    /**< 5: 浇水指引 (循环) */
    APP_ANIM_FLOODED_PANIC,     /**< 6: 淹水/惊慌 (循环) */
    APP_ANIM_GO_OUT,            /**< 7: 外出/消失 (单次) */
    APP_ANIM_COME_HOME,         /**< 8: 回家/出现 (单次) */
    APP_ANIM_ITEM_FOUND,        /**< 9: 带回物品 (单次) */
    APP_ANIM_COUNT              /**< 动画总数，用于数组大小 */
} app_anim_id_t;


/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief 初始化动画模块
 *
 * @details
 * - (未来) 加载所有动画资源
 * - (未来) 创建LVGL动画对象并进行初始设置
 * - **当前:** 仅作为占位符函数
 *
 * @param ui 指向全局 guider_ui 结构体的指针
 */
void app_anim_init(void *ui);

/**
 * @brief 播放指定的动画
 *
 * @details
 * 根据传入的动画ID，启动对应的动画。
 * 如果当前有其他动画正在播放，此函数会先停止它，再播放新的动画。
 *
 * @param anim_id 要播放的动画ID (app_anim_id_t)
 */
void app_anim_play(app_anim_id_t anim_id);

/**
 * @brief 停止当前正在播放的动画
 *
 */
void app_anim_stop(void);


#endif /* APP_ANIM_H */
