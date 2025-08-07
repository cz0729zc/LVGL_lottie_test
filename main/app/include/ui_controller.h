/**
 * @file ui_controller.h
 * @brief UI控制模块头文件
 *
 * @details
 * 该模块作为应用逻辑 (app_controller) 和具体UI实现 (gui_guider) 之间的中间层。
 * 它负责将高级状态指令转换为具体的LVGL UI操作，从而实现业务逻辑与UI的解耦。
 * app_controller只需关心状态（如“在家睡觉”、“被水淹了”）和关键数据（如湿度），
 * 而ui_controller则处理所有显示/隐藏对象、播放动画等细节。
 */
#ifndef _UI_CONTROLLER_H_
#define _UI_CONTROLLER_H_

#include "app_controller.h" // 需要 sprite_state_t

/**
 * @brief 初始化UI控制器
 *
 * @details
 * 保存一个指向全局UI结构体 (guider_ui) 的指针，为后续操作做准备。
 * 此函数应在UI和app_controller初始化之后调用。
 *
 * @param ui 指向全局 guider_ui 结构体的指针。
 */
void ui_controller_init(lv_ui *ui);

/**
 * @brief 更新整个UI的显示状态
 *
 * @details
 * 这是UI控制器的核心函数。app_controller在每次状态变更或数据更新后调用此函数。
 * 它会根据传入的状态和湿度，统一管理和更新屏幕上的所有UI元素。
 *
 * @param state 当前的小精灵逻辑状态 (sprite_state_t)。
 * @param humidity 当前的土壤湿度百分比 (0-100)。
 */
void ui_controller_update(sprite_state_t state, float humidity);

/**
 * @brief 播放一个“问候”动画序列
 * @details
 * 这是一个非阻塞的函数。它会启动一个“问候”动画，
 * 并在该动画播放完毕后，通过定时器回调自动切换到“开心”动画。
 * 这用于响应用户的“拍一拍”交互。
 */
void ui_controller_play_hello_animation(void);

/**
 * @brief 播放“植物”动画序列
 * @details
 * 这是一个非阻塞的函数。它会启动一个“植物”动画播放两次，
 * 并在播放完毕后通过定时器自动隐藏。
 * 这用于响应从外出归家的事件。
 */
void ui_controller_play_plant_animation(void);

/**
 * @brief 播放一个“悲伤”动画序列
 * @details
 * 这是一个非阻塞的函数。它会启动一个“悲伤”动画，
 * 并在该动画播放完毕后，通过定时器回调自动切换到“开心”动画。
 * 这用于在特定条件下（如缺水）响应用户的“拍一拍”交互。
 */
void ui_controller_play_sad_animation(void);

#endif // _UI_CONTROLLER_H_
