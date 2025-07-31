/**
 * @file app_timer_service.h
 * @brief 通用定时器服务，用于管理所有应用级定时器。
 *
 * 该服务提供统一的接口来启动、停止和管理应用所需的各种定时器，
 * 例如一次性或周期性定时器。它封装了底层的FreeRTOS定时器实现。
 */
#ifndef APP_TIMER_SERVICE_H
#define APP_TIMER_SERVICE_H

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_err.h"

/**
 * @brief 系统中所有定时器的唯一标识符。
 *
 * 该枚举允许应用的不同部分通过ID来引用特定的定时器，
 * 而无需关心其实现细节。
 */
typedef enum {
    TIMER_ID_5_MIN_GO_HOME,   /**< 用于触发“回家”事件的5分钟一次性定时器。 */
    TIMER_ID_48_HOUR_REMINDER,  /**< 用于通知的48小时周期性定时器（未来使用）。 */
    // 在此按需添加更多定时器ID
    TIMER_ID_MAX,               /**< 已定义定时器的总数。 */
} app_timer_id_t;

/**
 * @brief 初始化定时器服务。
 *
 * 此函数创建所有预定义的定时器对象。它必须在应用启动时调用一次，
 * 且在调用任何其他定时器函数之前。
 *
 * @return esp_err_t
 *         - ESP_OK: 初始化成功。
 *         - ESP_FAIL: 任何一个定时器创建失败。
 */
esp_err_t app_timer_service_init(void);

/**
 * @brief 根据ID启动一个指定的定时器。
 *
 * @param timer_id 要启动的定时器的ID。
 * @return esp_err_t
 *         - ESP_OK: 成功。
 *         - ESP_ERR_INVALID_ARG: 定时器ID无效。
 *         - ESP_FAIL: 定时器启动失败。
 */
esp_err_t app_timer_service_start(app_timer_id_t timer_id);

/**
 * @brief 根据ID停止一个指定的定时器。
 *
 * @param timer_id 要停止的定时器的ID。
 * @return esp_err_t
 *         - ESP_OK: 成功。
 *         - ESP_ERR_INVALID_ARG: 定时器ID无效。
 *         - ESP_FAIL: 定时器停止失败。
 */
esp_err_t app_timer_service_stop(app_timer_id_t timer_id);

#endif // APP_TIMER_SERVICE_H