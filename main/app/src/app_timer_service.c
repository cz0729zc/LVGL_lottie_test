/**
 * @file app_timer_service.c
 * @brief 通用定时器服务的实现。
 */
#include "app_timer_service.h"
#include "app_controller.h" // 用于通知控制器定时器事件
#include "esp_log.h"

// 本模块的日志标签
static const char *TAG = "timer_service";

/**
 * @brief 每个定时器的内部配置结构体。
 */
typedef struct {
    TimerHandle_t handle;       /**< FreeRTOS 定时器句柄。 */
    const char* name;           /**< 用于日志记录的描述性名称。 */
    uint32_t period_ms;         /**< 定时器周期（毫秒）。 */
    UBaseType_t is_periodic;    /**< pdTRUE 表示周期性，pdFALSE 表示一次性。 */
    EventBits_t event_bit;      /**< 定时器到期时，在控制器事件组中设置的事件位。 */
} timer_config_t;

// 本服务管理的所有定时器的配置表。
// 此表将 app_timer_id_t 中的定时器ID映射到其具体配置。
static timer_config_t timer_configs[TIMER_ID_MAX] = {
    [TIMER_ID_5_MIN_GO_HOME] = {
        .handle = NULL,
        .name = "5分钟回家定时器",
        .period_ms = 1 * 60 * 1000, // 5 分钟
        .is_periodic = pdFALSE,     // 一次性定时器
        .event_bit = EVENT_TIMER_5_MIN_EXPIRED
    },
    [TIMER_ID_48_HOUR_REMINDER] = {
        .handle = NULL,
        .name = "48小时提醒定时器",
        .period_ms = 48 * 3600 * 1000, // 48 小时
        .is_periodic = pdTRUE,      // 周期性定时器
        .event_bit = EVENT_TIMER_48_HOUR_EXPIRED
    },
    [TIMER_ID_10_MIN_AWAY] = {
        .handle = NULL,
        .name = "10分钟离家定时器",
        .period_ms = 2 * 60 * 1000, // 10 分钟
        .is_periodic = pdFALSE,      // 一次性定时器
        .event_bit = EVENT_TIMER_10_MIN_AWAY_EXPIRED
    },
};

/**
 * @brief 本服务创建的所有FreeRTOS定时器的统一回调函数。
 *
 * 当定时器到期时，此函数被调用。它通过检索定时器ID来识别是哪个定时器触发了，
 * 然后将相应的事件位发布到app_controller的事件组中。
 *
 * @param xTimer 到期的定时器的句柄。
 */
static void unified_timer_callback(TimerHandle_t xTimer) {
    // 定时器的ID在创建时已存储在其pvTimerID成员中。
    app_timer_id_t timer_id = (app_timer_id_t)(pvTimerGetTimerID(xTimer));

    if (timer_id < TIMER_ID_MAX) {
        ESP_LOGI(TAG, "定时器 '%s' 到期, 发送事件位: %lu", timer_configs[timer_id].name, timer_configs[timer_id].event_bit);
        // 通知应用控制器定时器事件已发生。
        app_controller_notify_event(timer_configs[timer_id].event_bit);
    } else {
        ESP_LOGE(TAG, "定时器回调收到无效ID。");
    }
}

/**
 * @brief 初始化定时器服务。
 * @see app_timer_service.h
 */
esp_err_t app_timer_service_init(void) {
    ESP_LOGI(TAG, "正在初始化定时器服务...");
    for (int i = 0; i < TIMER_ID_MAX; i++) {
        // 根据配置表创建FreeRTOS定时器对象。
        timer_configs[i].handle = xTimerCreate(
            timer_configs[i].name,
            pdMS_TO_TICKS(timer_configs[i].period_ms),
            timer_configs[i].is_periodic,
            (void*)i, // 使用定时器的枚举值作为其唯一ID。
            unified_timer_callback
        );

        if (timer_configs[i].handle == NULL) {
            ESP_LOGE(TAG, "创建定时器 '%s' 失败", timer_configs[i].name);
            // 在实际应用中，我们可能需要在此处清理已创建的定时器。
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "定时器服务初始化成功。");
    return ESP_OK;
}

/**
 * @brief 根据ID启动一个指定的定时器。
 * @see app_timer_service.h
 */
esp_err_t app_timer_service_start(app_timer_id_t timer_id) {
    // 参数验证
    if (timer_id >= TIMER_ID_MAX) {
        ESP_LOGE(TAG, "尝试启动无效的定时器ID: %d", timer_id);
        return ESP_ERR_INVALID_ARG;
    }

    // 在使用之前确保定时器句柄有效。
    if (timer_configs[timer_id].handle == NULL) {
        ESP_LOGE(TAG, "尝试启动一个未初始化的定时器: '%s'", timer_configs[timer_id].name);
        return ESP_FAIL;
    }

    // 使用 xTimerReset 来启动和重启定时器，这样更安全。
    if (xTimerReset(timer_configs[timer_id].handle, 0) != pdPASS) {
        ESP_LOGE(TAG, "启动/重置定时器 '%s' 失败", timer_configs[timer_id].name);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "定时器 '%s' 已启动。", timer_configs[timer_id].name);
    return ESP_OK;
}

/**
 * @brief 根据ID停止一个指定的定时器。
 * @see app_timer_service.h
 */
esp_err_t app_timer_service_stop(app_timer_id_t timer_id) {
    // 参数验证
    if (timer_id >= TIMER_ID_MAX) {
        ESP_LOGE(TAG, "尝试停止无效的定时器ID: %d", timer_id);
        return ESP_ERR_INVALID_ARG;
    }

    // 确保定时器句柄有效。
    if (timer_configs[timer_id].handle == NULL) {
        ESP_LOGE(TAG, "尝试停止一个未初始化的定时器: '%s'", timer_configs[timer_id].name);
        return ESP_FAIL;
    }

    // 在尝试停止之前，检查定时器是否处于活动状态。
    if (xTimerIsTimerActive(timer_configs[timer_id].handle) == pdFALSE) {
        ESP_LOGW(TAG, "定时器 '%s' 未激活，无需停止。", timer_configs[timer_id].name);
        return ESP_OK; // 这不是一个错误，只是定时器未在运行。
    }

    if (xTimerStop(timer_configs[timer_id].handle, 0) != pdPASS) {
        ESP_LOGE(TAG, "停止定时器 '%s' 失败", timer_configs[timer_id].name);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "定时器 '%s' 已停止。", timer_configs[timer_id].name);
    return ESP_OK;
}