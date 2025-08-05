/**
 * @file app_controller.h
 * @brief 应用核心控制器头文件
 *
 * 该模块是整个应用的核心，负责：
 * 1. 接收所有传感器和输入模块的数据/事件。
 * 2. 管理和运行核心的“小精灵”状态机。
 * 3. 根据状态机的决策，调度UI动画、LED灯光等输出。
 *
 * 通过事件组（Event Group）实现任务间的异步通信。
 */
#ifndef _APP_CONTROLLER_H_
#define _APP_CONTROLLER_H_


/**
 * @brief 定义小精灵的所有可能状态
 */
typedef enum {
    SPRITE_STATE_NULL,                  /**< 空状态或未定义状态 */
    SPRITE_STATE_AT_HOME_AWAKE,         /**< 在家且醒着 */
    SPRITE_STATE_AT_HOME_SLEEPING,      /**< 在家且睡觉 */
    SPRITE_STATE_AWAY_NORMAL,           /**< 正常外出 */
    SPRITE_STATE_AWAY_LOST,             /**< 因环境恶劣（如干旱）而迷路 */
    SPRITE_STATE_PREPARING_TO_GO_HOME,  /**< 准备回家（拍一拍后的中间状态） */
    SPRITE_STATE_EVENT_FLOODED,         /**< 因环境恶劣（如过湿）而触发淹水事件 */
} sprite_state_t;

/**
 * @brief 定义应用控制器使用的事件标志
 *
 * 用于FreeRTOS事件组，实现任务间的异步通知。
 */
#define SENSOR_ADC_DATA_READY    BIT0   /**< ADC数据准备就绪事件（土壤湿度） */
#define SENSOR_TEMP_DATA_READY   BIT1   /**< 温度数据准备就绪事件 */
#define SENSOR_LIGHT_DATA_READY  BIT2   /**< 光照数据准备就绪事件 */
#define WIFI_CONNECTED           BIT3   /**< Wi-Fi连接成功事件 */
#define USER_INTERACTION_TAP     BIT4   /**< 用户输入事件（拍一拍） */
// --- 定时器事件 (BIT5-BIT7) ---
#define EVENT_TIMER_5_MIN_EXPIRED    BIT5   /**< 5分钟回家倒计时结束事件 */
#define EVENT_TIMER_48_HOUR_EXPIRED  BIT6   /**< 48小时提醒周期到达事件 */
#define EVENT_TIMER_10_MIN_AWAY_EXPIRED BIT7  /**< 10分钟离家定时器到期事件 */
// --- 内部事件 (BIT8+) ---


/**
 * @brief 初始化应用控制器
 *
 * 创建事件组，初始化UI，并准备控制器任务所需资源。
 */
void app_controller_init(void);

/**
 * @brief 启动应用控制器主任务
 *
 * @param priority 任务优先级
 * @param stack_size 任务堆栈大小
 */
void app_controller_start_task(uint32_t priority, uint32_t stack_size);

// ------------------ 供其他模块调用的通知函数 ------------------

// /**
//  * @brief 请求UI切换到指定的屏幕
//  *
//  * @param screen_id 要加载的屏幕ID (ui_screen_id_t)
//  */
// void app_controller_set_ui_screen(ui_screen_id_t screen_id);

/**
 * @brief 通知控制器ADC数据已更新
 *
 * @param percent_ch0 通道0的百分比值（主要用于土壤湿度）
 * @param percent_ch1 通道1的百分比值
 */
void app_controller_notify_adc_data(float percent_ch0, float percent_ch1);

/**
 * @brief 通知控制器温度数据已更新
 *
 * @param temperature 当前温度值
 */
void app_controller_notify_temp_data(float temperature);

/**
 * @brief 通知控制器光照数据已更新
 *
 * @param uv_index 当前紫外线指数
 */
void app_controller_notify_light_data(float uv_index);

/**
 * @brief 通知控制器Wi-Fi已连接
 */
void app_controller_notify_wifi_connected(void);

/**
 * @brief 通知控制器检测到用户“拍一拍”交互
 */
void app_controller_notify_tap(void);

/**
 * @brief 向控制器发送一个通用事件
 *
 * 这是一个更通用的通知函数，允许任何模块将指定的事件位
 * 设置到控制器的事件组中。
 *
 * @param event_bit 要设置的事件位 (例如 EVENT_TIMER_5_MIN_EXPIRED)。
 */
void app_controller_notify_event(EventBits_t event_bit);

#endif
