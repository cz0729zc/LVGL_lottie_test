# 睡眠表情动画修复方案

## 问题描述

当设备处于睡眠状态 (`SPRITE_STATE_AT_HOME_SLEEPING`) 时，`expression_timer` 定时器每10秒触发一次，导致睡眠表情 (`screen_animimg_exp_sleep`) 被 `happy` 或 `sad` 表情覆盖。用户希望在睡眠状态下一直显示睡眠表情，不被其他表情覆盖。

## 原因分析

在 `ui_controller.c` 文件中：

1. 当状态为 `SPRITE_STATE_AT_HOME_SLEEPING` 时（第213-222行），会显示睡眠表情动画并启动 `expression_timer`：
   ```c
   lv_obj_clear_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
   lv_animimg_start(p_ui->screen_animimg_exp_sleep);
   xTimerStart(expression_timer, 0);
   ```

2. `expression_timer` 每10秒触发一次（第63行）：
   ```c
   expression_timer = xTimerCreate("expression_timer", pdMS_TO_TICKS(10000), pdTRUE, (void *)1, expression_timer_cb);
   ```

3. 在 `expression_timer_cb` 函数中（第398-417行），会隐藏所有表情（包括睡眠表情），然后根据湿度值显示 `sad` 或 `happy` 表情：
   ```c
   lv_obj_add_flag(p_ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
   lv_obj_add_flag(p_ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
   lv_obj_add_flag(p_ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
   lv_obj_add_flag(p_ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
   ```

## 解决方案

修改 `expression_timer_cb` 函数，让它在睡眠状态下不改变表情动画。具体步骤：

1. 添加一个全局变量来跟踪当前状态
2. 在 `ui_controller_update` 函数中更新这个变量
3. 修改 `expression_timer_cb` 函数，在睡眠状态下不执行表情切换逻辑

## 具体代码修改

### 1. 添加全局变量

在文件顶部的全局变量区域添加：

```c
// --- 状态缓存 ---
static float current_humidity = 0.0f;       // 当前湿度缓存
static uint8_t current_away_background = 0; // 当前外出背景选择
static sprite_state_t current_state = SPRITE_STATE_NULL; // 当前状态缓存
```

### 2. 更新 `ui_controller_update` 函数

在 `ui_controller_update` 函数中，添加对全局状态变量的更新：

```c
void ui_controller_update(sprite_state_t state, float humidity) {
    if (p_ui == NULL) return;

    current_humidity = humidity;
    current_state = state;  // 更新当前状态缓存
    
    // 其余代码保持不变
    ...
}
```

### 3. 修改 `expression_timer_cb` 函数

修改 `expression_timer_cb` 函数，在睡眠状态下不执行表情切换逻辑：

```c
static void expression_timer_cb(TimerHandle_t xTimer) {
    if (p_ui == NULL) return;
    ESP_LOGI(TAG, "Periodic expression timer expired.");
    
    // 如果当前是睡眠状态，不改变表情
    if (current_state == SPRITE_STATE_AT_HOME_SLEEPING) {
        ESP_LOGI(TAG, "In sleeping state, keeping sleep expression.");
        return;
    }
    
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
```

## 实施建议

请切换到 Code 模式来实现这些更改。修改完成后，测试睡眠状态下的表情动画是否能够持续显示，不被其他表情覆盖。