/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_lvgl_port.h"
#include "esp_lvgl_port_priv.h"
#include "lvgl.h"

static const char *TAG = "LVGL";

#define ESP_LVGL_PORT_TASK_MUX_DELAY_MS    10000

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct lvgl_port_ctx_s {
    TaskHandle_t        lvgl_task;
    SemaphoreHandle_t   lvgl_mux;
    SemaphoreHandle_t   timer_mux;
    EventGroupHandle_t  lvgl_events;
    SemaphoreHandle_t   task_init_mux;
    esp_timer_handle_t  tick_timer;
    bool                running;
    int                 task_max_sleep_ms;
    int                 timer_period_ms;
} lvgl_port_ctx_t;

/*******************************************************************************
* Local variables
*******************************************************************************/
static lvgl_port_ctx_t lvgl_port_ctx;

/*******************************************************************************
* Function definitions
*******************************************************************************/
static void lvgl_port_task(void *arg);
static esp_err_t lvgl_port_tick_init(void);
static void lvgl_port_task_deinit(void);

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t lvgl_port_init(const lvgl_port_cfg_t *cfg)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(cfg->task_affinity < (configNUM_CORES), ESP_ERR_INVALID_ARG, err, TAG, "Bad core number for task! Maximum core number is %d", (configNUM_CORES - 1));

    memset(&lvgl_port_ctx, 0, sizeof(lvgl_port_ctx));

    /* Tick init */
    lvgl_port_ctx.timer_period_ms = cfg->timer_period_ms;
    /* Create task */
    lvgl_port_ctx.task_max_sleep_ms = cfg->task_max_sleep_ms;
    if (lvgl_port_ctx.task_max_sleep_ms == 0) {
        lvgl_port_ctx.task_max_sleep_ms = 500;
    }
    /* Timer semaphore */
    lvgl_port_ctx.timer_mux = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.timer_mux, ESP_ERR_NO_MEM, err, TAG, "Create timer mutex fail!");
    /* LVGL semaphore */
    lvgl_port_ctx.lvgl_mux = xSemaphoreCreateRecursiveMutex();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.lvgl_mux, ESP_ERR_NO_MEM, err, TAG, "Create LVGL mutex fail!");
    /* Task init semaphore */
    lvgl_port_ctx.task_init_mux = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.task_init_mux, ESP_ERR_NO_MEM, err, TAG, "Create LVGL task sem fail!");
    /* Task queue */
    lvgl_port_ctx.lvgl_events = xEventGroupCreate();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.lvgl_events, ESP_ERR_NO_MEM, err, TAG, "Create LVGL Event Group fail!");

    BaseType_t res;
    if (cfg->task_affinity < 0) {
        res = xTaskCreate(lvgl_port_task, "taskLVGL", cfg->task_stack, xTaskGetCurrentTaskHandle(), cfg->task_priority, &lvgl_port_ctx.lvgl_task);
    } else {
        res = xTaskCreatePinnedToCore(lvgl_port_task, "taskLVGL", cfg->task_stack, xTaskGetCurrentTaskHandle(), cfg->task_priority, &lvgl_port_ctx.lvgl_task, cfg->task_affinity);
    }
    ESP_GOTO_ON_FALSE(res == pdPASS, ESP_FAIL, err, TAG, "Create LVGL task fail!");

    // Wait until taskLVGL starts
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) == 0) {
        ret = ESP_ERR_TIMEOUT;
    }

err:
    if (ret != ESP_OK) {
        lvgl_port_deinit();
    }

    return ret;
}

esp_err_t lvgl_port_resume(void)
{
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    if (lvgl_port_ctx.tick_timer != NULL) {
        lv_timer_enable(true);
        ret = esp_timer_start_periodic(lvgl_port_ctx.tick_timer, lvgl_port_ctx.timer_period_ms * 1000);
    }

    return ret;
}

esp_err_t lvgl_port_stop(void)
{
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    if (lvgl_port_ctx.tick_timer != NULL) {
        lv_timer_enable(false);
        ret = esp_timer_stop(lvgl_port_ctx.tick_timer);
    }

    return ret;
}

esp_err_t lvgl_port_deinit(void)
{
    /* Stop and delete timer */
    if (lvgl_port_ctx.tick_timer != NULL) {
        esp_timer_stop(lvgl_port_ctx.tick_timer);
        esp_timer_delete(lvgl_port_ctx.tick_timer);
        lvgl_port_ctx.tick_timer = NULL;
    }

    /* Stop running task */
    if (lvgl_port_ctx.running) {
        lvgl_port_ctx.running = false;
    }

    /* Wait for stop task */
    if (xSemaphoreTake(lvgl_port_ctx.task_init_mux, pdMS_TO_TICKS(ESP_LVGL_PORT_TASK_MUX_DELAY_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to stop LVGL task");
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "Stopped LVGL task");

    lvgl_port_task_deinit();

    return ESP_OK;
}

bool lvgl_port_lock(uint32_t timeout_ms)
{
    assert(lvgl_port_ctx.lvgl_mux && "lvgl_port_init must be called first");

    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_port_ctx.lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_port_unlock(void)
{
    assert(lvgl_port_ctx.lvgl_mux && "lvgl_port_init must be called first");
    xSemaphoreGiveRecursive(lvgl_port_ctx.lvgl_mux);
}

esp_err_t lvgl_port_task_wake(lvgl_port_event_type_t event, void *param)
{
    EventBits_t bits = 0;
    if (!lvgl_port_ctx.lvgl_events) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Get unprocessed bits */
    if (xPortInIsrContext() == pdTRUE) {
        bits = xEventGroupGetBitsFromISR(lvgl_port_ctx.lvgl_events);
    } else {
        bits = xEventGroupGetBits(lvgl_port_ctx.lvgl_events);
    }

    /* Set event */
    bits |= event;

    /* Save */
    if (xPortInIsrContext() == pdTRUE) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(lvgl_port_ctx.lvgl_events, bits, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR( );
        }
    } else {
        xEventGroupSetBits(lvgl_port_ctx.lvgl_events, bits);
    }

    return ESP_OK;
}

IRAM_ATTR bool lvgl_port_task_notify(uint32_t value)
{
    BaseType_t need_yield = pdFALSE;

    // Notify LVGL task
    if (xPortInIsrContext() == pdTRUE) {
        xTaskNotifyFromISR(lvgl_port_ctx.lvgl_task, value, eNoAction, &need_yield);
    } else {
        xTaskNotify(lvgl_port_ctx.lvgl_task, value, eNoAction);
    }

    return (need_yield == pdTRUE);
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static void lvgl_port_task(void *arg)
{
    TaskHandle_t task_to_notify = (TaskHandle_t)arg;
    EventBits_t events = 0;
    uint32_t task_delay_ms = 0;
    lv_indev_t *indev = NULL;

    /* Take the task semaphore */
    if (xSemaphoreTake(lvgl_port_ctx.task_init_mux, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take LVGL task sem");
        lvgl_port_task_deinit();
        vTaskDelete( NULL );
    }

    /* LVGL init */
    lv_init();
    /* LVGL is initialized, notify lvgl_port_init() function about it */
    xTaskNotifyGive(task_to_notify);
    /* Tick init */
    lvgl_port_tick_init();

    ESP_LOGI(TAG, "Starting LVGL task");
    lvgl_port_ctx.running = true;
    while (lvgl_port_ctx.running) {
        
        /* Wait for queue or timeout (sleep task) */
        TickType_t wait = (pdMS_TO_TICKS(task_delay_ms) >= 1 ? pdMS_TO_TICKS(task_delay_ms) : 1);
        events = xEventGroupWaitBits(lvgl_port_ctx.lvgl_events, 0xFF, pdTRUE, pdFALSE, wait);

        if (lv_display_get_default() && lvgl_port_lock(0)) {

            /* Call read input devices */
            if (events & LVGL_PORT_EVENT_TOUCH) {
                xSemaphoreTake(lvgl_port_ctx.timer_mux, portMAX_DELAY);
                indev = lv_indev_get_next(NULL);
                while (indev != NULL) {
                    lv_indev_read(indev);
                    indev = lv_indev_get_next(indev);
                }
                xSemaphoreGive(lvgl_port_ctx.timer_mux);
            }

            /* Handle LVGL */
            task_delay_ms = lv_timer_handler();
            lvgl_port_unlock();
        } else {
            task_delay_ms = 1; /*Keep trying*/
        }

        if (task_delay_ms == LV_NO_TIMER_READY) {
            task_delay_ms = lvgl_port_ctx.task_max_sleep_ms;
        }

        /* Minimal dealy for the task. When there is too much events, it takes time for other tasks and interrupts. */
        vTaskDelay(1);
    }

    /* Give semaphore back */
    xSemaphoreGive(lvgl_port_ctx.task_init_mux);

    /* Close task */
    vTaskDelete( NULL );
}

static void lvgl_port_task_deinit(void)
{
    if (lvgl_port_ctx.timer_mux) {
        vSemaphoreDelete(lvgl_port_ctx.timer_mux);
    }
    if (lvgl_port_ctx.lvgl_mux) {
        vSemaphoreDelete(lvgl_port_ctx.lvgl_mux);
    }
    if (lvgl_port_ctx.task_init_mux) {
        vSemaphoreDelete(lvgl_port_ctx.task_init_mux);
    }
    if (lvgl_port_ctx.lvgl_events) {
        vEventGroupDelete(lvgl_port_ctx.lvgl_events);
    }
    memset(&lvgl_port_ctx, 0, sizeof(lvgl_port_ctx));
#if LV_ENABLE_GC || !LV_MEM_CUSTOM
    /* Deinitialize LVGL */
    lv_deinit();
#endif
}

static void lvgl_port_tick_increment(void *arg)
{
    xSemaphoreTake(lvgl_port_ctx.timer_mux, portMAX_DELAY);
    /* Tell LVGL how many milliseconds have elapsed */
    lv_tick_inc(lvgl_port_ctx.timer_period_ms);
    xSemaphoreGive(lvgl_port_ctx.timer_mux);
}

static esp_err_t lvgl_port_tick_init(void)
{
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_port_tick_increment,
        .name = "LVGL tick",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&lvgl_tick_timer_args, &lvgl_port_ctx.tick_timer), TAG, "Creating LVGL timer filed!");
    return esp_timer_start_periodic(lvgl_port_ctx.tick_timer, lvgl_port_ctx.timer_period_ms * 1000);
}
