#include "bsp_ltr390uv.h"
#include "ltr390uv.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BSP_LTR390UV_I2C_PORT I2C_NUM_0
#define BSP_LTR390UV_SDA_GPIO GPIO_NUM_20
#define BSP_LTR390UV_SCL_GPIO GPIO_NUM_21

static ltr390uv_handle_t s_ltr390uv_hdl = NULL;
static i2c_master_bus_handle_t s_i2c_bus_hdl = NULL;
static const char *TAG = "BSP_LTR390UV";

static void ltr390uv_read(void)
{
    float ambient_light = 0, uvi = 0;
    uint32_t als_counts = 0, uvs_counts = 0;
    if (!s_ltr390uv_hdl) return;
    if (ltr390uv_get_ambient_light(s_ltr390uv_hdl, &ambient_light) == ESP_OK)
        ESP_LOGI(TAG, "Ambient: %.2f Lux", ambient_light);
    if (ltr390uv_get_ultraviolet_index(s_ltr390uv_hdl, &uvi) == ESP_OK)
        ESP_LOGI(TAG, "UVI: %.2f", uvi);
    if (ltr390uv_get_als(s_ltr390uv_hdl, &als_counts) == ESP_OK)
        ESP_LOGI(TAG, "ALS: %lu", als_counts);
    if (ltr390uv_get_uvs(s_ltr390uv_hdl, &uvs_counts) == ESP_OK)
        ESP_LOGI(TAG, "UVS: %lu", uvs_counts);
}

static void ltr390uv_read_task(void *pvParameters)
{
    while (1) {
        ltr390uv_read();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t bsp_ltr390uv_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BSP_LTR390UV_I2C_PORT,
        .sda_io_num = BSP_LTR390UV_SDA_GPIO,
        .scl_io_num = BSP_LTR390UV_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ltr390uv_config_t dev_cfg = I2C_LTR390UV_CONFIG_DEFAULT;
    ret = ltr390uv_init(s_i2c_bus_hdl, &dev_cfg, &s_ltr390uv_hdl);
    if (ret != ESP_OK || s_ltr390uv_hdl == NULL) {
        ESP_LOGE(TAG, "ltr390uv handle init failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "LTR390UV init success");
    return ESP_OK;
}

void bsp_ltr390uv_start_read_task(uint32_t priority, uint32_t stack_size)
{
    xTaskCreate(&ltr390uv_read_task, "ltr390uv_readTask", stack_size, NULL, priority, NULL);
} 