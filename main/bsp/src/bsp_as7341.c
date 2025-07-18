#include "bsp_as7341.h"
#include "as7341.h"
#include "esp_log.h"

#define AS7341_I2C_PORT I2C_NUM_0
#define AS7341_SDA_GPIO GPIO_NUM_20
#define AS7341_SCL_GPIO GPIO_NUM_21

static as7341_handle_t s_as7341_hdl = NULL;
static i2c_master_bus_handle_t s_i2c_bus_hdl = NULL;

esp_err_t bsp_as7341_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = AS7341_I2C_PORT,
        .sda_io_num = AS7341_SDA_GPIO,
        .scl_io_num = AS7341_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE("AS7341", "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    as7341_config_t dev_cfg = I2C_AS7341_CONFIG_DEFAULT;
    ret = as7341_init(s_i2c_bus_hdl, &dev_cfg, &s_as7341_hdl);
    if (ret != ESP_OK || s_as7341_hdl == NULL) {
        ESP_LOGE("AS7341", "as7341 handle init failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// // 你可以根据实际需要扩展参数
// esp_err_t bsp_as7341_read(/* 你需要的参数，比如结构体指针 */)
// {
//     if (!s_as7341_hdl) return ESP_ERR_INVALID_STATE;
//     // TODO: 调用 as7341_get_spectral_measurements/as7341_get_basic_counts 等接口
//     return ESP_OK;
// } 