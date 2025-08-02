#include "bsp_iis_MAX98357A.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "BSP_MAX98357A";
static i2s_chan_handle_t tx_chan; // I2S tx channel handler

esp_err_t bsp_iis_max98357a_init(uint32_t sample_rate)
{
    /* Setp 1: Determine the I2S channel configuration and allocate a TX channel */
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL), TAG, "Failed to create new I2S channel");

    /* Step 2: Setting the configurations of standard mode */
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = BSP_I2S_BCLK_PIN,
            .ws = BSP_I2S_WS_PIN,
            .dout = BSP_I2S_DOUT_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg), TAG, "Failed to init I2S channel");

    /* Step 3: Enable the TX channel before writing data */
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx_chan), TAG, "Failed to enable I2S channel");

    return ESP_OK;
}

esp_err_t bsp_iis_max98357a_write(const void *src, size_t data_size, size_t *bytes_written, uint32_t timeout_ms)
{
    if (tx_chan == NULL || src == NULL || bytes_written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2s_channel_write(tx_chan, src, data_size, bytes_written, timeout_ms);
}

esp_err_t bsp_iis_max98357a_deinit(void)
{
    if (tx_chan) {
        ESP_RETURN_ON_ERROR(i2s_channel_disable(tx_chan), TAG, "Failed to disable channel");
        ESP_RETURN_ON_ERROR(i2s_del_channel(tx_chan), TAG, "Failed to delete channel");
        tx_chan = NULL;
    }
    return ESP_OK;
}
esp_err_t bsp_iis_max98357a_stop(void)
{
    if (tx_chan) {
        return i2s_channel_disable(tx_chan);
    }
    return ESP_OK;
}