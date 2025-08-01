/**
 * @file audio_player.c
 * @brief 音频播放服务层实现
 * @version 0.1
 * @date 2025-08-01
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "audio_player.h"
#include "bsp_iis_MAX98357A.h"
#include "esp_log.h"

static const char *TAG = "AUDIO_PLAYER";

/**
 * @brief 初始化音频播放服务
 */
esp_err_t audio_player_init(uint32_t sample_rate)
{
    ESP_LOGI(TAG, "Initializing audio player with sample rate: %lu", sample_rate);
    return bsp_iis_max98357a_init(sample_rate);
}

/**
 * @brief 播放原始PCM音频数据
 */
esp_err_t audio_player_play(const int16_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t bytes_written = 0;
    esp_err_t ret = bsp_iis_max98357a_write(data, len, &bytes_written, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write audio data, error: %d", ret);
    }
    if (bytes_written < len) {
        ESP_LOGW(TAG, "Not all bytes were written. Total: %d, Written: %d", len, bytes_written);
    }
    return ret;
}

/**
 * @brief 反初始化音频播放服务
 */
esp_err_t audio_player_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing audio player");
    return bsp_iis_max98357a_deinit();
}