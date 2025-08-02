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
 * @brief 停止音频播放
 */
esp_err_t audio_player_stop(void)
{
    ESP_LOGI(TAG, "Stopping audio player");
    return bsp_iis_max98357a_stop();
}

/**
 * @brief 反初始化音频播放服务
 */
esp_err_t audio_player_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing audio player");
    return bsp_iis_max98357a_deinit();
}
/**
 * @brief 播放指定时长的音频
 */
esp_err_t audio_player_play_for(const int16_t *audio_data, size_t audio_data_len, uint32_t sample_rate, int duration_ms)
{
    if (audio_data == NULL || audio_data_len == 0 || sample_rate == 0 || duration_ms <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 计算单段音频数据的播放时长 (毫秒)
    // 由于数据是 int16_t (每个采样点2字节), 采样点数量为 audio_data_len / 2
    float single_play_duration_ms = ((float)audio_data_len / 2.0f / (float)sample_rate) * 1000.0f;

    if (single_play_duration_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 计算需要循环播放的次数
    int loop_count = (int)((float)duration_ms / single_play_duration_ms);
    if (loop_count == 0) {
        loop_count = 1; // 至少播放一次
    }

    ESP_LOGI(TAG, "开始播放音频，持续时间: %d ms，循环次数: %d", duration_ms, loop_count);

    // 循环播放音频数据
    for (int i = 0; i < loop_count; i++) {
        esp_err_t ret = audio_player_play(audio_data, audio_data_len);
        if (ret != ESP_OK) {
            // 如果写入失败, 立即停止并返回错误
            audio_player_stop();
            return ret;
        }
    }

    // 循环结束后停止播放
    return audio_player_stop();
}