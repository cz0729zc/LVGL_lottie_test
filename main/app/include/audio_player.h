/**
 * @file audio_player.h
 * @brief 音频播放服务层接口
 * @version 0.1
 * @date 2025-08-01
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化音频播放服务
 * @details
 *      - 初始化I2S驱动
 *      - 创建音频播放任务 (可选，如果需要后台播放)
 *
 * @param sample_rate 音频采样率 (e.g., 44100, 16000)
 * @return
 *      - ESP_OK: 成功
 *      - 其他: 失败
 */
esp_err_t audio_player_init(uint32_t sample_rate);

/**
 * @brief 播放原始PCM音频数据
 * @details
 *      将音频数据块写入I2S总线进行播放。
 *      这是一个阻塞函数，直到所有数据都写入缓冲区或超时。
 *
 * @param data 指向16位PCM音频数据的指针
 * @param len 要播放的数据长度 (以字节为单位)
 * @return
 *      - ESP_OK: 成功
 *      - ESP_ERR_INVALID_ARG: 参数错误
 *      - 其他: I2S写入失败
 */
esp_err_t audio_player_play(const int16_t *data, size_t len);

/**
 * @brief 反初始化音频播放服务
 * @details
 *      - 释放I2S资源
 *      - 删除音频播放任务 (如果已创建)
 *
 * @return
 *      - ESP_OK: 成功
 *      - 其他: 失败
 */
esp_err_t audio_player_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_PLAYER_H */