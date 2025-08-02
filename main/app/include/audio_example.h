/**
 * @file audio_example.h
 * @brief 示例音频数据 (PCM格式)
 * @version 0.1
 * @date 2025-08-01
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef AUDIO_EXAMPLE_H
#define AUDIO_EXAMPLE_H

#include <stdint.h>
#include <stddef.h>

// 这是一个 1kHz 正弦波的示例音频数据, 采样率 16000, 16位, 单声道
// 数据长度为 32 个采样点
const int16_t sine_wave_1khz_16bit_16000_mono[] = {
    0, 6392, 12539, 18204, 23169, 27245, 30272, 32137,
    32767, 32137, 30272, 27245, 23169, 18204, 12539, 6392,
    0, -6392, -12539, -18204, -23169, -27245, -30272, -32137,
    -32767, -32137, -30272, -27245, -23169, -18204, -12539, -6392,
};  

const size_t sine_wave_1khz_16bit_16000_mono_len = sizeof(sine_wave_1khz_16bit_16000_mono);

#endif /* AUDIO_EXAMPLE_H */