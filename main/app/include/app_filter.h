#ifndef APP_FILTER_H
#define APP_FILTER_H

#include <stdint.h>

// 定义中位值滤波器的采样窗口大小，可以根据需要调整，一般取奇数
#define MEDIAN_FILTER_SIZE 5

/**
 * @brief 中位值滤波器结构体
 */
typedef struct {
    uint16_t buffer[MEDIAN_FILTER_SIZE]; // 存储样本的缓冲区
    uint8_t index;                       // 当前缓冲区的索引
    uint8_t count;                       // 当前已存储的样本数
} median_filter_t;

/**
 * @brief 初始化中位值滤波器
 *
 * @param filter 指向滤波器实例的指针
 */
void median_filter_init(median_filter_t *filter);

/**
 * @brief 更新滤波器并获取中位值
 *
 * @param filter 指向滤波器实例的指针
 * @param new_value 新的采样值
 * @return uint16_t 滤波后的中位值
 */
uint16_t median_filter_update(median_filter_t *filter, uint16_t new_value);

#endif // APP_FILTER_H