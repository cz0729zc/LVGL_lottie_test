#include "app_filter.h"
#include <stdlib.h> // for qsort
#include <string.h> // for memcpy

// qsort的比较函数
static int compare_uint16(const void *a, const void *b) {
    return (*(uint16_t *)a - *(uint16_t *)b);
}

void median_filter_init(median_filter_t *filter) {
    filter->index = 0;
    filter->count = 0;
}

uint16_t median_filter_update(median_filter_t *filter, uint16_t new_value) {
    // 将新值存入缓冲区
    filter->buffer[filter->index] = new_value;
    filter->index = (filter->index + 1) % MEDIAN_FILTER_SIZE;

    if (filter->count < MEDIAN_FILTER_SIZE) {
        filter->count++;
    }

    // 如果缓冲区未满，直接返回当前值，不做滤波
    if (filter->count < MEDIAN_FILTER_SIZE) {
        return new_value;
    }

    // 缓冲区已满，进行排序并返回中位值
    uint16_t temp_buffer[MEDIAN_FILTER_SIZE];
    memcpy(temp_buffer, filter->buffer, MEDIAN_FILTER_SIZE * sizeof(uint16_t));
    
    // 使用C标准库的qsort进行排序
    qsort(temp_buffer, MEDIAN_FILTER_SIZE, sizeof(uint16_t), compare_uint16);

    // 返回中位值
    return temp_buffer[MEDIAN_FILTER_SIZE / 2];
}