#ifndef _APP_WIFI_H_
#define _APP_WIFI_H_

#include "esp_err.h"

// 初始化WiFi配网（自动判断是否已配网，未配网则进入配网流程）
esp_err_t app_wifi_init(void);

// 阻塞等待WiFi连接成功
void app_wifi_wait_connected(void);

#endif