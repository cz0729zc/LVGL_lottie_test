#ifndef APP_WIFI_H
#define APP_WIFI_H

#include "esp_err.h"

// 添加SoftAP相关配置定义
#define EXAMPLE_ESP_WIFI_SSID      "ESP32_SoftAP"
#define EXAMPLE_ESP_WIFI_PASS      "123456789"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       4

void app_wifi_init_sta(void);
void app_wifi_init_ap(void);
void app_wifi_wait_connected(void);
#endif