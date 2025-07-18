#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ds18b20.h"
#include "onewire_bus.h"
#include "bsp_ds18b20.h"

#define BSP_DS18B20_ONEWIRE_GPIO    18
#define BSP_DS18B20_MAX_DEVICES     2

static int s_ds18b20_device_num = 0;
static float s_temperature = 0.0;
static ds18b20_device_handle_t s_ds18b20s[BSP_DS18B20_MAX_DEVICES];
static const char *TAG = "BSP_DS18B20";

static void sensor_detect(void)
{
     // install 1-wire bus
     onewire_bus_handle_t bus = NULL;
     onewire_bus_config_t bus_config = {
         .bus_gpio_num = BSP_DS18B20_ONEWIRE_GPIO,
     };
     onewire_bus_rmt_config_t rmt_config = {
         .max_rx_bytes = 20, // 增大缓冲区，避免溢出
     };
     esp_err_t ret = onewire_new_bus_rmt(&bus_config, &rmt_config, &bus);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to create onewire bus: %s", esp_err_to_name(ret));
         return;
     }
 
     onewire_device_iter_handle_t iter = NULL;
     onewire_device_t next_onewire_device;
     esp_err_t search_result = ESP_OK;
 
     // create 1-wire device iterator, which is used for device search
     ret = onewire_new_device_iter(bus, &iter);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to create device iterator: %s", esp_err_to_name(ret));
         return;
     }
     
     ESP_LOGI(TAG, "Device iterator created, start searching...");
     do {
         search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
         if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
             ds18b20_config_t ds_cfg = {};
             // check if the device is a DS18B20, if so, return the ds18b20 handle
             if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &s_ds18b20s[s_ds18b20_device_num]) == ESP_OK) {
                 ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", s_ds18b20_device_num, next_onewire_device.address);
                 s_ds18b20_device_num++;
             } else {
                 ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
             }
         }
     } while (search_result != ESP_ERR_NOT_FOUND);
     
     onewire_del_device_iter(iter);
     ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", s_ds18b20_device_num);
}

static void sensor_read(void)
{
    for (int i = 0; i < s_ds18b20_device_num; i ++) {
        esp_err_t ret = ds18b20_trigger_temperature_conversion(s_ds18b20s[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to trigger conversion for device %d: %s", i, esp_err_to_name(ret));
            continue;
        }
        
        ret = ds18b20_get_temperature(s_ds18b20s[i], &s_temperature);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get temperature from device %d: %s", i, esp_err_to_name(ret));
            continue;
        }
        
        ESP_LOGI(TAG, "temperature read from DS18B20[%d]: %.2fC", i, s_temperature);
    }
}

static void sensor_readTask(void *pvParameters)
{
    // 等待一段时间让系统稳定
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1) {
        sensor_read();
        vTaskDelay(pdMS_TO_TICKS(2000)); // 增加延时，减少干扰
    }
}

esp_err_t bsp_ds18b20_init(void)
{
    sensor_detect();
    return s_ds18b20_device_num > 0 ? ESP_OK : ESP_FAIL;
}

void bsp_ds18b20_start_read_task(unsigned int priority, uint32_t stack_size)
{
    xTaskCreate(&sensor_readTask, "sensor_readTask", stack_size, NULL, priority, NULL);
}

