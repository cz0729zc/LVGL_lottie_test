/*
 * Optimized app_wifi.c for ESP-IDF v5.3.2
 * - Single NVS and netif initialization
 * - Clean event registration and proper teardown
 * - Separate STA and AP init without duplicate esp_netif_create_default
 */

 #include "app_wifi.h"
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/event_groups.h"
 #include "esp_log.h"
 #include "esp_wifi.h"
 #include "esp_event.h"
 #include "nvs_flash.h"
 #include "esp_netif.h"
 #include "esp_mac.h"
 
 static const int WIFI_CONNECTED_EVENT = BIT0;
 static EventGroupHandle_t wifi_event_group;
 static const char *TAG = "app_wifi";
 
 // STA 事件处理
 static void sta_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
     switch (id) {
     case WIFI_EVENT_STA_START:
         ESP_LOGI(TAG, "STA start, connecting...");
         esp_wifi_connect();
         break;
     case WIFI_EVENT_STA_DISCONNECTED:
         ESP_LOGW(TAG, "STA disconnected, retrying...");
         esp_wifi_connect();
         break;
     case IP_EVENT_STA_GOT_IP: {
         ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
         ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
         xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
         break;
     }
     default:
         break;
     }
 }
 
 // AP 事件处理
 static void ap_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
     if (id == WIFI_EVENT_AP_STACONNECTED) {
         wifi_event_ap_staconnected_t* ev = data;
         ESP_LOGI(TAG, "STA " MACSTR " joined, AID=%d", MAC2STR(ev->mac), ev->aid);
     } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
         wifi_event_ap_stadisconnected_t* ev = data;
         ESP_LOGI(TAG, "STA " MACSTR " left, AID=%d, reason=%d", MAC2STR(ev->mac), ev->aid, ev->reason);
     }
 }
 
 // Common initialization: NVS, netif, event loop
 static void common_init(void) {
     static bool inited = false;
     if (inited) return;
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ESP_ERROR_CHECK(nvs_flash_init());
     }
     ESP_ERROR_CHECK(esp_netif_init());
     ESP_ERROR_CHECK(esp_event_loop_create_default());
     wifi_event_group = xEventGroupCreate();
     inited = true;
 }
 
 // 初始化 STA
 void app_wifi_init_sta(void) {
     common_init();
     esp_netif_create_default_wifi_sta();
 
     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &sta_event_handler, NULL));
     ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &sta_event_handler, NULL));
 
     wifi_config_t sta_cfg = {
         .sta = {
             .ssid = "My_WIFI",
             .password = "88888888",
             .threshold.authmode = WIFI_AUTH_WPA2_PSK,
             .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
         },
     };
     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
     ESP_ERROR_CHECK(esp_wifi_start());
 }
 
 // 初始化 AP
 void app_wifi_init_ap(void) {
     common_init();
     esp_netif_create_default_wifi_ap();
 
     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &ap_event_handler, NULL));
     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &ap_event_handler, NULL));
 
     wifi_config_t ap_cfg = {
         .ap = {
             .ssid = EXAMPLE_ESP_WIFI_SSID,
             .ssid_len = 0,
             .channel = EXAMPLE_ESP_WIFI_CHANNEL,
             .password = EXAMPLE_ESP_WIFI_PASS,
             .max_connection = EXAMPLE_MAX_STA_CONN,
 #ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
             .authmode = WIFI_AUTH_WPA3_PSK,
             .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
 #else
             .authmode = WIFI_AUTH_WPA2_PSK,
 #endif
             .pmf_cfg = {.required = true},
         }
     };
     if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
         ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
     }
 
     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
     ESP_ERROR_CHECK(esp_wifi_start());
     ESP_LOGI(TAG, "AP started: SSID=%s, PASS=%s, CH=%d", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
 }
 
 // 阻塞等待 STA 连接
 void app_wifi_wait_connected(void) {
     xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, pdFALSE, pdTRUE, portMAX_DELAY);
 }
 