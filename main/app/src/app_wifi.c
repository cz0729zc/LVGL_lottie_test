#include "app_wifi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
// #include <network_provisioning/manager.h>
// #include <network_provisioning/scheme_softap.h>

static const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group = NULL;
static const char *TAG = "app_wifi";

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA start, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected. Reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    }
}

static void wifi_init_sta(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

esp_err_t app_wifi_init(void)
{
    ESP_LOGI(TAG, "Initializing NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    esp_netif_create_default_wifi_ap(); // 软AP接口
    esp_netif_create_default_wifi_sta(); // STA接口

    // network_prov_mgr_config_t prov_config = {
    //     .scheme = network_prov_scheme_softap,
    //     .scheme_event_handler = NETWORK_PROV_EVENT_HANDLER_NONE
    // };

    // ESP_ERROR_CHECK(network_prov_mgr_init(prov_config));

    // bool provisioned = false;
    // ESP_ERROR_CHECK(network_prov_mgr_is_wifi_provisioned(&provisioned));

    // if (!provisioned) {
    //     ESP_LOGI(TAG, "Starting provisioning");

    //     char service_name[12] = "wifi_test";
    //     const char *pop = "abcd1234";
    //     network_prov_security_t security = NETWORK_PROV_SECURITY_1;
    //     network_prov_security1_params_t *sec_params = pop;

    //     const char *service_key = "88888888"; // SoftAP密码

    //     ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(
    //         security,
    //         (const void *)&sec_params,
    //         service_name,
    //         service_key
    //     ));

    // } else {
    //     ESP_LOGI(TAG, "Already provisioned. Starting WiFi...");
    //     network_prov_mgr_deinit();
    //     wifi_init_sta();
    // }

    return ESP_OK;
}

void app_wifi_wait_connected(void)
{
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, portMAX_DELAY);
}
