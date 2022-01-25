#include <esp_log.h>
#include <esp_event.h>
#include <esp_wifi.h>

#include "freertos/FreeRTOS.h"
#include <freertos/event_groups.h>

#include <cstring>

#if 0
esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
  ESP_LOGD(__FUNCTION__, "Begin event->event_id=%d", event->event_id);
  esp_err_t rc;
  switch (event->event_id) {
  case SYSTEM_EVENT_STA_START:
    wifi_config_t conf;
    ESP_ERROR_CHECK(esp_wifi_get_config(ESP_IF_WIFI_STA, &conf));
    ESP_LOGD(__FUNCTION__, "Wifi connecting to '%s'", conf.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &conf));
    rc=esp_wifi_connect();
    if(rc!=ESP_OK){
      ESP_LOGE(__FUNCTION__, "esp_wifi_connect returned %x", rc);
    }
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    ESP_LOGI(__FILE__, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
    xEventGroupSetBits(inkplateStateEG, WIFI_CONNECTED_BIT);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED: {
    ESP_ERROR_CHECK(esp_wifi_connect());
    xEventGroupClearBits(inkplateStateEG, WIFI_CONNECTED_BIT);
    ESP_LOGI(__FILE__, "retry to connect to the AP");
    break;
  }
  default:
    ESP_LOGD(__FILE__, "Unhandeled wifi event %d", event->event_id);
    break;
  }
  return ESP_OK;
}
#else
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
      ESP_LOGW(__FUNCTION__, "Wifi started");
      esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
      ESP_LOGW(__FUNCTION__, "We got disconnected from the WiFi. Trying to re-connect...");
      esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
      ESP_LOGI(__FUNCTION__,"got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}
#endif

int8_t getAccessPointRSSI(){
  wifi_ap_record_t apInfo;
  esp_err_t rc=esp_wifi_sta_get_ap_info(&apInfo);
  if(rc==ESP_OK){
    return apInfo.rssi;
  }
  return 0;
}

void configureWifiNetworking() {
  ESP_LOGD(__FUNCTION__, "Begin");
#if 0
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL) );

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  strcpy((char*)wifi_config.sta.ssid, applicationConfiguration.ssid);
  strcpy((char*)wifi_config.sta.password, applicationConfiguration.password);
  wifi_config.sta.threshold.authmode=WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.listen_interval = 3;

//  wifi_config.sta.pmf_cfg.capable = true;
//  wifi_config.sta.pmf_cfg.required = false;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
//  ESP_LOGD(__FUNCTION__, "Wifi connecting to '%s' pass:'%s'", wifi_config.sta.ssid, wifi_config.sta.password);
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGD(__FUNCTION__, "Waiting for Wifi");
  xEventGroupWaitBits(inkplateStateEG, WIFI_CONNECTED_BIT,
    1, // Clear on exit
    0, // Wait for all bits
    portMAX_DELAY);
  ESP_LOGD(__FUNCTION__, "end");
#else
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

  // Initialize default station as network interface instance (esp-netif)
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  // Initialize and start WiFi
  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));

  strcpy((char*)wifi_config.sta.ssid, CONFIG_MPTM_WIFI_SSID);
  strcpy((char*)wifi_config.sta.password, CONFIG_MPTM_WIFI_PASSWORD);
  wifi_config.sta.threshold.authmode=WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.listen_interval = 3;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  //The sdkconfig settings seem to working better than this API
  //We need to reduce the power consumption at boot time otherwise, the masterview starves and reboots. Maybe we need to add some capacitors to reduce the power spikes?
  //Remove the power & charging LED?
//  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(8));
#endif
}

