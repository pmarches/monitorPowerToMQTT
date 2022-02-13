#include <esp_log.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_sntp.h>

#include <cstring>

//#define TAG __FUNCTION__
#define TAG __FILE__

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
      ESP_LOGW(TAG, "Wifi started, trying to connect to AP");
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
      ESP_LOGW(TAG, "We got disconnected from the WiFi. Trying to re-connect...");
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
      ESP_LOGI(TAG,"got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

int8_t getAccessPointRSSI(){
  wifi_ap_record_t apInfo;
  esp_err_t rc=esp_wifi_sta_get_ap_info(&apInfo);
  if(rc==ESP_OK){
    return apInfo.rssi;
  }
  return 0;
}

void configureWifiNetworking(esp_event_handler_t appWifiEventHandler) {
  ESP_LOGD(TAG, "Begin");
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  sntp_servermode_dhcp(1);      // accept NTP offers from DHCP server, if any

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, appWifiEventHandler, NULL, NULL));

  // Initialize default station as network interface instance (esp-netif)
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  // Initialize and start WiFi
  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  strcpy((char*)wifi_config.sta.ssid, CONFIG_MPTM_WIFI_SSID);
  strcpy((char*)wifi_config.sta.password, CONFIG_MPTM_WIFI_PASSWORD);
  wifi_config.sta.scan_method=WIFI_FAST_SCAN;
  wifi_config.sta.threshold.authmode=WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.listen_interval = 3; //Only applies when WIFI_PS_MAX_MODEM is enabled

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  //The sdkconfig settings seem to working better than this API
  //We need to reduce the power consumption at boot time otherwise, the masterview starves and reboots. Maybe we need to add some capacitors to reduce the power spikes?
  //Remove the power & charging LED?
//  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(8));
}

