#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "freertos/event_groups.h"

#include <nvs_flash.h>
#include "esp_system.h"
#include "esp_event.h"
#include <esp_log.h>
#include <mqtt_client.h>
#include <esp_gap_ble_api.h>

#include <cstring>

void configureWifiNetworking(esp_event_handler_t appWifiEventHandler);
void startMQTTClient(mqtt_event_callback_t);
void configureBLENetworking();
void configureVeNetworking();
void configureMasterbus();
void startTaskForwardMasterBusPacketsToMQTT();
void taskForwardMasterBusPacketsToMQTT();
void configureMqttAugmentation();

void uploadAppInfoToMQTT();
void uploadCoreDumpFromFlashIntoMQTTTopic();
void subscribeToAppUpdatesOverMQTT();
void onAppUpdateNotification(esp_mqtt_event_handle_t event);

static EventGroupHandle_t monitorPowerEventGroup;
const int WIFI_CONNECTED_BIT = BIT0;
const int MQTT_CONNECTED_BIT = BIT1;

static void appWifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
      ESP_LOGI(__FUNCTION__,"got ip:" IPSTR, IP2STR(&event->ip_info.ip));
      xEventGroupSetBits(monitorPowerEventGroup, WIFI_CONNECTED_BIT);
    }
}
void waitForWifiConnection(){
  ESP_LOGI(__FUNCTION__, "Waiting for Wifi");
  xEventGroupWaitBits(monitorPowerEventGroup, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
  ESP_LOGI(__FUNCTION__, "Done Waiting for Wifi");
}


esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
  if (NULL == event) {
    return 0;
  }

  if (MQTT_EVENT_CONNECTED == event->event_id) {
    ESP_LOGI(__FILE__, "MQTT Connected, starting BLE scanning");
    xEventGroupSetBits(monitorPowerEventGroup, MQTT_CONNECTED_BIT);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_start_scanning(0xffffffff));
  }
  else if (MQTT_EVENT_DISCONNECTED == event->event_id) {
    ESP_LOGE(__FILE__, "MQTT Disconnected");
    xEventGroupClearBits(monitorPowerEventGroup, MQTT_CONNECTED_BIT);
  }
  else if(MQTT_EVENT_DATA== event->event_id) {
    ESP_LOGE(__FILE__, "MQTT data update for topic %s", event->topic);
    onAppUpdateNotification(event);
  }

  return 0;
}
void waitForMQTTConnection(){
  ESP_LOGI(__FUNCTION__, "Waiting for MQTT");
  xEventGroupWaitBits(monitorPowerEventGroup, MQTT_CONNECTED_BIT, false, true, portMAX_DELAY);
  ESP_LOGI(__FUNCTION__, "Done waiting for MQTT");
}

void configureFlash(){
  esp_err_t ret=ESP_OK;
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );
}

void printHeapMemoryStats(void* args){
//  esp_get_free_heap_size();
  heap_caps_print_heap_info(MALLOC_CAP_8BIT);
}

void startHeapMonitorTimer(){
//  ESP_ERROR_CHECK(esp_timer_init());
  esp_timer_create_args_t timerArgs={
      .callback=printHeapMemoryStats,
      .arg=NULL,
      .dispatch_method=ESP_TIMER_TASK,
      .name="head monitor timer",
      .skip_unhandled_events= true
  };
  esp_timer_handle_t timerHandle;
  ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &timerHandle));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timerHandle, 1000000));
}

extern "C" void app_main(void) {
  monitorPowerEventGroup = xEventGroupCreate();
//  startHeapMonitorTimer();
#if 0
  esp_log_level_set("*", ESP_LOG_WARNING);
#else
  esp_log_level_set("*", ESP_LOG_DEBUG);
  esp_log_level_set("MQTT_CLIENT", ESP_LOG_INFO);
  esp_log_level_set("OUTBOX", ESP_LOG_INFO);
//  esp_log_level_set("esp_netif_lwip", ESP_LOG_INFO);

//  esp_log_level_set("./main/masterbusNetwork.cpp", ESP_LOG_INFO);
//  esp_log_level_set("./main/MCP2515.cpp", ESP_LOG_INFO);
//  esp_log_level_set("CANBUS_HEXDUMP", ESP_LOG_DEBUG);

  esp_log_level_set("./main/veNetworkingBleParser.cpp", ESP_LOG_INFO);
  esp_log_level_set("./main/bleNetwork.cpp", ESP_LOG_INFO);
//  esp_log_level_set("./main/mqttNetwork.cpp", ESP_LOG_DEBUG);
#endif

  configureFlash();
  configureMasterbus();
  configureWifiNetworking(appWifiEventHandler);
  configureVeNetworking();
  configureBLENetworking();

  waitForWifiConnection();
  startMQTTClient(mqtt_event_handler);

  waitForMQTTConnection();
  uploadAppInfoToMQTT();
//  uploadCoreDumpFromFlashIntoMQTTTopic(); //Out of memory?

  subscribeToAppUpdatesOverMQTT();
#if 1
//  startTaskForwardMasterBusPacketsToMQTT();
  taskForwardMasterBusPacketsToMQTT();
#endif
}
