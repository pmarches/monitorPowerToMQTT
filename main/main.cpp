#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_system.h>
#include <esp_event.h>
#include <esp_ota_ops.h>
#include <esp_log.h>
#include <mqtt_client.h>
#if 0
#include <esp_gap_ble_api.h>
#endif

#include <cstring>

#define TAG __FILE__

void configureWifiNetworking(esp_event_handler_t appWifiEventHandler);
void startMQTTClient(esp_event_handler_t );
void configureBLENetworking();
void configureVeNetworking();
void configureMasterbus();
void startTaskForwardMasterBusPacketsToMQTT();
void taskForwardMasterBusPacketsToMQTT();
void configureMqttAugmentation();
void sendOurStatusToMQTT();
void configureFlash();

void uploadAppInfoToMQTT();
void uploadCoreDumpFromFlashIntoMQTTTopic();
void subscribeToAppUpdatesOverMQTT();
void onMQTTUpdate(esp_mqtt_event_handle_t event);
void getLocalTimeFromNetwork();

void taskForwardVEDirectSentenceToMQTT(void* arg);

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

static void mqtt_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data){
  esp_mqtt_event_handle_t mqttEvent = (esp_mqtt_event_handle_t) event_data;
  if (MQTT_EVENT_CONNECTED == event_id) {
    ESP_LOGI(__FILE__, "MQTT Connected");
    xEventGroupSetBits(monitorPowerEventGroup, MQTT_CONNECTED_BIT);
    sendOurStatusToMQTT();

#if 0
    ESP_LOGI(__FILE__, "starting BLE scanning");
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_start_scanning(0xffffffff));
#endif
  }
  else if (MQTT_EVENT_DISCONNECTED == event_id) {
    ESP_LOGE(__FILE__, "MQTT Disconnected");
    xEventGroupClearBits(monitorPowerEventGroup, MQTT_CONNECTED_BIT);
  }
  else if(MQTT_EVENT_DATA== event_id) {
    if(mqttEvent->topic){
      ESP_LOGD(__FILE__, "MQTT data update for topic %.*s", mqttEvent->topic_len, mqttEvent->topic);
    }
    if(mqttEvent->total_data_len!=0){
      onMQTTUpdate(mqttEvent);
    }
  }
}

void waitForMQTTConnection(){
  ESP_LOGI(__FUNCTION__, "Waiting for MQTT");
  xEventGroupWaitBits(monitorPowerEventGroup, MQTT_CONNECTED_BIT, false, true, portMAX_DELAY);
  ESP_LOGI(__FUNCTION__, "Done waiting for MQTT");
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

  esp_log_level_set("./components/vedirect-hex-parser/main/vhp_parser.cpp", ESP_LOG_INFO);
  esp_log_level_set("./components/vedirect-hex-parser/main/vhp_command.cpp", ESP_LOG_INFO);
  esp_log_level_set("./components/vedirect-hex-parser/main/vhp_driver.cpp", ESP_LOG_INFO);
//  esp_log_level_set("./main/mqttNetwork.cpp", ESP_LOG_DEBUG);
#endif

  monitorPowerEventGroup = xEventGroupCreate();
//  startHeapMonitorTimer();

  configureFlash();
  configureMasterbus();
  configureWifiNetworking(appWifiEventHandler);
#if 0
  configureVeNetworking();
  configureBLENetworking();
#endif

  waitForWifiConnection();
  startMQTTClient(mqtt_event_handler);

  waitForMQTTConnection();
  uploadAppInfoToMQTT();
//  uploadCoreDumpFromFlashIntoMQTTTopic(); //Out of memory?
  subscribeToAppUpdatesOverMQTT();
  xTaskCreate(taskForwardVEDirectSentenceToMQTT, "taskForwardVEDirectSentenceToMQTT", 4096, NULL, 5, NULL);

#if 0
//  startTaskForwardMasterBusPacketsToMQTT();
  esp_ota_mark_app_valid_cancel_rollback();
  getLocalTimeFromNetwork();
  taskForwardMasterBusPacketsToMQTT();
#endif
}
