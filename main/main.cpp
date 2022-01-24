#include "freertos/FreeRTOS.h"
#include <freertos/task.h>

#include <nvs_flash.h>
#include "esp_system.h"
#include "esp_event.h"
#include <esp_log.h>

#include <cstring>

void configureWifiNetworking();
void configureMQTT();
void configureBLENetworking();
void configureVeNetworking();
void configureMasterbus();
void startTaskForwardCMasterBusPacketsToMQTT();
void taskForwardCMasterBusPacketsToMQTT();
void configureMqttAugmentation();
void uploadCoreDumpFromFlashIntoMQTTTopic();

void configureFlash(){
  esp_err_t ret=ESP_OK;
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );
}


extern "C" void app_main(void) {
#if 1
  esp_log_level_set("*", ESP_LOG_WARN);
//  esp_log_level_set("../main/masterbusNetwork.cpp", ESP_LOG_DEBUG);
//  esp_log_level_set("../main/MCP2515.cpp", ESP_LOG_DEBUG);
//  esp_log_level_set("CANBUS_HEXDUMP", ESP_LOG_DEBUG);
  esp_log_level_set("publishToMQTT", ESP_LOG_DEBUG);
#endif

  configureFlash();
  configureWifiNetworking();
  configureMQTT();

  uploadCoreDumpFromFlashIntoMQTTTopic();
#if 1
  configureBLENetworking();
  configureVeNetworking();
#endif

  configureMasterbus();
//  startTaskForwardCMasterBusPacketsToMQTT();
  taskForwardCMasterBusPacketsToMQTT();
  configureMqttAugmentation();
}
