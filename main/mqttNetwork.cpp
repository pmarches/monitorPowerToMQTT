#include <esp_log.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include "mdns.h"
#include <mqtt_client.h>
#include <cstring>
#include <tcpip_adapter_types.h>

#include <esp_ota_ops.h>
#include <esp_system.h>

#define TAG __FUNCTION__

const int MQTT_CONNECTED_BIT = BIT1;
esp_mqtt_client_handle_t mqttClient=0;

#define APP_UPDATE_IMAGE_TOPIC "monitorPowerToMQTT/appUpdate/image"
#define MONITOR_POWER_STATUS_TOPIC "monitorPowerToMQTT/status"
#define MONITOR_POWER_STATUS_MESSAGE_OFFLINE "offline"
#define MONITOR_POWER_STATUS_MESSAGE_ONLINE "online"

esp_mqtt_event_id_t ota_update_event_id=(esp_mqtt_event_id_t) 0;
esp_ota_handle_t otaHandle=0;
const esp_partition_t* writeableOTAPartition=NULL;
int nextExpectedOffset=0;

esp_err_t onOTAPacketReceived(int current_data_offset, uint8_t* data, int data_len, int total_data_len);

void sendOurStatusToMQTT(){
  tcpip_adapter_ip_info_t ipInfo;
  char onlineMessage[64];
  if(ESP_OK==tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo)){
    sprintf(onlineMessage, IPSTR, IP2STR(&ipInfo.ip));
  }
  else{
    sprintf(onlineMessage, MONITOR_POWER_STATUS_MESSAGE_ONLINE);
  }

  if(esp_mqtt_client_publish(mqttClient, MONITOR_POWER_STATUS_TOPIC, onlineMessage, 0, 1, 1)==-1){
    ESP_LOGE(TAG, "Failed to publish our status upon connection");
  }
}

void onMQTTUpdate(esp_mqtt_event_handle_t event){
  if(ota_update_event_id==event->event_id) {
    onOTAPacketReceived(event->current_data_offset, (uint8_t*) event->data, event->data_len, event->total_data_len);

    if(event->current_data_offset+event->data_len==event->total_data_len){
      ESP_LOGI(TAG, "Finished receiving the OTA update");
      ota_update_event_id=(esp_mqtt_event_id_t) 0;
    }
  }
  else if(strncmp(event->topic, APP_UPDATE_IMAGE_TOPIC, event->topic_len)==0){
    ota_update_event_id=event->event_id;
    onOTAPacketReceived(event->current_data_offset, (uint8_t*) event->data, event->data_len, event->total_data_len);
  }
}

void startMQTTClient(esp_event_handler_t mqttEventHandle) {
  esp_mqtt_client_config_t mqtt_cfg;
  memset(&mqtt_cfg, 0, sizeof(esp_mqtt_client_config_t));
  mqtt_cfg.uri = CONFIG_MPTM_MQTT_BROKER_URI;
//  mqtt_cfg.event_handle = mqttEventHandle;

  mqtt_cfg.lwt_topic=MONITOR_POWER_STATUS_TOPIC;
  mqtt_cfg.lwt_msg=MONITOR_POWER_STATUS_MESSAGE_OFFLINE;
  mqtt_cfg.lwt_msg_len=strlen(MONITOR_POWER_STATUS_MESSAGE_OFFLINE);
  mqtt_cfg.lwt_qos=1;
  mqtt_cfg.lwt_retain=1;

  mqttClient = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqttClient, (esp_mqtt_event_id_t) ESP_EVENT_ANY_ID, mqttEventHandle, NULL);
  ESP_ERROR_CHECK(esp_mqtt_client_start(mqttClient));
  ESP_LOGD(TAG, "startMQTTClient done");
}

#if 0
void resolve_mdns_host(const char *host_name) {
  printf("Query A: %s.local", host_name);

  esp_ip4_addr_t addr;
//  addr.addr = 0;
  esp_err_t err = mdns_query_a(host_name, 2000, &addr);
  if (err) {
    if (err == ESP_ERR_NOT_FOUND) {
      printf("Host was not found!");
      return;
    }
    printf("Query Failed");
    return;
  }

//  printf(IPSTR, IP2STR(&addr));
}
#endif

void publishToMQTT(const char* topic, const char* value){
  ESP_LOGD(TAG, "publish topic=%s value=%s", topic, value);
  const int qos=1;
  const int retain=0;
#if 1
  //The sdkconfig is such that MQTT packets are dropped when not connected. No need to track if we are connected or not.
  esp_mqtt_client_publish(mqttClient, topic, value, 0, qos, retain);
#else
  esp_mqtt_client_enqueue(mqttClient, topic, value, 0, qos, retain, true);
#endif
}

void subscribeToAppUpdatesOverMQTT(){
  const int QOS=0;
  int subscriptionId=esp_mqtt_client_subscribe(mqttClient, APP_UPDATE_IMAGE_TOPIC, QOS);
  if(subscriptionId==-1){
    ESP_LOGE(__FUNCTION__, "Failed to subscribe to the app update topic");
  }
}

esp_err_t onOTAPacketReceived(int current_data_offset, uint8_t* data, int data_len, int total_data_len){
  if(otaHandle==0){
    if(current_data_offset!=0){
      ESP_LOGE(TAG, "Wrong first packet");
      return ESP_FAIL;
    }
  }
  else {
    if(nextExpectedOffset != current_data_offset){
      ESP_LOGE(TAG, "I think we lost some OTA bytes in the air. Cancel! nextExpectedOffset=%d current_data_offset=%d", nextExpectedOffset, current_data_offset);
      esp_ota_end(otaHandle); //Ignore return code
      otaHandle=0;
      nextExpectedOffset=0;
    }
  }

  if(0==current_data_offset){
    esp_partition_t* currentPartition=NULL;
    writeableOTAPartition=esp_ota_get_next_update_partition(currentPartition);
    if(NULL==writeableOTAPartition){
      ESP_LOGE(TAG, "No OTA partition found");
      return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_ota_begin(writeableOTAPartition, total_data_len, &otaHandle));
  }
  if(0==otaHandle){
    ESP_LOGE(TAG, "No OTA handle ready for this packet");
    return ESP_FAIL;
  }

  if(esp_ota_write(otaHandle, data, data_len)!=ESP_OK){
    ESP_LOGE(TAG, "Failed to write to OTA partition. Aborting everything");
    esp_ota_end(otaHandle); //Ignore return code
    otaHandle=0;
    return ESP_FAIL;
  }
  nextExpectedOffset=current_data_offset+data_len;

  if(nextExpectedOffset==total_data_len){
    ESP_LOGW(TAG, "Have all the bytes (%d) for the OTA update. Ending the update", total_data_len);
    ESP_ERROR_CHECK(esp_ota_end(otaHandle));

    ESP_LOGD(TAG, "Changing boot partition writeableOTAPartition=%p", writeableOTAPartition);
    ESP_ERROR_CHECK(esp_ota_set_boot_partition(writeableOTAPartition));

    ESP_LOGW(TAG, "Rebooting");
    esp_restart();
  }
  return ESP_OK;
}
