#define APP_UPDATE_IMAGE_TOPIC "monitorPowerToMQTT/appUpdate/image"

#include <esp_log.h>
#include <mqtt_client.h>

extern esp_mqtt_client_handle_t mqttClient;

void subscribeToAppUpdatesOverMQTT(){
  const int QOS=0;
  int subscriptionId=esp_mqtt_client_subscribe(mqttClient, APP_UPDATE_IMAGE_TOPIC, QOS);
  if(subscriptionId==-1){
    ESP_LOGE(__FUNCTION__, "Failed to subscribe to the app update topic");
  }
}

void onAppUpdateNotification(esp_mqtt_event_handle_t event){
  ESP_LOGD(__FUNCTION__, "Event id=%d topic=%s", event->event_id, event->topic);
  if(NULL == event->topic) {
    //This is another segment of the data
  }
  else {
    //Begin writing to the OTA partition
  }

}
