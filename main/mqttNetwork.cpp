#include <esp_log.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include "mdns.h"
#include <mqtt_client.h>
#include <cstring>

const int MQTT_CONNECTED_BIT = BIT1;
esp_mqtt_client_handle_t mqttClient=0;

void configureMQTT(mqtt_event_callback_t mqttEventHandle) {
  esp_mqtt_client_config_t mqtt_cfg;
  memset(&mqtt_cfg, 0, sizeof(esp_mqtt_client_config_t));
  mqtt_cfg.uri = CONFIG_MPTM_MQTT_BROKER_URI;
  mqtt_cfg.event_handle = mqttEventHandle;
  mqttClient = esp_mqtt_client_init(&mqtt_cfg);
  ESP_ERROR_CHECK(esp_mqtt_client_start(mqttClient));
  ESP_LOGD(__FUNCTION__, "return");
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
  ESP_LOGD(__FUNCTION__, "publish topic=%s value=%s", topic, value);
  int qos=1;
  int retain=0;
#if 1
  //The sdkconfig is such that MQTT packets are dropped when not connected. No need to track if we are connected or not.
  esp_mqtt_client_publish(mqttClient, topic, value, 0, qos, retain);
#else
  esp_mqtt_client_enqueue(mqttClient, topic, value, 0, qos, retain, true);
#endif
}
