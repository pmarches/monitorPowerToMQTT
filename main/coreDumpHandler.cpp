extern "C" {
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
//#include "esp_app_format.h"

//#include "esp_flash_partitions.h"
#include <nvs_flash.h>
#include "esp_partition.h"
#include <esp_core_dump.h>
#include <esp_log.h>
//#include <esp_ota_ops.h>
#include <memory.h>

#include <mqtt_client.h>
}

//#define TAG __FUNCTION__
#define TAG __FILE__

#define SHA256_PRINTF_STR_HEX "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"
#define SHA256_PRINTF_STR(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7], addr[8], addr[9], addr[10], addr[11], addr[12], addr[13], addr[14], addr[15], addr[16], addr[17], addr[18], addr[19], addr[20], addr[21], addr[22], addr[23], addr[24], addr[25], addr[26], addr[27], addr[28], addr[29], addr[30], addr[31]

extern esp_mqtt_client_handle_t mqttClient;

const char* nvsTypeToString(nvs_type_t nvsType){
    if(nvsType==NVS_TYPE_U8) return "NVS_TYPE_U8";
    if(nvsType==NVS_TYPE_I8) return "NVS_TYPE_I8";
    if(nvsType==NVS_TYPE_U16) return "NVS_TYPE_U16";
    if(nvsType==NVS_TYPE_I16) return "NVS_TYPE_I16";
    if(nvsType==NVS_TYPE_U32) return "NVS_TYPE_U32";
    if(nvsType==NVS_TYPE_I32) return "NVS_TYPE_I32";
    if(nvsType==NVS_TYPE_U64) return "NVS_TYPE_U64";
    if(nvsType==NVS_TYPE_I64) return "NVS_TYPE_I64";
    if(nvsType==NVS_TYPE_STR) return "NVS_TYPE_STR";
    if(nvsType==NVS_TYPE_BLOB) return "NVS_TYPE_BLOB";
    if(nvsType==NVS_TYPE_ANY) return "NVS_TYPE_ANY";
    return "Unknown nvs_type_t";
}

#if 1
void printFlashPartition(){
  ESP_LOGD(TAG, "Begin");
  esp_partition_iterator_t partitionIt=esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while(partitionIt!=NULL){
    const esp_partition_t* partitionInfo=esp_partition_get(partitionIt);
    ESP_LOGW(TAG, "Partition %s", partitionInfo->label);
    esp_err_t errCode=nvs_flash_init_partition(partitionInfo->label);
    if(errCode!=ESP_OK){
      ESP_LOGW(TAG, "Partition %s could not be inited. Error 0x%x, skipping", partitionInfo->label, errCode);
      partitionIt=esp_partition_next(partitionIt);
      continue;
    }
    uint8_t partitionSha256[32];
    if(esp_partition_get_sha256(partitionInfo, partitionSha256)==ESP_OK){
      ESP_LOGD(TAG, "SHA256 "SHA256_PRINTF_STR_HEX, SHA256_PRINTF_STR(partitionSha256));
    }

    nvs_stats_t nvsStats;
    ESP_ERROR_CHECK(nvs_get_stats(partitionInfo->label, &nvsStats));
    ESP_LOGI(TAG, "%d namespace, %d/%d entries, %d free", nvsStats.namespace_count, nvsStats.used_entries, nvsStats.total_entries, nvsStats.free_entries);
    nvs_iterator_t entryIt=nvs_entry_find(partitionInfo->label, NULL, NVS_TYPE_ANY);
    while(entryIt!=NULL){
      nvs_entry_info_t entryInfo;
      nvs_entry_info(entryIt, &entryInfo);
      ESP_LOGI(TAG, "%s:%s, %s", entryInfo.namespace_name, entryInfo.key, nvsTypeToString(entryInfo.type));
      entryIt=nvs_entry_next(entryIt);
    }
    nvs_release_iterator(entryIt);
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_deinit_partition(partitionInfo->label));
    partitionIt=esp_partition_next(partitionIt);
  }
  esp_partition_iterator_release(partitionIt);
}
#endif

//Lifted from ./espcoredump/include_core_dump/esp_core_dump_priv.h
typedef struct _core_dump_header_t
{
    uint32_t data_len;  // data length
    uint32_t version;   // core dump struct version
    uint32_t tasks_num; // number of tasks
    uint32_t tcb_sz;    // size of TCB
} core_dump_header_t;

//Lifted from ./espcoredump/include_core_dump/esp_core_dump_priv.h
/** core dump task data header */
typedef struct _core_dump_task_header_t
{
    void *   tcb_addr;    // TCB address
    uint32_t stack_start; // stack start address
    uint32_t stack_end;   // stack end address
} core_dump_task_header_t;

typedef struct {
  uint32_t exit;
  uint32_t programCounter;
  uint32_t ps;
  uint32_t a0;
  uint32_t stackPointer;
} taskCoreDump_t;

const char* convertResetReasonToString(esp_reset_reason_t resetReason){
  switch(resetReason){
    case ESP_RST_UNKNOWN: return "ESP_RST_UNKNOWN";
    case ESP_RST_POWERON: return "ESP_RST_POWERON";
    case ESP_RST_EXT: return "ESP_RST_EXT";
    case ESP_RST_SW: return "ESP_RST_SW";
    case ESP_RST_PANIC: return "ESP_RST_PANIC";
    case ESP_RST_INT_WDT: return "ESP_RST_INT_WDT";
    case ESP_RST_TASK_WDT: return "ESP_RST_TASK_WDT";
    case ESP_RST_WDT: return "ESP_RST_WDT";
    case ESP_RST_DEEPSLEEP: return "ESP_RST_DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "ESP_RST_BROWNOUT";
    case ESP_RST_SDIO: return "ESP_RST_SDIO";
    default : return "Unkown unkown";
  }
}

void uploadAppInfoToMQTT(){
  const char* topicFormat="%s/appinfo/%s";
  const int retain=1;
  const int qos=1;

  ESP_LOGD(TAG, "Begin");
  const esp_app_desc_t *appDesc=esp_ota_get_app_description();
  char topic[128];
  //TODO Change this to a single JSON payload to ensure atomic integrity?
  sprintf(topic, topicFormat, appDesc->project_name, "appVersion");
  esp_mqtt_client_publish(mqttClient, topic, appDesc->version, 0, qos, retain);

  sprintf(topic, topicFormat, appDesc->project_name, "projectName");
  esp_mqtt_client_publish(mqttClient, topic, appDesc->project_name, 0, qos, retain);

  sprintf(topic, topicFormat, appDesc->project_name, "compileTime");
  esp_mqtt_client_publish(mqttClient, topic, appDesc->time, 0, qos, retain);

  sprintf(topic, topicFormat, appDesc->project_name, "compileDate");
  esp_mqtt_client_publish(mqttClient, topic, appDesc->date, 0, qos, retain);

  sprintf(topic, topicFormat, appDesc->project_name, "resetReason");
  esp_mqtt_client_publish(mqttClient, topic, convertResetReasonToString(esp_reset_reason()), 0, qos, retain);

//  coreDumpReport->has_applicationelfsha256=true;
//  coreDumpReport->applicationelfsha256.len=32;
//  coreDumpReport->applicationelfsha256.data=malloc(coreDumpReport->applicationelfsha256.len);
//  memcpy(coreDumpReport->applicationelfsha256.data, appDesc->app_elf_sha256, coreDumpReport->applicationelfsha256.len);
}

//FIXME Seems like we are running out of memory for uploading the coredump?
void uploadCoreDumpFromFlashIntoMQTTTopic(){
  char topic[128];
  const char* topicFormat="%s/cordedump/%s";
  const char* projectName="monitorPowerToMQTT";
  const int retain=1;
  const int qos=1;

  ESP_LOGD(TAG, "Begin");
  esp_core_dump_init();
  size_t coreAddr;
  size_t coreSize;
  if(esp_core_dump_image_get(&coreAddr, &coreSize)==ESP_OK){
    ESP_LOGD(TAG, "coreAddr=0x%x coreSize=%d", coreAddr, coreSize);
  }
  //See ~/projects/marine_is/esp32/esp-idf/components/espcoredump/src/core_dump_common.c
  char topicValueStr[32];
  sprintf(topicValueStr, "%d", coreSize);
  sprintf(topic, topicFormat, projectName, "coredumpsize");
  esp_mqtt_client_publish(mqttClient, topic, topicValueStr, 0, qos, retain);

  esp_partition_iterator_t partitionIt=esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, "coredump");
  if(partitionIt==NULL){
    ESP_LOGE(TAG, "Did not find the coredump partition");
    return;
  }

  const esp_partition_t* partitionInfo=esp_partition_get(partitionIt);
  ESP_LOGI(TAG, "Found a coredump partition, mapping it to memory");
  const void *startOfCoreDump=NULL;
  spi_flash_mmap_handle_t core_data_handle;
  ESP_ERROR_CHECK(esp_partition_mmap(partitionInfo, 0, coreSize, SPI_FLASH_MMAP_DATA, &startOfCoreDump, &core_data_handle));
  if(startOfCoreDump==NULL){
    ESP_LOGE(TAG, "Did not find the start of coredump");
    return;
  }

//    ESP_LOG_BUFFER_HEX_LEVEL(TAG, startOfCoreDump, 1024, ESP_LOG_INFO);
  ESP_LOGI(TAG, "Uploading coredump to MQTT server");
  sprintf(topic, topicFormat, projectName, "coredump");
  esp_mqtt_client_publish(mqttClient, topic, (const char*)startOfCoreDump, coreSize, qos, retain);

  core_dump_header_t* coredumpHeader=(core_dump_header_t*) startOfCoreDump;
  ESP_LOGD(TAG, "Number of Task=%d", coredumpHeader->tasks_num);
  ESP_LOGD(TAG, "Data len=%d", coredumpHeader->data_len);
  ESP_LOGD(TAG, "Task Control Block Size=%d", coredumpHeader->tcb_sz);

//    coreDumpReport->n_taskcoredumps=;
//    coreDumpReport->taskcoredumps=malloc(coreDumpReport->n_taskcoredumps*sizeof(FMDTaskCoreDumpReport*));

    const uint8_t* dataPtr=(const uint8_t*) startOfCoreDump+sizeof(core_dump_header_t);
    for(int i=0; i<coredumpHeader->tasks_num; i++){
      core_dump_task_header_t* taskHeader=(core_dump_task_header_t*) dataPtr;
      uint32_t stackSize=taskHeader->stack_end-taskHeader->stack_start;
      ESP_LOGD(TAG, "Task %p Stack TCBaddress=%p start-end=0x%x-0x%x stackSize=%d", taskHeader, taskHeader->tcb_addr, taskHeader->stack_start, taskHeader->stack_end, stackSize);
      dataPtr+=sizeof(core_dump_task_header_t);

      //      ESP_LOG_BUFFER_HEX_LEVEL("TCB For Task Task ", dataPtr, coredumpHeader->tcb_sz, ESP_LOG_INFO);
      dataPtr+=coredumpHeader->tcb_sz;
      taskCoreDump_t* taskCoreDump=(taskCoreDump_t*) dataPtr;
      //    ESP_LOGD(TAG, "programCounter = %p", taskCoreDump->programCounter);

      //      ESP_LOG_BUFFER_HEX_LEVEL("Stack For Task ", dataPtr, stackSize, ESP_LOG_INFO);
      dataPtr+=stackSize;

      //DISABLED because the messages are too big for LoRa. Re-enable for BLE?
      //    taskCoreDumpMsg->has_exit=true; taskCoreDumpMsg->exit=taskCoreDump->exit;
      sprintf(topic, topicFormat, projectName, "programcounter");
      sprintf(topicValueStr, "%d", taskCoreDump->programCounter);
      esp_mqtt_client_publish(mqttClient, topic, topicValueStr, 0, qos, retain);

      //    taskCoreDumpMsg->has_ps=true; taskCoreDumpMsg->ps=taskCoreDump->ps;
      //    taskCoreDumpMsg->has_a0=true; taskCoreDumpMsg->a0=taskCoreDump->a0;
      //    taskCoreDumpMsg->has_stackpointer=true; taskCoreDumpMsg->stackpointer=taskCoreDump->stackPointer;
    }

    spi_flash_munmap(core_data_handle);
    ESP_LOGD(TAG, "Done");
}
