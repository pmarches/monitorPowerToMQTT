#include <esp_log.h>
#include "masterbusController.h"
#include <mvParser.hpp>

#define SPI_CLK_PIN (gpio_num_t)13
#define SPI_MASTER_OUT_SLAVE_IN_PIN (gpio_num_t) 5
#define SPI_MASTER_IN_SLAVE_OUT_PIN (gpio_num_t) 4
#define SPI_CS_PIN (gpio_num_t) 3
#define SPI_INT_PIN (gpio_num_t)14

#define MYCANCTL_BUS SPI2_HOST
//#define TAG __FUNCTION__
#define TAG __FILE__

MasterbusController* g_mbctl;

void configurePinAsOutput(gpio_num_t pin);
void publishToMQTT(const char* topic, const char* value);
char* bytesToHex(uint8_t* bytes, int bytesLen);

SPIBus spi(MYCANCTL_BUS);
void initializeMasterbus(){
  ESP_LOGD(TAG, "init: mosi=%d, miso=%d, clk=%d, cs=%d", SPI_MASTER_OUT_SLAVE_IN_PIN, SPI_MASTER_IN_SLAVE_OUT_PIN, SPI_CLK_PIN, SPI_CS_PIN);

  ESP_ERROR_CHECK(spi.init(SPI_MASTER_OUT_SLAVE_IN_PIN, SPI_MASTER_IN_SLAVE_OUT_PIN, SPI_CLK_PIN, SPI_CS_PIN));
  SPIDevice* spiDevice=new SPIDevice(&spi, SPI_CS_PIN);
  ESP_ERROR_CHECK(spiDevice->init());
  MCP2515Class* mcp2515Rx=new MCP2515Class(spiDevice, SPI_CS_PIN, SPI_INT_PIN, 8E6);
  g_mbctl=new MasterbusController(mcp2515Rx);
  ESP_ERROR_CHECK(g_mbctl->configure(MCP2515Class::NORMAL_MODE));

  ESP_LOGD(TAG, "End");
}

void printBytesSuitableForWiresharkImport(uint8_t* canId, std::string& canFrame){
#if 1
  printf("%06X", 0);
  printf(" %02X %02X %02X %02X", canId[0], canId[1], canId[2], canId[3]);

  for(int i=0; i<canFrame.length(); i++){
    printf(" %02X", canFrame[i]);
  }
  printf("\n\n");
#else
  ESP_LOGD(TAG, "canid=%d frame length=%d", *canId, canFrame.length());
#endif
}

void readFromMasterbusAndLog(){
  ESP_LOGD(TAG, "Begin");
//  g_mbctl->startCANBusPump();
//  g_mbctl->pumpQueue

  CANBusPacket packetRead;
  while(true){
    ESP_LOGD(TAG, "Reding packet from masterbus controller");
    if(g_mbctl->readPacket(packetRead)){
      std::string canFrame=packetRead.getData();
      printBytesSuitableForWiresharkImport((uint8_t*)&packetRead.canId, canFrame);
    }
    vTaskDelay(100);
  }
  ESP_LOGD(TAG, "End");
}

void mqttPublishHexValue(const char* topic, std::string& payloadBytes){
  char* hexBytes=bytesToHex((uint8_t*) payloadBytes.c_str(), payloadBytes.length());
  publishToMQTT(topic, hexBytes);
  free(hexBytes);
}

void taskForwardMasterBusPacketsToMQTT(){
  MvParser mvParser;
  char topic[128];
  char valueStr[128];

  g_mbctl->startCANBusPump();
  CANBusPacket rxPacket;
  while(true){
    if(xQueueReceive(g_mbctl->pumpQueue, &rxPacket, portMAX_DELAY)){
//      ESP_LOGD(__FUNCTION__, "Got a canbus packet off the queue. dataLen=%d", rxPacket.dataLen);
//      g_mbctl->hexdumpCanBusPacket(rxPacket);

      std::string payload((char*) rxPacket.data, rxPacket.dataLen);
      MastervoltMessage* mvMessage=mvParser.parseStdCanbusId(rxPacket.stdCanbusId, rxPacket.extCanbusId, payload);
      if(NULL==mvMessage){
        continue;
      }

      ESP_LOGI(TAG, "Parsed a mastervolt message %s", mvMessage->toString().c_str());
      sprintf(topic, "masterbus/0x%02X/0x%02X", mvMessage->deviceUniqueId, mvMessage->attributeId);
      if(MastervoltMessage::MastervoltMessageType::FLOAT == mvMessage->type) {
        sprintf(valueStr, "%f", mvMessage->value.floatValue);
        publishToMQTT(topic, valueStr);
      }
      else if(MastervoltMessage::MastervoltMessageType::DATE == mvMessage->type) {
        sprintf(valueStr, "%02d/%02d/%d", mvMessage->value.day, mvMessage->value.month, mvMessage->value.year);
        publishToMQTT(topic, valueStr);
        //TODO Set the device's local time from this
      }
      else if(MastervoltMessage::MastervoltMessageType::TIME == mvMessage->type) {
        sprintf(valueStr, "%02d:%02d:%02d", mvMessage->value.hour, mvMessage->value.minute, mvMessage->value.second);
        publishToMQTT(topic, valueStr);
        //TODO Set the device's local time from this
      }
      delete mvMessage;
    }
  }
  //  g_mbctl->stopCANBusPump();
}

void startTaskForwardCMasterBusPacketsToMQTT(){
//  xTaskCreatePinnedToCore(taskForwardCMasterBusPacketsToMQTT, "taskForwardCMasterBusPacketsToMQTT", 2048, this, 5, &taskForwardCMasterBusPacketsToMQTTHandle, 0);
}

void configureMasterbus() {
  initializeMasterbus();
#if 0
  readFromMasterbusAndLog();
#endif
}
