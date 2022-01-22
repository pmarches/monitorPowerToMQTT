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
  ESP_LOGD(TAG, "Begin");

  gpio_reset_pin(SPI_CS_PIN);
  gpio_set_direction(SPI_CS_PIN, GPIO_MODE_OUTPUT);
  gpio_set_intr_type(SPI_CS_PIN, GPIO_INTR_DISABLE);
  gpio_set_level(SPI_CS_PIN, 1);

  configurePinAsOutput(SPI_CLK_PIN);
  configurePinAsOutput(SPI_MASTER_OUT_SLAVE_IN_PIN);
  gpio_set_direction(SPI_MASTER_IN_SLAVE_OUT_PIN, GPIO_MODE_INPUT);
  gpio_set_direction(SPI_INT_PIN, GPIO_MODE_INPUT);

  ESP_LOGD(TAG, "init: mosi=%d, miso=%d, clk=%d, cs=%d", SPI_MASTER_OUT_SLAVE_IN_PIN, SPI_MASTER_IN_SLAVE_OUT_PIN, SPI_CLK_PIN, SPI_CS_PIN);
#if 1
  spi.init(SPI_MASTER_OUT_SLAVE_IN_PIN, SPI_MASTER_IN_SLAVE_OUT_PIN, SPI_CLK_PIN);
  SPIDevice* spiRx=new SPIDevice(&spi, SPI_CS_PIN);
  ESP_ERROR_CHECK(spiRx->init());
  MCP2515Class* mcp2515Rx=new MCP2515Class(spiRx, SPI_CS_PIN, SPI_INT_PIN, 8E6);
  g_mbctl=new MasterbusController(mcp2515Rx);
#if 0
  while(true){
    if(g_mbctl->configure(MCP2515Class::NORMAL_MODE)==ESP_OK){
      break;
    }
    ESP_LOGW(TAG, "Failed  to configure MCP2515, retrying");
  }
#else
  ESP_ERROR_CHECK(g_mbctl->configure(MCP2515Class::NORMAL_MODE));
#endif
  ESP_LOGI(TAG, "MCP2515 configured successfully");
#else
  esp_err_t errRc;
  gpio_num_t csPin=SPI_CS_PIN;
  spi_host_device_t   m_host=MYCANCTL_BUS;
#if 1
  gpio_num_t mosiPin=SPI_MASTER_OUT_SLAVE_IN_PIN;
  gpio_num_t misoPin=SPI_MASTER_IN_SLAVE_OUT_PIN;
  gpio_num_t clkPin=SPI_CLK_PIN;

  ESP_LOGD(TAG, "init: mosi=%d, miso=%d, clk=%d, cs=%d", mosiPin, misoPin, clkPin, csPin);
  ESP_LOGI(TAG, "... Initializing bus; host=%d", m_host);

  spi_bus_config_t bus_config = {
    .mosi_io_num     = mosiPin, // MOSI
    .miso_io_num     = misoPin, // MISO
    .sclk_io_num     = clkPin,  // CLK
    .quadwp_io_num   = -1,      // Not used
    .quadhd_io_num   = -1,      // Not used
    .max_transfer_sz = 0,       // 0 means use default.
    .flags           = (SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MISO),
    .intr_flags =0,
  };

  errRc = ::spi_bus_initialize(m_host, &bus_config, 1 /*DMA Channel*/);
  if (errRc != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_initialize(): rc=%d", errRc);
    abort();
  }
  return;
#endif

  spi_device_handle_t m_handle;
  spi_device_interface_config_t dev_config;
  dev_config.address_bits     = 0;
  dev_config.command_bits     = 0;
  dev_config.dummy_bits       = 0;
  dev_config.mode             = 0;
  dev_config.duty_cycle_pos   = 0;
  dev_config.cs_ena_posttrans = 0;
  dev_config.cs_ena_pretrans  = 0;
  dev_config.clock_speed_hz   = 100000;
  dev_config.spics_io_num     = csPin;
  dev_config.flags            = SPI_DEVICE_NO_DUMMY;
  dev_config.queue_size       = 1;
  dev_config.pre_cb           = NULL;
  dev_config.post_cb          = NULL;
  ESP_LOGI(TAG, "... Adding device bus.");
  errRc = ::spi_bus_add_device(m_host, &dev_config, &m_handle);
  if (errRc != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_add_device(): rc=%d", errRc);
    abort();
  }

#endif

  ESP_LOGD(TAG, "End");
}

void printBytesSuitableForWiresharkImport(uint8_t* canId, std::string& canFrame){
  const int NB_BYTES_PER_LINE=16;
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

#define MQTT_TOPIC_FORMAT "masterbus/0x%02X/0x%02X"
void taskForwardCMasterBusPacketsToMQTT(){
  MvParser mvParser;
  char topic[256];
  char valueStr[256];

  g_mbctl->startCANBusPump();
  while(true){
    CANBusPacket rxPacket;
    if(xQueueReceive(g_mbctl->pumpQueue, &rxPacket, portMAX_DELAY)){
//      ESP_LOGD(__FUNCTION__, "Got a canbus packet off the queue. dataLen=%d", rxPacket.dataLen);
//      g_mbctl->hexdumpCanBusPacket(rxPacket);

      MastervoltMessage* mvMessage=mvParser.parseStdCanbusId(rxPacket.stdCanbusId, rxPacket.extCanbusId, std::string((char*) rxPacket.data, rxPacket.dataLen));
      if(NULL==mvMessage){
        continue;
      }

      ESP_LOGI(TAG, "Parsed a mastervolt message %s", mvMessage->toString().c_str());
      if(MastervoltMessage::MastervoltMessageType::FLOAT == mvMessage->type) {
        MastervoltMessageFloat* msgFloat=(MastervoltMessageFloat*) mvMessage;
        sprintf(topic, MQTT_TOPIC_FORMAT, msgFloat->deviceUniqueId, msgFloat->attributeId);
        sprintf(valueStr, "%f", msgFloat->floatValue);
        publishToMQTT(topic, valueStr);
      }
      else if(MastervoltMessage::MastervoltMessageType::DATE == mvMessage->type) {
        MastervoltMessageDate* msgDate=(MastervoltMessageDate*) mvMessage;
        sprintf(topic, MQTT_TOPIC_FORMAT, msgDate->deviceUniqueId, msgDate->attributeId);
        sprintf(valueStr, "%02d/%02d/%d", msgDate->day, msgDate->month, msgDate->year);
        publishToMQTT(topic, valueStr);
        //TODO Set the device's local time from this
      }
      else if(MastervoltMessage::MastervoltMessageType::TIME == mvMessage->type) {
        MastervoltMessageTime* msgTime=(MastervoltMessageTime*) mvMessage;
        sprintf(topic, MQTT_TOPIC_FORMAT, msgTime->deviceUniqueId, msgTime->attributeId);
        sprintf(valueStr, "%02d:%02d:%02d", msgTime->hour, msgTime->minute, msgTime->second);
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
