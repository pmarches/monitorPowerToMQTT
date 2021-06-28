#include <esp_log.h>
#include "masterbusController.h"
#include <mvParser.hpp>

#define SPI_CLK_PIN (gpio_num_t)13
#define SPI_MASTER_OUT_SLAVE_IN_PIN (gpio_num_t) 5
#define SPI_MASTER_IN_SLAVE_OUT_PIN (gpio_num_t) 4
#define SPI_CS_PIN (gpio_num_t) 3
#define SPI_INT_PIN (gpio_num_t)14

#define MYCANCTL_BUS SPI2_HOST

SPIBus spi(MYCANCTL_BUS);
MasterbusController* g_mbctl;

void configurePinAsOutput(gpio_num_t pin);
void publishToMQTT(const char* topic, const char* value);

void initializeMasterbus(){
  ESP_LOGD(__FUNCTION__, "Begin");

  gpio_reset_pin(SPI_CS_PIN);
  gpio_set_direction(SPI_CS_PIN, GPIO_MODE_OUTPUT);
  gpio_set_intr_type(SPI_CS_PIN, GPIO_INTR_DISABLE);
  gpio_set_level(SPI_CS_PIN, 1);

  configurePinAsOutput(SPI_CLK_PIN);
  configurePinAsOutput(SPI_MASTER_OUT_SLAVE_IN_PIN);
  gpio_set_direction(SPI_MASTER_IN_SLAVE_OUT_PIN, GPIO_MODE_INPUT);
  gpio_set_direction(SPI_INT_PIN, GPIO_MODE_INPUT);

  ESP_LOGD(__FUNCTION__, "init: mosi=%d, miso=%d, clk=%d, cs=%d", SPI_MASTER_OUT_SLAVE_IN_PIN, SPI_MASTER_IN_SLAVE_OUT_PIN, SPI_CLK_PIN, SPI_CS_PIN);
#if 1
  spi.init(SPI_MASTER_OUT_SLAVE_IN_PIN, SPI_MASTER_IN_SLAVE_OUT_PIN, SPI_CLK_PIN);
  SPIDevice* spiRx=new SPIDevice(&spi, SPI_CS_PIN);
  ESP_ERROR_CHECK(spiRx->init());
  MCP2515Class* mcp2515Rx=new MCP2515Class(spiRx, SPI_CS_PIN, SPI_INT_PIN, 8E6);
  g_mbctl=new MasterbusController(mcp2515Rx);
  while(true){
    if(g_mbctl->configure(MCP2515Class::NORMAL_MODE)==ESP_OK){
      break;
    }
    ESP_LOGW(__FUNCTION__, "Failed  to configure MCP2515, retrying");
  }
  ESP_LOGI(__FUNCTION__, "MCP2515 configured successfully");
#else
  esp_err_t errRc;
  gpio_num_t csPin=SPI_CS_PIN;
  spi_host_device_t   m_host=MYCANCTL_BUS;
#if 1
  gpio_num_t mosiPin=SPI_MASTER_OUT_SLAVE_IN_PIN;
  gpio_num_t misoPin=SPI_MASTER_IN_SLAVE_OUT_PIN;
  gpio_num_t clkPin=SPI_CLK_PIN;

  ESP_LOGD(__FUNCTION__, "init: mosi=%d, miso=%d, clk=%d, cs=%d", mosiPin, misoPin, clkPin, csPin);
  ESP_LOGI(__FUNCTION__, "... Initializing bus; host=%d", m_host);

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
    ESP_LOGE(__FUNCTION__, "spi_bus_initialize(): rc=%d", errRc);
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
  ESP_LOGI(__FUNCTION__, "... Adding device bus.");
  errRc = ::spi_bus_add_device(m_host, &dev_config, &m_handle);
  if (errRc != ESP_OK) {
    ESP_LOGE(__FUNCTION__, "spi_bus_add_device(): rc=%d", errRc);
    abort();
  }

#endif

  ESP_LOGD(__FUNCTION__, "End");
}

void printBytesSuitableForWiresharkImport(uint8_t* canId, std::string& canFrame){
  const int NB_BYTES_PER_LINE=16;

  printf("%06X", 0);
  printf(" %02X %02X %02X %02X", canId[0], canId[1], canId[2], canId[3]);

  for(int i=0; i<canFrame.length(); i++){
    printf(" %02X", canFrame[i]);
  }
  printf("\n\n");
}

void readFromMasterbusAndLog(){
  ESP_LOGD(__FUNCTION__, "Begin");
//  g_mbctl->startCANBusPump();
//  g_mbctl->pumpQueue

  CANBusPacket packetRead;
  while(true){
    if(g_mbctl->readPacket(packetRead)){
      std::string canFrame=packetRead.getData();
      printBytesSuitableForWiresharkImport((uint8_t*)&packetRead.canId, canFrame);
    }
  }
  ESP_LOGD(__FUNCTION__, "End");
}

class MastervoltVariant {
public:
  uint16_t deviceId=0;
  uint16_t attributeId=0;
  uint8_t encoding=0;
  enum ENCODING_TYPE {FLOAT=0x08, STRING=0x06, DATETIME};

  bool isRequest=0;
  float floatValue=0;

  float asFloat();
  bool isFloat() { return encoding==FLOAT; }
};

#include <cmath>
float MastervoltVariant::asFloat(){
  if(encoding!=FLOAT){
    return -666.666;
  }
  return floatValue;
}

void parseVariant(uint32_t canbusId, std::string stringToParse, MastervoltVariant& variant){
  uint32_t stdCanbusId=(canbusId&0xFFFC0000)>>18;
  uint32_t extCanbusId=(canbusId&0x0003FFFF);
  ESP_LOGD(__FUNCTION__, "canbusId=0x%X stdCanbusId=0x%X stdCanbusId=0x%X", canbusId, stdCanbusId, extCanbusId);
  ESP_LOG_BUFFER_HEX_LEVEL(__FUNCTION__, stringToParse.c_str(), stringToParse.length(), ESP_LOG_DEBUG);

  variant.deviceId=stdCanbusId&0x3FF;
  variant.isRequest=(stdCanbusId&0x400)!=0;
  variant.encoding=(uint8_t) ((canbusId&0x0F000000)>>24);
  const uint8_t* bytes=(const uint8_t*) stringToParse.c_str();
  variant.attributeId=*((uint16_t*) bytes);
  bytes+=2;
  ESP_LOGD(__FUNCTION__, ".deviceId=0x%X .encoding=0x%X .attributeId=0x%X .isRequest=%d", variant.deviceId, variant.encoding, variant.attributeId, variant.isRequest);
  if(MastervoltVariant::FLOAT==variant.encoding){
    variant.floatValue=*(float*)bytes;
    ESP_LOGD(__FUNCTION__, "Interpreting bytes as float : %f", variant.floatValue);
  }
  else {
    ESP_LOGW(__FUNCTION__, "Unhandeled encoding 0x%x", variant.encoding);
  }
}

void taskForwardCMasterBusPacketsToMQTT(){
  MvParser mvParser;
  char topic[256];
  char valueStr[256];

  g_mbctl->startCANBusPump();
  while(true){
    CANBusPacket rxPacket;
    if(xQueueReceive(g_mbctl->pumpQueue, &rxPacket, portMAX_DELAY)){
#if 0
      MastervoltMessage* mvMessage= mvParser.parseVariant(rxPacket.stdCanbusId, rxPacket.extCanbusId, mvParser.stringToParse);
      ESP_LOGI(__FUNCTION__, "Got canbus packet off the queue %s", mvMessage->toString().c_str());
      if(MastervoltMessageFloat* msgFloat=dynamic_cast<MastervoltMessageFloat*>(mvMessage)) {
        sprintf(topic, "masterbus/%X/%X", msgFloat->deviceKindId, msgFloat->attributeId);
        sprintf(valueStr, "%f", msgFloat->floatValue);
        publishToMQTT(topic, valueStr);
      }
      delete mvMessage;
#else
      std::string payloadStr=rxPacket.getData();
      printBytesSuitableForWiresharkImport((uint8_t*)&rxPacket.canId, payloadStr);
      MastervoltVariant variant;
      parseVariant(rxPacket.canId, payloadStr, variant);
      if(variant.isRequest){
        continue;
      }
      if(variant.isFloat()){
        sprintf(topic, "masterbus/%03X/%04X", variant.deviceId, variant.attributeId);
        sprintf(valueStr, "%f", variant.asFloat());
        publishToMQTT(topic, valueStr);
      }
#endif
    }
  }
  //  g_mbctl->stopCANBusPump();
}

void startTaskForwardCMasterBusPacketsToMQTT(){
}

void configureMasterbus() {
  initializeMasterbus();
#if 0
  readFromMasterbusAndLog();
#endif
}
