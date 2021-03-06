#include <sstream>
#include <vector>
#include <map>
#include <math.h>

#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/timers.h>

#include <vhp_parser.h>
#include <vhp_command.h>
#include <vhp_registers.h>
#include <vhp_driver.h>
#include <vhp_batchstreaming.h>

#define TAG __FILE__

void publishToMQTT(const char* topic, const char* value);

class MultiplexedSerial : public VHPSerial {
public:
  const static int DISABLED_CHANNEL=5;
  const static uint8_t NB_MPPT=4;
  //Cable Order cable end first to the board: Blue,Orange,Brown, Green
  const static uint8_t BLUE_CHANNEL=7;
  const static uint8_t ORANGE_CHANNEL=6;
  const static uint8_t BROWN_CHANNEL=1;
  const static uint8_t GREEN_CHANNEL=0;
  const static uint8_t CHANNELS_TO_USE[MultiplexedSerial::NB_MPPT];

  const static gpio_num_t MULTI_SERIAL_S3_PIN=GPIO_NUM_15;
  const static gpio_num_t MULTI_SERIAL_S2_PIN=GPIO_NUM_16;
  const static gpio_num_t MULTI_SERIAL_S1_PIN=GPIO_NUM_32;

  const static gpio_num_t VEDIRECT_UART_TX_PIN=GPIO_NUM_33;
  const static gpio_num_t VEDIRECT_UART_RX_PIN=GPIO_NUM_39;
  const static uart_port_t VEDIRECT_UART_NUM=UART_NUM_2;

  MultiplexedSerial();
  void configureVEDirectUART();
  void configure();
  void selectChannel(uint8_t channel);
  virtual const std::string readLine();
  const std::string readBufferFully();
  virtual void writeHexLine(const std::string& hexLine);
};

const uint8_t MultiplexedSerial::CHANNELS_TO_USE[MultiplexedSerial::NB_MPPT]={
    MultiplexedSerial::GREEN_CHANNEL,
    MultiplexedSerial::BROWN_CHANNEL,
    MultiplexedSerial::ORANGE_CHANNEL,
    MultiplexedSerial::BLUE_CHANNEL,
};

class MPPTParsedSentenceCache {
  std::map<uint16_t, std::string> stringCache;
  std::map<uint16_t, uint32_t> unsignedCache;
  std::map<uint16_t, int32_t> signedCache;
public:
  /***
   * Returns true if the value for this register has changed
   */
  bool update(uint16_t registerId, std::string& value){
    std::string oldValue=stringCache[registerId];
    if(oldValue==value){
      return false;
    }
    stringCache[registerId]=value;
    return true;
  }
  bool update(uint16_t registerId, uint32_t value){
    uint32_t oldValue=unsignedCache[registerId];
    if(oldValue==value){
      return false;
    }
    unsignedCache[registerId]=value;
    return true;
  }
  bool update(uint16_t registerId, int32_t value){
    int32_t oldValue=signedCache[registerId];
    if(oldValue==value){
      return false;
    }
    signedCache[registerId]=value;
    return true;
  }
};

class MPPTRegisterModel {
  std::map<uint16_t, std::string> stringCache;
public:
  bool update(const uint16_t registerId, const std::string& newStringValue) {
    auto oldValueIt=stringCache.find(registerId);
    if(oldValueIt==stringCache.end()){
      stringCache[registerId]=newStringValue;
      return true;
    }
    if(oldValueIt->second==newStringValue){
      return false;
    }
    stringCache[registerId]=newStringValue;
    return true;
  }
};

class MPPTChannelDescriptor {
public:
  uint32_t nbBytesRx; //Stats to see if this work
  const ProductDescription *productDesc;
  std::string serialNumber;
  MPPTParsedSentenceCache cache;
};

MPPTChannelDescriptor channelDesc[MultiplexedSerial::NB_MPPT];

MultiplexedSerial multiSerial;
VHPDriver driver(&multiSerial);

class NetworkOfMPPTModel {
  uint32_t panelPowerPerMppt[MultiplexedSerial::NB_MPPT];

  void publishNetworkStats() {
    float networkPanelPower = 0.0;
    for (int i = 0; i < MultiplexedSerial::NB_MPPT; i++) {
      networkPanelPower += panelPowerPerMppt[i]*0.01;
    }
    char floatStr[16];
    sprintf(floatStr, "%.02f", networkPanelPower);
    publishToMQTT("vedirect/network/panelPower", floatStr);

    for (int i = 0; i < MultiplexedSerial::NB_MPPT; i++) {
      float shareOfNetwork=(panelPowerPerMppt[i]*0.01)/networkPanelPower;
      if(isnan(shareOfNetwork)){
        shareOfNetwork=0.0;
      }
      sprintf(floatStr, "%.02f", shareOfNetwork);
      char topic[128];
      sprintf(topic, "vedirect/%s/shareOfNetwork", channelDesc[i].serialNumber.c_str());
      publishToMQTT(topic, floatStr);
    }
  }

public:
  void updatePanelPower(const uint8_t mpptIndex, uint32_t newPanelPower){
    panelPowerPerMppt[mpptIndex]=newPanelPower;
    if(mpptIndex==0){
      publishNetworkStats();
    }
  }
};
NetworkOfMPPTModel mpptNetworkModel;

MultiplexedSerial::MultiplexedSerial() {}

void MultiplexedSerial::configureVEDirectUART(){
  uart_config_t veDirectUartConfig;
  veDirectUartConfig.baud_rate = 19200;
  veDirectUartConfig.data_bits = UART_DATA_8_BITS;
  veDirectUartConfig.parity = UART_PARITY_DISABLE;
  veDirectUartConfig.stop_bits = UART_STOP_BITS_1;
  veDirectUartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  veDirectUartConfig.rx_flow_ctrl_thresh = 120;
  uart_param_config(VEDIRECT_UART_NUM, &veDirectUartConfig);
  uart_set_pin(VEDIRECT_UART_NUM, VEDIRECT_UART_TX_PIN, VEDIRECT_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  int TX_BUFFER_SIZE=0;
  int RX_BUFFER_SIZE=2048; //TODO
  uart_driver_install(VEDIRECT_UART_NUM, RX_BUFFER_SIZE, TX_BUFFER_SIZE, 0, NULL, 0);
}

void MultiplexedSerial::configure() {
  gpio_pad_select_gpio(MULTI_SERIAL_S3_PIN);
  ESP_ERROR_CHECK(gpio_set_direction(MULTI_SERIAL_S3_PIN, GPIO_MODE_OUTPUT));
  gpio_pad_select_gpio(MULTI_SERIAL_S2_PIN);
  ESP_ERROR_CHECK(gpio_set_direction(MULTI_SERIAL_S2_PIN, GPIO_MODE_OUTPUT));
  gpio_pad_select_gpio(MULTI_SERIAL_S1_PIN);
  ESP_ERROR_CHECK(gpio_set_direction(MULTI_SERIAL_S1_PIN, GPIO_MODE_OUTPUT));
  selectChannel(MultiplexedSerial::GREEN_CHANNEL);
  configureVEDirectUART();
}

void MultiplexedSerial::selectChannel(uint8_t channel) {
  ESP_LOGD(TAG, "channel %d selected", channel);
  ESP_ERROR_CHECK(gpio_set_level(MULTI_SERIAL_S3_PIN, (channel&0b100)>>2));
  ESP_ERROR_CHECK(gpio_set_level(MULTI_SERIAL_S2_PIN, (channel&0b010)>>1));
  ESP_ERROR_CHECK(gpio_set_level(MULTI_SERIAL_S1_PIN, (channel&0b001)>>0));

  vTaskDelay(10 / portTICK_RATE_MS); //Let any remainign data from this channel come in
  uart_flush(MultiplexedSerial::VEDIRECT_UART_NUM); //Discards anything left in the UART RX fifo
}

const std::string MultiplexedSerial::readLine() {
#if 0
  std::string buffer=readBufferFully();

  size_t pos = 0;
  std::string line;
  while ((pos = buffer.find("\n")) != std::string::npos) {
      line = buffer.substr(0, pos+1);
      std::cout <<"Readline="<< line << std::endl;
      buffer.erase(0, pos+1);
  }
  return ":51641F9\n";
#else
  char readBuffer[256];
  int nbBytesRead=0;
  while(true){
    int ret = uart_read_bytes(MultiplexedSerial::VEDIRECT_UART_NUM, readBuffer+nbBytesRead, 1, 100 / portTICK_RATE_MS);
    if(ret<0){
      perror("Error reading from uart_read_bytes");
      exit(1);
    }
    if(ret==0 || '\n'==readBuffer[nbBytesRead]){
      return std::string(readBuffer, nbBytesRead+1);
    }
    if(nbBytesRead<sizeof(readBuffer)-1){ //Overflow will overwrite the last char until we reach a newline.
      nbBytesRead++;
    }
  }
#endif
}

const std::string MultiplexedSerial::readBufferFully() {
  std::stringstream ss;
  char readBuffer[256];
  while(true) {
    int nbBytesRead = uart_read_bytes(MultiplexedSerial::VEDIRECT_UART_NUM, readBuffer, sizeof(readBuffer), 100 / portTICK_RATE_MS);
    if(nbBytesRead<0){
      ESP_LOGE(TAG, "read error\n");
      exit(1);
    }
    ss.write(readBuffer, nbBytesRead);
    ESP_LOGD(TAG, "Added %d bytes to ss", nbBytesRead);
    if(nbBytesRead==0){
      break;
    }
  }

//  multiSerial.selectChannel(MultiplexedSerial::DISABLED_CHANNEL);

  return ss.str();
}

void MultiplexedSerial::writeHexLine(const std::string& hexLine){
  ESP_LOGD(TAG, "Write: '%.*s'", hexLine.size(), hexLine.c_str());
  int totalNbBytesWritten=0;
  while(true){
    int nbBytesWritten=uart_write_bytes(VEDIRECT_UART_NUM, hexLine.c_str()+totalNbBytesWritten, hexLine.size()-totalNbBytesWritten);
    totalNbBytesWritten+=nbBytesWritten;
    if(totalNbBytesWritten==hexLine.size()){
      break;
    }
  }
}


void getMPPTInformationForEachChannel(){
  const uint16_t startupRegisters[]={VHP_REG_PRODUCT_ID, VHP_REG_MODEL_NAME, VHP_REG_DEVICE_MODE};
  std::string startupHexCommand=VHPBatchGetRegisters(startupRegisters, sizeof(startupRegisters)/sizeof(uint16_t));
//  ESP_LOG_BUFFER_HEX_LEVEL(TAG, startupCommand.c_str(), startupCommand.size(), ESP_LOG_DEBUG);
//  ESP_LOGD(TAG, "startupCommand=%.*s", startupCommand.size(), startupCommand.c_str());
#if 1
  for(int i=0; i<MultiplexedSerial::NB_MPPT; i++){
    uint8_t channel=MultiplexedSerial::CHANNELS_TO_USE[i];
    ESP_LOGI(TAG, "Testing Channel %d", channel);
    multiSerial.selectChannel(channel);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
#endif

  //FIXME This code hangs if one of the MPPT is disconnected
  for(int i=0; i<MultiplexedSerial::NB_MPPT; i++){
    uint8_t channel=MultiplexedSerial::CHANNELS_TO_USE[i];
    multiSerial.selectChannel(channel);
#if 0
    multiSerial.writeHexLine(startupHexCommand); //TODO Sending the commands in batch might reduce latency, since we have 4 MPPTs to talk

    while(true){
      VHParsedSentence* sentence=readSentence();
      if(sentence==NULL){
        continue;
      }
    }
#else
    channelDesc[i].productDesc=driver.getProductId();
    channelDesc[i].serialNumber=driver.getSerialNumber();
#endif
//    vTaskDelay(2000 / portTICK_RATE_MS);
  }
  ESP_LOGI(TAG, "Done with all channel");

  for(int i=0; i<MultiplexedSerial::NB_MPPT; i++){
    uint8_t channel=MultiplexedSerial::CHANNELS_TO_USE[i];
    if(NULL!=channelDesc[i].productDesc) {
      ESP_LOGI(TAG, "Channel %d: is product %s, serial number %s", channel, channelDesc[i].productDesc->productName, channelDesc[i].serialNumber.c_str());
    }
    else{
      ESP_LOGI(TAG, "Channel %d: is NULL", channel);
    }
  }

//  while(true) vTaskDelay(1000 / portTICK_RATE_MS);
}

void formatSentenceAndPublishToMQTT(uint8_t mpptIndex, VHParsedSentence* sentence){
  if(NULL==sentence){
    ESP_LOGW(TAG, "Can't publish NULL sentence");
    return;
  }
  if(sentence->isRegister()==false){
    ESP_LOGW(TAG, "Don't know how to MQTT publish sentence that is not a register");
    return;
  }

  const RegisterDesc* registerDesc=lookupRegister(sentence->registerId);
  if(NULL==registerDesc){
    ESP_LOGW(TAG, "Did not find a description of register 0x%02X", sentence->registerId);
    return;
  }

  char topic[256];
  sprintf(topic, "vedirect/%s/0x%02X", channelDesc[mpptIndex].serialNumber.c_str(), sentence->registerId);

  char value[128];
  if(sentence->type==VHParsedSentence::SIGNED_REGISTER){
    sprintf(value, "%f", sentence->sentence.signedRegister->value*registerDesc->scale);
  }
  else if(sentence->type==VHParsedSentence::UNSIGNED_REGISTER){
    sprintf(value, "%f", sentence->sentence.unsignedRegister->value*registerDesc->scale);
  }
  else if(sentence->type==VHParsedSentence::STRING){
    sprintf(value, "%s", sentence->sentence.stringValue->c_str());
  }
  else {
    ESP_LOGW(TAG, "Don't know how to MQTT publish sentence type %d", sentence->type);
    return;
  }
  publishToMQTT(topic, value);
}

void acceptSentence(uint8_t mpptIndex, VHParsedSentence* sentence){
  if(NULL==sentence){
    ESP_LOGW(TAG, "Can't publish NULL sentence");
    return;
  }
  if(sentence->isRegister()==false){
    ESP_LOGW(TAG, "Don't know how to MQTT publish sentence that is not a register");
    delete sentence;
    return;
  }

  if(sentence->type==VHParsedSentence::STRING){
    bool needPublish=channelDesc[mpptIndex].cache.update(VHP_REG_PANEL_POWER, *sentence->sentence.stringValue);
    if(needPublish){
      formatSentenceAndPublishToMQTT(mpptIndex, sentence);
    }
  }
  else if(sentence->type==VHParsedSentence::SIGNED_REGISTER){
    bool needPublish=channelDesc[mpptIndex].cache.update(VHP_REG_PANEL_POWER, sentence->sentence.signedRegister->value);
    if(needPublish){
      formatSentenceAndPublishToMQTT(mpptIndex, sentence);
    }
  }
  else if(sentence->type==VHParsedSentence::UNSIGNED_REGISTER){
    bool needPublish=channelDesc[mpptIndex].cache.update(VHP_REG_PANEL_POWER, sentence->sentence.unsignedRegister->value);
    if(needPublish){
      formatSentenceAndPublishToMQTT(mpptIndex, sentence);
      if(sentence->registerId==VHP_REG_PANEL_POWER){
        mpptNetworkModel.updatePanelPower(mpptIndex, sentence->sentence.unsignedRegister->value);
      }
    }
  }
  delete sentence;
}

void streamingSpike(){
  ESP_LOGI(TAG, "Streaming begin");
  VHPBatchStreaming streaming(&multiSerial);

//  const uint16_t startupRegisters[]={VHP_REG_PRODUCT_ID, VHP_REG_MODEL_NAME, VHP_REG_DEVICE_MODE};
//  std::string startupHexCommand=VHPBatchGetRegisters(startupRegisters, sizeof(startupRegisters)/sizeof(uint16_t));

  while(true){
#if 1
    multiSerial.selectChannel(MultiplexedSerial::BROWN_CHANNEL);
    vTaskDelay(1000/portTICK_RATE_MS);
    streaming.parseOneSentence();

    multiSerial.selectChannel(MultiplexedSerial::GREEN_CHANNEL);
    vTaskDelay(1000/portTICK_RATE_MS);
    streaming.parseOneSentence();

    multiSerial.selectChannel(MultiplexedSerial::ORANGE_CHANNEL);
    vTaskDelay(1000/portTICK_RATE_MS);
    streaming.parseOneSentence();

    multiSerial.selectChannel(MultiplexedSerial::BLUE_CHANNEL);
    vTaskDelay(1000/portTICK_RATE_MS);
    streaming.parseOneSentence();
#else
    multiSerial.selectChannel(6);
    vTaskDelay(1000/portTICK_RATE_MS);
    streaming.parseOneSentence();

    multiSerial.selectChannel(7);
    vTaskDelay(1000/portTICK_RATE_MS);
    streaming.parseOneSentence();

#endif
  }
}

/***
 * How is this supposed to work?
 * 1- Select the channel
 * 2- Write the commands I want, in batch
 * 3- Read some bytes from it
 * 4- Close channel (by opening a non-connected channel)
 * 5- Read any bytes that may have sneaked in before the channel closed. This is required to ensure the local UART buffer is empty for the next read
 */
void taskForwardVEDirectSentenceToMQTT(void* arg){
  ESP_LOGI(TAG, "taskForwardVEDirectSentenceToMQTT starting");
  multiSerial.configure();
//  streamingSpike();
  getMPPTInformationForEachChannel();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(true){
    for(int i=0;i<MultiplexedSerial::NB_MPPT; i++) {
      uint8_t channel=MultiplexedSerial::CHANNELS_TO_USE[i];
      multiSerial.selectChannel(channel);
      acceptSentence(i, driver.getRegisterValue(VHP_REG_PANEL_POWER));
      acceptSentence(i, driver.getRegisterValue(VHP_REG_PANEL_VOLTAGE));
      acceptSentence(i, driver.getRegisterValue(VHP_REG_PANEL_CURRENT));
      acceptSentence(i, driver.getRegisterValue(VHP_REG_CHARGER_CURRENT));
      acceptSentence(i, driver.getRegisterValue(VHP_REG_CHARGER_VOLTAGE));
      acceptSentence(i, driver.getRegisterValue(VHP_REG_DEVICE_MODE));
    }
//    multiSerial.selectChannel(MultiplexedSerial::DISABLED_CHANNEL);
    vTaskDelayUntil(&xLastWakeTime, 1000/portTICK_RATE_MS);
  }
}
