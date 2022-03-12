#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <sstream>
#include <vector>
#include <freertos/timers.h>

#include <vhp_parser.h>
#include <vhp_command.h>
#include <vhp_registers.h>
#include <vhp_driver.h>

#define TAG __FILE__


class MultiplexedSerial : public VHPSerial {
public:
  const static int DISABLED_CHANNEL=7;
  const static uint8_t NB_CHANNELS=4;

  const static gpio_num_t MULTI_SERIAL_S3_PIN=GPIO_NUM_32;
  const static gpio_num_t MULTI_SERIAL_S2_PIN=GPIO_NUM_33;
  const static gpio_num_t MULTI_SERIAL_S1_PIN=GPIO_NUM_25;

  const static gpio_num_t VEDIRECT_UART_TX_PIN=GPIO_NUM_17;
  const static gpio_num_t VEDIRECT_UART_RX_PIN=GPIO_NUM_16;
  const static uart_port_t VEDIRECT_UART_NUM=UART_NUM_2;

  MultiplexedSerial();
  void configureVEDirectUART();
  void configure();
  void selectChannel(uint8_t channel);
  virtual const std::string readLine();
  const std::string readBufferFully();
  virtual void writeHexLine(const std::string& hexLine);
};

class MPPTChannelDescriptor {
public:
  uint32_t nbBytesRx; //Stats to see if this work
  const ProductDescription *productDesc;
  std::string serialNumber;
};

MPPTChannelDescriptor channelDesc[MultiplexedSerial::NB_CHANNELS];

MultiplexedSerial multiSerial;
VHPDriver driver(&multiSerial);

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
  gpio_set_direction(MULTI_SERIAL_S3_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(MULTI_SERIAL_S2_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(MULTI_SERIAL_S1_PIN, GPIO_MODE_OUTPUT);
  selectChannel(MultiplexedSerial::DISABLED_CHANNEL);
  configureVEDirectUART();
}

void MultiplexedSerial::selectChannel(uint8_t channel) {
  ESP_LOGD(TAG, "channel %d selected", channel);
  gpio_set_level(MULTI_SERIAL_S3_PIN, channel&0b100);
  gpio_set_level(MULTI_SERIAL_S2_PIN, channel&0b010);
  gpio_set_level(MULTI_SERIAL_S1_PIN, channel&0b001);
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
    int ret = uart_read_bytes(MultiplexedSerial::VEDIRECT_UART_NUM, readBuffer+nbBytesRead, 1, 200 / portTICK_RATE_MS);
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
    int nbBytesRead = uart_read_bytes(MultiplexedSerial::VEDIRECT_UART_NUM, readBuffer, sizeof(readBuffer), 200 / portTICK_RATE_MS);
    if(nbBytesRead<0){
      ESP_LOGE(TAG, "read error\n");
      exit(1);
    }
    ss.write(readBuffer, nbBytesRead);
    ESP_LOGD(TAG, "Added %d bytes to ss", nbBytesRead);
    if(nbBytesRead<sizeof(readBuffer)){
      break;
    }
  }

  multiSerial.selectChannel(MultiplexedSerial::DISABLED_CHANNEL);
  //Empty any remaining bytes that may have sneaked in between the read an the channel change
  while(true) {
    int nbBytesRead = uart_read_bytes(MultiplexedSerial::VEDIRECT_UART_NUM, readBuffer, sizeof(readBuffer), 0);
    if(nbBytesRead==0){
      break;
    }
    else if(nbBytesRead<0){
      ESP_LOGE(TAG, "Some error occured while reading UART buffer");
      break;
    }
    ESP_LOGD(TAG, "Added stray %d bytes to ss", nbBytesRead);
    ss.write(readBuffer, nbBytesRead);
  }
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

  for(int channel=0; channel<MultiplexedSerial::NB_CHANNELS; channel++){
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
    channelDesc[channel].productDesc=driver.getProductId();
    if(NULL!=channelDesc[channel].productDesc) {
      ESP_LOGI(TAG, "Found product %s", channelDesc[channel].productDesc->productName);
    }
    channelDesc[channel].serialNumber=driver.getSerialNumber();
    ESP_LOGI(TAG, "Serial number %s", channelDesc[channel].serialNumber.c_str());
#endif
//    vTaskDelay(2000 / portTICK_RATE_MS);
  }
}

void publishToMQTT(const char* topic, const char* value);
void mqttPublishVeDirectSentence(uint8_t channel, std::vector<VHParsedSentence*> sentences){
  for(int i=0; i<sentences.size(); i++){
    VHParsedSentence* sentence=sentences[i];
    if(NULL==sentence){
      ESP_LOGW(TAG, "Can't publish NULL sentence");
      continue;
    }
    if(sentence->isRegister()==false){
      ESP_LOGW(TAG, "Don't know how to MQTT publish sentence that is not a register");
      continue;
    }
    const RegisterDesc* registerDesc=lookupRegister(sentence->registerId);
    if(NULL==registerDesc){
      printf("Did not find a description of this register\n");
      continue;
    }

    char topic[256];
    sprintf(topic, "vedirect/MPPT_%s/0x%02X", channelDesc[channel].serialNumber.c_str(), sentence->registerId);

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
      continue;
    }
    publishToMQTT(topic, value);
    delete sentence;
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
  getMPPTInformationForEachChannel();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 3000/portTICK_RATE_MS;

  while(true){
    for(int channel=0;; channel=(channel+1)%MultiplexedSerial::NB_CHANNELS) {
      multiSerial.selectChannel(channel);
      std::vector<VHParsedSentence*> registerResults;
      registerResults.push_back(driver.getRegisterValue(VHP_REG_PANEL_POWER));
      registerResults.push_back(driver.getRegisterValue(VHP_REG_PANEL_VOLTAGE));
      registerResults.push_back(driver.getRegisterValue(VHP_REG_PANEL_CURRENT));
      registerResults.push_back(driver.getRegisterValue(VHP_REG_CHARGER_CURRENT));
      registerResults.push_back(driver.getRegisterValue(VHP_REG_CHARGER_VOLTAGE));
      registerResults.push_back(driver.getRegisterValue(VHP_REG_DEVICE_MODE));
      mqttPublishVeDirectSentence(channel, registerResults);
    }
    multiSerial.selectChannel(MultiplexedSerial::DISABLED_CHANNEL);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    xLastWakeTime = xTaskGetTickCount();
  }
}
