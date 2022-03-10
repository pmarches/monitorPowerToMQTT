#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <sstream>

const gpio_num_t MULTI_SERIAL_S3=GPIO_NUM_32;
const gpio_num_t MULTI_SERIAL_S2=GPIO_NUM_33;
const gpio_num_t MULTI_SERIAL_S1=GPIO_NUM_25;

void multiSerialSelect(uint8_t channel){
  ESP_LOGD(__FUNCTION__, "channel %d selected", channel);
  gpio_set_level(MULTI_SERIAL_S3, channel&0b100);
  gpio_set_level(MULTI_SERIAL_S2, channel&0b010);
  gpio_set_level(MULTI_SERIAL_S1, channel&0b001);
}

const gpio_num_t VEDIRECT_UART_TX=GPIO_NUM_17;
const gpio_num_t VEDIRECT_UART_RX=GPIO_NUM_16;
const uart_port_t VEDIRECT_UART_NUM=UART_NUM_2;

void configureVEDirectUART(){
  uart_config_t veDirectUartConfig;
  veDirectUartConfig.baud_rate = 19200;
  veDirectUartConfig.data_bits = UART_DATA_8_BITS;
  veDirectUartConfig.parity = UART_PARITY_DISABLE;
  veDirectUartConfig.stop_bits = UART_STOP_BITS_1;
  veDirectUartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  veDirectUartConfig.rx_flow_ctrl_thresh = 120;
  uart_param_config(VEDIRECT_UART_NUM, &veDirectUartConfig);
  uart_set_pin(VEDIRECT_UART_NUM, VEDIRECT_UART_TX, VEDIRECT_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  int TX_BUFFER_SIZE=0;
  int RX_BUFFER_SIZE=2048; //TODO
  uart_driver_install(VEDIRECT_UART_NUM, RX_BUFFER_SIZE, TX_BUFFER_SIZE, 0, NULL, 0);
}

void configureMultiserial() {
  gpio_set_direction(MULTI_SERIAL_S3, GPIO_MODE_OUTPUT);
  gpio_set_direction(MULTI_SERIAL_S2, GPIO_MODE_OUTPUT);
  gpio_set_direction(MULTI_SERIAL_S1, GPIO_MODE_OUTPUT);
}

void veDirectWriteStuff(){
  ESP_LOGD(__FUNCTION__, "Wrote ping");
  uart_write_bytes(VEDIRECT_UART_NUM, ":154\n", 5);
}

void veDirectReadStuff(TickType_t ticksToWait){
  unsigned char readBuffer[2048];
  int nbBytesRead = uart_read_bytes(VEDIRECT_UART_NUM, readBuffer, sizeof(readBuffer), ticksToWait);
  ESP_LOGD(__FUNCTION__, "nbBytesRead=%d", nbBytesRead);
  if (nbBytesRead >0) {
    ESP_LOGD(__FUNCTION__, "%.*s", nbBytesRead, readBuffer);
  }
}

const int DISABLED_CHANNEL=7;

uint32_t nbBytesPerChannel[]={0,0,0,0};

void testMultiSerial(){
  configureVEDirectUART();
  configureMultiserial();

  for(int channel=1;; channel=(channel+1)%4) {
    multiSerialSelect(channel);
    veDirectWriteStuff();
//    vTaskDelay(300 / portTICK_RATE_MS);

    char readBuffer[2048];
    while(true) {
      int nbBytesRead = uart_read_bytes(VEDIRECT_UART_NUM, readBuffer, sizeof(readBuffer), 200 / portTICK_RATE_MS);
      if(nbBytesRead<0){
        ESP_LOGE(__FUNCTION__, "read error\n");
        exit(1);
      }
      nbBytesPerChannel[channel]+=nbBytesRead;
      ESP_LOGD(__FUNCTION__, "Added %d bytes to ss", nbBytesRead);
      if(nbBytesRead<sizeof(readBuffer)){
        break;
      }
//    std::stringstream ss;
//    ss<<std::string(readBuffer, nbBytesRead);
    }

    multiSerialSelect(DISABLED_CHANNEL);
    //Empty any remaining bytes that may have sneaked in between the read an the channel change
    while(true) {
      int nbBytesRead = uart_read_bytes(VEDIRECT_UART_NUM, readBuffer, sizeof(readBuffer), 0);
      if(nbBytesRead==0){
        break;
      }
      else if(nbBytesRead<0){
        ESP_LOGE(__FUNCTION__, "Some error occured while reading UART buffer");
        break;
      }
      ESP_LOGD(__FUNCTION__, "Added stray %d bytes to ss", nbBytesRead);
//      ss<<std::string(readBuffer, nbBytesRead);
      nbBytesPerChannel[channel]+=nbBytesRead;
    }

    if(channel==0){
      ESP_LOGD(__FUNCTION__, "*****************");
      for(int i=0; i<4; i++){
        ESP_LOGD(__FUNCTION__, "Channel %d nbBytesRead=%d", i, nbBytesPerChannel[i]);
      }
      ESP_LOGD(__FUNCTION__, "*****************");
    }
  }
}
