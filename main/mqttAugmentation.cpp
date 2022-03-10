#include <cstddef>
#include <time.h>
#include <sys/time.h>
#include <esp_log.h>
#include <esp_sntp.h>

#include <cmath>

#define TAG __FUNCTION__

class MovingWindowAverage {
public:
  float sum;
  size_t nbSamples;

  MovingWindowAverage() : sum(0), nbSamples(0) {}

  void addSample(float newSample){
    sum+=newSample;
    nbSamples++;
  }

  float getAverage(){
    if(nbSamples==0){
      return std::nanf("No samples");
    }
    return sum/nbSamples;
  }

  void reset(){
    sum=0.0;
    nbSamples=0;
  }
};

typedef enum {
  BATTVOLTAGE=0,
  AMPSFLOW,
  AMPSCONSUMED,
  MAX_STAT_ITEM
} ENUM_STAT_ITEM;
MovingWindowAverage averages[MAX_STAT_ITEM];
//TimeSerie timeSeries[MAX_STAT_ITEM];

void onSaveToFlash(void* arg){

}

void acceptStatItem(ENUM_STAT_ITEM statItem, float newValue){
  averages[statItem].addSample(newValue);
}

void onStatisticsTimerExhausted(void* args){
  for(int i=0; i<MAX_STAT_ITEM; i++){
    float averageForThisTimerPeriod=averages[i].getAverage();
    if(std::isnan(averageForThisTimerPeriod)){
      continue;
    }
    //  timeSeries[statItem].
    averages[i].reset();
  }
}

void configureMqttAugmentation() {
  acceptStatItem(BATTVOLTAGE, 12.34);
  acceptStatItem(AMPSFLOW, -45.34);
  acceptStatItem(AMPSCONSUMED, 160.3);

  esp_timer_create_args_t timerArgs={
      .callback=onStatisticsTimerExhausted,
      .arg=NULL,
      .dispatch_method=ESP_TIMER_TASK,
      .name="Statistics timer",
      .skip_unhandled_events= true
  };
  esp_timer_handle_t timerHandle;
  ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &timerHandle));
  const int STAT_TIMER_MINUTES=15;
  ESP_ERROR_CHECK(esp_timer_start_periodic(timerHandle, 10E6*60*STAT_TIMER_MINUTES));
  //FIXME: The timer here starts now, not on top of the 15 minutes window...
}

void getLocalTimeFromNetwork(){
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
//  sntp_setservername(0, CONFIG_MPTM_SNTP_HOSTNAME); //Use this only if the DHCP server does not provide a SNTP server address
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  sntp_init();
  //The chrony SNTP server does not get GPS time if instruments are turned off. It thus sets is local stratum to 16 (KOD). Fix this with the command:
  //chronyc> local stratum 3
  //200 OK

#define INET6_ADDRSTRLEN 48
  for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i){
    if (sntp_getservername(i)) {
      ESP_LOGI(TAG, "server %d: %s", i, sntp_getservername(i));
    } else {
      // we have either IPv4 or IPv6 address, let's print it
      char buff[INET6_ADDRSTRLEN];
      ip_addr_t const *ip = sntp_getserver(i);
      if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
        ESP_LOGI(TAG, "server %d: %s", i, buff);
    }
  }

  while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) {
      ESP_LOGI(TAG, "Waiting for system time to be set...");
      vTaskDelay(2000 / portTICK_PERIOD_MS);

      struct timeval tv;
      if (gettimeofday(&tv, NULL)!= 0) {
        ESP_LOGE(TAG, "Failed to obtain time");
        return;
      }
      ESP_LOGI(TAG, "sec:%ld usec=%ld", tv.tv_sec, tv.tv_usec);
  }
}
