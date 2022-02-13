#include <nvs_flash.h>
#include <esp_log.h>

#define TAG __FUNCTION__

#define STATS_STORAGE_NS "powerstats"
#define AVERAGES_STORAGE_NS "averages"

void configureFlash(){
  esp_err_t ret=ESP_OK;
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );
}

void writeStatsToFlash(){
  nvs_handle_t powerstatsStorageHandle;

  ESP_ERROR_CHECK(nvs_open(STATS_STORAGE_NS, NVS_READWRITE, &powerstatsStorageHandle));

  size_t averagesStorageSize = 0;  // value will default to 0, if not set yet in NVS
  esp_err_t err = nvs_get_blob(powerstatsStorageHandle, AVERAGES_STORAGE_NS, NULL, &averagesStorageSize);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND){
    ESP_LOGE(TAG, "Failed to nvs_get_blob. err=%d", err);
    nvs_close(powerstatsStorageHandle);
    return;
  }

  ESP_ERROR_CHECK(nvs_commit(powerstatsStorageHandle));
  nvs_close(powerstatsStorageHandle);
}
