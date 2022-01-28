#include <esp_log.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include <cstring>

//#define TAG __FUNCTION__
#define TAG __FILE__

void processVEAdvertisement(uint8_t* macAddress, uint8_t* manufacturer_data, uint8_t manufacturer_data_len);

void configureBLEScan(){
  static esp_ble_scan_params_t ble_scan_params = {
      .scan_type              = BLE_SCAN_TYPE_PASSIVE,
      .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
      .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
      //Parameters as per : https://esp32.com/viewtopic.php?t=6707&start=30
      .scan_interval          = 0x80, //Used to be 50 MS
      .scan_window            = 0x20, //Used to be 30 MS
      .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
  };
  ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));
}

void gapClientCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
//  ESP_LOGD(TAG, "Begin gapClientCallback %d", event);
  if(ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT==event) {
    if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
      ESP_LOGE(TAG, "config local privacy failed, error code =%x", param->local_privacy_cmpl.status);
    }
    configureBLEScan();
  }
  else if(ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT==event) {
    ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_start_scanning(0xffffffff));
  }
  else if(ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT==event) {
    ESP_LOGE(TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EV");
  }
  else if(ESP_GAP_BLE_SCAN_START_COMPLETE_EVT==event) {
    //scan start complete event to indicate scan start successfully or failed
    if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
    }
  }
  else if(ESP_GAP_BLE_PASSKEY_REQ_EVT==event) {                           /* passkey request event */
    ESP_LOGI(TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
  }
  else if(ESP_GAP_BLE_OOB_REQ_EVT==event) {
    ESP_LOGI(TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
    uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
    esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
  }
  else if(ESP_GAP_BLE_LOCAL_IR_EVT==event) {                               /* BLE local IR event */
    ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
  }
  else if(ESP_GAP_BLE_LOCAL_ER_EVT==event) {                               /* BLE local ER event */
    ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
  }
  else if(ESP_GAP_BLE_SEC_REQ_EVT==event) {
    /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
    esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
  }
  else if(ESP_GAP_BLE_NC_REQ_EVT==event){
    /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
    esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
    ESP_LOGI(TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
  }
  else if(ESP_GAP_BLE_PASSKEY_NOTIF_EVT==event) {  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
    ///show the passkey number to the user to input it in the peer device.
    ESP_LOGI(TAG, "The passkey Notify number:%06d", param->ble_security.key_notif.passkey);
  }
  else if(ESP_GAP_BLE_KEY_EVT==event) {
    //shows the ble key info share with peer device to the user.
#if 0
    ESP_LOGI(TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
#endif
  }
  else if(ESP_GAP_BLE_AUTH_CMPL_EVT==event) {
    esp_bd_addr_t bd_addr;
    memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
    ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x", (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3], (bd_addr[4] << 8) + bd_addr[5]);
    ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
    ESP_LOGI(TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
    if (!param->ble_security.auth_cmpl.success) {
      ESP_LOGI(TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
    }
    else {
#if 0
      ESP_LOGD(TAG, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
#endif
    }
  }
  else if(ESP_GAP_BLE_SCAN_RESULT_EVT==event) {
    esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

    if(ESP_GAP_SEARCH_INQ_RES_EVT==scan_result->scan_rst.search_evt) {
      ESP_LOGD(TAG, "ESP_GAP_SEARCH_INQ_RES_EVT");
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t), ESP_LOG_DEBUG);
//      ESP_LOGD(TAG, "Found Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);

//      adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
//      ESP_LOGD(TAG, "Found Device Name Len %d", adv_name_len);
//      esp_log_buffer_char(TAG, adv_name, adv_name_len);
//      ESP_LOGD(TAG, "\n");

      uint8_t manufacturer_data_len = 0;
      uint8_t *manufacturer_data = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &manufacturer_data_len);
      ESP_LOGD(TAG, "Manufacturer data len %d", manufacturer_data_len);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, manufacturer_data, manufacturer_data_len, ESP_LOG_DEBUG);
      uint8_t VICTRON_ENERGY_MANUFACTURER[]={0xE1, 0x02};
      if(manufacturer_data_len>2 && memcmp(manufacturer_data, VICTRON_ENERGY_MANUFACTURER, 2)==0){
        processVEAdvertisement(scan_result->scan_rst.bda, manufacturer_data, manufacturer_data_len);
      }
    }
    else if(ESP_GAP_SEARCH_INQ_CMPL_EVT==scan_result->scan_rst.search_evt) {
      ESP_LOGI(TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT"); //Scan completed
      esp_ble_gap_start_scanning(0xffffffff);
    }
    else {
      ESP_LOGW(TAG, "Unhandled scan search event %d", scan_result->scan_rst.search_evt);
    }
  }
  else if(ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT==event) {
    if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
      ESP_LOGE(TAG, "Scan stop failed, error status = %x", param->scan_stop_cmpl.status);
    }
    ESP_LOGI(TAG, "Stop scan successfully");
  }
  else if(ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT==event) {
    ESP_LOGI(TAG, "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
    ESP_LOGI(TAG, "Status %d", param->update_conn_params.status);
  }
  else {
    ESP_LOGW(TAG, "Unhandled GAP event %d", event);
  }
//  ESP_LOGD(TAG, "End esp_gap_cb");
}

void configureBLENetworking(){
  ESP_LOGD(TAG, "Begin");
  esp_err_t ret=ESP_OK;

  esp_bt_controller_status_t status=esp_bt_controller_get_status();
  ESP_LOGD(TAG, "status=%d", status);

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
      ESP_LOGE(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
      return;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
      ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
      return;
  }

//  ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
//  if (ret) {
//      ESP_LOGD(TAG, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
//      return;
//  }
  ret = esp_bluedroid_init();
  if (ret) {
      ESP_LOGE(TAG, "%s init bluetooth failed, error code = %x\n", __func__, ret);
      return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
      ESP_LOGE(TAG, "%s enable bluetooth failed, error code = %x\n", __func__, ret);
      return;
  }

  //register the  callback function to the gap module
  ret = esp_ble_gap_register_callback(gapClientCallback);
  if (ret){
    ESP_LOGE(TAG, "esp_ble_gap_register_callback failed = %s\n", esp_err_to_name(ret));
    return;
  }
  configureBLEScan();
}

