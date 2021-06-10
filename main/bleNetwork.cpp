#include <esp_log.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include <cstring>

void processVEAdvertisement(uint8_t* macAddress, uint8_t* manufacturer_data, uint8_t manufacturer_data_len);

void gapClientCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
//  ESP_LOGD(__FUNCTION__, "Begin gapClientCallback %d", event);
//  uint8_t *adv_name = NULL;
//  uint8_t adv_name_len = 0;
  if(ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT==event) {
    if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
      ESP_LOGE(__FUNCTION__, "config local privacy failed, error code =%x", param->local_privacy_cmpl.status);
    }
#if 0
    //  ESP_ERROR_CHECK(esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, ADDRESS_440_WATT, BLE_ADDR_TYPE_RANDOM, true));
#else
    esp_ble_scan_params_t ble_scan_params;
    memset(&ble_scan_params, 0, sizeof(ble_scan_params)); // Initialize all params
    ble_scan_params.scan_type          = BLE_SCAN_TYPE_PASSIVE; // Default is a passive scan.
    ble_scan_params.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
    ble_scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
    ble_scan_params.scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE;

    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (scan_ret){
      ESP_LOGE(__FUNCTION__, "set scan params error, error code = %x", scan_ret);
    }
#endif
  }
  else if(ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT==event) {
    //    if(appState==VEBT_DO_SCAN){
    //      //the unit of the duration is second
    //      uint32_t duration = 30;
    //      esp_ble_gap_start_scanning(duration);
    //    }
  }
  else if(ESP_GAP_BLE_SCAN_START_COMPLETE_EVT==event) {
    //scan start complete event to indicate scan start successfully or failed
    if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(__FUNCTION__, "scan start failed, error status = %x", param->scan_start_cmpl.status);
    }
  }
  else if(ESP_GAP_BLE_PASSKEY_REQ_EVT==event) {                           /* passkey request event */
    ESP_LOGD(__FUNCTION__, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
#if 0
    uint32_t victronPassKey=123456;
    /* Call the following function to input the passkey which is displayed on the remote device */
    esp_ble_passkey_reply(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, true, victronPassKey);
#endif
  }
  else if(ESP_GAP_BLE_OOB_REQ_EVT==event) {
    ESP_LOGD(__FUNCTION__, "ESP_GAP_BLE_OOB_REQ_EVT");
    uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
    esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
  }
  else if(ESP_GAP_BLE_LOCAL_IR_EVT==event) {                               /* BLE local IR event */
    ESP_LOGD(__FUNCTION__, "ESP_GAP_BLE_LOCAL_IR_EVT");
  }
  else if(ESP_GAP_BLE_LOCAL_ER_EVT==event) {                               /* BLE local ER event */
    ESP_LOGD(__FUNCTION__, "ESP_GAP_BLE_LOCAL_ER_EVT");
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
    ESP_LOGD(__FUNCTION__, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
  }
  else if(ESP_GAP_BLE_PASSKEY_NOTIF_EVT==event) {  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
    ///show the passkey number to the user to input it in the peer device.
    ESP_LOGD(__FUNCTION__, "The passkey Notify number:%06d", param->ble_security.key_notif.passkey);
  }
  else if(ESP_GAP_BLE_KEY_EVT==event) {
    //shows the ble key info share with peer device to the user.
#if 0
    ESP_LOGD(__FUNCTION__, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
#endif
  }
  else if(ESP_GAP_BLE_AUTH_CMPL_EVT==event) {
    esp_bd_addr_t bd_addr;
    memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
    ESP_LOGD(__FUNCTION__, "remote BD_ADDR: %08x%04x", (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3], (bd_addr[4] << 8) + bd_addr[5]);
    ESP_LOGD(__FUNCTION__, "address type = %d", param->ble_security.auth_cmpl.addr_type);
    ESP_LOGD(__FUNCTION__, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
    if (!param->ble_security.auth_cmpl.success) {
      ESP_LOGD(__FUNCTION__, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
    }
    else {
#if 0
      ESP_LOGD(__FUNCTION__, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
#endif
    }
  }
  else if(ESP_GAP_BLE_SCAN_RESULT_EVT==event) {
    esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

    if(ESP_GAP_SEARCH_INQ_RES_EVT==scan_result->scan_rst.search_evt) {
      ESP_LOGD(__FUNCTION__, "ESP_GAP_SEARCH_INQ_RES_EVT");
      ESP_LOG_BUFFER_HEX_LEVEL(__FUNCTION__, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t), ESP_LOG_DEBUG);
//      ESP_LOGD(__FUNCTION__, "Found Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);

//      adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
//      ESP_LOGD(__FUNCTION__, "Found Device Name Len %d", adv_name_len);
//      esp_log_buffer_char(__FUNCTION__, adv_name, adv_name_len);
//      ESP_LOGD(__FUNCTION__, "\n");

      uint8_t manufacturer_data_len = 0;
      uint8_t *manufacturer_data = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &manufacturer_data_len);
      ESP_LOGD(__FUNCTION__, "Manufacturer data len %d", manufacturer_data_len);
      ESP_LOG_BUFFER_HEX_LEVEL(__FUNCTION__, manufacturer_data, manufacturer_data_len, ESP_LOG_DEBUG);
      uint8_t VICTRON_ENERGY_MANUFACTURER[]={0xE1, 0x02};
      if(manufacturer_data_len>2 && memcmp(manufacturer_data, VICTRON_ENERGY_MANUFACTURER, 2)==0){
        processVEAdvertisement(scan_result->scan_rst.bda, manufacturer_data, manufacturer_data_len);
//        bool is440WattController=memcmp(scan_result->scan_rst.bda, ADDRESS_440_WATT, sizeof(esp_bd_addr_t))==0;
//        if (appState!=VEBT_DO_CONNECT && is440WattController) {
//          appState=VEBT_DO_CONNECT;
          //          ESP_LOGD(__FUNCTION__, "connect to the remote device.");
          //          esp_ble_gap_stop_scanning();
          //          ESP_ERROR_CHECK(esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true));
//        }
      }
    }
    else if(ESP_GAP_SEARCH_INQ_CMPL_EVT==scan_result->scan_rst.search_evt) {
      ESP_LOGD(__FUNCTION__, "ESP_GAP_SEARCH_INQ_CMPL_EVT"); //Scan completed
      esp_ble_gap_start_scanning(0xffffffff);
    }
    else {
      ESP_LOGD(__FUNCTION__, "Unhandled scan search event %d", scan_result->scan_rst.search_evt);
    }
  }
  else if(ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT==event) {
    if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
      ESP_LOGE(__FUNCTION__, "Scan stop failed, error status = %x", param->scan_stop_cmpl.status);
    }
    ESP_LOGD(__FUNCTION__, "Stop scan successfully");
  }
  else if(ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT==event) {
    ESP_LOGD(__FUNCTION__, "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
    ESP_LOGD(__FUNCTION__, "Status %d", param->update_conn_params.status);
  }
  else {
    ESP_LOGD(__FUNCTION__, "Unhandled GAP event %d", event);
  }
//  ESP_LOGD(__FUNCTION__, "End esp_gap_cb");
}

void configureBLEScan(){
  static esp_ble_scan_params_t ble_scan_params = {
      .scan_type              = BLE_SCAN_TYPE_PASSIVE,
      .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
      .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_interval          = 0x50,
      .scan_window            = 0x30,
      .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
  };
  esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
  if (scan_ret){
      ESP_LOGE(__FUNCTION__, "set scan params error, error code = %x", scan_ret);
  }
  esp_ble_gap_start_scanning(0xffffffff);
}

void configureBLENetworking(){
  ESP_LOGD(__FUNCTION__, "Begin");
  esp_err_t ret=ESP_OK;

  esp_bt_controller_status_t status=esp_bt_controller_get_status();
  ESP_LOGD(__FUNCTION__, "status=%d", status);

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
      ESP_LOGD(__FUNCTION__, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
      return;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
      ESP_LOGD(__FUNCTION__, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
      return;
  }

//  ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
//  if (ret) {
//      ESP_LOGD(__FUNCTION__, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
//      return;
//  }
  ret = esp_bluedroid_init();
  if (ret) {
      ESP_LOGE(__FUNCTION__, "%s init bluetooth failed, error code = %x\n", __func__, ret);
      return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
      ESP_LOGE(__FUNCTION__, "%s enable bluetooth failed, error code = %x\n", __func__, ret);
      return;
  }

  //register the  callback function to the gap module
  ret = esp_ble_gap_register_callback(gapClientCallback);
  if (ret){
    ESP_LOGE(__FUNCTION__, "esp_ble_gap_register_callback failed = %s\n", esp_err_to_name(ret));
    return;
  }
  configureBLEScan();
}

