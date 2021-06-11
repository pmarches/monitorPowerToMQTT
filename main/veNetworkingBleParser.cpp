#include <esp_log.h>
#include <arpa/inet.h>
#include <stdio.h>

#include "mbedtls/aes.h"
#include <string.h>
#include <cstring>

//stringUtils.cpp
char* bytesToHex(uint8_t* bytes, int bytesLen);
size_t hexToBytes(const char* hexString, uint8_t* resultByteArr, size_t resultByteArrSize);

//veAddress.cpp
void serialBytesToHumanRedable(uint8_t* serialBytes, char humanStr[12]);

uint8_t g_networkId[2]={0,0};
mbedtls_aes_context aesCtx;

#define VE_NETWORKING_SERIAL_BYTE_LEN 4
#define VE_NETWORKING_OVERFLOW_BYTE_LEN 2
#define VE_NETWORKING_SEQUENCE_BYTE_LEN 4
#define VE_NETWORKING_IV_BYTE_LEN 13
#define VE_NETWORKING_TAG_BYTE_LEN 4

struct DeviceOverflowRecord {
  uint8_t serial[VE_NETWORKING_SERIAL_BYTE_LEN];
  uint8_t overflow[VE_NETWORKING_OVERFLOW_BYTE_LEN];
//  char humanName[32];
} g_deviceOverflowCounter[16];
uint8_t g_deviceOverflowCounterSize=0;

//from mqttNetwork.cpp
void publishToMQTT(const char* topic, const char* value);

void configureVeNetworking(){
  hexToBytes(CONFIG_MPTM_VE_NETWORKING_NETWORK_ID, g_networkId, 2);
  ESP_LOGI(__FUNCTION__, "g_networkId %02X%02X", g_networkId[0], g_networkId[1]);

  memset(g_deviceOverflowCounter, 0, sizeof(g_deviceOverflowCounter));

  uint8_t keyBytes[16];
  hexToBytes(CONFIG_MPTM_VE_NETWORKING_KEY, keyBytes, 16);
  char* hexKey=bytesToHex(keyBytes, 16);
  ESP_LOGI(__FUNCTION__, "hexKey=%s", hexKey);
  free(hexKey);
  mbedtls_aes_init(&aesCtx);
  mbedtls_aes_setkey_dec(&aesCtx, keyBytes, 128);
}

char* macToStr(uint8_t* macBytes){
  char* hexString=(char*)malloc(6*3+1);
  sprintf(hexString, "%02X:%02X:%02X:%02X:%02X:%02X", macBytes[0], macBytes[1], macBytes[2], macBytes[3], macBytes[4], macBytes[5]);
  return hexString;
}
/**
 * Some registers are of little interest for us.
 */
bool shouldRegisterBePublished(uint16_t veRegister) {
  if(0x0100==veRegister || 0x0102==veRegister) {
    return false;
  }
  return true;
}

void parseCleartextVeNetworkingBytes(uint8_t* macAddress, uint8_t* serialBytes, uint8_t* clearText, uint8_t clearTextLen){
  char* serialHexStr=bytesToHex(serialBytes, VE_NETWORKING_SERIAL_BYTE_LEN);
  char* clearTextHexStr=bytesToHex(clearText, clearTextLen);
  char* macHexStr=macToStr(macAddress);
#if 0
  printf("CSV_BEGIN,%s,%s,%s,CSV_END\n", macHexStr, serialHexStr, clearTextHexStr);
#endif
  char topicStr[256];

  if(0x0f == clearText[0] || 0x08 == clearText[0]){
    for(uint8_t i=1; i<clearTextLen;){
      uint16_t* veRegister=(uint16_t*) (clearText+i);
      i+=2;

      uint8_t* nbBytesForValue=clearText+i;
      i+=1;

      char valueStr[64];
      if(*nbBytesForValue==4){
        uint32_t* intPtr=(uint32_t*) (clearText+i);
        sprintf(valueStr, "%u", *intPtr);
      }
      else if(*nbBytesForValue==2){
        uint16_t* intPtr=(uint16_t*) (clearText+i);
        sprintf(valueStr, "%hu", *intPtr);
      }
      else if(*nbBytesForValue==1){
        uint8_t* intPtr=(uint8_t*) (clearText+i);
        sprintf(valueStr, "%u", *intPtr);
      }
      else {
        ESP_LOGE(__FUNCTION__, "Unhandeled nbBytes=%d", *nbBytesForValue);
        i+=*nbBytesForValue;
        continue;
      }
      i+=*nbBytesForValue;

      if(shouldRegisterBePublished(*veRegister)){
        char humanReadableSerial[12];
//        uint32_t serial=*((unsigned int*));
        serialBytesToHumanRedable(serialBytes, humanReadableSerial);
        sprintf(topicStr, "venetworking/%s/%04X", humanReadableSerial, *veRegister);
        publishToMQTT(topicStr, valueStr);
      }
    }
  }
  else{
    sprintf(topicStr, "venetworking/%s/unparsedcleartext", macHexStr);
    publishToMQTT(topicStr, clearTextHexStr);
  }

  free(macHexStr);
  free(serialHexStr);
  free(clearTextHexStr);
}

struct DeviceOverflowRecord* findOverflowDeviceCounterFromSerial(uint8_t* serial) {
  for(uint8_t i=0; i<g_deviceOverflowCounterSize; i++){
    if(memcmp(g_deviceOverflowCounter[i].serial, serial, VE_NETWORKING_SERIAL_BYTE_LEN)==0){
      return g_deviceOverflowCounter+i;
    }
  }
  return NULL;
}

void recordOverflowCounterForSerial(uint8_t* serial, uint8_t* overflowCounter){
  ESP_LOGD(__FUNCTION__, "serial=%02X%02X%02X%02X overflowCounter=%02X%02X",
      serial[0], serial[1], serial[2], serial[3],
      overflowCounter[0], overflowCounter[1]);
  struct DeviceOverflowRecord* counterRecord=findOverflowDeviceCounterFromSerial(serial);
  if(counterRecord==NULL){
    counterRecord=g_deviceOverflowCounter+g_deviceOverflowCounterSize;
    g_deviceOverflowCounterSize++;
  }
  memcpy(counterRecord->serial, serial, VE_NETWORKING_SERIAL_BYTE_LEN);
  memcpy(counterRecord->overflow, overflowCounter, VE_NETWORKING_OVERFLOW_BYTE_LEN);
}
#pragma pack(push)
#pragma pack(1)
struct VeNetworkingMessageHeader {
  uint8_t groupId;
  uint8_t serial[VE_NETWORKING_SERIAL_BYTE_LEN];
  uint8_t sequence[VE_NETWORKING_SEQUENCE_BYTE_LEN];
};
#pragma pack(pop)

void computeCipherIVFromFields(uint8_t* ivBytesDestination, uint8_t opKind, uint8_t* sequence, uint8_t* overflowCounter, uint8_t* serial, const uint8_t* networkId){
  uint8_t* ivBytes=ivBytesDestination;
  ESP_LOGD(__FUNCTION__, "opKind=%d", opKind);
  ESP_LOG_BUFFER_HEX_LEVEL("sequence ", sequence, VE_NETWORKING_SEQUENCE_BYTE_LEN, ESP_LOG_DEBUG);
  ESP_LOG_BUFFER_HEX_LEVEL("overflowCounter ", overflowCounter, VE_NETWORKING_OVERFLOW_BYTE_LEN, ESP_LOG_DEBUG);
  ESP_LOG_BUFFER_HEX_LEVEL("serial ", serial, VE_NETWORKING_SERIAL_BYTE_LEN, ESP_LOG_DEBUG);
  ESP_LOG_BUFFER_HEX_LEVEL("networkId ", networkId, 2, ESP_LOG_DEBUG);
  ivBytes[0]=opKind;
  ivBytes+=1;
  memcpy(ivBytes, sequence, VE_NETWORKING_SEQUENCE_BYTE_LEN);
  ivBytes+=VE_NETWORKING_SEQUENCE_BYTE_LEN;
  memcpy(ivBytes, overflowCounter, VE_NETWORKING_OVERFLOW_BYTE_LEN);
  ivBytes+=VE_NETWORKING_OVERFLOW_BYTE_LEN;
  memcpy(ivBytes, serial, VE_NETWORKING_SERIAL_BYTE_LEN);
  ivBytes+=VE_NETWORKING_SERIAL_BYTE_LEN;
  ivBytes[0]=networkId[0];
  ivBytes[1]=networkId[1];
  ESP_LOG_BUFFER_HEX_LEVEL("resulting ivBytes ", ivBytesDestination, VE_NETWORKING_IV_BYTE_LEN, ESP_LOG_DEBUG);
}

void computeXOROn2ByteArrays(uint8_t* destination, uint8_t* src1, uint8_t* src2, int nbBytes){
    for(uint8_t i=0; i<nbBytes; i++){
        destination[i]=src1[i]^src2[i];
    }
}

uint8_t computeFlags(uint8_t nbBytesForTag, uint8_t fieldSizeInBytesForClearTextLen){
    if(nbBytesForTag<VE_NETWORKING_TAG_BYTE_LEN || nbBytesForTag>16 || nbBytesForTag%2==1){
        fprintf(stderr, "Invalid nbBytesForTag %d\n", nbBytesForTag);
        exit(1);
    }

    if(fieldSizeInBytesForClearTextLen<2 || fieldSizeInBytesForClearTextLen>8){
        fprintf(stderr, "Invalid fieldSizeInBytesForClearTextLen\n");
        exit(1);
    }
    uint8_t mPrime=((nbBytesForTag-2)/2)<<3; //M'
    uint8_t lPrime=(fieldSizeInBytesForClearTextLen-1); //L'
    return mPrime|lPrime;
}

void computeKeystreamForBlock(uint8_t blockNumber, uint8_t* ivBytes, uint8_t* destinationKeyStreamBlock, mbedtls_aes_context* aesCtx){
    struct {
        uint8_t flags;
        uint8_t nonceBytes[13];
        uint16_t counter;
    } keystreamNonceAx;
    keystreamNonceAx.flags=0x1; //Declare we use 2 bytes for the textLen (uint16_t big endian)
    memcpy(keystreamNonceAx.nonceBytes, ivBytes, VE_NETWORKING_IV_BYTE_LEN);
    keystreamNonceAx.counter=blockNumber<<8; //FIXME MSB encoding
//    ESP_LOG_BUFFER_HEX_LEVEL("keystreamNonceAx", &keystreamNonceAx, sizeof(keystreamNonceAx), ESP_LOG_DEBUG);
    mbedtls_internal_aes_encrypt(aesCtx, (uint8_t*) &keystreamNonceAx, destinationKeyStreamBlock);
}

bool aesCCMDecrypt(uint8_t* ivBytes, uint8_t* cipherText, uint8_t cipherTextLen, uint8_t* encryptedCcmTag, uint8_t* clearText){
  struct {
      uint8_t flags;
      uint8_t nonceBytes[VE_NETWORKING_IV_BYTE_LEN];
      uint8_t cipherTextLen[2];
  } authNonceB0;
  memset(&authNonceB0, 0, sizeof(authNonceB0));
  memcpy(authNonceB0.nonceBytes, ivBytes, VE_NETWORKING_IV_BYTE_LEN);
  authNonceB0.flags=computeFlags(VE_NETWORKING_TAG_BYTE_LEN, 2);
  authNonceB0.cipherTextLen[0]=0;
  authNonceB0.cipherTextLen[1]=cipherTextLen; //FIXME encode MSB first
  ESP_LOG_BUFFER_HEX_LEVEL("authNonceB0/CBC IV in", &authNonceB0, 16, ESP_LOG_DEBUG);
  uint8_t macValueT[16]; //AKA X_1
  mbedtls_internal_aes_encrypt(&aesCtx, (const unsigned char*) &authNonceB0, macValueT);
//  ESP_LOG_BUFFER_HEX_LEVEL("macValueT (AKA X_1)/CBC IV out", macValueT, 16, ESP_LOG_DEBUG);
//End computeX1

//Begin compute clearText
  uint8_t clearTextLen=cipherTextLen;
  uint8_t blockCounter=1;
  if(cipherTextLen>0){ //TODO Loop for multiple blocks
      uint8_t keystreamS1[16];
      computeKeystreamForBlock(blockCounter, ivBytes, keystreamS1, &aesCtx);
//      ESP_LOG_BUFFER_HEX_LEVEL("keystreamS1",  keystreamS1, 16, ESP_LOG_DEBUG);

      memset(clearText, 0, 16);
      computeXOROn2ByteArrays(clearText, cipherText, keystreamS1, cipherTextLen);
//      ESP_LOG_BUFFER_HEX_LEVEL("clearText", clearText, clearTextLen, ESP_LOG_DEBUG);

      computeXOROn2ByteArrays(macValueT, macValueT, clearText, 16);
      uint8_t authX2[16];
      mbedtls_internal_aes_encrypt(&aesCtx, macValueT, authX2);
//      ESP_LOG_BUFFER_HEX_LEVEL("authX2", authX2, 16, ESP_LOG_DEBUG);
//End computeClearText
//Begin computemacValue
      memcpy(macValueT, authX2, sizeof(macValueT));
  }
//End computeMacValue

//Begin copmuteTag
  uint8_t keystreamS0[16]; //S_0 is only used to encrypt the mac, yielding the tag
  computeKeystreamForBlock(0, ivBytes, keystreamS0, &aesCtx);
//  ESP_LOG_BUFFER_HEX_LEVEL("keystreamS0",  keystreamS0, 16, ESP_LOG_DEBUG);

  //Encrypt the MAC to yield the tag
  uint8_t computedEncryptedTagU[VE_NETWORKING_TAG_BYTE_LEN];
  computeXOROn2ByteArrays(computedEncryptedTagU, macValueT, keystreamS0, VE_NETWORKING_TAG_BYTE_LEN);
//  ESP_LOG_BUFFER_HEX_LEVEL("computedEncryptedTagU", computedEncryptedTagU, VE_NETWORKING_TAG_BYTE_LEN, ESP_LOG_DEBUG);
//  ESP_LOG_BUFFER_HEX_LEVEL("encryptedCcmTag", encryptedCcmTag, VE_NETWORKING_TAG_BYTE_LEN, ESP_LOG_DEBUG);

  bool isMessageDecryptedOk=memcmp(encryptedCcmTag, computedEncryptedTagU, VE_NETWORKING_TAG_BYTE_LEN)==0;
  if(isMessageDecryptedOk==false){
    ESP_LOGE(__FUNCTION__, "Message failed to decrypt correctly");
//    ESP_LOG_BUFFER_HEX_LEVEL("keyBytes ", keyBytes, 16, ESP_LOG_ERROR);
    ESP_LOG_BUFFER_HEX_LEVEL("ivBytes ", ivBytes, VE_NETWORKING_IV_BYTE_LEN, ESP_LOG_ERROR);
    ESP_LOGE(__FUNCTION__, "cipherTextLen=%d", cipherTextLen);
    ESP_LOG_BUFFER_HEX_LEVEL("cipherText ", cipherText, cipherTextLen, ESP_LOG_ERROR);
    ESP_LOG_BUFFER_HEX_LEVEL("encryptedCcmTag ", encryptedCcmTag, VE_NETWORKING_TAG_BYTE_LEN, ESP_LOG_ERROR);
    ESP_LOG_BUFFER_HEX_LEVEL("clearText (should not be shown) ", clearText, cipherTextLen, ESP_LOG_ERROR);
  }
  return isMessageDecryptedOk;
}

extern void parseCleartextVeNetworkingBytes(uint8_t* macAddress, uint8_t* serialBytes, uint8_t* clearText, uint8_t clearTextLen);

void onVeNetworkingMsgReceived(uint8_t* macAddress, uint8_t* manufacturer_data, uint8_t manufacturer_data_len){
  ESP_LOGD(__FUNCTION__, "Begin manufacturer_data_len=%d", manufacturer_data_len);
  struct VeNetworkingMessageHeader* networkingMsg = (struct VeNetworkingMessageHeader*) (manufacturer_data+3);
  struct DeviceOverflowRecord* deviceRecord=findOverflowDeviceCounterFromSerial(networkingMsg->serial);
  if(deviceRecord==NULL) {
    ESP_LOGW(__FUNCTION__, "No overflow record exist yet for serial %02X%02X. Ignoring this advertisement until we get an overflow record", networkingMsg->serial[0], networkingMsg->serial[1]);
    return;
  }

  uint8_t ivBytes[VE_NETWORKING_IV_BYTE_LEN];
  computeCipherIVFromFields(ivBytes, 0x02, networkingMsg->sequence, deviceRecord->overflow, networkingMsg->serial, g_networkId);
  uint8_t* cipherText=(uint8_t*) (networkingMsg+1);
  uint8_t* ccmTag=manufacturer_data+manufacturer_data_len-VE_NETWORKING_TAG_BYTE_LEN;
  uint8_t cipherTextLen=ccmTag-cipherText;
  uint8_t* clearText=(uint8_t*)malloc(16); //FIXME
  if(aesCCMDecrypt(ivBytes, cipherText, cipherTextLen, ccmTag, clearText)==false){
    ESP_LOGE(__FUNCTION__, "Decryption of networkingMessage failed");
    return;
  }
  parseCleartextVeNetworkingBytes(macAddress, networkingMsg->serial, clearText, cipherTextLen);
  free(clearText);
}

bool isGroupIdPartOfOurNetwork(uint8_t groupId){
  return groupId==g_networkId[0];
}

void onOverflowCounterReceived(uint8_t* macAddress, uint8_t* manufacturer_data, uint8_t manufacturer_data_len){
//  ESP_LOGD(__FUNCTION__, "Begin manufacturer_data_len=%d", manufacturer_data_len);
  struct VeNetworkingMessageHeader* overflowMsg = (struct VeNetworkingMessageHeader*) (manufacturer_data+3);
  if(isGroupIdPartOfOurNetwork(overflowMsg->groupId)==false){
    ESP_LOGW(__FUNCTION__, "Ignoring broadcast from foreign group %02X", overflowMsg->groupId);
    return;
  }
  uint8_t* overflowCounter=(uint8_t*) (overflowMsg+1);
  uint8_t* encryptedCcmTag=overflowCounter+VE_NETWORKING_OVERFLOW_BYTE_LEN;

  uint8_t ivBytes[VE_NETWORKING_IV_BYTE_LEN];
  computeCipherIVFromFields(ivBytes, 0x01, overflowMsg->sequence, overflowCounter, overflowMsg->serial, g_networkId);
  if(aesCCMDecrypt(ivBytes, NULL, 0, encryptedCcmTag, NULL)){
    recordOverflowCounterForSerial(overflowMsg->serial, overflowCounter);
  }
  else{
    ESP_LOGE(__FUNCTION__, "Decryption of overflow counter failed, this could be caused by out of network MPPTs with the same groupid as our network");
  }
}

void processVEAdvertisement(uint8_t* macAddress, uint8_t* manufacturer_data, uint8_t manufacturer_data_len) {
  uint8_t opKind=manufacturer_data[2];
  if(opKind==0x01) {
    onOverflowCounterReceived(macAddress, manufacturer_data, manufacturer_data_len);
  }
  else if(opKind==0x02) {
    onVeNetworkingMsgReceived(macAddress, manufacturer_data, manufacturer_data_len);
  }
  else if(opKind==0x10) {
    //Connection status update Not so interesting
  }
  else {
    ESP_LOGW(__FUNCTION__, "Unknown opKind=%d", opKind);
  }
}
