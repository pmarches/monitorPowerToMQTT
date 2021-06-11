#ifdef SPIKER
#define ESP_LOGE //ESP_LOGE
#else
#include <esp_log.h>
#endif

#include <memory.h>
#include <cstdio>
#include <stdint.h>
#include <cstdlib>

static int char2int(char input) {
  if(input >= '0' && input <= '9')
    return input - '0';
  if(input >= 'A' && input <= 'F')
    return input - 'A' + 10;
  if(input >= 'a' && input <= 'f')
    return input - 'a' + 10;
  return -1;
}

size_t hexToBytes(const char* hexString, uint8_t* resultByteArr, size_t resultByteArrSize) {
  size_t hexStrLen=strlen(hexString);
  if (hexStrLen % 2 != 0) {
    ESP_LOGE(__FUNCTION__, "Input string is in odd size");
    return 0;
  }

  memset(resultByteArr, 0, resultByteArrSize);
  for (size_t i = 0; i < hexStrLen; i += 2) {
    if (i >= resultByteArrSize * 2)
      return resultByteArrSize;

    int firstChar = char2int(hexString[i]);
    int secondChar = char2int(hexString[i + 1]);
    if (firstChar < 0 || secondChar < 0)
    {
      ESP_LOGE(__FUNCTION__, "Input string has an illegal character");
      resultByteArr[0] = '\0';
      return 0;
    }

    resultByteArr[i / 2] = (firstChar << 4) | secondChar;
  }

  return hexStrLen / 2;
}

char* bytesToHex(uint8_t* bytes, int bytesLen){
  char* hexString=(char*)malloc(bytesLen*2+1);

  int hexStringLen=0;
  for(size_t i=0; i<bytesLen; i++){
    hexStringLen+=sprintf(hexString+hexStringLen, "%02X", bytes[i]);
  }
  return hexString;
}
