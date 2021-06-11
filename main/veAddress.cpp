#include <cstddef>
#include <arpa/inet.h>
#include <stdio.h>
#include <cstring>

char* bytesToHex(uint8_t* bytes, int bytesLen);

const char BASE36[]={"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"};
void serialBytesToHumanRedable(uint8_t* serialBytes, char humanStr[12]){

#if 0
  printf("serial=%s\n", *serialPtr);
#else
//  char* hexStr=bytesToHex(serialBytes, 4);
//  printf("hexStr=%s\n", hexStr);
#endif
//  uint32_t serial=htonl(*serialPtr);
  uint32_t* serialPtr=(uint32_t*) serialBytes;
  uint32_t serial=*serialPtr;

  humanStr[0]=(uint8_t)(((unsigned long long)serial * 0x576c3311l) >> 0x3d) + 0x47;
  unsigned long long uVar3 = (unsigned long long)(serial >> 10) / 0xe6a9ul;
  humanStr[1]= (char)uVar3 + (char)(uVar3 / 26) * -26 + 'A';
  humanStr[2]= 'y'; //TODO Year of manufacture
  humanStr[3]= 'y';
  humanStr[4]= 'w'; //TODO Week of manufacture
  humanStr[5]= 'w';
  humanStr[6]= BASE36[((int)((ulong)serial / 0x19a100) + (int)(((ulong)serial / 0x19a100) / 36) * -36)];
  humanStr[7]= BASE36[((int)((ulong)serial / 0xb640) + (int)(((ulong)serial / 0xb640) / 36) * -36)];
  humanStr[8]= BASE36[((int)((ulong)serial / 0x510) + (int)(((ulong)serial / 0x510) / 36) * -36)];
  int div36 = (int)((ulong)serial / 36);
  humanStr[9]= BASE36[(div36 + (int)(((ulong)serial / 36) / 36) * -36)];
  humanStr[10]= BASE36[serial - div36 * 36];
  humanStr[11]=0;
}

#ifdef SPIKER

void testOne(const char* expected, uint8_t* serialBytes){
  char humanStr[12];
  serialBytesToHumanRedable(serialBytes, humanStr);
  if(strcmp(humanStr, expected)!=0){
    printf("FAILED expected %s got humanStr=%s\n", expected, humanStr);
  }
}

void testVeAddress(){
  struct {
    const char* humanName;
    uint8_t serial[4];
  }
  testdata[] = {
      {"HQyywwMAEI4", {0xec, 0x25, 0x9a, 0x99}},
      {"HQyyww7X83C", {0x38, 0xf6, 0x29, 0x98}},
      {"HQyywwNH7ZQ", {0xb6, 0xa1, 0xb8, 0x99}},
      {"HQyywwJEPUW", {0x68, 0x55, 0x50, 0x99}},
      {NULL, 0}
  };
  testOne(testdata[0].humanName, testdata[0].serial);
  testOne(testdata[1].humanName, testdata[1].serial);
  testOne(testdata[2].humanName, testdata[2].serial);
  testOne(testdata[3].humanName, testdata[3].serial);
}

int main(int argc, char* argv[]){
  testVeAddress();
}
#endif
