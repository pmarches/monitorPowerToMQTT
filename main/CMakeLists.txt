# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES nvs_flash mdns mqtt masterbus-esp32 app_update espcoredump)
set(COMPONENT_PRIV_REQUIRES vedirect-hex-parser)

set(COMPONENT_SRCS "main.cpp" 
  multiserial.cpp
  tcpNetwork.cpp 
  mqttNetwork.cpp 
  stringUtils.cpp 
  masterbusNetwork.cpp 
  mqttAugmentation.cpp 
  coreDumpHandler.cpp 
  nvsLogging.cpp)
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
