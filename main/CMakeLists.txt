# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES bt mdns mqtt)
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.cpp" bleNetwork.cpp veNetworkingBleParser.cpp veAddress.cpp tcpNetwork.cpp mqttNetwork.cpp stringUtils.cpp)
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
