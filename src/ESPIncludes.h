#pragma once
#if defined(ESP8266)

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>

inline uint32_t getChipId() {return ESP.getChipId();}
inline void restart() {ESP.reset();}
inline float uptime () {return micros64()/1e6;}
inline const char * hostname() {return wifi_station_get_hostname();}

typedef ESP8266WebServer WebServer;
typedef ESP8266HTTPUpdateServer UpdateServer;

#elif defined(ESP32)

#include <WiFi.h>
#include <Update.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>

#include <HTTPUpdateServer.h>

inline uint32_t getChipId() {return ESP.getEfuseMac() && 0x00FFFFFF;}
inline void restart() {ESP.restart();}
inline float uptime() {return 0.0;}
const char * hostname();

typedef HTTPUpdateServer UpdateServer;

#endif
