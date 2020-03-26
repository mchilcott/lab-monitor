#pragma once
#if defined(ESP8266)

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>

uint32_t getChipId() {return ESP.getChipId();}
void restart() {ESP.reset();}
float uptime () {return micros64()/1e6;}
const char * hostname() {return wifi_station_get_hostname();}

typedef ESP8266WebServer WebServer;
typedef ESP8266HTTPUpdateServer UpdateServer;

#elif defined(ESP32)

#include <WiFi.h>
#include <Update.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>

#include <HTTPUpdateServer.h>

uint32_t getChipId() {return ESP.getEfuseMac() && 0x00FFFFFF;}

void restart() {ESP.restart();}

float uptime() {return 0.0;}
const char * hostname() {return "NOT IMPLEMENTED";}

typedef HTTPUpdateServer UpdateServer;

#endif
