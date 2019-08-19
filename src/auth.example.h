#pragma once

const char* update_username = "admin"; //!< HTTP Username for firmware updates
const char* update_password = "admin"; //!< HTTP Password for firmware updates

const char* mqtt_username = nullptr; //<! MQTT Username for everything
const char* mqtt_password = nullptr; //<! MQTT Password for everything

char MQTTServer[64] = "dataserver.url"; //<! Default MQTT server address
char MQTTPort[8] = "1883";              //<! Default MQTT server port