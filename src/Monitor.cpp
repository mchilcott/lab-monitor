#include <ESP8266WiFi.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <FS.h>
#include <ArduinoJson.h>

#include "AnalogMonitor.h"
#include <memory>

// Put authorisation in separate file to keep out of repo
#include "auth.h"
//const char* update_username = "...";
//const char* update_password = "...";

/*
 * Some stuff for firmware updating
 */
const char* host = wifi_station_get_hostname();
const char* update_path = "/firmware";

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

// For the threading and data collection

char MQTTServer[64] = "hugin.px.otago.ac.nz";
char MQTTPort[8] = "1883";
WiFiClient espClient; // This is required by DCThread
ThreadManager tm;
MQTTClient node_client;

///////////////////////////////////////////////////////////
// BEGIN NODE CONFIGURATION
///////////////////////////////////////////////////////////
const char * node_name = "signal_monitor";

std::vector<DCThread *> collectors = {

  /////////////////////
  // Examples
  // - Monitor the analog voltage
 new AnalogMonitor (tm, "sensor/default/analog"),
  // - Temperature sensor
  // new DS18B20Monitor(tm, "sensor/cooling/tank/temperature"), 
  // - Flow rate sensor with arduino (period = 1000 ms)
  // new I2CWordMonitor(tm, "sensor/cooling/water_in/flowrate", 1000, 1.0/4.1, 0, "L/min")
  // - Pressure sensor connected to analog input
  // new AnalogMonitor (tm, "sensor/cooling/water_in/pressure", 1000, (1200.0/2.62), -155, "kPa"),
  // Atmospheric sensor
  // new DHTTemperatureMonitor(tm, "sensor/lab/atmosphere")
  // Atmospheric sensor

  /////////////////////
  // Actual Device Settings
  // - Dev Lab Monitor settings
  // new DS18B20Monitor(tm, "sensor/cooling/tank/temperature", 1000),
  // new DHTTemperatureMonitor(tm, "sensor/lab_dev/atmosphere", 30000)
  // - Main Lab Water inlet monitoring
  // new DHTTemperatureMonitor(tm, "sensor/lab/atmosphere", 30000),
  // new I2CWordMonitor(tm, "sensor/cooling/water_in/flowrate", 1000, 1.0/4.1, 0, "L/min"),
  // new AnalogMonitor (tm, "sensor/cooling/water_in/pressure", 1000, (1200.0/2.62), -155, "kPa")
  // - Outside Atmospheric sensor
  // new DHTTemperatureMonitor(tm, "sensor/outside/atmosphere", 30000)
  // - Helmholtz Coil Monitor
  // new DS18B20Monitor(tm, "sensor/cooling/coil/temperature", 1000)
};

///////////////////////////////////////////////////////////
// END NODE CONFIGURATION
///////////////////////////////////////////////////////////

// Saving config data: Should we do it now?
bool shouldSaveConfig = false;

// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// Status info update rate (seconds)
const int update_interval = 30;
unsigned long last_update = 0;
// Status info update
void publish_status ()
{
  String topic = "status/";
  topic += ESP.getChipId();
  topic += "/";
  node_client.publish(topic + "$name", node_name);
  node_client.publish(topic + "$localip", WiFi.localIP().toString());
  node_client.publish(topic + "$mac", WiFi.macAddress());
  node_client.publish(topic + "$stats/rssi", String() + WiFi.RSSI());
  node_client.publish(topic + "$stats/interval", String() + update_interval);

  // Build a node string
  String node_str = "";

  for (auto it = collectors.begin(); it != collectors.end(); ++it)
  {
    node_str += (*it)->topic();
    node_str += ',';
  }

  node_client.publish(topic + "$nodes", node_str + "[]");
  node_client.publish(topic + "$fw/name", "ArduinoMonitor");
  node_client.publish(topic + "$fw/version", __DATE__ " " __TIME__);
  node_client.publish(topic + "$implementation", "NodeMCU");
 
}


void setup() {
  
  Serial.begin(115200);
  Serial.println("Booting");
  
  Serial.print("Reset Reason: ");
  Serial.println(ESP.getResetReason());

  ///////////////////////////////////////////////////////////
  // Read configuration from Flash
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      // File exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strncpy(MQTTServer, json["mqtt_server"], sizeof(MQTTServer));
          strncpy(MQTTPort, json["mqtt_port"], sizeof(MQTTPort));
          // Make sure we have null terminated strings (This is thoroughly paranoid)
          MQTTServer[sizeof(MQTTServer)-1] = MQTTPort[sizeof(MQTTPort)-1] = '\0';

        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  ///////////////////////////////////////////////////////////
  // Use some magic to connect to another network if things go wrong.
  // See lib/WiFiManager/README.md (also leave room for a null byte at end of string)
  WiFiManagerParameter custom_mqtt_server("server", "MQTT server", MQTTServer, sizeof(MQTTServer)-1);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT port", MQTTPort, sizeof(MQTTPort)-1);

  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.autoConnect();
  WiFi.setAutoReconnect(true);

  ///////////////////////////////////////////////////////////
  // Deal with parameters if they are updated
  strncpy(MQTTServer, custom_mqtt_server.getValue(), sizeof(MQTTServer));
  strncpy(MQTTPort, custom_mqtt_port.getValue(), sizeof(MQTTPort));
  // Make sure we have null terminated strings (This is thoroughly paranoid)
  MQTTServer[sizeof(MQTTServer)-1] = MQTTPort[sizeof(MQTTPort)-1] = '\0';

  if (shouldSaveConfig) {
      Serial.println("saving config");
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      json["mqtt_server"] = MQTTServer;
      json["mqtt_port"] = MQTTPort;
      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }

      json.printTo(Serial);
      json.printTo(configFile);
      configFile.close();
      //end save
  }
  
  ///////////////////////////////////////////////////////////
  // Setup Web-based firmware updates and info
  MDNS.begin(host);
  
  // TODO: Add some info about available measurements
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.on("/", []() {
    String msg = "This Node is alive\n\n";
    msg += "Services: ";
    for (auto it = collectors.begin(); it != collectors.end(); ++it)
      {
        msg += (*it)->topic();
        msg += ',';
      }
    httpServer.send(200, "text/plain", msg);
  });
  httpServer.begin();
  
  MDNS.addService("http", "tcp", 80);

  ///////////////////////////////////////////////////////////
  // Print some useful info
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.println(host);

  ///////////////////////////////////////////////////////////
  // Notify the MQTT server of our existance
 
  node_client.begin(MQTTServer, atoi(MQTTPort), espClient);
  // Number of connection attempts:
  int n_attempts = 10;
  bool connected = false;
  String label =" Node:";
  label += ESP.getChipId();
  label += ":";
  label += node_name;

  String topic = "status/";
  topic += ESP.getChipId();
  topic += "/";
  node_client.setWill((topic + "$status").c_str(), "lost");

  for (int i = 0; i < n_attempts; ++i)
  {
    if (node_client.connect(label.c_str()))
    {
      connected = true;
      break;
    }
    Serial.print(MQTTServer);
    Serial.print(":");
    Serial.print(MQTTPort);
    Serial.println(" - Failed MQTT connection attempt.");
    delay(1000);
  }

  if (!connected)
  {
    // Still couldn't connect - start up the config portal to attempt to get things going
    // This very hacky, because the sensible way didn't seem to work
    Serial.println("Unable to connect to MQTT");
    wifiManager.resetSettings();
    delay(500);
    ESP.reset();
    //WiFiManagerParameter custom_text("<h2>Unable to connect to MQTT</h2>");
    //wifiManager.addParameter(&custom_text);
    //wifiManager.startConfigPortal();
  }

  // Send some data to the server. This is very badly based on the homie convention

  node_client.publish(topic + "$status", "init");
  publish_status();
  last_update = millis();
  ///////////////////////////////////////////////////////////
  // Start up the monitoring threads
  //DCThread dc(tm, "Analog Data", MQTTServer, atoi(MQTTPort));

  for (auto it = collectors.begin(); it != collectors.end(); ++it)
  {
    (*it)->beginMQTT(MQTTServer, atoi(MQTTPort));
  }
  node_client.publish(topic + "$status", "ready");
}

void loop() {
  httpServer.handleClient();
  node_client.loop();
  tm.handle();
  if (millis() - last_update > update_interval * 1000)
  {
    last_update = millis();
    publish_status();
  }
}
