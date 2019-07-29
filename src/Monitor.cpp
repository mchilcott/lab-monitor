#include <ESP8266WiFi.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <FS.h>
#include <ArduinoJson.h>

#include "NodeThreads.h"
#include "MonitorThreads.h"

#include <memory>

// Put authorisation in separate file to keep out of repo
#include "auth.h"
// Should be in auth.h:
//const char* update_username = "...";
//const char* update_password = "...";
//const char* mqtt_username = "..."; - nullptr for no user
//const char* mqtt_password = "..."; - nullptr for no pass

// Include configuration
 #include "sensor_config.h"

// For the threading and data collection
char MQTTServer[64] = "hugin.px.otago.ac.nz";
char MQTTPort[8] = "1883";
ThreadManager tm;
std::map<MQTTClient *, std::function<void(String &, String &)>> MQTTCallbackBroker::msCallbacks;
DSM501A_Monitor * DSM501A_Monitor::myself;

HomieThread homie (node_name, collectors);

// Saving config data: Should we do it now?
bool shouldSaveConfig = false;

// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


/*
 * Some stuff for firmware updating
 */
const char* host = wifi_station_get_hostname();
const char* update_path = "/firmware";

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

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
 
  homie.beginMQTT(MQTTServer, atoi(MQTTPort), mqtt_username, mqtt_password);
  // Number of connection attempts:
  int n_attempts = 10;
  bool connected = false;

  for (int i = 0; i < n_attempts; ++i)
  {
    if (homie.try_connect())
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

  homie.publish_status("init");
  homie.poll();

  tm.add(&homie);

  ///////////////////////////////////////////////////////////
  // Start up the monitoring threads, and add them to the manager
  for (auto it = collectors.begin(); it != collectors.end(); ++it)
  {
    tm.add(*it);
    (*it)->beginMQTT(MQTTServer, atoi(MQTTPort), mqtt_username, mqtt_password);
  }
  homie.publish_status("ready");
}

void loop() {
  httpServer.handleClient();
  tm.handle();
}
