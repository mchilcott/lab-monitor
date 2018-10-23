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

char MQTTServer[64] = "192.168.4.2";
char MQTTPort[8] = "1883";
WiFiClient espClient; // This is required by DCThread
ThreadManager tm;
MQTTClient node_client;

AnalogMonitor am (tm, "AnalogMonitor");

// Saving config data: Should we do it now?
bool shouldSaveConfig = false;

// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("Booting");
  
  Serial.print("Reset Reason: ");
  Serial.println(ESP.getResetReason());

  ///////////////////////////////////////////////////////////
  // Read configuration from Flash
  Serial.println("mounting FS...");

  if (SPIFFS.begin() && false) {
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
  // Setup Web-based firmware updates and info
  MDNS.begin(host);
  
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.on("/", []() {
    String msg = "This Node is alive\n\n";
    msg += "ADC Value: ";
    msg += analogRead(0);
    
    httpServer.send(200, "text/plain", msg);
  });
  httpServer.begin();
  
  MDNS.addService("http", "tcp", 80);

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
  int n_attempts = 5;
  bool connected = false;
  const char * label = (String() + "Node:" + ESP.getChipId()).c_str();
  String topic = "/sensor/status/";
  topic += ESP.getChipId();
  topic += "/";
  node_client.setWill((topic + "$status").c_str(), "lost");
  for (int i = 0; i < n_attempts; ++i)
  {
    if (node_client.connect(label))
    {
      connected = true;
      break;
    }
    delay(500);
  }

  if (!connected)
  {
    // Still couldn't connect - start up the config portal to attemp to get things going
    // This is not yet functional.
    Serial.println("Unable to connect to MQTT");
    delay(500);
    ESP.reset();
    //WiFiManagerParameter custom_text("<h2>Unable to connect to MQTT</h2>");
    //wifiManager.addParameter(&custom_text);
    //wifiManager.startConfigPortal();
  }

  // Send some data to the server. This is very badly based on the homie convention

  node_client.publish(topic + "$status", "init");
  node_client.publish(topic + "$name", "MonitoringStation");
  node_client.publish(topic + "$localip", WiFi.localIP().toString());
  node_client.publish(topic + "$mac", WiFi.macAddress());
  node_client.publish(topic + "$stats/rssi", String() + WiFi.RSSI());
  node_client.publish(topic + "$stats/interval", "-1");
  node_client.publish(topic + "$nodes", "AnalogMonitor[]");
  node_client.publish(topic + "$fw/name", "ArduinoMonitor");
  node_client.publish(topic + "$fw/version", __DATE__ " " __TIME__);
  node_client.publish(topic + "$implementation", "NodeMCU");
 
  ///////////////////////////////////////////////////////////
  // Start up the monitoring threads
  //DCThread dc(tm, "Analog Data", MQTTServer, atoi(MQTTPort));

  am.beginMQTT(MQTTServer, atoi(MQTTPort));

  node_client.publish(topic + "$status", "ready");
}

void loop() {
  httpServer.handleClient();
  node_client.loop();
  tm.handle();
}
