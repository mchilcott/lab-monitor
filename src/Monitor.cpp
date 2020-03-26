
#include "ESPIncludes.h"

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AutoConnect.h>
#include <FS.h>
#include <ArduinoJson.h>

#include "NodeThreads.h"
#include "MonitorThreads.h"
#include "logging.h"

#include <memory>

// Include configuration
#include "auth.h"
#include "sensor_config.h"

// For the threading and data collection
ThreadManager tm;

// Instances of static things
std::map<MQTTClient *, std::function<void(String &, String &)>> MQTTCallbackBroker::msCallbacks;
DSM501A_Monitor * DSM501A_Monitor::myself;

char gLogBuffer[LOG_ENTRIES][LOG_STRING_SIZE];
char gLogCount (0);
char gLogNext (0);


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
const char* host = hostname();

const char* update_path = "/firmware";

WebServer httpServer(80);
AutoConnect Portal(httpServer);

UpdateServer httpUpdate;
AutoConnectAux updater(update_path, "UPDATE");

AutoConnectAux statusPage("/status", "Device Status");

AutoConnectAux mqttConfig;

static const char MQTT_JSON[] PROGMEM = R"(
{
  "name" : "MQTTConfig",
  "uri" : "/mqtt",
  "menu" : true,
  "element" : [
    {
      "name": "hostname",
      "type": "ACInput",
      "label": "Server Hostname",
    },
    {
      "name": "port",
      "type": "ACInput",
      "label": "Port",
      "placeholder": "1883"
    },
    {
      "name": "user",
      "type": "ACInput",
      "label": "Username",
    },
    {
      "name": "pass",
      "type": "ACInput",
      "label": "Password"
    },
    {
      "name": "send",
      "type": "ACSubmit",
      "uri": "/mqtt_save"
    }
  ]
}
)";


String mqttSave(AutoConnectAux& aux, PageArgument& args){
  SPIFFS.begin();
  fs::File param = SPIFFS.open("/params", "w");

  AutoConnectAux* hello = Portal.aux(Portal.where());

  // Save elements as the parameters.
  hello->saveElement(param, { "hostname", "port", "user", "pass" });

  // Close a parameter file.
  param.close();
  SPIFFS.end();

  return String("");
}

void mqttLoad(){
  SPIFFS.begin();
  fs::File param = SPIFFS.open("/params", "w");
  
  // If the file doesn't exist, stop here
  if (!param) return;

  mqttConfig.loadElement(param, { "hostname", "port", "user", "pass" });

  param.close();
  SPIFFS.end();
}

void statusFunc() {
  String msg = "This Node (";
  msg += node_name;
  msg += " @ ";
  msg += hostname();
  msg += ") is alive\n\n";
  msg += "Services: ";
  for (auto it = collectors.begin(); it != collectors.end(); ++it)
    {
      if (it != collectors.begin())
        msg += ", ";
      msg += (*it)->topic();
    }

  msg += "\n\nLogged Messages \n\n";
    
  for (int i = 0; i < gLogCount && i < LOG_ENTRIES; ++i)
    {
      char ind = (gLogNext + LOG_ENTRIES - i - 1) % LOG_ENTRIES;
      msg += gLogBuffer[ind];
      msg += '\n';
    }

  httpServer.send(200, "text/plain", msg);
}

void setup() {

  delay(1000);
  Serial.begin(115200);
  write_log("Booting");
  
  // Get the portal connecting
  httpServer.on("/status", statusFunc);
  httpUpdate.setup(&httpServer, update_path, update_username, update_password);

  mqttConfig.load(MQTT_JSON);
  Portal.on(String("/mqtt_save"), mqttSave);
  mqttLoad();
  
  
  // Custom Pages in the portal
  Portal.join(statusPage);
  Portal.join(mqttConfig);
  Portal.join(updater);
  
  if (Portal.begin()) {
    write_log(("WiFi connected: " + WiFi.localIP().toString()).c_str());
  }
  
  ///////////////////////////////////////////////////////////
  // MDNS Broadcasr
  MDNS.begin(host); 
  MDNS.addService("http", "tcp", 80);


  ///////////////////////////////////////////////////////////
  // Notify the MQTT server of our existance

  MQTTServer =     mqttConfig.getElement<AutoConnectText>("hostname").value.c_str();
  MQTTPort =       mqttConfig.getElement<AutoConnectText>("port").value.c_str();
  mqtt_username =  mqttConfig.getElement<AutoConnectText>("user").value.c_str();
  mqtt_password =  mqttConfig.getElement<AutoConnectText>("pass").value.c_str();
  
  homie.beginMQTT(MQTTServer.c_str(), atoi(MQTTPort.c_str()), mqtt_username.c_str(), mqtt_password.c_str());
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
    // Still couldn't connect - Don't bother starting up
    write_log("Unable to connect to MQTT");
    while(true){
      // Try again once reset
      Portal.handleClient();
    }
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
    (*it)->beginMQTT(MQTTServer.c_str(), atoi(MQTTPort.c_str()), mqtt_username.c_str(), mqtt_password.c_str());
  }
  homie.publish_status("ready");
}

void loop() {
  Portal.handleClient();
  tm.handle();
}
