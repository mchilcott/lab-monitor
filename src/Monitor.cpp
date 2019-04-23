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

// For the threading and data collection

char MQTTServer[64] = "hugin.px.otago.ac.nz";
char MQTTPort[8] = "1883";
ThreadManager tm;
std::map<MQTTClient *, std::function<void(String &, String &)>> CallbackBroker::msCallbacks;
DSM501A_Monitor * DSM501A_Monitor::myself;

///////////////////////////////////////////////////////////
// BEGIN NODE CONFIGURATION
///////////////////////////////////////////////////////////
const char * node_name = "default_monitor";

std::vector<DCThread *> collectors = {

  /////////////////////
  // Examples
  // - Monitor the analog voltage
  //new AnalogMonitor (tm, "sensor/default/analog"),
  // - Temperature sensor
  // new DS18B20Monitor(tm, "sensor/cooling/tank/temperature"), 
  // - Flow rate sensor with arduino (period = 1000 ms)
  // new I2CWordMonitor(tm, "sensor/cooling/water_in/flowrate", 1000, 1.0/4.1, 0, "L/min")
  // - Pressure sensor connected to analog input
  // new AnalogMonitor (tm, "sensor/cooling/water_in/pressure", 1000, (1200.0/2.62), -155, "kPa"),
  // - Atmospheric sensor
  // new DHTTemperatureMonitor(tm, "sensor/lab/atmosphere")
  // - Many Temperature Sensors
  // new DS18B20MultiMonitor(tm,
  //     {
  //       std::make_pair(0, "sensor/cooling/mosfet/main0"),
  //       std::make_pair(1, "sensor/cooling/mosfet/main1"),
  //       std::make_pair(2, "sensor/cooling/mosfet/main2"),
  //       std::make_pair(3, "sensor/cooling/mosfet/main3"),
  //       std::make_pair(4, "sensor/cooling/mosfet/main4"),
  //       std::make_pair(5, "sensor/cooling/mosfet/main5")
  //     }
  //   )

  //new DSM501A_Monitor(tm)
  
  // Standard 3V3TTL type signal on NodeMCU D0
  // new DigitalOutput(tm, "control/default/digital"),

  //Driving a 5V relay board (active low, with open drain) from NodeMCU pin D1 - D4
  new DigitalOutput(tm, "control/default/digital1", HIGH, false, true, 2),
  new DigitalOutput(tm, "control/default/digital2", HIGH, false, true, 0),
  new DigitalOutput(tm, "control/default/digital3", HIGH, false, true, 4),
  new DigitalOutput(tm, "control/default/digital4", HIGH, false, true, 5)


  // Multiple analog inputs with a Mux
  // new AnalogMuxMonitor(tm,
  //   {
  //     std::make_pair(4, "sensor/big_laser/pump_diode/temperature_set"),
  //     std::make_pair(5, "sensor/big_laser/pump_diode/temperature"),
  //     std::make_pair(6, "sensor/big_laser/pump_diode/current"),
  //     std::make_pair(7, "sensor/big_laser/power"),
  //     std::make_pair(1, "sensor/big_laser/seed/power")
  //   },
  //   {0.947, 0.947, 0.947 * 5.065, 0.947 * 13.87, 2.24 * 10.3},
  //   {0.0, 0.0, -0.026, 0.321, -0.044},
  //   {"V", "V", "A", "W", "mW"}
  // )

  /////////////////////
  // Actual Device Settings (NB: Don't forget to change node_name)
  // - Dev Lab Monitor settings
  // new DS18B20Monitor(tm, "sensor/cooling/tank/temperature", 1000),
  // new DHTTemperatureMonitor(tm, "sensor/lab_dev/atmosphere", 30000),
  // new AnalogMonitor (tm, "sensor/cooling/tank/base_pressure", 1000, (1200.0/2.62), -155, "kPa")
  // - Main Lab Water inlet monitoring
  // new DHTTemperatureMonitor(tm, "sensor/lab/atmosphere", 30000),
  // new I2CWordMonitor(tm, "sensor/cooling/water_in/flowrate", 1000, 1.0/4.1, 0, "L/min"),
  // new AnalogMonitor (tm, "sensor/cooling/water_in/pressure", 1000, (1200.0/2.62), -155, "kPa")
  // - Main Lab Water outlet monitoring
  // new I2CWordMonitor(tm, "sensor/cooling/water_out/flowrate", 1000, 1.0/4.1, 0, "L/min"),
  // new AnalogMonitor (tm, "sensor/cooling/water_out/pressure", 1000, (1200.0/2.62), -169.7, "kPa")
  // - Outside Atmospheric sensor
  // new DHTTemperatureMonitor(tm, "sensor/outside/atmosphere", 30000)
  // - Helmholtz Coil Monitor
  // new DS18B20Monitor(tm, "sensor/cooling/coil/temperature", 1000)
  // - Monitor for Dipole Trap Laser
  // new AnalogMuxMonitor(tm,
  //   {
  //     std::make_pair(4, "sensor/big_laser/pump_diode/temperature_set"),
  //     std::make_pair(5, "sensor/big_laser/pump_diode/temperature"),
  //     std::make_pair(6, "sensor/big_laser/pump_diode/current"),
  //     std::make_pair(7, "sensor/big_laser/power"),
  //     std::make_pair(1, "sensor/big_laser/seed/power")
  //   },
  //   {0.947, 0.947, 0.947 * 5.065, 0.947 * 13.87, 2.24 * 10.3},
  //   {0.0, 0.0, -0.026, 0.321, -0.044},
  //   {"V", "V", "A", "W", "mW"}
  // )
  // - Pump control Driving a 5V relay board (active low, with open drain) from NodeMCU pin D1
  // new DigitalOutput(tm, "control/water/pump", HIGH, false, true, 5)
};

HomieThread homie (tm, node_name);
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
 
  homie.beginMQTT(MQTTServer, atoi(MQTTPort));
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

  ///////////////////////////////////////////////////////////
  // Start up the monitoring threads

  for (auto it = collectors.begin(); it != collectors.end(); ++it)
  {
    (*it)->beginMQTT(MQTTServer, atoi(MQTTPort));
  }
  homie.publish_status("ready");
}

void loop() {
  httpServer.handleClient();
  tm.handle();
}
