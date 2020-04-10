
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
AutoConnectConfig PortalConfig;

UpdateServer httpUpdate;
AutoConnectAux updater(update_path, "Update Firmware");

const char * param_file = "/param.json";
const char * MQTT_path = "/mqtt_setting";
const char * MQTTSave_path = "/mqtt_save";


String  serverName;
int     serverPort;
String  userName;
String  userPass;

String loadParams(AutoConnectAux& aux, PageArgument& args) {
  (void)(args);
  SPIFFS.begin();
  File param = SPIFFS.open(param_file, "r");
  if (param)
    {
      aux.loadElement(param);
      param.close();
    }
  else
    {
      write_log("Failed to open param file");
    }
  SPIFFS.end();
  return String("");
}

String saveParams(AutoConnectAux& aux, PageArgument& args) {
  SPIFFS.begin();
  serverName = args.arg("mqttserver");
  serverName.trim();

  String tmp = args.arg("port");
  tmp.trim();
  serverPort = tmp.toInt();

  userName = args.arg("user");
  userName.trim();
  
  userPass = args.arg("pass");
  userPass.trim();

  
  // The entered value is owned by AutoConnectAux of /mqtt_setting.
  // To retrieve the elements of /mqtt_setting, it is necessary to get
  // the AutoConnectAux object of /mqtt_setting.
  File param = SPIFFS.open(param_file, "w");
  Portal.aux(MQTT_path)->saveElement(param, { "mqttserver", "port", "user", "pass"});
  param.close();

  // Echo back saved parameters to AutoConnectAux page.
  AutoConnectText&  echo = aux["parameters"].as<AutoConnectText>();
  echo.value = "Server: " + serverName + "<br>";
  echo.value += "Port: " + String(serverPort) + "<br>";
  echo.value += "Username: " + userName + "<br>";
  echo.value += "Password: " + userPass + "<br>";
  SPIFFS.end();
  return String("");
}

// Load AutoConnectAux JSON from SPIFFS.
bool loadAux(const String auxName) {
  SPIFFS.begin();
  bool  rc = false;
  String  fn = auxName + ".json";
  File fs = SPIFFS.open(fn.c_str(), "r");
  if (fs) {
    rc = Portal.load(fs);
    fs.close();
  }
  else
    write_log("SPIFFS open failed: " + fn);
  SPIFFS.end();
  return rc;
}


void statusFunc() {
  String page = PSTR(
                     "<html>"
                       "<head>"
                         "<title>Monitor Node Status</title>"
                         "<style type=\"text/css\">body {font-family: sans-serif;}</style>"
                       "</head>"
                     "<body>"
                     "<p style=\"text-align: right;\">" AUTOCONNECT_LINK(BAR_32) "</p>"
                     );
  
  page += "<h1>This Node (";
  page += node_name;
  page += " @ ";
  page += hostname();
  page += ") is alive</h1>";
  page += "<h2>Services</h2><ul>";
  for (auto it = collectors.begin(); it != collectors.end(); ++it)
    {
      page += "<li>";
      page += (*it)->topic();
      page += "</li>";
    }

  page += "</ul><h2>Logged Messages</h2><ul>";
    
  for (int i = 0; i < gLogCount && i < LOG_ENTRIES; ++i)
    {
      page += "<li>";
      char ind = (gLogNext + LOG_ENTRIES - i - 1) % LOG_ENTRIES;
      page += gLogBuffer[ind];
      page += "</li>";
    }

  page += String(F("</ul></body></html>"));
  
  httpServer.send(200, "text/html", page);
}

void setup() {

  delay(1000);
  Serial.begin(115200);
  write_log("Booting");
  
  // Get the portal connecting

  PortalConfig.apid = String("Setup-") + hostname();
  PortalConfig.psk  = "monitornode";
  PortalConfig.title ="Node Config";
  Portal.config(PortalConfig);

  
  loadAux(MQTT_path);
  loadAux(MQTTSave_path);
  Portal.on(MQTT_path, loadParams);
  Portal.on(MQTTSave_path, saveParams);
  
  AutoConnectAux* setting = Portal.aux(MQTT_path);
  if (setting) {
    PageArgument  args;
    AutoConnectAux& mqtt_setting = *setting;
    loadParams(mqtt_setting, args);
    
    serverName = mqtt_setting["mqttserver"].as<AutoConnectInput>().value;
    serverName.trim();
    
    String tmp = mqtt_setting["port"].as<AutoConnectInput>().value;
    tmp.trim();
    serverPort = tmp.toInt();
    
    userName = mqtt_setting["user"].as<AutoConnectInput>().value;
    userName.trim();
  
    userPass = mqtt_setting["pass"].as<AutoConnectInput>().value;
    userPass.trim();
  } else {
    write_log("aux. load error");
  }
  
  
  httpServer.on("/", statusFunc);
  httpUpdate.setup(&httpServer, update_path, update_username, update_password);

  
  
  // Custom Pages in the portal
  Portal.join(updater);
  //Portal.join(mqttConfig);
  

  
  if (Portal.begin()) {
    write_log("WiFi connected: " + WiFi.localIP().toString());
  }
  
  ///////////////////////////////////////////////////////////
  // MDNS Broadcasr
  MDNS.begin(host); 
  MDNS.addService("http", "tcp", 80);


  ///////////////////////////////////////////////////////////
  // Notify the MQTT server of our existance
  
  homie.beginMQTT(serverName.c_str(), serverPort, userName.c_str(), userPass.c_str());
  // Number of connection attempts:
  int n_attempts = 4;
  bool connected = false;

  for (int i = 0; i < n_attempts; ++i)
  {
    if (homie.try_connect())
    {
      connected = true;
      break;
    }
    write_log(serverName + ":" + serverPort + " - Failed MQTT connection attempt.");
    delay(3000);
  }

  if (!connected)
  {
    // Still couldn't connect - Don't bother starting up
    write_log("Unable to connect to MQTT - Please check configuration");
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
    (*it)->beginMQTT(serverName.c_str(), serverPort, userName.c_str(), userPass.c_str());
  }
  homie.publish_status("ready");
}

void loop() {
  Portal.handleClient();
  tm.handle();
}
