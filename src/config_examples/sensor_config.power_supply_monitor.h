#include "MonitorThreads.h"

//////////////////////////////////////////
// Configure this monitor node below
//////////////////////////////////////////

const char * node_name = "power_supply_monitor";

std::vector<DCThread *> collectors = {

  new DHTTemperatureMonitor("sensor/power/cabinet", 30000, 0, 22),
  new SerialMonitor(
    // Init function
    [](SoftwareSerial &conn){
      conn.write("++auto 1\n");
      conn.write("++read_tmo_ms 200\n");
    },
    // Series of requests
    {

      // Quad Trap Requests
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        conn.write("++addr 5\r");
        conn.write("MEAS:VOLT?\n");
        mon.waitFor('\n');
      },
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        String value = mon.read();
        double num_value (0);
        sscanf(value.c_str(), "%lf\r", &num_value);
        
        mqtt.publish(
            "sensor/power/quad_supply/voltage",
            String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"V\"}"
          );
        
        conn.write("MEAS:CURR?\n");
        mon.waitFor('\n');
       },
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        String value = mon.read();
        double num_value (0);
        sscanf(value.c_str(), "%lf\r", &num_value);
        
        mqtt.publish(
            "sensor/power/quad_supply/current",
            String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"A\"}"
          );

        // This is very important: it gives control back to the powersupply front panel
        conn.write("++loc\n");

        ////////////////////////////////////
        // IP Supply
        conn.write("++addr 4\r");
        conn.write("MEAS:VOLT?\n");
        mon.waitFor('\n');
      },
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        String value = mon.read();
        double num_value (0);
        sscanf(value.c_str(), "%lf\r", &num_value);
        
        mqtt.publish(
            "sensor/power/ip_supply/voltage",
            String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"A\"}"
          );
        
        conn.write("MEAS:CURR?\n");
        mon.waitFor('\n');
       },
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        String value = mon.read();
        double num_value (0);
        sscanf(value.c_str(), "%lf\r", &num_value);
        
        mqtt.publish(
            "sensor/power/ip_supply/current",
            String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"A\"}"
          );
        // This is very important: it gives control back to the powersupply front panel
        conn.write("++loc\n");

        ////////////////////////////////////
        // Helmholtz Supply
        conn.write("++addr 6\r");
        conn.write("MEAS:VOLT?\n");
        mon.waitFor('\n');
      },
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        String value = mon.read();
        double num_value (0);
        sscanf(value.c_str(), "%lf\r", &num_value);
        
        mqtt.publish(
            "sensor/power/hh_supply/voltage",
            String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"V\"}"
          );
        
        conn.write("MEAS:CURR?\n");
        mon.waitFor('\n');
       },
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        String value = mon.read();
        double num_value (0);
        sscanf(value.c_str(), "%lf\r", &num_value);
        
        mqtt.publish(
            "sensor/power/hh_supply/current",
            String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"A\"}"
          );

        // This is very important: it gives control back to the powersupply front panel
        conn.write("++loc\n");
       }
    }, "sensor/power"
  )

};

///////////////////////////////////////////////////////////
// END NODE CONFIGURATION
///////////////////////////////////////////////////////////
