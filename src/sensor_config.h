#include "MonitorThreads.h"
extern ThreadManager tm;

//////////////////////////////////////////
// Configure this monitor node below
//////////////////////////////////////////

const char * node_name = "HP34401A_GPIB_mon";

std::vector<DCThread *> collectors = {

  /////////////////////
  // Examples
  
  // - Monitor the analog voltage
  // new AnalogMonitor ("sensor/default/analog"),
  
  // - Temperature sensor
  // new DS18B20Monitor("sensor/uv_led/temperature"), 
  
  // - Flow rate sensor with arduino (period = 1000 ms)
  // new I2CWordMonitor("sensor/cooling/water_in/flowrate", 1000, 1.0/4.1, 0, "L/min")
  
  // - Pressure sensor connected to analog input
  // new AnalogMonitor ("sensor/cooling/water_in/pressure", 1000, (1200.0/2.62), -155, "kPa"),
  
  // - Atmospheric sensor
  // new DHTTemperatureMonitor("sensor/lab/atmosphere")
  
  // - Many Temperature Sensors

  // new MCP9600Monitor("sensor/default/temperature"),

  // - Monitor a digital signal
  // new DigitalMonitor("state/digital/power", 2000, 12, false)

  // - Dust particle detection
  // new DSM501A_Monitor(tm)
  
  // Standard 3V3TTL type signal on NodeMCU D0
  // new DigitalOutput("control/default/digital"),

  // Driving a 5V relay board (active low, with open drain) from NodeMCU pin D1 - D4
  // new DigitalOutput("control/default/digital1", HIGH, false, true, 2),
  // new DigitalOutput("control/default/digital2", HIGH, false, true, 0),
  // new DigitalOutput("control/default/digital3", HIGH, false, true, 4),
  // new DigitalOutput("control/default/digital4", HIGH, false, true, 5)

  // Multiple analog inputs with a Mux
  // new AnalogMuxMonitor(
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
  // new DS18B20Monitor("sensor/cooling/tank/temperature", 1000),
  // new DHTTemperatureMonitor("sensor/lab_dev/atmosphere", 30000),
  // new AnalogMonitor ("sensor/cooling/tank/base_pressure", 1000, (1200.0/2.62), -155, "kPa")
  // - Main Lab Water inlet monitoring
  // new DHTTemperatureMonitor("sensor/lab/atmosphere", 30000),
  // new I2CWordMonitor("sensor/cooling/water_in/flowrate", 1000, 3.0/8.2, 0, "L/min"),
  // new AnalogMonitor ("sensor/cooling/water_in/pressure", 1000, (1200.0/3.92), -166, "kPa") // Each sensor may need new cal!?!
  // - Main Lab Water outlet monitoring
  // new I2CWordMonitor("sensor/cooling/water_return/flowrate", 1000, 3.0/8.2, 0, "L/min"),
  // - Outside Atmospheric sensor
  // new DHTTemperatureMonitor("sensor/outside/atmosphere", 30000)
  // - Helmholtz Coil Monitor
  // new DS18B20Monitor("sensor/cooling/coil/temperature", 1000)
  // - Monitor for Dipole Trap Laser
  // new AnalogMuxMonitor(
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
  // new DigitalOutput("control/water/pump", HIGH, false, true, 5)
  // - IP trap monitor
  // new AnalogMonitor ("sensor/power/ip_supply_current", 1000, 440.0/3.3, 0, "A"),
  // - Quad trap monitor
  // new AnalogMonitor ("sensor/power/quad_supply_current", 1000, 440.0/3.3, 0, "A"),
  // new DHTTemperatureMonitor("sensor/power/magnet_cabinet/atmosphere", 30000),
  // - Main Servo MOSFET monitor
  // new DS18B20MultiMonitor(
  //   {
  //     std::make_pair(0, "sensor/power/mosfets/main/0/temperature"),
  //     std::make_pair(1, "sensor/power/mosfets/main/1/temperature"),
  //     std::make_pair(2, "sensor/power/mosfets/main/2/temperature"),
  //     std::make_pair(3, "sensor/power/mosfets/main/3/temperature"),
  //     std::make_pair(4, "sensor/power/mosfets/main/4/temperature"),
  //     std::make_pair(5, "sensor/power/mosfets/main/5/temperature")
  //   }
  // )

  new SerialMonitor(
    // Init function
    [](SoftwareSerial &conn){
      conn.write("++auto 1\n");
      //conn.write("++addr 2\r");
      conn.write("++read_tmo_ms 3000\n");
      Serial.println("Inside INIT Function");
    },
    // Series of requests
    {
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        conn.write("MEAS:VOLT:AC?\n");
        mon.waitFor('\n');
      },
      [](SoftwareSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
        String value = mon.read();
        double num_value (0);
        sscanf(value.c_str(), "%lf\r", &num_value);
        
        mqtt.publish(
            "sensor/agilent_34401A/volts_ac",
            String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"V RMS\"}"
          );
       }
    }, "sensor/agilent_34401A/volts_ac"
  )
};

///////////////////////////////////////////////////////////
// END NODE CONFIGURATION
///////////////////////////////////////////////////////////
