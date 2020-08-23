#include "MonitorThreads.h"

//////////////////////////////////////////
// Node to monitor the amplitude / width / position of a signal
// on an E4401A spectrum analyser
//
// This board is an ESP32 module, but uses pin 32 for TX, as pin 10 (like on the PCB) is broken.


const char * node_name = "beatnote_monitor";

std::vector<DCThread *> collectors = {

  new SerialMonitor(
    Serial1,
    // Init function
    [](AuxSerial &conn){
        conn.begin(
            115200,     // baud rate
            SERIAL_8N1, // 8 bits, No parity bit, 1 stop bits
            13,         // RX pin
            32          // TX pin
        );

        conn.write("++addr 18\n");
        conn.write("++auto 1\n");
        write_log("Starting Init");
        conn.write("CALC:BAND:STATE ON;\n");
        conn.write("CALC:BAND:NDB -20;\n");
        conn.write("INIT:CONT OFF;\n");

    },
    // Series of requests
    {
        [](AuxSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
            conn.write("INIT:IMM;\n");  // Run a sweep
            conn.write("CALC:MARK:MAX;\n"); // Jump to the maximum
            conn.write("CALC:BAND:RES?;\n");
            mon.waitFor('\n');
        },
        [](AuxSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
            String value = mon.read();
            double num_value (0);
            sscanf(value.c_str(), "%lf\r", &num_value);
            
            mqtt.publish(
                String(mon.topic()) + "/bandwidth",
                String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"Hz\"}"
            );

            conn.write("CALC:MARK:X?;\n");
            mon.waitFor('\n');
        },   
        [](AuxSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
            String value = mon.read();
            double num_value (0);
            sscanf(value.c_str(), "%lf\r", &num_value);
            
            mqtt.publish(
                String(mon.topic()) + "/centre",
                String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"Hz\"}"
            );

            conn.write("CALC:MARK:Y?;\n");
            mon.waitFor('\n');
       },
       [](AuxSerial &conn, MQTTClient &mqtt, SerialMonitor &mon){
            String value = mon.read();
            double num_value (0);
            sscanf(value.c_str(), "%lf\r", &num_value);
            
            mqtt.publish(
                String(mon.topic()) + "/amplitude",
                String("{\"mean\": ") + String(num_value, 10) + ", \"units\": \"dBm\"}"
            );

       }
    }, "sensor/beatnote", 10000
  )

};

///////////////////////////////////////////////////////////
// END NODE CONFIGURATION
///////////////////////////////////////////////////////////
