#pragma once

#include "NodeThreads.h"


class AnalogMonitor : public DCThread
{
 private:
  const char *mName;
  const char *mHost;
  double mScale;
  double mOffset;
  unsigned int mPeriod;
  const char * mUnits;
  const char * mTopic;
  
 public:
  /**
     Construct a thread to monitor the ADC, and send data to the given
     mqtt channel. Parameters are provided to make a linear mapping of
     the input data:
     
     output = (input [V]) * scale + offset.

     The reported units of output are also configurable.
     
     \param name Name of the measurement as sent via MQTT
     \param period Period of measurement (in milliseconds)

     \param scale A scale factor between input voltage and output data
     \param offset The zero offset of the data
   */
  AnalogMonitor(ThreadManager & mgr,
                const char * name,
                unsigned int period = 2000,
                double scale = 1,
                double offset = 0,
                const char * units = "V",
                const char * topic = "/sensor/data/analog")
    : DCThread(mgr, name, period), mName(name), mScale(scale), mOffset(offset), mUnits(units), mTopic(topic)
  {}

  virtual void poll()
  {
    // Note the magic number to convert from number to volts
    double measurement = analogRead(0) * (3.3/1024.0) * mScale + mOffset;
    
    // Cheap JSON
    String output = "{\"mean\": ";
    output += measurement;
    output += ", \"name\": \"";
    output += mName;
    output += "\", \"units\": \"";
    output += mUnits;
    output += "\"}";

    // Put it out there
    mClient.publish(mTopic, output);
  }
  

};
