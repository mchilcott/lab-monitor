#pragma once

#include "NodeThreads.h"


class AnalogMonitor : public DCThread
{
 private:
  const char *mHost;
  double mScale;
  double mOffset;
  unsigned int mPeriod;
  const char * mUnits;
  
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
  AnalogMonitor(
                  ThreadManager & mgr,
                  const char * topic = "sensor/default/analongmon",
                  unsigned int period = 2000,
                  double scale = 1,
                  double offset = 0,
                  const char * units = "V"
                )
    : DCThread(mgr, topic, period), mScale(scale), mOffset(offset), mUnits(units)
  {}

  virtual void poll()
  {
    // Note the magic number to convert from number to volts
    double measurement = analogRead(0) * (3.3/1024.0) * mScale + mOffset;
    
    // Cheap JSON
    String output = "{\"mean\": ";
    output += String(measurement, 3);
    output += ", \"units\": \"";
    output += mUnits;
    output += "\"}";

    // Put it out there
    mClient.publish(topic(), output);
  }
  

};

#include <OneWire.h>
#include <DallasTemperature.h>

class DS18B20Monitor : public DCThread {
  private:
    boolean mInit; // Make sure init() runs on the first loop() call
    unsigned int mPin; // Pin the OneWire signal is connected to

    OneWire mOW;
    DallasTemperature mSensor;
    unsigned int mResolution; // Desired resolution of conversion
    unsigned int mConversionDelay; // in ms
    unsigned long mLastRequestTime;
    bool mConverting;
  public:
    DS18B20Monitor(
                    ThreadManager &mgr,
                    const char * topic = "sensor/default/temperature",
                    unsigned int period = 2000,
                    unsigned int pin = 2, // NodeMCU D4. Remeber: This is the GPIO pin number for the ESP module, not the NodeMCU label
                    unsigned int resolution = 12
                  )
      : DCThread(mgr, topic, period), mInit(false), mPin(pin), mOW(pin), mSensor(&mOW), mResolution(resolution),
      mConversionDelay(750 / (1 << (12 - resolution))), // This conversion formula is from the DallasTemperature WaitForConversion example
      mLastRequestTime(millis()), mConverting(false)
    {} 

    void init () 
    {
      // Prevent silly things.
      if (mPeriod < mConversionDelay)
      {
        mPeriod = mConversionDelay;
      }
      
      mResolution = std::max(mResolution, (unsigned int) 9);
      mResolution = std::min(mResolution, (unsigned int) 12);

      DeviceAddress tempDeviceAddress;
      mSensor.begin();
      mSensor.getAddress(tempDeviceAddress, 0); // Just use first device
      mSensor.setResolution(tempDeviceAddress, mResolution);
      // We want this to be async
      mSensor.setWaitForConversion(false);
    }

    virtual void poll()
    {
      mConverting = true;
      mSensor.requestTemperatures();
      mLastRequestTime = millis();
    }

    virtual void loop()
    { 
      if(!mInit)
      {
        init();
        mInit = true;
      }

      if(mConverting && millis() - mLastRequestTime > mConversionDelay)
      {
        mConverting = false;
        // Done converting
        float temperature = mSensor.getTempCByIndex(0);
        // Cheap JSON
        String output = "{\"mean\": ";
        output += String(temperature, mResolution - 8);
        output += ", \"units\": \"deg C\"}";
        mClient.publish(topic(), output);
      }

      // Manage the polling, connection, etc
      DCThread::loop();
    }
};


#include <Wire.h>

class I2CWordMonitor : public DCThread
{
 private:
  const char *mHost;
  double mScale;
  double mOffset;
  unsigned int mPeriod;
  const char * mUnits;
  unsigned int mAddr;
  unsigned int mI2C_sda;
  unsigned int mI2C_sdc;
  
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
  I2CWordMonitor(
                  ThreadManager & mgr,
                  const char * topic = "sensor/default/I2Cword",
                  unsigned int period = 2000,
                  double scale = 1,
                  double offset = 0,
                  const char * units = "V",
                  unsigned int i2c_addr = 8, //Address of the remote device
                  unsigned int i2c_sda = 4, // NodeMCU D2. Connect to A4 on Arduino nano
                  unsigned int i2c_sdc = 5  // NodeMCU D1. Connect to A5 on Arduino nano
                )
    : DCThread(mgr, topic, period), mScale(scale), mOffset(offset), mUnits(units), mAddr(i2c_addr), mI2C_sda(i2c_sda), mI2C_sdc(i2c_sdc)
  {}

  uint32_t get_data()
  {
    Wire.begin(mI2C_sda, mI2C_sdc);
    Wire.requestFrom(mAddr, 4);

    uint32_t output = 0;

    for(int shift = 0; shift < 32 && Wire.available(); shift += 8)
      {
        // Assumes it comes in little endian
        char c = Wire.read();
        output += ((int) c) << shift;
      }

      return output;
  }

  virtual void poll()
  {
    // Note the magic number to convert from number to volts
    double measurement = (((double) get_data()) / 1000.0) * mScale + mOffset;
    
    // Cheap JSON
    String output = "{\"mean\": ";
    output += String(measurement, 3);
    output += ", \"units\": \"";
    output += mUnits;
    output += "\"}";

    // Put it out there
    mClient.publish(topic(), output);
  }

};


#include <DHT.h>
#include <DHT_U.h>

class DHTTemperatureMonitor : public DCThread {
  private:
    boolean mInit; // Make sure init() runs on the first loop() call
    unsigned int mPin; // Pin the OneWire signal is connected to
    const char * mTopic;

    DHT_Unified mDHT;
    unsigned int mConversionDelay; // in ms
    unsigned long mLastRequestTime;
    bool mConverting;
  public:
    DHTTemperatureMonitor(
                    ThreadManager &mgr,
                    const char * topic = "sensor/default/atmosphere",
                    unsigned int period = 2000,
                    unsigned int pin = 0, // NodeMCU D3
                    unsigned int dht_type = DHT22
                  )
      : DCThread(mgr, topic, period), mInit(false), mPin(pin), mTopic(topic), mDHT(pin, dht_type), mLastRequestTime(millis()),
      mConverting(false)
    {
      sensor_t sensor;
      mDHT.temperature().getSensor(&sensor);
      mConversionDelay = sensor.min_delay / 1000;
    } 

    void init () 
    {
      // Prevent silly things.
      if (mPeriod < mConversionDelay)
      {
        mPeriod = mConversionDelay;
      }
    }

    virtual void poll()
    {
      mConverting = true;
      mLastRequestTime = millis();
    }

    virtual void loop()
    { 
      if(!mInit)
      {
        init();
        mInit = true;
      }

      if(mConverting && millis() - mLastRequestTime > mConversionDelay)
      {
        sensors_event_t event;  
        mDHT.temperature().getEvent(&event);
        mConverting = false;

        if (isnan(event.temperature)) {
            Serial.println("Error reading temperature!");
        } else {
          float temperature = event.temperature;
          // Cheap JSON
          String output = "{\"mean\": ";
          output += String(temperature, 3);
          output += ", \"units\": \"deg C\"}";
          String output_topic = topic();
          output_topic += "/temperature";
          mClient.publish(output_topic, output);
        }

        mDHT.humidity().getEvent(&event);
        if (isnan(event.relative_humidity)) {
            Serial.println("Error reading humidity!");
        } else {
          float humidity = event.temperature;
          // Cheap JSON
          String output = "{\"mean\": ";
          output += String(humidity, 3);
          output += ", \"units\": \"%\"}";
          String output_topic = topic();
          output_topic += "/rel_humidity";
          mClient.publish(output_topic, output);
        }
      }

      // Manage the polling, connection, etc
      DCThread::loop();
    }
};