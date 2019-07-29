#pragma once

#include "NodeThreads.h"

/**
 * Uses the onboard ADC to make measurements.
 * 
 * This class can be reconfigured to take a variety of measurements
 * 
 * Note that this class assumes the range on the ADC to be 3.3 V as per the NodeMCU boards.
 */
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
    
    \f$
    \text{output} = (\text{input} [V]) \times \text{scale} + \text{offset}.
    \f$

    The reported units of output are also configurable.

    \param mgr Thread Manager to run this thread
    \param topic MQTT Topic to publish these measurements to
    \param period Period of measurement (in milliseconds)
    \param scale A scale factor between input voltage and output data
    \param offset The zero offset of the data
    \param units The reported units of this measurement
   */
  AnalogMonitor(
                  const char * topic = "sensor/default/analongmon",
                  unsigned int period = 2000,
                  double scale = 1,
                  double offset = 0,
                  const char * units = "V"
                )
    : DCThread(topic, period), mScale(scale), mOffset(offset), mUnits(units)
  {}

  /**
   * Read the ADC and publish the result
   */
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

/**
 * Monitor measurements from a single [DS18B20](https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf) digital temperature sensor.
 */
class DS18B20Monitor : public DCThread {
  private:
    boolean mInit; //!< Make sure init() runs on the first loop() call
    unsigned int mPin; //!< Pin the OneWire signal is connected to

    OneWire mOW;
    DallasTemperature mSensor;
    unsigned int mResolution; //!< Desired resolution of conversion
    unsigned int mConversionDelay; //!< Calculated conversion delay in ms
    unsigned long mLastRequestTime;
    bool mConverting;
  public:
    DS18B20Monitor(
                    const char * topic = "sensor/default/temperature",
                    unsigned int period = 2000,
                    unsigned int pin = 2, // NodeMCU D4. Remeber: This is the GPIO pin number for the ESP module, not the NodeMCU label
                    unsigned int resolution = 12
                  )
      : DCThread(topic, period), mInit(false), mPin(pin), mOW(pin), mSensor(&mOW), mResolution(resolution),
      mConversionDelay(750 / (1 << (12 - resolution)) + 20), // This conversion formula is from the DallasTemperature WaitForConversion example (With an extra 20 ms for safety)
      mLastRequestTime(millis()), mConverting(false)
    {} 

    /**
     * Initialise the sensor connection, and sanity check parameters
     * 
     * This function is run automatically the first time loop() is called.
     */
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

    /**
     * Starts a measurement. The publication of the data is performed later in the 
     * loop() function, because we must also wait for the conversion to finish.
     */
    virtual void poll()
    {
      mConverting = true;
      mSensor.requestTemperatures();
      mLastRequestTime = millis();
    }

    /**
     * Thread's main loop. Publishes measurement after conversion is finished.
     */
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
/**
 * Reads and publishes a 32 bit word from an I2C device.
 * 
 * This is partially an example, and also used for communication with an Arduino Nano programmed as a frequency counter.
 * 
 * This class could so with some tidying up.
 */
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
    Construct a thread to poll an I2C device, and send data to the given
    mqtt channel. Parameters are provided to make a linear mapping of
    the input data:
    
    \f$
    \text{output} = \left(\frac{\text{input}}{1000}\right) \times \text{scale} + \text{offset}
    \f$

    where input is the 32 bit unsigned integer read from the I2C bus.

    The reported units of output are also configurable.

    \param topic MQTT topic measurement is published to
    \param period Period of measurement (in milliseconds)

    \param scale A scale factor between input voltage and output data
    \param offset The zero offset of the data

    \param i2c_addr Address of the remote I2C device 
    \param i2c_sda Pin used for the SDA (data) line of the I2C bus
    \param i2c_sdc Pin used for the SDC (clock) line of the I2C bus
   */
  I2CWordMonitor(
                  const char * topic = "sensor/default/I2Cword",
                  unsigned int period = 2000,
                  double scale = 1,
                  double offset = 0,
                  const char * units = "V",
                  unsigned int i2c_addr = 8, //Address of the remote device
                  unsigned int i2c_sda = 4, // NodeMCU D2. Connect to A4 on Arduino nano
                  unsigned int i2c_sdc = 5  // NodeMCU D1. Connect to A5 on Arduino nano
                )
    : DCThread(topic, period), mScale(scale), mOffset(offset), mUnits(units), mAddr(i2c_addr), mI2C_sda(i2c_sda), mI2C_sdc(i2c_sdc)
  {}

  /**
   * Reads a 32 bit number from the I2C line.
   */
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

  /**
   * Publish data from the I2C request
   */
  virtual void poll()
  {
    //TODO: Fix magic number
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
/**
 * Monitor a [DHT22 / AM2302 Atmospheric Temperature/Humidity sensor](https://cdn-shop.adafruit.com/datasheets/DHT22.pdf).
 */
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
                    const char * topic = "sensor/default/atmosphere",
                    unsigned int period = 2000,
                    unsigned int pin = 0, // NodeMCU D3
                    unsigned int dht_type = DHT22
                  )
      : DCThread(topic, period), mInit(false), mPin(pin), mTopic(topic), mDHT(pin, dht_type), mLastRequestTime(millis()),
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

/**
 * Monitor measurements from multiple [DS18B20](https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf) digital temperature sensors.
 * 
 * These sensors will be connected to the same data pin. When starting up, this class will print some details about the sensors out the serial port.
 * This may be useful for identifying which sensor is which, but ultimately, one can work out which sensor is which by watching the temperature change
 * caused by holding one of the sensors.
 */
class DS18B20MultiMonitor : public DCThread {
  private:
    boolean mInit; //!< Make sure init() runs on the first loop() call
    unsigned int mPin; //!< Pin the OneWire signal is connected to

    OneWire mOW;
    DallasTemperature mSensor;
    unsigned int mResolution; //!< Desired resolution of conversion
    unsigned int mConversionDelay; //!< in ms
    unsigned long mLastRequestTime;
    bool mConverting;
    std::vector<std::pair<int, const char *> > mIndexToTopic; //!< Listing between sensor index and MQTT topic of measurement

  public:
    DS18B20MultiMonitor(
                    std::vector<std::pair<int, const char *> > index_to_topic,
                    unsigned int period = 2000,
                    unsigned int pin = 2, // NodeMCU D4. Remeber: This is the GPIO pin number for the ESP module, not the NodeMCU label
                    unsigned int resolution = 12
                  )
      : DCThread(index_to_topic[0].second, period), mInit(false), mPin(pin), mOW(pin), mSensor(&mOW), mResolution(resolution),
      mConversionDelay(750 / (1 << (12 - resolution)) + 20), // This conversion formula is from the DallasTemperature WaitForConversion example (with an extra 20 ms for safety)
      mLastRequestTime(millis()), mConverting(false), mIndexToTopic(index_to_topic)
    {} 


    /**
     * Start up sensors. This function prints out some details about the sensors via serial, which may help in identifying sensors.
     */
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

      // List all devices on the bus
      int num_sens = mSensor.getDeviceCount();
      Serial.print("Enumerating Temperature sensors (");
      Serial.print(num_sens);
      Serial.print(") on pin ");
      Serial.println(mPin);

      for (int i = 0; i < num_sens; ++i)
      {
        mSensor.getAddress(tempDeviceAddress, 0);

        for (unsigned int j = 0; j < sizeof(DeviceAddress); ++j)
        {
          Serial.print(tempDeviceAddress[j]);
          Serial.print(" ");
        }
        Serial.print("has index ");
        Serial.println(i);
      }

      for (auto it = mIndexToTopic.begin(); it != mIndexToTopic.end(); ++it)
      {
        // Set the resolution of each device used
        mSensor.getAddress(tempDeviceAddress, it->first);
        mSensor.setResolution(tempDeviceAddress, mResolution);
      }
      // We want this to be async
      mSensor.setWaitForConversion(false);
    }

    /**
     * Start the sensors converting
     */
    virtual void poll()
    {
      mConverting = true;
      mSensor.requestTemperatures();
      mLastRequestTime = millis();
    }

    /**
     * Publish measurements
     */
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

        for (auto it = mIndexToTopic.begin(); it != mIndexToTopic.end(); ++it)
        {
          float temperature = mSensor.getTempCByIndex(it->first);
          // Cheap JSON
          String output = "{\"mean\": ";
          output += String(temperature, mResolution - 8);
          output += ", \"units\": \"deg C\"}";
          mClient.publish(it->second, output);
        }
      }

      // Manage the polling, connection, etc
      DCThread::loop();
    }
};

/**
 * Using a multiplexer (MUX) to monitor multiple inputs from the onboard ADC.
 * 
 * This class can be considered an example, but is considered deprecated!
 */
class AnalogMuxMonitor : public DCThread {
  private:
    std::vector<unsigned int> mPins;
    std::vector<std::pair<int, const char *> > mIndexToTopic;

    std::vector<double> mScale;
    std::vector<double> mOffset;
    std::vector<const char *> mUnits;

  public:
    AnalogMuxMonitor(
                    std::vector<std::pair<int, const char *> > index_to_topic,
                    std::vector<double> scales,
                    std::vector<double> offsets,
                    std::vector<const char *> units,
                    unsigned int period = 2000,
                    std::vector<unsigned int> pins = {16, 5, 4} // GPIO pin numbers for mux pins A,B,c,...
                  )
      : DCThread(index_to_topic[0].second, period), mPins(pins), mIndexToTopic(index_to_topic),
      mScale(scales), mOffset(offsets), mUnits(units)
    {
      for (auto it = mPins.begin(); it != mPins.end(); ++it)
          pinMode(*it, OUTPUT);
    } 


    void set_mux(unsigned int index)
    {
      // This makes the assumption that the pins A,B,C,.. address like the 4051, etc (i.e. the pin numbers are least significant to most significant bits)
      for (auto it = mPins.begin(); it != mPins.end(); ++it)
      {
        digitalWrite(*it, (index % 2)? HIGH : LOW);
        index >>= 1;
      }
    }

    virtual void poll()
    {
      for (unsigned int i = 0; i < mIndexToTopic.size(); ++i)
      {
        auto index = mIndexToTopic[i].first;
        auto topic = mIndexToTopic[i].second;
        auto scale = mScale[i];
        auto offset = mOffset[i];
        auto units = mUnits[i];

        set_mux(index);
        delay(1);


        // Note the magic number to convert from number to volts
        double measurement = analogRead(0) * (3.3/1024.0) * scale + offset;
        
        // Cheap JSON
        String output = "{\"mean\": ";
        output += String(measurement, 3);
        output += ", \"units\": \"";
        output += units;
        output += "\"}";

        mClient.publish(topic, output);
      }
    }
};

/**
 * Monitor a DSM501A particle sensor to detect air quality.
 * 
 * This class is under development, and not currently functional.
 */
class DSM501A_Monitor : public DCThread
{
  // Caution: This class is currently a singleton!! Only one instance should be instantiated.

  private:
    boolean mInit; // Make sure init() runs on the first loop() call
    
    unsigned int mPin_1u;
    unsigned int mPin_2u5;

    unsigned long mLastMeasurementMillis;

    unsigned long mLowMillis_1u;
    unsigned long mLowMillis_2u5;

    unsigned long mLastFallMillis_1u;
    unsigned long mLastFallMillis_2u5;

    static DSM501A_Monitor * myself;

  public:
    DSM501A_Monitor(
                    const char *topic = "sensor/default/atmosphere",
                    unsigned int period = 30000,
                    unsigned int pin_1u = 13,   // Node D7 -> Vout2 (1 um particles)
                    unsigned int pin_2u5 = 15   // Node D8 -> Vout1 (2.5 um particles)
                  )
      : DCThread(topic, period), mPin_1u(pin_1u), mPin_2u5(pin_2u5), mLowMillis_1u(0), mLowMillis_2u5(0)
    {
      pinMode(mPin_1u, INPUT);
      pinMode(mPin_2u5, INPUT);

      attachInterrupt(mPin_1u, this->rise_1u, RISING);
      attachInterrupt(mPin_1u, this->fall_1u, FALLING);

      attachInterrupt(mPin_2u5, this->rise_2u5, RISING);
      attachInterrupt(mPin_2u5, this->fall_2u5, FALLING);

      mLastMeasurementMillis = millis();
      mLastFallMillis_1u = mLastFallMillis_2u5 = millis();
      myself = this;
    }

    static void rise_1u()
    {
      myself->mLowMillis_1u += millis() - myself->mLastFallMillis_1u;
    }

    static void fall_1u()
    {
      myself->mLastFallMillis_1u = millis();
    }

    static void rise_2u5()
    {
      myself->mLowMillis_2u5 += millis() - myself->mLastFallMillis_2u5;
    }

    static void fall_2u5()
    {
      myself->mLastFallMillis_2u5 = millis();
    }

    virtual void poll()
    {
      // Calculate low time ratio
      double ratio_1u = ((double) mLowMillis_1u) / mPeriod;
      double ratio_2u5 = ((double) mLowMillis_2u5) / mPeriod;

      // Reset
      mLowMillis_1u = mLowMillis_2u5 = 0;

      String topic (mTopic);

      String output = "{\"mean\": ";
      output += String(ratio_1u, 3);
      output += ", \"units\": \"";
      output += "%";
      output += "\"}";

      mClient.publish(topic + "/1u_particles", output);

      output = "{\"mean\": ";
      output += String(ratio_2u5, 3);
      output += ", \"units\": \"";
      output += "%";
      output += "\"}";

      mClient.publish(topic + "/2u5_particles", output);
    }

};


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
/**
 * Monitor the HMC5883 3 axis magnetic field sensor.
 */
class HMC5883Monitor : public DCThread {

  private:
    boolean mInit; // Make sure init() runs on the first loop() call
    const char * mTopic;
    unsigned int mI2C_sda;
    unsigned int mI2C_sdc;
    Adafruit_HMC5883_Unified mMag;
  public:
    HMC5883Monitor(
                    const char * topic = "sensor/default/mag_field",
                    unsigned int period = 2000,
                    unsigned int i2c_sda = 4, // NodeMCU D2.
                    unsigned int i2c_sdc = 5  // NodeMCU D1.
                  )
      : DCThread(topic, period), mInit(false), mTopic(topic), mMag(), mI2C_sda(i2c_sda), mI2C_sdc(i2c_sdc)
    {
        Wire.begin(mI2C_sda, mI2C_sdc);
        if(!mMag.begin())
            {
                /* There was a problem detecting the HMC5883 ... check your connections */
                Serial.println("HMC5883 not detected ... Check your wiring!");
            }
    } 

    virtual void poll()
    {
        Serial.println("Mag measure");
        Wire.begin(mI2C_sda, mI2C_sdc);
        sensors_event_t event; 
        mMag.getEvent(&event);
        return;

        float x = event.magnetic.x;
        float y = event.magnetic.y;
        float z = event.magnetic.z;
        // Cheap JSON
        String output = "{\"mean\": ";
        output += String(x, 3);
        output += ", \"units\": \"uT\"}";
        String output_topic = topic();
        output_topic += "/x";
        mClient.publish(output_topic, output);
        
        output = "{\"mean\": ";
        output += String(y, 3);
        output += ", \"units\": \"uT\"}";
        output_topic = topic();
        output_topic += "/y";
        mClient.publish(output_topic, output);
        
        output = "{\"mean\": ";
        output += String(z, 3);
        output += ", \"units\": \"uT\"}";
        output_topic = topic();
        output_topic += "/z";
        mClient.publish(output_topic, output);
    }

    virtual void loop()
    { 

      // Manage the polling, connection, etc
      DCThread::loop();
    }
};

#include <SPI.h>
#include <Adafruit_MAX31855.h>
/**
 * Monitor a Thermocouple with the MAX31855 thermocouple driver.
 * 
 * Only needs the MISO pin (Not MOSI), because data is only read, never written
 */
class MAX31855Monitor : public DCThread {

  private:
    unsigned int mPin; // Pin the OneWire signal is connected to
    const char * mTopic;

    Adafruit_MAX31855 mTherm;
  public:
    MAX31855Monitor(
                    const char * topic = "sensor/default/temp",
                    unsigned int period = 2000,
                    unsigned int pin_clk = 0,  // NodeMCU pin D3
                    unsigned int pin_cs = 4,   // NodeMCU pin D2
                    unsigned int pin_miso = 5  // NodeMCU pin D1
                  )
      : DCThread(topic, period), mTopic(topic),  mTherm(pin_clk, pin_cs, pin_miso)
    {
    }

    virtual void poll()
    {

        double c = mTherm.readCelsius();
        
        if (isnan(c)) {
          Serial.println("Something wrong with thermocouple!");
          return;
        }

        // Cheap JSON
        String output = "{\"mean\": ";
        output += String(c, 3);
        output += ", \"units\": \"deg C\"}";
    
        mClient.publish(topic(), output);

    }

    virtual void loop()
    { 


      // Manage the polling, connection, etc
      DCThread::loop();
    }
};

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_MCP9600.h>

/**
 * Monitor a Thermocouple with the MCP9600 thermocouple driver.
 */
class MCP9600Monitor : public DCThread {

  private:
    bool mInit;
    unsigned int mPin; // Pin the OneWire signal is connected to
    const char * mTopic;

    TwoWire mWire;
    MCP9600_ThemocoupleType mType;
    unsigned int mI2C_addr;
    
    Adafruit_MCP9600 mTherm;
    
  public:
    MCP9600Monitor(
                    const char * topic = "sensor/default/temp",
                    unsigned int period = 2000,
                    MCP9600_ThemocoupleType type = MCP9600_TYPE_K,
                    unsigned int i2c_sda = 4, // NodeMCU D2.
                    unsigned int i2c_sdc = 5,  // NodeMCU D1.
                    unsigned int i2c_addr = MCP9600_I2CADDR_DEFAULT //Address of the remote device
                  )
      : DCThread(topic, period), mTopic(topic), mInit(false), mWire(), mType(type), mI2C_addr(i2c_addr),  mTherm()
    {
        mWire.begin(i2c_sda, i2c_sdc);
    }

    void init()
    {
        if (! mTherm.begin(mI2C_addr, &mWire)) {
            Serial.println("Sensor not found. Check wiring!");
        }
        mTherm.setADCresolution(MCP9600_ADCRESOLUTION_18);
        mTherm.setThermocoupleType(mType);
        mTherm.setFilterCoefficient(3);
        mTherm.enable(true);
        Serial.println("Started Sensor");
    }

    virtual void poll()
    {
        mTherm.readADC(); // This seems to help get everything consistent
        double amb = mTherm.readAmbient();
        double t = mTherm.readThermocouple();
        
        // Cheap JSON
        String output = "{\"mean\": ";
        output += String(t, 3);
        output += ", \"units\": \"deg C\"}";
        String out_topic = topic();
        out_topic += "/probe_temperature";
        mClient.publish(out_topic, output);

        output = "{\"mean\": ";
        output += String(amb, 3);
        output += ", \"units\": \"deg C\"}";
        out_topic = topic();
        out_topic += "/ambient_temperature";
        mClient.publish(out_topic, output);

    }

    virtual void loop()
    { 
      if(!mInit)
        {
          init();
          mInit = true;
        }

      // Manage the polling, connection, etc
      DCThread::loop();
    }
};

/**
 * Redeclare attachInterrupt from the ESP8266 framework. The header file for the framework doesn't acknowledge that
 * the implementation uses a std::function rather than a function pointer, so prevents us from using the more powerful std::function
 */
void attachInterrupt(uint8_t pin, std::function<void(void)> callback, int mode);

/**
 * Monitor a digital signal.
 * 
 * This class has some strange behaviours due to the callbacks not behaving as expected (or perhaps lack of debouncing).
 */
class DigitalMonitor : public DCThread 
{
  private:
    unsigned int mPin;

  public:
    DigitalMonitor(
                    const char * topic = "state/digital",
                    unsigned int period = 10000,
                    unsigned int pin = 4,  //!< pin to be monitored
                    bool pullup = false
                  )
      : DCThread(topic, period), mPin(pin)
    {
      pinMode(mPin, pullup ? INPUT_PULLUP : INPUT);

      attachInterrupt(mPin, [&](){
        this->poll();
      }, RISING);

      attachInterrupt(mPin, [&](){
        this->poll();
      }, FALLING);
      // There is some funny business to sort out with the above, and the need for debouncing
      // (Atleast when testing with a switch)
    }

    //! Produce regular updates, even when not changing.
    void poll()
    {
      int state = digitalRead(mPin);

      String s_state ((state == HIGH)? "HIGH" : "LOW");

      String output = "{\"state\": \"";
      output += s_state;
      output += "\"}";
      mClient.publish(topic(), output);

    }

};

#include <SoftwareSerial.h>
/**
 * Monitor a serial device. This class provides a way of sending and receiving serial commands, making it capable of
 * talking to TTL serial devices, or with the help of adaptors, RS232 and GPIB devices. GPIB requires a bit more effort,
 * and is accomplished with an Arduino, using, for example [AR488](https://github.com/Twilight-Logic/AR488).
 * 
 * Note that as we are using a software serial port, there can be issues with timing when running the port at high speed.
 * For this reason, if using the GPIB software, it works much better if you modify the arduino firmware to talk serial at a
 * lower rate than 115200 baud.
 * 
 * SerialMonitor is a bit of a pain to use and needs more documentation.
 */
class SerialMonitor : public DCThread
{
  private:
    std::vector<std::function<void(SoftwareSerial &, MQTTClient &, SerialMonitor &)>> mRequests;
    std::function<void(SoftwareSerial &)> mInitFunc;
    SoftwareSerial mConnection;
    String mInput;
    char mWaitingFor;
    unsigned int mRequestStep;
    unsigned long mBaud;
    bool mInit;

  public:
    SerialMonitor(
                    std::function<void(SoftwareSerial &)> initFunc,
                    std::vector<std::function<void(SoftwareSerial &, MQTTClient &, SerialMonitor &)>> request_funcs,
                    const char *name,
                    unsigned int period = 3000,
                    unsigned int rx_pin = 14, // D5
                    unsigned int tx_pin = 12, // D6
                    unsigned long baud = 19200
                  )
      : DCThread(name, period), mRequests(request_funcs),
      mConnection(rx_pin, tx_pin, false, 256),
      mInput(), mWaitingFor('\0'), mRequestStep(0),
      mInitFunc(initFunc),
      mBaud(baud), mInit(false)
      {}

  void init(){
        mConnection.begin(mBaud);
        if (mBaud >= 115200)
          mConnection.enableIntTx(false);
        mInitFunc(mConnection);
  }

  void waitFor (char c){
    mWaitingFor = c;
  }

  String read() {
    return mInput;
  }

  void handleRequest()
    {
      Serial.println("HANDLING A REQUEST");
      Serial.flush();
      mRequests[mRequestStep](mConnection, mClient, *this);
      mRequestStep ++;
      Serial.println("DONE REQUEST STEP");

      if (mRequestStep == mRequests.size())
        {
          mRequestStep = 0;
          mWaitingFor = '\0';
        }


      mInput = String();
    }

  virtual void poll()
    {
      mRequestStep = 0;
      handleRequest();
    }
    
  virtual void loop()
    {
      Serial.flush();
      if(!mInit)
        {
          init();
          mInit = true;
        }
      // Do polling

      //Serial.println("Polling");

      // Read from serial
      while(mConnection.available() > 0)
      {  
        
        char c = mConnection.read();
        Serial.print("Got a char: ");
        Serial.println(c);
        mInput += String(c);

        if (c == mWaitingFor)
          handleRequest();
      }
      DCThread::loop();
    }


};
