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
    double measurement = analogRead(0) * (3.2/1024.0) * mScale + mOffset;
    
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
      : DCThread(topic, period), mPin(pin), mOW(pin), mSensor(&mOW), mResolution(resolution),
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
      // Manage the polling, connection, etc
      DCThread::loop();

      if(mConverting && millis() - mLastRequestTime > mConversionDelay)
      {
        mConverting = false;
        // Done converting
        float temperature = mSensor.getTempCByIndex(0);
        if (temperature == DEVICE_DISCONNECTED_C) {
            write_log("Error reading temperature!");
            return;
        }
        // Cheap JSON
        String output = "{\"mean\": ";
        output += String(temperature, mResolution - 8);
        output += ", \"units\": \"deg C\"}";
        mClient.publish(topic(), output);
      }

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
      : DCThread(topic, period), mPin(pin), mTopic(topic), mDHT(pin, dht_type), mLastRequestTime(millis()),
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

      // Manage the polling, connection, etc
      DCThread::loop();

      if(mConverting && millis() - mLastRequestTime > mConversionDelay)
      {
        sensors_event_t event;  
        mDHT.temperature().getEvent(&event);
        mConverting = false;

        if (isnan(event.temperature)) {
            write_log("Error reading temperature!");
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
            write_log("Error reading humidity!");
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
      : DCThread(index_to_topic[0].second, period), mPin(pin), mOW(pin), mSensor(&mOW), mResolution(resolution),
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
 
      // Manage the polling, connection, etc
      DCThread::loop();

      if(mConverting && millis() - mLastRequestTime > mConversionDelay)
      {
        mConverting = false;
        // Done converting

        for (auto it = mIndexToTopic.begin(); it != mIndexToTopic.end(); ++it)
        {
          float temperature = mSensor.getTempCByIndex(it->first);
          // Cheap JSON
          if (temperature == DEVICE_DISCONNECTED_C) {
            write_log("Error reading temperature!");
            continue;
          }
          String output = "{\"mean\": ";
          output += String(temperature, mResolution - 8);
          output += ", \"units\": \"deg C\"}";
          mClient.publish(it->second, output);
        }
      }


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
 * This class is under development, and not currently functional. It needs debugging, and adjusting to use the std::function callbacks. This will also let us 
 * fix it so the warning below doesn't apply.
 * 
 * \warning This class is currently a singleton!! Only one instance should be made. Also, the myself pointer must have an instance made in the main cpp file.
 * 
 * \warning Note also the ICACHE_RAM_ATTR attribute on the interrupt service routines. Not having these causes a crash, at least when using CHANGE flags.
 */
class DSM501A_Monitor : public DCThread
{
  // 

  private:
    
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
      myself = this;
      // Attempt to set up pin interrrups.
      pinMode(mPin_1u, INPUT);
      pinMode(mPin_2u5, INPUT);

      attachInterrupt(digitalPinToInterrupt(mPin_1u), myself->rise_1u, CHANGE);
      attachInterrupt(digitalPinToInterrupt(mPin_2u5), this->rise_2u5, CHANGE);

      mLastMeasurementMillis = millis();
      mLastFallMillis_1u = mLastFallMillis_2u5 = millis();
      
    }

    /**
     * To be called when the 1 um particle pin changes
     */
    static void ICACHE_RAM_ATTR rise_1u()
    {
      if(digitalRead(myself->mPin_1u) == HIGH)
      {
        // Rising edge
        int length = millis() - myself->mLastFallMillis_1u;
        myself->mLowMillis_1u += length;
      }
      else
      {
        // Falling Edge
        myself->mLastFallMillis_1u = millis();
      }
    }
    /**
     * To be called when the 2.5 um particle pin rises
     */
    static void ICACHE_RAM_ATTR rise_2u5()
    {
      if(digitalRead(myself->mPin_2u5) == HIGH)
      {
        // Rising edge
        int length =  millis() - myself->mLastFallMillis_2u5;
        myself->mLowMillis_2u5 += length;
      }
      else
      {
        // Falling Edge
        myself->mLastFallMillis_2u5 = millis();
      }
    }

    virtual void poll()
    {
      // Calculate low time ratio
      double ratio_1u = ((double) mLowMillis_1u) / mPeriod * 100;
      double ratio_2u5 = ((double) mLowMillis_2u5) / mPeriod * 100;

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
 * Monitor the HMC5883(L) 3 axis magnetic field sensor.
 */
class HMC5883Monitor : public DCThread {

  private:
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
      : DCThread(topic, period), mI2C_sda(i2c_sda), mI2C_sdc(i2c_sdc), mMag()
    {} 

    virtual void init()
    {
      Wire.begin(mI2C_sda, mI2C_sdc);
      if(!mMag.begin())
        {
          /* There was a problem detecting the HMC5883 ... check your connections */
          write_log("HMC5883 not detected ... Check your wiring!");
        }
    }

    virtual void poll()
    {
        Wire.begin(mI2C_sda, mI2C_sdc);
        
        sensors_event_t event; 
        mMag.getEvent(&event);

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
};


/**
 * Monitor the [QMC5883](http://wiki.epalsite.com/index.php?title=QMC5883L_Electronic_Compass) 3 axis magnetic field sensor. This is a chinese clone of the HMC5833. 
 */
class QMC5883Monitor : public DCThread {

  private:
    unsigned int mI2C_sda;
    unsigned int mI2C_sdc;

    uint8_t QMC_ADDR = (0x1A>>1);
    uint8_t DATA_REGISTER_BEGIN = (0x00);
    
    uint8_t SET_RESET_REGISTER = (0x0B);
    uint8_t RESET_PERIOD = (0x01); //!< Recommended reset register value according to datasheet

    uint8_t CONTROL_REGISTER = (0x09);

    uint8_t CONTROL_MODE_STANDBY =    (0b00 << 0);
    uint8_t CONTROL_MODE_CONTINUOUS = (0b01 << 0);

    uint8_t CONTROL_RATE_10HZ =  (0b00 << 2);
    uint8_t CONTROL_RATE_50HZ =  (0b01 << 2);
    uint8_t CONTROL_RATE_100HZ = (0b10 << 2);
    uint8_t CONTROL_RATE_200HZ = (0b11 << 2);

    uint8_t CONTROL_RANGE_2G = (0b00 << 4);
    uint8_t CONTROL_RANGE_8G = (0b01 << 4);

    uint8_t CONTROL_OVERSAMPLE_512 = (0b00 << 6);
    uint8_t CONTROL_OVERSAMPLE_265 = (0b01 << 6);
    uint8_t CONTROL_OVERSAMPLE_128 = (0b10 << 6);
    uint8_t CONTROL_OVERSAMPLE_64 =  (0b11 << 6);



  public:
    QMC5883Monitor(
                    const char * topic = "sensor/default/mag_field",
                    unsigned int period = 2000,
                    unsigned int i2c_sda = 4, // NodeMCU D2.
                    unsigned int i2c_sdc = 5  // NodeMCU D1.
                  )
      : DCThread(topic, period), mI2C_sda(i2c_sda), mI2C_sdc(i2c_sdc)
    {} 

    virtual void init()
    {
      Wire.begin(mI2C_sda, mI2C_sdc);

      Wire.beginTransmission(QMC_ADDR);
      Wire.write(SET_RESET_REGISTER);
      Wire.write(RESET_PERIOD);
      Wire.endTransmission();

      Wire.beginTransmission(QMC_ADDR);
      Wire.write(CONTROL_REGISTER);
      Wire.write(CONTROL_MODE_CONTINUOUS | CONTROL_RATE_50HZ | CONTROL_RANGE_8G | CONTROL_OVERSAMPLE_512);
      Wire.endTransmission();
    }

    virtual void poll()
    {
        Wire.begin(mI2C_sda, mI2C_sdc);

        int16_t x, y, z; //triple axis data

        Wire.beginTransmission(QMC_ADDR);
        Wire.write(DATA_REGISTER_BEGIN); //start with register 3.
        Wire.endTransmission();

        //Read the data.. 2 bytes for each axis.. 6 total bytes
        Wire.requestFrom(QMC_ADDR, 6);
        
        x = Wire.read(); //LSB x
        x |= Wire.read() << 8; //MSB x
        z = Wire.read(); //LSB z
        z |= Wire.read() << 8; //MSB z
        y = Wire.read(); //LSB y
        y |= Wire.read() << 8; //MSB y
        

        float coeff = 8.0 / 32767;
        float fx = x * coeff;
        float fy = y * coeff;
        float fz = z * coeff;
        // Cheap JSON
        String output = "{\"mean\": ";
        output += String(fx, 5);
        output += ", \"units\": \"G\"}";
        String output_topic = topic();
        output_topic += "/x";
        mClient.publish(output_topic, output);
        
        output = "{\"mean\": ";
        output += String(fy, 5);
        output += ", \"units\": \"G\"}";
        output_topic = topic();
        output_topic += "/y";
        mClient.publish(output_topic, output);
        
        output = "{\"mean\": ";
        output += String(fz, 5);
        output += ", \"units\": \"G\"}";
        output_topic = topic();
        output_topic += "/z";
        mClient.publish(output_topic, output);
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
          write_log("Something wrong with thermocouple!");
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
      : DCThread(topic, period), mTopic(topic), mWire(), mType(type), mI2C_addr(i2c_addr),  mTherm()
    {
        mWire.begin(i2c_sda, i2c_sdc);
    }

    void init()
    {
        if (! mTherm.begin(mI2C_addr, &mWire)) {
            write_log("Sensor not found. Check wiring!");
        }
        mTherm.setADCresolution(MCP9600_ADCRESOLUTION_18);
        mTherm.setThermocoupleType(mType);
        mTherm.setFilterCoefficient(3);
        mTherm.enable(true);
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
};

/**
 * Redeclare attachInterrupt from the ESP8266 framework. The header file for the framework doesn't acknowledge that
 * the implementation uses a std::function rather than a function pointer, so prevents us from using the more powerful std::function
 */
void attachInterrupt(uint8_t pin, std::function<void(void)> callback, int mode);

/**
 * Monitor a digital signal.
 */
class DigitalMonitor : public DCThread 
{
  private:
    unsigned int mPin;
    bool mASCII;

  public:
    DigitalMonitor(
                    const char * topic = "state/digital",
                    unsigned int period = 10000,
                    unsigned int pin = 4,  //!< pin to be monitored
                    bool pullup = false,
                    bool output_ASCII = false //!< Output HIGH/LOW values instead of 1/0
                  )
      : DCThread(topic, period), mPin(pin), mASCII(output_ASCII)
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

      String s_state;

      if(mASCII)
        s_state = ((state == HIGH)? "HIGH" : "LOW");
      else
        s_state = ((state == HIGH)? "1" : "0");
        

      String output = "{\"state\": \"";
      output += s_state;
      output += "\"}";
      mClient.publish(topic(), output);

    }

};



#if defined(ESP8266)
#include <SoftwareSerial.h>



/**
 * Monitor a serial device. This class provides a way of sending and receiving serial commands, making it capable of
 * talking to TTL serial devices, or with the help of adaptors, RS232 and GPIB devices. GPIB requires a bit more effort,
 * and is accomplished with an Arduino, using, for example [AR488](https://github.com/Twilight-Logic/AR488).
 * 
 * Note that as we are using a software serial port, there can be issues with timing when running the port at high speed.
 * For this reason, if using the GPIB software, it works much better if you modify the arduino firmware to talk serial at a
 * lower rate than 115200 baud. (We have success with 19200)
 * 
 * Becasue of the threading model we use, this class is currently a bit convoluted. Apart from the serial data, the constructor takes two
 * std::function based inputs. The vector of request_functions is used because your code *cannot* wait for a serial response from the device.
 * This is especially true if the device takes a noticable time to respond. Rather than trying to read in characters inside a request_function,
 * one should call waitFor() with a character to signify the end of the message from the device, and then end the function. When the SerialMonitor has
 * read in the specified character, the next function in request_func is called, and the string read in via serial is available by calling read(). Note
 * that the delimiter is still part of the string.
 * 
 * If the series of request_funcs has not completed by the time of the next poll(), the sequence is started again. It is likely that you do not want
 * this to happen, so make sure that the period is sufficiently long that the sequence has time to finish. This interruption means that the whole
 * process doesn't hang indefinitely if there is a communication error, or the device is disconnected.
 * 
 * Note that the initFunc gives access to the SoftwareSerial, allowing you to configure your serial device. The request_funcs have access to the
 * SoftwareSerial (to issue measurement commands), the MQTTClient (to submit your measurements), and the SerialMonitor instance (for waitFor(), read(), etc).
 * 
 * An example of using this with the AR488 flashed on an arduino, and connecting the TTL signal of the arduino to the SoftwareSerial pins.
 * \verbatim
   new SerialMonitor(
    // Init function
    [](SoftwareSerial &conn){
      conn.write("++auto 1\n");
      conn.write("++addr 2\r");
      conn.write("++read_tmo_ms 3000\n");
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
\endverbatim
 * Note that the `++` commands configure the AR488, and let us setup the GPIB bus. The `MEAS:VOLT:AC?\n` command is sent via GPIB
 * to an agilent 34401A 6.5 digit multimeter. (Because we haven't specified a range, the device autoranges each time and takes a while).
 * The measurement is then read in, scanned into a double, and formatted into a JSON string for MQTT transmission. One can (and probably should) use
 * the JSON library for this, but this is cheap and effective.
 * 
 * With our system, we have noticed that numbers sent via GPIB are often not formatted in a way that the JSON decoder on Telegraf works. (In 
 * particular, a prefixed `+` on the number)
 */
class SerialMonitor : public DCThread
{
  private:
    std::vector<std::function<void(SoftwareSerial &, MQTTClient &, SerialMonitor &)>> mRequests;
    SoftwareSerial mConnection;
    String mInput;
    char mWaitingFor;
    unsigned int mRequestStep;
    std::function<void(SoftwareSerial &)> mInitFunc;
    unsigned long mBaud;

  public:
    SerialMonitor(
                    std::function<void(SoftwareSerial &)> initFunc,
                    std::vector<std::function<void(SoftwareSerial &, MQTTClient &, SerialMonitor &)>> request_funcs,
                    const char *name, //!< Measurement name (like MQTT topic)
                    unsigned int period = 3000, //<! measurement period in milliseconds
                    unsigned int rx_pin = 14, // D5
                    unsigned int tx_pin = 12, // D6
                    unsigned long baud = 19200
                  )
      : DCThread(name, period), mRequests(request_funcs),
      mConnection(rx_pin, tx_pin, false),
      mInput(), mWaitingFor('\0'), mRequestStep(0),
      mInitFunc(initFunc),
      mBaud(baud)
      {}

  void init(){
        mConnection.begin(mBaud);
        // Blocking transmission for high-speeds.
        // Not sure it actually helps.
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
      mRequests[mRequestStep](mConnection, mClient, *this);
      mRequestStep ++;

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
      // Do polling
      DCThread::loop();
      //Serial.println("Polling");

      // Read from serial
      while(mConnection.available() > 0)
      {  
        
        char c = mConnection.read();
        mInput += String(c);

        if (c == mWaitingFor)
          handleRequest();
      }
     
    }


};

#endif


#include <Adafruit_ADS1015.h>
/**
 * Monitor analog voltages with the ADS1115 ADC.
 * 
 * 
 */
class ADS1115Monitor : public DCThread {
  private:

    Adafruit_ADS1115 mDevice;
    std::vector<const char *> mTopics; //!< Listing between sensor index and MQTT topic of measurement

    unsigned int mSDA;
    unsigned int mSDC;

    adsGain_t mGain;

  public:
    ADS1115Monitor(
                    std::vector<const char *> topics,
                    unsigned int period = 2000,
                    uint8_t address_offset = 0, //!< Offset depending on connection of ADDR pin. 0 = GND, 1 = VCC, 2 = SDA, 3 = SCL
                    unsigned int i2c_sda = 4, // NodeMCU D2.
                    unsigned int i2c_sdc = 5,  // NodeMCU D1.
                    adsGain_t gain = GAIN_TWOTHIRDS

                  )
      : DCThread(topics[0], period), mDevice(ADS1015_ADDRESS + address_offset), mTopics(topics), mSDA(i2c_sda), mSDC(i2c_sdc), mGain(gain)
    {
    } 


    /**
     * Start up sensors. This function prints out some details about the sensors via serial, which may help in identifying sensors.
     */
    void init () 
    {
      Wire.begin(mSDA, mSDC);
      mDevice.setGain(mGain);
    }

    double scale_factor(adsGain_t gain)
    {
      double factor = 1;
      switch(gain)
      {
        case GAIN_TWOTHIRDS:
          factor = 2.0/3.0; break;
        case GAIN_ONE:
          factor = 1.0; break;
        case GAIN_TWO:
          factor = 2.0; break; 
        case GAIN_FOUR:
          factor = 4.0; break; 
        case GAIN_EIGHT:
          factor = 8.0; break; 
        case GAIN_SIXTEEN:
          factor = 16.0; break; 
      }

      factor = (4.096 / 32768) / factor;
      return factor;
    }

    /**
     * Start the sensors converting
     */
    virtual void poll()
    {
      Wire.begin(mSDA, mSDC);
      for(uint8_t i = 0; i < mTopics.size() && i < 4 ; ++i)
      {
        int16_t value = mDevice.readADC_SingleEnded(i);
        double result = value * scale_factor(mGain);

        String output = "{\"mean\": ";
        output += String(result, 8);
        output += ", \"units\": \"V\"}";

        mClient.publish(mTopics[i], output);

      }
    }

};

/**
 * Monitor analog voltages with the ADS1115 ADC.
 * 
 * 
 */
class ADS1115MonitorOverSampled : public DCThread {
  private:

    Adafruit_ADS1115 mDevice;
    std::vector<const char *> mTopics; //!< Listing between sensor index and MQTT topic of measurement

    unsigned int mSDA;
    unsigned int mSDC;

    adsGain_t mGain;
    
    unsigned int mSamples;
    unsigned int mSampleCounter;

    std::vector<double> mMean;
    std::vector<double> mMax;
    std::vector<double> mMin;
    
  public:
    ADS1115MonitorOverSampled(
                    std::vector<const char *> topics,
                    unsigned int period = 2000,
                    uint8_t address_offset = 0, //!< Offset depending on connection of ADDR pin. 0 = GND, 1 = VCC, 2 = SDA, 3 = SCL
                    unsigned int i2c_sda = 4, // NodeMCU D2.
                    unsigned int i2c_sdc = 5,  // NodeMCU D1.
                    adsGain_t gain = GAIN_TWOTHIRDS,
                    unsigned int oversample_rate = 20
                  )
      : DCThread(topics[0], period/oversample_rate), mDevice(ADS1015_ADDRESS + address_offset), mTopics(topics), mSDA(i2c_sda), mSDC(i2c_sdc), mGain(gain),
      mSamples(oversample_rate), mSampleCounter(0), mMean(topics.size(), 0), mMax(topics.size(), NAN), mMin(topics.size(), NAN)
    {
    } 


    /**
     * Start up sensors. This function prints out some details about the sensors via serial, which may help in identifying sensors.
     */
    void init () 
    {
      Wire.begin(mSDA, mSDC);
      mDevice.setGain(mGain);
    }

    double scale_factor(adsGain_t gain)
    {
      double factor = 1;
      switch(gain)
      {
        case GAIN_TWOTHIRDS:
          factor = 2.0/3.0; break;
        case GAIN_ONE:
          factor = 1.0; break;
        case GAIN_TWO:
          factor = 2.0; break; 
        case GAIN_FOUR:
          factor = 4.0; break; 
        case GAIN_EIGHT:
          factor = 8.0; break; 
        case GAIN_SIXTEEN:
          factor = 16.0; break; 
      }

      factor = (4.096 / 32768) / factor;
      return factor;
    }

    /**
     * Start the sensors converting
     */
    virtual void poll()
    {
      mSampleCounter ++;
      Wire.begin(mSDA, mSDC);
      for(uint8_t i = 0; i < mTopics.size() && i < 4 ; ++i)
      {
        int16_t value = mDevice.readADC_SingleEnded(i);
        double result = value * scale_factor(mGain);

        mMean[i] += result / mSamples;

        if (result > mMax[i] || isnan(mMax[i]))
        {
          mMax[i] = result;
        }

        if (result < mMin[i] || isnan(mMin[i]))
        {
          mMin[i] = result;
        }

      }

      if(mSampleCounter == mSamples)
      {
        
        mSampleCounter = 0;

        for(uint8_t i = 0; i < mTopics.size() && i < 4 ; ++i)
        {
          String output = "{\"mean\": ";
          output += String(mMean[i], 8);
          output += ", \"units\": \"V\"}";

          mClient.publish(mTopics[i], output);
          mMean[i]=0;

          output = "{\"min\": ";
          output += String(mMin[i], 8);
          output += ", \"units\": \"V\"}";

          mClient.publish(mTopics[i], output);
          mMin[i]=NAN;

          output = "{\"max\": ";
          output += String(mMax[i], 8);
          output += ", \"units\": \"V\"}";

          mClient.publish(mTopics[i], output);
          mMax[i]=NAN;
        }
      }

    }

};

#include <Adafruit_BME280.h>
/**
 * Monitor a [BME280 Atmospheric Temperature/Humidity/Pressure sensor](https://www.bosch-sensortec.com/bst/products/all_products/bme280).
 * 
 * Note that as well as connecting VCC, GND, then SDI to SDA, and SDC to SCK, you must also connect CSB to VCC (to select I2C), and 
 * connect SDO to either VCC or GND. SDO will change the I2C address used, but the code will search for both addresses. It is simply 
 * important that this is well defined.
 */
class BME280Monitor : public DCThread {
  private:
    const char * mTopic;

    unsigned int mSDA;
    unsigned int mSDC;

    Adafruit_BME280 mDevice;

    bool mConverting;
  public:
    BME280Monitor(
                    const char * topic = "sensor/default/atmosphere",
                    unsigned int period = 10000,
                    unsigned int i2c_sda = 4, // NodeMCU D2.
                    unsigned int i2c_sdc = 5  // NodeMCU D1.
                  )
      : DCThread(topic, period), mSDA(i2c_sda), mSDC(i2c_sdc)

    {
    } 

    void init () 
    {
      Wire.begin(mSDA, mSDC);
      if(!mDevice.begin())
      {
        write_log("Cannot find BME280");
        mPeriod = 0;
      }

      mDevice.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X16,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X16,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
    }

    virtual void poll()
    {
      Wire.begin(mSDA, mSDC);

      float temp = mDevice.readTemperature();
      String output = "{\"mean\": ";
      output += String(temp, 3);
      output += ", \"units\": \"deg C\"}";
      String output_topic = topic();
      output_topic += "/temperature";
      mClient.publish(output_topic, output);

      float pressure = mDevice.readPressure();
      output = "{\"mean\": ";
      output += String(pressure, 3);
      output += ", \"units\": \"Pa\"}";
      output_topic = topic();
      output_topic += "/pressure";
      mClient.publish(output_topic, output);

      float humidity = mDevice.readHumidity();
      output = "{\"mean\": ";
      output += String(humidity, 3);
      output += ", \"units\": \"%\"}";
      output_topic = topic();
      output_topic += "/rel_humidity";
      mClient.publish(output_topic, output);
    }

};

#include <Adafruit_MLX90393.h>
/**
 * Monitor the MLX 90393 axis magnetic field sensor.
 * 
 * When connecting the board with I2C, pull the CS pin high.
 */
class MLX90393Monitor : public DCThread {

  private:
    unsigned int mI2C_sda;
    unsigned int mI2C_sdc;
    Adafruit_MLX90393 mMag;
  public:
    MLX90393Monitor(
                    const char * topic = "sensor/default/mag_field",
                    unsigned int period = 2000,
                    unsigned int i2c_sda = 4, // NodeMCU D2.
                    unsigned int i2c_sdc = 5  // NodeMCU D1.
                  )
      : DCThread(topic, period), mI2C_sda(i2c_sda), mI2C_sdc(i2c_sdc), mMag()
    {} 

    virtual void init()
    {
      Wire.begin(mI2C_sda, mI2C_sdc);
      if(!mMag.begin())
        {
          /* There was a problem detecting the HMC5883 ... check your connections */
          write_log("MLX90393 not detected ... Check your wiring!");
        }

      // Set up some over sampling - See datasheet for explaination
      int osr = 3; // Over sample rate setting (0 to 3) 
      int dig_filt = 7; // Digital filter setting (0 to 7)

      int osr2 = 3; // Temperature oversampling
      int res_xyz = 0; // Resolution flag

      int register_value = (osr2 << 11) | (res_xyz << 5) | (dig_filt << 2) | (osr);
      char tx[] = {MLX90393_REG_WR, (register_value >> 8),  (register_value & 0xFF), MLX90393_CONF3};
      Wire.write((uint8_t *) tx, sizeof(tx));
    }

    virtual void poll()
    {
        Wire.begin(mI2C_sda, mI2C_sdc);

        float x(0) ,y(0) ,z(0);

        if(!mMag.readData(&x, &y, &z)) {
            write_log("Unable to read data from the MLX90393.");
            return;
        }

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
};
