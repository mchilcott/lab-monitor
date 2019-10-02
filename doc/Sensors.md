# Sensors

Here are some sensors we have used, and have drivers for. See the comments of the mentioned class for more details, which are found in `src/MonitorThreads.h`. Examples for driving these sensors can be found in the files in `src/config_examples/`.

When wiring things up, it is worth noting that the pin numbers in the code refer to the GPIO pin number of the ESP8266, not the D number on the NodeMCU boards. This is generally specified in the comments for each class.

## Analog Monitoring

### Internal ADC
This can be monitored using the `AnalogMonitor` class

If using a NodeMCU, then this input can be 0-3.2 V, or 0-1 V for a bare ESP8266. The ADC is capable of making 10-bit measurements.

If using the NodeMCU, then the quickest way to extend the input range (at least for positive values)
is to add a resistor R in series with the input, which will give you a max voltage of `(R + 320k)/(100k)`.

### ADS1115 ADC
Using the `ADS1115Monitor` class, or the `ADS1115MonitorOverSampled` class. The latter takes multiple samples for each measurement and reports the min, max and mean value.

This sensor has a 16-bit resolution, and programmable gain. Note that it will not take negative voltages, or voltages greater than the supply voltage (Use a 5V supply, rather than 3.3 V to get more range). The drivers here are not set up to make differential measurements, which allows negative signals.

The ADS1115 requires the user to connect the ADDR pin to ground, VCC, SDA or SCL to set the serial address. See the comments on ADS1115Monitor's addresss_offset and set it accordingly.

## Digital signals
`DigitalMonitor` handles using the GPIO pins for monitoring digital signals. Note that the ESP8266 is a 3.3 V device, but the inputs can handle 5 V (i.e. standard TTL) without need for protection. For larger voltages, add protection to ensure that a current larger than 12 mA cannot flow through the pins, or add voltage limiting circuitly - a diode and resistor works well here.

## Temperature

### DS18B20
This is our favourite. A very conventient way to measure temperature. Can be used with the `DS18B20Monitor` class by connecting the DS18B20 data pin to the DS18B20Monitor's set pin (Default is D4), VCC to 3V3, and GND to GND.

Many of these sensors can be connected to the same pin with the `DS18B20MultiMonitor` class. Working out which sensor is which can be done with the heat from a finger. This class also prints a list of the sensors connected on startup.

### MAX31855
A thermocouple reader. The `MAX31855Monitor` class monitors these.

### MCP9600
Another (higher resolution) thermocouple reader. `MCP9600Monitor` class is used for these, and reports both the temperature at the thermocouple probe (hot junction), and at the MCP9600 (cold junction).

## Atmospheric Sensors

### DHT22 / AM2302
A cheap sensor for measuring the ambient temperature and relative humidity.

 There is a `DHTTemperatureMonitor` class for dealing with these. 

  Connect the sensor pins as:
  ```
  | 1 | +3.3 V        |
  | 2 | D3 on nodeMCU |
  | 3 | NC            |
  | 4 | GND           |
  ```

  for the default configuration.

### BME280
A nice sensor with ambient temperature, relative humidity and air pressure.

These are used as I2C sensors, and need some wiring accordingly: as well as connecting VCC to 3V3, GND to GND, then SDI to SDA (NodeMCU D2 be default), and SDC to SCK (NodeMCU D1 be default), you must also connect CSB to VCC (to select I2C), and connect SDO to either VCC or GND. SDO will change the I2C address used, but the code will search for both addresses. It is simply important that this is well defined.

## Reading from an Arduino
The `I2CWordMonitor` is the class we use to take a measurement from an Arduino via I2C. This is effectly a shell class, and can be extended for a specific application if necessary.

## Magnetic Field

### HMC5883B
An older but common digital compass sensor, these can be read with the `HMC5883Monitor` class. Maybe.

### QMC5883
A Chinese clone of the HMC5883B, these can be read with the `QMC5883Monitor` class. Try this if you think you have an HMC5883B, but the `HMC5883Monitor` doesn't work at all. (You might find that the NodeMCU hangs and won't do anything.)

### MLX90393
These are a high-resolution field sensor which can measure relatively large fields. They can be monitored with the `MLX90393Monitor` class.

# Serial Monitoring
This is one of the more complex operations of our modules. See the comments and examples of the `SerialMonitor` class. Using this, the ESP8266 can communicate with TTL serial devices. We talk to RS232 devices (using a MAX232 type device), and GPIB (using an arduino with the [AR488](https://github.com/Twilight-Logic/AR488) Firmware) to convert between TTL and standard serial protocols.

See the comments and examples of this class for more details.

Note that this class works better at slower serial rates, and we have had some data loss when trying to run it at 115200 baud.