# Lab Monitoring System

This project contains the firmware for the monitoring nodes used in [Kjaergaard Lab](http://physics.otago.ac.nz/research/kjaergaardlab/), University of Otago, New Zealand.

## System Overview


## Getting Started (Firmware)
1. To compile and upload this firmware, we use [PlatformIO](https://platformio.org/). PlatformIO is capable of programming many different devices, automatically configureing toolchains and doing much of the hard work of setting a development environment up for you. We recommend most people install PlatformIO along with VSCode by following the instructions [here](https://platformio.org/install/ide?install=vscode).

  > If one has a preference for a different editor, many are supported by PlatformIO, or one can use [PlatformIO Core](https://docs.platformio.org/en/latest/core.html) from the commandline (or makefiles, or configure commands in another IDE). This project can also be compiled from the Arduino IDE, though this is left as an exercise for the reader. (One must take care to follow the Arduino conventions for libraries and file names, and also install the arduino toolchain for ESP8266.) We will proceed assuming that you are using VSCode, and PlatformIO.

2. Get the code, if you haven't already. You can download a zip file.

  > For the most up-to-date version of the code, using git one can run
  > ```
  > git clone --recurse-submodules git://github.com/mchilcott/nodemcu_monitor.git
  > ```
  > This will fetch the latest verson of our code, as well as the libraries we use for connecting to different sensors, which are loaded as submodules in git 
  >
  > You may also want to generate the documentation with Doxygen (if installed) using
  > ```
  > doxygen Doxyfile
  > ```

3. Load the code directory into VSCode -- Use `Open Project` from the PlatformIO Home.

4. Open `src/auth.h`. Inside this file, one can set the username and password used for remote firmware updates, and for the MQTT connection. Pick a username and password combination for the firmware update. If you don't use MQTT authentication (this is the default with the server setup below), then these can be left as `nullptr`, which disables MQTT authentication.

5. Open `src/sensor_config.h`. Give the `node_name` variable a meaningful parameter. Each name should be unique to your monitoring system, and explain what it does. One can then add a list of monitoring classes. For now, we can start off by setting up a simple analog monitor:

'''
const char * node_name = "analog_example_monitor";

std::vector<DCThread *> collectors = {
    new AnalogMonitor ("sensor/example/analog"),
};
'''

The parameter here is the MQTT topic to which the data will be posted. One can think of this as the monitored signal's name.

The `AnalogMonitor` class allows other parameters. The full list can be seen in `MonitorThreads.h` or in the generated documentation at `doc/generated/html/classAnalogMonitor.html`

If we were measuring an analog signal that corresponded to a resistance with a ratio of 1 mV/ohm, which we wanted to measure once every 0.5 seconds (500 milliseconds), we could set up the measurement
```
std::vector<DCThread *> collectors = {
    new AnalogMonitor ("sensor/example/resistance", 500, 1000 /* ohms per volt */, 0, "Ohms"),
};
```

The measurements would then be reported in ohms once every half second.

6. Connect the NodeMCU board to your computer via USB, and upload the firmware. In VSCode, this is done with upload button at the bottom of the screen, or by Terminal -> Run Task (Ctrl+Alt+T), and selecting Platform IO: Upload.

This will compile your firmware, and upload via USB. If the compilation fails, pay attention to the errors, and check your code. If the upload fails, make sure that the NodeMCU is connected, and that platformIO is using the correct serial port. This can be configured in `platformio.ini` -- see the example in this file, or see the [platformio docs](https://docs.platformio.org/en/latest/projectconf/section_env_upload.html).

7. Take the freshly programmed node to the lab. For the analog example, connect the signal between A0 and GND and supply power. Power can be supplied by the USB connector, into the Vin port (Which will take anything between 4.75V and 10V), or put 3.3 V into one of the the 3V3 pins. (Be careful if taking the last option.)

8. On your phone (or other convenient wifi device), look for a WiFi network with the name `ESP` followed by a sequence of numbers. Connect to this, then point your web browser to 192.168.4.1, or try to open a website, and you should be redirected here.

9. From the page that shows up, select to connect to WiFi, and then:
  - Select the network (SSID) your monitoring server is on, and provide the password
  - Set the address of the monitoring server. (Port 1883 is the default for MQTT. Unless you have a different setup, you won't have to change this.)
  - Save this configuration

10. Confirm that data is getting sent your monitoring server. The WiFi network you found in 7 should disappear. If the node isn't able to connect to WiFi, or the server, then check that there isn't an issue with your configuration and repeat set 7.

11. Enjoy collecting data.


## Getting Started (Services)
In our lab, the data collected by the nodes is sent to a MQTT server (mosquitto). From here, data is put in a database (InfluxDB, put there by Telegraf) for visualisation (with Grafana), and made available to an alerting system (NodeRED).

> The firmware expects to publish data to an MQTT server. The rest of the pipeline is flexible, and these measurements can be integrated with an existing system. Telegraf is a useful tool for connecting different data sources (e.g. MQTT) to different data sinks. One can also find libraries for MQTT in many different languages, simplifying developing your own interface.

To set up the services that we use:
0. Install Docker (Community Edition) on the server - You click on your platform [here](https://docs.docker.com/install/#supported-platforms) to get instructions on how to install docker. (While this is open-source software, it is getting increasingly difficult to download and install docker without having to make an account, and even harder to get from their home page to a place where you can find the installer.)

The advantage of Docker, is that it will manage downloading, running and integrating the services.

It may help to erify that docker is installed properly, by running
```
docker run hello-world
```

Note that you may need to prefix this command, and those following with `sudo` if you're running linux, and haven't added yourself to the [docker group](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user). If you get a "permission denied" type error, try adding `sudo` at the start. You'll probably be prompted to enter your login password. 

1. Install [docker-compose](https://docs.docker.com/compose/install/). This tool is used to automate setting up multiple services together.

2. Copy `telegraf.conf` and `docker-compose.yml` to a directory on your server

3. From this directory, run the command
```
docker-compose up
```

4. If you haven't run any of these services before, take a well-deserved break. Docker will now download all the services (which takes a while) and start them up.

5. Point your web browser at `http://<server>:3000` to get access to Grafana, whiere you can start to build queries, `http://<server>:8888` to get access to Chronograf, which lets you configure the database,
or `http://<server>:1880` to get access to NodeRED and start to build automation flows.

For more information on playing with these services, see `doc/service-guide.md`.