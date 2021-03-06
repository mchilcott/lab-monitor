# Project documentation

This folder hosts the following:
- [generated/html/index.html](generated/html/index.html): Documentation generated from the code by Doxygen. If you pulled this repository from github, you will need to run doxygen on `../Doxyfile` to generate this.
- [firmware_updates.md](firmware_updates.md): How to update Over-the-Air (OTA) using a web browser.
- [Services.org](Serivces.org): More indepth notes on setting up and using the data collection/analysis services. May contain some out of date information.
- [docker-compose.yml](docker-compose.yml), [telegraf.conf]([telegraf.conf]): Files to automate setting up the data collection/analysis services. See the [main readme](../README.md) for details.
- [usb_device_monitor.py](usb_device_monitor.py): An outline in python for submitting values to the monitoring system from a computer (e.g. from polling a USB instrument). We also provide [an example](wavemeter/) of monitoring a laser beam with a Bristol 621 Wavelength meter, which has a USB (virtual serial) interface.
- [nodered_examples/](nodered_examples/): A Small collection of examples of useful flows for NodeRED.
- [Sensors.md](Sensors.md): Some notes on all the sensors we have used.
