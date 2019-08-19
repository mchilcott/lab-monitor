# Firmware Updating

## PlatformIO

Once a device has ben programmed via USB, this can be done again.

## Remote Upgrade.

1) Build the firmware. PlatformIO will print out a line near the end of the compilation saying something like
   ```
   Creating BIN file ".pio/build/nodemcuv2/firmware.bin" using ".pio/build/nodemcuv2/firmware.elf"
   ```
   Take note of this.
2) Point a web browser at the sensor node to be updated. This can be done using it's IP address, or by its host name. One can find out the hostname from the serial line on start up (and write it down), or one can listen in to
   `status/#` on the mqtt server (e.g. using NodeRED). Every device broadcasts some details into this channel (using something resembling the Homie Protocol) periodically. This includes its name, hostname and IP address.

   See [nodered_examples/node_details.md](nodered_examples/node_details.md) for a useful NodeRED flow for collecting this information.

   Confirm that you can get a web page from the node, and that the sensors listed here match what you expect.

3) Navigate the browser to `http://<node>/firmware`, and enter the username and password you have specified in `auth.h`.

4) Select the `firmware.bin` file from the directory specified in step 1. Upload this.

    Note that:
        - The directory specified is relative to where PlatformIO is run.
        - The `.pio` directory is hidden by default.
5) Enjoy new firmware. The node will remember WiFi and MQTT config from last time (unless you have made changes to this code in the firmware), so should simply start monitoring again.