# Provided PCBs

Here we provide a coupld of PCB layouts. One for talking to RS232 devices, and one for GPIB devices. These are produced in KiCad, which is freely available. Most of the documentation is in the schematics, with a couple of notes about them both here.


## RX/TX pins
For the first revision of these boards, we had issues with the connection of the serial interface to the RX pin specified in the datasheet, as this is connected to serial flash, causing the device to crash if you try to use it. Fortunately, the UART pins on the ESP32 are configurable, and can be moved to different pins.

Both the RS232 and GPIB boards are in use, and should work without much in the way of debugging outside of the code running on the ESP32. Do note however that the hardware handshaking for RS232 has not been tested, but is connected to the default pins in the ESP32 datasheet. We have not needed to use this in practise yet. If in doubt, you can leave R2, R4 on the RS232 board unpopulated, and the odds of any trouble are very small. Especially as most lab hardware doesn't support hardware handshakes on their RS232 interface.
