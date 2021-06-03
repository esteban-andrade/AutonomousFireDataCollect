# Electronics Firmware

1. Sensors-Teensy4.0: To be loaded onto the Teensy 4.0. The code is subject to change for fixes and improvements. It's main purpose is to collect data from all attached sensors and parse the data over Serial to ROS SERIAL. The device will all the user to start and stop data recording using a user interface panel on board.

2. ESP8266 - GPS: The ESP8266 module is intended to collect GPS NEMA data and translate that into GPS coordinates for use. The ESP8266 will parse the data and send it over UART to the Teensy 4.0. 

Note: The firmware is written in VSCode using PlatformIOIDE. Please install the required software to modify the code.

Electronic schematic and Altium Designer PCB files inlcuding the BOM can be found here: 
https://github.com/kyleprr/Autonomous-Perception-Bushfires-Electronics
