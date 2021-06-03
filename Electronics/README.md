# Electronics

#### The code folder contains the firmware required for the following microcontrollers:
1. [`Sensors-Teensy4.0`](https://github.com/kyleprr/AutonomousFireDataCollect/tree/main/Electronics/Sensors-Teensy4.0): Once loaded onto the Teensy 4.0, it's main purpose is to collect data from all attached sensors and parse the data over Serial to ROS SERIAL. The device will all the user to start and stop data recording using a user interface panel on board.
2. [`ESP8266`](https://github.com/kyleprr/AutonomousFireDataCollect/tree/main/Electronics/Sensors-Teensy4.0): The ESP8266 module is intended to collect GPS NEMA data and translate that into GPS coordinates for use. The ESP8266 will parse the data and send it over UART to the Teensy 4.0. 

Note: The firmware is written in VSCode using PlatformIOIDE. Please install the required software to modify the code.



The pcb_project_files folder contains the Altium Designer schematic and PCB files for the four PCB modules as shown below:

### Custom Printed Circuit Boards Developed


#### 1. Main Controller:
<img src="https://github.com/kyleprr/Autonomous-Perception-Bushfires-Electronics/blob/main/Images/Main-Controller.jpg" width="400">

#### 2. Power Management:
<img src="https://github.com/kyleprr/Autonomous-Perception-Bushfires-Electronics/blob/main/Images/Power-Management.jpg" width="400">

#### 3. Sensors:
<img src="https://github.com/kyleprr/Autonomous-Perception-Bushfires-Electronics/blob/main/Images/Sensors.jpg" width="400">

#### 4. User Controller:
<img src="https://github.com/kyleprr/Autonomous-Perception-Bushfires-Electronics/blob/main/Images/User-Controller.jpg" width="400">

#### 5. Connections/Layout:
<img src="https://github.com/kyleprr/Autonomous-Perception-Bushfires-Electronics/blob/main/Images/Connections.png" width="1200">
