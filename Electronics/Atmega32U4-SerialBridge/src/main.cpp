// To be loaded onto the ATMEGA32U4 onboard the LattePanda. (ONE TIME ONLY!)
// Author: Team 15 - Autonomous Perception for Bushfires (Ignus Automation)
// Date: 10/04/2021
// Version: 1.0

#include <Arduino.h>

void setup() {
   Serial1.begin(115200); // Baud Rate of Teensy
   Serial.begin(115200); // Output Baud Rate (ATMEGA32U4) to ROS Serial
}

void loop() {
  if (Serial.available()) { // Check for ROS Serial
  Serial1.write(Serial.read()); // Send data to Teensy from ATMEGA32U4
  }
  
  if (Serial1.available()) { // Check for Serial Out from Teensy
  Serial.write(Serial1.read()); // Send data from Teensy to ATMEGA32U4
  }
}