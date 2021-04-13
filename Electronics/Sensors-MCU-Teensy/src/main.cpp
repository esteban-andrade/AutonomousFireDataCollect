// To be loaded onto the Teensy 4.0
// Author: Team 15 - Autonomous Perception for Bushfires (Ignus Automation)
// Date: 10/04/2021
// Version: 1.0

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MQ2.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME280.h"
#include <TinyGPS++.h>

#define SEALEVELPRESSURE_HPA (1013.25) // Calibration for Pressure Level

// Software Serial pins for GPS
int RXPin = 2;
int TXPin = 3;
int GPSBaud = 9600; // Default NEO-6M Baud Rate
SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps; // TinyGPS++ Object

int Analog_Input = A0; // Smoke Sensor (MQ2)
int lpg, co, smoke;
MQ2 mq2(Analog_Input);

Adafruit_BME280 bme; // BMP280 Sensor Object

void displayInfo();

void setup()
{
  Serial.begin(115200); // Baud Rate
  Serial.println("MQ2 Gas sensor warming up");
  if (!bme.begin(0x76)) { // I2C address for BMP280 is either 0x76 or 0x77
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  mq2.begin(); // Start MQ2 Gas sensor
  gpsSerial.begin(GPSBaud); // Start Software Serial Port
}

void loop()
{
  /////////// MQ2 ///////////
  Serial.print("LPG:");
  Serial.print(mq2.readLPG());
  Serial.println(" ppm\t");
  Serial.print("CO:");
  Serial.print(mq2.readCO());
  Serial.println(" ppm\t");
  Serial.print("SMOKE:");
  Serial.print(mq2.readSmoke());
  Serial.println(" ppm\n");

  /////////// BMP280 ///////////
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println("*C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println("hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println("m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println("%");

  /////////// GPS ///////////
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  // If 5000 milliseconds pass and there are no characters coming in over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while (true);
  }
  delay(1000); // Wsit 1s for next reading
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }

  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }
}