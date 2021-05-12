#include <Arduino.h>
#include "TinyGPS++.h"
#include <SoftwareSerial.h>

static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.while (ss.available() > 0) if (gps.encode(ss.read()))
  {

    if (gps.location.isValid() || gps.date.isValid() || gps.time.isValid() || gps.altitude.isValid() || gps.satellites.isValid())
    {
      Serial.print("Lat: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Log: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print(F("Date: "));
      Serial.print(gps.date.month());
      Serial.print(F("/"));
      Serial.print(gps.date.day());
      Serial.print(F("/"));
      Serial.println(gps.date.year());
      Serial.print("Time: ");
      if (gps.time.hour() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10)
        Serial.print(F("0"));
      Serial.println(gps.time.second());
      Serial.print("Alt: ");
      Serial.print(gps.altitude.meters());
      Serial.println(F("m"));
      Serial.print("Sats: ");
      Serial.print(gps.satellites.value());
    }

    else
    {
      Serial.print(F("INVALID"));
    }

    Serial.println();
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true)
      ;
  }
}
