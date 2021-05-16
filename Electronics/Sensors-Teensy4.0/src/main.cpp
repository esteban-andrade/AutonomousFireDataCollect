#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include "SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Particle_SN-GCJA5
SFE_PARTICLE_SENSOR myAirSensor;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

unsigned long previousMillis = 0;
const long delayInterval = 1000;
bool sendData = false;

String datapayload;

void setup()
{
  // Initialise Serial
  Serial.begin(115200);

  Wire.begin();

  if (!bme.begin())
  {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1)
      ;
  }

  if (myAirSensor.begin() == false)
  {
    Serial.println("The particle sensor did not respond. Please check wiring. Freezing...");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  delay(5000);
}

void loop()
{
  if (Serial.available())
  {
    char incomingChar = Serial.read();
    // Check for call and response
    if (incomingChar == 'J')
    {
      Serial.print('I');
    }
    if (incomingChar == '0')
    {
      sendData = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
    if (incomingChar == '1')
    {
      sendData = true;
      Serial.println("MQ2-Gas,Temp,Pressure,Humidity,BME680-Gas,PM1.0,PM2.5,PM10,PC0.5,PC1.0,PC2.5,PC5,PC7.5,PC10");
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= delayInterval)
  {
    previousMillis = currentMillis;
    if (sendData)
    {
      int sensorValue = analogRead(A0);
      //Serial.print("MQ2-Gas: ");
      // Serial.print(sensorValue);

      ///////////////////////////////////////////////////////////////////////

      // Tell BME680 to begin measurement.
      unsigned long endTime = bme.beginReading();
      if (endTime == 0)
      {
        Serial.println(F("Failed to begin reading :("));
        return;
      }

      if (!bme.endReading())
      {
        Serial.println(F("Failed to complete reading :("));
        return;
      }
      // Serial.print(",");
      //Serial.print(F("Temperature = "));
      // Serial.print(bme.temperature);
      //Serial.println(F("°C"));

      // Serial.print(",");
      //Serial.print(F("Pressure = "));
      // Serial.print(bme.pressure / 100.0);
      //Serial.println(F("hPa"));

      // Serial.print(",");
      //Serial.print(F("Humidity = "));
      // Serial.print(bme.humidity);
      //Serial.println(F("%"));

      //Serial.print(F("Approx. Altitude = "));
      //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      //Serial.println(F("m"));

      // Serial.print(",");
      //Serial.print(F("Gas = "));
      // Serial.print(bme.gas_resistance / 1000.0);
      //Serial.println(F("KΩ"));

      //////////////////////////////////////////////////////////////////////////////

      // Serial.print(",");
      //Serial.print("PM1.0: ");
      float pm1_0 = myAirSensor.getPM1_0();
      // Serial.print(pm1_0, 3); //Print float with 2 decimals

      // Serial.print(",");
      //Serial.print("PM2.5: ");
      float pm2_5 = myAirSensor.getPM2_5();
      // Serial.print(pm2_5, 3);

      // Serial.print(",");
      //Serial.print("PM10: ");
      float pm10 = myAirSensor.getPM10();
      // Serial.print(pm10, 3);

      // Serial.print(",");
      //Serial.print("PC0.5: ");
      unsigned int pc0_5 = myAirSensor.getPC0_5();
      // Serial.print(pc0_5);

      // Serial.print(",");
      //Serial.print("PC1.0: ");
      unsigned int pc1_0 = myAirSensor.getPC1_0();
      // Serial.print(pc1_0);

      // Serial.print(",");
      //Serial.print("PC2.5: ");
      unsigned int pc2_5 = myAirSensor.getPC2_5();
      // Serial.print(pc2_5);

      // Serial.print(",");
      //Serial.print("PC5.0: ");
      unsigned int pc5_0 = myAirSensor.getPC5_0();
      // Serial.print(pc5_0);

      // Serial.print(",");
      //Serial.print("PC7.5: ");
      unsigned int pc7_5 = myAirSensor.getPC7_5();
      // Serial.print(pc7_5);

      // Serial.print(",");
      //Serial.print("PC10: ");
      unsigned int pc10 = myAirSensor.getPC10();
      // Serial.print(pc10);
      String sensor_value = sensorValue;
      String bme_temp = bme.temperature;
      String bme_press = (bme.pressure / 100.0);
      String bme_gass = (bme.gas_resistance / 1000.0);
      String bme_hum = bme.humidity;
      String pm1 = pm1_0;
      String pm2 = pm2_5;
      String pm10_ = pm10;
      String pc0_5_ = pc0_5;
      String pc1_0_ = pc1_0;
      String pc2_5_ = pc2_5;
      String pc5_0_ = pc5_0;
      String pc7_5_ = pc7_5;
      String pc10_ = pc10;

      // datapayload = (sensorValue, bme.temperature, (bme.pressure / 100.0), bme.humidity, (bme.gas_resistance / 1000.0),
      //                (pm1_0, 3), (pm2_5, 3), (pm10, 3), pc0_5, pc1_0, pc2_5, pc5_0, pc7_5, pc10);
      datapayload = sensor_value + ";" + bme_temp + ";" + bme_press + ";" + bme_hum + ";" + bme_gass + ";" + pm1 + ";" + pm2 + ";" + pm10_ + ";" + pc0_5_ + ";" + pc1_0_ + ";" + pc2_5_ + ";" + pc5_0_ + ";" + pc7_5_ + ";" + pc10_;
      Serial.println(datapayload);
    }
  }
}

// END OF CODE