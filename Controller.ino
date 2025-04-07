#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
MQ135 mq135_sensor(A2);

unsigned long delayTime;
float temperature, humidity, pressure, altitude;
float rzero, correctedRZero, resistance, ppm, correctedPPM;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // time to get serial running

  unsigned status = bme.begin(0x76);
  Serial.print("SensorID was: 0x");
   Serial.println(bme.sensorID(), 16);

  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1) delay(10);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();
}


void loop() {
  readValues();
  printValues();
  delay(delayTime);
}

void readValues() {
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();

  rzero = mq135_sensor.getRZero();
  correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
  resistance = mq135_sensor.getResistance();
  ppm = mq135_sensor.getPPM();
  correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
}

void printValues() {
  Serial.print("T = " + String(temperature) + " °C | ");
  Serial.print("P = " + String(pressure) + " hPa | ");
  Serial.print("Altitude = " + String(altitude) + " m | ");
  Serial.print("Humidity = " + String(humidity) + "%");
  Serial.println();

  Serial.print("MQ135 RZero: " + String(rzero));
  Serial.print(" | Corrected RZero: " + String(correctedRZero));
  Serial.print(" | Resistance: " + String(resistance));
  Serial.print(" | PPM: " + String(ppm));
  Serial.print(" | Corrected PPM: " + String(correctedPPM));
  Serial.println();
}
