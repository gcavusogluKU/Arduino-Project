#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>
#include <ph4502c_sensor.h>


#define SEALEVELPRESSURE_HPA (1013.25)
#define humidityPin A3
#define PH4502C_PH_LEVEL_PIN A0
#define PH4502C_TEMP_PIN A1

Adafruit_BME280 bme;
MQ135 mq135_sensor(A2);
PH4502C_Sensor ph4502(PH4502C_PH_LEVEL_PIN, PH4502C_TEMP_PIN);

unsigned long delayTime;
float temperature, humidity, pressure, altitude;
float rzero, correctedRZero, resistance, ppm, correctedPPM;
float soilHumidity;
float pHLevel, temperature2;

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
  pinMode(humidityPin,INPUT);
  ph4502.init();

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

  soilHumidity = (float)(1023 - analogRead(humidityPin))/1024*100;

  pHLevel = ph4502.read_ph_level(); //read_ph_level_single()
  temperature2 = ph4502.read_temp() - 687;
}

void printValues() {
  Serial.print("Temperature=" + String(temperature) + "|");
  Serial.print("Pressure=" + String(pressure) + "|");
  Serial.print("Altitude=" + String(altitude) + "|");
  Serial.print("Humidity=" + String(humidity) + "");
  Serial.println();

  Serial.print("RZero=" + String(rzero) + "|");
  Serial.print("CorrectedRZero=" + String(correctedRZero) + "|");
  Serial.print("Resistance=" + String(resistance) + "|");
  Serial.print("PPM=" + String(ppm) + "|");
  Serial.print("CorrectedPPM=" + String(correctedPPM) + "");
  Serial.println();

  Serial.print("SoilHumidity=" + String(soilHumidity) + "|");
  Serial.print("pHLevel=" + String(pHLevel) + "|");
  Serial.print("Temperature2=" + String(temperature2) + "");
  Serial.println();
}
