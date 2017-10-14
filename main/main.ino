#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MAX31855.h>

//Data will be outputted every COLLECTION_POINTS * LOOP_DELAY milliseconds
#define COLLECTION_POINTS 10
#define LOOP_DELAY 100
#define DEBUG true

//TODO: Get actual values. Should be in celsius.
#define MIN_INTERNAL_TEMP 15
#define MAX_INTERNAL_TEMP 30

//Sensors
Adafruit_BMP280 bmp;
Adafruit_MAX31855 therm(14, 10, 12);

//Pins
int led = 13;
int heaters = 22;

//Data Buffers
float pressureBuffer[COLLECTION_POINTS];
float internalTempBuffer[COLLECTION_POINTS];
float externalTempBuffer[COLLECTION_POINTS];
float altitudeBuffer[COLLECTION_POINTS];

//Housekeeping Variables
int loopNum;
bool buffersReady;

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(heaters, OUTPUT);

  loopNum = 0;
  buffersReady = false;
  
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (true);
  }
  therm.begin();
}

void loop() {
  if (loopNum == 0 && buffersReady) {
    digitalWrite(led, HIGH);
    outputDebugStatus();
  }
  else {
    digitalWrite(led, LOW);
  }

  handleHeaters();

  updateBuffers(loopNum);
  loopNum = (loopNum + 1) % COLLECTION_POINTS;
  if (loopNum == 0) {
    buffersReady = true;
  }

  delay(LOOP_DELAY);
}

void handleHeaters() {
  float temp = getInternalTemp();
  if (temp > (MAX_INTERNAL_TEMP - MIN_INTERNAL_TEMP) / 2 + MAX_INTERNAL_TEMP) {
    digitalWrite(heaters, LOW);
  }
  else {
    digitalWrite(heaters, HIGH);
  }
}

void updateBuffers(int loopNum) {
    internalTempBuffer[loopNum] = bmp.readTemperature(); 
    externalTempBuffer[loopNum] = therm.readCelsius();
    pressureBuffer[loopNum] = bmp.readPressure() / 1000;//Readings are in Pa
    altitudeBuffer[loopNum] = bmp.readAltitude(1013.25);// TODO: this should be adjusted
}

float getInternalTemp() {
  return getAverage(internalTempBuffer);
}

float getExternalTemp() {
  return getAverage(externalTempBuffer);
}

float getPressure() {
  return getAverage(pressureBuffer);
}

float getAltitude() {
  return getAverage(altitudeBuffer);
}

float getAverage(float list[]) {
  float sum = 0;
  int len = sizeof(list) / sizeof(float);
  for (int i = 0; i < len; i++) {
    sum += list[i];
  }
  return sum / len;
}

void outputDebugStatus() {
    Serial.print("Internal Temperature = ");
    Serial.print(getInternalTemp());
    Serial.println(" *C\n");

    Serial.print("External Temperature = ");
    Serial.print(getExternalTemp());
    Serial.println(" *C\n");

    Serial.print("Pressure = ");
    Serial.print(getPressure());
    Serial.println(" kPa\n");

    Serial.print("Altitude = ");
    Serial.print(getAltitude()); 
    Serial.println(" m\n");
}


