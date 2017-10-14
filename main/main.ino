#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//Data will be outputted every COLLECTION_POINTS * LOOP_DELAY milliseconds
#define COLLECTION_POINTS 10
#define LOOP_DELAY 100

// Prototypes
float getAverage(float list[]);

//Sensors
Adafruit_BMP280 bmp;

//Pins
int led = 13;
//Data Buffers
float pressureBuffer[COLLECTION_POINTS];
float temperatureBuffer[COLLECTION_POINTS];
float altitudeBuffer[COLLECTION_POINTS];

//Housekeeping Variables
int loopNum;
bool buffersReady;

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(gps, INPUT);
  loopNum = 0;
  buffersReady = false;
  
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (true);
  }
}

void loop() {
  if (loopNum == 0 && buffersReady) {
    digitalWrite(led, HIGH);
    Serial.print(F("Temperature = "));
    Serial.print(getAverage(temperatureBuffer));
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(getAverage(pressureBuffer));
    Serial.println(" kPa");

    Serial.print(F("Altitude = "));
    Serial.print(getAverage(altitudeBuffer)); 
    Serial.println(" m");
  }
  else {
    digitalWrite(led, LOW);
    temperatureBuffer[loopNum] = bmp.readTemperature(); 
    pressureBuffer[loopNum] = bmp.readPressure() / 1000;//Readings are in Pa
    altitudeBuffer[loopNum] = bmp.readAltitude(1013.25);// TODO: this should be adjusted
  }
  //Serial.print(loopNum);
  loopNum = (loopNum + 1) % COLLECTION_POINTS;
  if (loopNum == 0) {
    buffersReady = true;
  }

  

  delay(LOOP_DELAY);
}

float getAverage(float list[]) {
  float sum = 0;
  int len = sizeof(list) / sizeof(float);
  for (int i = 0; i < len; i++) {
    sum += list[i];
  }
  return sum / len;
}

