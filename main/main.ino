#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MAX31855.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Data will be outputted every COLLECTION_POINTS * LOOP_DELAY milliseconds
#define COLLECTION_POINTS 25
#define LOOP_DELAY 30
#define DEBUG true

//TODO: Get actual values. Should be in celsius.
#define MIN_INTERNAL_TEMP 15
#define MAX_INTERNAL_TEMP 30

// Prototypes
float getAverage(float list[]);

//Pins
int led = -1;
int heaters = -1; //TODO: Get actual values
int gpsRX = 7;
int gpsTX = 8;
int GPSBaud = 9600;

//Sensors
Adafruit_BMP280 bmp;
Adafruit_MAX31855 therm(13, 21, 6);
Adafruit_BNO055 bno = Adafruit_BNO055();
TinyGPSPlus gps;
SoftwareSerial gpsSS(gpsRX, gpsTX);


//Data Buffers
float pressureBuffer[COLLECTION_POINTS];
float internalTempBuffer[COLLECTION_POINTS];
float externalTempBuffer[COLLECTION_POINTS];
float altitudeBuffer[COLLECTION_POINTS];
//x - roll, y - pitch, z - yaw
float rollBuffer[COLLECTION_POINTS];
float pitchBuffer[COLLECTION_POINTS];
float yawBuffer[COLLECTION_POINTS];

//Housekeeping Variables
int loopNum;
bool buffersReady;

void setup() {
  Serial.begin(9600);
  //pinMode(led, OUTPUT);
//<<<<<<< HEAD
//  pinMode(gps, INPUT);
//=======
  pinMode(heaters, OUTPUT);
  

//>>>>>>> dcdf4817144ccf3eca1ee145283ac59d317f1e8c
  loopNum = 0;
  buffersReady = false;
  
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (true);
  }
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  therm.begin();

  // GPS
  gpsSS.begin(GPSBaud);
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
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    rollBuffer[loopNum] = euler.x();
    pitchBuffer[loopNum] = euler.y();
    yawBuffer[loopNum] = euler.z();
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

float getRollAngle() {
  return getAverage(rollBuffer);
}

float getPitchAngle() {
  return getAverage(pitchBuffer);
}

float getYawAngle() {
  return getAverage(yawBuffer);
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
    Serial.println("===============================");
    Serial.print("Internal Temperature = ");
    Serial.print(getInternalTemp());
    Serial.println(" *C");

    Serial.print("External Temperature = ");
    Serial.print(getExternalTemp());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(getPressure());
    Serial.println(" kPa");

    Serial.print("Altitude = ");
    Serial.print(getAltitude()); 
    Serial.println(" m");

    Serial.print("Roll = ");
    Serial.print(getRollAngle());
    Serial.println("*");

    Serial.print("Pitch = ");
    Serial.print(getPitchAngle());
    Serial.println("*");

    Serial.print("Yaw = ");
    Serial.print(getYawAngle());
    Serial.println("*");

    Serial.print("Lat = ");
    Serial.print(gps.location.lat());
    Serial.print(", Long = ");
    Serial.println(gps.location.lng());

    Serial.print("# of GPS Sats = ");
    Serial.println(gps.satellites.value());
    Serial.println("===============================");
}


