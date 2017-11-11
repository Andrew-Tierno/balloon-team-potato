#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MAX31855.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <IridiumSBD.h>

//Data will be outputted every COLLECTION_POINTS * LOOP_DELAY milliseconds
#define COLLECTION_POINTS 25
#define DEBUG false

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
const int chipSelect = 20;

////Sensors
Adafruit_BMP280 bmp = Adafruit_BMP280();
//Adafruit_MAX31855 therm(21);
//Adafruit_BNO055 bno = Adafruit_BNO055();
TinyGPSPlus gps;
SoftwareSerial gpsSS(gpsRX, gpsTX);

//RockBLOCK
#define IridiumSerial Serial1
IridiumSBD modem(IridiumSerial);
int err;

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
bool bmpStarted;
unsigned long startTime;

void setup() {
  Serial.begin(9600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  //pinMode(heaters, OUTPUT);
  
  loopNum = 0;
  startTime = millis();
  buffersReady = false;

  bmpStarted = bmp.begin();
  
//  if (!bmp.begin()) {  
//    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
//    while (true);
//  }
//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
  
  //therm.begin();
  
  // GPS
  gpsSS.begin(GPSBaud);

  // RockBLOCK
  IridiumSerial.begin(19200);
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    return;
  } else {
    Serial.println("Connected to Rockblock");
  }
}

void loop() {
  smartDelay(50);
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

    
  if (millis() > startTime + 5*60*1000)  { // every five minutes
    Serial.println("About to call comms");
    callComms();
    startTime = millis();
  }
  if (loopNum == 0 && buffersReady) {
    digitalWrite(led, HIGH);
    outputDebugStatus();
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    String dataString =           String(gps.location.lat())
                          + "," + String(gps.location.lng())
                          + "," + String(getAltitude())
                          + "," + String(getInternalTemp())
                          + "," + String(getExternalTemp())
                          + "," + String(getPressure())
                          + "\n";

    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }
  else {
    Serial.println("error opening datalog.txt");
  }
    
    //logData();
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
    if (bmpStarted) {
        internalTempBuffer[loopNum] = bmp.readTemperature(); 
        pressureBuffer[loopNum] = bmp.readPressure() / 1000;//Readings are in Pa
        altitudeBuffer[loopNum] = bmp.readAltitude(1013.25);// TODO: this should be adjusted
    }
    else {
        internalTempBuffer[loopNum] = 0; 
        pressureBuffer[loopNum] = 0;//Readings are in Pa
        altitudeBuffer[loopNum] = 0;// TODO: this should be adjusted
    }
  
    //externalTempBuffer[loopNum] = therm.readCelsius();
    
//    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//    rollBuffer[loopNum] = euler.x();
//    pitchBuffer[loopNum] = euler.y();
//    yawBuffer[loopNum] = euler.z();
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

void logData() {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    String dataString =           String(gps.location.lat())
                          + "," + String(gps.location.lng())
                          + "," + String(getAltitude())
                          + "," + String(getInternalTemp())
                          + "," + String(getExternalTemp())
                          + "," + String(getPressure())
                          + "\n";

    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }
  else {
    Serial.println("error opening datalog.txt");
  }
}

void outputDebugStatus() {
    if (DEBUG) {
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
}

// RockBLOCK functions

int getSRBignalQuality() {
  int signalQuality = -1;
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS) {
    Serial.println("ERROR RECEIVING SIGNAL QUALITY");
  }
  return signalQuality;
}

void callComms() {
  String message = String(gps.location.lat()) + "," +  String(gps.location.lng()) + "," + String(getAltitude())  + "," + String(getExternalTemp()) + "," + String(getInternalTemp());
  char messageBuf[80]; // Plus one for null-terminator!
  message.toCharArray(messageBuf, sizeof(messageBuf));
  Serial.println(messageBuf);
  
//  err = modem.sendSBDText(messageBuf);
  err = modem.sendSBDText(messageBuf);
  if (err == ISBD_SENDRECEIVE_TIMEOUT) {
    Serial.println("FAILURE TO SEND MESSAGE. TRY AGAIN WITH BETTER VIEW OF SKY.");
  } else {
    Serial.println("Message Sent!");
    Serial.println(err);
  }
}

bool ISBDCallback() {
  //Serial.println("In call-back");
  smartDelay(50);
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

    
  if (loopNum == 0 && buffersReady) {
    digitalWrite(led, HIGH);
    outputDebugStatus();
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    String dataString =           String(gps.location.lat())
                          + "," + String(gps.location.lng())
                          + "," + String(getAltitude())
                          + "," + String(getInternalTemp())
                          + "," + String(getExternalTemp())
                          + "," + String(getPressure())
                          + "\n";

    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }
  else {
    Serial.println("error opening datalog.txt");
  }
    
    //logData();
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
//  if (loopNum == 0 && buffersReady) {
//    digitalWrite(led, HIGH);
//    outputDebugStatus();
//  }
//  else {
//    digitalWrite(led, LOW);
//  }
//
//  handleHeaters();
//  updateBuffers(loopNum);
//
//  loopNum = (loopNum + 1) % COLLECTION_POINTS;
//  if (loopNum == 0) {
//    buffersReady = true;
//  }

  return true;
}

// if millis() - lasttime > threshold
// call rockblock and updated lasttime


// gps lat long, altitude, int temp, ext temp, 
// 


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSS.available())
      gps.encode(gpsSS.read());
  } while (millis() - start < ms);
}

