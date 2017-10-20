
/* Found this herehttps://forum.pjrc.com/threads/24979-Teensy-3-1-Ultimate-GPS-code
 *  
 * Copyright (C) 2014 by KC3ARY Richard Nash
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _GPS_H
#define _GPS_H

#include <WProgram.h>

// Return from "parseSentence()"
#define PARSED_NONE     0
#define PARSED_UNKNOWN  1
#define PARSED_ERROR    2
#define PARSED_GGA      3
#define PARSED_RMC      4
#define PARSED_ACK      5

// different commands to set the update rate from once per ten seconds (0.1 Hz) to 10 times a second (10Hz)
#define UPDATE_RATE_10000   "$PMTK220,10000*2F"
#define UPDATE_RATE_5000    "$PMTK220,5000*1B"
#define UPDATE_RATE_2000    "$PMTK220,2000*1C"
#define UPDATE_RATE_1000    "$PMTK220,1000*1F"
#define UPDATE_RATE_200     "$PMTK220,200*2C"
#define UPDATE_RATE_100     "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define OUTPUT_RMC      "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on just GGA
#define OUTPUT_GGA      "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define OUTPUT_RMC_GGA  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define OUTPUT_ALLDATA  "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define OUTPUT_OFF      "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

class GPS {
public:
    GPS(HardwareSerial * const, bool ); // Constructor
    void startSerial(const uint16_t);
    void poll(void);
    uint8_t parseSentence(void);
    bool sentenceAvailable(void);

    void standby();
    void setUpdateRate(const char*);
    void setSentencesToReceive(const char*);

    void dataRead();
    bool newValuesSinceDataRead();
    
    // No accessors, just feel free to read this stuff directly.
    // Don't set it though
    uint32_t millisHeadingAcquired, millisAltitudeAcquired, millisPositionAcquired, millisDataRead;
    uint8_t hour, minute, seconds, year, month, day;
    float latitudeDegreesAndFractionalMinutes; 
    float longitudeDegreesAndFractionalMinutes;
    float latitude; // In decimal degrees (no funny degrees and fractional minutes)
    float longitude; // In degrees
    float altitude; // In meters
    float speed; // In knots
    uint16_t heading; // Degrees 0-North, 90-East, 180-South, 270-West
    boolean fix;
    uint8_t fixquality, satellites;

private:
    // Private Methods
    uint8_t parseGGA(const char*, const uint32_t);
    uint8_t parseRMC(const char*, const uint32_t);
    uint8_t parse(const char*, const uint32_t);
    void sendCommand(const char*, bool, const uint32_t);
    void waitForAck(const uint32_t);

    // Private Members
    IntervalTimer pollTimer;
    bool runTimer;
    float degreesAndFractionalMinutesToDegrees(float, const char);
    HardwareSerial *  gpsSwSerial;

#ifdef SIMULATE
    long millisSinceLastSend;
    int simCount;
#endif
};


#endif


/*
 * Copyright (C) 2014 by KC3ARY Richard Nash
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

//#include "GPS.h"

#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_4800 "$PMTK251,4800*14"

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*3"
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// Max number of characters we can read in a GPS sentence
#define GPS_BUFFER_SIZE 100

#define POLL_TIME 10000  // In microseconds, 1/10th of a seconds should be fast enough

// Double buffered interrupt driven reading
static char buf1[GPS_BUFFER_SIZE];
static char buf2[GPS_BUFFER_SIZE];
static char *onBuffer;
static int nCharsRead;
static char *bufferComplete;

// Constructor using SoftwareSerial
// Note: interruptDrivenPolling uses up an IntervalTimer, which will
// be running constantly.
// Note: There is a known race condition in this interrupt driven code
//       In the polling loop which is called at 10 Hz when the buffer fills
//       up, the state of the variable "bufferComplete" is assigned which
//       buffer just filled. That is also the flag for indicating a sentence
//       is ready for parsing. If you check that variable, make sure you
//       parse the sentence before the next one completes or the double
//       buffering could get corrupted.
// Note: If you are not using interruptDrivenPolling, then make sure you
// call the "poll" function fast enough to keep up with the incoming data.
// Ohterwise you can lose characters and get corrupted sentences.
//
// Finally: When using interruptDrivenPolling, just check for "sentenceAvailable"
// and call "parseSentence" faster than the next sentence coming available.
GPS::GPS(HardwareSerial * const ser, bool interruptDrivenPolling )
{
    runTimer = interruptDrivenPolling;
    bufferComplete = 0;
    onBuffer = buf1;
    nCharsRead = 0;
    hour = minute = seconds = year = month = day =
    fixquality = satellites = 0; // uint8_t
    fix = false; // boolean
    latitude = longitude = altitude = speed = 0.0; // float
    heading = 0;
    gpsSwSerial = ser;
    millisHeadingAcquired = 0;
    millisAltitudeAcquired = 0;
    millisPositionAcquired = 0;
#ifdef SIMULATE
    millisSinceLastSend = millis();
    simCount = 0;
#endif
}

void GPS::dataRead()
{
  millisDataRead = millis();
}

bool GPS::newValuesSinceDataRead()
{
  if (millisHeadingAcquired > millisDataRead && millisAltitudeAcquired > millisDataRead && millisPositionAcquired > millisDataRead) return true;
  return false;
}

static GPS *mainGPS; // The only GPS instance that gets callbacks. It's probably best to only have one
// Turn the function call into a method call on this class;
static void poll_fun()
{
  mainGPS->poll();
}

void GPS::startSerial(const uint16_t baud)
{
    gpsSwSerial->begin(baud);
    delay(10);
    sendCommand(OUTPUT_OFF, true, 500);
    if (runTimer) {
      delay(500);
      mainGPS = this;
      pollTimer.begin( poll_fun, POLL_TIME );
    }
}

void GPS::standby()
{
    sendCommand(PMTK_STANDBY, true, 500);
}

void GPS::setUpdateRate(const char*updateString)
{
    sendCommand(updateString, true, 500);
}

void GPS::setSentencesToReceive(const char*sentences)
{
    sendCommand(sentences, true, 500);
}

float GPS::degreesAndFractionalMinutesToDegrees(float deg, const char hemi)
{
    const float degPart = (int)(deg/100.0);
    deg = (deg - degPart*100.0) / 60.0 + degPart;
    if (hemi == 'S' || hemi == 'W') {
        return -deg;
    } else {
        return deg;
    }
}

uint8_t GPS::parseGGA(const char* nmea, const uint32_t now)
{
    // Assuming a GGA sentence
    const char* p = nmea;
    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
    
    // parse out latitude
    p = strchr(p, ',')+1;
    latitude = atof(p);
    
    p = strchr(p, ',')+1; // Should be 'N' or 'S'
    latitude = degreesAndFractionalMinutesToDegrees(latitude, p[0]);
    
    // parse out longitude
    p = strchr(p, ',')+1;
    longitude = atof(p);
    
    p = strchr(p, ',')+1; // Should be 'W' or 'E'
    longitude = degreesAndFractionalMinutesToDegrees(longitude, p[0]);
    
    p = strchr(p, ',')+1;
    fixquality = atoi(p);
    
    p = strchr(p, ',')+1;
    satellites = atoi(p);
    
    p = strchr(p, ',')+1; // HDOP, throw away
    
    p = strchr(p, ',')+1;
    altitude = atof(p);
    
    millisAltitudeAcquired = now;
    millisPositionAcquired = now;
    
    return PARSED_GGA;    
}

uint8_t GPS::parseRMC(const char* nmea, const uint32_t now)
{
    // found RMC
    const char* p = nmea;
    
    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
    
    p = strchr(p, ',')+1;
    fix = (p[0] == 'A');
    
    // parse out latitude
    p = strchr(p, ',')+1;
    latitude = atof(p);
    
    p = strchr(p, ',')+1; // Should be 'N' or 'S'
    latitude = degreesAndFractionalMinutesToDegrees(latitude, p[0]);
    
    // parse out longitude
    p = strchr(p, ',')+1;
    longitude = atof(p);
    
    p = strchr(p, ',')+1; // Should be 'W' or 'E'
    longitude = degreesAndFractionalMinutesToDegrees(longitude, p[0]);
    
    // speed
    p = strchr(p, ',')+1;
    speed = atof(p); // Speed in knots
    
    // heading
    p = strchr(p, ',')+1;
    heading = (uint16_t)atof(p);
    
    p = strchr(p, ',')+1;
    uint32_t fulldate = atof(p);
    day = fulldate / 10000;
    month = (fulldate % 10000) / 100;
    year = (fulldate % 100);
    
    millisHeadingAcquired = now;
    millisPositionAcquired = now;
    
    return PARSED_RMC;
}

uint8_t GPS::parse(const char* nmea, const uint32_t now)
{
  //Serial.print("P: ");
  //Serial.println(nmea);
    if (*nmea != '$') {
        return PARSED_ERROR;
    }
    if (strstr(nmea, "$GPGGA") == nmea) {
        if (strlen(nmea) < 65) return PARSED_UNKNOWN; // No fix
        return parseGGA(nmea, now);
    } else if (strstr(nmea, "$GPRMC") == nmea) {
        if (strlen(nmea) < 65) return PARSED_UNKNOWN; // No fix
        return parseRMC(nmea, now);
    } else if (strstr(nmea, "$PMTK001") == nmea) {
        return PARSED_ACK;
    }
    return PARSED_UNKNOWN;
}

void GPS::sendCommand(const char* str, const bool wait, const uint32_t timeout) {
#ifdef SIMULATE
    // Do nothing if in simulation mode
#else
  //Serial.print("GPS(");
  //Serial.print(str);
  //Serial.println(")");
    gpsSwSerial->println(str);
    if (wait) waitForAck(timeout);
#endif
}

// Wait until the command sent gets an ACK back
void GPS::waitForAck(const uint32_t timeout)
{
    uint32_t giveUpTime = 0;
    if (timeout) {
        giveUpTime = millis()+timeout;
    }
    while (!giveUpTime || millis() < giveUpTime) {
        if (parseSentence() == PARSED_ACK) break;
    }
    
}

// Call to check for available characters
void GPS::poll(void)
{
  while (gpsSwSerial->available()) {
    const char c = gpsSwSerial->read();
    if (c != -1) {
        onBuffer[nCharsRead++] = c;
        if (nCharsRead == GPS_BUFFER_SIZE-1 || c == '\n') {
            onBuffer[nCharsRead] = 0;
            nCharsRead = 0;
            if (onBuffer == buf1) {
                onBuffer = buf2;
                bufferComplete = buf1;
            } else {
                onBuffer = buf1;
                bufferComplete = buf2;
            }
        }
    }
  }
}

 // Has a sentence been made available?
bool GPS::sentenceAvailable(void)
{
  return bufferComplete != NULL;
}

// Call this from loop to parse any available sentences
uint8_t GPS::parseSentence(void)
{
    uint8_t retVal = PARSED_NONE;
    if (sentenceAvailable()) {
      retVal = parse(bufferComplete, millis());
      bufferComplete = 0;
    }
    return retVal;
}


// GPSExample.cpp

/*
 * Copyright (C) 2014 by Richard Nash (KC3ARY)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <WProgram.h>
//#include <GPS.h>

HardwareSerial &gpsSerial = Serial1;
GPS gps(&gpsSerial,true);

// setup() method runs once, when the sketch starts
void setup()
{
  Serial.begin(38400); // For debugging output over the USB port
  gps.startSerial(9600);
  delay(1000);
  gps.setSentencesToReceive(OUTPUT_RMC_GGA);
}

bool gotGPS = false;
// the loop() methor runs over and over again,
// as long as the board has power
void loop()
{
  Serial.println("Starting loop");
  if (gps.sentenceAvailable()) gps.parseSentence();

  if (gps.newValuesSinceDataRead()) {
    gotGPS = true;
    gps.dataRead();
    Serial.printf("Location: %f, %f altitude %f\n\r",
      gps.latitude, gps.longitude, gps.altitude);
  }

  delay(100);
}

// Called from the powerup interrupt servicing routine.
int main(void)
{
  setup();
  while (true) {
    loop();
    yield();
  }
  return 0; // Never reached.
}


