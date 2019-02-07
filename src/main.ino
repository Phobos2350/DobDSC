/**
    DobDSC - Arduino/ESP32 Dobsonia telescope Push-To system
    Copyright (C) 2018 A Flight
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    Inspiration and sources used in development:

    Toshimi Taki's methods and Equations for Pointing Telescope
    http://www.geocities.jp/toshimi_taki/aim/aim.htm

    Astronomical Algorithms
    Jean Meeus
    http://www.willbell.com/math/mc1.htm

    How to compute planetary positions
    Paul Schlyter, Stockholm, Sweden
    http://www.stjarnhimlen.se/comp/ppcomp.html

    DobsonianDSC - Simple low cost circuit to connect an inexpensive optical rotary encoder (for azimuth) and a high resolution accelerometer (for altitude) to a dobsonian style telescope
    Copyright GNU GPL (c) 2017 vlaate
    https://github.com/vlaate/DobsonianDSC

    rDUINOScope - Arduino based telescope control system (GOTO).
    Copyright GNU GPL (c) 2016 Dessislav Gouzgounov (Desso)  
    http://rduinoscope.byethost24.com

    Stellarduino - A multi platform, Arduino powered, open source, and open hardware telescope controller and PC interface.
    Copyright (c) 2013 Casey Fulton
    https://github.com/caseyfw/Stellarduino

    OnStep - Arduino telescope goto for equatorial and alt/az mounts
    http://www.stellarjourney.com/index.php?r=site/equipment_onstep

    Arduino-Telescope-Control - Telescope control with Stellarium, Python and Arduino
    Copyright MIT (c) 2012 Juan Ramón 
    https://github.com/juanrmn/Arduino-Telescope-Control

    DSCClient - Digital Setting Circles program for my telescope with equatorial platform
    Copyright GNU GPL (c) AlexyBobkov 
    https://github.com/AlexeyBobkov/DSCClient

    arduino_planet_ephi_positions - Arduino calculates ephemeride planet positions
    mobifu1
    https://github.com/mobifu1/arduino_planet_ephi_positions

**/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEFINES AND INIT

#define FS_NO_GLOBALS
#include <FS.h>
#include <SPIFFS.h>
#include <SD.h>

#include <SPI.h>
#include <Wire.h>

#include <WiFi.h>
#include <BluetoothSerial.h>
#include <TimeLib.h>

#include <Encoder.h>
#include <TinyGPS++.h>
#include <TFT_eSPI.h>
#include <RtcDS3231.h>
#include <LSM303.h> // Install this on Arudino IDE: "LSM303 Library by Pololu" (I used version 3.0.1), https://github.com/pololu/lsm303-arduino

#include <libnova_main.h>

// #define SERIAL_DEBUG // comment out to deactivate the serial debug mode
// #define PERFECT_ALIGN
// #define ALIGN_OFFSET
// #define DEBUG_REFRACTION
// #define DEBUG_MOUNT_ERRS

#define GEAR_LARGE 60.0f
#define GEAR_SMALL 18.0f
#define ENCODER_RES 2400.0f
#define EMA_WEIGHT 0.05f

// To give sufficient CPU time to the TCP Server, a time delta between measurements is enforced:
#define MEASUREMENT_PERIOD 50 // how many milliseconds must have elapsed since last measurement in order to take a new one

// Assign human-readable names to some common 16-bit color values:
#define BLACK 0x0000  /*   0,   0,   0 */
#define MAROON 0x7800 /* 128,   0,   0 */
#define RED 0xF800    /* 255,   0,   0 */
#define WHITE 0xFFFF  /* 255, 255, 255 */

#define CALIBRATION_FILE "/TouchCalData1"
#define REPEAT_CAL false

#define BT_START 1
#define BT_END 2
#define START_CHAR ':'
#define END_CHAR '#'
#define GET_RA "GR"
#define GET_DEC "GD"
#define CHANGE_PRECISION "U"

////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMU CLASS

class IMU : public LSM303
{
    // Calibration of accelerometer: These are the minimum and maximum values measured for X, Y and Z:
    LSM303::vector<int16_t> acc_min = (LSM303::vector<int16_t>){-20480, -20336, -22608};
    LSM303::vector<int16_t> acc_max = (LSM303::vector<int16_t>){+19280, +20112, +20896};

    // measurements are stored here after calling calculatePosition();
  public:
    float azimuthReading; // multiply this by 180 / PI to get degrees
  public:
    float altitudeReading; // multiply this by 180 / PI to get degrees

  public:
    float smoothAzimuthReading; // value is already converted from radians to steps, and smoothing alorithm applied
  public:
    float smoothAltitudeReading; // value is already converted from radians to steps, and smoothing alorithm applied

    vector<float> East = {0, 0, 0};

    const float GEAR_RATIO = GEAR_LARGE / GEAR_SMALL;
    const int STEPS_IN_FULL_CIRCLE = ENCODER_RES * GEAR_RATIO;
    const double STEPS_TO_RAD = STEPS_IN_FULL_CIRCLE / (PI * 2.0);

  public:
    void calculatePosition()
    {
        vector<int> zAxis = (vector<int>){0, 0, 1}; // the heading will be measured relative to Z axis, make sure to place your sensor in a vertical position (Z axis pointing to the horizon)

        // get raw accelerometer data:
        vector<float> acc_reading = {a.x, a.y, a.z};

        // calibrate accelerometer bias and scale using measured maximums and minimums, and apply smoothing algorithm to reduce outlier values:
        acc_reading.x = acc_reading.x * (1 - EMA_WEIGHT) + EMA_WEIGHT * (32767.0 * ((float)a.x - (float)acc_min.x) / ((float)acc_max.x - (float)acc_min.x) - 16383.5);
        acc_reading.y = acc_reading.y * (1 - EMA_WEIGHT) + EMA_WEIGHT * (32767.0 * ((float)a.y - (float)acc_min.y) / ((float)acc_max.y - (float)acc_min.y) - 16383.5);
        acc_reading.z = acc_reading.z * (1 - EMA_WEIGHT) + EMA_WEIGHT * (32767.0 * ((float)a.z - (float)acc_min.z) / ((float)acc_max.z - (float)acc_min.z) - 16383.5);

        // calculate the altitude as an angle from 0º to 90º, see equation #26 from https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
        altitudeReading = -1 * atan2(acc_reading.y, sqrt(acc_reading.x * acc_reading.x + acc_reading.z * acc_reading.z));

        // now adapt the 0º to 90º reading to the 0º to 360º st_yle encoder output expected by Skysafari:
        if (acc_reading.x > 0 && acc_reading.y < 0)
        {
            // 1st quadrant - do nothing
        }
        else if (acc_reading.x < 0 && acc_reading.y < 0)
        {
            // 2nd quadrant
            altitudeReading = PI - altitudeReading;
        }
        else if (acc_reading.x < 0 && acc_reading.y > 0)
        {
            // 3rd quadrant
            altitudeReading = PI - altitudeReading;
        }
        else if (acc_reading.x > 0 && acc_reading.y > 0)
        {
            // 4th quadrant
            altitudeReading = 2 * PI + altitudeReading;
        }

        float newAltitudeReading = altitudeReading * STEPS_TO_RAD;

        /* Final smoothing algotithm: */
        // When the new readings are less than 0.5 degrees off from the old readings, use 0.05 * EMA_WEIGHT as the alpha, for a highly smoothed result
        if (abs(newAltitudeReading - smoothAltitudeReading) < STEPS_IN_FULL_CIRCLE / 720)
        {
            smoothAltitudeReading = newAltitudeReading * EMA_WEIGHT * 0.05 + ((1 - EMA_WEIGHT * 0.05) * smoothAltitudeReading);
        }
        else // When the new readings are more than 0.5 degrees off from the old readings, the regular EMA_WEIGHT as the alpha, fast, responsive user experience
        {
            smoothAltitudeReading = newAltitudeReading * EMA_WEIGHT + ((1 - EMA_WEIGHT) * smoothAltitudeReading);
        }

        // if values exceed the encoder scale, fix them (for example: 32002 becomes 00002)
        if (smoothAzimuthReading > STEPS_IN_FULL_CIRCLE)
        {
            smoothAzimuthReading -= STEPS_IN_FULL_CIRCLE;
        }
        if (smoothAltitudeReading > STEPS_IN_FULL_CIRCLE)
        {
            smoothAltitudeReading -= STEPS_IN_FULL_CIRCLE;
        }
    }
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARS AND INIT

// Scope Struct
struct Scope
{
    struct ln_date date;
    struct ln_zonedate localDate;
    struct ln_equ_posn equPos;
    struct ln_hrz_posn hrzPos;
    struct ln_lnlat_posn lnLatPos;

    long double JD;
    double LST, GMST, GAST;

    float observationAlt;
};

// Star Structs
struct Object
{
    struct ln_equ_posn equPos;
    struct ln_equ_posn propMotion;
    struct ln_hrz_posn hrzPos;
    struct ln_rst_time rstTime;
    struct ln_zonedate rise, set;

    String name;
    String type;
    String constellation;
    String description;
    String size;
    String mag;
    String riseSetStr;
};
//

boolean CORRECT_REFRACTION = false;
boolean CORRECT_PRECESSION_ETC = false;
boolean CORRECT_MOUNT_ERRS = false;
boolean ALIGN_READY = false;

const char *WIFI_AP_NAME = "Skywatcher-WiFi"; // Name of the WiFi access point this device will create for your tablet/phone to connect to.
const char *WIFI_PASS = "3mzbZSMNNx9d";

//
const String FirmwareDate = "16 07 18";
const String FirmwareNumber = "v0.1.1 DobDSC";
const String FirmwareName = "DobDSC";
const String FirmwareTime = "21:00:00";
//

const long double ONE_REV = PI * 2.0;
const long double THREE_QTR_REV = PI * 3.0 / 2.0;
const long double HALF_REV = PI;
const long double QTR_REV = PI / 2.0;
const long double HOUR_TO_RAD = PI / 12.0;
const long double MINUTE_TO_RAD = PI / 720.0;
const long double SEC_TO_RAD = PI / 43200;
const long double DEG_TO_REV = 1.0 / 360.0;
const long double ARCMIN_TO_RAD = PI / 10800.0;
const long double ARCSEC_TO_RAD = PI / 648000.0;
const long double TENTH_ARCSEC_TO_RAD = ARCSEC_TO_RAD / 10.0;

const int NUM_ALIGN_STARS = 100;

const float GEAR_RATIO = GEAR_LARGE / GEAR_SMALL;
const int STEPS_IN_FULL_CIRCLE = ENCODER_RES * GEAR_RATIO;
const double STEPS_TO_RAD = STEPS_IN_FULL_CIRCLE / ONE_REV;

uint8_t ALIGN_STEP = 1;   // Using this variable to count the allignment steps - 1: Synchronize, 2: Allign and centre, 3:....
uint8_t ALIGN_TYPE = 0;   // Variable to store the alignment type (0-Skip Alignment, 1-1 Star alignment, 2-2 Star alignment
uint8_t LOADED_OBJECTS = 0; // What Objects are loaded from SPIFFS 1 == alignment, 2 == messier, 3 == treasure
uint8_t LOCAL_TIME_H_OFFSET = 0;
uint8_t LOCAL_TIME_D_OFFSET = 0;
uint8_t GPS_ITERATIONS = 0;
uint8_t CURRENT_SCREEN = -1;
uint8_t OLD_MIN, OLD_DAY;
uint8_t NUM_ALIGNMENT_STARS = 2;

boolean IS_BT_MODE_ON = false;
boolean IS_WIFI_MODE_ON = false;
boolean IS_IN_OPERATION = false; // This variable becomes True when Main screen appears

// Default values to load when CANCEL button is hit on the GPS screen
float OBSERVATION_LONGITUDE = -0.01365997; // (-0.01365997* - Home)
float OBSERVATION_LATTITUDE = 51.18078754; // (51.18078754* - Home)
float OBSERVATION_ALTITUDE = 52.00;        // Lingfield, UK
double ATMO_PRESS = 1010;                  // Default 1010 mBar atmospheric pressure
double ATMO_TEMP = 10;                     // Default 10C air temperature

unsigned int AZ_STEPS, ALT_STEPS; // Current position of the decoders in Steps! - when movement occures, values are changed accordingly

unsigned long UPDATE_LAST, UPDATE_TIME, UPDATE_CLOCK;
unsigned long LAST_MEASUREMENT = 0;  // millisecond timestamp of last measurement, to measure only every XXX milliseconds
unsigned long SCOPE_POS_UPDATE = 30; // 30ms updated = 30FPS
unsigned long LAST_POS_UPDATE = 0;

double Z1_ERR, Z2_ERR, Z3_ERR;
double ALT_MULTIPLIER, AZ_MULTIPLIER;

String OBJECTS[130];

int LAST_BUTTON, MESS_PAGER, CALD_PAGER, TREAS_PAGER, STARS_PAGER, LOAD_SELECTOR; // selector to show which LOADING mechanism is used: 1 - Messier, 2 - File, 3 - NGCs
int W_DATE_TIME[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                       // array to store date - as updated from updater screen - Wishing_Date
int DATE_ENTRY_POS = 0;
int SUMMER_TIME = 0;
int16_t L_TEXT, D_TEXT, BTN_L_BORDER, BTN_BLK_TEXT, TITLE_TEXT_BG, TITLE_TEXT, MSG_BOX_BG, MSG_BOX_TEXT; // defines string constants for the clor - Depending on the DAY/NIGHT modes

double W;
double Q[4][4];
double V[4][4];
double R[4][4];
double X[4][4];
double Y[4][4];

String BT_COMMAND;
int BT_COMMAND_STR;
boolean BT_PRECISION = true;

Scope SCOPE;
Object CURRENT_ALIGN_STAR;
Object ALIGNMENT_STARS[3];
Object CURRENT_OBJECT;

RtcDateTime NOW;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INIT HARDWARE
WiFiServer server(4030); // 4030 is the default port Skysafari uses for WiFi connection to telescopes
WiFiClient remoteClient; // represents the connection to the remote app (Skysafari)

// BLE Client
BluetoothSerial SerialBT;

// Init RTC Lib for Clock
RtcDS3231<TwoWire> rtc(Wire);

// Enable GPS Module at HardwareSerial 2
TinyGPSPlus gps;

// Enable Encoder
// CCW
//Encoder azEnc(12, 13);
// CW
Encoder azEnc(13, 12);

// Enable IMU for Altitude measurement
IMU imu;

// Enable TFT+Touch
TFT_eSPI tft = TFT_eSPI();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS

// Mathematical Utility Functions

// Haversine function
double hav(double angle)
{
    double ang = ln_deg_to_rad(angle);
    return ln_rad_to_deg(0.5 * (1 - cos(ang)));
}

// Find encoder value in steps 0-STEPS_IN_FULL_CIRCLE, rolling over at either end
int encToSteps(int x)
{
    int validatedSteps = x % STEPS_IN_FULL_CIRCLE;
    if (validatedSteps < 0)
        validatedSteps += STEPS_IN_FULL_CIRCLE;
    return validatedSteps;
    //return x - floor(x / STEPS_IN_FULL_CIRCLE) * STEPS_IN_FULL_CIRCLE;
}

// bring radian within range of 0 to one revolution (360 degrees or 2 * PI)
double validRev(double rad)
{
    double validatedRad = fmod(rad, ONE_REV);
    if (validatedRad < 0.0)
        validatedRad += ONE_REV;
    return validatedRad;
}

// bring radian within range of -half revolution to half revolution (-180 to 180 degrees or -PI to PI)
double validHalfRev(double rad)
{
    double validatedRad = validRev(rad);
    if (validatedRad > HALF_REV)
        validatedRad -= ONE_REV;
    return validatedRad;
}

// bring Declination within range of -90 to +90 degrees
double validDec(double rad)
{
    double correctedRad, validatedRad;
    validatedRad = validHalfRev(rad);
    if (validatedRad > QTR_REV)
        correctedRad = HALF_REV - validatedRad; // > 90 degrees: reverse scale
    else if (validatedRad >= -QTR_REV)
        correctedRad = validatedRad; // between -90 and 90 degrees: don't change
    else
        correctedRad = -HALF_REV - validatedRad; // < -90 degrees: reverse negative scale
    return correctedRad;
}

double reverseRev(double rad)
{
    return ONE_REV - rad;
}

double calcDecIsFlipped(double rad)
{
    return rad > QTR_REV || rad < -QTR_REV;
}

double flipRA(double rad)
{
    return validRev(rad + HALF_REV);
}

double flipDEC(double rad)
{
    double halfRevDec, flippedDec;
    halfRevDec = validHalfRev(rad);
    if (halfRevDec >= 0)
        flippedDec = HALF_REV - halfRevDec;
    else
        flippedDec = -HALF_REV - halfRevDec;
    return flippedDec;
}

// Keep the GPS sensor "fed" until we find the data.
static void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (Serial2.available())
            gps.encode(Serial2.read());
    } while (millis() - start < ms);
}

String padding(String str, uint8_t length)
{
    while (str.length() < length)
    {
        str = "0" + str;
    }
    return str;
}

String rad2hms(double rad, boolean highPrecision, boolean withUnits)
{
    double degs;

    rad = ln_range_radians(rad);
    degs = ln_rad_to_deg(rad);

    return deg2hms(degs, highPrecision, withUnits);
}

String rad2dms(double rad, boolean highPrecision, boolean asAzimuth)
{
    double degs = ln_rad_to_deg(rad);
    return deg2dms(degs, highPrecision, asAzimuth);
}

String deg2hms(double deg, boolean highPrecision, boolean withUnits)
{
    struct ln_hms lnHMS;
    ln_deg_to_hms(deg, &lnHMS);

    if (highPrecision)
    {
        if (withUnits)
        {
            return padding((String) int(lnHMS.hours), (uint8_t)2) + "h" +
                   padding((String) int(lnHMS.minutes), (uint8_t)2) + "m" +
                   padding((String) int(lnHMS.seconds), (uint8_t)2) + "s";
        }
        else
        {
            return padding((String) int(lnHMS.hours), (uint8_t)2) + ":" +
                   padding((String) int(lnHMS.minutes), (uint8_t)2) + ":" +
                   padding((String) int(lnHMS.seconds), (uint8_t)2);
        }
    }
    else
    {
        if (withUnits)
        {
            return padding((String) int(lnHMS.hours), (uint8_t)2) + "h" +
                   padding((String) int(lnHMS.minutes), (uint8_t)2) + "." +
                   (String) int(lnHMS.seconds / 60.0) + "m";
        }
        return padding((String) int(lnHMS.hours), (uint8_t)2) + ":" +
               padding((String) int(lnHMS.minutes), (uint8_t)2) + "." +
               (String) int(lnHMS.seconds / 60.0);
    }
}

String deg2dms(double deg, boolean highPrecision, boolean asAzimuth)
{
    struct ln_dms lnDMS;
    ln_deg_to_dms(deg, &lnDMS);
    String sign = "";

    if (!asAzimuth)
    {
        sign = "+";
        if (lnDMS.neg == 1)
            sign = "-";
    }

    if (highPrecision)
    {
        return sign + padding((String) int(lnDMS.degrees), (uint8_t)2) + "*" +
               padding((String) int(lnDMS.minutes), (uint8_t)2) + "'" +
               padding((String) int(lnDMS.seconds), (uint8_t)2);
    }
    else
    {
        return sign + padding((String) int(lnDMS.degrees), (uint8_t)2) + "*" +
               padding((String) int(lnDMS.minutes), (uint8_t)2);
    }
}

String hrs2hms(double decTime, boolean highPrecision, boolean withUnits)
{
    return deg2hms(decTime * 15.0, highPrecision, withUnits);
}

void getLocalDate(double jd, struct ln_zonedate *zonedate)
{
    struct ln_date date;
    long gmtoff = 0;
    // Rough calculation of the timezone delay based on longitude
    int timeZone = round(SCOPE.lnLatPos.lng * 4.0 / 60.0);

    gmtoff += (timeZone * 3600);

    if (SUMMER_TIME)
        gmtoff += 3600;

    ln_get_date(jd, &date);
    ln_date_to_zonedate(&date, zonedate, gmtoff);
}

void setScopeDateTime()
{
    NOW = rtc.GetDateTime();
    SCOPE.date.years = NOW.Year();
    SCOPE.date.months = NOW.Month();
    SCOPE.date.days = NOW.Day();
    SCOPE.date.hours = NOW.Hour();
    SCOPE.date.minutes = NOW.Minute();
    SCOPE.date.seconds = NOW.Second();
}

void readEncoders()
{
    int newAzPos = azEnc.read();
    int newAltPos = imu.smoothAltitudeReading;
    if (newAzPos != AZ_STEPS)
    {
        AZ_STEPS = encToSteps(newAzPos);
        //AZ_STEPS = newAzPos;
    }

    if (newAltPos != ALT_STEPS)
    {
        //ALT_STEPS = encToSteps(newAltPos);
        ALT_STEPS = newAltPos;
    }
    // #ifdef SERIAL_DEBUG
    //     if ((millis() - UPDATE_LAST) > 1000)
    //     {
    //         Serial.print("AZ ENCODER: ");
    //         Serial.print(AZ_STEPS);
    //         Serial.print(" ALT ENCODER STEPS: ");
    //         Serial.print(ALT_STEPS);
    //         Serial.println("");
    //     }
    // #endif
}

void currentElevAngle()
{
    readEncoders();
#ifdef SERIAL_DEBUG
    AZ_STEPS = AZ_STEPS + 1;
#endif
    SCOPE.hrzPos.alt = ln_rad_to_deg(ALT_MULTIPLIER * ALT_STEPS);
    SCOPE.hrzPos.az = ln_rad_to_deg(AZ_MULTIPLIER * AZ_STEPS);
}

void telescopeRaDec()
{
    scopeToEquatorial(&SCOPE.hrzPos, &SCOPE.lnLatPos, SCOPE.JD, &SCOPE.equPos);
}

void objectAltAz()
{
    equatorialToScope(&CURRENT_OBJECT.equPos, &SCOPE.lnLatPos, SCOPE.JD, &CURRENT_OBJECT.hrzPos);
}

void calculateLST()
{
    setScopeDateTime();
    SCOPE.JD = ln_get_julian_day(&SCOPE.date);
    SCOPE.GMST = ln_get_mean_sidereal_time(SCOPE.JD);
    SCOPE.GAST = ln_get_apparent_sidereal_time(SCOPE.JD);
    SCOPE.LST = SCOPE.GAST + (SCOPE.lnLatPos.lng / 15.0); // Convert Longitude degrees to hours
#ifdef SERIAL_DEBUG
    Serial.print("calculateLST() - TIME: ");
    Serial.printf("%i/%i/%i - %i:%i:%2.0f", SCOPE.date.years, SCOPE.date.months, SCOPE.date.days, SCOPE.date.hours, SCOPE.date.minutes, SCOPE.date.seconds);
    Serial.print(" JD: ");
    Serial.print((double)SCOPE.JD);
    Serial.print(" GMST: ");
    Serial.print(hrs2hms(SCOPE.GMST, true, false));
    Serial.print(" GAST: ");
    Serial.print(hrs2hms(SCOPE.GAST, true, false));
    Serial.print(" LST: ");
    Serial.print(hrs2hms(SCOPE.LST, true, false));
    Serial.println("");
#endif
}

double convertPrimaryAxis(double fromPri, double fromSec, double toSec, double lat)
{
    if (lat == QTR_REV)
        return validRev(fromPri + HALF_REV);

    double cosToPri = (sin(fromSec) - sin(lat) * sin(toSec)) / (cos(lat) * cos(toSec));
    if (cosToPri < -1)
        cosToPri = -1;
    else if (cosToPri > 1)
        cosToPri = 1;
    double toPri = acos(cosToPri);
    if (sin(fromPri) > 0)
        toPri = reverseRev(toPri);
    return toPri;
}

double convertSecondaryAxis(double fromPri, double fromSec, double lat)
{
    double sinToSec = sin(fromSec) * sin(lat) + cos(fromSec) * cos(lat) * cos(fromPri);
    return asin(sinToSec);
}

void getAltAzTrig(struct ln_equ_posn *object, struct ln_lnlat_posn *observer, double sidereal, struct ln_hrz_posn *position)
{
    long double ra, dec, ha, absLat, lat, lng, alt, az;

    // Change sidereal from hours to radians
    sidereal *= HOUR_TO_RAD;

    lat = ln_deg_to_rad(observer->lat);
    lng = ln_deg_to_rad(observer->lng);
    absLat = fabs(lat);

    dec = ln_deg_to_rad(object->dec);
    ra = ln_deg_to_rad(object->ra);

    // Flip for Southern Hemisphere
    if (lat < 0)
        dec = -dec;
    if (dec > QTR_REV || dec < -QTR_REV)
    {
        dec = HALF_REV - dec;
        ra = flipRA(ra);
    }

    // Calculate Hour Angle from LST - RA
    ha = (sidereal + lng) - ra;

    alt = convertSecondaryAxis(ha, dec, absLat);
    az = convertPrimaryAxis(ha, dec, alt, absLat);

    // Flip for Southern Hemisphere
    if (lat < 0)
        az = reverseRev(az);
    position->alt = ln_rad_to_deg(alt);
    position->az = ln_rad_to_deg(az);
#ifdef SERIAL_DEBUG
    Serial.print("getAltAzTrig() - RA: ");
    Serial.print(rad2hms(ra, true, true));
    Serial.print(" DEC: ");
    Serial.print(rad2dms(dec, true, false));
    Serial.print(" HA: ");
    Serial.print(rad2hms(ha, true, true));
    Serial.print(" ---> AZ: ");
    Serial.print(rad2dms(az, true, true));
    Serial.print(" ALT: ");
    Serial.print(rad2dms(alt, true, true));
    Serial.println("");
#endif
}

void getEquatTrig(struct ln_hrz_posn *object, struct ln_lnlat_posn *observer, double jd, struct ln_equ_posn *position)
{
    long double az, alt, absLat, lat, lng, ha, ra, dec, sidereal;

    // Calc apparent LST and convert to radians
    sidereal = ln_get_apparent_sidereal_time(jd);
    sidereal *= HOUR_TO_RAD;

    lat = ln_deg_to_rad(observer->lat);
    lng = ln_deg_to_rad(observer->lng);
    absLat = fabs(lat);

    az = ln_deg_to_rad(object->az);
    alt = ln_deg_to_rad(object->alt);

    // Flip for Southern Hemisphere
    if (alt > QTR_REV || alt < -QTR_REV)
    {
        alt = HALF_REV - alt;
        az = flipRA(az);
    }
    if (lat < 0)
        az = reverseRev(az);

    dec = convertSecondaryAxis(az, alt, absLat);
    ha = convertPrimaryAxis(az, alt, dec, absLat);
    ra = validRev(sidereal - ha);

    position->ra = ra;
    if (lat < 0)
        position->dec = -dec;
    else
        position->dec = dec;
#ifdef SERIAL_DEBUG
    Serial.print("getEquatTrig() - AZ: ");
    Serial.print(rad2dms(az, true, true));
    Serial.print(" ALT: ");
    Serial.print(rad2dms(alt, true, false));
    Serial.print(" ---> RA: ");
    Serial.print(rad2hms(ra, true, true));
    Serial.print(" HA: ");
    Serial.print(rad2hms(ha, true, true));
    Serial.print(" DEC: ");
    Serial.print(rad2dms(dec, true, false));
    Serial.println("");
#endif
}

void selectObject(int index, int objects)
{
    if (objects == 1)
    {
        // Selected Star Object
        if (LOADED_OBJECTS != 1)
        {
            loadStarsFromSPIFFS("/alignment.csv");
            LOADED_OBJECTS = 1;
        }
        processObject(index, CURRENT_OBJECT);
        objectAltAz();
    }
    else if (objects == 2)
    {
        // I've selected a Messier Object
        if (LOADED_OBJECTS != 2)
        {
            loadStarsFromSPIFFS("/messier.csv");
            LOADED_OBJECTS = 2;
        }
        processObject(index, CURRENT_OBJECT);
        objectAltAz();
    }
    else if (objects == 3)
    {
        if (LOADED_OBJECTS != 3)
        {
            loadStarsFromSPIFFS("/caldwell.csv");
            LOADED_OBJECTS = 3;
        }
        processObject(index, CURRENT_OBJECT);
        objectAltAz();
    }
    else if (objects == 4)
    {
        // I've selected a Treasure Object
        if (LOADED_OBJECTS != 4)
        {
            loadStarsFromSPIFFS("/treasure.csv");
            LOADED_OBJECTS = 4;
        }
        processObject(index, CURRENT_OBJECT);
        objectAltAz();
    }
}

bool isSummerTime()
{
    bool SUMMER_TIME = false;
    int month = NOW.Month();
    int day = NOW.Day();
    int dayOfWeek = NOW.DayOfWeek();
    int hour = NOW.Hour();

#ifdef SERIAL_DEBUG
    Serial.print("CURRENT DATE/TIME: ");
    Serial.print(day);
    Serial.print("/");
    Serial.print(month);
    Serial.print("/");
    Serial.print(NOW.Year());
    Serial.print("  weekday: ");
    Serial.print(dayOfWeek);
    Serial.print(" ");
    Serial.print(hour);
    Serial.print(":");
    Serial.print(NOW.Minute());
    Serial.println("");
#endif

    // If I'm in October
    if (month == 10)
    {
        // If it's Sunday
        if (weekday() == 1)
        {
            if (day + 7 > 31 && hour >= 3)
                SUMMER_TIME = false;
            else
                SUMMER_TIME = true;
        }
        else
        {
            // If last sunday has passed
            if ((day + 7 - (dayOfWeek - 1)) > 31)
                SUMMER_TIME = false;
            else
                SUMMER_TIME = true;
        }
    }
    // If I'm in March
    else if (month == 3)
    {
        // If It's Sunday
        if (dayOfWeek == 1)
        {
            // If It is the Last Sunday
            if ((day + 7) > 31 && hour >= 2)
                SUMMER_TIME = true;
            else
                SUMMER_TIME = false;
        }
        else
        {
            // If it's not Sunday,but the last has already passed
            if ((day + 7 - (dayOfWeek - 1)) > 31)
                SUMMER_TIME = true;
            else
                SUMMER_TIME = false;
        }
    }
    // If the Day Light Saving Time
    else if (month >= 4 && month <= 9)
        SUMMER_TIME = true;
    // If we are in the Winter Time Period
    else if ((month >= 1 && month <= 2) || (month >= 11 && month <= 12))
        SUMMER_TIME = false;

    return SUMMER_TIME;
}

void touchCalibrate()
{
    uint16_t calData[5];
    uint8_t calDataOK = 0;

    // check file system exists
    if (!SPIFFS.begin())
    {
#ifdef SERIAL_DEBUG
        Serial.println("Formating file system");
#endif
        SPIFFS.format();
        SPIFFS.begin();
    }

    // check if calibration file exists and size is correct
    if (SPIFFS.exists(CALIBRATION_FILE))
    {
        if (REPEAT_CAL)
        {
            // Delete if we want to re-calibrate
            SPIFFS.remove(CALIBRATION_FILE);
        }
        else
        {
            File f = SPIFFS.open(CALIBRATION_FILE, "r");
            if (f)
            {
                if (f.readBytes((char *)calData, 14) == 14)
                    calDataOK = 1;
                f.close();
            }
        }
    }

    if (calDataOK && !REPEAT_CAL)
    {
        // calibration data valid
        tft.setTouch(calData);
    }
    else
    {
        // data not valid so recalibrate
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(20, 0);
        tft.setTextFont(2);
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

        tft.println("Touch corners as indicated");

        tft.setTextFont(1);
        tft.println();

        if (REPEAT_CAL)
        {
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.println("Set REPEAT_CAL to false to stop this running again!");
        }

        tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.println("Calibration complete!");

        // store data
        File f = SPIFFS.open(CALIBRATION_FILE, "w");
        if (f)
        {
            f.write((const unsigned char *)calData, 14);
            f.close();
        }
    }
}

void loadGPSFromSPIFFS()
{
    char in_char;
    String items = "";
    int i = 0;
    File dataFile = SPIFFS.open("/GPSData", "r");
    if (dataFile)
    {
#ifdef SERIAL_DEBUG
        Serial.print("Read GPS data from file...");
        Serial.println("");
#endif
        while (dataFile.available())
        {
            in_char = dataFile.read();
            items += in_char;
            if (in_char == '\n')
            {
                if (i == 0)
                {
                    OBSERVATION_LATTITUDE = items.toFloat();
#ifdef SERIAL_DEBUG
                    Serial.print("Lat: ");
                    Serial.print(OBSERVATION_LATTITUDE);
#endif
                }
                else if (i == 1)
                {
                    OBSERVATION_LONGITUDE = items.toFloat();
#ifdef SERIAL_DEBUG
                    Serial.print(" Lon: ");
                    Serial.print(OBSERVATION_LONGITUDE);
#endif
                }
                else if (i == 2)
                {
                    OBSERVATION_ALTITUDE = items.toFloat();
#ifdef SERIAL_DEBUG
                    Serial.print(" Alt: ");
                    Serial.print(OBSERVATION_ALTITUDE);
#endif
                }
                i += 1;
                items = "";
            }
        }
#ifdef SERIAL_DEBUG
        Serial.println("");
#endif
    }
}

void writeGPSToSPIFFS()
{
    String items = "";
    File dataFile = SPIFFS.open("/GPSData", "w");
    if (dataFile)
    {
        dataFile.println(OBSERVATION_LATTITUDE);
        dataFile.println(OBSERVATION_LONGITUDE);
        dataFile.println(OBSERVATION_ALTITUDE);
        dataFile.close();
    }
#ifdef SERIAL_DEBUG
    Serial.print("Writing GPS data to file...");
    Serial.println("");
#endif
}

void loadStarsFromSPIFFS(String filename)
{
    char in_char;
    String items = "";
    int j = 0, k = 0;
    File dataFile = SPIFFS.open(filename);
    if (dataFile)
    {
#ifdef SERIAL_DEBUG
        Serial.print("Opened ");
        Serial.print(dataFile.name());
        Serial.print(" from SPIFFS");
        Serial.println("");
#endif
        while (dataFile.available())
        {
            in_char = dataFile.read();
            items += in_char;
            if (in_char == '\n')
            {
                // Skip Header Row
                if (k == 0)
                {
                    k++;
                    items = "";
                }
                else
                {
                    k++;
                    OBJECTS[j] = items;
                    j++;
                    items = "";
                }
            }
        }
    }
    else
    {
#ifdef SERIAL_DEBUG
        Serial.println("Failed to open ");
        Serial.println(dataFile.name());
#endif
    }
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.print(file.name());
            Serial.println("");
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.print(file.size());
            Serial.println("");
        }
        file = root.openNextFile();
    }
}

void autoSelectAlignmentStars()
{
#ifdef SERIAL_DEBUG
    Serial.println("autoSelectAlignmentStars()");
#endif
    // Alignment star counter.
    if (LOADED_OBJECTS != 1)
    {
        loadStarsFromSPIFFS("/alignment.csv");
        LOADED_OBJECTS = 1;
    }

    // Load each star from the alignment.csv database into the CURRENT_ALIGN_STAR object
    int n = 0;
    for (int i = 0; i < NUM_ALIGN_STARS; i++)
    {
        processAlignmentStar(i, CURRENT_ALIGN_STAR);
        // If catalogue star is higher than 25 degrees above the horizon.
        if (CURRENT_ALIGN_STAR.hrzPos.alt > 25.0)
        {
            // Copy catalogue star to alignment star.
            ALIGNMENT_STARS[n] = CURRENT_ALIGN_STAR;
#ifdef SERIAL_DEBUG
            Serial.print("Star number ");
            Serial.print(n);
            Serial.println("");
            Serial.print(ALIGNMENT_STARS[n].name);
            Serial.print(" - RA: ");
            Serial.print(deg2hms(ALIGNMENT_STARS[n].equPos.ra, true, true));
            Serial.print(" DEC: ");
            Serial.print(deg2dms(ALIGNMENT_STARS[n].equPos.dec, true, false));
            Serial.print(" ALT: ");
            Serial.print(deg2dms(ALIGNMENT_STARS[n].hrzPos.alt, true, false));
            Serial.print(" AZ: ");
            Serial.print(deg2dms(ALIGNMENT_STARS[n].hrzPos.az, true, true));
            Serial.println("");
#endif
            n++;
            // If all alignment stars have been selected, return.
            if (n >= NUM_ALIGNMENT_STARS)
            {
                return;
            }
        }
    }
}

void processAlignmentStar(int index, Object &star)
{
    int i1 = OBJECTS[index].indexOf(',');
    int i2 = OBJECTS[index].indexOf(',', i1 + 1);
    int i3 = OBJECTS[index].indexOf(',', i2 + 1);
    int i4 = OBJECTS[index].indexOf(',', i3 + 1);
    int i5 = OBJECTS[index].indexOf(',', i4 + 1);
    int i6 = OBJECTS[index].indexOf(',', i5 + 1);
    int i7 = OBJECTS[index].indexOf(',', i6 + 1);
    int i8 = OBJECTS[index].indexOf(',', i7 + 1);
    int i9 = OBJECTS[index].indexOf(',', i8 + 1);

    star.name = OBJECTS[index].substring(0, i1);

    String ra = OBJECTS[index].substring(i1 + 1, i2);
    String dec = OBJECTS[index].substring(i2 + 1, i3);

    star.constellation = OBJECTS[index].substring(i3 + 1, i4);
    star.type = OBJECTS[index].substring(i4 + 1, i5);
    star.mag = OBJECTS[index].substring(i5 + 1, i6);
    star.size = OBJECTS[index].substring(i6 + 1, i7);
    star.description = OBJECTS[index].substring(i7 + 1, i8);

    String pmRa = OBJECTS[index].substring(i8 + 1, i9);
    String pmDec = OBJECTS[index].substring(i9 + 1, OBJECTS[index].length() - 1);

    char pmRaSign = pmRa[0];
    char pmDecSign = pmDec[0];
    long double pmRaDeg = pmRa.substring(1, pmRa.length() - 1).toFloat();
    long double pmDecDeg = pmDec.substring(1, pmDec.length() - 1).toFloat();
    // Check sign of proper motion corrections, multiply accordingly
    pmRaSign == '+' ? pmRaDeg *= 1.0 : pmRaDeg *= -1.0;
    pmDecSign == '+' ? pmDecDeg *= 1.0 : pmDecDeg *= -1.0;
    // Convert milliarcseconds into degrees (1000 millis in 1", 3600" in 1deg)
    pmRaDeg /= (1000.0 * 3600.0);
    pmDecDeg /= (1000.0 * 3600.0);

    star.propMotion.ra = pmRaDeg;
    star.propMotion.dec = pmDecDeg;

    unsigned short ra_h = ra.substring(0, ra.indexOf('h')).toFloat();
    unsigned short ra_m = ra.substring(ra.indexOf('h') + 1, ra.indexOf('m')).toFloat();
    unsigned short ra_s = ra.substring(ra.indexOf('m') + 1, ra.length()).toFloat();

    char sign = dec[0];
    unsigned short dec_d = dec.substring(1, dec.indexOf('°')).toFloat();
    unsigned short dec_m = dec.substring(dec.indexOf('°') + 1, dec.indexOf('\'')).toFloat();
    unsigned short dec_s = dec.substring(dec.indexOf('\'') + 1, dec.length()).toFloat();

    unsigned short dec_neg = (sign == '+' ? 0 : 1);
    struct lnh_equ_posn hEquPos;

    hEquPos.ra.hours = ra_h;
    hEquPos.ra.minutes = ra_m;
    hEquPos.ra.seconds = ra_s;
    hEquPos.dec.neg = dec_neg;
    hEquPos.dec.degrees = dec_d;
    hEquPos.dec.minutes = dec_m;
    hEquPos.dec.seconds = dec_s;

    ln_hequ_to_equ(&hEquPos, &star.equPos);

#ifdef SERIAL_DEBUG
    Serial.print("processAlignmentStar() ");
    Serial.print(star.name);
    Serial.print(" RA: ");
    Serial.print(deg2hms(star.equPos.ra, true, true));
    Serial.print(" DEC: ");
    Serial.print(deg2dms(star.equPos.dec, true, false));
    Serial.println("");
#endif

    // Correct for astronomical proper motion, aberration, precession and nutation
    if (CORRECT_PRECESSION_ETC)
    {
        ln_get_apparent_posn(&star.equPos, &star.propMotion, SCOPE.JD, &star.equPos);
#ifdef SERIAL_DEBUG
        Serial.print("Added Corrections - RA: ");
        Serial.print(deg2hms(star.equPos.ra, true, true));
        Serial.print(" DEC: ");
        Serial.print(deg2dms(star.equPos.dec, true, false));
        Serial.println("");
#endif
    }

    // Calculate Rise/Set times
    int rstStatus = ln_get_object_rst(SCOPE.JD, &SCOPE.lnLatPos, &star.equPos, &star.rstTime);
    if (rstStatus == 0)
    {
        getLocalDate(star.rstTime.rise, &star.rise);
        getLocalDate(star.rstTime.set, &star.set);
        star.riseSetStr = "RISE: " + getTimeString(star.rise);
        star.riseSetStr += "       SET: " + getTimeString(star.set);
    }
    else if (rstStatus == 1)
    {
        star.riseSetStr = "ALWAYS VISIBLE";
    }
    else
    {
        star.riseSetStr = "ALWAYS BELOW HORIZON";
    }

    ln_get_hrz_from_equ(&star.equPos, &SCOPE.lnLatPos, SCOPE.JD, &star.hrzPos);
    // getAltAzTrig(&star.equPos, &SCOPE.lnLatPos, SCOPE.LST, &star.hrzPos);
    double azTmp = ln_deg_to_rad(star.hrzPos.az);
    // azTmp = validRev(azTmp + HALF_REV);
    azTmp = flipRA(azTmp);
    star.hrzPos.az = ln_rad_to_deg(azTmp);
}

void processObject(int index, Object &object)
{
    int i1 = OBJECTS[index].indexOf(',');
    int i2 = OBJECTS[index].indexOf(',', i1 + 1);
    int i3 = OBJECTS[index].indexOf(',', i2 + 1);
    int i4 = OBJECTS[index].indexOf(',', i3 + 1);
    int i5 = OBJECTS[index].indexOf(',', i4 + 1);
    int i6 = OBJECTS[index].indexOf(',', i5 + 1);
    int i7 = OBJECTS[index].indexOf(',', i6 + 1);

    object.name = OBJECTS[index].substring(0, i1);

    String ra = OBJECTS[index].substring(i1 + 1, i2);
    String dec = OBJECTS[index].substring(i2 + 1, i3);

    object.constellation = OBJECTS[index].substring(i3 + 1, i4);
    object.type = OBJECTS[index].substring(i4 + 1, i5);
    object.mag = OBJECTS[index].substring(i5 + 1, i6);
    object.size = OBJECTS[index].substring(i6 + 1, i7);
    if (LOADED_OBJECTS == 1)
        object.description = OBJECTS[index].substring(i7 + 1, OBJECTS[index].indexOf(',', i7 + 1));
    else
        object.description = OBJECTS[index].substring(i7 + 1, OBJECTS[index].length() - 1);

    unsigned short ra_h = ra.substring(0, ra.indexOf('h')).toFloat();
    unsigned short ra_m = ra.substring(ra.indexOf('h') + 1, ra.indexOf('m')).toFloat();
    unsigned short ra_s = ra.substring(ra.indexOf('m') + 1, ra.length()).toFloat();

    char sign = dec[0];
    unsigned short dec_d = dec.substring(1, dec.indexOf('°')).toFloat();
    unsigned short dec_m = dec.substring(dec.indexOf('°') + 1, dec.indexOf('\'')).toFloat();
    unsigned short dec_s = dec.substring(dec.indexOf('\'') + 1, dec.length()).toFloat();

    unsigned short dec_neg = (sign == '+' ? 0 : 1);
    struct lnh_equ_posn hEquPos;

    hEquPos.ra.hours = ra_h;
    hEquPos.ra.minutes = ra_m;
    hEquPos.ra.seconds = ra_s;
    hEquPos.dec.neg = dec_neg;
    hEquPos.dec.degrees = dec_d;
    hEquPos.dec.minutes = dec_m;
    hEquPos.dec.seconds = dec_s;

    ln_hequ_to_equ(&hEquPos, &object.equPos);

#ifdef SERIAL_DEBUG
    Serial.print("processObject() ");
    Serial.print(object.name);
    Serial.print(" - RA: ");
    Serial.print(deg2hms(object.equPos.ra, true, true));
    Serial.print(" DEC: ");
    Serial.print(deg2dms(object.equPos.dec, true, false));
    Serial.println("");
#endif
    // Correct for astronomical proper motion, aberration, precession and nutation
    if (CORRECT_PRECESSION_ETC)
    {
        ln_get_apparent_posn(&object.equPos, &object.propMotion, SCOPE.JD, &object.equPos);
#ifdef SERIAL_DEBUG
        Serial.print("Added Corrections - RA: ");
        Serial.print(deg2hms(object.equPos.ra, true, true));
        Serial.print(" DEC: ");
        Serial.print(deg2dms(object.equPos.dec, true, false));
        Serial.println("");
#endif
    }

    // Calculate Rise/Set times
    int rstStatus = ln_get_object_rst(SCOPE.JD, &SCOPE.lnLatPos, &object.equPos, &object.rstTime);
    if (rstStatus == 0)
    {
        getLocalDate(object.rstTime.rise, &object.rise);
        getLocalDate(object.rstTime.set, &object.set);
        object.riseSetStr = "RISE: " + getTimeString(object.rise);
        object.riseSetStr += "       SET: " + getTimeString(object.set);
    }
    else if (rstStatus == 1)
    {
        object.riseSetStr = "ALWAYS VISIBLE";
    }
    else
    {
        object.riseSetStr = "ALWAYS BELOW HORIZON";
    }
}

void zeroArrays()
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            Q[i][j] = 0;
            V[i][j] = 0;
            R[i][j] = 0;
            X[i][j] = 0;
            Y[i][j] = 0;
        }
    }
}

void initAlign()
{
    Z1_ERR = 0; // Mount error angle between horizontal axis and vertical axis
    Z2_ERR = 0; // Mount error angle between vertical axis and telescope optical axis
    Z3_ERR = 0; // Mount error angle, zero point shift (azimuth axis of rotation vs. instrument altitude angle)
    ALIGN_READY = false;
    zeroArrays();
}

boolean isReady()
{
    return ALIGN_READY;
}

void scopeToEquatorial(struct ln_hrz_posn *hor, struct ln_lnlat_posn *obs, double JD, struct ln_equ_posn *pos)
{
    long double ha, sidereal, ra, dec, az, alt, lng;

    // Correct for atmospheric refraction before doing anything else
    alt = hor->alt;
    if (CORRECT_REFRACTION)
    {
        double deltaAlt = ln_get_refraction_adj(alt, ATMO_PRESS, ATMO_TEMP);
        alt -= deltaAlt;

#ifdef SERIAL_DEBUG
#ifdef DEBUG_REFRACTION
        Serial.print("calREFRACTIONFromApparent - ");
        Serial.print(" Refract Amount(ARCMIN): ");
        Serial.print(deltaAlt);
        Serial.println("");
#endif
#endif
    }

    az = ln_deg_to_rad(hor->az);
    alt = ln_deg_to_rad(alt);
    lng = ln_deg_to_rad(obs->lng);

    // double pri = reverseRev(az);
    double pri = flipRA(az);
    double sec = alt + Z3_ERR;
    subroutine_1(pri, sec);
    X[1][1] = Y[1][0];
    X[2][1] = Y[2][0];
    X[3][1] = Y[3][0];
    Y[1][1] = 0;
    Y[2][1] = 0;
    Y[3][1] = 0;

    for (int i = 1; i <= 3; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            Y[i][1] += (Q[i][j] * X[j][1]);
        }
    }

    angleSubroutine(&ha, &dec);

    sidereal = ln_get_apparent_sidereal_time(JD);
    sidereal *= 2.0 * PI / 24.0;
    ra = ln_rad_to_deg(sidereal - ha + lng);

    pos->ra = ln_range_degrees(ra);
    pos->dec = ln_rad_to_deg(dec);
#ifdef SERIAL_DEBUG
    if ((millis() - UPDATE_LAST) > 5000)
    {
        Serial.print("scopeToEquatorial() - AZ: ");
        Serial.print(deg2dms(hor->az, true, true));
        Serial.print(" ALT: ");
        Serial.print(deg2dms(hor->alt, true, false));
        Serial.print(" ---> RA: ");
        Serial.print(deg2hms(pos->ra, true, true));
        Serial.print(" HA: ");
        Serial.print(deg2hms(ha, true, true));
        Serial.print(" DEC: ");
        Serial.print(deg2dms(pos->dec, true, false));
        Serial.println("");
    }
#endif
}

void equatorialToScope(struct ln_equ_posn *pos, struct ln_lnlat_posn *obs, double JD, struct ln_hrz_posn *hor)
{
    long double ha, ra, dec, az, alt;
    double sidereal = ln_get_mean_sidereal_time(JD);
    sidereal *= 2.0 * PI / 24.0; // Change sidereal time from hours to radians

    ra = ln_deg_to_rad(pos->ra);
    dec = ln_deg_to_rad(pos->dec);
    ha = ra - sidereal + ln_deg_to_rad(obs->lng); // HourAngle from ra - LST (backwards)

    X[1][1] = cos(dec) * cos(ha);
    X[2][1] = cos(dec) * sin(ha);
    X[3][1] = sin(dec);
    Y[1][1] = 0;
    Y[2][1] = 0;
    Y[3][1] = 0;

    for (int i = 1; i <= 3; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            Y[i][1] += R[i][j] * X[j][1];
        }
    }
    angleSubroutine(&az, &alt);
    subroutine_2(az, alt, 1);
    angleSubroutine(&az, &alt);

    // Bring alt back into degrees before adding refraction or mount error corrections
    alt = ln_rad_to_deg(alt);

//     if (CORRECT_REFRACTION)
//     {
//         double deltaAlt = ln_get_refraction_adj(alt, ATMO_PRESS, ATMO_TEMP);
//         alt += deltaAlt;
// #ifdef SERIAL_DEBUG
// #ifdef DEBUG_REFRACTION
//         Serial.print("calREFRACTIONFromTrue - ");
//         Serial.print(" Refract Amount(ARCMIN): ");
//         Serial.print(deltaAlt);
//         Serial.println("");
// #endif
// #endif
//     }

    hor->alt = alt - ln_rad_to_deg(Z3_ERR);
    // hor->az = ln_rad_to_deg(reverseRev(validRev(az)));
    hor->az = ln_rad_to_deg(flipRA(az));
}

void addStar(int starNum, int totalAlignStars, double JD, struct ln_lnlat_posn *obs, Object *star)
{
    long double ha, ra, dec, az, alt;
    double sidereal = ln_get_mean_sidereal_time(JD);
    sidereal *= 2.0 * PI / 24.0; // Change sidereal time from hours to radians

    ra = ln_deg_to_rad(star->equPos.ra);
    dec = ln_deg_to_rad(star->equPos.dec);
    az = ln_deg_to_rad(star->hrzPos.az);
    alt = ln_deg_to_rad(star->hrzPos.alt);

    ha = ra - sidereal + ln_deg_to_rad(obs->lng);
    // double ha = sidT - ra;
    double cosDec = cos(dec);
    double sinDec = sin(dec);
    double cosHA = cos(ha);
    double sinHA = sin(ha);

#ifdef SERIAL_DEBUG
    Serial.print("addStar() - RA: ");
    Serial.print(rad2hms(ra, true, true));
    Serial.print(" DEC: ");
    Serial.print(rad2dms(dec, true, false));
    Serial.print(" HA: ");
    Serial.print(rad2hms(ha, true, true));
    Serial.print(" ---> AZ: ");
    Serial.print(rad2dms(az, true, true));
    Serial.print(" ALT: ");
    Serial.print(rad2dms(alt, true, true));
    Serial.println("");
#endif

    X[1][starNum] = cosDec * cosHA;
    X[2][starNum] = cosDec * sinHA;
    X[3][starNum] = sinDec;

    // double pri = reverseRev(az);
    double pri = flipRA(az);
    double sec = alt + Z3_ERR;
    subroutine_1(pri, sec);
    Y[1][starNum] = Y[1][0];
    Y[2][starNum] = Y[2][0];
    Y[3][starNum] = Y[3][0];
}

void generateThirdStar()
{
    X[1][3] = X[2][1] * X[3][2] - X[3][1] * X[2][2];
    X[2][3] = X[3][1] * X[1][2] - X[1][1] * X[3][2];
    X[3][3] = X[1][1] * X[2][2] - X[2][1] * X[1][2];

    double a = sqrt(sq(X[1][3]) + sq(X[2][3]) + sq(X[3][3]));
    for (int i = 1; i <= 3; i++)
    {
        X[i][3] /= a;
    }

    Y[1][3] = Y[2][1] * Y[3][2] - Y[3][1] * Y[2][2];
    Y[2][3] = Y[3][1] * Y[1][2] - Y[1][1] * Y[3][2];
    Y[3][3] = Y[1][1] * Y[2][2] - Y[2][1] * Y[1][2];
    a = sqrt(sq(Y[1][3]) + sq(Y[2][3]) + sq(Y[3][3]));
    for (int i = 1; i <= 3; i++)
    {
        Y[i][3] /= a;
    }
}

void transformMatrix()
{
    // Transform Matrix
    for (int i = 1; i <= 3; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            V[i][j] = X[i][j];
        }
    }
    determinateSubroutine();

    double e = W;
    for (int m = 1; m <= 3; m++)
    {
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                V[i][j] = X[i][j];
            }
        }
        for (int n = 1; n <= 3; n++)
        {
            V[1][m] = 0;
            V[2][m] = 0;
            V[3][m] = 0;
            V[n][m] = 1;
            determinateSubroutine();
            Q[m][n] = W / e;
        }
    }

    for (int i = 1; i <= 3; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            R[i][j] = 0;
        }
    }

    for (int i = 1; i <= 3; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            for (int l = 1; l <= 3; l++)
            {
                R[i][j] = R[i][j] + Y[i][l] * Q[l][j];
            }
        }
    }

    for (int m = 1; m <= 3; m++)
    {
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                V[i][j] = R[i][j];
            }
        }
        determinateSubroutine();
        e = W;
        for (int n = 1; n <= 3; n++)
        {
            V[1][m] = 0;
            V[2][m] = 0;
            V[3][m] = 0;
            V[n][m] = 1;
            determinateSubroutine();
            Q[m][n] = W / e;
        }
    }

    ALIGN_READY = true;
}

void determinateSubroutine()
{
    W = V[1][1] * V[2][2] * V[3][3] + V[1][2] * V[2][3] * V[3][1];
    W = W + V[1][3] * V[3][2] * V[2][1];
    W = W - V[1][3] * V[2][2] * V[3][1] - V[1][1] * V[3][2] * V[2][3];
    W = W - V[1][2] * V[2][1] * V[3][3];
}

void angleSubroutine(long double *pri, long double *sec)
{
    double c = sqrt(sq(Y[1][1]) + sq(Y[2][1]));

    if (c == 0 && Y[3][1] == 0)
        (*sec) = 0;
    else
        (*sec) = atan2(Y[3][1], c);

    if (c == 0)
        (*pri) = 0;
    else
        (*pri) = atan2(Y[2][1], Y[1][1]);

    (*pri) = validRev(*pri);
}

void subroutine_1(double pri, double sec)
{
    double cosPri = cos(pri);
    double sinPri = sin(pri);
    double cosSec = cos(sec);
    double sinSec = sin(sec);
    if (Z1_ERR == 0 && Z2_ERR == 0)
    {
        Y[1][0] = cosPri * cosSec;
        Y[2][0] = sinPri * cosSec;
        Y[3][0] = sinSec;
    }
    else
    {
        double cosZ1 = cos(Z1_ERR);
        double cosZ2 = cos(Z2_ERR);
        double sinZ1 = sin(Z1_ERR);
        double sinZ2 = sin(Z2_ERR);

        Y[1][0] = cosPri * cosSec * cosZ2 - sinPri * cosZ1 * sinZ2 + sinPri * sinSec * sinZ1 * cosZ2;
        Y[2][0] = sinPri * cosSec * cosZ2 + cosPri * sinZ2 * cosZ1 - cosPri * sinSec * sinZ1 * cosZ2;
        Y[3][0] = sinSec * cosZ1 * cosZ2 + sinZ1 * sinZ2;
    }
}

void subroutine_2(long double pri, long double sec, int type)
{
    double cosPri = cos(pri);
    double sinPri = sin(pri);
    double cosSec = cos(sec);
    double sinSec = sin(sec);
    if (Z1_ERR == 0 && Z2_ERR == 0)
    {
        Y[1][0] = cosPri * cosSec;
        Y[2][0] = sinPri * cosSec;
        Y[3][0] = sinSec;
    }
    else
    {
        double cosZ1 = cos(Z1_ERR);
        double cosZ2 = cos(Z2_ERR);
        double sinZ1 = sin(Z1_ERR);
        double sinZ2 = sin(Z2_ERR);
        if (type == 1)
            takiSimple(cosPri, cosSec, sinPri, sinSec);
        else if (type == 2)
            takiSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        else if (type == 3)
            bellIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        else if (type == 4)
            takiIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        else if (type == 5)
            bellTakiIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    }
}

void takiSimple(double cosPri, double cosSec, double sinPri, double sinSec)
{
    Y[1][1] = cosSec * cosPri + Z2_ERR * sinPri - Z1_ERR * sinSec * sinPri;
    Y[2][1] = cosSec * sinPri - Z2_ERR * cosPri - Z1_ERR * sinSec * cosPri;
    Y[3][1] = sinSec;
}

void takiSmallAngle(double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    Y[1][1] = (cosSec * cosPri + sinPri * cosZ1 * sinZ2 - sinSec * sinPri * sinZ1 * cosZ2) / cosZ2;
    Y[2][1] = (cosSec * sinPri - cosPri * cosZ1 * sinZ2 + sinSec * cosPri * sinZ1 * cosZ2) / cosZ2;
    Y[3][1] = (sinSec - sinZ1 * sinZ2) / (cosZ1 * cosZ2);
}

void bellIterative(long double pri, long double sec, double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    double trueAz = pri;
    double tanTrueAz = tan(trueAz);
    double apparentAlt = getApparentAlt(sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    double cosApparentSec = cos(apparentAlt);
    double sinApparentSec = sin(apparentAlt);
    double g = cosZ2 * sinZ1 * sinApparentSec * tanTrueAz - tanTrueAz * sinZ2 * cosZ1 - cosZ2 * cosApparentSec;
    double h = sinZ2 * cosZ1 - cosZ2 * sinZ1 * sinApparentSec - tanTrueAz * cosZ2 * cosApparentSec;

    takiSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    angleSubroutine(&pri, &sec);
    double apparentAz = pri;

    double bestApparentAz = apparentAz;
    double holdGoalSeek = LONG_MAX;
    double incr = ARCMIN_TO_RAD;
    double minIncr = ARCSEC_TO_RAD;
    boolean dir = true;
    int subrLCount = 0;

    double goalSeek;
    do
    {
        if (dir)
            apparentAz += incr;
        else
            apparentAz -= incr;

        goalSeek = g * sin(apparentAz) - h * cos(apparentAz);

        if (fabs(goalSeek) <= fabs(holdGoalSeek))
            bestApparentAz = apparentAz;
        else
        {
            incr /= 2;
            dir = !dir;
        }
        holdGoalSeek = goalSeek;
        subrLCount++;
    } while (incr >= minIncr);

    cosPri = cos(bestApparentAz);
    sinPri = sin(bestApparentAz);
    cosSec = cos(apparentAlt);
    sinSec = sin(apparentAlt);

    Y[1][1] = cosPri * cosSec;
    Y[2][1] = sinPri * cosSec;
    Y[2][1] = sinSec;
}

void takiIterative(long double pri, long double sec, double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    int maxLoopCount = 25;
    int subrTCount = 0;
    double holdPri = pri;
    double holdSec = sec;
    double lastPri, lastSec, errPri, errSec;

    lastPri = LONG_MAX / 2;
    lastSec = LONG_MAX / 2;

    takiSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    do
    {
        angleSubroutine(&pri, &sec);
        errPri = fabs(lastPri - pri);
        errSec = fabs(lastSec - sec);
        lastPri = pri;
        lastSec = sec;

        double cosF1 = cos(pri);
        double sinF1 = sin(pri);

        Y[1][1] = (cosSec * cosPri + sinF1 * cosZ1 * sinZ2 - (sinSec - sinZ1 * sinZ2) * sinF1 * sinZ1 / cosZ1) / cosZ2;
        Y[2][1] = (cosSec * sinPri - cosF1 * cosZ1 * sinZ2 + (sinSec - sinZ1 * sinZ2) * cosF1 * sinZ1 / cosZ1) / cosZ2;
        Y[3][1] = (sinSec - sinZ1 * sinZ2) / (cosZ1 * cosZ2);

        subrTCount++;
        if (subrTCount > maxLoopCount)
        {
            pri = holdPri;
            sec = holdSec;
            bellIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        }
    } while (errSec > TENTH_ARCSEC_TO_RAD || errPri > TENTH_ARCSEC_TO_RAD);
}

void bellTakiIterative(long double pri, long double sec, double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    double apparentAlt = getApparentAlt(sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    takiIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    angleSubroutine(&pri, &sec);

    cosPri = cos(pri);
    sinPri = sin(pri);
    cosSec = cos(apparentAlt);
    sinSec = sin(apparentAlt);

    Y[1][1] = cosPri * cosSec;
    Y[2][1] = sinPri * cosSec;
    Y[3][1] = sinSec;
}

double getApparentAlt(double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    double v1 = (sinSec - sinZ1 * sinZ2) * cosZ1 * (cosZ2 / ((sinZ1 * sinZ1 - 1) * sinZ2 * sinZ2 - 1));
    return asin(v1);
}

// BEST Z1/Z2
// nRange to range is search area in radians
// incr is increment +/- radians
void bestZ12(int n)
{
    double startZ1 = 0.0, startZ2 = 0.0;
    double z12Range = 25200.0 * ARCSEC_TO_RAD;
    double z12Resolution = 1800.0 * ARCSEC_TO_RAD;
    double minResolution = 3.6 * ARCSEC_TO_RAD;
    double bestZ1 = 0;
    double bestZ2 = 0;
    double bestPointingErrorRMS = LONG_MAX;
    double pointingErrorRMS, pointingErrorRMSTotal, altError, azError;
    pointingErrorRMSTotal = 0;

    do {
        for (Z1_ERR = startZ1; Z1_ERR < z12Range; Z1_ERR += z12Resolution)
        {
            for (Z2_ERR = startZ1; Z2_ERR < z12Range; Z2_ERR += z12Resolution)
            {
                pointingErrorRMSTotal = 0.0;
                for (int i = 0; i < n; i++)
                {
                    double alt1 = ln_deg_to_rad(ALIGNMENT_STARS[i].hrzPos.alt);
                    double az1 = ln_deg_to_rad(ALIGNMENT_STARS[i].hrzPos.az);

    #ifdef DEBUG_MOUNT_ERRS
                    // Add an error to the altitude to help debug mount error routines
                    az1 += ln_deg_to_rad(3.5);
                    alt1 += ln_deg_to_rad(1.5);
    #endif
                    equatorialToScope(&ALIGNMENT_STARS[i].equPos, &SCOPE.lnLatPos, SCOPE.JD, &ALIGNMENT_STARS[i].hrzPos);
                    double alt2 = ln_deg_to_rad(ALIGNMENT_STARS[i].hrzPos.alt);
                    double az2 = ln_deg_to_rad(ALIGNMENT_STARS[i].hrzPos.az);
                    altError = alt1 - alt2;
                    azError = (az1 - az2) * cos(alt1);
                    pointingErrorRMS = sqrt(sq(altError) + sq(azError));
                    pointingErrorRMSTotal += pointingErrorRMS;
                }
                pointingErrorRMSTotal /= n;
                if (pointingErrorRMSTotal < bestPointingErrorRMS)
                {
                    bestPointingErrorRMS = pointingErrorRMSTotal;
                    bestZ1 = Z1_ERR;
                    bestZ2 = Z2_ERR;
    #ifdef SERIAL_DEBUG
                    Serial.print("new bestPointingErrorRMS: ");
                    Serial.print(bestPointingErrorRMS);
                    Serial.print(" Z1 Error: ");
                    Serial.print(bestZ1);
                    Serial.print(" Z2 Error: ");
                    Serial.print(bestZ2);
                    Serial.println("");
    #endif
                }
            }
        }
        startZ1 = bestZ1;
        startZ2 = bestZ2;
        z12Range /= 10.0;
        z12Resolution /= 10.0;
    } while (z12Resolution >= minResolution);

    Z1_ERR = bestZ1;
    Z2_ERR = bestZ2;
#ifdef SERIAL_DEBUG
    Serial.print("Best Z1/2 Found: ");
    Serial.print(" Z1 Error: ");
    Serial.print(bestZ1);
    Serial.print(" Z2 Error: ");
    Serial.print(bestZ2);
    Serial.println("");
#endif
}

double calcAngSepHaversine(Object *a, Object *z, boolean equatorial)
{
    double aAlt = ln_deg_to_rad(a->hrzPos.alt);
    double zAlt = ln_deg_to_rad(z->hrzPos.alt);
    double aAz = ln_deg_to_rad(a->hrzPos.az);
    double zAz = ln_deg_to_rad(z->hrzPos.az);
    double aRa = ln_deg_to_rad(a->equPos.ra);
    double zRa = ln_deg_to_rad(z->equPos.ra);
    double aDec = ln_deg_to_rad(a->equPos.dec);
    double zDec = ln_deg_to_rad(z->equPos.dec);

    if (equatorial)
        return (2 * asin(sqrt(hav(zDec - aDec) + cos(aDec) * cos(zDec) * hav(zRa - aRa))));
    else
        return (2 * asin(sqrt(hav(zAlt - aAlt) + cos(aAlt) * cos(zAlt) * hav(zAz - aAz))));
}

double calcAngSepDiff(Object *a, Object *z)
{
    return fabs(fabs(calcAngSepHaversine(a, z, true)) - fabs(calcAngSepHaversine(a, z, false)));
}

double calcAltOffsetDirectly(Object &a, Object &z)
{
    double altOffset;
    double aAlt = ln_deg_to_rad(a.hrzPos.alt);
    double zAlt = ln_deg_to_rad(z.hrzPos.alt);
    double aAz = ln_deg_to_rad(a.hrzPos.az);
    double zAz = ln_deg_to_rad(z.hrzPos.az);

#ifdef DEBUG_MOUNT_ERRS
    // Add an error to the altitude to help debug mount error routines
    aAlt += ln_deg_to_rad(3.5);
    zAlt += ln_deg_to_rad(3.5);
#endif

    double n = cos(aAz - zAz);
    double m = cos(ln_deg_to_rad(ln_get_angular_separation(&a.equPos, &z.equPos)));
    double x = (2 * m - (n + 1) * cos(aAlt - zAlt)) / (n - 1);

    double a1 = 0.5 * (+acos(x) - aAlt - zAlt);
    double a2 = 0.5 * (-acos(x) - aAlt - zAlt);

    if (fabs(a1) < fabs(a2))
        altOffset = a1;
    else
        altOffset = a2;
    return altOffset;
}

double calcAltOffsetIteratively(Object &a, Object &z)
{
    double aAlt, zAlt, diff, lastDiff, bestDiff;
    int searchRangeDeg = 45;
    double altIncr = ARCSEC_TO_RAD;
    double iMax = searchRangeDeg * DEG_TO_RAD / altIncr;
    double i = 0;
    double bestAltOff = 0;

    aAlt = ln_deg_to_rad(a.hrzPos.alt);
    zAlt = ln_deg_to_rad(z.hrzPos.alt);
    bestDiff = QTR_REV;
    lastDiff = QTR_REV;

    while (i < iMax)
    {
        diff = calcAngSepDiff(&a, &z);
#ifdef SERIAL_DEBUG
        Serial.print("AngSepDiff: ");
        Serial.print(diff);
        Serial.println("");
#endif
        if (diff < bestDiff)
        {
            bestDiff = diff;
            bestAltOff = aAlt - ln_deg_to_rad(a.hrzPos.alt);
        }
        if (diff > lastDiff)
            break;
        else
            lastDiff = diff;
        i++;
        aAlt += altIncr;
        zAlt += altIncr;
    }
    aAlt = ln_deg_to_rad(a.hrzPos.alt);
    zAlt = ln_deg_to_rad(z.hrzPos.alt);
    lastDiff = LONG_MAX;
    i = 0;
    while (i < iMax)
    {
        diff = calcAngSepDiff(&a, &z);
#ifdef SERIAL_DEBUG
        Serial.print("AngSepDiff: ");
        Serial.print(diff);
        Serial.println("");
#endif
        if (diff < bestDiff)
        {
            bestDiff = diff;
            bestAltOff = aAlt - ln_deg_to_rad(a.hrzPos.alt);
        }
        if (diff > lastDiff)
            break;
        else
            lastDiff = diff;
        i++;
        aAlt -= altIncr;
        zAlt -= altIncr;
    }
    return bestAltOff;
}

double getAltOffset(Object &a, Object &z)
{
    try
    {
        return calcAltOffsetDirectly(a, z);
    }
    catch (const std::exception &e)
    {
        return calcAltOffsetIteratively(a, z);
    }
}

void bestZ3(int n)
{
    double accumAltOffset;
    int count;

    accumAltOffset = getAltOffset(ALIGNMENT_STARS[0], ALIGNMENT_STARS[1]);
    count = 1;
    if (n == 3)
    {
        accumAltOffset += getAltOffset(ALIGNMENT_STARS[0], ALIGNMENT_STARS[2]);
        count++;
        accumAltOffset += getAltOffset(ALIGNMENT_STARS[1], ALIGNMENT_STARS[2]);
        count++;
    }
    Z3_ERR = accumAltOffset / (double)count;
#ifdef SERIAL_DEBUG
    Serial.print("bestZ3 - ");
    Serial.print(ln_rad_to_deg(Z3_ERR));
    Serial.println("");
#endif
}

void calcBestZ12()
{
    // handles searching for Z1/2 up to +/- 1 degree
    bestZ12(3); // 10 iterations
}

void calcBestZ3()
{
    // Search for Z3 +/- 10 Degrees
    bestZ3(3);
    // bestZ3(3, - 10.0, 10.0, 2.0);   // Runs 10x
    // bestZ3(3, ln_rad_to_deg(Z3_ERR) - 2.0, ln_rad_to_deg(Z3_ERR) + 2.0, 0.5);     // Runs 8x
    // bestZ3(3, ln_rad_to_deg(Z3_ERR) - 0.5, ln_rad_to_deg(Z3_ERR) + 0.5, 0.062);   // Runs 16x
}

void bestZ123()
{
    if (ALIGN_READY && CORRECT_MOUNT_ERRS)
    {
        calcBestZ3();
        calcBestZ12();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Calculate Planet Positions

String getTimeString(struct ln_zonedate date)
{
    String toStr;
    if (date.hours < 10)
        toStr = '0' + String(date.hours);
    else
        toStr = String(date.hours);

    toStr += ':';

    if (date.minutes < 10)
        toStr += '0' + String(date.minutes);
    else
        toStr += String(date.minutes);
    return toStr;
}

void getPlanetPosition(int planetNum, Object &planet)
{
    struct ln_rst_time rst;
    struct ln_zonedate rise, set;
    String name;
    int rstStatus = 0;
    double dist = 0.0, mag = 0.0, size = 0.0, illum = 0.0;

    switch (planetNum)
    {
    // MERCURY
    case 1:
        name = "MERCURY";
        ln_get_mercury_equ_coords(SCOPE.JD, &planet.equPos);
        dist = ln_get_mercury_earth_dist(SCOPE.JD);
        mag = ln_get_mercury_magnitude(SCOPE.JD);
        size = ln_get_mercury_sdiam(SCOPE.JD);
        illum = ln_get_mercury_disk(SCOPE.JD);
        rstStatus = ln_get_mercury_rst(SCOPE.JD, &SCOPE.lnLatPos, &rst);
        break;
    case 2:
        name = "VENUS";
        ln_get_venus_equ_coords(SCOPE.JD, &planet.equPos);
        dist = ln_get_venus_earth_dist(SCOPE.JD);
        mag = ln_get_venus_magnitude(SCOPE.JD);
        size = ln_get_venus_sdiam(SCOPE.JD);
        illum = ln_get_venus_disk(SCOPE.JD);
        rstStatus = ln_get_venus_rst(SCOPE.JD, &SCOPE.lnLatPos, &rst);
        break;
    case 3:
        name = "EARTH";
        dist = 0.0;
        mag = 0.0;
        size = 0.0;
        illum = 0.0;
        rstStatus = 1;
        break;
    case 4:
        name = "MARS";
        ln_get_mars_equ_coords(SCOPE.JD, &planet.equPos);
        dist = ln_get_mars_earth_dist(SCOPE.JD);
        mag = ln_get_mars_magnitude(SCOPE.JD);
        size = ln_get_mars_sdiam(SCOPE.JD);
        illum = ln_get_mars_disk(SCOPE.JD);
        rstStatus = ln_get_mars_rst(SCOPE.JD, &SCOPE.lnLatPos, &rst);
        break;
    case 5:
        name = "JUPITER";
        ln_get_jupiter_equ_coords(SCOPE.JD, &planet.equPos);
        dist = ln_get_jupiter_earth_dist(SCOPE.JD);
        mag = ln_get_jupiter_magnitude(SCOPE.JD);
        size = ln_get_jupiter_equ_sdiam(SCOPE.JD);
        illum = ln_get_jupiter_disk(SCOPE.JD);
        rstStatus = ln_get_jupiter_rst(SCOPE.JD, &SCOPE.lnLatPos, &rst);
        break;
    case 6:
        name = "SATURN";
        ln_get_saturn_equ_coords(SCOPE.JD, &planet.equPos);
        dist = ln_get_saturn_earth_dist(SCOPE.JD);
        mag = ln_get_saturn_magnitude(SCOPE.JD);
        size = ln_get_saturn_equ_sdiam(SCOPE.JD);
        illum = ln_get_saturn_disk(SCOPE.JD);
        rstStatus = ln_get_saturn_rst(SCOPE.JD, &SCOPE.lnLatPos, &rst);
        break;
    case 7:
        name = "URANUS";
        ln_get_uranus_equ_coords(SCOPE.JD, &planet.equPos);
        dist = ln_get_uranus_earth_dist(SCOPE.JD);
        mag = ln_get_uranus_magnitude(SCOPE.JD);
        size = ln_get_uranus_sdiam(SCOPE.JD);
        illum = ln_get_uranus_disk(SCOPE.JD);
        rstStatus = ln_get_uranus_rst(SCOPE.JD, &SCOPE.lnLatPos, &rst);
        break;
    case 8:
        name = "NEPTUNE";
        ln_get_neptune_equ_coords(SCOPE.JD, &planet.equPos);
        dist = ln_get_neptune_earth_dist(SCOPE.JD);
        mag = ln_get_neptune_magnitude(SCOPE.JD);
        size = ln_get_neptune_sdiam(SCOPE.JD);
        illum = ln_get_neptune_disk(SCOPE.JD);
        rstStatus = ln_get_neptune_rst(SCOPE.JD, &SCOPE.lnLatPos, &rst);
        break;
    case 9:
        name = "PLUTO";
        ln_get_pluto_equ_coords(SCOPE.JD, &planet.equPos);
        dist = ln_get_pluto_earth_dist(SCOPE.JD);
        mag = ln_get_pluto_magnitude(SCOPE.JD);
        size = ln_get_pluto_sdiam(SCOPE.JD);
        illum = ln_get_pluto_disk(SCOPE.JD);
        rstStatus = ln_get_pluto_rst(SCOPE.JD, &SCOPE.lnLatPos, &rst);
        break;
    default:
        name = "NONE";
        dist = 0.0;
        mag = 0.0;
        size = 0.0;
        illum = 0.0;
        rstStatus = 1;
        break;
    }

    planet.name = name;
    planet.description = "DIST: " + String(dist, 2) + " AU";
    planet.mag = String(mag, 2);
    planet.size = String(size * 2.0, 2) + '"'; // Multiply semidiameter by 2.0 to get apparent diameter/size
    planet.type = "ILUM: " + String(illum * 100.0, 2) + '%';
    if (rstStatus == 0)
    {
        getLocalDate(rst.rise, &rise);
        getLocalDate(rst.set, &set);
        planet.constellation = "RISE: " + getTimeString(rise);
        planet.constellation += "       SET: " + getTimeString(set);
    }
    else if (rstStatus == 1)
    {
        planet.constellation = "ALWAYS VISIBLE";
    }
    else
    {
        planet.constellation = "ALWAYS BELOW HORIZON";
    }
    // Correct for astronomical proper motion, aberration, precession and nutation
    if (CORRECT_PRECESSION_ETC)
    {
        ln_get_apparent_posn(&planet.equPos, &planet.propMotion, SCOPE.JD, &planet.equPos);
    }
#ifdef SERIAL_DEBUG
    Serial.print("Added Corrections - RA: ");
    Serial.print(deg2hms(planet.equPos.ra, true, true));
    Serial.print(" DEC: ");
    Serial.print(deg2dms(planet.equPos.dec, true, false));
    Serial.println("");
#endif
}

void drawButton(int X, int Y, int Width, int Height, String Caption, int16_t BodyColor, int16_t BorderColor, int16_t TextColor, int tSize)
{
    //  type: 0:Solid color, no Frame; 1: Frame Only button; 2: Solid color and Frame button;

    if ((BodyColor != 0) && (BorderColor == 0))
    {
        // Button type = 0 ... Solid color, no Frame
        tft.fillRect(X, Y, Width, Height, BodyColor);
    }
    else if ((BodyColor == 0) && (BorderColor != 0))
    {
        // Button type = 1 ... Frame Only button
        tft.drawRect(X, Y, Width, Height, BorderColor);
        tft.fillRect(X + 1, Y + 1, Width - 2, Height - 2, BLACK);
    }
    else if ((BodyColor != 0) && (BorderColor != 0))
    {
        // Button type = 2 ... Solid color and Frame button
        tft.drawRect(X, Y, Width, Height, BorderColor);
        tft.fillRect(X + 1, Y + 1, Width - 2, Height - 2, BodyColor);
    }
    else
    {
        // Will not Draw Button and will return to code!
        return;
    }

    float t_x = 0;
    float t_y = 0;
    if (tSize == 2)
    { // 10 x 14 px. (W x H)
        t_x = (X + 1 + Width / 2) - (Caption.length() * 6);
        t_y = Y + Height / 2 - 5;
    }
    else if (tSize == 1)
    { // 5 x 7 px. (W x H)
        t_x = (X + 1 + Width / 2) - (Caption.length() * 3);
        t_y = Y + Height / 2 - 3;
    }
    else if (tSize == 3)
    { // 15 x 21 px. (W x H)
        t_x = (X + 1 + Width / 2) - (Caption.length() * 8);
        t_y = Y + Height / 2 - 10;
    }
    tft.setCursor((int)t_x, (int)t_y);
    tft.setTextSize(tSize);
    tft.setTextColor(TextColor);
    if (Caption == "+")
    {
        t_x -= 5;
        tft.drawLine((int)t_x + 10, (int)t_y - 5, (int)t_x + 10, (int)t_y - 5, TextColor);
        tft.drawLine((int)t_x + 8, (int)t_y - 4, (int)t_x + 12, (int)t_y - 4, TextColor);
        tft.drawLine((int)t_x + 6, (int)t_y - 3, (int)t_x + 14, (int)t_y - 3, TextColor);
        tft.drawLine((int)t_x + 4, (int)t_y - 2, (int)t_x + 16, (int)t_y - 2, TextColor);
        tft.drawLine((int)t_x + 2, (int)t_y - 1, (int)t_x + 18, (int)t_y - 1, TextColor);
        tft.drawLine((int)t_x, (int)t_y, (int)t_x + 20, (int)t_y, TextColor);
        tft.drawLine((int)t_x - 2, (int)t_y + 1, (int)t_x + 22, (int)t_y + 1, TextColor);
        tft.drawLine((int)t_x - 4, (int)t_y + 2, (int)t_x + 24, (int)t_y + 2, TextColor);

        tft.drawLine((int)t_x - 4, (int)t_y + 8, (int)t_x + 24, (int)t_y + 8, TextColor);
        tft.drawLine((int)t_x - 2, (int)t_y + 9, (int)t_x + 22, (int)t_y + 9, TextColor);
        tft.drawLine((int)t_x, (int)t_y + 10, (int)t_x + 20, (int)t_y + 10, TextColor);
        tft.drawLine((int)t_x + 2, (int)t_y + 11, (int)t_x + 18, (int)t_y + 11, TextColor);
        tft.drawLine((int)t_x + 4, (int)t_y + 12, (int)t_x + 16, (int)t_y + 12, TextColor);
        tft.drawLine((int)t_x + 6, (int)t_y + 13, (int)t_x + 14, (int)t_y + 13, TextColor);
        tft.drawLine((int)t_x + 8, (int)t_y + 14, (int)t_x + 12, (int)t_y + 14, TextColor);
        tft.drawLine((int)t_x + 10, (int)t_y + 15, (int)t_x + 10, (int)t_y + 15, TextColor);
    }
    else
    {
        tft.println(Caption);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// REGULAR UPDATES

void considerTelescopeMove()
{
    currentElevAngle();
    if (IS_IN_OPERATION)
        telescopeRaDec();
}

void considerTimeUpdates()
{ // UPDATEs time on Screen1 && Screen4 -  Clock Screen and Main Screen
    int changes = 0;
    for (int y = 0; y < 12; y++)
    {
        if (W_DATE_TIME[y] != 0)
        {
            changes = 1;
        }
    }
    // Update clock every 2mins
    if ((millis() - UPDATE_CLOCK) > 120000)
    {
        NOW = rtc.GetDateTime();
        setTime(NOW.Hour(), NOW.Minute(), NOW.Second(), NOW.Day(), NOW.Month(), NOW.Year());
        UPDATE_CLOCK = millis();
    }

    if (CURRENT_SCREEN == 3 && (millis() - UPDATE_TIME) > 5000)
    {
        // Main Screen
        setScopeDateTime();
        getLocalDate(SCOPE.JD, &SCOPE.localDate);
        tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
        tft.setTextSize(2);
        if (OLD_MIN != SCOPE.localDate.minutes)
        {
            OLD_MIN = SCOPE.localDate.minutes;
            tft.fillRect(0, 0, 80, 20, BLACK);
            tft.setCursor(10, 10);
            if (SCOPE.localDate.hours < 10)
            {
                tft.print("0");
            }
            tft.print(SCOPE.localDate.hours);
            tft.print(":");
            if (SCOPE.localDate.minutes < 10)
            {
                tft.print("0");
            }
            tft.print(SCOPE.localDate.minutes);
        }
        tft.fillRect(100, 0, 140, 20, BLACK);
        tft.setCursor(100, 10);
        tft.print("LST ");
        tft.print(hrs2hms(SCOPE.LST, false, false));

        if ((CURRENT_OBJECT.name != ""))
        {
            objectAltAz();
            tft.fillRect(70, 205, 170, 40, BLACK);
            tft.setTextColor(L_TEXT);
            tft.setTextSize(2);

            tft.setCursor(70, 205);
            tft.print(deg2dms(CURRENT_OBJECT.hrzPos.az, true, true));
            tft.setCursor(70, 225);
            tft.print(deg2dms(CURRENT_OBJECT.hrzPos.alt, true, false));
        }

        UPDATE_TIME = millis();
    }
    else if (CURRENT_SCREEN == 1 && (millis() - UPDATE_TIME) > 10000 && changes == 0)
    {
        // Clock Screen
        NOW = rtc.GetDateTime();
        // Day has Changed
        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        if (OLD_DAY != NOW.Day())
        {
            tft.fillRect(0, 50, 240, 16, BLACK);
            tft.setCursor(70, 50);

            if (NOW.Day() < 10)
            {
                tft.print("0");
            }
            tft.print(NOW.Day());
            tft.print("/");
            if (NOW.Month() < 10)
            {
                tft.print("0");
            }
            tft.print(NOW.Month());
            tft.print("/");
            tft.print(NOW.Year());
        }
        // Minute has changed
        if (OLD_MIN != NOW.Minute())
        {
            tft.fillRect(0, 70, 249, 16, BLACK);

            tft.setCursor(80, 70);
            if (NOW.Hour() < 10)
            {
                tft.print("0");
            }
            tft.print(NOW.Hour());
            tft.print(":");
            if (NOW.Minute() < 10)
            {
                tft.print("0");
            }
            tft.print(NOW.Minute());
        }

        UPDATE_TIME = millis();
    }
    else if (CURRENT_SCREEN == 0 && (millis() - UPDATE_TIME) > 5000)
    {
        // GPS Screen
        tft.fillRect(0, 95, 240, 170, BLACK);
        tft.setTextColor(L_TEXT);
        tft.setTextSize(2);
        tft.setCursor(10, 100);
        tft.print("Lat: ");
        tft.print(gps.location.lat(), 4);

        tft.setCursor(10, 120);
        tft.print("Lng: ");
        tft.print(gps.location.lng(), 4);

        tft.setCursor(10, 140);
        tft.print("Alt: ");
        tft.print(gps.altitude.meters());
        tft.print("m");

        tft.setCursor(10, 160);
        tft.print("Sats: ");
        tft.print(gps.satellites.value());

        tft.setCursor(10, 180);
        tft.print("GMT: ");
        if (gps.time.hour() < 10)
        {
            tft.print("0");
        }
        tft.print(gps.time.hour());
        tft.print(":");
        if (gps.time.minute() < 10)
        {
            tft.print("0");
        }
        tft.print(gps.time.minute());

        if (gps.satellites.value() == 0)
        {
            smartDelay(1000);
        }
        else
        {
            tft.setCursor(10, 200);
            tft.print("Verifying Co-ords");
            tft.setCursor(10, 220);
            tft.print("Iteration #");
            tft.print((GPS_ITERATIONS + 1));
            GPS_ITERATIONS += 1;
            smartDelay(1000);
        }

        if ((GPS_ITERATIONS > 2) && (gps.location.lat() != 0))
        {
            SCOPE.lnLatPos.lat = gps.location.lat();
            SCOPE.lnLatPos.lng = gps.location.lng();
            SCOPE.observationAlt = gps.altitude.meters();
            // Save this location to SPIFFS for next session
            OBSERVATION_LATTITUDE = SCOPE.lnLatPos.lat;
            OBSERVATION_LONGITUDE = SCOPE.lnLatPos.lng;
            OBSERVATION_ALTITUDE = SCOPE.observationAlt;
            writeGPSToSPIFFS();
#ifdef SERIAL_DEBUG
            Serial.print("GOT GPS DATA - LAT: ");
            Serial.print(OBSERVATION_LATTITUDE);
            Serial.print(" LNG: ");
            Serial.print(OBSERVATION_LONGITUDE);
            Serial.print(" ALT: ");
            Serial.print(OBSERVATION_ALTITUDE);
            Serial.println("");
#endif
            setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
            RtcDateTime gpsTime = RtcDateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
            rtc.SetDateTime(gpsTime);
            SUMMER_TIME = isSummerTime();
            calculateLST();
            delay(150);
            drawAlignCorrectionsScreen();
        }
        UPDATE_TIME = millis();
    }
    else if (CURRENT_SCREEN == 7 && (millis() - UPDATE_TIME) > 5000)
    {
        uint8_t starNum = ALIGN_STEP - 1;
        tft.fillRect(70, 210, 170, 50, BLACK);
        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        tft.setCursor(70, 210);
        tft.print(deg2dms(ALIGNMENT_STARS[starNum].hrzPos.az, true, true));
        tft.setCursor(70, 230);
        tft.print(deg2dms(ALIGNMENT_STARS[starNum].hrzPos.alt, true, false));
    }
    else if (CURRENT_SCREEN == 14 && (millis() - UPDATE_TIME) > 5000)
    {
        tft.fillRect(70, 210, 170, 50, BLACK);
        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        tft.setCursor(70, 210);
        tft.print(deg2dms(CURRENT_OBJECT.hrzPos.az, true, true));
        tft.setCursor(70, 230);
        tft.print(deg2dms(CURRENT_OBJECT.hrzPos.alt, true, false));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GUI AND DISPLAY
// ......................................................................
//  This part of the code take care of:
//  - Drawing interface screens;
//  - Draws symbols on certain screens (e.g. XX/XX/XXXX in day/time screen)
//  - Draws OnScreen Messages
//
//  Screens are separated like:
//    * CURRENT_SCREEN == -1  - drawInitScreen() - Initialise the system
//    * CURRENT_SCREEN == 0   - drawGPSScreen() Where the GPS coordinates are displayed
//    * CURRENT_SCREEN == 1   - drawClockScreen() Captures updates on the time and date
//    * CURRENT_SCREEN == 2   - drawSelectAlignment() Select Alignment method (only have 3 buttons)
//    * CURRENT_SCREEN == 3   - drawMainScreen() Captures all clicks on the MAIN Screen of the application
//    * CURRENT_SCREEN == 4   - drawLoadScreen() Captures input on Load screen (all of them: Messier && Treasures)
//    * CURRENT_SCREEN == 5   - drawOptionsScreen();
//    * CURRENT_SCREEN == 6   - drawStarSelectScreen() - To Select Alignment Star;
//    * CURRENT_SCREEN == 7   - drawAlignScreen() - to actually align on Star. Called few times per alignment procedure
//    * CURRENT_SCREEN == 8   - drawRemoteScreen() - disable internal functions and run as slave to remote app
//    * CURRENT_SCREEN == 9   - drawPlanetScreen() -
//    * CURRENT_SCREEN == 10  - drawAlignModeScreen() - Select Master ALIGNMENT_METHOD
//    * CURRENT_SCREEN == 11  - drawAlignCorrectionsScreen() - Enable Corrections
//    * CURRENT_SCREEN == 12  - drawCatalogueScreen() - Select Catalogue to load objects from
//    * CURRENT_SCREEN == 13  - drawCatalogueSummaryScreen() - Show summary and info about selected Catalogue
//    * CURRENT_SCREEN == 14  - drawObjectSummaryScreen() - Display summary of selected object

void removeTime_addXX()
{
    if (DATE_ENTRY_POS == 0)
    {
        tft.fillRect(0, 40, 240, 60, BLACK);
        tft.setTextSize(2);
        tft.setTextColor(D_TEXT);
        tft.setCursor(70, 50);
        tft.print("XX/XX/XXXX");
        tft.setCursor(80, 70);
        tft.print("XX:XX");
    }
}

void drawInitScreen()
{
    CURRENT_SCREEN = -1;
    tft.fillScreen(BLACK);

    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.setCursor(10, 10);
    tft.print(" DobDSC");

    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 40);
    tft.print("Concept by <dEskoG>");
    tft.setCursor(10, 60);
    tft.print("DobDSC by A Flight");

    tft.setTextSize(1);
    tft.setCursor(10, 80);
    tft.print("GNU General Public License");
    tft.setCursor(10, 90);
    tft.println("Version: " + FirmwareNumber);

#ifdef SERIAL_DEBUG
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setCursor(10, 100);
    tft.print(" - DEBUG MODE");
#endif

    volatile bool check = true;
    MESS_PAGER = 0;
    CALD_PAGER = 0;
    TREAS_PAGER = 0;
    STARS_PAGER = 0;

    tft.setTextColor(D_TEXT);
    tft.setCursor(10, 120);
    tft.print("Initialising Touchscreen -> ");
    tft.setTextColor(L_TEXT);
    tft.print("OK");
    tft.setTextColor(D_TEXT);
    delay(500);

    tft.setCursor(10, 140);
    tft.print("Initialising Clock -> ");
    if (isnan(rtc.GetTemperature().AsFloatDegC()))
    {
        tft.setTextColor(L_TEXT);
        tft.print("FAIL");
        tft.setTextColor(D_TEXT);
        check = false;
    }
    else
    {
        tft.setTextColor(L_TEXT);
        tft.print("OK");
        tft.setTextColor(D_TEXT);
    }
    delay(500);

    tft.setCursor(10, 160);
    tft.print("Initialising BlueTooth -> ");
    if (SerialBT.available() > 0)
    {
        tft.setTextColor(L_TEXT);
        tft.print("OK");
        tft.setTextColor(D_TEXT);
    }
    else
    {
        tft.setTextColor(L_TEXT);
        tft.print("FAIL");
        tft.setTextColor(D_TEXT);
    }
    delay(500);

    tft.setCursor(10, 180);
    tft.print("Initialising GPS -> ");
    if (gps.satellites.value() != -1)
    {
        tft.setTextColor(L_TEXT);
        tft.print("OK");
        tft.setTextColor(D_TEXT);
    }
    else
    {
        tft.setTextColor(L_TEXT);
        tft.print("FAIL");
        tft.setTextColor(D_TEXT);
    }
    delay(5000);

    drawGPSScreen();

#ifndef SERIAL_DEBUG
    if (check == false)
        while (1)
            ; //don't do anything more
#endif
}

void drawGPSScreen()
{
    CURRENT_SCREEN = 0;
    // First try to load saved co-ords from last session
    loadGPSFromSPIFFS();

    // Now fill scope pos with loaded values or hard-coded values
    SCOPE.lnLatPos.lng = OBSERVATION_LONGITUDE;
    SCOPE.lnLatPos.lat = OBSERVATION_LATTITUDE;
    SCOPE.observationAlt = OBSERVATION_ALTITUDE;

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" GPS");

    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 40);
    tft.print("Searching for");
    tft.setCursor(10, 60);
    tft.print("Satellites...");

    tft.setTextSize(1);
    tft.setCursor(10, 80);
    tft.print("Allow > 60s for cold start up");

    drawButton(40, 270, 160, 40, "SKIP", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawClockScreen()
{
    CURRENT_SCREEN = 1;
    NOW = rtc.GetDateTime();
    setTime(NOW.Hour(), NOW.Minute(), NOW.Second(), NOW.Day(), NOW.Month(), NOW.Year());
    setScopeDateTime();

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" GMT/UTC");

    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.setCursor(70, 50);

    if (NOW.Day() < 10)
    {
        tft.print("0");
    }
    tft.print(NOW.Day());
    tft.print("/");
    if (NOW.Month() < 10)
    {
        tft.print("0");
    }
    tft.print(NOW.Month());
    tft.print("/");
    tft.print(NOW.Year());

    tft.setCursor(80, 70);
    if (NOW.Hour() < 10)
    {
        tft.print("0");
    }
    tft.print(NOW.Hour());
    tft.print(":");
    if (NOW.Minute() < 10)
    {
        tft.print("0");
    }
    tft.print(NOW.Minute());

    // Draw keypad....
    tft.setTextColor(L_TEXT);
    int kk = 1;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (kk == 10)
            {
                if (SUMMER_TIME == 1)
                {
                    drawButton(10, 270, 65, 40, "DST", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                }
                else
                {
                    drawButton(10, 270, 65, 40, "DST", 0, BTN_L_BORDER, L_TEXT, 2);
                }
            }
            else if (kk == 11)
            {
                drawButton(85, 270, 65, 40, "0", BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);
            }
            else if (kk == 12)
            {
                drawButton(160, 270, 65, 40, "OK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
            }
            else
            {
                drawButton(((j * 75) + 10), ((i * 50) + 120), 65, 40, String(kk), BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);
            }
            kk += 1;
        }
    }
}

void drawAlignCorrectionsScreen()
{
    CURRENT_SCREEN = 11;

    // Load in Alignment Stars
    LOAD_SELECTOR = 1;

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" CORRECT");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    if (CORRECT_REFRACTION)
        drawButton(20, 50, 200, 40, "REFRACTION", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    else
        drawButton(20, 50, 200, 40, "REFRACTION", 0, BTN_L_BORDER, L_TEXT, 2);
    if (CORRECT_PRECESSION_ETC)
        drawButton(20, 110, 200, 40, "ORBITAL ANOMS.", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    else
        drawButton(20, 110, 200, 40, "ORBITAL ANOMS.", 0, BTN_L_BORDER, L_TEXT, 2);
    if (CORRECT_MOUNT_ERRS)
        drawButton(20, 170, 200, 40, "MOUNT ERRORS", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    else
        drawButton(20, 170, 200, 40, "MOUNT ERRORS", 0, BTN_L_BORDER, L_TEXT, 2);

    tft.setTextColor(L_TEXT);
    tft.setCursor(20, 230);
    tft.setTextSize(1);
    tft.print("Z1 Error: ");
    tft.print(rad2dms(Z1_ERR, true, false));
    tft.setCursor(20, 240);
    tft.print("Z2 Error: ");
    tft.print(rad2dms(Z2_ERR, true, false));
    tft.setCursor(20, 250);
    tft.print("Z3 Error: ");
    tft.print(rad2dms(Z3_ERR, true, false));

    if (!ALIGN_READY)
        drawButton(20, 270, 200, 40, "BEGIN ALIGN", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawSelectAlignment()
{
    CURRENT_SCREEN = 2;
    ALIGN_STEP = 1;

    initAlign();

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" ALIGN");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(20, 50, 200, 40, "2-STAR AUTO", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 110, 200, 40, "3-STAR AUTO", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 170, 200, 40, "2-STAR MANUAL", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 230, 200, 40, "3-STAR MANUAL", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawStarSelectScreen()
{
    CURRENT_SCREEN = 6;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" ALIGN");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 270, 100, 40, "< PREV", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 270, 100, 40, "NEXT >", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawLoadObjects();
}

void drawLoadAlignStars()
{
    if (LOADED_OBJECTS != 1)
    {
        loadStarsFromSPIFFS("/alignment.csv");
        LOADED_OBJECTS = 1;
    }

    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 50);
    tft.print("Brightest Stars");

    tft.setTextColor(BTN_BLK_TEXT);
    tft.setTextSize(1);
    int kk = STARS_PAGER * 12;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            String S_NAME = OBJECTS[kk].substring(0, OBJECTS[kk].indexOf(','));
            // There are 100 Objects in the Alignment Star Catalogue
            if (S_NAME == "" || kk > 100)
            {
                break;
            }
            tft.fillRect(((j * 75) + 10), ((i * 40) + 90), 71, 35, BTN_L_BORDER);
            if (S_NAME.length() > 10)
            {
                String S_NAME_1 = S_NAME.substring(0, 9);
                String S_NAME_2 = S_NAME.substring(9, S_NAME.length());
                int l1 = (S_NAME_1.length() / 2) * 6;
                int l2 = (S_NAME_2.length() / 2) * 6;
                tft.setCursor(((j * 75) + (45 - l1)), ((i * 40) + 99));
                tft.print(S_NAME_1);
                tft.setCursor(((j * 75) + (45 - l2)), ((i * 40) + 109));
                tft.print(S_NAME_2);
            }
            else
            {
                int l = (S_NAME.length() / 2) * 6;
                tft.setCursor(((j * 75) + (45 - l)), ((i * 40) + 104));
                tft.print(S_NAME);
            }
            kk += 1;
        }
    }
}

void drawMainScreen()
{
    CURRENT_SCREEN = 3;
    IS_IN_OPERATION = true;
    UPDATE_TIME = millis();
    W_DATE_TIME[0] = 0;

    NOW = rtc.GetDateTime();
    calculateLST();

    tft.fillScreen(BLACK);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setCursor(10, 10);
    tft.setTextSize(2);
    if (NOW.Hour() < 10)
    {
        tft.print("0");
    }
    tft.print(NOW.Hour());
    tft.print(":");
    if (NOW.Minute() < 10)
    {
        tft.print("0");
    }
    tft.print(NOW.Minute());

    // Local Sidereal Time
    tft.setCursor(100, 10);
    tft.print("LST ");
    tft.print(hrs2hms(SCOPE.LST, false, false));

    tft.setTextSize(1);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 30);
    tft.print("LAT: ");
    tft.print(SCOPE.lnLatPos.lat, 2);
    tft.print(" LNG: ");
    tft.print(SCOPE.lnLatPos.lng, 2);
    tft.print(" ALT: ");
    tft.print(SCOPE.observationAlt, 0);
    tft.print("m");

    tft.setTextSize(2);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setCursor(10, 45);
    tft.println("TELESCOPE");

    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 70);
    tft.print("AZ: ");
    tft.setCursor(70, 70);
    tft.print(deg2dms(SCOPE.hrzPos.az, true, true));
    tft.setCursor(10, 90);
    tft.print("ALT: ");
    tft.setCursor(70, 90);
    tft.print(deg2dms(SCOPE.hrzPos.alt, true, false));

    tft.setTextSize(1);
    tft.setCursor(10, 110);
    tft.print("RA: ");
    tft.setCursor(70, 110);
    tft.print(deg2hms(SCOPE.equPos.ra, true, true));
    tft.setCursor(10, 120);
    tft.print("DEC: ");
    tft.setCursor(70, 120);
    tft.print(deg2dms(SCOPE.equPos.dec, true, false));

    if (CURRENT_OBJECT.name != "")
    {
        tft.setTextSize(2);
        tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
        tft.setCursor(10, 135);
        tft.print(CURRENT_OBJECT.name);

        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 155);
        if (CURRENT_OBJECT.description.length() > 20)
        {
            tft.print(CURRENT_OBJECT.description.substring(0, 19));
            int spacing = 230 - ((CURRENT_OBJECT.description.length() - 19) * 11);
            tft.setCursor(spacing, 170);
            tft.print("-" + CURRENT_OBJECT.description.substring(19, CURRENT_OBJECT.description.length()));
        }
        else
        {
            tft.print(CURRENT_OBJECT.description);
        }

        tft.setTextSize(1);
        tft.setCursor(10, 175);
        tft.print(CURRENT_OBJECT.constellation);
        tft.setCursor(10, 185);
        tft.print(CURRENT_OBJECT.type);

        if (CURRENT_OBJECT.hrzPos.alt < 0)
        {
            tft.setTextSize(1);
            tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
            tft.setCursor(10, 195);
            tft.println("OBJECT NOT VISIBLE!");
        }
        else
        {
            tft.setTextSize(1);
            tft.setTextColor(L_TEXT);
            tft.setCursor(10, 195);
            tft.print("MAG:  ");
            tft.print(CURRENT_OBJECT.mag);
            tft.setCursor(120, 195);
            if (LOADED_OBJECTS == 1)
                tft.print("DIST: " + CURRENT_OBJECT.size);
            else
                tft.print("SIZE: " + CURRENT_OBJECT.size);
        }

        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);

        tft.setCursor(10, 205);
        tft.print("AZ: ");
        tft.setCursor(70, 205);
        tft.print(deg2dms(CURRENT_OBJECT.hrzPos.az, true, true));
        tft.setCursor(10, 225);
        tft.print("ALT: ");
        tft.setCursor(70, 225);
        tft.print(deg2dms(CURRENT_OBJECT.hrzPos.alt, true, false));

        tft.setTextSize(1);
        tft.setCursor(10, 245);
        tft.print("RA: ");
        tft.setCursor(70, 245);
        tft.print(deg2hms(CURRENT_OBJECT.equPos.ra, true, true));
        tft.setCursor(10, 255);
        tft.print("DEC: ");
        tft.setCursor(70, 255);
        tft.print(deg2dms(CURRENT_OBJECT.equPos.dec, true, false));
    }
    else
    {
        tft.setTextSize(2);
        tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
        tft.setCursor(10, 135);
        tft.println("NO TARGET SELECTED!");
    }

    drawButton(10, 270, 65, 40, "FIND", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(88, 270, 65, 40, "PLNT", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(165, 270, 65, 40, "MENU", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawCatalogueScreen()
{
    CURRENT_SCREEN = 12;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" FIND");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(20, 50, 200, 40, "MESSIER", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 110, 200, 40, "CALDWELL", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 170, 200, 40, "TREASURES", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 230, 200, 40, "BRIGHT STARS", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawCatalogueSummaryScreen()
{
    CURRENT_SCREEN = 13;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" CATALOGUE");

    if (LOAD_SELECTOR == 1)
    {
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 40);
        tft.setTextSize(2);
        tft.print("Bright Stars");

        tft.setCursor(10, 60);
        tft.setTextSize(1);
        tft.print("100 of the Brightest Stars");
        tft.setCursor(10, 70);
        tft.print("in the sky");
    }
    else if (LOAD_SELECTOR == 2)
    {
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 40);
        tft.setTextSize(2);
        tft.print("Messier");

        tft.setCursor(10, 60);
        tft.setTextSize(1);
        tft.print("110 Objects catalogued by Charles");
        tft.setCursor(10, 70);
        tft.print("Messier in 1771-81");

        tft.setTextSize(2);
        tft.setCursor(10, 90);
        tft.print("Asterisms:");
        tft.setCursor(210, 90);
        tft.print("1");

        tft.setCursor(10, 110);
        tft.print("Glob. Clst.");
        tft.setCursor(210, 110);
        tft.print("29");

        tft.setCursor(10, 130);
        tft.print("Open Clst.");
        tft.setCursor(210, 130);
        tft.print("26");

        tft.setCursor(10, 150);
        tft.print("Galaxies");
        tft.setCursor(210, 150);
        tft.print("40");

        tft.setCursor(10, 170);
        tft.print("Diffuse Neb.");
        tft.setCursor(210, 170);
        tft.print("7");

        tft.setCursor(10, 190);
        tft.print("Planetary Neb.");
        tft.setCursor(210, 190);
        tft.print("4");

        tft.setCursor(10, 210);
        tft.print("Supernova Rmnt.");
        tft.setCursor(210, 210);
        tft.print("1");

        tft.setCursor(10, 230);
        tft.print("Star Cloud");
        tft.setCursor(210, 230);
        tft.print("1");

        tft.setCursor(10, 250);
        tft.print("Double Star");
        tft.setCursor(210, 250);
        tft.print("1");
    }
    else if (LOAD_SELECTOR == 3)
    {
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 40);
        tft.setTextSize(2);
        tft.print("Caldwell");

        tft.setCursor(10, 60);
        tft.setTextSize(1);
        tft.print("109 Objects catalogued by Sir");
        tft.setCursor(10, 70);
        tft.print("Patrick Moore in 1995");

        tft.setTextSize(2);
        tft.setCursor(10, 90);
        tft.print("Glob. Clst.");
        tft.setCursor(210, 90);
        tft.print("18");

        tft.setCursor(10, 110);
        tft.print("Open Clst.");
        tft.setCursor(210, 110);
        tft.print("25");

        tft.setCursor(10, 130);
        tft.print("Galaxies");
        tft.setCursor(210, 130);
        tft.print("35");

        tft.setCursor(10, 150);
        tft.print("Dark Neb.");
        tft.setCursor(210, 150);
        tft.print("1");

        tft.setCursor(10, 170);
        tft.print("Diffuse Neb.");
        tft.setCursor(210, 170);
        tft.print("15");

        tft.setCursor(10, 190);
        tft.print("Planetary Neb.");
        tft.setCursor(210, 190);
        tft.print("13");

        tft.setCursor(10, 210);
        tft.print("Supernova Rmnt.");
        tft.setCursor(210, 210);
        tft.print("2");
    }
    else if (LOAD_SELECTOR == 4)
    {
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 40);
        tft.setTextSize(2);
        tft.print("Treasures");

        tft.setCursor(10, 60);
        tft.setTextSize(1);
        tft.print("128 Objects catalogued by");
        tft.setCursor(10, 70);
        tft.print("Stephen O'Meara in 2007");

        tft.setTextSize(2);
        tft.setCursor(10, 90);
        tft.print("Asterisms");
        tft.setCursor(210, 90);
        tft.print("4");

        tft.setCursor(10, 110);
        tft.print("Glob. Clst.");
        tft.setCursor(210, 110);
        tft.print("11");

        tft.setCursor(10, 130);
        tft.print("Open Clst.");
        tft.setCursor(210, 130);
        tft.print("43");

        tft.setCursor(10, 150);
        tft.print("Galaxies");
        tft.setCursor(210, 150);
        tft.print("39");

        tft.setCursor(10, 170);
        tft.print("Dark Neb.");
        tft.setCursor(210, 170);
        tft.print("1");

        tft.setCursor(10, 190);
        tft.print("Diffuse Neb.");
        tft.setCursor(210, 190);
        tft.print("10");

        tft.setCursor(10, 210);
        tft.print("Planetary Neb.");
        tft.setCursor(210, 210);
        tft.print("20");
    }
    drawButton(10, 270, 100, 40, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 270, 100, 40, "CONT.", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawLoadScreen()
{
    CURRENT_SCREEN = 4;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" FIND");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 270, 100, 40, "< PREV", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 270, 100, 40, "NEXT >", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawLoadObjects();
}

void drawLoadObjects()
{
    tft.fillRect(0, 90, 240, 170, BLACK);
    // Brightest Stars Screen
    if (LOAD_SELECTOR == 1)
    {
        if (LOADED_OBJECTS != 1)
        {
            loadStarsFromSPIFFS("/alignment.csv");
            LOADED_OBJECTS = 1;
        }
        drawLoadAlignStars();
    }
    /////// Messier Screen /////////////
    else if (LOAD_SELECTOR == 2)
    {
        if (LOADED_OBJECTS != 2)
        {
            loadStarsFromSPIFFS("/messier.csv");
            LOADED_OBJECTS = 2;
        }
        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 50);
        tft.print("Messier Catalogue");

        tft.setTextSize(1);
        int kk = MESS_PAGER * 12;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                String M_NAME = OBJECTS[kk].substring(0, OBJECTS[kk].indexOf(','));
                // There are 110 Objects in the Messier Catalogue
                if (M_NAME == "" || kk > 110)
                {
                    break;
                }
                drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                kk += 1;
            }
        }
    }
    ///////     Caldwell Screen  /////////////
    else if (LOAD_SELECTOR == 3)
    {
        if (LOADED_OBJECTS != 3)
        {
            loadStarsFromSPIFFS("/caldwell.csv");
            LOADED_OBJECTS = 3;
        }
        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 50);
        tft.print("Caldwell Catalogue");

        tft.setTextSize(1);
        int ll = CALD_PAGER * 12;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                String M_NAME = OBJECTS[ll].substring(0, OBJECTS[ll].indexOf(','));
                // There are 109 Objects in the Caldwell Catalogue
                if (M_NAME == "" || ll > 109)
                {
                    break;
                }
                drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                ll += 1;
            }
        }
    }
    ///////     Treasures Screen /////////////
    else if (LOAD_SELECTOR == 4)
    {
        if (LOADED_OBJECTS != 4)
        {
            loadStarsFromSPIFFS("/treasure.csv");
            LOADED_OBJECTS = 4;
        }
        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 50);
        tft.print("Hidden Treasures");

        tft.setTextSize(1);
        int mm = TREAS_PAGER * 12;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                String M_NAME = OBJECTS[mm].substring(0, OBJECTS[mm].indexOf(','));
                // There are 128 Objects in the Hidden Treasures Catalogue
                if (M_NAME == "" || mm > 128)
                {
                    break;
                }
                drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, BTN_L_BORDER, 0, BTN_BLK_TEXT, 1);
                mm += 1;
            }
        }
    }
}

void drawObjectSummaryScreen()
{
    CURRENT_SCREEN = 14;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" OBJECT");

    if (CURRENT_OBJECT.name != "")
    {
        tft.setTextSize(2);
        tft.setCursor(10, 50);
        tft.setTextColor(L_TEXT);
        tft.print(CURRENT_OBJECT.name);
        
        tft.setCursor(10, 70);
        if (CURRENT_OBJECT.description.length() > 20)
        {
            tft.print(CURRENT_OBJECT.description.substring(0, 19));
            tft.setCursor(10, 90);
            tft.print("-" + CURRENT_OBJECT.description.substring(19, CURRENT_OBJECT.description.length()));
        }
        else
        {
            tft.print(CURRENT_OBJECT.description);
        }
        tft.setTextSize(1);
        tft.setCursor(10, 110);
        tft.print(CURRENT_OBJECT.constellation);
        tft.setCursor(10, 120);
        tft.print(CURRENT_OBJECT.type);
        tft.setCursor(10, 130);
        tft.print("MAG: " + CURRENT_OBJECT.mag);
        tft.setCursor(120, 130);
        if (LOADED_OBJECTS == 1)
            tft.print("DIST: " + CURRENT_OBJECT.size);
        else
            tft.print("SIZE: " + CURRENT_OBJECT.size);
        tft.setCursor(10, 140);
        tft.print(CURRENT_OBJECT.riseSetStr);

        tft.setTextSize(2);
        if (CURRENT_OBJECT.hrzPos.alt < 0)
        {
            tft.setCursor(10, 150);
            tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
            tft.println("OBJECT NOT VISIBLE!");
            tft.setTextColor(L_TEXT);
        }

        tft.setCursor(10, 170);
        tft.print("RA: ");
        tft.setCursor(70, 170);
        tft.print(deg2hms(CURRENT_OBJECT.equPos.ra, true, true));
        tft.setCursor(10, 190);
        tft.print("DEC: ");
        tft.setCursor(70, 190);
        tft.print(deg2dms(CURRENT_OBJECT.equPos.dec, true, false));

        tft.setCursor(10, 210);
        tft.print("AZ: ");
        tft.setCursor(70, 210);
        tft.print(deg2dms(CURRENT_OBJECT.hrzPos.az, true, true));
        tft.setCursor(10, 230);
        tft.print("ALT: ");
        tft.setCursor(70, 230);
        tft.print(deg2dms(CURRENT_OBJECT.hrzPos.alt, true, false));
    }
    else
    {
        tft.setTextSize(2);
        tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
        tft.setCursor(10, 50);
        tft.println("NO TARGET SELECTED!");
    }

    drawButton(10, 270, 100, 40, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 270, 100, 40, "SELECT", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawOptionsScreen()
{
    CURRENT_SCREEN = 5;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" OPTIONS");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    // Toggle BT and WiFi
    tft.setCursor(10, 50);
    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.print("Remote Modes");

    drawButton(10, 70, 65, 40, "BT", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(85, 70, 65, 40, "WIFI", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    // Adjust configurations
    tft.setCursor(10, 130);
    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.print("Configuration");

    drawButton(10, 150, 65, 40, "ALIGN", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(85, 150, 65, 40, "GPS", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(165, 150, 65, 40, "CLOCK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    tft.setCursor(10, 210);
    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.print("Corrections");

    drawButton(10, 230, 65, 40, "CORR.", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawAlignScreen()
{
    CURRENT_SCREEN = 7;
    uint8_t starNum = ALIGN_STEP - 1;

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" ALIGN");

    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 50);
    tft.print(ALIGNMENT_STARS[starNum].name);

    tft.setTextSize(1);
    tft.setCursor(10, 70);
    tft.print(ALIGNMENT_STARS[starNum].description);
    tft.setCursor(10, 80);
    tft.print(ALIGNMENT_STARS[starNum].constellation);
    tft.setCursor(10, 90);
    tft.print(ALIGNMENT_STARS[starNum].type);
    tft.setCursor(10, 100);
    tft.print("MAG: " + ALIGNMENT_STARS[starNum].mag);
    tft.setCursor(120, 100);
    tft.print("DIST: " + ALIGNMENT_STARS[starNum].size);
    tft.setCursor(10, 110);
    tft.print(ALIGNMENT_STARS[starNum].riseSetStr);

    tft.setTextSize(2);
    if (CURRENT_OBJECT.hrzPos.alt < 0)
    {
        tft.setCursor(10, 140);
        tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
        tft.println("OBJECT NOT VISIBLE!");
        tft.setTextColor(L_TEXT);
    }

    tft.setTextSize(2);
    tft.setCursor(10, 170);
    tft.print("RA: ");
    tft.setCursor(70, 170);
    tft.print(deg2hms(ALIGNMENT_STARS[starNum].equPos.ra, true, true));
    tft.setCursor(10, 190);
    tft.print("DEC: ");
    tft.setCursor(70, 190);
    tft.print(deg2dms(ALIGNMENT_STARS[starNum].equPos.dec, true, false));

    tft.setCursor(10, 210);
    tft.print("AZ: ");
    tft.setCursor(70, 210);
    tft.print(deg2dms(ALIGNMENT_STARS[starNum].hrzPos.az, true, true));
    tft.setCursor(10, 230);
    tft.print("ALT: ");
    tft.setCursor(70, 230);
    tft.print(deg2dms(ALIGNMENT_STARS[starNum].hrzPos.alt, true, false));

    if (ALIGN_TYPE == 2)
    {
        drawButton(10, 270, 100, 40, "< CANCEL", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    }
    else
    {
        drawButton(10, 270, 100, 40, "< BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    }

    drawButton(130, 270, 100, 40, "ALIGN", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawRemoteScreen()
{
    CURRENT_SCREEN = 8;
    IS_IN_OPERATION = false;

    tft.fillScreen(BLACK);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.setCursor(10, 10);

    if (IS_BT_MODE_ON)
    {
        tft.print(" BT");
        // Init Bluetooth
        SerialBT.begin("Skywatcher-BLE");
    }
    else if (IS_WIFI_MODE_ON)
    //if (IS_WIFI_MODE_ON)
    {
        tft.print(" WIFI");
    }

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    tft.setTextColor(L_TEXT);
    tft.setTextSize(2);
    tft.setCursor(10, 50);
    tft.print("Currently slaved to");
    tft.setCursor(10, 70);
    tft.print("wireless app");
}

void drawPlanetScreen()
{
    CURRENT_SCREEN = 9;
    IS_IN_OPERATION = false;

    tft.fillScreen(BLACK);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.setCursor(10, 10);
    tft.print(" PLANETS");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 50, 100, 40, "MERCURY", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 50, 100, 40, "VENUS", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 110, 100, 40, "MARS", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 110, 100, 40, "JUPITER", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 170, 100, 40, "SATURN", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 170, 100, 40, "URANUS", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 230, 100, 40, "NEPTUNE", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 230, 100, 40, "PLUTO", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void OnScreenMsg(int Msg)
{
    tft.fillRect(40, 110, 160, 100, MSG_BOX_BG);
    tft.drawRect(40, 110, 160, 100, MSG_BOX_TEXT);
    tft.setTextColor(MSG_BOX_TEXT);
    if (Msg == 1)
    {
        tft.setCursor(70, 140);
        tft.setTextSize(3);
        tft.print("ERROR!");
        tft.setCursor(55, 170);
        tft.setTextSize(2);
        tft.print("Not Visible");
    }
}

void TimerUpdateDraw(int z)
{
    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    W_DATE_TIME[DATE_ENTRY_POS] = z;
    if (DATE_ENTRY_POS >= 0 && DATE_ENTRY_POS < 2)
    {
        tft.fillRect((DATE_ENTRY_POS * 12) + 70, 50, 12, 16, BLACK);
        tft.setCursor((DATE_ENTRY_POS * 12) + 70, 50);
    }
    else if (DATE_ENTRY_POS > 1 && DATE_ENTRY_POS < 4)
    {
        tft.fillRect((DATE_ENTRY_POS * 12) + 82, 50, 12, 16, BLACK);
        tft.setCursor((DATE_ENTRY_POS * 12) + 82, 50);
    }
    else if (DATE_ENTRY_POS > 3 && DATE_ENTRY_POS < 8)
    {
        tft.fillRect((DATE_ENTRY_POS * 12) + 94, 50, 12, 16, BLACK);
        tft.setCursor((DATE_ENTRY_POS * 12) + 94, 50);
    }
    else if (DATE_ENTRY_POS > 7 && DATE_ENTRY_POS < 10)
    {
        tft.fillRect(((DATE_ENTRY_POS % 8) * 12) + 80, 70, 12, 16, BLACK);
        tft.setCursor(((DATE_ENTRY_POS % 8) * 12) + 80, 70);
    }
    else if (DATE_ENTRY_POS > 9)
    {
        tft.fillRect(((DATE_ENTRY_POS % 8) * 12) + 92, 70, 12, 16, BLACK);
        tft.setCursor(((DATE_ENTRY_POS % 8) * 12) + 92, 70);
    }
    tft.print(W_DATE_TIME[DATE_ENTRY_POS]);
    if (DATE_ENTRY_POS > 10)
    {
        DATE_ENTRY_POS = 0;
    }
    else
    {
        DATE_ENTRY_POS += 1;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// TOUCH SCREEN AND INPUT
//  - Please note, that Touches are separated in 2 section to capture OnPress && OnRelease!
//    You will notice that "if (lx > 0 && ly > 0 )" this part defines OnPress activities.

void considerTouchInput(int lx, int ly)
{
    //**************************************************************
    //
    //      BUTTON DOWN Events start here
    //
    //      - only executed when the user touches the screen - PRESS
    //**************************************************************
    if (lx > 0 && ly > 0)
    {
        if (CURRENT_SCREEN == 0)
        { // captures touches on drawGPSScreen()
            if (lx > 40 && lx < 200 && ly > 270 && ly < 310)
            {
                LAST_BUTTON = 1;
                delay(150);
                drawButton(40, 270, 160, 40, "SKIP", 0, BTN_L_BORDER, L_TEXT, 2);
            }
        }
        else if (CURRENT_SCREEN == 1)
        { // captures touches on drawClockScreen()
            if (lx > 165 && lx < 230 && ly > 270 && ly < 310)
            {
                // BTN OK pressed
                drawButton(160, 270, 65, 40, "OK", 0, BTN_L_BORDER, L_TEXT, 2);
                int changes = 0;
                for (int y = 0; y < 12; y++)
                {
                    if (W_DATE_TIME[y] != 0)
                    {
                        changes = 1;
                    }
                }
                if (changes == 1)
                {
                    // Do the magic as the date and time has been updated... Update the RTC accordingly
                    int hh = (W_DATE_TIME[8] * 10) + W_DATE_TIME[9];
                    int mm = (W_DATE_TIME[10] * 10) + W_DATE_TIME[11];
                    int dd = (W_DATE_TIME[0] * 10) + W_DATE_TIME[1];
                    int mo = (W_DATE_TIME[2] * 10) + W_DATE_TIME[3];
                    int yy = (W_DATE_TIME[4] * 1000) + (W_DATE_TIME[5] * 100) + (W_DATE_TIME[6] * 10) + W_DATE_TIME[7];
                    rtc.SetDateTime(RtcDateTime(yy, mo, dd, hh, mm, 00));
                }

                NOW = rtc.GetDateTime();
                setTime(NOW.Hour(), NOW.Minute(), NOW.Second(), NOW.Day(), NOW.Month(), NOW.Year());
                SUMMER_TIME = isSummerTime();
                calculateLST();
                delay(150);
                // Load Alignment Stars
                drawAlignCorrectionsScreen();
#ifdef SERIAL_DEBUG
                Serial.print("Date/Time: ");
                Serial.print(NOW.Year());
                Serial.print("/");
                Serial.print(NOW.Month());
                Serial.print("/");
                Serial.print(NOW.Day());
                Serial.print(" - ");
                Serial.print(NOW.Hour());
                Serial.print(":");
                Serial.print(NOW.Minute());
                Serial.print(":");
                Serial.print(NOW.Second());
                Serial.println("");
#endif
            }
            else if (lx > 10 && lx < 75 && ly > 120 && ly < 160)
            {
                // BTN 1 pressed
                tft.drawRect(10, 120, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 1;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 85 && lx < 150 && ly > 120 && ly < 160)
            {
                // BTN 2 pressed
                tft.drawRect(85, 120, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 2;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 160 && lx < 225 && ly > 120 && ly < 160)
            {
                // BTN 3 pressed
                tft.drawRect(160, 120, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 3;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 10 && lx < 75 && ly > 170 && ly < 210)
            {
                // BTN 4 pressed
                tft.drawRect(10, 170, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 4;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 85 && lx < 150 && ly > 170 && ly < 210)
            {
                // BTN 5 pressed
                tft.drawRect(85, 170, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 5;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 160 && lx < 225 && ly > 170 && ly < 210)
            {
                // BTN 6 pressed
                tft.drawRect(160, 170, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 6;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 10 && lx < 75 && ly > 220 && ly < 260)
            {
                // BTN 7 pressed
                tft.drawRect(10, 220, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 7;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 85 && lx < 150 && ly > 220 && ly < 260)
            {
                // BTN 8 pressed
                tft.drawRect(85, 220, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 8;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 160 && lx < 225 && ly > 220 && ly < 260)
            {
                // BTN 9 pressed
                tft.drawRect(160, 220, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 9;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 85 && lx < 150 && ly > 270 && ly < 310)
            {
                // BTN 0 pressed
                tft.drawRect(85, 270, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 10;
                delay(150);
                removeTime_addXX();
            }
            else if (lx > 10 && lx < 75 && ly > 270 && ly < 310)
            {
                // BTN SummerTime pressed
                tft.drawRect(10, 270, 65, 40, BTN_L_BORDER);
                LAST_BUTTON = 22;
                delay(150);
            }
        }
        else if (CURRENT_SCREEN == 11)
        {
            // On drawAlignCorrectionsScreen() Screen
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN <Back pressed// BTN <Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                if (ALIGN_READY)
                    drawMainScreen();
                else
                    drawAlignCorrectionsScreen();
            }
            if (lx > 20 && lx < 220 && ly > 50 && ly < 90)
            {
                // BTN "REFRACTION" pressed
                CORRECT_REFRACTION = !CORRECT_REFRACTION;
                if (CORRECT_REFRACTION)
                    drawButton(20, 50, 200, 40, "REFRACTION", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                else
                    drawButton(20, 50, 200, 40, "REFRACTION", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
            }
            else if (lx > 20 && lx < 220 && ly > 110 && ly < 150)
            {
                // BTN "ORBITAL ANOMS." pressed
                CORRECT_PRECESSION_ETC = !CORRECT_PRECESSION_ETC;
                if (CORRECT_PRECESSION_ETC)
                    drawButton(20, 110, 200, 40, "ORBITAL ANOMS.", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                else
                    drawButton(20, 110, 200, 40, "ORBITAL ANOMS.", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
            }
            else if (lx > 20 && lx < 220 && ly > 170 && ly < 210)
            {
                // BTN "MOUNT ERRORS" pressed
                CORRECT_MOUNT_ERRS = !CORRECT_MOUNT_ERRS;
                if (CORRECT_MOUNT_ERRS)
                    drawButton(20, 170, 200, 40, "MOUNT ERRORS", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                else
                    drawButton(20, 170, 200, 40, "MOUNT ERRORS", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
            }
            else if (lx > 20 && lx < 220 && ly > 270 && ly < 310)
            {
                // BTN "Begin Align" pressed
                if (!ALIGN_READY) // Don't let us align if we're just checking errors
                {
                    drawButton(20, 270, 200, 40, "BEGIN ALIGN", 0, BTN_L_BORDER, L_TEXT, 2);
                    delay(150);
                    drawSelectAlignment();
                }
            }
        }
        else if (CURRENT_SCREEN == 2)
        {
            // On drawSelectAlignment() Screen
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN <Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawAlignCorrectionsScreen();
            }
            if (lx > 20 && lx < 220 && ly > 50 && ly < 90)
            {
                // BTN "2-Star Auto" pressed
                ALIGN_TYPE = 1;
                NUM_ALIGNMENT_STARS = 2;
                drawButton(20, 50, 200, 40, "2-STAR AUTO", 0, BTN_L_BORDER, L_TEXT, 2);
                autoSelectAlignmentStars();
                delay(150);
                drawAlignScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 110 && ly < 150)
            {
                // BTN "3-Star Auto" pressed
                ALIGN_TYPE = 1;
                NUM_ALIGNMENT_STARS = 3;
                drawButton(20, 110, 200, 40, "3-STAR AUTO", 0, BTN_L_BORDER, L_TEXT, 2);
                autoSelectAlignmentStars();
                delay(150);
                drawAlignScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 170 && ly < 210)
            {
                // BTN "2-Star Manual" pressed
                ALIGN_TYPE = 2;
                NUM_ALIGNMENT_STARS = 2;
                drawButton(20, 170, 200, 40, "2-STAR MANUAL", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawStarSelectScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 230 && ly < 270)
            {
                // BTN "3-Star Manual" pressed
                ALIGN_TYPE = 2;
                NUM_ALIGNMENT_STARS = 3;
                drawButton(20, 230, 200, 40, "3-STAR MANUAL", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawStarSelectScreen();
            }
        }
        else if (CURRENT_SCREEN == 3)
        { // captures touches on drawMainScreen()
            // Find Button - Go to Load screen to search for targets
            if (lx > 10 && lx < 75 && ly > 270 && ly < 310)
            {
                drawButton(10, 270, 65, 40, "FIND", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawCatalogueScreen();
            }
            if (lx > 88 && lx < 153 && ly > 270 && ly < 310)
            {
                drawButton(88, 270, 65, 40, "PLNT", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawPlanetScreen();
            }
            // Options Button Touched
            if (lx > 165 && lx < 230 && ly > 270 && ly < 310)
            {
                drawButton(165, 270, 65, 40, "MENU", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawOptionsScreen();
            }
        }
        else if (CURRENT_SCREEN == 4)
        { // captures touches on drawLoadScreen() .. the one that loads objects from DB
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawCatalogueScreen();
            }
            //////////////      Messier Screen //////////////
            if (lx > 130 && lx < 230 && ly > 270 && ly < 310)
            {
                // BTN next> pressed
                LAST_BUTTON = 1;
                drawButton(130, 270, 100, 40, "NEXT >", 0, BTN_L_BORDER, L_TEXT, 2);
                if (LOAD_SELECTOR == 1)
                {
                    STARS_PAGER += 1;
                    if (STARS_PAGER >= 10)
                    {
                        STARS_PAGER = 9;
                    }
                }
                else if (LOAD_SELECTOR == 2)
                {
                    MESS_PAGER += 1;
                    if (MESS_PAGER >= 10)
                    {
                        MESS_PAGER = 9;
                    }
                }
                else if (LOAD_SELECTOR == 3)
                {
                    CALD_PAGER += 1;
                    if (CALD_PAGER >= 10)
                    {
                        CALD_PAGER = 9;
                    }
                }
                else if (LOAD_SELECTOR == 4)
                {
                    TREAS_PAGER += 1;
                    if (TREAS_PAGER >= 11)
                    {
                        TREAS_PAGER = 10;
                    }
                }
                delay(150);
                drawLoadObjects();
            }
            if (lx > 10 && lx < 110 && ly > 270 && ly < 310)
            {
                // BTN <prev pressed
                LAST_BUTTON = 2;
                drawButton(10, 270, 100, 40, "< PREV", 0, BTN_L_BORDER, L_TEXT, 2);
                if (LOAD_SELECTOR == 1)
                {
                    STARS_PAGER -= 1;
                    if (STARS_PAGER < 0)
                    {
                        STARS_PAGER = 0;
                    }
                }
                else if (LOAD_SELECTOR == 2)
                {
                    MESS_PAGER -= 1;
                    if (MESS_PAGER < 0)
                    {
                        MESS_PAGER = 0;
                    }
                }
                else if (LOAD_SELECTOR == 3)
                {
                    CALD_PAGER -= 1;
                    if (CALD_PAGER < 0)
                    {
                        CALD_PAGER = 0;
                    }
                }
                else if (LOAD_SELECTOR == 4)
                {
                    TREAS_PAGER -= 1;
                    if (TREAS_PAGER < 0)
                    {
                        TREAS_PAGER = 0;
                    }
                }
                delay(150);
                drawLoadObjects();
            }
            ///////     Bright Star Screen /////////////
            if (LOAD_SELECTOR == 1)
            {
                if (LOADED_OBJECTS != 1)
                {
                    loadStarsFromSPIFFS("/alignment.csv");
                    LOADED_OBJECTS = 1;
                }
                tft.setTextColor(L_TEXT);
                tft.setTextSize(1);
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (lx > ((j * 75) + 10) && lx < ((j * 75) + 81) && ly > ((i * 40) + 90) && ly < ((i * 40) + 130))
                        {
                            int zz = ((STARS_PAGER * 12) + (i * 3) + j);
                            String S_NAME = OBJECTS[zz].substring(0, OBJECTS[zz].indexOf(','));
                            tft.fillRect(((j * 75) + 10), ((i * 40) + 90), 71, 35, BTN_BLK_TEXT);
                            tft.drawRect(((j * 75) + 10), ((i * 40) + 90), 71, 35, L_TEXT);
                            if (S_NAME.length() > 10)
                            {
                                String S_NAME_1 = S_NAME.substring(0, 9);
                                String S_NAME_2 = S_NAME.substring(9, S_NAME.length());
                                int l1 = (S_NAME_1.length() / 2) * 6;
                                int l2 = (S_NAME_2.length() / 2) * 6;
                                tft.setCursor(((j * 75) + (45 - l1)), ((i * 40) + 99));
                                tft.print(S_NAME_1);
                                tft.setCursor(((j * 75) + (45 - l2)), ((i * 40) + 109));
                                tft.print(S_NAME_2);
                            }
                            else
                            {
                                int l = (S_NAME.length() / 2) * 6;
                                tft.setCursor(((j * 75) + (45 - l)), ((i * 40) + 104));
                                tft.print(S_NAME);
                            }
                            if (OBJECTS[zz] != "")
                            {
                                selectObject(zz, 1);
                                delay(150);
                                drawObjectSummaryScreen();
                            }
                        }
                    }
                }
            }
            ///////     Messier Screen /////////////
            else if (LOAD_SELECTOR == 2)
            {
                if (LOADED_OBJECTS != 2)
                {
                    loadStarsFromSPIFFS("/messier.csv");
                    LOADED_OBJECTS = 2;
                }
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (lx > ((j * 75) + 10) && lx < ((j * 75) + 81) && ly > ((i * 40) + 90) && ly < ((i * 40) + 130))
                        {
                            int zz = ((MESS_PAGER * 12) + (i * 3) + j);
                            String M_NAME = OBJECTS[zz].substring(0, OBJECTS[zz].indexOf(','));
                            drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, 0, BTN_L_BORDER, L_TEXT, 2);
                            if (OBJECTS[zz] != "")
                            {
                                selectObject(zz, 2);
                                delay(150);
                                drawObjectSummaryScreen();
                            }
                        }
                    }
                }
            }
            ///////     Caldwell Screen /////////////
            else if (LOAD_SELECTOR == 3)
            {
                if (LOADED_OBJECTS != 3)
                {
                    loadStarsFromSPIFFS("/caldwell.csv");
                    LOADED_OBJECTS = 3;
                }
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (lx > ((j * 75) + 10) && lx < ((j * 75) + 81) && ly > ((i * 40) + 90) && ly < ((i * 40) + 130))
                        {
                            int zz = ((CALD_PAGER * 12) + (i * 3) + j);
                            String M_NAME = OBJECTS[zz].substring(0, OBJECTS[zz].indexOf(','));
                            drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, 0, BTN_L_BORDER, L_TEXT, 2);
                            if (OBJECTS[zz] != "")
                            {
                                selectObject(zz, 3);
                                delay(150);
                                drawObjectSummaryScreen();
                            }
                        }
                    }
                }
            }
            ///////     Treasures Screen /////////////
            else if (LOAD_SELECTOR == 4)
            {
                // I'm in TREASURES selector and need to check which Treasure object is pressed
                if (LOADED_OBJECTS != 4)
                {
                    loadStarsFromSPIFFS("/treasure.csv");
                    LOADED_OBJECTS = 4;
                }
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (lx > ((j * 75) + 10) && lx < ((j * 75) + 81) && ly > ((i * 40) + 90) && ly < ((i * 40) + 130))
                        {
                            int zz = ((TREAS_PAGER * 12) + (i * 3) + j);
                            String M_NAME = OBJECTS[zz].substring(0, OBJECTS[zz].indexOf(','));
                            drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, 0, BTN_L_BORDER, L_TEXT, 1);
                            if (OBJECTS[zz] != "")
                            {
                                selectObject(zz, 4);
                                delay(150);
                                drawObjectSummaryScreen();
                            }
                        }
                    }
                }
            }
        }
        else if (CURRENT_SCREEN == 12)
        { // captures touches on drawCatalogueScreen()
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN <Back pressed// BTN <Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawMainScreen();
            }
            if (lx > 20 && lx < 220 && ly > 50 && ly < 90)
            {
                LOAD_SELECTOR = 2;
                drawButton(20, 50, 200, 40, "MESSIER", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawCatalogueSummaryScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 110 && ly < 150)
            {
                LOAD_SELECTOR = 3;
                drawButton(20, 110, 200, 40, "CALDWELL", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawCatalogueSummaryScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 170 && ly < 210)
            {
                LOAD_SELECTOR = 4;
                drawButton(20, 170, 200, 40, "TREASURES", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawCatalogueSummaryScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 230 && ly < 270)
            {
                LOAD_SELECTOR = 1;
                drawButton(20, 230, 200, 40, "BRIGHT STARS", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawCatalogueSummaryScreen();
            }
        }
        else if (CURRENT_SCREEN == 13)
        { // captures touches on drawCatalogueSummaryScreen()
            if (lx > 10 && lx < 110 && ly > 270 && ly < 310)
            {
                // BTN <Back pressed// BTN <Back pressed
                drawButton(10, 270, 100, 40, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawCatalogueScreen();
            }
            else if (lx > 130 && lx < 230 && ly > 270 && ly < 310)
            {
                // BTN "Continue" pressed
                drawButton(130, 270, 100, 40, "CONT.", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawLoadScreen();
            }
        }
        else if (CURRENT_SCREEN == 14)
        { // captures touches on drawObjectSummaryScreen()
            if (lx > 10 && lx < 110 && ly > 270 && ly < 310)
            {
                // BTN <Back pressed// BTN "BACK" pressed
                drawButton(10, 270, 100, 40, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawLoadScreen();
            }
            else if (lx > 130 && lx < 230 && ly > 270 && ly < 310)
            {
                // BTN "SELECT" pressed
                drawButton(130, 270, 100, 40, "SELECT", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawMainScreen();
            }
        }
        else if (CURRENT_SCREEN == 5)
        {
            // captures touches on drawOptionsScreen()
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN <Back pressed// BTN <Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 75 && ly > 70 && ly < 110)
            {
                // Touched BT Toggle
                tft.drawRect(10, 70, 65, 40, BLACK);
                drawButton(10, 70, 65, 40, "BT", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                IS_BT_MODE_ON = true;
                drawRemoteScreen();
            }
            if (lx > 85 && lx < 150 && ly > 70 && ly < 110)
            {
                // Touched WiFi Toggle
                tft.drawRect(85, 70, 65, 40, BLACK);
                drawButton(85, 70, 65, 40, "WIFI", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                IS_WIFI_MODE_ON = true;
                drawRemoteScreen();
            }
            if (lx > 10 && lx < 75 && ly > 150 && ly < 190)
            {
                // Touched Align Button
                drawButton(10, 150, 65, 40, "ALIGN", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawSelectAlignment();
            }
            if (lx > 85 && lx < 150 && ly > 150 && ly < 190)
            {
                //Touched GPS configuration
                drawButton(85, 150, 65, 40, "GPS", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawGPSScreen();
            }
            if (lx > 165 && lx < 230 && ly > 150 && ly < 190)
            {
                //Touched Clock configuration
                drawButton(165, 130, 65, 40, "CLOCK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawClockScreen();
            }
            if (lx > 10 && lx < 75 && ly > 230 && ly < 270)
            {
                drawButton(10, 230, 65, 40, "CORR.", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawAlignCorrectionsScreen();
            }
        }
        else if (CURRENT_SCREEN == 6) // captures touches on drawStarSelectScreen()
        {
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN Done pressed
                drawButton(165, 10, 65, 30, "EXIT", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 270 && ly < 310)
            {
                // BTN next> pressed
                LAST_BUTTON = 1;
                drawButton(130, 270, 100, 40, "NEXT >", 0, BTN_L_BORDER, L_TEXT, 2);
                STARS_PAGER += 1;
                if (STARS_PAGER >= 10)
                {
                    STARS_PAGER = 9;
                }
                delay(150);
                drawLoadObjects();
            }
            if (lx > 10 && lx < 110 && ly > 270 && ly < 310)
            {
                // BTN <prev pressed
                LAST_BUTTON = 2;
                drawButton(10, 270, 100, 40, "< PREV", 0, BTN_L_BORDER, L_TEXT, 2);
                STARS_PAGER -= 1;
                if (STARS_PAGER < 0)
                {
                    STARS_PAGER = 0;
                }
                delay(150);
                drawLoadObjects();
            }
            tft.setTextColor(L_TEXT);
            tft.setTextSize(1);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (lx > ((j * 75) + 10) && lx < ((j * 75) + 81) && ly > ((i * 40) + 90) && ly < ((i * 40) + 130))
                    {
                        int zz = ((STARS_PAGER * 12) + (i * 3) + j);
                        String S_NAME = OBJECTS[zz].substring(0, OBJECTS[zz].indexOf(','));
                        tft.fillRect(((j * 75) + 10), ((i * 40) + 90), 71, 35, BTN_BLK_TEXT);
                        tft.drawRect(((j * 75) + 10), ((i * 40) + 90), 71, 35, L_TEXT);
                        if (S_NAME.length() > 10)
                        {
                            String S_NAME_1 = S_NAME.substring(0, 9);
                            String S_NAME_2 = S_NAME.substring(9, S_NAME.length());
                            int l1 = (S_NAME_1.length() / 2) * 6;
                            int l2 = (S_NAME_2.length() / 2) * 6;
                            tft.setCursor(((j * 75) + (45 - l1)), ((i * 40) + 99));
                            tft.print(S_NAME_1);
                            tft.setCursor(((j * 75) + (45 - l2)), ((i * 40) + 109));
                            tft.print(S_NAME_2);
                        }
                        else
                        {
                            int l = (S_NAME.length() / 2) * 6;
                            tft.setCursor(((j * 75) + (45 - l)), ((i * 40) + 104));
                            tft.print(S_NAME);
                        }
                        if (OBJECTS[zz] != "")
                        {
                            uint8_t n = ALIGN_STEP - 1;
                            processAlignmentStar(zz, CURRENT_ALIGN_STAR);
                            // If catalogue star is higher than 10 degrees above the horizon.
                            if (CURRENT_ALIGN_STAR.hrzPos.alt > 10)
                            {
                                // Copy catalogue star to alignment star.
                                ALIGNMENT_STARS[n] = CURRENT_ALIGN_STAR;
#ifdef SERIAL_DEBUG
                                Serial.print("Star number ");
                                Serial.print(n);
                                Serial.println("");
                                Serial.print(ALIGNMENT_STARS[n].name);
                                Serial.print(" - RA: ");
                                Serial.print(deg2hms(ALIGNMENT_STARS[n].equPos.ra, true, true));
                                Serial.print(" DEC: ");
                                Serial.print(deg2dms(ALIGNMENT_STARS[n].equPos.dec, true, false));
                                Serial.print(" ALT: ");
                                Serial.print(deg2dms(ALIGNMENT_STARS[n].hrzPos.alt, true, false));
                                Serial.print(" AZ: ");
                                Serial.print(deg2dms(ALIGNMENT_STARS[n].hrzPos.az, true, true));
                                Serial.println("");
#endif
                                delay(150);
                                drawAlignScreen();
                            }
                            else
                            {
                                OnScreenMsg(1);
                                delay(2000);
                                drawStarSelectScreen();
                            }
                        }
                    }
                }
            }
        }
        else if (CURRENT_SCREEN == 7)
        { // captures touches on drawAlignScreen()
            if (lx > 10 && lx < 110 && ly > 270 && ly < 310)
            {
                // BTN "<Repeat" or "<EXIT" pressed
                if (ALIGN_TYPE == 1)
                {
                    drawButton(10, 270, 100, 40, "< CANCEL", 0, BTN_L_BORDER, L_TEXT, 2);
                    delay(150);
                    drawSelectAlignment();
                }
                else
                {
                    drawButton(10, 270, 100, 40, "< BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                    delay(150);
                    drawStarSelectScreen();
                }
            }
            if (lx > 130 && lx < 230 && ly > 270 && ly < 310)
            {
                // BTN "ALIGN!" pressed
                drawButton(130, 270, 100, 40, "ALIGN", 0, BTN_L_BORDER, L_TEXT, 2);

                uint8_t starNum = ALIGN_STEP - 1;
                CURRENT_ALIGN_STAR = ALIGNMENT_STARS[starNum];

                currentElevAngle();
                CURRENT_ALIGN_STAR.hrzPos.alt = SCOPE.hrzPos.alt;
                CURRENT_ALIGN_STAR.hrzPos.az = SCOPE.hrzPos.az;

#ifdef PERFECT_ALIGN
                // Debug with proper trig values
                CURRENT_ALIGN_STAR.hrzPos.alt = ALIGNMENT_STARS[starNum].hrzPos.alt;
                CURRENT_ALIGN_STAR.hrzPos.az = ALIGNMENT_STARS[starNum].hrzPos.az;
#ifdef ALIGN_OFFSET
                // Add a 90 degree mis-alignment of azimuth in the mount to compensate for
                CURRENT_ALIGN_STAR.hrzPos.az += 90.0;
#endif
#endif
                // Correct for atmospheric refraction before doing anything else
                if (CORRECT_REFRACTION)
                {
                    double deltaAlt = ln_get_refraction_adj(CURRENT_ALIGN_STAR.hrzPos.alt, ATMO_PRESS, ATMO_TEMP);
                    CURRENT_ALIGN_STAR.hrzPos.alt -= deltaAlt;

#ifdef SERIAL_DEBUG
#ifdef DEBUG_REFRACTION
                    Serial.print("calREFRACTIONFromApparent - ");
                    Serial.print(" Refract Amount(ARCMIN): ");
                    Serial.print(deltaAlt);
                    Serial.println("");
#endif
#endif
                }

                ALIGNMENT_STARS[starNum] = CURRENT_ALIGN_STAR;
#ifdef SERIAL_DEBUG
                if (ALIGN_TYPE == 1)
                    Serial.print("Auto Align on ");
                else
                    Serial.print("Manual Align on ");
                Serial.print(ALIGNMENT_STARS[starNum].name);
                Serial.println("");
                Serial.print("AZ ");
                Serial.print(deg2dms(ALIGNMENT_STARS[starNum].hrzPos.az, true, true));
                Serial.print(" Alt ");
                Serial.print(deg2dms(ALIGNMENT_STARS[starNum].hrzPos.alt, true, false));
                Serial.println("");
#endif
                delay(150);
                if (ALIGN_STEP == 1)
                {
                    ALIGN_STEP++;
                    initAlign();
                    addStar(starNum + 1, NUM_ALIGNMENT_STARS, SCOPE.JD, &SCOPE.lnLatPos, &ALIGNMENT_STARS[starNum]);

                    if (ALIGN_TYPE == 1)
                        drawAlignScreen();
                    else
                        drawStarSelectScreen();
                }
                else if (ALIGN_STEP == 2)
                {
                    ALIGN_STEP++;
                    addStar(starNum + 1, NUM_ALIGNMENT_STARS, SCOPE.JD, &SCOPE.lnLatPos, &ALIGNMENT_STARS[starNum]);

                    if (NUM_ALIGNMENT_STARS == 2)
                    {
                        generateThirdStar();
                        transformMatrix();
                        drawMainScreen();
                    }
                    else if (ALIGN_TYPE == 1)
                        drawAlignScreen();
                    else
                        drawStarSelectScreen();
                }
                else if (ALIGN_STEP == 3)
                {
                    ALIGN_STEP = 0;
                    addStar(starNum + 1, NUM_ALIGNMENT_STARS, SCOPE.JD, &SCOPE.lnLatPos, &ALIGNMENT_STARS[starNum]);
                    transformMatrix();
                    if (CORRECT_MOUNT_ERRS)
                        bestZ123();
                    drawMainScreen();
                }
            }
        }
        else if (CURRENT_SCREEN == 8)
        {
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                IS_BT_MODE_ON = false;
                IS_WIFI_MODE_ON = false;
                if (IS_BT_MODE_ON)
                {
                    SerialBT.end();
                }

                delay(150);
                drawOptionsScreen();
            }
        }
        else if (CURRENT_SCREEN == 9)
        {
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 110 && ly > 50 && ly < 90)
            {
                // BTN MERCURY pressed
                drawButton(10, 50, 100, 40, "MERCURY", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(1, CURRENT_OBJECT);
                objectAltAz();
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 50 && ly < 90)
            {
                // BTN VENUS pressed
                drawButton(130, 50, 100, 40, "VENUS", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(2, CURRENT_OBJECT);
                objectAltAz();
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 110 && ly > 110 && ly < 150)
            {
                // BTN MARS pressed
                drawButton(10, 110, 100, 40, "MARS", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(4, CURRENT_OBJECT);
                objectAltAz();
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 110 && ly < 150)
            {
                // BTN JUPITER pressed
                drawButton(130, 110, 100, 40, "JUPITER", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(5, CURRENT_OBJECT);
                objectAltAz();
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 110 && ly > 170 && ly < 210)
            {
                // BTN SATURN pressed
                drawButton(10, 170, 100, 40, "SATURN", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(6, CURRENT_OBJECT);
                objectAltAz();
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 170 && ly < 210)
            {
                // BTN URANUS pressed
                drawButton(130, 170, 100, 40, "URANUS", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(7, CURRENT_OBJECT);
                objectAltAz();
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 110 && ly > 230 && ly < 270)
            {
                // BTN NEPTUNE pressed
                drawButton(10, 230, 100, 40, "NEPTUNE", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(8, CURRENT_OBJECT);
                objectAltAz();
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 230 && ly < 270)
            {
                // BTN PLUTO pressed
                drawButton(130, 230, 100, 40, "PLUTO", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(9, CURRENT_OBJECT);
                objectAltAz();
                delay(150);
                drawMainScreen();
            }
        }
    }
    else
    {
        //**************************************************************
        //
        //      BUTTON UP Events start here
        //
        //      - only executed when the user touches the screen - RELEASE
        //**************************************************************
        if (CURRENT_SCREEN == 0)
        {
            if (LAST_BUTTON == 1)
            {
                LAST_BUTTON = 0;
                drawButton(40, 270, 160, 40, "SKIP", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
#ifdef SERIAL_DEBUG
                Serial.print("OBSERVATION LATTITUDE: ");
                Serial.print(OBSERVATION_LATTITUDE);
                Serial.println("");
#endif
                drawClockScreen();
            }
        }
        else if (CURRENT_SCREEN == 1)
        {
            if (LAST_BUTTON == 1)
            {
                LAST_BUTTON = 0;
                tft.drawRect(10, 120, 65, 40, BLACK);
                TimerUpdateDraw(1);
            }
            if (LAST_BUTTON == 2)
            {
                LAST_BUTTON = 0;
                tft.drawRect(85, 120, 65, 40, BLACK);
                TimerUpdateDraw(2);
            }
            if (LAST_BUTTON == 3)
            {
                LAST_BUTTON = 0;
                tft.drawRect(160, 120, 65, 40, BLACK);
                TimerUpdateDraw(3);
            }
            if (LAST_BUTTON == 4)
            {
                LAST_BUTTON = 0;
                tft.drawRect(10, 170, 65, 40, BLACK);
                TimerUpdateDraw(4);
            }
            if (LAST_BUTTON == 5)
            {
                LAST_BUTTON = 0;
                tft.drawRect(85, 170, 65, 40, BLACK);
                TimerUpdateDraw(5);
            }
            if (LAST_BUTTON == 6)
            {
                LAST_BUTTON = 0;
                tft.drawRect(160, 170, 65, 40, BLACK);
                TimerUpdateDraw(6);
            }
            if (LAST_BUTTON == 7)
            {
                LAST_BUTTON = 0;
                tft.drawRect(10, 220, 65, 40, BLACK);
                TimerUpdateDraw(7);
            }
            if (LAST_BUTTON == 8)
            {
                LAST_BUTTON = 0;
                tft.drawRect(85, 220, 65, 40, BLACK);
                TimerUpdateDraw(8);
            }
            if (LAST_BUTTON == 9)
            {
                LAST_BUTTON = 0;
                tft.drawRect(160, 220, 65, 40, BLACK);
                TimerUpdateDraw(9);
            }
            if (LAST_BUTTON == 10)
            {
                LAST_BUTTON = 0;
                tft.drawRect(85, 270, 65, 40, BLACK);
                TimerUpdateDraw(0);
            }
            if (LAST_BUTTON == 22)
            {
                if (SUMMER_TIME == 1)
                {
                    SUMMER_TIME = 0;
                    tft.fillRect(10, 270, 65, 40, BLACK);
                    drawButton(10, 270, 65, 40, "DST", 0, BTN_L_BORDER, L_TEXT, 2);
                }
                else
                {
                    SUMMER_TIME = 1;
                    tft.fillRect(10, 270, 65, 40, BLACK);
                    drawButton(10, 270, 65, 40, "DST", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                }
                tft.setTextColor(L_TEXT);
                tft.setTextSize(3);
                LAST_BUTTON = 0;
            }
        }
        else if (CURRENT_SCREEN == 4 || CURRENT_SCREEN == 6)
        {
            if (LAST_BUTTON == 1)
            {
                LAST_BUTTON = 0;
                tft.fillRect(130, 270, 100, 40, BLACK);
                drawButton(130, 270, 100, 40, "NEXT >", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
            }
            if (LAST_BUTTON == 2)
            {
                LAST_BUTTON = 0;
                tft.fillRect(10, 270, 100, 40, BLACK);
                drawButton(10, 270, 100, 40, "< PREV", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// BLUETOOTH AND WIFI
// ......................................................................
//  This part of the code take care of:
//  - Bluetooth communication (both ways);
//
void attendTcpRequests()
{
    // check for new or lost connections:
    if (server.hasClient())
    {
#ifdef SERIAL_DEBUG
        Serial.println("hasClient!");
#endif
        if (!remoteClient || !remoteClient.connected())
        {
            if (remoteClient)
            {
#ifdef SERIAL_DEBUG
                Serial.println("Client Disconnected");
#endif
                remoteClient.stop();
            }
            remoteClient = server.available();
#ifdef SERIAL_DEBUG
            Serial.print("Inbound connection from: ");
            Serial.println(remoteClient.remoteIP());
#endif
            //  remoteClient.flush();
            remoteClient.setNoDelay(true);
        }
    }

    // when we have a new incoming connection from Skysafari:
    while (remoteClient.available())
    {
#ifdef SERIAL_DEBUG
        Serial.println("Has Client");
#endif
        byte skySafariCommand = remoteClient.read();

        if (skySafariCommand == 81) // 81 is ascii for Q, which is the only command skysafari sends to "basic encoders"
        {
            char encoderResponse[20];
            int iAzimuthReading = azEnc.read();
            int iAltitudeReading = imu.smoothAltitudeReading;
            sprintf(encoderResponse, "%i\t%i\t\n", iAzimuthReading, iAltitudeReading);
#ifdef SERIAL_DEBUG
            Serial.println(encoderResponse);
#endif
            remoteClient.println(encoderResponse);
        }
        else if (skySafariCommand == 72) // 'H' - request for encoder resolution, e.g. 10000-10000\n
        {
            char response[20];
            // Resolution on both axis is equal
            snprintf(response, 20, "%u-%u", STEPS_IN_FULL_CIRCLE, STEPS_IN_FULL_CIRCLE);
#ifdef SERIAL_DEBUG
            Serial.println(response);
#endif
            remoteClient.println(response);
        }
        else
        {
#ifdef SERIAL_DEBUG
            Serial.println("*****");
            Serial.println(skySafariCommand);
#endif
        }
    }
}

void attendBTRequests()
{

    byte skySafariCommand = SerialBT.read();

    if (skySafariCommand == 81) // 81 is ascii for Q, which is the only command skysafari sends to "basic encoders"
    {
        char encoderResponse[20];
        int iAzimuthReading = azEnc.read();
        int iAltitudeReading = imu.smoothAltitudeReading;
        sprintf(encoderResponse, "%i\t%i\t\n", iAzimuthReading, iAltitudeReading);
#ifdef SERIAL_DEBUG
        Serial.println(encoderResponse);
#endif
        SerialBT.print(encoderResponse);
    }
    else if (skySafariCommand == 72) // 'H' - request for encoder resolution, e.g. 10000-10000\n
    {
        char response[20];
        // Resolution on both axis is equal
        snprintf(response, 20, "%u-%u", STEPS_IN_FULL_CIRCLE, STEPS_IN_FULL_CIRCLE);
#ifdef SERIAL_DEBUG
        Serial.println(response);
#endif
        SerialBT.print(response);
    }
    else
    {
#ifdef SERIAL_DEBUG
        Serial.println("*****");
        Serial.println(skySafariCommand);
#endif
    }
}

void considerBTCommands()
{
#ifdef SERIAL_DEBUG
    Serial.print("Called considerBTCommands with: ");
    Serial.print(BT_COMMAND_STR);
    Serial.println("");
#endif
    String bt_reply = "";
    // :GR#  - Request RA coordinate
    if (BT_COMMAND_STR == 1)
    {
        telescopeRaDec();
        bt_reply = deg2hms(SCOPE.equPos.ra, true, false);
        // bt_reply = rad2hms(CURR_RA_RADS, true, false);
        bt_reply += "#";
        //bt_reply = rad2hms(this_ra, true, false);
#ifdef SERIAL_DEBUG
        Serial.print(":GR Called - Reply: ");
        Serial.print(bt_reply);
        Serial.println("");
#endif
        SerialBT.print(bt_reply);
    }
    // :GD#  - Request DEC coordinate
    else if (BT_COMMAND_STR == 2)
    {
        telescopeRaDec();
        bt_reply = deg2dms(SCOPE.equPos.dec, true, false);
        // bt_reply = rad2dms(CURR_DEC_RADS, true, false);
        bt_reply += "#";
//bt_reply = rad2dms(this_dec, true, false);
#ifdef SERIAL_DEBUG
        Serial.print(":GD Called - Reply: ");
        Serial.print(bt_reply);
        Serial.println("");
#endif
        SerialBT.print(bt_reply);
    }
    // :GA# - Request ALT
    else if (BT_COMMAND_STR == 3)
    {
        bt_reply = deg2dms(SCOPE.hrzPos.alt, true, false);
        // bt_reply = rad2dms(CURR_ALT_RADS, true, false);
        bt_reply += "#";
#ifdef SERIAL_DEBUG
        Serial.print(":GA Called - Reply: ");
        Serial.print(bt_reply);
        Serial.println("");
#endif
        SerialBT.print(bt_reply);
    }
    // :GZ# - Request AZ
    else if (BT_COMMAND_STR == 4)
    {
        bt_reply = deg2dms(SCOPE.hrzPos.az, true, true);
        // bt_reply = rad2dms(CURR_AZ_RADS, true, true);
        bt_reply += "#";
#ifdef SERIAL_DEBUG
        Serial.print(":GZ Called - Reply: ");
        Serial.print(bt_reply);
        Serial.println("");
#endif
        SerialBT.print(bt_reply);
    }
    SerialBT.flush();
    BT_COMMAND_STR = 0;
    bt_reply = "";
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP AND LOOP

void setup(void)
{
#ifdef SERIAL_DEBUG
    Serial.begin(9600);

    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS initialisation failed!");
        while (1)
            yield(); // Stay here twiddling thumbs waiting
    }
    Serial.println("SPIFFS initialised");

    Serial.printf("Flash Chip Size = %d byte\r\n", ESP.getFlashChipSize());
    delay(100);
    listDir(SPIFFS, "/", 0);
    SPIFFS.end();
#endif

    // Init GPS on Serial 2 and Pins RX2 (16) and t_x2 (17)
    Serial2.begin(9600);

    // Init I2C
    Wire.begin(21, 22);

    // Enable WiFi right away and setup access point
    //WiFi.mode(WIFI_AP);
    IPAddress ip(1, 2, 3, 4); // The "telescope IP address" that Skysafari should connect to is 1.2.3.4 which is easy to remember.
    IPAddress gateway(1, 2, 3, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(ip, gateway, subnet);
    WiFi.softAP(WIFI_AP_NAME, WIFI_PASS);

    IPAddress myIP = WiFi.softAPIP();

    // Init accelerometer
    // Apply compass calibration values:
    // Magnetometer Calibration
    // min: {  -606,   -596,   -319}    max: {  +739,   +652,  +1281}
    //Acceleromter Calibration
    // min: {-32448, -20640, -23056}    max: {+18912, +22496, +26560}
    imu.m_min = (LSM303::vector<int16_t>){-1589, -1730, -621};
    imu.m_max = (LSM303::vector<int16_t>){+820, +530, +1101};
    imu.init();
    imu.enableDefault();

    // Clock
    rtc.Begin();

    // Touchscreen
    tft.begin();
    tft.setRotation(0);
    touchCalibrate();

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);

    ALT_MULTIPLIER = ONE_REV / (float)STEPS_IN_FULL_CIRCLE;
    AZ_MULTIPLIER = ONE_REV / (float)STEPS_IN_FULL_CIRCLE;

    LOAD_SELECTOR = 1; // Load Alignment Stars by default

    L_TEXT = RED;
    D_TEXT = MAROON;
    TITLE_TEXT_BG = RED;
    TITLE_TEXT = BLACK;
    BTN_BLK_TEXT = BLACK;
    BTN_L_BORDER = RED;
    MSG_BOX_BG = RED;
    MSG_BOX_TEXT = BLACK;

    UPDATE_LAST = millis();
    CURRENT_SCREEN = -1;
    drawInitScreen();

    server.begin();
    server.setNoDelay(true);
}

void loop(void)
{
    if (IS_WIFI_MODE_ON)
    {
        attendTcpRequests(); // gets priority to prevent timeouts on Skysafari. Measured AVG execution time = 18ms
        yield();

        // Take new Alt/Az measurements from sensor/encoder
        if ((millis() - LAST_MEASUREMENT) > MEASUREMENT_PERIOD) // only take new measurements if enough time has elapsed.
        {
            imu.read();
            imu.calculatePosition();
            LAST_MEASUREMENT = millis();
        }
    }
    else if (IS_BT_MODE_ON)
    {
        attendBTRequests();
        yield();
        // Take new Alt/Az measurements from sensor/encoder
        if ((millis() - LAST_MEASUREMENT) > MEASUREMENT_PERIOD) // only take new measurements if enough time has elapsed.
        {
            imu.read();
            imu.calculatePosition();
            LAST_MEASUREMENT = millis();
        }
    }
    else
    {
        // Take new Alt/Az measurements from sensor/encoder
        if ((millis() - LAST_MEASUREMENT) > MEASUREMENT_PERIOD) // only take new measurements if enough time has elapsed.
        {
            imu.read();
            imu.calculatePosition();
            considerTelescopeMove();
            LAST_MEASUREMENT = millis();
        }
        // Update current telescope position on main screen every 500ms
        if (CURRENT_SCREEN == 3 && (millis() - LAST_POS_UPDATE) > SCOPE_POS_UPDATE)
        {
            tft.fillRect(70, 70, 170, 60, BLACK);
            tft.setTextColor(L_TEXT);

            tft.setTextSize(2);
            tft.setTextColor(L_TEXT);
            tft.setCursor(70, 70);
            tft.print(deg2dms(SCOPE.hrzPos.az, true, true));
            tft.setCursor(70, 90);
            tft.print(deg2dms(SCOPE.hrzPos.alt, true, false));

            tft.setTextSize(1);
            tft.setCursor(70, 110);
            tft.print(deg2hms(SCOPE.equPos.ra, true, true));
            tft.setCursor(70, 120);
            tft.print(deg2dms(SCOPE.equPos.dec, true, false));
            LAST_POS_UPDATE = millis();
        }
        // Do regular time updates every 5000ms
        if ((millis() - UPDATE_LAST) > 5000)
        {
            calculateLST();
            considerTimeUpdates();
            UPDATE_LAST = millis();
        }
    }
    uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
    tft.getTouch(&t_x, &t_y);
    considerTouchInput(t_x, t_y);
    yield();
}
