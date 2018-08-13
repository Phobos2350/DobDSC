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

// #define SERIAL_DEBUG // comment out to deactivate the serial debug mode
// #define PERFECT_ALIGN

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

// Star Structs
struct Object
{
    String name;
    String type;
    String constellation;
    String description;
    String size;
    double ra;
    double dec;
    double alt;
    double az;
    float mag;
};

struct AlignmentStar
{
    String name;
    String constellation;
    double ra;
    double dec;
    double alt;
    double az;
    double time;
    double lat;
    float mag;
};
//

// Transform Objects
class Position
{
  public:
    double ra, dec, az, alt, sidT, ha;
    //Position();
    //Position(double ra, double dec, double az, double alt, double sidT, double ha);

    Position()
    {
        ra = 0;
        dec = 0;
        az = 0;
        alt = 0;
        sidT = 0;
        ha = 0;
    }

    Position(double ra, double dec, double az, double alt, double sidT, double ha)
    {
        ra = ra;
        dec = dec;
        az = az;
        alt = alt;
        sidT = sidT;
        ha = ha;
    }

    //~Position();
};

class ConvertMatrixWorkingStorage
{
  public:
    int dimen, subroutineCountTaki, subroutineCountBell, alignPositions, convertSubroutineType, initType;
    double pri, sec, determinate, z1Error, z2Error, z3Error;
    double qq[3][3];
    double vv[3][3];
    double rr[3][3];
    double xx[3][3];
    double yy[3][3];

    Position current, one, two, three;
    Position alignMatrixPosition, recoverAlignPosition;
    Position initAltazResultsPosition, presetPosition;
    Position calcPointErrorsPosition, bestZ12Position;

    ConvertMatrixWorkingStorage()
    {
        dimen = 3;
        subroutineCountTaki = 0;
        subroutineCountBell = 0;
        alignPositions = 0;
        pri = 0;
        sec = 0;
        determinate = 0;
        z1Error = 0;
        z2Error = 0;
        z3Error = 0;
        convertSubroutineType = 4;
        initType = 1;
    }

    //~ConvertMatrixWorkingStorage();
};

// All Alignment-Specific settings
enum Alignment_Enum
{
    TAKI_BASIC,
    TAKI_ADVANCED
};

Alignment_Enum ALIGNMENT_METHOD = TAKI_BASIC;
boolean CORRECT_REFRACTION = false;
boolean CORRECT_PRECESSION_ETC = false;
boolean CORRECT_MOUNT_ERRS = false;
boolean BASIC_ALIGN_READY = false;

const char *WIFI_AP_NAME = "Skywatcher-WiFi"; // Name of the WiFi access point this device will create for your tablet/phone to connect to.
const char *WIFI_PASS = "3mzbZSMNNx9d";

//
const String FirmwareDate = "16 07 18";
const String FirmwareNumber = "v0.1.1 DobDSC";
const String FirmwareName = "DobDSC";
const String FirmwareTime = "21:00:00";
//
const String object_name[9] = {"Mercury", "Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"};
const String star_name[1] = {"Sun"};

const double ONE_REV = PI * 2.0;
const double THREE_QRT_REV = PI * 3.0 / 2.0;
const double HALF_REV = PI;
const double QRT_REV = PI / 2.0;
const double HOUR_TO_REV = 1.0 / 24;
const double HOUR_TO_RAD = PI / 12;
const double MINUTE_TO_REV = 1.0 / 1440;
const double MINUTE_TO_RAD = PI / 720;
const double SEC_TO_REV = 1.0 / 86400;
const double SEC_TO_RAD = PI / 43200;
const double DEG_TO_REV = 1.0 / 360;
const double ARCMIN_TO_REV = 1.0 / 21600;
const double ARCMIN_TO_RAD = PI / 10800;
const double ARCSEC_TO_REV = 1.0 / 1296000;
const double ARCSEC_TO_RAD = PI / 648000;
const double TENTH_ARCSEC_TO_RAD = ARCSEC_TO_RAD / 10.0;

const float JD2000 = 2451545.0;
const float JDMOD = 2400000.5;
const float JDYEAR = 365.25;
const float JDCENTURY = 36525.0;
const float SIDEREAL_FRACTION = 1.0027379093;

const int NUM_ALIGN_STARS = 100;

const float GEAR_RATIO = GEAR_LARGE / GEAR_SMALL;
const int STEPS_IN_FULL_CIRCLE = ENCODER_RES * GEAR_RATIO;
const double STEPS_TO_RAD = STEPS_IN_FULL_CIRCLE / ONE_REV;



const float OBJECT_DATA[9][14] = {
    // a, aΔ, e, eΔ, i, iΔ,  L, LΔ, ω, ωΔ, N, NΔ  >>> L2000 , diameter
    {0.38709927, 0.00000037, 0.20563593, 0.00001906, 7.00497902, -0.00594749, 252.25032350, 149472.67411175, 77.45779628, 0.16047689, 48.33076593, -0.12534081, 6.74, 0.2},   // Mercury
    {0.72333566, 0.00000390, 0.00677672, -0.00004107, 3.39467605, -0.00078890, 181.97909950, 58517.81538729, 131.60246718, 0.00268329, 76.67984255, -0.27769418, 16.92, 0.6}, // Venus
    {1.00000261, 0.00000562, 0.01671123, -0.00004392, -0.00001531, -0.01294668, 100.46457166, 35999.37244981, 102.93768193, 0.32327364, 0, 0, 0, 1},                          // Earth
    {1.52371034, 0.00001847, 0.09339410, 0.00007882, 1.84969142, -0.00813131, -4.55343205, 19140.30268499, -23.94362959, 0.44441088, 49.55953891, -0.29257343, 9.31, 1.8},    // Mars
    {5.20288700, -0.00011607, 0.04838624, -0.00013253, 1.30439695, -0.00183714, 34.39644051, 3034.74612775, 14.72847983, 0.21252668, 100.47390909, 0.20469106, 191, 11.8},    // Jupiter
    {9.53667594, -0.00125060, 0.05386179, -0.00050991, 2.48599187, 0.00193609, 49.95424423, 1222.49362201, 92.59887831, -0.41897216, 113.66242448, -0.28867794, 157, 29.4},   // Saturn
    {19.1891646, -0.00196176, 0.04725744, -0.00004397, 0.77263783, -0.00242939, 313.23810451, 428.48202785, 170.95427630, 0.40805281, 074.01692503, 0.04240589, 64, 84},      // Uranus
    {30.06992276, 0.00026291, 0.00859048, 0.00005105, 1.77004347, 0.00035372, -55.12002969, 218.45945325, 44.96476227, -0.32241464, 131.78422574, -0.00508664, 61.5, 164.8},  // Neptune
    {39.48211675, -0.00031596, 0.24882730, 0.00005170, 17.14001206, 0.00004818, 238.92903833, 145.20780515, 224.06891629, -0.04062942, 110.30393684, -0.01183482, 3.28, 248}  // Pluto
};

const float REFRACTION_TABLE[12][2] = {
    {90, 0},
    {60, 0.55},
    {30, 1.7},
    {20, 2.6},
    {15, 3.5},
    {10, 5.2},
    {8, 6.4},
    {6, 8.3},
    {4, 11.5},
    {2, 18},
    {0, 34.5},
    {-1, 42.75}};

uint8_t ALIGN_STEP = 1;   // Using this variable to count the allignment steps - 1: Synchronize, 2: Allign and centre, 3:....
uint8_t ALIGN_TYPE = 0;   // Variable to store the alignment type (0-Skip Alignment, 1-1 Star alignment, 2-2 Star alignment
uint8_t LOADED_STARS = 0; // What Objects are loaded from SPIFFS 1 == alignment, 2 == messier, 3 == treasure
uint8_t TIME_ZONE = 0;
uint8_t GPS_ITERATIONS = 0;
uint8_t CURRENT_SCREEN = -1;
uint8_t OLD_MIN, OLD_DAY;
uint8_t NUM_ALIGNMENT_STARS = 2;

boolean IS_BT_MODE_ON = false;
boolean IS_WIFI_MODE_ON = false;
boolean IS_IN_OPERATION = false; // This variable becomes True when Main screen appears
boolean ALIGNMENT_READY = false;

// Default values to load when CANCEL button is hit on the GPS screen
float OBSERVATION_LONGITUDE = -0.01365997; // (-0.01365997* - Home)
float OBSERVATION_LATTITUDE = 51.18078754; // (51.18078754* - Home)
float OBSERVATION_ALTITUDE = 52.00;        // Lingfield, UK
double OBSERVATION_LONGITUDE_RADS = -0.000238237442896635;
double OBSERVATION_LATTITUDE_RADS = 0.89327172847324593;

unsigned int AZ_STEPS, ALT_STEPS; // Current position of the decoders in Steps! - when movement occures, values are changed accordingly

unsigned long UPDATE_LAST, UPDATE_TIME;
unsigned long LAST_MEASUREMENT = 0;  // millisecond timestamp of last measurement, to measure only every XXX milliseconds
unsigned long SCOPE_POS_UPDATE = 30; // 30ms updated = 30FPS
unsigned long LAST_POS_UPDATE = 0;

double LST, LST_RADS, JDN;
double CURR_ALT_RADS, CURR_AZ_RADS;
double CURR_RA_RADS, CURR_DEC_RADS;
double Z1_ERR, Z2_ERR, Z3_ERR;

double INIT_LAT, AZ_OFFSET, HA_OFFSET;

double COUNTS_PER_REV, COUNTS_PER_DEG, COUNTS_PER_ARCSEC;
double ALT_MULTIPLIER, AZ_MULTIPLIER;

double ECCLIPTIC_EARTH_ORBIT, EARTH_PERIHELION_LON, SUN_TRUE_LON, NUTATION_LON, NUTATION_OBLIQ, ECLIPTIC_OBLIQ;
double PROPER_MOTION_RA = 0, PROPER_MOTION_DEC = 0;

float INITIAL_TIME;
float ECLIPTIC_ANGLE = 23.43928;

float x_coord;
float y_coord;
float z_coord;

float x_earth;
float y_earth;
float z_earth;

float dist_earth_to_object = 0;
float dist_earth_to_sun = 0;
float dist_object_to_sun = 0;

float PLANET_RA, PLANET_DECL, PLANET_AZ, PLANET_ALT, PLANET_LON, PLANET_LAT, PLANET_MAG, PLANET_SIZE;

String CURR_ALT, CURR_AZ, CURR_RA, CURR_DEC;
String OBJECTS[130];
int BT_COMMAND_STR;

int LAST_BUTTON, MESS_PAGER, TREAS_PAGER, STARS_PAGER, LOAD_SELECTOR; // selector to show which LOADING mechanism is used: 1 - Messier, 2 - File, 3 - NGCs

int W_DATE_TIME[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // array to store date - as updated from updater screen - Wishing_Date
int DATE_ENTRY_POS = 0;
int SUMMER_TIME = 0;
int16_t L_TEXT, D_TEXT, BTN_L_BORDER, BTN_BLK_TEXT, TITLE_TEXT_BG, TITLE_TEXT, MSG_BOX_BG, MSG_BOX_TEXT; // defines string constants for the clor - Depending on the DAY/NIGHT modes

// Toshimi Taki’s Alignment variables
bool IS_SET_R1, IS_SET_R2, IS_SET_R3;
// Calculation vectors.
float HVC1[3], EVC1[3], HVC2[3], EVC2[3], HVC3[3], EVC3[3];
// Matricies.
float TRANSFORM_MATRIX[3][3];
float INV_TRANSFORM_MATRIX[3][3];

double W;
double Q[4][4];
double V[4][4];
double R[4][4];
double X[4][4];
double Y[4][4];

uint8_t BT_STATE = BT_START;
char BT_CHARACTER;
String BT_COMMAND;
boolean BT_PRECISION = true;

ConvertMatrixWorkingStorage CMWS_ALIGN;

AlignmentStar CURRENT_ALIGN_STAR;
AlignmentStar ALIGNMENT_STARS[3];
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
double cbrt(double x)
{
    if (x > 0.0)
        return exp(log(x) / 3.0);
    else if (x < 0.0)
        return -cbrt(-x);
    else // x == 0.0
        return 0.0;
}

// Haversine function
double hav(double angle)
{
    return 0.5 * (1 - cos(angle));
}

boolean withinRange(double actual, double expected, double range)
{
    return actual <= expected + range && actual >= expected - range;
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
    if (validatedRad > QRT_REV)
        correctedRad = HALF_REV - validatedRad; // > 90 degrees: reverse scale
    else if (validatedRad >= -QRT_REV)
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
    return rad > QRT_REV || rad < -QRT_REV;
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

String rad2hms(float rad, boolean highPrecision, boolean withUnits)
{
    // if (rad < 0)
    //     rad = rad + 2.0 * PI;

    double validatedRad = validRev(rad);
    float hours = validatedRad * 12.0 / PI;
    float minutes = (hours - floor(hours)) * 60.0;
    float seconds = (minutes - floor(minutes)) * 60.0;

    if (highPrecision)
    {
        if (withUnits)
        {
            return padding((String) int(floor(hours)), (uint8_t)2) + "h" +
                   padding((String) int(floor(minutes)), (uint8_t)2) + "m" +
                   padding((String) int(floor(seconds)), (uint8_t)2) + "s";
        }
        else
        {
            return padding((String) int(floor(hours)), (uint8_t)2) + ":" +
                   padding((String) int(floor(minutes)), (uint8_t)2) + ":" +
                   padding((String) int(floor(seconds)), (uint8_t)2);
        }
    }
    else
    {
        if (withUnits)
        {
            return padding((String) int(floor(hours)), (uint8_t)2) + "h" +
                   padding((String) int(floor(minutes)), (uint8_t)2) + "." +
                   (String) int(floor((minutes - floor(minutes)) * 10.0)) + "m";
        }
        return padding((String) int(floor(hours)), (uint8_t)2) + ":" +
               padding((String) int(floor(minutes)), (uint8_t)2) + "." +
               (String) int(floor((minutes - floor(minutes)) * 10.0));
    }
}

String rad2dms(float rad, boolean highPrecision, boolean asAzimuth)
{
#ifdef SERIAL_DEBUG
    if (rad > 2.0 * PI || rad < -2.0 * PI || isnan(rad))
    {
        Serial.print("rad2dms() called on RAD= ");
        Serial.print(rad);
        Serial.println("");
    }
#endif
    float degs = fabs(rad) * RAD_TO_DEG;
    float minutes = (degs - floor(degs)) * 60.0;
    float seconds = (minutes - floor(minutes)) * 60.0;
    String sign = "";

    if (!asAzimuth)
    {
        sign = "+";
        if (rad < 0)
            sign = "-";
    }

    if (highPrecision)
    {
        return sign + padding((String) int(floor(fabs(degs))), (uint8_t)2) + (char)247 +
               padding((String) int(floor(minutes)), (uint8_t)2) + "'" +
               padding((String) int(floor(seconds)), (uint8_t)2);
    }
    else
    {
        return sign + padding((String) int(floor(fabs(degs))), (uint8_t)2) + "*" +
               padding((String) int(floor(minutes)), (uint8_t)2);
    }
}

String deg2hms(float deg, boolean highPrecision, boolean withUnits)
{
    float hours = deg / 15;
    float minutes = (hours - floor(hours)) * 60.0;
    float seconds = (minutes - floor(minutes)) * 60.0;

    if (highPrecision)
    {
        if (withUnits)
        {
            return padding((String) int(floor(hours)), (uint8_t)2) + "h" +
                   padding((String) int(floor(minutes)), (uint8_t)2) + "m" +
                   padding((String) int(floor(seconds)), (uint8_t)2) + "s";
        }
        else
        {
            return padding((String) int(floor(hours)), (uint8_t)2) + ":" +
                   padding((String) int(floor(minutes)), (uint8_t)2) + ":" +
                   padding((String) int(floor(seconds)), (uint8_t)2);
        }
    }
    else
    {
        if (withUnits)
        {
            return padding((String) int(floor(hours)), (uint8_t)2) + "h" +
                   padding((String) int(floor(minutes)), (uint8_t)2) + "." +
                   (String) int(floor((minutes - floor(minutes)) * 10.0)) + "m";
        }
        return padding((String) int(floor(hours)), (uint8_t)2) + ":" +
               padding((String) int(floor(minutes)), (uint8_t)2) + "." +
               (String) int(floor((minutes - floor(minutes)) * 10.0));
    }
}

String deg2dms(float deg, boolean highPrecision, boolean asAzimuth)
{
    float degs = fabs(deg);
    float minutes = (degs - floor(degs)) * 60.0;
    float seconds = (minutes - floor(minutes)) * 60.0;
    String sign = "";

    if (!asAzimuth)
    {
        sign = "+";
        if (deg < 0)
            sign = "-";
    }

    if (highPrecision)
    {
        return sign + padding((String) int(floor(fabs(deg))), (uint8_t)2) + "*" +
               padding((String) int(floor(minutes)), (uint8_t)2) + "'" +
               padding((String) int(floor(seconds)), (uint8_t)2);
    }
    else
    {
        return sign + padding((String) int(floor(fabs(deg))), (uint8_t)2) + "*" +
               padding((String) int(floor(minutes)), (uint8_t)2);
    }
}

double hms2rads(float h, float m, float s)
{
    return (h + (m / 60.0) + (s / 3600.0)) * HOUR_TO_RAD;
}

double dms2rads(float d, float m, float s)
{
    if (d < 0 || m < 0 || s < 0)
    {
        d = -fabs(d);
        m = -fabs(m);
        s = -fabs(s);
    }
    else
    {
        d = fabs(d);
        m = fabs(m);
        s = fabs(s);
    }
    return (d + (m / 60.0) + (s / 3600.0)) * DEG_TO_RAD;
}

// Converts rectangular co-ords to spherical where:
//      returnArray[0] = Heliocentric Distance
//      returnArray[1] = RA / AZIMUTH / LONG
//      returnArray[2] = DECL / ALT / LAT
void toSpherical(double x, double y, double z, double *returnArray)
{
    returnArray[0] = sqrt(sq(x) + sq(y) + sq(z));
    returnArray[1] = atan2(y, x);
    returnArray[2] = atan2(z, sqrt(sq(x) + sq(y)));
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
    CURR_ALT_RADS = ALT_MULTIPLIER * ALT_STEPS;
    CURR_AZ_RADS = AZ_MULTIPLIER * AZ_STEPS;
    CURR_ALT = rad2dms(CURR_ALT_RADS, true, false);
    CURR_AZ = rad2dms(CURR_AZ_RADS, true, true);
}

void telescopeRaDec()
{
    //getECoords(CURR_AZ_RADS, CURR_ALT_RADS, LST_RADS, &CURR_RA_RADS, &CURR_DEC_RADS);
    //getEquatorialMatrix(CURR_ALT_RADS, CURR_AZ_RADS, LST_RADS, &CURR_RA_RADS, &CURR_DEC_RADS);

    if (ALIGNMENT_METHOD == TAKI_ADVANCED)
    {
        CMWS_ALIGN.current.alt = CURR_ALT_RADS;
        CMWS_ALIGN.current.az = CURR_AZ_RADS;
        CMWS_ALIGN.current.sidT = LST_RADS;
        getEquatorialMatrix(CMWS_ALIGN);
        CURR_RA_RADS = CMWS_ALIGN.current.ra;
        CURR_DEC_RADS = CMWS_ALIGN.current.dec;
    }
    else
    {
        basic_ScopeToEquatorial(CURR_AZ_RADS, CURR_ALT_RADS, LST_RADS, &CURR_RA_RADS, &CURR_DEC_RADS);
    }

    // Correct for astronomical movements and refraction if needed
    // double delta_ra, delta_dec;
    // if (CORRECT_REFRACTION)
    // {
    //     calcRefractionFromApparentEquatorialCorrection(CURR_RA_RADS, CURR_DEC_RADS, LST_RADS, OBSERVATION_LATTITUDE_RADS, &delta_ra, &delta_dec);
    //     CURR_RA_RADS -= delta_ra;
    //     CURR_DEC_RADS -= delta_dec;
    // }
    // if (CORRECT_PRECESSION_ETC)
    // {
    //     calcProperMotionPrecessionNutationAberration(CURR_RA_RADS, CURR_DEC_RADS, PROPER_MOTION_RA, PROPER_MOTION_DEC, &delta_ra, &delta_dec);
    //     CURR_RA_RADS -= delta_ra;
    //     CURR_RA_RADS -= delta_dec;
    // }

    CURR_RA = rad2hms(CURR_RA_RADS, true, true);
    CURR_DEC = rad2dms(CURR_DEC_RADS, true, false);
}

void objectAltAz()
{
    //double this_alt, this_az;
    //calculateLST();
    //getHCoords(CURRENT_OBJECT.ra, CURRENT_OBJECT.dec, LST_RADS, &this_az, &this_alt);
    //getAltAzMatrix(CURRENT_OBJECT.ra, CURRENT_OBJECT.dec, LST_RADS, &this_alt, &this_az, 4);
    // CURRENT_OBJECT.alt = this_alt;
    // CURRENT_OBJECT.az = this_az;

    if (ALIGNMENT_METHOD == TAKI_ADVANCED)
    {
        CMWS_ALIGN.current.ra = CURRENT_OBJECT.ra;
        CMWS_ALIGN.current.dec = CURRENT_OBJECT.dec;
        CMWS_ALIGN.current.sidT = LST_RADS;
        getAltAzMatrix(CMWS_ALIGN);
        CURRENT_OBJECT.alt = CMWS_ALIGN.current.alt;
        CURRENT_OBJECT.az = CMWS_ALIGN.current.az;
    }
    else
    {
        basic_EquatorialToScope(CURRENT_OBJECT.ra, CURRENT_OBJECT.dec, LST_RADS, &CURRENT_OBJECT.az, &CURRENT_OBJECT.alt);
    }
}

void celestialToEquatorial(float ra, float dec, float lat,
                           float lst, double *alt, double *az)
{
    // float ha = lst - ra;
    // if (ha < 0)
    // {
    //     ha += 2.0 * PI;
    // }
    // (*alt) = asin(sin(dec) * sin(lat) + cos(dec) * cos(lat) * cos(ha));
    // (*az) = acos((sin(dec) - sin(*alt) * sin(lat)) / (cos(*alt) * cos(lat)));
    double absLat = fabs(lat);
    if (lat < 0)
        dec = -dec;
    // Bring DEC to within -90 to +90
    if (dec > QRT_REV || dec < -QRT_REV)
    {
        dec = HALF_REV - dec;
        ra = validRev(ra + HALF_REV);
    }

    double ha = lst - ra;
    double alt_tmp = convertSecondaryAxis(ha, dec, absLat);
    double az_tmp = convertPrimaryAxis(ha, dec, alt_tmp, absLat);
    if (lat < 0)
        az_tmp = reverseRev(az_tmp);
    *alt = alt_tmp;
    *az = az_tmp;
}

void equatorialToCelestial(float alt, float az, float lat,
                           float lst, double *ra, double *dec)
{
    // float ha = atan(sin(az) / (cos(az) * sin(lat)) + (tan(alt) * cos(lat)));
    // (*dec) = asin((sin(lat) * sin(alt)) - (cos(lat) * cos(alt) * cos(az)));
    // (*ra) = lst - ha;

    // Bring ALT to within -90 to +90
    if (alt > QRT_REV || alt < -QRT_REV)
    {
        alt = HALF_REV - alt;
        az = validRev(az + HALF_REV);
    }
    double absLat = fabs(lat);
    if (lat < 0)
        az = reverseRev(az);
    double dec_tmp = convertSecondaryAxis(az, alt, absLat);
    double ha_tmp = convertPrimaryAxis(az, alt, dec_tmp, absLat);
    *ra = validRev(lst - ha_tmp);
    if (lat < 0)
        *dec = -dec_tmp;
    else
        *dec = dec_tmp;
}

int getJulianYear(int jd)
{
    return (jd - JD2000) / JDYEAR + 2000;
}

void calcProperMotion(double motionRa, double motionDec, double deltaJulianYear, double *deltaRa, double *deltaDec)
{
    (*deltaRa) = motionRa * deltaJulianYear;
    (*deltaDec) = motionDec * deltaJulianYear;
}

void calcPrecession(double ra, double dec, double deltaYear, double *deltaRa, double *deltaDec)
{
    double m = 46.124 + 0.00279 * deltaYear / 100;
    double n = 20.043 - 0.0085 * deltaYear / 100;
    (*deltaRa) = deltaYear * (m + n * sin(ra) * tan(dec)) * ARCSEC_TO_RAD;
    (*deltaDec) = deltaYear * n * cos(ra) * ARCSEC_TO_RAD;
}

void calcPrecessionRigorous(double ra, double dec, double startJulianYear, double deltaJulianYear, double *deltaRa, double *deltaDec)
{
    double eta, zeta, theta;
    // from Meeus: t1 and t2 are in Julian centuries where: t1 is difference between starting date and 2000, and t2 is the difference between ending and starting date
    double t1 = (startJulianYear - 2000) / 100;
    double t2 = deltaJulianYear / 100;
    double t1Sqr = sq(t1);
    double t2Sqr = sq(t2);
    double t2Cube = t2Sqr * t2;

    if (t1 == 0)
    {
        eta = (2306.2181 * t2 + 0.30188 * t2Sqr + 0.017998 * t2Cube) * ARCSEC_TO_RAD;
        zeta = (2306.2181 * t2 + 1.09468 * t2Sqr + 0.018203 * t2Cube) * ARCSEC_TO_RAD;
        theta = (2004.3109 * t2 - 0.42665 * t2Sqr + 0.041883 * t2Cube) * ARCSEC_TO_RAD;
    }
    else
    {
        eta = ((2306.2181 + 1.39656 * t1 - 0.000139 * t1Sqr) * t2 + (0.30188 - 0.000344 * t1) * t2Sqr + 0.017998 * t2Cube) * ARCSEC_TO_RAD;
        zeta = ((2306.2181 + 1.39656 * t1 - 0.000139 * t1Sqr) * t2 + (1.09468 + 0.000066 * t1) * t2Sqr + 0.018203 * t2Cube) * ARCSEC_TO_RAD;
        theta = ((2004.3109 - 0.8533 * t1 - 0.000217 * t1Sqr) * t2 - (0.42665 + 0.000217 * t1) * t2Sqr + 0.041883 * t2Cube) * ARCSEC_TO_RAD;
    }
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);

    // if necessary, bring Dec within range of +-90deg and rotate RA by 12 hrs
    double unflippedDec, unflippedRa;
    if (calcDecIsFlipped(dec))
    {
        unflippedDec = flipDEC(dec);
        unflippedRa = flipRA(ra);
    }
    else
    {
        unflippedDec = dec;
        unflippedRa = ra;
    }

    double sinDec = sin(unflippedDec);
    double cosDec = cos(unflippedDec);
    double raPlusEta = ra + eta;
    double cosRaPlusEta = cos(raPlusEta);
    double a = cosDec * sin(raPlusEta);
    double b = cosTheta * cosDec * cosRaPlusEta - sinTheta * sinDec;
    double c = sinTheta * cosDec * cosRaPlusEta + cosTheta * sinDec;

    // RA
    double precessedRa = atan2(a, b) + zeta;
    (*deltaRa) = validHalfRev(precessedRa - unflippedRa);

    // DEC
    // Use alternative formula if very close to Pole
    double precessedDec;
    if (withinRange(fabs(unflippedDec), QRT_REV, ARCMIN_TO_RAD))
    {
        double aCosParam = sqrt(sq(a) + sq(b));
        precessedDec = acos(aCosParam);
    }
    else
    {
        precessedDec = asin(c);
    }

    (*deltaDec) = validDec(precessedDec - unflippedDec);
}

void celestialParameters(int julianYearsSinceJD2000)
{
    double t = julianYearsSinceJD2000 / 100;
    // Longitudes
    double sunMeanLng = validRev((280.46646 + 36000.76983 * t) * DEG_TO_RAD);
    double moonMeanLng = validRev((218.3165 + 481267.8813 * t) * DEG_TO_RAD);
    // Eccentricity
    double eccEarthOrbit = 0.016708634 - 0.000042037 * t - 0.0000001267 * t * t;
    double eccEarthPerihelionLng = validRev((102.93735 + 1.71946 * t + 0.00046 * t * t) * DEG_TO_RAD);
    double sunMeanAnom = validRev((357.52772 + 35999.050340 * t - 0.0001603 * t * t - t * t * t / 300000) * DEG_TO_RAD);
    //double moonMeanAnom = validRev((134.96298 + 477198.867398 * t - 0.0086972 * t * t + t * t * t / 56250) * DEG_TO_RAD);

    double moonAscNodeLng = validRev((125.04452 - 1934.136261 * t + 0.0020708 * t * t + t * t * t / 450000) * DEG_TO_RAD);
    double meanOblqEcliptic = (23.43929 - 46.815 / 3600 * t - 0.00059 / 3600 * t * t + 0.001813 / 3600 * t * t * t) * DEG_TO_RAD;
    double sunEquationOfCenter = (1.914602 - 0.004817 * t - 0.000014 * t * t) * sin(sunMeanAnom) + (0.019993 - 0.000101 * t) * sin(2 * sunMeanAnom) + 0.000289 * sin(3 * sunMeanAnom);
    sunEquationOfCenter *= DEG_TO_RAD;
    double sunTrueLng = validRev(sunMeanLng + sunEquationOfCenter);

    double nutationLng = -17.2 * sin(moonAscNodeLng) - 1.32 * sin(2 * sunMeanLng) - 0.23 * sin(2 * moonMeanLng) + 0.21 * sin(2 * moonAscNodeLng);
    double nutationOb = 9.2 * cos(moonAscNodeLng) + 0.57 * cos(2 * sunMeanLng) + 0.1 * cos(2 * moonMeanLng) - 0.09 * cos(2 * moonAscNodeLng);

    double eclipticOb = meanOblqEcliptic + nutationOb * ARCSEC_TO_RAD;

    ECCLIPTIC_EARTH_ORBIT = eccEarthOrbit;
    EARTH_PERIHELION_LON = eccEarthPerihelionLng;
    SUN_TRUE_LON = sunTrueLng;
    NUTATION_LON = nutationLng;
    NUTATION_OBLIQ = nutationOb;
    ECLIPTIC_OBLIQ = eclipticOb;
}

void calcNutation(double ra, double dec, double *deltaRA, double *deltaDEC)
{
    double cosRa = cos(ra);
    double sinRa = sin(ra);
    double tanDec = tan(dec);
    double cosEclipObliq = cos(ECLIPTIC_OBLIQ);
    double sinEclipObliq = sin(ECLIPTIC_OBLIQ);
    double deltaRa = (cosEclipObliq + sinEclipObliq * sinRa * tanDec) * NUTATION_LON - (cosRa * tanDec) * NUTATION_OBLIQ;
    deltaRa *= ARCSEC_TO_RAD;

    double deltaDec = sinEclipObliq * cosRa * NUTATION_LON + sinRa * NUTATION_OBLIQ;
    deltaDec *= ARCSEC_TO_RAD;

    (*deltaRA) = deltaRa;
    (*deltaDEC) = deltaDec;
}

void calcAnnualAberration(double ra, double dec, double *deltaRA, double *deltaDEC)
{
    double k = 20.49552;
    double cosRa = cos(ra);
    double sinRa = sin(ra);
    double cosDec = cos(dec);
    double sinDec = sin(dec);
    double cosSunTrueLon = cos(SUN_TRUE_LON);
    double sinSunTrueLon = sin(SUN_TRUE_LON);
    double cosEclipObliq = cos(ECLIPTIC_OBLIQ);
    double tanEclipObliq = tan(ECLIPTIC_OBLIQ);
    double cosEarthPerihelionLon = cos(EARTH_PERIHELION_LON);
    double sinEarthPerihelionLon = sin(EARTH_PERIHELION_LON);

    double deltaRa = -k * ((cosRa * cosSunTrueLon * cosEclipObliq + sinRa * sinSunTrueLon) / cosDec) + ECCLIPTIC_EARTH_ORBIT * k * (cosRa * cosEarthPerihelionLon * cosEclipObliq + sinRa * sinEarthPerihelionLon / cosDec);
    deltaRa *= ARCSEC_TO_RAD;
    double deltaDec = -k * (cosSunTrueLon * cosEclipObliq * (tanEclipObliq * cosDec - sinRa * sinDec) + cosRa * sinDec * sinSunTrueLon) + ECCLIPTIC_EARTH_ORBIT * k * (cosEarthPerihelionLon * cosEclipObliq * (tanEclipObliq * cosDec - sinRa * sinDec) + cosRa * sinDec * sinEarthPerihelionLon);
    deltaDec *= ARCSEC_TO_RAD;

    (*deltaRA) = deltaRa;
    (*deltaDEC) = deltaDec;
}

void calcProperMotionPrecessionNutationAberration(double ra, double dec, double motionRa, double motionDec, double *deltaRa, double *deltaDec)
{
    double tmpRa, tmpDec, totalDeltaRa, totalDeltaDec, properMotionRa, properMotionDec;
    double precessedRa, precessedDec, nutationRa, nutationDec, aberrationRa, aberrationDec;
    double coordJDYear = getJulianYear(JD2000);
    double currJDYear = getJulianYear(JDN);
    double deltaJulianYear = currJDYear - coordJDYear;
    //double julianYearSinceJD2000 = currJDYear - 2000;

    totalDeltaDec = 0;
    totalDeltaRa = 0;
    calcProperMotion(motionRa, motionDec, deltaJulianYear, &properMotionRa, &properMotionDec);
    totalDeltaRa += properMotionRa;
    totalDeltaDec += properMotionDec;
    //calcPrecession(properMotionRa, properMotionDec, deltaJulianYear, &tmpRa, &tmpDec);
    calcPrecessionRigorous(properMotionRa, properMotionDec, coordJDYear, deltaJulianYear, &precessedRa, &precessedDec);
    totalDeltaRa += precessedRa;
    totalDeltaDec += precessedDec;
    //celestialParameters(julianYearSinceJD2000);
    calcNutation(properMotionRa, properMotionDec, &nutationRa, &nutationDec);
    totalDeltaRa += nutationRa;
    totalDeltaDec += nutationDec;
    calcAnnualAberration(properMotionRa, properMotionDec, &aberrationRa, &aberrationDec);
    totalDeltaRa += aberrationRa;
    totalDeltaDec += aberrationDec;

    (*deltaRa) = totalDeltaRa;
    (*deltaDec) = totalDeltaDec;

#ifdef SERIAL_DEBUG
    Serial.print("calcProperMotionPrecessionNutationAberration() - RA(before): ");
    Serial.print(rad2hms(ra, true, true));
    Serial.print(" DEC(before): ");
    Serial.print(rad2dms(dec, true, false));
    Serial.print(" DELTA JULIAN YEAR: ");
    Serial.print(deltaJulianYear);
    Serial.println("");
    Serial.print(" ---> P_MOTION RA: ");
    Serial.print(rad2hms(properMotionRa, true, false));
    Serial.print(" P_MOTION DEC: ");
    Serial.print(rad2dms(properMotionDec, true, false));
    Serial.print(" PRECESSED RA: ");
    Serial.print(rad2hms(precessedRa, true, true));
    Serial.print(" PRECESSED DEC: ");
    Serial.print(rad2dms(precessedDec, true, true));
    Serial.println("");
    Serial.print(" NUTATION RA: ");
    Serial.print(rad2hms(nutationRa, true, true));
    Serial.print(" NUTATION DEC: ");
    Serial.print(rad2dms(nutationDec, true, true));
    Serial.print(" ABERRATION RA: ");
    Serial.print(rad2hms(aberrationRa, true, true));
    Serial.print(" ABERRATION DEC: ");
    Serial.print(rad2dms(aberrationDec, true, true));
    Serial.println("");
    Serial.print(" DELTA_RA: ");
    Serial.print(rad2hms(totalDeltaRa, true, true));
    Serial.print(" DELTA_DEC: ");
    Serial.print(rad2dms(totalDeltaDec, true, true));
    Serial.println("");
#endif
}

void calcCelestialParams()
{
    double currJDYear = getJulianYear(JDN);
    double julianYearSinceJD2000 = currJDYear - 2000;
    celestialParameters(julianYearSinceJD2000);
}

double calcAngularSepHaversine(Position &aStar, Position &bStar, boolean equatorial)
{
    if (equatorial)
        return (2 * asin(sqrt(hav(bStar.dec - aStar.dec) + cos(aStar.dec) * cos(bStar.dec) * hav(bStar.ra - aStar.ra))));
    else
        return (2 * asin(sqrt(hav(bStar.alt - aStar.alt) + cos(aStar.alt) * cos(bStar.alt) * hav(bStar.az - aStar.az))));
}

double calcAngularSepUsingEquatorial(Position &aStar, Position &bStar)
{
    double aHa = aStar.sidT - aStar.ra;
    double bHa = bStar.sidT - bStar.ra;
    double deltaHa = aHa - bHa;

    return acos(sin(aStar.dec) * sin(bStar.dec) + cos(aStar.dec) * cos(bStar.dec) * cos(deltaHa));
}

double calcAngularSepUsingAltAz(Position &aStar, Position &bStar)
{
    double deltaAz = aStar.az - bStar.az;

    return acos(sin(aStar.alt) * sin(bStar.alt) + cos(aStar.alt) * cos(bStar.alt) * cos(deltaAz));
}

double calcAngularSepDiff(Position &aStar, Position &bStar)
{
    return fabs(fabs(calcAngularSepUsingEquatorial(aStar, bStar)) - fabs(calcAngularSepUsingAltAz(aStar, bStar)));
}

double calcAngularSepDiffHaversine(Position &aStar, Position &bStar)
{
    return fabs(fabs(calcAngularSepHaversine(aStar, bStar, true)) - fabs(calcAngularSepHaversine(aStar, bStar, false)));
}

void setRefractionWorkVars(double elevDeg, double *bp, double *ep, double *br, double *er)
{
    int i;
    for (i = 0; i < 11; i++) // Refraction table is length 12
    {
        if (elevDeg > REFRACTION_TABLE[i][0])
            break;
    }
    (*bp) = REFRACTION_TABLE[i - 1][0];
    (*ep) = REFRACTION_TABLE[i][0];
    (*br) = REFRACTION_TABLE[i - 1][1];
    (*er) = REFRACTION_TABLE[i][1];
}

double calcRefractionFromTrue(double trueElev)
{
    double bp, ep, br, er;
    double trueElevDeg = trueElev * RAD_TO_DEG;
    setRefractionWorkVars(trueElevDeg, &bp, &ep, &br, &er);
    double refraction = br + (trueElevDeg - bp) * (er - br) / (ep - bp);
    return refraction * ARCMIN_TO_RAD;
}

double calcRefractionFromApparent(double apparentElev)
{
    double bp, ep, br, er;
    double apparentElevDeg = apparentElev * RAD_TO_DEG;
    setRefractionWorkVars(apparentElevDeg, &bp, &ep, &br, &er);
    double apparentElevArcMin = apparentElevDeg * 60;
    bp *= 60;
    ep *= 60;
    double trueElevArcMin = (apparentElevArcMin * (ep - bp) - br * ep + bp * er) / (ep - bp + er - br);
    double refractionArcMin = apparentElevDeg * 60 - trueElevArcMin;
    return refractionArcMin * ARCMIN_TO_RAD;
}

void calcRefractionFromTrueEquatorialCorrection(double ra, double dec, double sidT, double lat, double *deltaRa, double *deltaDec)
{
    double alt_tmp, az_tmp, ra_tmp, dec_tmp;
    getAltAzTrig(ra, dec, sidT, lat, &alt_tmp, &az_tmp);
    alt_tmp += calcRefractionFromTrue(alt_tmp);
    getEquatTrig(alt_tmp, az_tmp, sidT, lat, &ra_tmp, &dec_tmp);
    (*deltaRa) = ra_tmp - ra;
    (*deltaDec) = dec_tmp - dec;
}

void calcRefractionFromApparentEquatorialCorrection(double ra, double dec, double sidT, double lat, double *deltaRa, double *deltaDec)
{
    double alt_tmp, az_tmp, ra_tmp, dec_tmp;
    getAltAzTrig(ra, dec, sidT, lat, &alt_tmp, &az_tmp);
    alt_tmp -= calcRefractionFromApparent(alt_tmp);
    getEquatTrig(alt_tmp, az_tmp, sidT, lat, &ra_tmp, &dec_tmp);
    (*deltaRa) = ra_tmp - ra;
    (*deltaDec) = dec_tmp - dec;
}

void calcRefractionFromApparentBennett(double alt, double &deltaAlt)
{
    double apparentAltDeg = alt * RAD_TO_DEG;
    double refract = 1 / tan(apparentAltDeg + (7.31 / (apparentAltDeg + 4.4)));
    refract = -0.06 * sin(14.7 * refract + 13);
    deltaAlt = refract * ARCMIN_TO_RAD;
}

void calcRefractionFromTrueSamundsson(double alt, double &deltaAlt)
{
    double apparentAltDeg = alt * RAD_TO_DEG;
    double refract = 1.02 / tan(alt + (10.3 / (alt + 5.11)));
    deltaAlt = refract * ARCMIN_TO_RAD;
}

void setPositionDeg(double raDeg, double decDeg, double azDeg, double altDeg, double sidTDeg, double haDeg, Position &position)
{
    position.ra = raDeg * DEG_TO_RAD;
    position.dec = decDeg * DEG_TO_RAD;
    position.az = azDeg * DEG_TO_RAD;
    position.alt = altDeg * DEG_TO_RAD;
    position.sidT = sidTDeg * DEG_TO_RAD;
    position.ha = haDeg * DEG_TO_RAD;
}

void copyPosition(Position &from, Position &to)
{
    to.ra = from.ra;
    to.dec = from.dec;
    to.az = from.az;
    to.alt = from.alt;
    to.sidT = from.sidT;
    to.ha = from.ha;
}

void clearPosition(Position &position)
{
    position.ra = 0;
    position.dec = 0;
    position.az = 0;
    position.alt = 0;
    position.sidT = 0;
    position.ha = 0;
}

double convertPrimaryAxis(double fromPri, double fromSec, double toSec, double lat)
{
    if (lat == QRT_REV)
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

void getAltAzTrig(double ra, double dec, double sidT, double lat, double *alt, double *az)
{
    double absLat = fabs(lat);
    double dec_tmp = dec;
    double ra_tmp = ra;
    if (lat < 0)
        dec_tmp = -dec_tmp;

    if (dec_tmp > QRT_REV || dec_tmp < -QRT_REV)
    {
        dec_tmp = HALF_REV - dec_tmp;
        ra_tmp = validRev(ra_tmp + HALF_REV);
    }

    double ha = sidT - ra_tmp;
    double alt_tmp = convertSecondaryAxis(ha, dec_tmp, absLat);
    double az_tmp = convertPrimaryAxis(ha, dec_tmp, alt_tmp, absLat);
    if (lat < 0)
        az_tmp = reverseRev(az_tmp);
    *alt = alt_tmp;
    *az = az_tmp;
#ifdef SERIAL_DEBUG
    Serial.print("getAltAzTrig() - RA: ");
    Serial.print(rad2hms(ra_tmp, true, true));
    Serial.print(" DEC: ");
    Serial.print(rad2dms(dec_tmp, true, false));
    Serial.print(" HA: ");
    Serial.print(rad2hms(ha, true, true));
    Serial.print(" ---> AZ: ");
    Serial.print(rad2dms(az_tmp, true, true));
    Serial.print(" ALT: ");
    Serial.print(rad2dms(alt_tmp, true, true));
    Serial.println("");
#endif
}

void getEquatTrig(double alt, double az, double sidT, double lat, double *ra, double *dec)
{
    double alt_tmp = alt;
    double az_tmp = az;
    if (alt_tmp > QRT_REV || alt_tmp < -QRT_REV)
    {
        alt_tmp = HALF_REV - alt_tmp;
        az_tmp = validRev(az_tmp + HALF_REV);
    }

    double absLat = fabs(lat);
    if (lat < 0)
        az_tmp = reverseRev(az_tmp);

    double dec_tmp = convertSecondaryAxis(az_tmp, alt_tmp, absLat);
    double ha = convertPrimaryAxis(az_tmp, alt_tmp, dec_tmp, absLat);
    *ra = validRev(sidT - ha);
    if (lat < 0)
        *dec = -dec_tmp;
    else
        *dec = dec_tmp;
#ifdef SERIAL_DEBUG
    Serial.print("getEquatTrig() - AZ: ");
    Serial.print(rad2dms(az_tmp, true, true));
    Serial.print(" ALT: ");
    Serial.print(rad2dms(alt_tmp, true, false));
    Serial.print(" ---> RA: ");
    Serial.print(rad2hms(*ra, true, true));
    Serial.print(" HA: ");
    Serial.print(rad2hms(ha, true, true));
    Serial.print(" DEC: ");
    Serial.print(rad2dms(dec_tmp, true, false));
    Serial.println("");
#endif
}

void zeroArrays(ConvertMatrixWorkingStorage &cmws)
{
    for (int i = 0; i < cmws.dimen; i++)
    {
        for (int j = 0; j < cmws.dimen; j++)
        {
            cmws.qq[i][j] = 0;
            cmws.vv[i][j] = 0;
            cmws.rr[i][j] = 0;
            cmws.xx[i][j] = 0;
            cmws.yy[i][j] = 0;
        }
    }
}

double getApparentAlt(double cosZ1, double cosZ2, double sinZ1, double sinZ2, ConvertMatrixWorkingStorage &cmws)
{
    double v1 = (sin(cmws.sec) - sinZ1 * sinZ2) * cosZ1 * (cosZ2 / ((sinZ1 * sinZ1 - 1) * (sinZ2 * sinZ2 - 1)));
    return asin(v1);
}

void angleSubroutine(ConvertMatrixWorkingStorage &cmws)
{
    double c = sqrt(sq(cmws.yy[0][1]) + sq(cmws.yy[1][1]));
    if (c == 0 && cmws.yy[2][1] > 0)
        cmws.sec = QRT_REV;
    else if (c == 0 && cmws.yy[2][1] < 0)
        cmws.sec = -QRT_REV;
    else if (c != 0)
        cmws.sec = atan(cmws.yy[2][1] / c);
    else
        cmws.sec = 0;

    if (c == 0)
        cmws.pri = 1000.0 * DEG_TO_RAD;
    else if (c != 0 && cmws.yy[0][1] == 0 && cmws.yy[1][1] > 0)
        cmws.pri = QRT_REV;
    else if (c != 0 && cmws.yy[0][1] == 0 && cmws.yy[1][1] < 0)
        cmws.pri = THREE_QRT_REV;
    else if (cmws.yy[0][1] > 0)
        cmws.pri = atan(cmws.yy[1][1] / cmws.yy[0][1]);
    else if (cmws.yy[0][1] < 0)
        cmws.pri = atan(cmws.yy[1][1] / cmws.yy[0][1]) + HALF_REV;
    else
        cmws.pri = 0;

    cmws.pri = validRev(cmws.pri);
}

void z12TakiMountErrSimple(double cosPri, double cosSec, double sinPri, double sinSec, ConvertMatrixWorkingStorage &cmws)
{
    cmws.yy[0][1] = cosSec * cosPri + cmws.z2Error * sinPri - cmws.z1Error * sinSec * sinPri;
    cmws.yy[1][1] = cosSec * sinPri - cmws.z2Error * cosPri - cmws.z1Error * sinSec * cosPri;
    cmws.yy[2][1] = sinSec;
}

void z12TakiMountErrSmallAngle(double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2, ConvertMatrixWorkingStorage &cmws)
{
    cmws.yy[0][1] = (cosSec * cosPri + sinPri * cosZ1 * sinZ2 - sinSec * sinPri * sinZ1 * cosZ2) / cosZ2;
    cmws.yy[1][1] = (cosSec * sinPri - cosPri * cosZ1 * sinZ2 + sinSec * cosPri * sinZ1 * cosZ2) / cosZ2;
    cmws.yy[2][1] = (sinSec - sinZ1 * sinZ2) / (cosZ1 * cosZ2);
}

void z12BellIterative(double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2, ConvertMatrixWorkingStorage &cmws)
{
    double goalSeek;
    double trueAz = cmws.pri;
    double tanTrueAz = tan(trueAz);
    double apparentAlt = getApparentAlt(cosZ1, cosZ2, sinZ1, sinZ2, cmws);
    double cosApparentSec = cos(apparentAlt);
    double sinApparentSec = sin(apparentAlt);
    double g = cosZ2 * sinZ1 * sinApparentSec * tanTrueAz - tanTrueAz * sinZ2 * cosZ1 - cosZ2 * cosApparentSec;
    double h = sinZ2 * cosZ1 - cosZ2 * sinZ1 * sinApparentSec - tanTrueAz * cosZ2 * cosApparentSec;

    z12TakiMountErrSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2, cmws);
    angleSubroutine(cmws);

    double apparentAz = cmws.pri;
    double bestApparentAz = apparentAz;
    double holdGoalSeek = LONG_MAX;
    double incr = ARCMIN_TO_RAD;
    double minIncr = ARCSEC_TO_RAD;
    boolean dir = true;
    int subrLCount = 0;

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

    cmws.yy[0][1] = cos(bestApparentAz) * cos(apparentAlt);
    cmws.yy[1][1] = sin(bestApparentAz) * cos(apparentAlt);
    cmws.yy[2][1] = sin(apparentAlt);
}

void z12TakiIterative(double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2, ConvertMatrixWorkingStorage &cmws)
{
    int maxLoopCount = 25;
    int subTCount = 0;
    double hold_pri = cmws.pri;
    double hold_sec = cmws.sec;
    double lastPri, lastSec, errPri, errSec, cosF1, sinF1;

    lastPri = LONG_MAX / 2;
    lastSec = LONG_MAX / 2;

    z12TakiMountErrSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2, cmws);
    do
    {
        angleSubroutine(cmws);
        errPri = fabs(lastPri - cmws.pri);
        errSec = fabs(lastSec - cmws.sec);
        lastPri = cmws.pri;
        lastSec = cmws.sec;
        cosF1 = cos(cmws.pri);
        sinF1 = sin(cmws.pri);

        cmws.yy[0][1] = (cosSec * cosPri + sinF1 * cosZ1 * sinZ2 - (sinSec - sinZ1 * sinZ2) * sinF1 * sinZ1 / cosZ1) / cosZ2;
        cmws.yy[1][1] = (cosSec * sinPri - cosF1 * cosZ1 * sinZ2 + (sinSec - sinZ1 * sinZ2) * cosF1 * sinZ1 / cosZ1) / cosZ2;
        cmws.yy[2][1] = (sinSec - sinZ1 * sinZ2) / (cosZ1 * cosZ2);

        subTCount++;
        if (subTCount > maxLoopCount)
        {
            cmws.pri = hold_pri;
            cmws.sec = hold_sec;
            z12BellIterative(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2, cmws);
        }
    } while (errSec > TENTH_ARCSEC_TO_RAD || errPri > TENTH_ARCSEC_TO_RAD);
}

void z12BellTakiIterative(double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2, ConvertMatrixWorkingStorage &cmws)
{
    double apparentAlt = getApparentAlt(cosZ1, cosZ2, sinZ1, sinZ2, cmws);

    z12TakiIterative(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2, cmws);
    angleSubroutine(cmws);

    double cosSec_tmp = cos(apparentAlt);
    double sinSec_tmp = sin(apparentAlt);
    double cosPri_tmp = cos(cmws.pri);
    double sinPri_tmp = sin(cmws.pri);

    cmws.yy[0][1] = cosPri_tmp * cosSec_tmp;
    cmws.yy[1][1] = sinPri_tmp * cosSec_tmp;
    cmws.yy[2][1] = sinSec_tmp;
}

void subroutineSwitcher(ConvertMatrixWorkingStorage &cmws)
{
    double cosPri = cos(cmws.pri);
    double cosSec = cos(cmws.sec);
    double sinPri = sin(cmws.pri);
    double sinSec = sin(cmws.sec);

    if (cmws.z1Error == 0 && cmws.z2Error == 0)
    {
        cmws.yy[0][1] = cosPri * cosSec;
        cmws.yy[1][1] = sinPri * cosSec;
        cmws.yy[2][1] = sinSec;
    }
    else
    {
        double cosZ1 = cos(cmws.z1Error);
        double cosZ2 = cos(cmws.z2Error);
        double sinZ1 = sin(cmws.z1Error);
        double sinZ2 = sin(cmws.z2Error);

        if (cmws.convertSubroutineType == 0)
            z12TakiMountErrSimple(cosPri, cosSec, sinPri, sinSec, cmws);
        else if (cmws.convertSubroutineType == 1)
            z12TakiMountErrSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2, cmws);
        else if (cmws.convertSubroutineType == 2)
            z12BellIterative(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2, cmws);
        else if (cmws.convertSubroutineType == 3)
            z12TakiIterative(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2, cmws);
        else if (cmws.convertSubroutineType == 4)
            z12BellTakiIterative(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2, cmws);
    }
}

void determinateSubroutine(ConvertMatrixWorkingStorage &cmws)
{
    cmws.determinate = cmws.vv[0][1] * cmws.vv[1][2] * cmws.vv[2][3] +
                       cmws.vv[0][2] * cmws.vv[1][3] * cmws.vv[2][1] +
                       cmws.vv[0][3] * cmws.vv[2][2] * cmws.vv[1][1] -
                       cmws.vv[0][3] * cmws.vv[1][2] * cmws.vv[2][1] -
                       cmws.vv[0][1] * cmws.vv[2][2] * cmws.vv[1][3] -
                       cmws.vv[0][2] * cmws.vv[1][1] * cmws.vv[2][3];
}

void calcRectCoords(ConvertMatrixWorkingStorage &cmws)
{
    double cosPri = cos(cmws.pri);
    double cosSec = cos(cmws.sec);
    double sinPri = sin(cmws.pri);
    double sinSec = sin(cmws.sec);

    if (cmws.z1Error == 0 && cmws.z2Error == 0)
    {
        cmws.yy[0][0] = cosPri * cosSec;
        cmws.yy[1][0] = sinPri * cosSec;
        cmws.yy[2][0] = sinSec;
    }
    else
    {
        double cosZ1 = cos(Z1_ERR);
        double cosZ2 = cos(Z2_ERR);
        double sinZ1 = sin(Z1_ERR);
        double sinZ2 = sin(Z2_ERR);
        cmws.yy[0][0] = cosPri * cosSec * cosZ2 - sinPri * cosZ1 * sinZ2 + sinPri * sinSec * sinZ1 * cosZ2;
        cmws.yy[1][0] = sinPri * cosSec * cosZ2 + cosPri * sinZ2 * cosZ1 - cosPri * sinSec * sinZ1 * cosZ2;
        cmws.yy[2][0] = sinSec * cosZ1 * cosZ2 + sinZ1 * sinZ2;
    }
}

void getAltAzMatrix(ConvertMatrixWorkingStorage &cmws)
{
    // HA is CCW so this HA formula is written backwards
    double ha = cmws.current.ra - cmws.current.sidT;
    //double ha = cmws.current.sidT - cmws.current.ra;
    double cosDec = cos(cmws.current.dec);
    double sinDec = sin(cmws.current.dec);
    cmws.xx[0][1] = cosDec * cos(ha);
    cmws.xx[1][1] = cosDec * sin(ha);
    cmws.xx[2][1] = sinDec;
    cmws.xx[0][1] = 0;
    cmws.xx[1][1] = 0;
    cmws.xx[2][1] = 0;

    // Multiply X by transform matrix R to get equatorial rectangular coordinates
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cmws.yy[i][1] += cmws.rr[i][j + 1] * cmws.xx[j][1];
    // Convert to Celestial Coords
    angleSubroutine(cmws);
    // Modify for Z1/Z2/Z3 Errors
    subroutineSwitcher(cmws);
    angleSubroutine(cmws);

    cmws.current.alt = cmws.sec;
    cmws.current.az = reverseRev(validRev(cmws.pri));
    // cmws.current.az = validRev(cmws.pri);
    cmws.current.alt -= cmws.z3Error;
#ifdef SERIAL_DEBUG
    Serial.print("getAltAzMatrix() - RA: ");
    Serial.print(rad2hms(cmws.current.ra, true, true));
    Serial.print(" DEC: ");
    Serial.print(rad2dms(cmws.current.dec, true, false));
    Serial.print(" HA: ");
    Serial.print(rad2hms(ha, true, true));
    Serial.print(" ---> AZ: ");
    Serial.print(rad2dms(cmws.current.az, true, true));
    Serial.print(" ALT: ");
    Serial.print(rad2dms(cmws.current.alt, true, true));
    Serial.println("");
#endif
}

void getEquatorialMatrix(ConvertMatrixWorkingStorage &cmws)
{
    double hold_alt = cmws.current.alt;
    double hold_az = cmws.current.az;
    cmws.current.alt += cmws.z3Error;
    cmws.sec = cmws.current.alt;
    // Reverse CW to CCW
    cmws.pri = reverseRev(cmws.current.az);
    //cmws.pri = cmws.current.az;

    cmws.current.alt = hold_alt;
    cmws.current.az = hold_az;

    calcRectCoords(cmws);
    cmws.xx[0][1] = cmws.yy[0][0];
    cmws.xx[1][1] = cmws.yy[1][0];
    cmws.xx[2][1] = cmws.yy[2][0];
    cmws.yy[0][1] = 0;
    cmws.yy[1][1] = 0;
    cmws.yy[2][1] = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cmws.yy[i][1] += (cmws.qq[i][j + 1] * cmws.xx[j][1]);

    angleSubroutine(cmws);
    cmws.pri += cmws.current.sidT;
    cmws.current.ra = validRev(cmws.pri);
    // cmws.current.ra = cmws.pri - round(cmws.pri / ONE_REV) * ONE_REV;
    cmws.current.dec = cmws.sec;
#ifdef SERIAL_DEBUG
    Serial.print("getEquatMatrix() - AZ: ");
    Serial.print(rad2dms(cmws.current.az, true, true));
    Serial.print(" ALT: ");
    Serial.print(rad2dms(cmws.current.alt, true, false));
    Serial.print(" ---> RA: ");
    Serial.print(rad2hms(cmws.current.ra, true, true));
    Serial.print(" HA: ");
    Serial.print(rad2hms(cmws.current.ra - cmws.current.sidT, true, true));
    Serial.print(" DEC: ");
    Serial.print(rad2dms(cmws.current.dec, true, false));
    Serial.println("");
#endif
}

void addAlignStar(int i, ConvertMatrixWorkingStorage &cmws)
{
    if (i == 1)
        zeroArrays(cmws);

    // CCW
    double ha = cmws.current.ra - cmws.current.sidT;
    //double ha = cmws.current.sidT - cmws.current.ra;
    double cosDec = cos(cmws.current.dec);
    double sinDec = sin(cmws.current.dec);
    cmws.xx[0][i] = cosDec * cos(ha);
    cmws.xx[1][i] = cosDec * sin(ha);
    cmws.xx[2][i] = sinDec;

    cmws.pri = reverseRev(cmws.current.az);
    // cmws.pri = cmws.current.az;
    cmws.sec = cmws.current.alt + cmws.z3Error;
    calcRectCoords(cmws);
    cmws.yy[0][i] = cmws.yy[0][0];
    cmws.yy[1][i] = cmws.yy[1][0];
    cmws.yy[2][i] = cmws.yy[2][0];
}

void generateThirdStar(ConvertMatrixWorkingStorage &cmws)
{
    cmws.xx[0][3] = cmws.xx[1][1] * cmws.xx[2][2] - cmws.xx[2][1] * cmws.xx[1][2];
    cmws.xx[1][3] = cmws.xx[2][1] * cmws.xx[0][2] - cmws.xx[0][1] * cmws.xx[2][2];
    cmws.xx[2][3] = cmws.xx[0][1] * cmws.xx[1][2] - cmws.xx[1][1] * cmws.xx[0][2];

    double a = sqrt(sq(cmws.xx[0][3]) + sq(cmws.xx[1][3]) + sq(cmws.xx[2][3]));
    for (int i = 0; i < 3; i++)
    {
        if (a == 0)
            cmws.xx[i][3] = LONG_MAX;
        else
            cmws.xx[i][3] /= a;
    }

    cmws.yy[0][3] = cmws.yy[1][1] * cmws.yy[2][2] - cmws.yy[2][1] * cmws.yy[1][2];
    cmws.yy[1][3] = cmws.yy[2][1] * cmws.yy[0][2] - cmws.yy[0][1] * cmws.yy[2][2];
    cmws.yy[2][3] = cmws.yy[0][1] * cmws.yy[1][2] - cmws.yy[1][1] * cmws.yy[0][2];

    a = sqrt(sq(cmws.yy[0][3]) + sq(cmws.yy[1][3]) + sq(cmws.yy[2][3]));
    for (int i = 0; i < 3; i++)
    {
        if (a == 0)
            cmws.yy[i][3] = LONG_MAX;
        else
            cmws.yy[i][3] /= a;
    }
}

void transformMatrix(ConvertMatrixWorkingStorage &cmws)
{
    double e;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            cmws.vv[i][j + 1] = cmws.xx[i][j + 1];
    }

    determinateSubroutine(cmws);
    e = cmws.determinate;
#ifdef SERIAL_DEBUG
    Serial.print("Determinate: ");
    Serial.print(e);
    Serial.println("");
#endif
    for (int m = 0; m < 3; m++)
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cmws.vv[i][j + 1] = cmws.xx[i][j + 1];
        for (int n = 0; n < 3; n++)
        {
            cmws.vv[0][m + 1] = 0;
            cmws.vv[1][m + 1] = 0;
            cmws.vv[2][m + 1] = 0;
            cmws.vv[n][m + 1] = 1;
            determinateSubroutine(cmws);
            if (e == 0)
                cmws.qq[m][n + 1] = LONG_MAX;
            else
                cmws.qq[m][n + 1] = cmws.determinate / e;
        }
    }

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cmws.rr[i][j + 1] = 0;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int l = 0; l < 3; l++)
                cmws.rr[i][j + 1] += (cmws.yy[i][l + 1] * cmws.qq[l][j + 1]);

    for (int m = 0; m < 3; m++)
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cmws.vv[i][j + 1] = cmws.rr[i][j + 1];
        determinateSubroutine(cmws);
        e = cmws.determinate;
#ifdef SERIAL_DEBUG
        Serial.print("Determinate: ");
        Serial.print(e);
        Serial.println("");
#endif
        for (int n = 0; n < 3; n++)
        {
            cmws.vv[0][m + 1] = 0;
            cmws.vv[1][m + 1] = 0;
            cmws.vv[2][m + 1] = 0;
            cmws.vv[n][m + 1] = 1;
            determinateSubroutine(cmws);
            if (e == 0)
                cmws.qq[m][n + 1] = LONG_MAX;
            else
                cmws.qq[m][n + 1] = cmws.determinate / e;
        }
    }
}

// void initMatrix(int starNum, ConvertMatrixWorkingStorage &cmws)
// {
//     if (starNum == 3 && cmws.alignPositions == 2)
//     {
// #ifdef SERIAL_DEBUG
//         Serial.print("initMatrix() - Star Num: 3 - Adding 3rd Star");
//         Serial.println("");
// #endif
//         copyPosition(cmws.current, cmws.alignMatrixPosition);
//         copyPosition(cmws.one, cmws.current);
//         addAlignStar(1, cmws);
//         copyPosition(cmws.two, cmws.current);
//         addAlignStar(2, cmws);
//         copyPosition(cmws.alignMatrixPosition, cmws.current);
//         addAlignStar(3, cmws);
//         transformMatrix(cmws);
//         copyPosition(cmws.current, cmws.three);
//         cmws.alignPositions = 3;
//     }
//     else if (starNum == 2 && cmws.alignPositions == 3)
//     {
// #ifdef SERIAL_DEBUG
//         Serial.print("initMatrix() - Star Num: 2 - Adding 2nd Star");
//         Serial.println("");
// #endif
//         copyPosition(cmws.current, cmws.alignMatrixPosition);
//         copyPosition(cmws.one, cmws.current);
//         addAlignStar(1, cmws);
//         copyPosition(cmws.alignMatrixPosition, cmws.current);
//         addAlignStar(2, cmws);
//         copyPosition(cmws.current, cmws.alignMatrixPosition);
//         copyPosition(cmws.three, cmws.current);
//         addAlignStar(3, cmws);
//         copyPosition(cmws.alignMatrixPosition, cmws.current);
//         transformMatrix(cmws);
//         copyPosition(cmws.current, cmws.two);
//     }
//     else if (starNum == 2 && (cmws.alignPositions == 1 || cmws.alignPositions == 2))
//     {
// #ifdef SERIAL_DEBUG
//         Serial.print("initMatrix() - Star Num: 2 - Adding 1st, 2nd, Generating 3rd Star");
//         Serial.println("");
// #endif
//         copyPosition(cmws.current, cmws.alignMatrixPosition);
//         copyPosition(cmws.one, cmws.current);
//         addAlignStar(1, cmws);
//         copyPosition(cmws.alignMatrixPosition, cmws.current);
//         addAlignStar(2, cmws);
//         generateThirdStar(cmws);
//         transformMatrix(cmws);
//         copyPosition(cmws.current, cmws.two);
//         cmws.alignPositions = 2;
//     }
//     else if (starNum == 1 && cmws.alignPositions == 3)
//     {
// #ifdef SERIAL_DEBUG
//         Serial.print("initMatrix() - Star Num: 1 - Adding 1st, 2nd, 3rd Star");
//         Serial.println("");
// #endif
//         addAlignStar(1, cmws);
//         copyPosition(cmws.current, cmws.alignMatrixPosition);
//         copyPosition(cmws.two, cmws.current);
//         addAlignStar(2, cmws);
//         copyPosition(cmws.three, cmws.current);
//         addAlignStar(3, cmws);
//         copyPosition(cmws.alignMatrixPosition, cmws.current);
//         transformMatrix(cmws);
//         copyPosition(cmws.current, cmws.one);
//     }
//     else if (starNum == 1 && cmws.alignPositions == 2)
//     {
// #ifdef SERIAL_DEBUG
//         Serial.print("initMatrix() - Star Num: 1 - Adding 1st, 2nd, Generating 3rd Star");
//         Serial.println("");
// #endif
//         addAlignStar(1, cmws);
//         copyPosition(cmws.current, cmws.alignMatrixPosition);
//         copyPosition(cmws.two, cmws.current);
//         addAlignStar(2, cmws);
//         copyPosition(cmws.alignMatrixPosition, cmws.current);
//         generateThirdStar(cmws);
//         transformMatrix(cmws);
//         copyPosition(cmws.current, cmws.one);
//     }
//     else if (starNum == 1 && (cmws.alignPositions == 0 || cmws.alignPositions == 1))
//     {
// #ifdef SERIAL_DEBUG
//         Serial.print("initMatrix() - Star Num: 1 - Adding 1st Star");
//         Serial.println("");
// #endif
//         addAlignStar(1, cmws);
//         copyPosition(cmws.current, cmws.one);
//         cmws.alignPositions = 1;
//     }
//     else
//     {
//         Serial.print("MATRIX INIT ERROR!");
//     }
// }

void initMatrix(int starNum, ConvertMatrixWorkingStorage &cmws)
{
    if (starNum == 2)
    {
        generateThirdStar(cmws);
        transformMatrix(cmws);
        cmws.alignPositions = 3;
    }
    else if (starNum == 3)
    {
        transformMatrix(cmws);
        cmws.alignPositions = 3;
    }
    else
    {
#ifdef SERIAL_DEBUG
        Serial.print("MATRIX INIT ERROR!");
        Serial.println("");
#endif
    }
}

double calcAltOffsetDirectly(Position &aStar, Position &bStar)
{
    double n = cos(aStar.az - bStar.az);
    double m = cos(calcAngularSepUsingEquatorial(aStar, bStar));
    double x = (2 * m - (n + 1) * cos(aStar.alt - bStar.alt)) / (n - 1);
    double a1 = 0.5 * (acos(x) - aStar.alt - bStar.alt);
    double a2 = 0.5 * (-acos(x) - aStar.alt - bStar.alt);
#ifdef SERIAL_DEBUG
    Serial.print("calcAltOffsetDirectly - n: ");
    Serial.print(n);
    Serial.print(" m: ");
    Serial.print(m);
    Serial.print(" x: ");
    Serial.print(x);
    Serial.print(" a1: ");
    Serial.print(a1);
    Serial.print(" a2: ");
    Serial.print(a2);
    Serial.println("");
#endif
    if (abs(a1) < abs(a2))
        return a1;
    else
        return a2;
}

double calcAltOffsetIteratively(Position &aStar, Position &bStar)
{
    double diff, bestAltOffset, aAlt_tmp, bAlt_tmp;
    double searchRange = 45 * DEG_TO_RAD;
    double altIncr = ARCSEC_TO_RAD;
    double maxIter = searchRange / altIncr;
    double bestDiff = LONG_MAX;
    double lastDiff = LONG_MAX;
    double i = 0;

    aAlt_tmp = aStar.alt;
    bAlt_tmp = bStar.alt;

    while (i < maxIter)
    {
        //diff = calcAngularSepDiff(aStar, bStar);
        diff = calcAngularSepDiffHaversine(aStar, bStar);
        if (diff < bestDiff)
        {
            bestDiff = diff;
            bestAltOffset = aAlt_tmp - aStar.alt;
        }
        if (diff > lastDiff)
            break;
        else
            lastDiff = diff;
        i++;
        aAlt_tmp += altIncr;
        bAlt_tmp += altIncr;
    }

    aAlt_tmp = aStar.alt;
    bAlt_tmp = bStar.alt;
    lastDiff = LONG_MAX;
    i = 0;
    while (i < maxIter)
    {
        //diff = calcAngularSepDiff(aStar, bStar);
        diff = calcAngularSepDiffHaversine(aStar, bStar);
        if (diff < bestDiff)
        {
            bestDiff = diff;
            bestAltOffset = aAlt_tmp - aStar.alt;
        }
        if (diff > lastDiff)
            break;
        else
            lastDiff = diff;
        i++;
        aAlt_tmp -= altIncr;
        bAlt_tmp -= altIncr;
    }
    return bestAltOffset;
}

double getAltOffset(Position &aStar, Position &bStar)
{
    try
    {
        return calcAltOffsetDirectly(aStar, bStar);
    }
    catch (const std::exception &e)
    {
        return calcAltOffsetIteratively(aStar, bStar);
    }
}

double bestZ3(ConvertMatrixWorkingStorage &cmws)
{
    double accumAltOffset;
    int count;

    accumAltOffset = getAltOffset(cmws.one, cmws.two);
    count = 1;
    if (cmws.alignPositions == 3)
    {
        accumAltOffset += getAltOffset(cmws.one, cmws.three);
        count++;
        accumAltOffset += getAltOffset(cmws.two, cmws.three);
        count++;
    }

    return accumAltOffset / count;
}

double calcPointError(ConvertMatrixWorkingStorage &cmws)
{
    double pointingErrorRMS, pointingErrorRMSTotal, altError, azError;
    copyPosition(cmws.current, cmws.calcPointErrorsPosition);
    pointingErrorRMSTotal = 0;

    // First Check Pos 1
    copyPosition(cmws.one, cmws.current);
    // getAltAz with trial z123, compare to given altaz
    getAltAzMatrix(cmws);
    // If scope aimed higher or more CW than it should, define as +ve error
    altError = cmws.one.alt - cmws.current.alt;
    // Azimuth errors in terms of true field decr towards zenith
    azError = (cmws.one.az - cmws.current.az) * cos(cmws.one.alt);
    pointingErrorRMS = sqrt(sq(altError) + sq(azError));
    pointingErrorRMSTotal += pointingErrorRMS;

    // Next check Pos 2
    copyPosition(cmws.two, cmws.current);
    // getAltAz with trial z123, compare to given altaz
    getAltAzMatrix(cmws);
    // If scope aimed higher or more CW than it should, define as +ve error
    altError = cmws.two.alt - cmws.current.alt;
    // Azimuth errors in terms of true field decr towards zenith
    azError = (cmws.two.az - cmws.current.az) * cos(cmws.two.alt);
    pointingErrorRMS = sqrt(sq(altError) + sq(azError));
    pointingErrorRMSTotal += pointingErrorRMS;

    // If we have pos 3 check that
    if (cmws.alignPositions == 3)
    {
        copyPosition(cmws.three, cmws.current);
        // getAltAz with trial z123, compare to given altaz
        getAltAzMatrix(cmws);
        // If scope aimed higher or more CW than it should, define as +ve error
        altError = cmws.three.alt - cmws.current.alt;
        // Azimuth errors in terms of true field decr towards zenith
        azError = (cmws.three.az - cmws.current.az) * cos(cmws.three.alt);
        pointingErrorRMS = sqrt(sq(altError) + sq(azError));
        pointingErrorRMSTotal += pointingErrorRMS;
    }

    copyPosition(cmws.calcPointErrorsPosition, cmws.current);

    return pointingErrorRMSTotal / cmws.alignPositions;
}

void bestZ12Core(ConvertMatrixWorkingStorage &cmws, double z1, double z2, double &bestZ1, double &bestZ2, double bestZ3, double &bestPointingErrorRMS)
{
    cmws.z1Error = z1;
    cmws.z2Error = z2;
    cmws.z3Error = bestZ3;
    copyPosition(cmws.one, cmws.current);
    initMatrix(1, cmws);

    double pointingErrorRMS = calcPointError(cmws);
    if (pointingErrorRMS < bestPointingErrorRMS - ARCSEC_TO_RAD)
    {
        bestPointingErrorRMS = pointingErrorRMS;
        bestZ1 = z1;
        bestZ2 = z2;
    }
}

void bestZ12(ConvertMatrixWorkingStorage &cmws, double bestZ3, double z1Range, double z2Range, double resolution, double &z1, double &z2)
{

    double bestPointingErrorRMS = LONG_MAX;
    double bestZ1 = 0;
    double bestZ2 = 0;

    double z1_tmp = cmws.z1Error;
    double z2_tmp = cmws.z2Error;
    double z3_tmp = cmws.z3Error;

    copyPosition(cmws.current, cmws.bestZ12Position);

    for (int z1 = 0; z1 < z2Range; z1 += resolution)
    {
        for (int z2 = 0; z2 < z1Range; z2 += resolution)
        {
            bestZ12Core(cmws, z1, z2, bestZ1, bestZ2, bestZ3, bestPointingErrorRMS);
        }
    }
    for (int z1 = 0; z1 < z2Range; z1 += resolution)
    {
        for (int z2 = 0; z2 < z1Range; z2 += resolution)
        {
            bestZ12Core(cmws, z1, z2, bestZ1, bestZ2, bestZ3, bestPointingErrorRMS);
        }
    }
    cmws.z1Error = z1_tmp;
    cmws.z2Error = z2_tmp;
    cmws.z3Error = z3_tmp;

    copyPosition(cmws.one, cmws.current);
    initMatrix(1, cmws);
    copyPosition(cmws.bestZ12Position, cmws.current);

    z1 = bestZ1;
    z2 = bestZ2;
}

void bestZ123(ConvertMatrixWorkingStorage &cmws, double &z1Error, double &z2Error, double &z3Error)
{
    double z1Result, z2Result;
    double z3Result = bestZ3(cmws);
    double z1Range = DEG_TO_RAD;
    double z2Range = DEG_TO_RAD;
    double resolution = ARCMIN_TO_RAD;
    bestZ12(cmws, z3Result, z1Range, z2Range, resolution, z1Result, z2Result);
    z1Error = z1Result;
    z2Error = z2Result;
    z3Error = z3Result;
}

void bestZ13(ConvertMatrixWorkingStorage &cmws, double &z1Error, double &z3Error)
{
    double z1Result, z2Result;
    double z3Result = bestZ3(cmws);
    double z1Range = 5 * DEG_TO_RAD;
    double z2Range = 0;
    double resolution = ARCMIN_TO_RAD;
    bestZ12(cmws, z3Result, z1Range, z2Range, resolution, z1Result, z2Result);
    z1Error = z1Result;
    z3Error = z3Result;
}

void calculateLST()
{
    NOW = rtc.GetDateTime();
    int D = NOW.Day();
    int M = NOW.Month();
    int Y = NOW.Year();
    int H = NOW.Hour();
    int MN = NOW.Minute();
    int S = NOW.Second();
    if (SUMMER_TIME == 1)
    {
        H -= 1;
    }
    if (M < 3)
    {
        M = M + 12;
        Y = Y - 1;
    }

    float HH = H + ((float)MN / 60.00) + ((float)S / 3600.00);
    float AA = (int)(JDYEAR * (Y + 4716));
    float BB = (int)(30.6001 * (M + 1));
    JDN = AA + BB + D - 1537.5 + (HH - TIME_ZONE) / 24;

    //calculate terms required for LST calcuation and calculate GMST using an approximation
    double MJD = JDN - JDMOD;
    int MJD0 = (int)MJD;
    float ut = (MJD - MJD0) * 24.0;
    double t_eph = (MJD0 - 51544.5) / JDCENTURY;
    double GMST = 6.697374558 + SIDEREAL_FRACTION * ut + (8640184.812866 + (0.093104 - 0.0000062 * t_eph) * t_eph) * t_eph / 3600.0;

    LST = GMST + OBSERVATION_LONGITUDE / 15.0;

    //reduce it to 24 format
    int LSTint = (int)LST;
    LSTint /= 24;
    LST = LST - (double)LSTint * 24;
    LST_RADS = LST / 12.0 * PI;
#ifdef SERIAL_DEBUG
    Serial.print("calculateLST() - JDN: ");
    Serial.print(JDN);
    Serial.print(" LST: ");
    Serial.print(LST);
    Serial.print(" LST_RADS: ");
    Serial.print(LST_RADS);
    Serial.println("");
#endif
}

void selectObject(int index_, int objects)
{
    if (objects == 0)
    {
        // I've selected a Messier Object
        if (LOADED_STARS != 2)
        {
            loadStarsFromSPIFFS("/messier.csv");
            LOADED_STARS = 2;
        }
        processObject(index_, CURRENT_OBJECT);
        objectAltAz();
    }
    else if (objects == 1)
    {
        // I've selected a Treasure Object
        if (LOADED_STARS != 3)
        {
            loadStarsFromSPIFFS("/treasure.csv");
            LOADED_STARS = 3;
        }
        processObject(index_, CURRENT_OBJECT);
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

void loadStarsFromSPIFFS(String filename)
{
    char in_char;
    String items = "";
    int j = 0;
    int k = 0;
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
            k += 1;
            if (in_char == '\n')
            {
                OBJECTS[j] = items;
                j += 1;
                items = "";
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
            Serial.println(file.name());
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
            Serial.println(file.size());
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
    if (LOADED_STARS != 1)
    {
        loadStarsFromSPIFFS("/alignment.csv");
        LOADED_STARS = 1;
    }

    // Load each star from the alignment.csv database into the CURRENT_ALIGN_STAR object
    int n = 0;
    for (int i = 0; i < NUM_ALIGN_STARS; i++)
    {
        processAlignmentStar(i, CURRENT_ALIGN_STAR);
        //double this_alt, this_az;
        //celestialToEquatorial(CURRENT_ALIGN_STAR.ra, CURRENT_ALIGN_STAR.dec,
        //                      OBSERVATION_LATTITUDE_RADS, LST_RADS,
        //                      &this_alt, &this_az);
        getAltAzTrig(CURRENT_ALIGN_STAR.ra, CURRENT_ALIGN_STAR.dec, CURRENT_ALIGN_STAR.time, CURRENT_ALIGN_STAR.lat, &CURRENT_ALIGN_STAR.alt, &CURRENT_ALIGN_STAR.az);
        // If catalogue star is higher than 25 degrees above the horizon.
        if (CURRENT_ALIGN_STAR.alt > 0.436332313)
        {
            // Copy catalogue star to alignment star.
            ALIGNMENT_STARS[n] = CURRENT_ALIGN_STAR;
#ifdef SERIAL_DEBUG
            Serial.print("Star number ");
            Serial.print(n);
            Serial.println("");
            Serial.print(ALIGNMENT_STARS[n].name);
            Serial.print(" - RA: ");
            Serial.print(ALIGNMENT_STARS[n].ra);
            Serial.print(" DEC: ");
            Serial.print(ALIGNMENT_STARS[n].dec);
            Serial.print(" ALT: ");
            Serial.print(ALIGNMENT_STARS[n].alt);
            Serial.print(" AZ: ");
            Serial.print(ALIGNMENT_STARS[n].az);
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

void processAlignmentStar(int index, AlignmentStar &star)
{

    int i1 = OBJECTS[index].indexOf(',');
    int i2 = OBJECTS[index].indexOf(',', i1 + 1);
    int i3 = OBJECTS[index].indexOf(',', i2 + 1);
    int i4 = OBJECTS[index].indexOf(',', i3 + 1);

    star.constellation = OBJECTS[index].substring(0, i1);
    star.name = OBJECTS[index].substring(i1 + 1, i2);
    String ra = OBJECTS[index].substring(i2 + 1, i3);
    String dec = OBJECTS[index].substring(i3 + 1, i4);
    star.mag = OBJECTS[index].substring(i4 + 1, OBJECTS[index].length()).toFloat();

    float ra_h = ra.substring(0, ra.indexOf('h')).toFloat();
    float ra_m = ra.substring(ra.indexOf('h') + 1, ra.indexOf('m')).toFloat();
    float ra_s = ra.substring(ra.indexOf('m') + 1, ra.length() - 1).toFloat();

    float dec_d = dec.substring(0, dec.indexOf('°')).toFloat();
    float dec_m = dec.substring(dec.indexOf('°') + 1, ra.indexOf('\'')).toFloat();
    float dec_s = dec.substring(dec.indexOf('\'') + 1, ra.length() - 1).toFloat();

    // double decimal_ra = ((ra_h + (ra_m / 60)) / 24) * 360;
    // double decimal_rads_ra = decimal_ra * DEG_TO_RAD;
    // double decimal_rads_dec = dec_d * DEG_TO_RAD;
    // star.ra = decimal_rads_ra;
    // star.dec = decimal_rads_dec;

    star.ra = hms2rads(ra_h, ra_m, ra_s);
    star.dec = dms2rads(dec_d, dec_m, dec_s);
    star.time = LST_RADS;
    star.lat = OBSERVATION_LATTITUDE_RADS;

#ifdef SERIAL_DEBUG
    Serial.print("processAlignmentStar() ");
    Serial.print(star.name);
    Serial.print(" - RA: ");
    Serial.print(star.ra);
    Serial.print(" DEC: ");
    Serial.print(star.dec);
    Serial.println("");
#endif
    // Correct for astronomical movements and refraction if needed
    double delta_ra, delta_dec;
    if (CORRECT_PRECESSION_ETC)
    {
        calcProperMotionPrecessionNutationAberration(star.ra, star.dec, PROPER_MOTION_RA, PROPER_MOTION_DEC, &delta_ra, &delta_dec);
        star.ra += delta_ra;
        star.dec += delta_dec;
    }
    // if (CORRECT_REFRACTION)
    // {
    //     calcRefractionFromTrueEquatorialCorrection(star.ra, star.dec, star.time, star.lat, &delta_ra, &delta_dec);
    //     star.ra += delta_ra;
    //     star.dec += delta_dec;
    // }
#ifdef SERIAL_DEBUG
    Serial.print("Added Corrections - RA: ");
    Serial.print(star.ra);
    Serial.print(" DEC: ");
    Serial.print(star.dec);
    Serial.println("");
#endif
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
    object.description = OBJECTS[index].substring(i7 + 1, OBJECTS[index].length() - 1);
    String ra = OBJECTS[index].substring(i1, i2);
    String dec = OBJECTS[index].substring(i2, i3);

    float ra_h = ra.substring(1, ra.indexOf('h')).toFloat();
    float ra_m = ra.substring(ra.indexOf('h') + 1, ra.length() - 1).toFloat();

    float dec_d = dec.substring(1, dec.indexOf('°')).toFloat();
    float dec_m = dec.substring(dec.indexOf('°') + 1, dec.length() - 1).toFloat();

    // double decimal_ra = ((ra_h + (ra_m / 60)) / 24) * 360;
    // double decimal_rads_ra = decimal_ra * DEG_TO_RAD;
    // double decimal_rads_dec = (dec_d + (dec_m / 60)) * DEG_TO_RAD;
    // object.ra = decimal_rads_ra;
    // object.dec = decimal_rads_dec;

    object.ra = hms2rads(ra_h, ra_m, 0);
    object.dec = dms2rads(dec_d, dec_m, 0);

    object.constellation = OBJECTS[index].substring(i3 + 1, i4);
    object.type = OBJECTS[index].substring(i4 + 1, i5);
    object.mag = OBJECTS[index].substring(i5 + 1, i6).toFloat();
    object.size = OBJECTS[index].substring(i6 + 1, i7);
#ifdef SERIAL_DEBUG
    Serial.print("processObject() ");
    Serial.print(object.name);
    Serial.print(" - RA: ");
    Serial.print(object.ra);
    Serial.print(" DEC: ");
    Serial.print(object.dec);
    Serial.println("");
#endif
    // Correct for astronomical movements
    double delta_ra, delta_dec;
    if (CORRECT_PRECESSION_ETC)
    {
        calcProperMotionPrecessionNutationAberration(object.ra, object.dec, PROPER_MOTION_RA, PROPER_MOTION_DEC, &delta_ra, &delta_dec);
        object.ra += delta_ra;
        object.dec += delta_dec;
    }
#ifdef SERIAL_DEBUG
    Serial.print("Added Corrections - RA: ");
    Serial.print(object.ra);
    Serial.print(" DEC: ");
    Serial.print(object.dec);
    Serial.println("");
#endif
}

void printMatrix(float *A, int m, int n, String label)
{
    // A = input matrix (m x n)
    int i, j;
    Serial.println();
    Serial.println(label);
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            Serial.print(A[n * i + j]);
            Serial.print("\t");
        }
        Serial.println();
    }
}

double basic_AngDist(double ra1, double dec1, double ra2, double dec2)
{
    // Default Formula
    //return acos(sin(dec1) * sin(dec2) + cos(dec1) * cos(dec2) * cos(ra2 - ra1));

    // Haversine formula
    return (2 * asin(sqrt(hav(dec2 - dec1) + cos(dec1) * cos(dec2) * hav(ra2 - ra1))));
}

void basic_zeroArrays()
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

void basic_Init()
{
    Z1_ERR = 0; // Mount error angle between horizontal axis and vertical axis
    Z2_ERR = 0; // Mount error angle between vertical axis and telescope optical axis
    Z3_ERR = 0; // Mount error angle, zero point shift (azimuth axis of rotation vs. instrument altitude angle)
    BASIC_ALIGN_READY = false;
    basic_zeroArrays();
}

boolean basic_isReady()
{
    return BASIC_ALIGN_READY;
}

void basic_ScopeToEquatorial(double az, double alt, double sidT, double *ra, double *dec)
{
    double ra_tmp, ha_tmp, dec_tmp;
    if (CORRECT_REFRACTION && alt > -0.0349066 || alt < 1.55334) // If alt is > -2 deg or < 89 deg
    {
        double deltaAlt;
        calcRefractionFromApparentBennett(alt, deltaAlt);
        alt -= deltaAlt;
    }

    double pri = reverseRev(az);
    double sec = alt + Z3_ERR;
    basic_Subroutine1(pri, sec);
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

    basic_AngleSubroutine(&ha_tmp, &dec_tmp);
    ra_tmp = ha_tmp + sidT;
    //(*ra) = ra_tmp - round(ra_tmp / ONE_REV) * ONE_REV;
    (*ra) = validRev(ra_tmp);
    (*dec) = dec_tmp;
#ifdef SERIAL_DEBUG
    if ((millis() - UPDATE_LAST) > 5000)
    {
        Serial.print("basic_ScopeToEquatorial() - AZ: ");
        Serial.print(rad2dms(az, true, true));
        Serial.print(" ALT: ");
        Serial.print(rad2dms(alt, true, false));
        Serial.print(" ---> RA: ");
        Serial.print(rad2hms(*ra, true, true));
        Serial.print(" HA: ");
        Serial.print(rad2hms(ha_tmp, true, true));
        Serial.print(" DEC: ");
        Serial.print(rad2dms(*dec, true, false));
        Serial.println("");
    }
#endif
}

void basic_EquatorialToScope(double ra, double dec, double sidT, double *az, double *alt)
{
    double az_tmp, alt_tmp;
    double ha_tmp = ra - sidT;
    X[1][1] = cos(dec) * cos(ha_tmp);
    X[2][1] = cos(dec) * sin(ha_tmp);
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
    basic_AngleSubroutine(&az_tmp, &alt_tmp);
    basic_Subroutine2(az_tmp, alt_tmp, 1);
    basic_AngleSubroutine(&az_tmp, &alt_tmp);

    if (CORRECT_REFRACTION && alt_tmp > -0.0349066 || alt_tmp < 1.55334) // If alt is > -2 deg or < 89 deg
    {
        double deltaAlt;
        calcRefractionFromTrueSamundsson(alt_tmp, deltaAlt);
        alt_tmp += deltaAlt;
    }

    (*alt) = alt_tmp - Z3_ERR;
    (*az) = reverseRev(validRev(az_tmp));
    // (*az) = az_tmp;
#ifdef SERIAL_DEBUG
    Serial.print("basic_EquatorialToScope() - RA: ");
    Serial.print(rad2hms(ra, true, true));
    Serial.print(" DEC: ");
    Serial.print(rad2dms(dec, true, false));
    Serial.print(" HA: ");
    Serial.print(rad2hms(ha_tmp, true, true));
    Serial.print(" ---> AZ: ");
    Serial.print(rad2dms(*az, true, true));
    Serial.print(" AZ(RAW): ");
    Serial.print(rad2dms(az_tmp, true, true));
    Serial.print(" ALT: ");
    Serial.print(rad2dms(*alt, true, true));
    Serial.println("");
#endif
}

void basic_AddStar(int starNum, int totalAlignStars, double ra, double dec, double sidT, double alt, double az)
{
    //double b = ra - SIDEREAL_FRACTION * (sidT - INITIAL_TIME);
    double ha = ra - sidT;
    // double ha = sidT - ra;
    double cosDec = cos(dec);
    double sinDec = sin(dec);
    double cosHA = cos(ha);
    double sinHA = sin(ha);

#ifdef SERIAL_DEBUG
    Serial.print("basic_AddStar() - RA: ");
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

    double pri = reverseRev(az);
    double sec = alt + Z3_ERR;
    basic_Subroutine1(pri, sec);
    Y[1][starNum] = Y[1][0];
    Y[2][starNum] = Y[2][0];
    Y[3][starNum] = Y[3][0];
}

void basic_GenerateThirdStar()
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

void basic_TransformMatrix()
{
    // Transform Matrix
    for (int i = 1; i <= 3; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            V[i][j] = X[i][j];
        }
    }
    basic_DeterminateSubroutine();

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
            basic_DeterminateSubroutine();
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
        basic_DeterminateSubroutine();
        e = W;
        for (int n = 1; n <= 3; n++)
        {
            V[1][m] = 0;
            V[2][m] = 0;
            V[3][m] = 0;
            V[n][m] = 1;
            basic_DeterminateSubroutine();
            Q[m][n] = W / e;
        }
    }

    BASIC_ALIGN_READY = true;
}

void basic_DeterminateSubroutine()
{
    W = V[1][1] * V[2][2] * V[3][3] + V[1][2] * V[2][3] * V[3][1];
    W = W + V[1][3] * V[3][2] * V[2][1];
    W = W - V[1][3] * V[2][2] * V[3][1] - V[1][1] * V[3][2] * V[2][3];
    W = W - V[1][2] * V[2][1] * V[3][3];
}

void basic_AngleSubroutine(double *pri, double *sec)
{
    double c = sqrt(sq(Y[1][1]) + sq(Y[2][1]));
    // if (c == 0 && Y[3][1] > 0)
    //     (*sec) = QRT_REV;
    // else if (c == 0 && Y[3][1] < 0)
    //     (*sec) = -QRT_REV;
    // else if (c != 0)
    //     (*sec) = atan(Y[3][1] / c);
    // else 
    //     (*sec) = 0;

    // if (c == 0)
    //     (*pri) = 0;
    // else if (c != 0 && Y[1][1] == 0 && Y[2][1] > 0)
    //     (*pri) = QRT_REV;
    // else if (c != 0 && Y[1][1] == 0 && Y[2][1] < 0)
    //     (*pri) = reverseRev(QRT_REV);
    // else if (Y[1][1] > 0)
    //     (*pri) = atan(Y[2][1] / Y[1][1]);
    // else if (Y[1][1] < 0)
    //     (*pri) = atan(Y[2][1] / Y[1][1]) + HALF_REV;
    
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

void basic_Subroutine1(double pri, double sec)
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
    // else
    // {
    //     Y[1][0] = cosPri * cosSec - sinPri * Z2_ERR;
    //     Y[1][0] = Y[1][0] + sinPri * sinSec * Z1_ERR;
    //     Y[2][0] = sinPri * cosSec + cosPri * Z2_ERR;
    //     Y[2][0] = Y[2][0] - cosPri * sinSec * Z1_ERR;
    //     Y[3][0] = sinSec;
    // }
}

void basic_Subroutine2(double pri, double sec, int type)
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
            basic_TakiSimple(cosPri, cosSec, sinPri, sinSec);
        else if (type == 2)
            basic_TakiSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        else if (type == 3)
            basic_BellIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        else if (type == 4)
            basic_TakiIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        else if (type == 5)
            basic_BellTakiIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        // Y[1][1] = cosPri * cosSec + sinPri * Z2_ERR;
        // Y[1][1] = Y[1][1] - sinPri * sinSec * Z1_ERR;
        // Y[2][1] = sinPri * cosSec - cosPri * Z2_ERR;
        // Y[2][1] = Y[2][1] + cosPri * sinSec * Z1_ERR;
        // Y[3][1] = sinSec;
    }
}

void basic_TakiSimple(double cosPri, double cosSec, double sinPri, double sinSec)
{
    Y[1][1] = cosSec * cosPri + Z2_ERR * sinPri - Z1_ERR * sinSec * sinPri;
    Y[2][1] = cosSec * sinPri - Z2_ERR * cosPri - Z1_ERR * sinSec * cosPri;
    Y[3][1] = sinSec;
}

void basic_TakiSmallAngle(double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    Y[1][1] = (cosSec * cosPri + sinPri * cosZ1 * sinZ2 - sinSec * sinPri * sinZ1 * cosZ2) / cosZ2;
    Y[2][1] = (cosSec * sinPri - cosPri * cosZ1 * sinZ2 + sinSec * cosPri * sinZ1 * cosZ2) / cosZ2;
    Y[3][1] = (sinSec - sinZ1 * sinZ2) / (cosZ1 * cosZ2);
}

void basic_BellIterative(double pri, double sec, double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    double trueAz = pri;
    double tanTrueAz = tan(trueAz);
    double apparentAlt = basic_ApparentAlt(sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    double cosApparentSec = cos(apparentAlt);
    double sinApparentSec = sin(apparentAlt);
    double g = cosZ2 * sinZ1 * sinApparentSec * tanTrueAz - tanTrueAz * sinZ2 * cosZ1 - cosZ2 * cosApparentSec;
    double h = sinZ2 * cosZ1 - cosZ2 * sinZ1 * sinApparentSec - tanTrueAz * cosZ2 * cosApparentSec;

    basic_TakiSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    basic_AngleSubroutine(&pri, &sec);
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

void basic_TakiIterative(double pri, double sec, double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    int maxLoopCount = 25;
    int subrTCount = 0;
    double holdPri = pri;
    double holdSec = sec;
    double lastPri, lastSec, errPri, errSec;

    lastPri = LONG_MAX / 2;
    lastSec = LONG_MAX / 2;
    
    basic_TakiSmallAngle(cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    do 
    {
        basic_AngleSubroutine(&pri, &sec);
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
            basic_BellIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
        } 
    } while (errSec > TENTH_ARCSEC_TO_RAD || errPri > TENTH_ARCSEC_TO_RAD);
}

void basic_BellTakiIterative(double pri, double sec, double cosPri, double cosSec, double sinPri, double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    double apparentAlt = basic_ApparentAlt(sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    basic_TakiIterative(pri, sec, cosPri, cosSec, sinPri, sinSec, cosZ1, cosZ2, sinZ1, sinZ2);
    basic_AngleSubroutine(&pri, &sec);

    cosPri = cos(pri);
    sinPri = sin(pri);
    cosSec = cos(apparentAlt);
    sinSec = sin(apparentAlt);

    Y[1][1] = cosPri * cosSec;
    Y[2][1] = sinPri * cosSec;
    Y[3][1] = sinSec;
}

double basic_ApparentAlt(double sinSec, double cosZ1, double cosZ2, double sinZ1, double sinZ2)
{
    double v1 = (sinSec - sinZ1 * sinZ2) * cosZ1 * (cosZ2 / ((sinZ1 * sinZ1 - 1) * sinZ2 * sinZ2 - 1));
    return asin(v1);
}

// BEST Z1/Z2
// nRange to range is search area in radians
// incr is increment +/- radians
void basic_BestZ12(int n, double range, double resolution)
{
    double ra1, dec1;
    double ra2, dec2;
    double bestZ1 = HALF_REV;
    double bestZ2 = HALF_REV;
    double bestPointingErrorRMS = HALF_REV;
    double alt1, alt2, az1, az2, pointingErrorRMS, pointingErrorRMSTotal, altError, azError;
    pointingErrorRMSTotal = 0;

    for (Z1_ERR = -range ; Z1_ERR < range; Z1_ERR += resolution)
    {
        for (Z2_ERR = -range; Z2_ERR < range; Z2_ERR += resolution)
        {
            for (int i = 0; i < n; n++)
            {
                alt1 = ALIGNMENT_STARS[i].alt;
                az1 = ALIGNMENT_STARS[i].az;
                basic_EquatorialToScope(ALIGNMENT_STARS[i].ra, ALIGNMENT_STARS[i].dec, ALIGNMENT_STARS[i].time, &az2, &alt2);
                altError = alt1 - alt2;
                azError = (az1 - az2) * cos(alt1);
                pointingErrorRMS = sqrt(sq(altError) + sq(azError));
                pointingErrorRMSTotal += pointingErrorRMS;
            }
            pointingErrorRMSTotal /= n;
            if (pointingErrorRMSTotal < bestPointingErrorRMS - ARCSEC_TO_RAD)
            {
                bestPointingErrorRMS = pointingErrorRMSTotal;
                bestZ1 = Z1_ERR;
                bestZ2 = Z2_ERR;
            }
        }
    }

    Z1_ERR = bestZ1;
    Z2_ERR = bestZ2;
}

// BEST Z3
// nRange to range is search area in degrees
// incr is increment +/- degrees
void basic_BestZ3(int n, double startRange, double endRange, double resolution)
{
    double ra1, dec1;
    double ra2, dec2;
    double bestZ3 = HALF_REV;
    double bestDist = HALF_REV;

    // Covnert search ranges and resolution to Radians
    startRange *= DEG_TO_RAD;
    endRange *= DEG_TO_RAD;
    resolution *= DEG_TO_RAD;

    for (Z3_ERR = startRange; Z3_ERR < endRange; Z3_ERR += resolution)
    {
        for (int j = 0; j < n; j++)
        {
            basic_ScopeToEquatorial(ALIGNMENT_STARS[j].az, ALIGNMENT_STARS[j].alt, ALIGNMENT_STARS[j].time, &ra1, &dec1);
            double dist = 0;
            for (int k = 0; k < n; k++)
            {
                if (j != k)
                {
                    // Star j to k catalogue vs aligned
                    // Catalogue RA/DEC Angular Distance
                    double dist1 = basic_AngDist(ALIGNMENT_STARS[j].ra, ALIGNMENT_STARS[j].dec, ALIGNMENT_STARS[k].ra, ALIGNMENT_STARS[k].dec);
                    // Aligned RA/DEC Angular Distance
                    basic_ScopeToEquatorial(ALIGNMENT_STARS[k].az, ALIGNMENT_STARS[k].az, ALIGNMENT_STARS[k].time, &ra2, &dec2);
                    double dist2 = basic_AngDist(ra1, dec1, ra2, dec2);

                    dist += abs(dist1 - dist2);
                }
            }
            dist = dist / n;
            if (dist < bestDist)
            {
                bestZ3 = Z3_ERR;
                bestDist = dist;
            }
        }
    }
    Z3_ERR = bestZ3;
}

void basic_CalcBestZ12()
{
    // handles searching for Z1/2 up to +/- 1 degree
    basic_BestZ12(3, DEG_TO_RAD, ARCMIN_TO_RAD);                                                   // 10 iterations
}

void basic_CalcBestZ3()
{
    // handles searching for Z3 up to +/- 10 degrees
    basic_BestZ3(3, -10.0, 10.0, 2.0);                                                  // 10 iterations
    basic_BestZ3(3, (Z3_ERR * RAD_TO_DEG) - 2.0, (Z3_ERR * RAD_TO_DEG) + 2.0, 0.5);     // 8 iterations
    basic_BestZ3(3, (Z3_ERR * RAD_TO_DEG) - 0.5, (Z3_ERR * RAD_TO_DEG) + 0.5, 0.062);   // 16 iterations
}

void basic_BestZ123()
{
    if (BASIC_ALIGN_READY && CORRECT_MOUNT_ERRS)
    {
        basic_CalcBestZ3();
        basic_CalcBestZ12();
    }
}

void invertMatrix(float m[3][3], float res[3][3])
{
    float idet;
    idet = 1 / ((m[0][0] * m[1][1] * m[2][2]) + (m[0][1] * m[1][2] * m[2][0]) + (m[0][2] * m[1][0] * m[2][1]) - (m[0][2] * m[1][1] * m[2][0]) - (m[0][1] * m[1][0] * m[2][2]) - (m[0][0] * m[1][2] * m[2][1]));

    res[0][0] = ((m[1][1] * m[2][2]) - (m[2][1] * m[1][2])) * idet;
    res[0][1] = ((m[2][1] * m[0][2]) - (m[0][1] * m[2][2])) * idet;
    res[0][2] = ((m[0][1] * m[1][2]) - (m[1][1] * m[0][2])) * idet;

    res[1][0] = ((m[1][2] * m[2][0]) - (m[2][2] * m[1][0])) * idet;
    res[1][1] = ((m[2][2] * m[0][0]) - (m[0][2] * m[2][0])) * idet;
    res[1][2] = ((m[0][2] * m[1][0]) - (m[1][2] * m[0][0])) * idet;

    res[2][0] = ((m[1][0] * m[2][1]) - (m[2][0] * m[1][1])) * idet;
    res[2][1] = ((m[2][0] * m[0][1]) - (m[0][0] * m[2][1])) * idet;
    res[2][2] = ((m[0][0] * m[1][1]) - (m[1][0] * m[0][1])) * idet;
}

void matrixProduct(float m1[3][3], float m2[3][3], float res[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            res[i][j] = 0.0;
            for (int k = 0; k < 3; k++) //multiplying row by column
                res[i][j] += m1[i][k] * m2[k][j];
        }
}

void setEVC(float ra, float dec, float t, float *EVC)
{
    EVC[0] = cos(dec) * cos(ra - SIDEREAL_FRACTION * (t - INITIAL_TIME));
    EVC[1] = cos(dec) * sin(ra - SIDEREAL_FRACTION * (t - INITIAL_TIME));
    EVC[2] = sin(dec);
}

void setHVC(float az, float alt, float *HVC)
{
    HVC[0] = cos(alt) * cos(az);
    HVC[1] = cos(alt) * sin(az);
    HVC[2] = sin(alt);
}

void setRef_1(float ra, float dec, float t, float az, float alt)
{
    setEVC(ra, dec, t, EVC1);
    setHVC(az, alt, HVC1);
    // printMatrix(EVC1, 3, 1, "EVC1 Matrix");
    // printMatrix(HVC1, 3, 1, "HVC1 Matrix");
    IS_SET_R1 = true;
    IS_SET_R3 = false;

    if (IS_SET_R1 && IS_SET_R2 && IS_SET_R3)
        setTransformMatrix();
}

void setRef_2(float ra, float dec, float t, float az, float alt)
{
    setEVC(ra, dec, t, EVC2);
    setHVC(az, alt, HVC2);
    // printMatrix(EVC2, 3, 1, "EVC2 Matrix");
    // printMatrix(HVC2, 3, 1, "HVC2 Matrix");
    IS_SET_R2 = true;
    IS_SET_R3 = false;

    if (IS_SET_R1 && IS_SET_R2 && IS_SET_R3)
        setTransformMatrix();
}

void setRef_3(float ra, float dec, float t, float az, float alt)
{
    setEVC(ra, dec, t, EVC3);
    setHVC(az, alt, HVC3);
    // printMatrix(EVC3, 3, 1, "EVC3 Matrix");
    // printMatrix(HVC3, 3, 1, "HVC3 Matrix");
    IS_SET_R3 = true;

    if (IS_SET_R1 && IS_SET_R2 && IS_SET_R3)
        setTransformMatrix();
}

bool isConfigured()
{
    return (IS_SET_R1 && IS_SET_R2 && IS_SET_R3);
}

void autoRef_3()
{
    float sqrt1, sqrt2;

    if (IS_SET_R1 && IS_SET_R2)
    {
        sqrt1 = (1 / (sqrt(sq(((HVC1[1] * HVC2[2]) - (HVC1[2] * HVC2[1]))) +
                           sq(((HVC1[2] * HVC2[0]) - (HVC1[0] * HVC2[2]))) +
                           sq(((HVC1[0] * HVC2[1]) - (HVC1[1] * HVC2[0]))))));
        HVC3[0] = sqrt1 * ((HVC1[1] * HVC2[2]) - (HVC1[2] * HVC2[1]));
        HVC3[1] = sqrt1 * ((HVC1[2] * HVC2[0]) - (HVC1[0] * HVC2[2]));
        HVC3[2] = sqrt1 * ((HVC1[0] * HVC2[1]) - (HVC1[1] * HVC2[0]));

        sqrt2 = (1 / (sqrt(sq(((EVC1[1] * EVC2[2]) - (EVC1[2] * EVC2[1]))) +
                           sq(((EVC1[2] * EVC2[0]) - (EVC1[0] * EVC2[2]))) +
                           sq(((EVC1[0] * EVC2[1]) - (EVC1[1] * EVC2[0]))))));
        EVC3[0] = sqrt2 * ((EVC1[1] * EVC2[2]) - (EVC1[2] * EVC2[1]));
        EVC3[1] = sqrt2 * ((EVC1[2] * EVC2[0]) - (EVC1[0] * EVC2[2]));
        EVC3[2] = sqrt2 * ((EVC1[0] * EVC2[1]) - (EVC1[1] * EVC2[0]));
        IS_SET_R3 = true;

        if (IS_SET_R1 && IS_SET_R2 && IS_SET_R3)
            setTransformMatrix();
    }
}

void setTransformMatrix()
{
    float subT1[3][3], subT2[3][3], aux[3][3];

    subT1[0][0] = HVC1[0];
    subT1[0][1] = HVC2[0];
    subT1[0][2] = HVC3[0];
    subT1[1][0] = HVC1[1];
    subT1[1][1] = HVC2[1];
    subT1[1][2] = HVC3[1];
    subT1[2][0] = HVC1[2];
    subT1[2][1] = HVC2[2];
    subT1[2][2] = HVC3[2];

    subT2[0][0] = EVC1[0];
    subT2[0][1] = EVC2[0];
    subT2[0][2] = EVC3[0];
    subT2[1][0] = EVC1[1];
    subT2[1][1] = EVC2[1];
    subT2[1][2] = EVC3[1];
    subT2[2][0] = EVC1[2];
    subT2[2][1] = EVC2[2];
    subT2[2][2] = EVC3[2];

    // printMatrix(*subT1, 3, 3, "SubT1 Matrix");
    // printMatrix(*subT2, 3, 3, "SubT2 Matrix");
    invertMatrix(subT2, aux);
    // printMatrix(*aux, 3, 3, "Aux Matrix");
    matrixProduct(subT1, aux, TRANSFORM_MATRIX);
    // printMatrix(*TRANSFORM_MATRIX, 3, 3, "TRANSFORM Matrix");
    invertMatrix(TRANSFORM_MATRIX, INV_TRANSFORM_MATRIX);
    // printMatrix(*INV_TRANSFORM_MATRIX, 3, 3, "INV_TRANSFORM Matrix");
}

void getHCoords(float ra, float dec, float t, double *az, double *alt)
{
    float HVC[3];
    float EVC[3];
    setEVC(ra, dec, t, EVC);
    // printMatrix(EVC, 3, 1, "EVC Matrix");

    if (!IS_SET_R3)
        autoRef_3();

    for (int i = 0; i < 3; i++)
        HVC[i] = 0.0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            HVC[i] += TRANSFORM_MATRIX[i][j] * EVC[j];

    // printMatrix(HVC, 3, 3, "HVC Matrix");

    //(*az) = reverseRev(validRev(atan2(HVC[1], HVC[0])));
    (*az) = validRev(atan2(HVC[1], HVC[0]));
    // Check for valid alt between -90 and 90
    double c = sqrt(sq(HVC[0]) + sq(HVC[1]));
    if (c == 0 && HVC[2] > 0)
        (*alt) = QRT_REV;
    else if (c == 0 && HVC[2] < 0)
        (*alt) = -QRT_REV;
    else if (c != 0)
        (*alt) = atan(HVC[2] / c);
    else
        (*alt) = 0;
}

void getECoords(float az, float alt, float t, double *ra, double *dec)
{
    float HVC[3];
    float EVC[3];
    //az = reverseRev(az);
    setHVC(az, alt, HVC);
    // printMatrix(HVC, 3, 3, "HVC Matrix");

    if (!IS_SET_R3)
        autoRef_3();

    for (int i = 0; i < 3; i++)
        EVC[i] = 0.0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            EVC[i] += INV_TRANSFORM_MATRIX[i][j] * HVC[j];

    // printMatrix(EVC, 3, 1, "EVC Matrix");

    (*ra) = validRev(atan2(EVC[1], EVC[0]) + (SIDEREAL_FRACTION * (t - INITIAL_TIME)));
    // Check for valid dec between -90 and 90
    double c = sqrt(sq(EVC[0]) + sq(EVC[1]));
    if (c == 0 && EVC[2] > 0)
        (*dec) = QRT_REV;
    else if (c == 0 && EVC[2] < 0)
        (*dec) = -QRT_REV;
    else if (c != 0)
        (*dec) = atan(EVC[2] / c);
    else
        (*dec) = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Calculate Planet Positions

void getPlanetPosition(int object_number, Object &object)
{
    float T = (JDN - JD2000) / JDCENTURY;

    float semiMajorAxis = OBJECT_DATA[object_number][0] + (T * OBJECT_DATA[object_number][1]); // offset + T * delta
    float eccentricity = OBJECT_DATA[object_number][2] + (T * OBJECT_DATA[object_number][3]);
    float inclination = OBJECT_DATA[object_number][4] + (T * OBJECT_DATA[object_number][5]);
    float meanLongitude = OBJECT_DATA[object_number][6] + (T * OBJECT_DATA[object_number][7]);
    float longitudePerihelion = OBJECT_DATA[object_number][8] + (T * OBJECT_DATA[object_number][9]);
    float longitudeAscendingNode = OBJECT_DATA[object_number][10] + (T * OBJECT_DATA[object_number][11]);
    float meanAnomaly = meanLongitude - longitudePerihelion;
    float argumentPerihelion = longitudePerihelion - longitudeAscendingNode;

    inclination = calc_format_angle_deg(inclination);
    meanLongitude = calc_format_angle_deg(meanLongitude);
    longitudePerihelion = calc_format_angle_deg(longitudePerihelion);
    longitudeAscendingNode = calc_format_angle_deg(longitudeAscendingNode);
    meanAnomaly = calc_format_angle_deg(meanAnomaly);
    argumentPerihelion = calc_format_angle_deg(argumentPerihelion);

    //---------------------------------
    float eccentricAnomaly = calc_eccentricAnomaly(meanAnomaly, eccentricity);
    eccentricAnomaly = calc_format_angle_deg(eccentricAnomaly);
//---------------------------------
//to orbital Coordinates:
#ifdef SERIAL_DEBUG
    Serial.println(F("----------------------------------------------------"));
    Serial.println("Object:" + object_name[object_number]);
    Serial.println("T:" + String(T, DEC));
    Serial.println("ST:" + String(LST, DEC));
    Serial.println("semiMajorAxis:" + String(semiMajorAxis, DEC));
    Serial.println("eccentricity:" + String(eccentricity, DEC));
    Serial.println("inclination:" + String(inclination, DEC));
    Serial.println("meanLongitude:" + String(meanLongitude, DEC));
    Serial.println("longitudePerihelion:" + String(longitudePerihelion, DEC));
    Serial.println("longitudeAscendingNode:" + String(longitudeAscendingNode, DEC));
    Serial.println("meanAnomaly:" + String(meanAnomaly, DEC));
    Serial.println("argumentPerihelion:" + String(argumentPerihelion, DEC));
    Serial.println("eccentricAnomaly:" + String(eccentricAnomaly, DEC));
    Serial.println(F("orbital coordinates:"));
#endif
    calc_orbital_coordinates(semiMajorAxis, eccentricity, eccentricAnomaly);
    //---------------------------------
    //to heliocentric ecliptic coordinates:
    rot_z(argumentPerihelion);
    rot_x(inclination);
    rot_z(longitudeAscendingNode);
    //---------------------------------
    if (object_number == 2)
    { //object earth

        x_earth = x_coord;
        y_earth = y_coord;
        z_earth = z_coord;
        //---------------------------------
        //calc the sun position from earth:
        calc_vector_subtract(x_earth, 0, y_earth, 0, z_earth, 0); // earth - sun coordinates
        calc_vector(x_coord, y_coord, z_coord, "");
#ifdef SERIAL_DEBUG
        Serial.println(F("geocentric equatorial results of sun:"));
        Serial.println(F("geocentric ecliptic results of sun:"));
#endif
        rot_x(ECLIPTIC_ANGLE); //rotate x > earth ecliptic angle
        calc_vector(x_coord, y_coord, z_coord, "");
        dist_earth_to_sun = dist_earth_to_object;
        calc_azimuthal_equatorial_position(PLANET_LON, PLANET_LAT, OBSERVATION_LATTITUDE, LST);
    }
    //---------------------------------
    if (object_number != 2)
    { //all other objects

        calc_vector_subtract(x_earth, x_coord, y_earth, y_coord, z_earth, z_coord); // earth - object coordinates
        calc_vector(x_coord, y_coord, z_coord, "");
#ifdef SERIAL_DEBUG
        Serial.println(F("geocentric ecliptic results of object:"));
        Serial.println(F("geocentric equatorial results of object:"));
#endif
        rot_x(ECLIPTIC_ANGLE); //rotate x > earth ecliptic angle
        calc_vector(x_coord, y_coord, z_coord, "");
        calc_azimuthal_equatorial_position(PLANET_LON, PLANET_LAT, OBSERVATION_LATTITUDE, LST);
        calc_magnitude(object_number, dist_earth_to_object);
    }

    object.name = object_name[object_number];
    object.description = String("Orbit ") + OBJECT_DATA[object_number][13] + String(" yrs");
    object.ra = PLANET_RA;
    object.dec = PLANET_DECL;

    object.constellation = "";
    object.type = "";
    object.mag = PLANET_MAG;
    object.size = PLANET_SIZE + String("'");

    double delta_ra, delta_dec;
    if (CORRECT_PRECESSION_ETC)
    {
        calcProperMotionPrecessionNutationAberration(object.ra, object.dec, PROPER_MOTION_RA, PROPER_MOTION_DEC, &delta_ra, &delta_dec);
        object.ra += delta_ra;
        object.dec += delta_dec;
    }

    objectAltAz();
}

float calc_format_angle_deg(float deg)
{ //0-360 degrees

    if (deg >= 360 || deg < 0)
    {
        if (deg < 0)
        {
            while (deg < 0)
            {
                deg += 360;
            }
        }
        long x = (long)deg;
        float comma = deg - x;
        long y = x % 360; //modulo 360
        return comma += y;
    }
    return deg;
}

float calc_eccentricAnomaly(float meanAnomaly, float eccentricity)
{

    meanAnomaly *= DEG_TO_RAD;

    int iterations = 0;
    float eccentricAnomaly = meanAnomaly + (eccentricity * sin(meanAnomaly));
    float deltaEccentricAnomaly = 1;

    while (fabs(deltaEccentricAnomaly) > 0.000001)
    { // 0.0000001
        deltaEccentricAnomaly = (meanAnomaly - eccentricAnomaly + (eccentricity * sin(eccentricAnomaly))) / (1 - eccentricity * cos(eccentricAnomaly));
        eccentricAnomaly += deltaEccentricAnomaly;
        iterations++;
        if (iterations > 20)
        {
#ifdef SERIAL_DEBUG
            Serial.println(F("calc_eccentricAnomaly() Error: Max Iterations Reached!!!!!"));
#endif
            eccentricAnomaly = 0;
            break;
        }
    }
#ifdef SERIAL_DEBUG
    Serial.println(String(eccentricAnomaly, DEC));
    Serial.println(String(deltaEccentricAnomaly, DEC));
    Serial.println(String(eccentricAnomaly, DEC));
#endif
    eccentricAnomaly *= RAD_TO_DEG;
    return eccentricAnomaly;
}

void calc_orbital_coordinates(float semiMajorAxis, float eccentricity, float eccentricAnomaly)
{

    eccentricAnomaly *= DEG_TO_RAD;
    float true_Anomaly = 2 * atan(sqrt((1 + eccentricity) / (1 - eccentricity)) * tan(eccentricAnomaly / 2));
    true_Anomaly *= RAD_TO_DEG;
    true_Anomaly = calc_format_angle_deg(true_Anomaly);

    float radius = semiMajorAxis * (1 - (eccentricity * cos(eccentricAnomaly)));
    dist_object_to_sun = radius;

#ifdef SERIAL_DEBUG
    Serial.println("true_Anomaly:" + String(true_Anomaly, DEC));
    Serial.println("radius:" + String(radius, DEC));
#endif

    calc_vector(0, true_Anomaly, radius, "to_rectangular"); // x = beta / y = true_Anomaly / z = radius
}

void calc_vector(float x, float y, float z, String mode)
{

    // convert to rectangular coordinates:
    if (mode == F("to_rectangular"))
    {

        x *= DEG_TO_RAD;
        y *= DEG_TO_RAD;

        x_coord = z * cos(x) * cos(y);
        y_coord = z * cos(x) * sin(y);
        z_coord = z * sin(x);

        x = x_coord;
        y = y_coord;
        z = z_coord;
    }
    // convert to spherical coordinates:
    //get Longitude:
    float lon = atan2(y, x);
    lon *= RAD_TO_DEG;
    lon = calc_format_angle_deg(lon);
    PLANET_LON = lon;
    format_angle(lon, F("degrees"));

    //get Latitude:
    float lat = atan2(z, (sqrt(x * x + y * y)));
    lat *= RAD_TO_DEG;
    lat = calc_format_angle_deg(lat);
    PLANET_LAT = lat;
    format_angle(lat, F("degrees-latitude"));

    //getDistance:
    dist_earth_to_object = sqrt(x * x + y * y + z * z);
#ifdef SERIAL_DEBUG
    Serial.println("x_coord:" + String(x, DEC));
    Serial.println("y_coord:" + String(y, DEC));
    Serial.println("z_coord:" + String(z, DEC));
    Serial.println("LON:" + String(lon, DEC));
    Serial.println("LAT:" + String(lat, DEC));
    Serial.println("DIS:" + String(dist_earth_to_object, DEC));
#endif
}

void format_angle(float angle, String format)
{

    int d = 0;
    int m = 0;
    int s = 0;
    float rest = 0;
    String sign = "";

    if (format == F("degrees") || format == F("degrees-latitude"))
    {

        rest = calc_format_angle_deg(angle);

        if (format == F("degrees-latitude") && rest > 90)
        {
            rest -= 360;
        }
        if (rest >= 0)
        {
            sign = "+";
        }
        else
        {
            sign = "-";
        }

        rest = fabs(rest);
        d = (int)(rest);
        rest = (rest - (float)d) * 60;
        m = (int)(rest);
        rest = (rest - (float)m) * 60;
        s = (int)(rest);
#ifdef SERIAL_DEBUG
        Serial.println(sign + String(d) + ":" + String(m) + ":" + String(s));
#endif
    }
}

void rot_x(float alpha)
{

    alpha *= DEG_TO_RAD;
    float y = cos(alpha) * y_coord - sin(alpha) * z_coord;
    float z = sin(alpha) * y_coord + cos(alpha) * z_coord;
    y_coord = y;
    z_coord = z;
}

void rot_y(float alpha)
{

    alpha *= DEG_TO_RAD;
    float x = cos(alpha) * x_coord + sin(alpha) * z_coord;
    float z = sin(alpha) * x_coord + cos(alpha) * z_coord;
    x_coord = x;
    z_coord = z;
}

void rot_z(float alpha)
{

    alpha *= DEG_TO_RAD;
    float x = cos(alpha) * x_coord - sin(alpha) * y_coord;
    float y = sin(alpha) * x_coord + cos(alpha) * y_coord;
    x_coord = x;
    y_coord = y;
}

void calc_vector_subtract(float xe, float xo, float ye, float yo, float ze, float zo)
{

    x_coord = xo - xe;
    y_coord = yo - ye;
    z_coord = zo - ze;
}

void calc_azimuthal_equatorial_position(float ra, float dec, float lat, float sidereal_time)
{
    float ha = (sidereal_time * 15) - ra; //ha = hours of angle  (-180 to 180 RAD_TO_DEG)
    if (ha < -180)
        ha += 360;
    if (ha > 180)
        ha -= 360;
    if (dec < -90)
        dec += 360;
    if (dec > 90)
        dec -= 360;

    ha *= DEG_TO_RAD;
    dec *= DEG_TO_RAD;
    lat *= DEG_TO_RAD;

    float x = cos(ha) * cos(dec);
    float y = sin(ha) * cos(dec);
    float z = sin(dec);

    //rotate y
    float x_hor = x * sin(lat) - z * cos(lat); //horizon position
    float y_hor = y;
    float z_hor = x * cos(lat) + z * sin(lat);

    PLANET_RA = ra * DEG_TO_RAD;
    PLANET_DECL = dec;
    PLANET_AZ = atan2(y_hor, x_hor) + PI;
    PLANET_ALT = atan2(z_hor, sqrt(x_hor * x_hor + y_hor * y_hor));
    PLANET_AZ *= RAD_TO_DEG;  //0=north, 90=east, 180=south, 270=west
    PLANET_ALT *= RAD_TO_DEG; //0=horizon, 90=zenith, -90=down
#ifdef SERIAL_DEBUG
    Serial.println("RA:" + String(PLANET_RA, DEC));
    Serial.println("DECL:" + String(PLANET_DECL, DEC));
    Serial.println("azimuth:" + String(PLANET_AZ, DEC));
    Serial.println("altitude:" + String(PLANET_ALT, DEC));
    Serial.println("distance:" + String(dist_earth_to_object, DEC));
#endif
}

void calc_magnitude(int object_number, float R)
{ // R = distance earth to object in AE
    float apparent_diameter = OBJECT_DATA[object_number][12] / R;
    float r = dist_object_to_sun; //r = distance in AE
    float s = dist_earth_to_sun;  //s = distance in AE

    float elon = acos((s * s + R * R - r * r) / (2 * s * R));
    elon *= RAD_TO_DEG;

    float phase_angle = acos((r * r + R * R - s * s) / (2 * r * R));
    //float phase = (1 + cos(phase_angle)) / 2;
    phase_angle *= RAD_TO_DEG;

    float magnitude = 0;
    float ring_magn = -0.74;
    if (object_number == 0)
        magnitude = -0.36 + 5 * log10(r * R) + 0.027 * phase_angle + 2.2e-13 * pow(phase_angle, 6.0); //Mercury
    if (object_number == 1)
        magnitude = -4.34 + 5 * log10(r * R) + 0.013 * phase_angle + 4.2e-7 * pow(phase_angle, 3.0); //Venus
    if (object_number == 3)
        magnitude = -1.51 + 5 * log10(r * R) + 0.016 * phase_angle; //Mars
    if (object_number == 4)
        magnitude = -9.25 + 5 * log10(r * R) + 0.014 * phase_angle; //Jupiter
    if (object_number == 5)
        magnitude = -9.00 + 5 * log10(r * R) + 0.044 * phase_angle + ring_magn; //Saturn
    if (object_number == 6)
        magnitude = -7.15 + 5 * log10(r * R) + 0.001 * phase_angle; //Uranus
    if (object_number == 7)
        magnitude = -6.90 + 5 * log10(r * R) + 0.001 * phase_angle; //Neptune
    if (object_number == 8)
        magnitude = 0.82 + 5 * log10(r * R) + 0.001 * phase_angle; //Pluto
#ifdef SERIAL_DEBUG
    Serial.print("apparent diameter:" + String(apparent_diameter, 2)); //Arc seconds
    if (object_number == 5)
        Serial.print(" + ring = " + String(apparent_diameter + 20, 2)); //Arc seconds
    Serial.println();
    Serial.println("elongation:" + String(elon, 2));
    Serial.println("phase angle:" + String(phase_angle, 2));
    float phase = (1 + cos(phase_angle)) / 2;
    Serial.println("phase:" + String(phase, 2));
    Serial.println("magnitude:" + String(magnitude, 2));
#endif
    PLANET_SIZE = apparent_diameter;
    PLANET_MAG = magnitude;
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
    if (CURRENT_SCREEN == 3 && (millis() - UPDATE_TIME) > 5000)
    {
        // Main Screen
        NOW = rtc.GetDateTime();

        tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
        tft.setTextSize(2);
        if (OLD_MIN != NOW.Minute())
        {
            OLD_MIN = NOW.Minute();
            tft.fillRect(0, 0, 80, 20, BLACK);
            tft.setCursor(10, 10);
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
        tft.fillRect(100, 0, 140, 20, BLACK);
        tft.setCursor(100, 10);
        tft.print("LST ");
        if ((int)LST < 10)
        {
            tft.print("0");
        }
        tft.print((int)LST);
        tft.print(":");
        if ((LST - (int)LST) * 60 < 10)
        {
            tft.print("0");
        }
        tft.print((LST - (int)LST) * 60, 0);

        // Update Telescope RA/DEC accoriding to new time
        tft.fillRect(70, 110, 170, 20, BLACK);
        tft.setTextColor(L_TEXT);
        tft.setTextSize(1);

        // tft.setCursor(10, 110);
        // tft.print("RA: ");
        tft.setCursor(70, 110);
        tft.print(CURR_RA);
        // tft.setCursor(10, 120);
        // tft.print("DEC: ");
        tft.setCursor(70, 120);
        tft.print(CURR_DEC);

        if ((CURRENT_OBJECT.name != ""))
        {
            objectAltAz();
            tft.fillRect(70, 205, 170, 40, BLACK);
            tft.setTextColor(L_TEXT);
            tft.setTextSize(2);

            // tft.setCursor(10, 205);
            // tft.print("ALT: ");
            tft.setCursor(70, 205);
            tft.print(rad2dms(CURRENT_OBJECT.alt, true, false));
            // tft.setCursor(10, 225);
            // tft.print("AZ: ");
            tft.setCursor(70, 225);
            tft.print(rad2dms(CURRENT_OBJECT.az, true, true));
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
        tft.fillRect(0, 95, 240, 100, BLACK);
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
            tft.print("Verifying Coordinates");
            tft.setCursor(10, 220);
            tft.print("Iteration #");
            tft.print(GPS_ITERATIONS);
            GPS_ITERATIONS += 1;
            smartDelay(1000);
        }

        if ((GPS_ITERATIONS > 2) && (gps.location.lat() != 0))
        {
            OBSERVATION_LONGITUDE = gps.location.lng();
            OBSERVATION_LATTITUDE = gps.location.lat();
            OBSERVATION_ALTITUDE = gps.altitude.meters();
            OBSERVATION_LONGITUDE_RADS = OBSERVATION_LONGITUDE * DEG_TO_RAD;
            OBSERVATION_LATTITUDE_RADS = OBSERVATION_LATTITUDE * DEG_TO_RAD;
#ifdef SERIAL_DEBUG
            Serial.print("OBSERVATION LATTITUDE: ");
            Serial.print(OBSERVATION_LATTITUDE);
            Serial.println("");
#endif
            CURRENT_SCREEN = 1;

            int ora, date_delay = 0;
            int time_delay = round(gps.location.lng() * 4 / 60); //rough calculation of the timezone delay

            // convert to epoch
            setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
#ifdef SERIAL_DEBUG
            Serial.print("EPOCH: ");
            Serial.print(now());
            Serial.println("");
#endif

            if (isSummerTime())
            {
                //If in summer time sum 1h and put SUMMER_TIME flag as 1
                time_delay += 1;
                SUMMER_TIME = 1;
            }

            //update the value of the variable ora
            ora = gps.time.hour() + time_delay;

            //to update the real time
            if (ora >= 24)
            {
                ora -= 24;
                date_delay = 1;
            }

            setTime(ora, gps.time.minute(), gps.time.second(), gps.date.day() + date_delay, gps.date.month(), gps.date.year());
            RtcDateTime gpsTime = RtcDateTime(gps.date.year(), gps.date.month(), gps.date.day() + date_delay, ora, gps.time.minute(), gps.time.second());
            rtc.SetDateTime(gpsTime);

            drawClockScreen();
        }
        UPDATE_TIME = millis();
    }
    else if (CURRENT_SCREEN == 7 && (millis() - UPDATE_TIME) > 5000)
    {
        uint8_t starNum = ALIGN_STEP - 1;
        tft.fillRect(70, 170, 170, 50, BLACK);
        tft.setTextColor(L_TEXT);
        // tft.setCursor(10, 170);
        // tft.print("ALT: ");
        tft.setCursor(70, 170);
        tft.print(rad2dms(ALIGNMENT_STARS[starNum].alt, true, false));
        // tft.setCursor(10, 190);
        // tft.print("AZ: ");
        tft.setCursor(70, 190);
        tft.print(rad2dms(ALIGNMENT_STARS[starNum].az, true, true));
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
    tft.setTextColor(D_TEXT);
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

    // tft.setCursor(10, 160);
    // tft.print("Initialising BlueTooth -> ");
    // if (SerialBT.available() > 0)
    // {
    //     tft.setTextColor(L_TEXT);
    //     tft.print("OK");
    //     tft.setTextColor(D_TEXT);
    // }
    // else
    // {
    //     tft.setTextColor(L_TEXT);
    //     tft.print("FAIL");
    //     tft.setTextColor(D_TEXT);
    // }
    // delay(500);

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

    drawButton(40, 270, 160, 40, "SKIP", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawClockScreen()
{
    CURRENT_SCREEN = 1;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" Date/Time");

    NOW = rtc.GetDateTime();
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

void drawAlignModeScreen()
{
    CURRENT_SCREEN = 10;

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" Align Mode");

    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 40);
    tft.setTextSize(2);
    tft.print("Select Master");
    tft.setCursor(10, 60);
    tft.print("Alignment Mode");
    drawButton(20, 110, 200, 40, "Taki Basic", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 170, 200, 40, "Taki Advanced", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawAlignCorrectionsScreen()
{
    CURRENT_SCREEN = 11;

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" Corrections");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);

    if (CORRECT_REFRACTION)
        drawButton(20, 50, 200, 40, "Refraction", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    else
        drawButton(20, 50, 200, 40, "Refraction", 0, BTN_L_BORDER, L_TEXT, 2);
    if (CORRECT_PRECESSION_ETC)
        drawButton(20, 110, 200, 40, "Orbital Anoms.", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    else
        drawButton(20, 110, 200, 40, "Orbital Anoms.", 0, BTN_L_BORDER, L_TEXT, 2);
    if (CORRECT_MOUNT_ERRS)
        drawButton(20, 170, 200, 40, "Mount Errors", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    else
        drawButton(20, 170, 200, 40, "Mount Errors", 0, BTN_L_BORDER, L_TEXT, 2);

    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 230);
    tft.setTextSize(1);
    tft.print("Z1 Error: ");
    tft.print(rad2dms(Z1_ERR, true, false));
    tft.setCursor(10, 240);
    tft.print("Z2 Error: ");
    tft.print(rad2dms(Z2_ERR, true, false));
    tft.setCursor(10, 250);
    tft.print("Z3 Error: ");
    tft.print(rad2dms(Z3_ERR, true, false));

    if (!BASIC_ALIGN_READY)
        drawButton(20, 270, 200, 40, "Begin Align", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawSelectAlignment()
{
    CURRENT_SCREEN = 2;
    ALIGN_STEP = 1;

    basic_Init();
    zeroArrays(CMWS_ALIGN);

    if (CORRECT_PRECESSION_ETC)
        calcCelestialParams();

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" Alignment");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);

    drawButton(20, 50, 200, 40, "2-Star Auto", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 110, 200, 40, "3-Star Auto", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 170, 200, 40, "2-Star Manual", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(20, 230, 200, 40, "3-Star Manual", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    //drawButton(20, 270, 200, 40, "Skip Align", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawMainScreen()
{
    CURRENT_SCREEN = 3;
    IS_IN_OPERATION = true;
    UPDATE_TIME = millis();
    W_DATE_TIME[0] = 0;

    NOW = rtc.GetDateTime();
    calculateLST();
    INITIAL_TIME = LST_RADS;

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
    if ((int)LST < 10)
    {
        tft.print("0");
    }
    tft.print((int)LST);
    tft.print(":");
    if ((LST - (int)LST) * 60 < 10)
    {
        tft.print("0");
    }
    tft.print((LST - (int)LST) * 60, 0);

    tft.setTextSize(1);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 30);
    tft.print("LAT: ");
    tft.print(OBSERVATION_LATTITUDE, 2);
    tft.print(" LNG: ");
    tft.print(OBSERVATION_LONGITUDE, 2);
    tft.print(" ALT: ");
    tft.print(OBSERVATION_ALTITUDE, 0);
    tft.print("m");

    tft.setTextSize(2);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setCursor(10, 45);
    tft.println("TELESCOPE");

    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 70);
    tft.print("ALT: ");
    tft.setCursor(70, 70);
    tft.print(CURR_ALT);
    tft.setCursor(10, 90);
    tft.print("AZ: ");
    tft.setCursor(70, 90);
    tft.print(CURR_AZ);

    tft.setTextSize(1);
    tft.setCursor(10, 110);
    tft.print("RA: ");
    tft.setCursor(70, 110);
    tft.print(CURR_RA);
    tft.setCursor(10, 120);
    tft.print("DEC: ");
    tft.setCursor(70, 120);
    tft.print(CURR_DEC);

    tft.setTextSize(2);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setCursor(10, 135);
    tft.println("TARGET");

    if (CURRENT_OBJECT.name != "")
    {
        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 155);
        tft.print(CURRENT_OBJECT.name);
        tft.setTextSize(1);
        tft.print(" - " + CURRENT_OBJECT.description);

        tft.setTextSize(2);
        tft.setCursor(10, 175);
        if (CURRENT_OBJECT.alt < 0)
        {
            tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
            tft.println("OBJECT NOT VISIBLE!");
        }
        else
        {
            tft.setTextSize(1);
            tft.setTextColor(L_TEXT);
            tft.print(CURRENT_OBJECT.constellation);
            tft.setCursor(10, 185);
            tft.print(CURRENT_OBJECT.type);
            tft.setCursor(10, 195);
            tft.print("Mag. ");
            tft.print(CURRENT_OBJECT.mag);
            tft.print(" size ");
            tft.print(CURRENT_OBJECT.size);
        }

        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 205);
        tft.print("ALT: ");
        tft.setCursor(70, 205);
        tft.print(rad2dms(CURRENT_OBJECT.alt, true, false));
        tft.setCursor(10, 225);
        tft.print("AZ: ");
        tft.setCursor(70, 225);
        tft.print(rad2dms(CURRENT_OBJECT.az, true, true));

        tft.setTextSize(1);
        tft.setCursor(10, 245);
        tft.print("RA: ");
        tft.setCursor(70, 245);
        tft.print(rad2hms(CURRENT_OBJECT.ra, true, true));

        tft.setCursor(10, 255);
        tft.print("DEC: ");
        tft.setCursor(70, 255);
        tft.print(rad2dms(CURRENT_OBJECT.dec, true, false));
    }
    else
    {
        tft.setTextSize(2);
        tft.setTextColor(L_TEXT);
        tft.setCursor(10, 155);
        tft.println("No target selected!");
    }

    drawButton(10, 270, 65, 40, "FIND", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(88, 270, 65, 40, "PLNT", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(165, 270, 65, 40, "MENU", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawLoadScreen()
{
    CURRENT_SCREEN = 4;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" FIND");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);

    // Draw buttons to load CSVs
    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    if (LOAD_SELECTOR == 1)
    {
        drawButton(10, 50, 105, 30, "Messier", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    }
    else
    {
        drawButton(10, 50, 105, 30, "Messier", 0, BTN_L_BORDER, L_TEXT, 2);
    }
    if (LOAD_SELECTOR == 2)
    {
        drawButton(125, 50, 105, 30, "Treasures", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    }
    else
    {
        drawButton(125, 50, 105, 30, "Treasures", 0, BTN_L_BORDER, L_TEXT, 2);
    }

    drawButton(10, 270, 100, 40, "< PREV", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 270, 100, 40, "NEXT >", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawLoadObjects();
}

void drawLoadObjects()
{
    /////// Messier Screen /////////////
    tft.fillRect(0, 90, 240, 170, BLACK);
    if (LOAD_SELECTOR == 1)
    {
        // I'll draw 12 objects per page, thus "(pager*12)" will give me the start of the [index_]
        if (LOADED_STARS != 2)
        {
            loadStarsFromSPIFFS("/messier.csv");
            LOADED_STARS = 2;
        }

        tft.setTextSize(1);
        int kk = MESS_PAGER * 12;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // String M_NAME = Messier_Array[kk].substring(0, Messier_Array[kk].indexOf(','));
                String M_NAME = OBJECTS[kk].substring(0, OBJECTS[kk].indexOf(','));
                if (M_NAME == "")
                {
                    break;
                }
                drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                kk += 1;
            }
        }
    }
    ///////     Treasures Screen /////////////
    else if (LOAD_SELECTOR == 2)
    {
        if (LOADED_STARS != 3)
        {
            loadStarsFromSPIFFS("/treasure.csv");
            LOADED_STARS = 3;
        }

        tft.setTextSize(1);
        int ll = TREAS_PAGER * 12;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                //String M_NAME = Treasure_Array[ll].substring(0, Treasure_Array[ll].indexOf(','));
                String M_NAME = OBJECTS[ll].substring(0, OBJECTS[ll].indexOf(','));
                if (M_NAME == "")
                {
                    break;
                }
                drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, BTN_L_BORDER, 0, BTN_BLK_TEXT, 1);
                ll += 1;
            }
        }
    }
}

void drawOptionsScreen()
{
    CURRENT_SCREEN = 5;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" Options");

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);

    // Toggle BT and WiFi
    tft.setCursor(10, 50);
    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.print("Remote Modes");

    drawButton(10, 70, 65, 40, "BT", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(85, 70, 65, 40, "WiFi", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    // Adjust configurations
    tft.setCursor(10, 130);
    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.print("Configuration");

    drawButton(10, 150, 65, 40, "Align", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(85, 150, 65, 40, "GPS", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(165, 150, 65, 40, "Clock", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    tft.setCursor(10, 210);
    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.print("Corrections");

    drawButton(10, 230, 65, 40, "Corr.", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
}

void drawStarSelectScreen()
{
    CURRENT_SCREEN = 6;
    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" Align");

    drawButton(165, 10, 65, 30, "EXIT", BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);

    drawButton(10, 270, 100, 40, "< PREV", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 270, 100, 40, "NEXT >", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    // First Load in the Alignment OBJECTS Array from SPIFFS storage
    if (LOADED_STARS != 1)
    {
        loadStarsFromSPIFFS("/alignment.csv");
        LOADED_STARS = 1;
    }
    int kk = STARS_PAGER * 12;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            int i1 = OBJECTS[kk].indexOf(',');
            int i2 = OBJECTS[kk].indexOf(',', i1 + 1);
            String S_NAME = OBJECTS[kk].substring(i1 + 1, i2);
            String C_NAME = OBJECTS[kk].substring(0, i1);
            if (S_NAME == "")
            {
                break;
            }
            tft.setTextColor(BTN_BLK_TEXT);
            tft.setTextSize(1);
            tft.fillRect(((j * 75) + 10), ((i * 50) + 45), 71, 45, BTN_L_BORDER);

            if (S_NAME.length() > 10)
            {
                String S_NAME_1 = S_NAME.substring(0, 9);
                String S_NAME_2 = S_NAME.substring(9, S_NAME.length() - 1);
                int l1 = (S_NAME_1.length() / 2) * 6;
                int l2 = (S_NAME_2.length() / 2) * 6;
                tft.setCursor(((j * 75) + (44 - l1)), ((i * 50) + 50));
                tft.print(S_NAME_1);
                tft.setCursor(((j * 75) + (44 - l2)), ((i * 50) + 60));
                tft.print(S_NAME_2);
            }
            else
            {
                int l = (S_NAME.length() / 2) * 6;
                tft.setCursor(((j * 75) + (44 - l)), ((i * 50) + 50));
                tft.print(S_NAME);
            }

            tft.setTextSize(2);
            tft.setCursor(((j * 75) + 29), ((i * 50) + 70));
            tft.print(C_NAME);
            kk += 1;
        }
    }
}

void drawAlignScreen()
{
    CURRENT_SCREEN = 7;
    uint8_t starNum = ALIGN_STEP - 1;

    tft.fillScreen(BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TITLE_TEXT, TITLE_TEXT_BG);
    tft.setTextSize(3);
    tft.print(" Alignment");

    tft.setTextSize(2);
    tft.setTextColor(L_TEXT);
    tft.setCursor(10, 50);
    tft.print("Selected Star");

    tft.setCursor(10, 70);
    tft.print(ALIGNMENT_STARS[starNum].name);

    tft.setTextSize(1);
    tft.setCursor(10, 90);
    tft.print("Constellation: " + ALIGNMENT_STARS[starNum].constellation);
    tft.setCursor(10, 100);
    tft.print("Magnitude: " + (String)ALIGNMENT_STARS[starNum].mag);

    tft.setTextSize(2);
    tft.setCursor(10, 120);
    tft.print("RA: ");
    tft.setCursor(70, 120);
    tft.print(rad2hms(ALIGNMENT_STARS[starNum].ra, true, true));
    tft.setCursor(10, 140);
    tft.print("DEC: ");
    tft.setCursor(70, 140);
    tft.print(rad2dms(ALIGNMENT_STARS[starNum].dec, true, false));

    tft.setCursor(10, 170);
    tft.print("ALT: ");
    tft.setCursor(70, 170);
    tft.print(rad2dms(ALIGNMENT_STARS[starNum].alt, true, false));
    tft.setCursor(10, 190);
    tft.print("AZ: ");
    tft.setCursor(70, 190);
    tft.print(rad2dms(ALIGNMENT_STARS[starNum].az, true, true));

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
        tft.print(" WiFi");
    }

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);

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

    drawButton(165, 10, 65, 30, "BACK", BTN_L_BORDER, 0, BTN_BLK_TEXT, 3);

    drawButton(10, 50, 100, 40, "Mercury", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 50, 100, 40, "Venus", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 110, 100, 40, "Mars", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 110, 100, 40, "Jupiter", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 170, 100, 40, "Saturn", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 170, 100, 40, "Uranus", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    drawButton(10, 230, 100, 40, "Neptune", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
    drawButton(130, 230, 100, 40, "Pluto", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);

    if (dist_earth_to_sun == 0)
        getPlanetPosition(2, CURRENT_OBJECT);
}

void OnScreenMsg(int Msg)
{
    String m1, m2, m3;
    tft.fillRect(40, 110, 160, 100, MSG_BOX_BG);
    tft.drawRect(40, 110, 160, 100, MSG_BOX_TEXT);
    tft.setTextColor(MSG_BOX_TEXT);
    if (Msg == 1)
    {
        m1 = "ERROR!";
        m2 = "Not Visible";
        tft.setCursor(80, 140);
        tft.setTextSize(3);
        tft.println(m1);
        tft.setCursor(60, 160);
        tft.setTextSize(2);
        tft.print(m2);
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

void drawAlignObjects_ali()
{
    if (LOADED_STARS != 1)
    {
        loadStarsFromSPIFFS("/alignment.csv");
        LOADED_STARS = 1;
    }
    // First ensure the old objects are gone
    tft.fillRect(0, 45, 240, 220, BLACK);
    int kk = STARS_PAGER * 12;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            int i1 = OBJECTS[kk].indexOf(',');
            int i2 = OBJECTS[kk].indexOf(',', i1 + 1);
            String S_NAME = OBJECTS[kk].substring(i1 + 1, i2);
            String C_NAME = OBJECTS[kk].substring(0, i1);
            if (S_NAME == "")
            {
                break;
            }
            tft.setTextColor(BTN_BLK_TEXT);
            tft.setTextSize(1);
            tft.fillRect(((j * 75) + 10), ((i * 50) + 45), 71, 45, BTN_L_BORDER);

            if (S_NAME.length() > 10)
            {
                String S_NAME_1 = S_NAME.substring(0, 9);
                String S_NAME_2 = S_NAME.substring(9, S_NAME.length() - 1);
                int l1 = (S_NAME_1.length() / 2) * 6;
                int l2 = (S_NAME_2.length() / 2) * 6;
                tft.setCursor(((j * 75) + (44 - l1)), ((i * 50) + 50));
                tft.print(S_NAME_1);
                tft.setCursor(((j * 75) + (44 - l2)), ((i * 50) + 60));
                tft.print(S_NAME_2);
            }
            else
            {
                int l = (S_NAME.length() / 2) * 6;
                tft.setCursor(((j * 75) + (44 - l)), ((i * 50) + 50));
                tft.print(S_NAME);
            }

            tft.setTextSize(2);
            tft.setCursor(((j * 75) + 29), ((i * 50) + 70));
            tft.print(C_NAME);
            kk += 1;
        }
    }
    // }
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
                OLD_DAY = rtc.GetDateTime();
                delay(150);
                drawAlignModeScreen();
                //drawSelectAlignment();
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
        else if (CURRENT_SCREEN == 10)
        {
            // On drawAlignModeScreen() Screen
            if (lx > 20 && lx < 220 && ly > 110 && ly < 150)
            {
                // BTN "Taki Basic" pressed
                drawButton(20, 110, 200, 40, "Taki Basic", 0, BTN_L_BORDER, L_TEXT, 2);
                ALIGNMENT_METHOD = TAKI_BASIC;
                delay(150);
                drawAlignCorrectionsScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 170 && ly < 210)
            {
                // BTN "Taki Advanced" pressed
                drawButton(20, 170, 200, 40, "Taki Advanced", 0, BTN_L_BORDER, L_TEXT, 2);
                ALIGNMENT_METHOD = TAKI_ADVANCED;
                delay(150);
                drawAlignCorrectionsScreen();
            }
        }
        else if (CURRENT_SCREEN == 11)
        {
            // On drawAlignCorrectionsScreen() Screen
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN <Back pressed// BTN <Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 3);
                delay(150);
                if (BASIC_ALIGN_READY)
                    drawMainScreen();
                else
                    drawAlignModeScreen();
            }
            if (lx > 20 && lx < 220 && ly > 50 && ly < 90)
            {
                // BTN "Refraction" pressed
                CORRECT_REFRACTION = !CORRECT_REFRACTION;
                if (CORRECT_REFRACTION)
                    drawButton(20, 50, 200, 40, "Refraction", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                else
                    drawButton(20, 50, 200, 40, "Refraction", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
            }
            else if (lx > 20 && lx < 220 && ly > 110 && ly < 150)
            {
                // BTN "Orbital Anoms." pressed
                CORRECT_PRECESSION_ETC = !CORRECT_PRECESSION_ETC;
                if (CORRECT_PRECESSION_ETC)
                    drawButton(20, 110, 200, 40, "Orbital Anoms.", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                else
                    drawButton(20, 110, 200, 40, "Orbital Anoms.", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
            }
            else if (lx > 20 && lx < 220 && ly > 170 && ly < 210)
            {
                // BTN "Mount Errors" pressed
                CORRECT_MOUNT_ERRS = !CORRECT_MOUNT_ERRS;
                if (CORRECT_MOUNT_ERRS)
                    drawButton(20, 170, 200, 40, "Mount Errors", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                else
                    drawButton(20, 170, 200, 40, "Mount Errors", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
            }
            else if (lx > 20 && lx < 220 && ly > 270 && ly < 310)
            {
                // BTN "Begin Align" pressed
                if (!BASIC_ALIGN_READY) // Don't let us align if we're just checking errors
                {
                    drawButton(20, 270, 200, 40, "Begin Align", 0, BTN_L_BORDER, L_TEXT, 2);
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
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 3);
                delay(150);
                drawAlignCorrectionsScreen();
            }
            if (lx > 20 && lx < 220 && ly > 50 && ly < 90)
            {
                // BTN "2-Star Auto" pressed
                ALIGN_TYPE = 1;
                NUM_ALIGNMENT_STARS = 2;
                drawButton(20, 50, 200, 40, "2-Star Auto", 0, BTN_L_BORDER, L_TEXT, 2);
                autoSelectAlignmentStars();
                delay(150);
                drawAlignScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 110 && ly < 150)
            {
                // BTN "3-Star Auto" pressed
                ALIGN_TYPE = 1;
                NUM_ALIGNMENT_STARS = 3;
                drawButton(20, 110, 200, 40, "3-Star Auto", 0, BTN_L_BORDER, L_TEXT, 2);
                autoSelectAlignmentStars();
                delay(150);
                drawAlignScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 170 && ly < 210)
            {
                // BTN "2-Star Manual" pressed
                ALIGN_TYPE = 2;
                NUM_ALIGNMENT_STARS = 2;
                drawButton(20, 110, 200, 40, "2-Star Manual", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawStarSelectScreen();
            }
            else if (lx > 20 && lx < 220 && ly > 110 && ly < 150)
            {
                // BTN "3-Star Manual" pressed
                ALIGN_TYPE = 2;
                NUM_ALIGNMENT_STARS = 3;
                drawButton(20, 110, 200, 40, "3-Star Manual", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawStarSelectScreen();
            }
            // else if (lx > 20 && lx < 220 && ly > 270 && ly < 310)
            // {
            //     // BTN "Skip Alignment" pressed
            //     drawButton(20, 270, 200, 40, "Skip Align", 0, BTN_L_BORDER, L_TEXT, 2);
            //     delay(150);
            //     drawMainScreen();
            // }
        }
        else if (CURRENT_SCREEN == 3)
        { // captures touches on drawMainScreen()
            // Find Button - Go to Load screen to search for targets
            if (lx > 10 && lx < 75 && ly > 270 && ly < 310)
            {
                drawButton(10, 270, 65, 40, "FIND", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawLoadScreen();
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
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 3);
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 115 && ly > 50 && ly < 80)
            {
                // BTN Messier pressed
                LOAD_SELECTOR = 1;
                drawButton(10, 50, 105, 30, "Messier", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                drawButton(125, 50, 105, 30, "Treasures", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawLoadObjects();
            }
            if (lx > 125 && lx < 230 && ly > 50 && ly < 80)
            {
                // BTN Treasures pressed
                LOAD_SELECTOR = 2;
                drawButton(10, 50, 105, 30, "Messier", 0, BTN_L_BORDER, L_TEXT, 2);
                drawButton(125, 50, 105, 30, "Treasures", BTN_L_BORDER, 0, BTN_BLK_TEXT, 2);
                delay(150);
                drawLoadObjects();
            }
            //////////////      Messier Screen //////////////
            if (lx > 130 && lx < 230 && ly > 270 && ly < 310)
            {
                // BTN next> pressed  TREAS_PAGER
                LAST_BUTTON = 1;
                drawButton(130, 270, 100, 40, "NEXT >", 0, BTN_L_BORDER, L_TEXT, 2);
                if (LOAD_SELECTOR == 1)
                {
                    MESS_PAGER += 1;
                    if (MESS_PAGER >= 10)
                    {
                        MESS_PAGER = 9;
                    }
                }
                else
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
                    MESS_PAGER -= 1;
                    if (MESS_PAGER < 0)
                    {
                        MESS_PAGER = 0;
                    }
                }
                else
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
            if (LOAD_SELECTOR == 1)
            {
                // I'm in MESSIER selector and need to check which Messier object is pressed
                if (LOADED_STARS != 2)
                {
                    loadStarsFromSPIFFS("/messier.csv");
                    LOADED_STARS = 2;
                }
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (lx > ((j * 75) + 10) && lx < ((j * 75) + 81) && ly > ((i * 40) + 90) && ly < ((i * 40) + 130))
                        {
                            // found button pressed.... now I need to get his ID and link to the ARRAY;
                            LAST_BUTTON = 10;
                            int zz = ((MESS_PAGER * 12) + (i * 3) + j);
                            String M_NAME = OBJECTS[zz].substring(0, OBJECTS[zz].indexOf(','));
                            drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, 0, BTN_L_BORDER, L_TEXT, 2);
                            if (OBJECTS[zz] != "")
                            //if (Messier_Array[zz] != "")
                            {
                                selectObject(zz, 0);
                                MESS_PAGER == 0;
                                delay(150);
                                drawMainScreen();
                            }
                        }
                    }
                }
                ///////     Treasures Screen /////////////
            }
            else if (LOAD_SELECTOR == 2)
            {
                // I'm in TREASURES selector and need to check which Treasure object is pressed
                if (LOADED_STARS != 3)
                {
                    loadStarsFromSPIFFS("/treasure.csv");
                    LOADED_STARS = 3;
                }
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (lx > ((j * 75) + 10) && lx < ((j * 75) + 81) && ly > ((i * 40) + 90) && ly < ((i * 40) + 130))
                        {
                            LAST_BUTTON = 10;
                            // found button pressed.... now I need to get his ID and link to the ARRAY;
                            int zz = ((TREAS_PAGER * 12) + (i * 3) + j);
                            String M_NAME = OBJECTS[zz].substring(0, OBJECTS[zz].indexOf(','));
                            drawButton(((j * 75) + 10), ((i * 40) + 90), 71, 35, M_NAME, 0, BTN_L_BORDER, L_TEXT, 1);
                            if (OBJECTS[zz] != "")
                            //if (Treasure_Array[zz] != "")
                            {
                                selectObject(zz, 1);
                                TREAS_PAGER == 0;
                                delay(150);
                                drawMainScreen();
                            }
                        }
                    }
                }
            }
        }
        else if (CURRENT_SCREEN == 5)
        {
            // captures touches on drawOptionsScreen()
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN <Back pressed// BTN <Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 3);
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
                drawButton(85, 70, 65, 40, "WiFi", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                IS_WIFI_MODE_ON = true;
                drawRemoteScreen();
            }
            if (lx > 10 && lx < 75 && ly > 150 && ly < 190)
            {
                // Touched Align Button
                drawButton(10, 150, 65, 40, "Align", 0, BTN_L_BORDER, L_TEXT, 2);
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
                drawButton(165, 130, 65, 40, "Clock", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawClockScreen();
            }
            if (lx > 10 && lx < 75 && ly > 230 && ly < 270)
            {
                drawButton(10, 230, 65, 40, "Corr.", 0, BTN_L_BORDER, L_TEXT, 2);
                delay(150);
                drawAlignCorrectionsScreen();
            }
        }
        else if (CURRENT_SCREEN == 6) // captures touches on drawStarSelectScreen()
        {
            int total_pages = NUM_ALIGN_STARS / 12; // (100 OBJECTS / 12 per page) = 8.3 pages
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
                if (STARS_PAGER < total_pages)
                {
                    delay(150);
                    drawAlignObjects_ali();
                }
                else
                {
                    STARS_PAGER = total_pages - 1;
                }
            }
            if (lx > 10 && lx < 110 && ly > 270 && ly < 310)
            {
                // BTN <prev pressed
                LAST_BUTTON = 2;
                drawButton(10, 270, 100, 40, "< PREV", 0, BTN_L_BORDER, L_TEXT, 2);
                STARS_PAGER -= 1;
                if (STARS_PAGER >= 0)
                {
                    delay(150);
                    drawAlignObjects_ali();
                }
                else
                {
                    STARS_PAGER = 0;
                }
            }
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (lx > ((j * 75) + 10) && lx < ((j * 75) + 81) && ly > ((i * 50) + 45) && ly < ((i * 50) + 90))
                    {
                        // found button pressed.... now I need to get his ID and link to the ARRAY;
                        tft.drawRect(((j * 75) + 10), ((i * 50) + 45), 71, 45, BTN_L_BORDER);
                        int zz = ((STARS_PAGER * 12) + (i * 3) + j);
#ifdef SERIAL_DEBUG
                        Serial.print("Selected Alignment Star index ");
                        Serial.print(zz);
                        Serial.println("");
#endif
                        if (OBJECTS[zz] != "")
                        {
                            processAlignmentStar(zz, CURRENT_ALIGN_STAR);
                            // celestialToEquatorial(CURRENT_ALIGN_STAR.ra, CURRENT_ALIGN_STAR.dec,
                            //                       OBSERVATION_LATTITUDE_RADS, LST_RADS,
                            //                       &this_alt, &this_az);
                            getAltAzTrig(CURRENT_ALIGN_STAR.ra, CURRENT_ALIGN_STAR.dec, CURRENT_ALIGN_STAR.time, CURRENT_ALIGN_STAR.lat, &CURRENT_ALIGN_STAR.alt, &CURRENT_ALIGN_STAR.az);
                            uint8_t n = ALIGN_STEP - 1;
                            // If catalogue star is higher than 25 degrees above the horizon.
                            if (CURRENT_ALIGN_STAR.alt > 0.436332313)
                            {
                                // Copy catalogue star to alignment star.
                                ALIGNMENT_STARS[n] = CURRENT_ALIGN_STAR;
#ifdef SERIAL_DEBUG
                                Serial.print("Star number ");
                                Serial.print(n);
                                Serial.println("");
                                Serial.print(ALIGNMENT_STARS[n].name);
                                Serial.print(" - RA: ");
                                Serial.print(ALIGNMENT_STARS[n].ra);
                                Serial.print(" DEC: ");
                                Serial.print(ALIGNMENT_STARS[n].dec);
                                Serial.print(" ALT: ");
                                Serial.print(ALIGNMENT_STARS[n].alt);
                                Serial.print(" AZ: ");
                                Serial.print(ALIGNMENT_STARS[n].az);
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
                // Here we need to know which Star is this - 1st, 2nd, 3rd... etc ?
                // In order to use Ralph Pass alignment procedure described on http://rppass.com/
                // http://rppass.com/align.pdf - the actual PDF
                // if (ALIGN_STEP == 1)
                // {
                drawButton(130, 270, 100, 40, "ALIGN", 0, BTN_L_BORDER, L_TEXT, 2);

                uint8_t starNum = ALIGN_STEP - 1;
                CURRENT_ALIGN_STAR = ALIGNMENT_STARS[starNum];

                currentElevAngle();
                // if (OBSERVATION_LATTITUDE_RADS < 0)
                //     az_tmp = reverseRev(az_tmp);
                // if (alt_tmp > QRT_REV || alt_tmp < -QRT_REV)
                // {
                //     alt_tmp = HALF_REV - alt_tmp;
                //     az_tmp = validRev(az_tmp + HALF_REV);
                // }
                CURRENT_ALIGN_STAR.time = LST_RADS;
                CURRENT_ALIGN_STAR.lat = OBSERVATION_LATTITUDE_RADS;
                CURRENT_ALIGN_STAR.alt = CURR_ALT_RADS;
                CURRENT_ALIGN_STAR.az = CURR_AZ_RADS;

#ifdef PERFECT_ALIGN
                // Debug with proper trig values
                CURRENT_ALIGN_STAR.alt = ALIGNMENT_STARS[starNum].alt;
                CURRENT_ALIGN_STAR.az = ALIGNMENT_STARS[starNum].az;
#endif

                ALIGNMENT_STARS[starNum] = CURRENT_ALIGN_STAR;
#ifdef SERIAL_DEBUG
                if (ALIGN_TYPE == 1)
                    Serial.print("Auto Align on ");
                else
                    Serial.print("Manual Align on ");
                Serial.print(ALIGNMENT_STARS[starNum].name);
                Serial.println("");
                Serial.print("Alt ");
                Serial.print(ALIGNMENT_STARS[starNum].alt);
                Serial.print(" Az ");
                Serial.print(ALIGNMENT_STARS[starNum].az);
                Serial.println("");
#endif
                delay(150);
                if (ALIGN_STEP == 1)
                {
                    ALIGN_STEP++;
                    //setRef_1(ALIGNMENT_STARS[starNum].ra, ALIGNMENT_STARS[starNum].dec, ALIGNMENT_STARS[starNum].time, ALIGNMENT_STARS[starNum].az, ALIGNMENT_STARS[starNum].alt);
                    if (ALIGNMENT_METHOD == TAKI_BASIC)
                    {
                        basic_Init();
                        basic_AddStar(starNum + 1, NUM_ALIGNMENT_STARS, ALIGNMENT_STARS[starNum].ra, ALIGNMENT_STARS[starNum].dec, ALIGNMENT_STARS[starNum].time, ALIGNMENT_STARS[starNum].alt, ALIGNMENT_STARS[starNum].az);
                    }
                    else if (ALIGNMENT_METHOD == TAKI_ADVANCED)
                    {
                        zeroArrays(CMWS_ALIGN);
                        CMWS_ALIGN.one.ra = ALIGNMENT_STARS[starNum].ra;
                        CMWS_ALIGN.one.dec = ALIGNMENT_STARS[starNum].dec;
                        CMWS_ALIGN.one.sidT = ALIGNMENT_STARS[starNum].time;
                        CMWS_ALIGN.one.alt = ALIGNMENT_STARS[starNum].alt;
                        CMWS_ALIGN.one.az = ALIGNMENT_STARS[starNum].az;
                        copyPosition(CMWS_ALIGN.one, CMWS_ALIGN.current);
                        addAlignStar(starNum + 1, CMWS_ALIGN);
                    }

                    if (ALIGN_TYPE == 1)
                        drawAlignScreen();
                    else
                        drawStarSelectScreen();
                }
                else if (ALIGN_STEP == 2)
                {
                    ALIGN_STEP++;
                    //setRef_2(ALIGNMENT_STARS[starNum].ra, ALIGNMENT_STARS[starNum].dec, ALIGNMENT_STARS[starNum].time, ALIGNMENT_STARS[starNum].az, ALIGNMENT_STARS[starNum].alt);
                    if (ALIGNMENT_METHOD == TAKI_BASIC)
                    {
                        basic_AddStar(starNum + 1, NUM_ALIGNMENT_STARS, ALIGNMENT_STARS[starNum].ra, ALIGNMENT_STARS[starNum].dec, ALIGNMENT_STARS[starNum].time, ALIGNMENT_STARS[starNum].alt, ALIGNMENT_STARS[starNum].az);
                    }
                    else if (ALIGNMENT_METHOD == TAKI_ADVANCED)
                    {
                        CMWS_ALIGN.two.ra = ALIGNMENT_STARS[starNum].ra;
                        CMWS_ALIGN.two.dec = ALIGNMENT_STARS[starNum].dec;
                        CMWS_ALIGN.two.sidT = ALIGNMENT_STARS[starNum].time;
                        CMWS_ALIGN.two.alt = ALIGNMENT_STARS[starNum].alt;
                        CMWS_ALIGN.two.az = ALIGNMENT_STARS[starNum].az;
                        copyPosition(CMWS_ALIGN.two, CMWS_ALIGN.current);
                        addAlignStar(starNum + 1, CMWS_ALIGN);
                    }

                    if (NUM_ALIGNMENT_STARS == 2)
                    {
                        //autoRef_3();
                        if (ALIGNMENT_METHOD == TAKI_BASIC)
                        {
                            basic_GenerateThirdStar();
                            basic_TransformMatrix();
                            if (CORRECT_MOUNT_ERRS)
                                basic_BestZ123();
                        }
                        else if (ALIGNMENT_METHOD == TAKI_ADVANCED)
                        {
                            initMatrix(2, CMWS_ALIGN);
                            if (CORRECT_MOUNT_ERRS)
                                bestZ123(CMWS_ALIGN, CMWS_ALIGN.z1Error, CMWS_ALIGN.z2Error, CMWS_ALIGN.z3Error);
                        }
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
                    //setRef_3(ALIGNMENT_STARS[starNum].ra, ALIGNMENT_STARS[starNum].dec, ALIGNMENT_STARS[starNum].time, ALIGNMENT_STARS[starNum].az, ALIGNMENT_STARS[starNum].alt);
                    if (ALIGNMENT_METHOD == TAKI_BASIC)
                    {
                        basic_AddStar(starNum + 1, NUM_ALIGNMENT_STARS, ALIGNMENT_STARS[starNum].ra, ALIGNMENT_STARS[starNum].dec, ALIGNMENT_STARS[starNum].time, ALIGNMENT_STARS[starNum].alt, ALIGNMENT_STARS[starNum].az);
                        basic_TransformMatrix();
                        if (CORRECT_MOUNT_ERRS)
                            basic_BestZ123();
                    }
                    else if (ALIGNMENT_METHOD == TAKI_ADVANCED)
                    {
                        CMWS_ALIGN.three.ra = ALIGNMENT_STARS[starNum].ra;
                        CMWS_ALIGN.three.dec = ALIGNMENT_STARS[starNum].dec;
                        CMWS_ALIGN.three.sidT = ALIGNMENT_STARS[starNum].time;
                        CMWS_ALIGN.three.alt = ALIGNMENT_STARS[starNum].alt;
                        CMWS_ALIGN.three.az = ALIGNMENT_STARS[starNum].az;
                        copyPosition(CMWS_ALIGN.three, CMWS_ALIGN.current);
                        addAlignStar(starNum + 1, CMWS_ALIGN);
                        initMatrix(3, CMWS_ALIGN);
                        if (CORRECT_MOUNT_ERRS)
                            bestZ123(CMWS_ALIGN, CMWS_ALIGN.z1Error, CMWS_ALIGN.z2Error, CMWS_ALIGN.z3Error);
                    }
                    drawMainScreen();
                }
            }
        }
        else if (CURRENT_SCREEN == 8)
        {
            if (lx > 165 && lx < 230 && ly > 10 && ly < 40)
            {
                // BTN Back pressed
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 3);
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
                drawButton(165, 10, 65, 30, "BACK", 0, BTN_L_BORDER, L_TEXT, 3);
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 110 && ly > 50 && ly < 90)
            {
                // BTN Mercury pressed
                drawButton(10, 50, 100, 40, "Mercury", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(0, CURRENT_OBJECT);
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 50 && ly < 90)
            {
                // BTN Mercury pressed
                drawButton(130, 50, 100, 40, "Venus", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(1, CURRENT_OBJECT);
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 110 && ly > 110 && ly < 150)
            {
                // BTN Mercury pressed
                drawButton(10, 110, 100, 40, "Mars", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(3, CURRENT_OBJECT);
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 110 && ly < 150)
            {
                // BTN Mercury pressed
                drawButton(130, 110, 100, 40, "Jupiter", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(4, CURRENT_OBJECT);
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 110 && ly > 170 && ly < 210)
            {
                // BTN Mercury pressed
                drawButton(10, 170, 100, 40, "Saturn", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(5, CURRENT_OBJECT);
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 170 && ly < 210)
            {
                // BTN Mercury pressed
                drawButton(130, 170, 100, 40, "Uranus", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(6, CURRENT_OBJECT);
                delay(150);
                drawMainScreen();
            }
            if (lx > 10 && lx < 110 && ly > 230 && ly < 270)
            {
                // BTN Mercury pressed
                drawButton(10, 230, 100, 40, "Neptune", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(7, CURRENT_OBJECT);
                delay(150);
                drawMainScreen();
            }
            if (lx > 130 && lx < 230 && ly > 230 && ly < 270)
            {
                // BTN Mercury pressed
                drawButton(130, 230, 100, 40, "Pluto", 0, BTN_L_BORDER, L_TEXT, 2);
                getPlanetPosition(8, CURRENT_OBJECT);
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
        bt_reply = rad2hms(CURR_RA_RADS, true, false);
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
        bt_reply = rad2dms(CURR_DEC_RADS, true, false);
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
        bt_reply = rad2dms(CURR_ALT_RADS, true, false);
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
        bt_reply = rad2dms(CURR_AZ_RADS, true, true);
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

void processBTCommand()
{
#ifdef SERIAL_DEBUG
    Serial.print("processBTCommand: ");
    Serial.print(BT_COMMAND);
    Serial.println("");
#endif
    if (BT_COMMAND == GET_RA)
    {
        SerialBT.print("#" + rad2hms(CURR_RA_RADS, BT_PRECISION, false) + "#");
#ifdef SERIAL_DEBUG
        Serial.print(":GR Called: ");
        Serial.println("");
#endif
    }
    else if (BT_COMMAND == GET_DEC)
    {
        SerialBT.print("#" + rad2dms(CURR_DEC_RADS, BT_PRECISION, false) + "#");
#ifdef SERIAL_DEBUG
        Serial.print(":GD Called: ");
        Serial.println("");
#endif
    }
    else if (BT_COMMAND == CHANGE_PRECISION)
        BT_PRECISION = !BT_PRECISION;
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

    CMWS_ALIGN = ConvertMatrixWorkingStorage();

    LOAD_SELECTOR = 1; // Load Messier by default

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

    // Init Bluetooth
    // SerialBT.begin("Skywatcher-BLE");

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

        //             BT_CHARACTER = SerialBT.read();
        //             if (BT_STATE == BT_START)
        //             {
        //                 if (BT_CHARACTER == START_CHAR)
        //                 {
        //                     BT_STATE = BT_END;
        //                     BT_COMMAND = "";
        // #ifdef SERIAL_DEBUG
        //                     Serial.print("BT - GOT START CHAR: ");
        //                     Serial.println("");
        // #endif
        //                 }
        //             }
        //             else if (BT_STATE == BT_END)
        //             {
        //                 if (BT_CHARACTER == END_CHAR)
        //                 {
        //                     processBTCommand();
        //                     BT_STATE = BT_START;
        // #ifdef SERIAL_DEBUG
        //                     Serial.print("BT - GOT END CHAR: ");
        //                     Serial.println("");
        // #endif
        //                 }
        //                 else
        //                 {
        //                     BT_COMMAND += BT_CHARACTER;
        //                 }
        //             }

        // char input[4];
        // SerialBT.readBytes(input, 4);
        // if (input[0] == START_CHAR && input[3] == END_CHAR)
        // {
        //     if (input[1] == (char)'G')
        //     {
        //         if (input[2] == (char)'R')
        //             BT_COMMAND_STR = 1;
        //         else if (input[2] == (char)'D')
        //             BT_COMMAND_STR = 2;
        //         if (input[2] == (char)'A')
        //             BT_COMMAND_STR = 3;
        //         else if (input[2] == (char)'Z')
        //             BT_COMMAND_STR = 4;
        //     }
        //     // if (input[1] == (char)'G' && input[2] == (char)'A')
        //     //     BT_COMMAND_STR = 1;
        //     // else if (input[1] == (char)'G' && input[2] == (char)'Z')
        //     //     BT_COMMAND_STR = 2;
        //     // else if (input[1] == (char)'G' && input[2] == (char)'R')
        //     //     BT_COMMAND_STR = 3;
        //     // else if (input[1] == (char)'G' && input[2] == (char)'D')
        //     //     BT_COMMAND_STR = 4;

        //     considerBTCommands();
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
            //tft.print("ALT: ");
            tft.print(CURR_ALT);
            tft.setCursor(70, 90);
            //tft.print("AZ: ");
            tft.print(CURR_AZ);

            tft.setTextSize(1);
            tft.setCursor(70, 110);
            //tft.print("RA: ");
            tft.print(CURR_RA);
            tft.setCursor(70, 120);
            //tft.print("DEC: ");
            tft.print(CURR_DEC);
            LAST_POS_UPDATE = millis();
        }
        // Do regular time updates every 5000ms
        if ((millis() - UPDATE_LAST) > 5000)
        {
            calculateLST();
            considerTimeUpdates();
            UPDATE_LAST = millis();
            // #ifdef SERIAL_DEBUG
            //                 Serial.print("CURRENT_AZ_RADS: ");
            //                 Serial.print(CURR_AZ_RADS);
            //                 Serial.print(" DEGREES: ");
            //                 Serial.print(rad2dms(CURR_AZ_RADS, true, true));
            //                 Serial.println("");
            // #endif
        }
    }
    uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
    tft.getTouch(&t_x, &t_y);
    considerTouchInput(t_x, t_y);
    yield();
}
