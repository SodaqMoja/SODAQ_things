/*
Copyright (c) 2018, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "lib/CborBuilder.h"
#include "lib/Sodaq_wdt.h"
#include "Globals.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/

#define TEMPERATURE_SENSOR_PIN  A7
#define ULTRASONIC_SENSOR_PIN   A7
#define LIGHT_SENSOR_PIN        A7
#define TILT_SENSOR_PIN         A7
#define PIR_SENSOR_PIN          A7
#define BUZZER_PIN              A7

#define DELAY_DEBOUNCE          50
#define DELAY_READSENSOR        1000
#define MORZE_DOT               100
#define MORZE_DASH              300
#define MORZE_DOT_SEPARATOR     200
#define MORZE_CHAR_SEPARATOR    400

enum SensorType {
  TemperatureSensor = 0,
  UltrasonicSensor  = 1,
  LightSensor       = 2,
  TiltSensor        = 3,
  PirSensor         = 4,
  Buzzer            = 5
};


/*******************************************************************************
 * State
 ******************************************************************************/

static Command menuCommands[] = {
  { "AT Command Mode", "CMD",  BootMenu::setCommandMode, Command::show_string },
  { "Init network   ", "INIT", BootMenu::setAtInitMode,  Command::show_string }
};

static Command menuParams[] = {
  { "IMEI                       ", 0,      0,                   Command::show_string, &modem.imei },
  { "Sensor Type                ", "sty=", Command::set_uint8,  Command::show_uint8,  &params._sensorType },
  { "Upload Interval (min)      ", "sri=", Command::set_uint16, Command::show_uint16, &params._sensorReadInterval },
  { "GPS Fix Timeout (sec)      ", "gft=", Command::set_uint16, Command::show_uint16, &params._gpsFixTimeout },
  { "Minimum sat count          ", "sat=", Command::set_uint8,  Command::show_uint8,  &params._gpsMinSatelliteCount },
  { "APN                        ", "apn=", Command::set_string, Command::show_string,  params._apn, sizeof(params._apn) },
  { "Network (LTE-M=7, NB-Iot=8)", "rat=", Command::set_uint8,  Command::show_uint8,  &params._urat },
  { "Band Mask                  ", "bnm=", Command::set_string, Command::show_string,  params._bandMask, sizeof(params._bandMask) },
  { "Server URL/IP              ", "url=", Command::set_string, Command::show_string,  params._endpointUrl, sizeof(params._endpointUrl) },
  { "Server port                ", "prt=", Command::set_uint16, Command::show_uint16, &params._endpointPort },
  { "GPS (OFF=0 / ON=1)         ", "gps=", Command::set_uint8,  Command::show_uint8,  &params._isGpsEnabled },
  { "Debug (OFF=0 / ON=1)       ", "dbg=", Command::set_uint8,  Command::show_uint8,  &params._isDebugOn },
  { "High temperature trigger   ", "hgh=", Command::set_int8,   Command::show_int8,   &params._temperatureHigh },
  { "Low temperature trigger    ", "low=", Command::set_int8,   Command::show_int8,   &params._temperatureLow },
  { "On-the-move Functionality  ", 0,      0,                   Command::show_title,   0 },
  { "Acceleration% (100% = 8g)  ", "acc=", Command::set_uint8,  Command::show_uint8,  &params._accelerationPercentage },
  { "Acceleration Duration      ", "acd=", Command::set_uint8,  Command::show_uint8,  &params._accelerationDuration },
  { "Accelerator Trigger",         0,      0,                   Command::show_text,    0 },
  { "  Sensitivity (1-50)       ", "act=", Command::set_uint8,  Command::show_uint8,  &params._accelerationSensitivity },
  { "  Back-off time (sec)      ", "bot=", Command::set_uint8,  Command::show_uint8,  &params._onTheMoveBackoffTime }
};

BootMenu menu(menuCommands, sizeof(menuCommands) / sizeof(menuCommands[0]));
Config   config(menuParams, sizeof(menuParams)   / sizeof(menuParams[0]));
Display  display;
Maker    maker;
Modem    modem;
Params   params;

static unsigned char SeeedLogo[] = {
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x3f, 0x1f, 0x0f, 0x0f, 0x07, 0x87, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc7, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x0f, 0x07, 
	0x87, 0x83, 0xc3, 0xc3, 0xc3, 0xc3, 0x83, 0x87, 0x87, 0x07, 0x0f, 0x0f, 0x1f, 0x3f, 0x7f, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x03, 0x03, 0x03, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0x83, 
	0x87, 0x87, 0x07, 0x0f, 0x0f, 0x1f, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 
	0x1f, 0x0f, 0x0f, 0x87, 0x87, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0x87, 0x87, 0x07, 0x0f, 0x1f, 0x3f, 
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x0f, 0x07, 0x87, 0x87, 
	0x83, 0xc3, 0xc3, 0xc3, 0xc3, 0x83, 0x87, 0x07, 0x0f, 0x0f, 0x1f, 0x3f, 0x7f, 0xff, 0xff, 0xff, 
	0x83, 0x80, 0x80, 0x80, 0x80, 0x86, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 
	0x87, 0x07, 0x07, 0x07, 0xff, 0xff, 0xff, 0x1f, 0x03, 0x00, 0x00, 0xe0, 0xf8, 0xfe, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf0, 0x00, 0x00, 0x01, 
	0x03, 0x7f, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf0, 0x00, 0x00, 0x01, 0x0f, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 
	0x00, 0x80, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x82, 0x00, 0x00, 
	0x00, 0x01, 0xff, 0xff, 0xff, 0x3f, 0x03, 0x00, 0x00, 0x80, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0xc0, 0x00, 0x00, 0x03, 0x1f, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 
	0x1f, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xf8, 0xc0, 0x00, 0x00, 0x07, 0x1f, 0x3f, 0x7f, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x0f, 0x00, 0x00, 0x80, 
	0xe0, 0xfe, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0x7f, 0x3f, 0x07, 0x00, 0x00, 0x80, 0xf0, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
	0x00, 0x00, 0xff, 0xff, 0xff, 0xfc, 0xc0, 0x80, 0x00, 0x01, 0x0f, 0x3f, 0x7f, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xcf, 0x87, 0x03, 0x07, 0x0f, 0x1f, 0x1f, 0x03, 0x00, 0x00, 0xc0, 0xf8, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xe1, 0xe3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xe1, 0xe0, 0xf0, 
	0xf0, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xf0, 0xe0, 
	0xe1, 0xe1, 0xc3, 0xc3, 0xc3, 0xc3, 0xc1, 0xe1, 0xe1, 0xe0, 0xf0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc1, 
	0xe1, 0xe1, 0xe0, 0xf0, 0xf0, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xc0, 0xc0, 
	0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 
	0xc0, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xf0, 0xe0, 0xe1, 0xe1, 
	0xc1, 0xc3, 0xc3, 0xc3, 0xc1, 0xe1, 0xe1, 0xe0, 0xf0, 0xf0, 0xf0, 0xe0, 0xe0, 0xe1, 0xf3, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

const char* SensorHeaders[]          = { "Temperature", "Distance", "Light", "Tilt", "PIR Motion", "" };
const char* SensorName[]             = { "Temperature", "Ultrasonic", "Light", "Tilt", "PIR Motion", "Buzzer" };
const char* SensorMeasurementUnits[] = { "C", "cm", "", "", "", "" };
const char* EmptyString              = "";

volatile bool     isDigitalSensorStateChanged     = false;
volatile bool     currentDigitalSensorStableState = false;
volatile bool     lastDigitalSensorStableState    = false;
volatile uint32_t lastDigitalSensorChanged        = 0;

volatile uint32_t lastSensorRead      = 0;
static   float    sensorValue         = 0.0;
static   float    previousSensorValue = 0.0;
static   bool     sensorTrigger       = false;

static CborBuilder cbor;

static uint8_t digitalSensorPin;


/*******************************************************************************
 * Public methods
 ******************************************************************************/

void Maker::setup()
{
  sodaq_wdt_disable();
  Wire.begin();

  display.init();
  display.printBitmap(SeeedLogo, 1024); // 1024 = 128 Pixels * 64 Pixels / 8

  setupBoot();

  debugPrintln("Initializing...");
  display.clear();
  display.printLine("Initializing...", NormalMessageType);
  if (!initSensor()) {
    debugPrintln("Failed to init sensor");
    display.printLine("Failed to",   ErrorMessageType);
    display.printLine("init sensor", ErrorMessageType);
    fatal();
  }

  setupFinal();

  if (isGpsInitialized) {
      readSensorValue();
      handleSensorReadProcedure(false);
  }
}

void Maker::loop()
{
  preLoop();
  if (params.getSensorType() == Buzzer && updateOnTheMoveTimestampFlag) { soundMorseIOT(); }
  checkSensorState();
  if (pendingSensorRead) { handleSensorReadProcedure(true); }
  postLoop();
}


/*******************************************************************************
 * Protected virtual methods
 ******************************************************************************/

void Maker::displayState(uint8_t state)
{
  display.clear();
  display.printLine("Turn board to",  NormalMessageType);

  if (state > 5) {
    display.printLine(EmptyString,  NormalMessageType);
    display.printLine(params.getIsGpsEnabled() ? "switch GPS off" : "switch GPS on", NormalMessageType);
    return;
  }

  display.printLine(state < 5 ? "select sensor:" : "select:", NormalMessageType);
  display.printLine(EmptyString,  NormalMessageType);
  display.printLine(SensorName[state] , NormalMessageType);
}

void Maker::setState(uint8_t state)
{
  if (state <= 5) {
    if (state != params.getSensorType()) {
      params._sensorType = state;
      config.setNeedCommitFlag();
    }
  } else if (state > 5) {
    params._isGpsEnabled = params.getIsGpsEnabled() ? 0 : 1;
    config.setNeedCommitFlag();
  }

  if (params.getSensorType() > 5) { params._sensorType = 0; }

  display.clear();
  display.printLine(params.getSensorType() < 5 ? "Selected sensor:" : "Connect", NormalMessageType);
  display.printLine(SensorName[params.getSensorType()] , NormalMessageType);
  display.printLine(params.getIsGpsEnabled() ? "GPS: on" : "GPS: off", NormalMessageType);
  display.printLine("To change sensor", NormalMessageType);
  display.printLine("or activate GPS",  NormalMessageType);
  display.printLine("hold board",       NormalMessageType);
  display.printLine("upside down...",   NormalMessageType);
}


/*******************************************************************************
 * Private methods
 ******************************************************************************/

void Maker::checkSensorDigital()
{
  if (isDigitalSensorStateChanged && abs(millis() - lastDigitalSensorChanged) > DELAY_DEBOUNCE) {
    readSensorValue();

    if (currentDigitalSensorStableState != lastDigitalSensorStableState) {
      lastDigitalSensorStableState = currentDigitalSensorStableState;
      pendingSensorRead = true;
      debugPrintln("Sensor state changed, pending sensor read...");
    }

    isDigitalSensorStateChanged = false;
  }
}

void Maker::checkSensorLight()
{
  if (abs(millis() - lastSensorRead) > DELAY_READSENSOR) {
    previousSensorValue = sensorValue;
    readSensorValue();
    lastSensorRead = millis();

    if (sensorTrigger && abs(sensorValue - previousSensorValue) > LIGHT_SENSOR_DELTA) {
      pendingSensorRead = true;
      debugPrintln("Light sensor delta > 10000, pending sensor read...");
    }

    sensorTrigger = true;
  }
}

void Maker::checkSensorState()
{
  switch (params.getSensorType()) {
    case TemperatureSensor: checkSensorTemperature(); break;
    case UltrasonicSensor:  checkSensorUltrasonic();  break;
    case LightSensor:       checkSensorLight();       break;
    case TiltSensor:
    case PirSensor:         checkSensorDigital();     break;
  }
}

void Maker::checkSensorTemperature()
{
  if (abs(millis() - lastSensorRead) > DELAY_READSENSOR) {
    readSensorValue();

    if ((sensorValue > params.getTemperatureLow() + 2) && (sensorValue < params.getTemperatureHigh() - 2)) {
      sensorTrigger = true;
      return;
    }

    if (pendingSensorRead) { return; }

    if (sensorTrigger && sensorValue > params.getTemperatureHigh()) {
      pendingSensorRead = true;
      debugPrint("Temperature sensor value > ");
      debugPrint(params.getTemperatureHigh());
      debugPrintln(", pending sensor read...");
      sensorTrigger = false;
      return;
    }

    if (sensorTrigger && sensorValue < params.getTemperatureLow()) {
      pendingSensorRead = true;
      debugPrint("Temperature sensor value < ");
      debugPrint(params.getTemperatureLow());
      debugPrintln(", pending sensor read...");
      sensorTrigger = false;
    }
  }
}

void Maker::checkSensorUltrasonic()
{
  if (abs(millis() - lastSensorRead) > DELAY_READSENSOR) { readSensorValue(); }
}

void Maker::handleSensorReadProcedure(bool isFromLoop)
{
  char lat[64];
  char lon[64];

  if (isGpsInitialized) { display.printFooter("GPS Fix..."); }

  bool gpsSuccess = isFromLoop && handleGpsFixSequence();

  if (gpsSuccess) {
    sprintf(lat, "Lat: %2d.%d", tmpPosition.Lat / 10000000, abs(tmpPosition.Lat % 10000000 / 100));
    sprintf(lon, "Lon: %2d.%d", tmpPosition.Lon / 10000000, abs(tmpPosition.Lon % 10000000 / 100));
  }

  if (params.getSensorType() != Buzzer) {
    debugPrint("Sensor value: ");
    debugPrintln(sensorValue);

    char sensorValueString[64];
    sprintf(sensorValueString, "%d.%d %s",
            (int)sensorValue, abs((int)(sensorValue * 10) % 10), getSensorMeasurementUnit());

    if (gpsSuccess) {
      display.printSensorValue(getSensorHeader(), sensorValueString, lat, lon);
    } else {
      display.printSensorValue(getSensorHeader(), sensorValueString);
    }
  } else if (gpsSuccess) {
    display.printSensorValue(EmptyString, EmptyString, lat, lon);
  }

  uint16_t batteryValue = getBatteryVoltageMV();
  debugPrint("Battery voltage: ");
  debugPrint(batteryValue);
  debugPrintln(" mV");

  display.printFooter("Uploading...");

  if (params.getSensorType() == Buzzer) { soundMorseSMS(); }

  updateCborPayload(sensorValue, batteryValue);

  bool b = uploadData(cbor.getData(), cbor.getSize(), params.getIsATTHeaderOn(), isFromLoop);

  // wait for some time before clearing the footer
  sodaq_wdt_safe_delay(2000);
  display.printFooter(b ? "Uploaded" : EmptyString);
}

uint16_t Maker::getLightSensorValue()
{
  return map(analogRead(LIGHT_SENSOR_PIN), 0, 800, 0, UINT16_MAX-1);
}

const char* Maker::getSensorHeader()
{
  if (params.getSensorType() < sizeof(SensorHeaders) / sizeof(SensorHeaders[0])) {
    return SensorHeaders[params.getSensorType()];
  }

  return EmptyString;
}

const char* Maker::getSensorMeasurementUnit()
{
  if (params.getSensorType() < sizeof(SensorMeasurementUnits) / sizeof(SensorMeasurementUnits[0])) {
    return SensorMeasurementUnits[params.getSensorType()];
  }

  return EmptyString;
}

float Maker::getTemperatureSensorValue()
{
  const int B  = 4275;   // B value of the thermistor
  const int R0 = 100000; // R0 = 100k

  int   a = analogRead(TEMPERATURE_SENSOR_PIN);
  float R = (1023.0 / a - 1.0) * R0;

  return 1.0 / (log(R / R0) / B + 1 / 298.15) - 273.15; // convert to temperature via datasheet
}

uint32_t Maker::getUltrasonicSensorValue()
{
  pinMode(ULTRASONIC_SENSOR_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_SENSOR_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(ULTRASONIC_SENSOR_PIN, HIGH);
  delayMicroseconds(5);

  digitalWrite(ULTRASONIC_SENSOR_PIN, LOW);
  pinMode(ULTRASONIC_SENSOR_PIN, INPUT);

  uint32_t duration = pulseIn(ULTRASONIC_SENSOR_PIN, HIGH);
  uint32_t rangeInCentimeters = duration / 29 / 2;

  return rangeInCentimeters;
}

bool Maker::initSensor()
{
  void digitalSensorISR();

  switch (params.getSensorType())
  {
    case TemperatureSensor: // nothing to do
      break;
    case UltrasonicSensor:  // nothing to do
      break;
    case LightSensor:       // nothing to do
      break;
    case TiltSensor:
      digitalSensorPin = TILT_SENSOR_PIN;
      pinMode(TILT_SENSOR_PIN, INPUT);
      attachInterrupt(TILT_SENSOR_PIN, digitalSensorISR, CHANGE);
      currentDigitalSensorStableState = digitalRead(TILT_SENSOR_PIN); // read first value
      break;
    case PirSensor:
      digitalSensorPin = PIR_SENSOR_PIN;
      pinMode(PIR_SENSOR_PIN, INPUT);
      attachInterrupt(PIR_SENSOR_PIN, digitalSensorISR, CHANGE);
      currentDigitalSensorStableState = digitalRead(PIR_SENSOR_PIN); // read first value
      break;
    case Buzzer:
      pinMode(BUZZER_PIN, OUTPUT);
      break;
    default:
      return false;
      break;
  }

  return true;
}

void Maker::readSensorValue()
{
  switch (params.getSensorType()) {
    case TemperatureSensor: sensorValue = getTemperatureSensorValue(); break;
    case UltrasonicSensor:  sensorValue = getUltrasonicSensorValue();  break;
    case LightSensor:       sensorValue = getLightSensorValue();       break;
    case TiltSensor:
    case PirSensor:         sensorValue = currentDigitalSensorStableState ? 1.0 : 0.0; break;
  }

  lastSensorRead = millis();
}

void Maker::updateCborPayload(float sensorValue, uint16_t batteryValue)
{
  uint8_t t = params.getSensorType();
  uint8_t i = t <= 4 ? 2 : 1;
  if (isGpsInitialized) { i++; }

  cbor.reset();
  cbor.writeTag(120);
  cbor.writeArray(1);
  cbor.map(i);

  switch (t)
  {
    case TemperatureSensor: cbor.addNumber (sensorValue, "t");          break;
    case UltrasonicSensor:  cbor.addNumber (sensorValue, "u");          break;
    case LightSensor:       cbor.addNumber (sensorValue, "l");          break;
    case TiltSensor:        cbor.addBoolean(sensorValue > 0.8, "tilt"); break;
    case PirSensor:         cbor.addBoolean(sensorValue > 0.8, "p");    break;
  }

  cbor.addInteger(batteryValue, "v");

  if (isGpsInitialized) {
    cbor.addGps(tmpPosition.Lat / GPS_SCALING, tmpPosition.Lon / GPS_SCALING, tmpPosition.Alt, "g");
  }
}


/*******************************************************************************
 * Private static methods
 ******************************************************************************/

void Maker::soundMorseDash()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(MORZE_DASH);
  digitalWrite(BUZZER_PIN, LOW);
}

void Maker::soundMorseDot()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(MORZE_DOT);
  digitalWrite(BUZZER_PIN, LOW);
}

void Maker::soundMorseCharDelay()
{
  delay(MORZE_CHAR_SEPARATOR);
}

void Maker::soundMorseDotDelay()
{
  delay(MORZE_DOT_SEPARATOR);
}

void Maker::soundMorseIOT()
{
  soundMorseDot();  soundMorseDotDelay();
  soundMorseDot();  soundMorseCharDelay();
  soundMorseDash(); soundMorseDotDelay();
  soundMorseDash(); soundMorseDotDelay();
  soundMorseDash(); soundMorseDotDelay();
  soundMorseDash(); soundMorseCharDelay();
  soundMorseDash(); soundMorseCharDelay();
}

void Maker::soundMorseSMS()
{
  soundMorseDot();  soundMorseDotDelay();
  soundMorseDot();  soundMorseDotDelay();
  soundMorseDot();  soundMorseCharDelay();
  soundMorseDash(); soundMorseDotDelay();
  soundMorseDash(); soundMorseCharDelay();
  soundMorseDot();  soundMorseDotDelay();
  soundMorseDot();  soundMorseDotDelay();
  soundMorseDot();  soundMorseCharDelay();
}


/*******************************************************************************
 * Handlers
 ******************************************************************************/

void digitalSensorISR()
{
  currentDigitalSensorStableState = digitalRead(digitalSensorPin);
  isDigitalSensorStateChanged     = true;
  lastDigitalSensorChanged        = millis();
}
