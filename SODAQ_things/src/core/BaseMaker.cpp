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

#include <Wire.h>
#include "../lib/LedColor.h"
#include "../lib/MyTime.h"
#include "../lib/RTCTimer.h"
#include "../lib/RTCZero.h"
#include "../lib/Sodaq_wdt.h"
#include "../Globals.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/

#define ADC_AREF   3.3F
#define BATVOLT_R1 4.7F
#define BATVOLT_R2 10.0F


/*******************************************************************************
 * State
 ******************************************************************************/

static Time       time;
static RTCTimer   timer;
static RTCZero    rtc;
static int64_t    rtcEpochDelta;        // set in setNow() and used in getGpsFixAndHandle() for correcting time in loop
static UBlox      ublox;
static UdpHeader  defaultUdpHeader;

volatile bool     isTmpPositionAltered; // this is set to true only when tmpPosition is written by the gps delegate
volatile uint32_t lastOnTheMoveActivationTimestamp;
volatile bool     isBoardUpsideDown;
volatile uint32_t lastBoardUpsideDown;
volatile bool     minuteFlag;
volatile bool     updateOnTheMoveTimestampFlag;

static double     accelX = 0;
static double     accelY = 0;
static double     accelZ = 0;
static bool       isUSB;
static bool       isStateChanged;
static uint8_t    sensorState;

Sodaq_LSM303AGR BaseMaker::accelerometer;
Position BaseMaker::tmpPosition;

bool BaseMaker::isGpsInitialized             = false;
bool BaseMaker::pendingSensorRead            = false;
bool BaseMaker::updateOnTheMoveTimestampFlag = false;


/*******************************************************************************
 * Public methods
 ******************************************************************************/

bool BaseMaker::checkAccelerometer()
{
  if (ENABLE_DEEP_SLEEP || !ENABLE_DISPLAY) { return false; }

  double z = accelerometer.getZ();

  if (z < -0.7) {
    if (!isBoardUpsideDown) {
      isBoardUpsideDown   = true;
      lastBoardUpsideDown = millis();
      return false;
    }

    uint32_t i = millis();
    if (abs(i - lastBoardUpsideDown) > SWITCH_SENSOR_DELAY) {
      sensorState = sensorState < 6 ? sensorState + 1 : 0;
      isStateChanged      = true;
      lastBoardUpsideDown = i;
      displayState(sensorState);
      return true;
    }
  } else {
    isBoardUpsideDown = false;

    if (z > -0.2 && isStateChanged) {
      setState(sensorState);
      isStateChanged = false;
      return true;
    }
  }

  return false;
}


/*******************************************************************************
 * Protected methods
 ******************************************************************************/

void BaseMaker::fatal()
{
  setLedColor(RED);

  while (true) {
    sodaq_wdt_reset();
    if (ENABLE_DEEP_SLEEP) { systemSleep(); }
  }
}

/**
* Tries to get a GPS fix and handles the data.
* Times-out after params.getGpsFixTimeout seconds.
*/
bool BaseMaker::handleGpsFixSequence()
{
  if (!isGpsInitialized) {
    debugPrintln("GPS is not initialized, not used in this sequence.");
    return false;
  }

  setLedColor(MAGENTA);

  debugPrintln("GPS Fix sequence started...");

  bool isFixSuccessful = false;
  setGpsActive(true);

  tmpPosition.NumberOfSatellites = 0; // reset satellites to use them as a quality metric in the loop
  uint32_t startTime = getNow();
  while ((getNow() - startTime <= params.getGpsFixTimeout())
      && (tmpPosition.NumberOfSatellites < params.getGpsMinSatelliteCount()))
  {
    sodaq_wdt_reset();
    uint16_t bytes = ublox.available();

    if (bytes) {
      rtcEpochDelta = 0;
      isTmpPositionAltered = false;
      ublox.GetPeriodic(bytes); // calls the delegate method for passing results

      startTime += rtcEpochDelta; // just in case the clock was changed (by the delegate in ublox.GetPeriodic)

      // isPendingReportDataRecordNew guarantees at least a 3d fix or GNSS + dead reckoning combined
      // and is good enough to keep, but the while loop should keep trying until timeout or sat count larger than set
      if (isTmpPositionAltered) { isFixSuccessful = true; }
    }
  }

  setGpsActive(false); // turn off gps as soon as it is not needed

  // populate all fields of the report record
  tmpPosition.Timestamp = getNow();

  if (isFixSuccessful) {
    tmpPosition.TimeToFix = tmpPosition.Timestamp - startTime;
  } else {
    tmpPosition.TimeToFix = 0xFF;

    tmpPosition.Lat = 0;
    tmpPosition.Lon = 0;

    setLedColor(RED);
    sodaq_wdt_safe_delay(5000);
  }

  setLedColor(NONE);

  return isFixSuccessful;
}

void BaseMaker::postLoop()
{
  if (pendingSensorRead) {
    pendingSensorRead = false;
    lastOnTheMoveActivationTimestamp = getNow();

    saveAccelerometerPosition();
    setLedColor(NONE);
  }

  updateOnTheMoveTimestampFlag = false;

  if (ENABLE_DEEP_SLEEP) { systemSleep(); }
}

void BaseMaker::preLoop()
{
  if (sodaq_wdt_flag) {
    sodaq_wdt_reset();
    sodaq_wdt_flag = false;
  }

  if (!pendingSensorRead && !updateOnTheMoveTimestampFlag) {
    double t = (double)params.getAccelerationSensitivity() / 10;
    updateOnTheMoveTimestampFlag =
      abs(accelerometer.getX() - accelX) >= t ||
      abs(accelerometer.getY() - accelY) >= t ||
      abs(accelerometer.getZ() - accelZ) >= t;
  }

  if (updateOnTheMoveTimestampFlag) {
    setLedColor(BLUE);

    uint32_t i = getNow();
    uint32_t t = abs(i - lastOnTheMoveActivationTimestamp);
    if (t > params.getOnTheMoveBackoffTime()) {
      pendingSensorRead = true;
      lastOnTheMoveActivationTimestamp = i;
      debugPrintln("Acceleration handled, pending sensor read...");
    } else {
      t = params.getOnTheMoveBackoffTime() - t;
      if (t > 0 && t < 10000) {
        char buffer[128];

        sprintf(buffer, "Acceleration handled, no action required, wait %d seconds...", t);
        debugPrintln(buffer);

        sprintf(buffer, "Wait %d sec", t);
        display.printFooter(buffer);
      }

      sodaq_wdt_safe_delay(2000);
      setLedColor(NONE);
      display.printFooter("");
    }

    saveAccelerometerPosition();
  }

  checkTimer();
}

void BaseMaker::setupBoot()
{
  sodaq_wdt_disable();

  setLedColor(YELLOW);

  CONSOLE_STREAM.begin(CONSOLE_BAUD);

  pinMode(USB_DETECT, INPUT);
  isUSB = digitalRead(USB_DETECT);
  if (isUSB) { while (!CONSOLE_STREAM && millis() < STARTUP_DELAY); }

  CONSOLE_STREAM.println(PROJECT_VERSION "\r\nBooting up...");

  if (ENABLE_DEEP_SLEEP) { sodaq_wdt_enable(WDT_PERIOD_8X); }

  sodaq_wdt_reset();

  if ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM) { DEBUG_STREAM.begin(DEBUG_BAUD); }

  initRtc();

  Wire.begin();

  config.read();

  initAccelerometer();

  modem.initModem();
  modem.on();
  modem.readIMEI();

  if (ENABLE_DEEP_SLEEP) { sodaq_wdt_disable(); }
  handleBootUpCommands();

  modem.off();

  if(!config.isValid()) {
    display.printLine("Failed to",     ErrorMessageType);
    display.printLine("configuration", ErrorMessageType);
    fatal();
  }

  if (ENABLE_DEEP_SLEEP) { sodaq_wdt_enable(WDT_PERIOD_8X); }
}

void BaseMaker::setupFinal()
{
  // make sure the debug option is honored
  if (params.getIsDebugOn() && ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM)) { DEBUG_STREAM.begin(DEBUG_BAUD); }

  if (params.getIsGpsEnabled()) {
    debugPrintln("Initializing GPS...");
    display.printLine("Init GPS...", NormalMessageType);

    isGpsInitialized = initGps();
    if (!isGpsInitialized) {
      debugPrintln("Failed to initialize GPS!");
      display.printLine("Failed to", ErrorMessageType);
      display.printLine("init GPS",  ErrorMessageType);
      fatal();
    }
  }

  accelerometer.disableMagnetometer();
  pinMode(MAG_INT, OUTPUT);
  digitalWrite(MAG_INT, LOW);
  if (params.getAccelerationPercentage() > 0) { initOnTheMove(); }

  initRtcTimer();
  initUdpHeader();

  // disable the USB if not needed for debugging
  if (!params.getIsDebugOn() || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
    CONSOLE_STREAM.println("The USB is going to be disabled now.");
    modem.disableDiag();

    SerialUSB.flush();
    sodaq_wdt_safe_delay(3000);
    SerialUSB.end();
    USBDevice.detach();
    USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
  }

  // disable the debug stream if it is not disabled by the above
  if (!params.getIsDebugOn() && ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
    DEBUG_STREAM.flush();
    DEBUG_STREAM.end();
  }

  // disable the console stream if it is not disabled by the above,
  // and only if it is different than the debug stream
  if ((long)&CONSOLE_STREAM != (long)&SerialUSB && ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM)) {
    CONSOLE_STREAM.flush();
    CONSOLE_STREAM.end();
  }

  // wait for 2s before starting first sensor sequence
  sodaq_wdt_safe_delay(2000);

  saveAccelerometerPosition();

  pendingSensorRead = true;

  if (ENABLE_DEEP_SLEEP) { initSleep(); }
}

bool BaseMaker::uploadData(uint8_t* data, uint8_t size, bool needHeader, bool needBlink)
{
  bool b;

  if (needHeader) {
    size_t headerSize = sizeof(defaultUdpHeader.Raw);

    uint8_t buffer[headerSize + size];
    memcpy(buffer, defaultUdpHeader.Raw, headerSize);
    memcpy(buffer + headerSize, data, size);

    b = modem.transmit(buffer, headerSize + size);
  } else {
    b = modem.transmit(data, size);
  }

  showUploadStatus(b, needBlink);

  return b;
}


/*******************************************************************************
 * Protected virtual methods
 ******************************************************************************/

void BaseMaker::displayState(uint8_t state) {}

void BaseMaker::setState(uint8_t state) {}


/*******************************************************************************
 * Protected static methods
 ******************************************************************************/

uint16_t BaseMaker::getBatteryVoltageMV()
{
  return ADC_AREF * 1000 / ((1 << 10) - 1) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * analogRead(BAT_VOLT);
}


/*******************************************************************************
 * Private methods
 ******************************************************************************/

void BaseMaker::checkTimer()
{
  if (minuteFlag) {
    timer.update(); // handle scheduled events
    minuteFlag = false;
  }
}

/**
* Shows and handles the boot up commands.
*/
void BaseMaker::handleBootUpCommands()
{
  sensorState = params.getSensorType();
  isStateChanged = false;
  setState(sensorState);

  if (isUSB) {
    do { menu.showBootMenu(); } while (!config.isValid());
    config.showConfig();
  } else if (ENABLE_DISPLAY) {
    menu.showBootMenuOled();
  }

  debugPrintln(config.commit() ? "Configuration changes committed" : "Configuration not changed");
}

void BaseMaker::initAccelerometer()
{
  // Configure EIC to use GCLK1 which uses XOSC32K, XOSC32K is already running in standby
  // This has to be done after the first call to attachInterrupt()
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

  accelerometer.enableAccelerometer(
    Sodaq_LSM303AGR::LowPowerMode,
    Sodaq_LSM303AGR::HrNormalLowPower10Hz,
    Sodaq_LSM303AGR::XYZ,
    Sodaq_LSM303AGR::Scale8g,
    true);

  sodaq_wdt_safe_delay(100);
}

/**
* Initializes the GPS and leaves it on if succesful.
* Returns true if successful.
*/
bool BaseMaker::initGps()
{
  pinMode(GPS_ENABLE, OUTPUT);
  pinMode(GPS_TIMEPULSE, INPUT);

  // attempt to turn on and communicate with the GPS
  ublox.enable();
  digitalWrite(GPS_ENABLE, HIGH);
  ublox.flush();

  uint32_t startTime = getNow();
  bool found = false;
  while (!found && (getNow() - startTime <= GPS_COMM_CHECK_TIMEOUT)) {
    sodaq_wdt_reset();
    found = ublox.exists();
  }

  // check for success
  if (found) {
    setGpsActive(true); // properly turn on before returning
    return true;
  }

  debugPrintln("*** GPS not found!");

  // turn off before returning in case of failure
  setGpsActive(false);

  return false;
}

/**
* Initializes the on-the-move functionality (interrupt on acceleration).
*/
void BaseMaker::initOnTheMove()
{
  void accelerometerInt1Handler();

  pinMode(ACCEL_INT1, INPUT);
  attachInterrupt(ACCEL_INT1, accelerometerInt1Handler, CHANGE);

  accelerometer.enableInterrupt1(
    Sodaq_LSM303AGR::XHigh | Sodaq_LSM303AGR::XLow |
    Sodaq_LSM303AGR::YHigh | Sodaq_LSM303AGR::YLow |
    Sodaq_LSM303AGR::ZHigh | Sodaq_LSM303AGR::ZLow,
    params.getAccelerationPercentage() * 8.0 / 100.0,
    params.getAccelerationDuration(),
    Sodaq_LSM303AGR::MovementRecognition);
}

/**
 * Initializes the RTC Timer and schedules the default events.
 */
void BaseMaker::initRtcTimer()
{
  timer.setNowCallback(getNow); // set how to get the current time
  timer.allowMultipleEvents();

  resetRtcTimerEvents();
}

void BaseMaker::initUdpHeader()
{
  defaultUdpHeader.Version = 1; // always version=1
  memcpy(defaultUdpHeader.Imei, &modem.cachedImei, sizeof(defaultUdpHeader.Imei));
  // reverse the array
  // (the exaple shows that IMEI = 391855893742972 should be 01 64 64 0F 59 85 7C
  size_t n = sizeof(defaultUdpHeader.Imei);
  for (size_t i = 0; i < n / 2; ++i) {
    int tmp = defaultUdpHeader.Imei[i];
    defaultUdpHeader.Imei[i] = defaultUdpHeader.Imei[n - 1 - i];
    defaultUdpHeader.Imei[n - 1 - i] = tmp;
  }

  strncpy(defaultUdpHeader.Token, params.getAttToken(), sizeof(defaultUdpHeader.Token));
}

/**
* Clears the RTC Timer events and schedules the default events.
*/
void BaseMaker::resetRtcTimerEvents()
{
  void runSensorReadEvent(uint32_t now);

  timer.clearAllEvents();

  // Schedule the default fix event (if applicable)
  if (params.getSensorReadInterval() > 0) {
    timer.every(params.getSensorReadInterval() * 60, runSensorReadEvent);
  }
}

/**
* Turns the GPS on or off.
*/
void BaseMaker::setGpsActive(bool on)
{
  sodaq_wdt_reset();

  if (on) {
    ublox.enable();
    digitalWrite(GPS_ENABLE, HIGH);
    ublox.flush();

    sodaq_wdt_safe_delay(80);

    PortConfigurationDDC pcd;

    uint8_t maxRetries = 6;
    int8_t retriesLeft;

    retriesLeft = maxRetries;
    while (!ublox.getPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
      debugPrintln("Retrying ublox.getPortConfigurationDDC(&pcd)...");
      sodaq_wdt_safe_delay(15);
    }
    if (retriesLeft == -1) {
      debugPrintln("ublox.getPortConfigurationDDC(&pcd) failed!");
      return;
    }

    pcd.outProtoMask = 1; // Disable NMEA
    retriesLeft = maxRetries;
    while (!ublox.setPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
      debugPrintln("Retrying ublox.setPortConfigurationDDC(&pcd)...");
      sodaq_wdt_safe_delay(15);
    }
    if (retriesLeft == -1) {
      debugPrintln("ublox.setPortConfigurationDDC(&pcd) failed!");
      return;
    }

    ublox.CfgMsg(UBX_NAV_PVT, 1); // Navigation Position Velocity TimeSolution
    ublox.funcNavPvt = delegateNavPvt;
  } else {
    ublox.disable();
    digitalWrite(GPS_ENABLE, LOW);
  }
}

void BaseMaker::showUploadStatus(bool result, bool needBlink)
{
  debugPrintln(result ? "Upload Successful" : "Upload Failed");

  if (needBlink) {
    setLedColor(result ? GREEN : RED);
    sodaq_wdt_safe_delay(1000);
    setLedColor(NONE);
  }

  debugPrintln();
}

void BaseMaker::systemSleep()
{
  // go to sleep, unless USB is used for debugging
  if (params.getIsDebugOn() && ((long)&DEBUG_STREAM == (long)&SerialUSB)) { return; }

  MODEM_STREAM.flush();

  setLedColor(NONE);
  setGpsActive(false); // explicitly disable after resetting the pins

  noInterrupts();
  if (!(sodaq_wdt_flag || minuteFlag)) {
    interrupts();
    __WFI(); // SAMD sleep
  }
  interrupts();
}


/*******************************************************************************
 * Private static methods
 ******************************************************************************/

/**
*  Checks validity of data, adds valid points to the points list, syncs the RTC
*/
void BaseMaker::delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt)
{
  sodaq_wdt_reset();

  if (!isGpsInitialized) { return; }

  // note: db_printf gets enabled/disabled according to the "DEBUG" define (ublox.cpp)
  ublox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\r\n",
    NavPvt->year, NavPvt->month, NavPvt->day,
    NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
    NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

  // sync the RTC time
  if ((NavPvt->valid & GPS_TIME_VALIDITY) == GPS_TIME_VALIDITY) {
    uint32_t epoch = time.mktime(NavPvt->year, NavPvt->month, NavPvt->day, NavPvt->hour, NavPvt->minute, NavPvt->seconds);

    // check if there is an actual offset before setting the RTC
    if (abs((int64_t)getNow() - (int64_t)epoch) > MAX_RTC_EPOCH_OFFSET) { setNow(epoch); }
  }

  // check that the fix is OK and that it is a 3d fix or GNSS + dead reckoning combined
  if (((NavPvt->flags & GPS_FIX_FLAGS) == GPS_FIX_FLAGS) && ((NavPvt->fixType == 3) || (NavPvt->fixType == 4))) {
    tmpPosition.Lat = NavPvt->lat;
    tmpPosition.Lon = NavPvt->lon;
    tmpPosition.Alt = (int16_t)constrain(NavPvt->hMSL / 1000, INT16_MIN, INT16_MAX); // mm to m
    tmpPosition.NumberOfSatellites = NavPvt->numSV;

    isTmpPositionAltered = true;
  }
}

/**
 * Returns the current datetime (seconds since unix epoch).
 */
uint32_t BaseMaker::getNow() { return rtc.getEpoch(); }

/**
 * Initializes the RTC.
 */
void BaseMaker::initRtc()
{
  void rtcAlarmHandler();

  rtc.begin();

  // Schedule the wakeup interrupt for every minute
  // Alarm is triggered 1 cycle after match
  rtc.setAlarmSeconds(59);
  rtc.enableAlarm(RTCZero::MATCH_SS);   // alarm every minute
  rtc.attachInterrupt(rtcAlarmHandler); // attach handler

  // This sets it to 2000-01-01
  rtc.setEpoch(0);
}

void BaseMaker::initSleep()
{
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

void BaseMaker::saveAccelerometerPosition()
{
  accelX = accelerometer.getX();
  accelY = accelerometer.getY();
  accelZ = accelerometer.getZ();
}

/**
* Sets the RTC epoch and "rtcEpochDelta".
*/
void BaseMaker::setNow(uint32_t newEpoch)
{
  uint32_t currentEpoch = getNow();

  rtcEpochDelta = newEpoch - currentEpoch;
  rtc.setEpoch(newEpoch);

  timer.adjust(currentEpoch, newEpoch);
}


/*******************************************************************************
 * Handlers
 ******************************************************************************/

/**
 * Runs every time acceleration is over the limits
 * set by the user (if enabled).
 */
void accelerometerInt1Handler()
{
  if (digitalRead(ACCEL_INT1)) { updateOnTheMoveTimestampFlag = true; }
}

void runSensorReadEvent(uint32_t now) { BaseMaker::pendingSensorRead = true; }

/**
* Runs every minute by the rtc alarm.
*/
void rtcAlarmHandler() { minuteFlag = true; }
