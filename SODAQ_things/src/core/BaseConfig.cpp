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

#include "../lib/FlashStorage.h"
#include "../Globals.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/

#define DEFAULT_HEADER 0xBEEF


/*******************************************************************************
 * State
 ******************************************************************************/

FlashStorage(flash, Params);


/*******************************************************************************
 * Public methods
 ******************************************************************************/

BaseConfig::BaseConfig(Command *args, uint16_t argsLength) : args(args), argsLength(argsLength), needsCommit(false) {}

bool BaseConfig::commit()
{
  if (!needsCommit) { return false; }

  params._header = DEFAULT_HEADER;
  params._crc16  = getCRC16();

  flash.write(params);

  needsCommit = false;

  return true;
}

bool BaseConfig::execCommand(const char* line)
{
  if (strncasecmp(line, "show", 4) == 0) {
    int ix = Command::findCommand(args, argsLength, line + 4);
    if (ix < 0) { return false; }

    const Command *a = &args[ix];

    if (strtol(line + 4 + strlen(a->cmd_prefix), NULL, 0) == 0) {
      params._paramsHidden |= (1 << ix);
    } else {
      params._paramsHidden &= ~(1 << ix);
    }

    needsCommit = true;

    return true;
  }

  bool b = Command::execCommand(args, argsLength, line);
  if (b) { needsCommit = true; }
  return b;
}

bool BaseConfig::isValid()
{
  bool fail = false;

  if (params._accelerationPercentage > 100) {
    CONSOLE_STREAM.println("Acceleration% must be 0-100");
    fail = true;
  }

  if (params._accelerationSensitivity < 1 || params._accelerationSensitivity > 50) {
    CONSOLE_STREAM.println("Acceleration Trigger Sensitivity must be 1-50");
    fail = true;
  }

  if (params._band != 8 && params._band != 20) {
    CONSOLE_STREAM.println("NB-IoT Band must be either 8 or 20");
    fail = true;
  }



  if (params._isDebugOn > 1) {
    CONSOLE_STREAM.println("Debug must be either 0 or 1");
    fail = true;
  }

  if (params._isGpsEnabled > 1) {
    CONSOLE_STREAM.println("GPS ON/OFF must be either 0 or 1");
    fail = true;
  }

  if (params._isUDP > 1) {
    CONSOLE_STREAM.println("UDP/COAP (coap=0, udp=1) must be either 0 or 1");
    fail = true;
  }

  if (params._payloadType > 2) {
    CONSOLE_STREAM.println("Payload type must be either 0, 1 or 2");
    fail = true;
  }

  if (params._sensorType > 5) {
    CONSOLE_STREAM.println("The sensor type must be 0-5");
    fail = true;
  }

  return !fail;
}

void BaseConfig::read()
{
  flash.read(&params);

  if (params._header != DEFAULT_HEADER || params._crc16 != getCRC16()) { reset(); }

  needsCommit = false;
}

void BaseConfig::setNeedCommitFlag() { needsCommit = true; }

void BaseConfig::showConfig()
{
  CONSOLE_STREAM.println("\r\nSettings:");

  for (size_t i = 0; i < argsLength; ++i) {
    if ((params._paramsHidden & (1 << i)) == 0) {
      const Command *a = args + i;
      if (a->show_func) { a->show_func(a); }
    }
  }
}


/*******************************************************************************
 * Public virtual methods
 ******************************************************************************/

void BaseConfig::reset()
{
  params._paramsHidden            = 0;
  params._isDebugOn               = DEFAULT_ISDEBUG_ON;

  params._isGpsEnabled            = DEFAULT_ISGPS_ENABLED;
  params._gpsFixTimeout           = DEFAULT_FIX_TIMEOUT;
  params._gpsMinSatelliteCount    = DEFAULT_MIN_SATELLITE_COUNT;

  params._apn[0]                  = '\0';
  params._attToken[0]             = '\0';
  params._band                    = DEFAULT_BAND;
  params._bandMask[0]             = '\0';
  params._endpointUrl[0]          = '\0';
  params._endpointPort            = 0;
  params._forceOperator[0]        = '\0';
  params._isATTHeaderOn           = DEFAULT_ISATT_HEADER_ON;
  params._isUDP                   = DEFAULT_ISUDP_ON;
  params._payloadType             = DEFAULT_PAYLOAD_TYPE;
  params._urat                    = DEFAULT_URAT;

  params._accelerationDuration    = DEFAULT_ACCELERATION_DURATION;
  params._accelerationPercentage  = DEFAULT_ACCELERATION_PERCENTAGE;
  params._accelerationSensitivity = DEFAULT_ACCELERATION_SENSITIVITY;
  params._onTheMoveBackoffTime    = DEFAULT_ONTHEMOVE_BACKOFF_TIME;

  params._sensorType              = DEFAULT_SENSOR_TYPE;
  params._sensorReadInterval      = DEFAULT_SENSOR_READ_INTERVAL;
  params._temperatureOffset       = DEFAULT_TEMPERATURE_OFFSET;
}


/*******************************************************************************
 * Private methods
 ******************************************************************************/

uint16_t BaseConfig::getCRC16()
{
  return crc16ccitt((uint8_t*)&params._header, sizeof(Params) - sizeof(params._crc16));
}


/*******************************************************************************
 * Private static methods
 ******************************************************************************/

uint16_t BaseConfig::crc16ccitt(const uint8_t *buf, size_t len)
{
  uint16_t crc = 0;
  while (len--) {
    crc ^= (*buf++ << 8);
    for (uint8_t i = 0; i < 8; ++i) { crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1; }
  }
  return crc;
}
