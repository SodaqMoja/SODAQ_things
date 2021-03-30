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

#ifndef BASEPARAMS_H_
#define BASEPARAMS_H_

class BaseParams
{
public:
  uint16_t _crc16;
  uint16_t _header;

  uint32_t _paramsHidden;
  uint8_t  _isDebugOn;

  uint8_t  _isGpsEnabled;
  uint16_t _gpsFixTimeout;
  uint8_t  _gpsMinSatelliteCount;

  char     _apn[32 + 1];
  char     _attToken[32 + 1];
  uint8_t  _band;
  char     _bandMask[32 + 1];
  uint16_t _endpointPort;
  char     _endpointUrl[128 + 1];
  char     _forceOperator[32 + 1];
  uint8_t  _isATTHeaderOn;
  uint8_t  _isUDP;
  uint8_t  _payloadType;
  uint8_t  _urat;

  uint8_t  _accelerationDuration;
  uint8_t  _accelerationPercentage;
  uint8_t  _accelerationSensitivity;
  uint8_t  _onTheMoveBackoffTime;

  uint8_t  _sensorType;
  uint16_t _sensorReadInterval;
  int8_t   _temperatureOffset;

  uint8_t     getIsDebugOn()               const { return _isDebugOn; }

  const char* getApn()                     const { return _apn; }
  const char* getAttToken()                const { return _attToken; }
  uint8_t     getBand()                    const { return _band; }
  const char* getBandMask()                const { return _bandMask; }
  uint16_t    getEndpointPort()            const { return _endpointPort; }
  const char* getEndpointUrl()             const { return _endpointUrl; }
  const char* getForceOperator()           const { return _forceOperator; }
  uint8_t     getIsATTHeaderOn()           const { return _isATTHeaderOn; }
  uint8_t     getIsUDP()                   const { return _isUDP; }
  uint8_t     getPayloadType()             const { return _payloadType; }
  uint8_t     getUrat()                    const { return _urat; }

  uint8_t     getIsGpsEnabled()            const { return _isGpsEnabled; }
  uint16_t    getGpsFixTimeout()           const { return _gpsFixTimeout; }
  uint8_t     getGpsMinSatelliteCount()    const { return _gpsMinSatelliteCount; }

  uint8_t     getAccelerationDuration()    const { return _accelerationDuration; }
  uint8_t     getAccelerationPercentage()  const { return _accelerationPercentage; }
  uint8_t     getAccelerationSensitivity() const { return _accelerationSensitivity; }
  uint8_t     getOnTheMoveBackoffTime()    const { return _onTheMoveBackoffTime; }

  uint8_t     getSensorType()              const { return _sensorType; }
  uint16_t    getSensorReadInterval()      const { return _sensorReadInterval; }
  int8_t      getTemperatureOffset()       const { return _temperatureOffset; }
};

#endif /* BASEPARAMS_H_ */
