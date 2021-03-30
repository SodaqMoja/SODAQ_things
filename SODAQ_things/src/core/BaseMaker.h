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

#ifndef BASEMAKER_H
#define BASEMAKER_H

#include "../lib/Sodaq_LSM303AGR.h"
#include "../lib/ublox.h"

struct Position
{
  uint32_t Timestamp;
  int32_t  Lat;
  int32_t  Lon;
  int16_t  Alt;
  uint8_t  NumberOfSatellites;
  uint8_t  TimeToFix;
} __attribute__((packed));

union UdpHeader
{
  uint8_t Raw[16];
  struct
  {
    uint8_t Version;
    uint8_t Imei[7];
    char    Token[8];
  } __attribute__((packed));
};

class BaseMaker
{
public:
  static bool pendingSensorRead;

public:
  bool checkAccelerometer();

protected:
  static Sodaq_LSM303AGR accelerometer;
  static Position tmpPosition;

  static bool isGpsInitialized;
  static bool updateOnTheMoveTimestampFlag;

protected:
  void fatal();
  bool handleGpsFixSequence();
  void postLoop();
  void preLoop();
  void setupBoot();
  void setupFinal();
  bool uploadData(uint8_t* data, uint8_t size, bool needHeader, bool needBlink);

  virtual void displayState(uint8_t state);
  virtual void setState    (uint8_t state);

  static uint16_t getBatteryVoltageMV();

private:
  void checkTimer();
  void handleBootUpCommands();
  void initAccelerometer();
  bool initGps();
  void initOnTheMove();
  void initRtcTimer();
  void initUdpHeader();
  void resetRtcTimerEvents();
  void setGpsActive(bool on);
  void systemSleep();
  void showUploadStatus(bool result, bool needBlink);

  static void     delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt);
  static uint32_t getNow();
  static void     initRtc();
  static void     initSleep();
  static void     saveAccelerometerPosition();
  static void     setNow(uint32_t newEpoch);
};

#endif /* BASEMAKER_H */
