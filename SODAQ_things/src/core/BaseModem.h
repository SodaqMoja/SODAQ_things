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

#ifndef BASEMODEM_H_
#define BASEMODEM_H_

#include "Common.h"

class BaseModem
{
public:
  bool     isR4XX;
  char     imei[32];
  uint64_t cachedImei;
  bool     isConnected;

public:
  BaseModem(bool isR4XX);

  bool handleAtCommand(char *buffer);
  void initModem();
  void readIMEI();
  void showMenuInfo();
  bool transmit(const uint8_t* buffer, uint8_t size);

  static void disableDiag();
  static void enableDiag();
  static void off();
  static void on();

protected:
  bool execCommand(const char* command, uint32_t timeout = MODEM_DEFAULT_TIMEOUT,
                   char* buffer = NULL, size_t size = 0);

  virtual bool putAtCommand(char* buffer, uint16_t index) = 0;

private:
  bool   connect();
  bool   transmitCOAP(const uint8_t* buffer, uint8_t size);
  bool   transmitUDP (const uint8_t* buffer, uint8_t size);

  bool   connectN211();

  bool   attach();
  int8_t checkApn(); // -1: error, 0: ip not valid => need attach, 1: valid ip
  bool   checkCFUN();
  bool   checkCOPS();
  bool   checkBandMask();
  bool   checkUrat();
  bool   connectR4XX();
};

#endif /* BASEMODEM_H_ */
