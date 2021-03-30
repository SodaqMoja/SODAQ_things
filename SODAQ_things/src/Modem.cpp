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

#include "Globals.h"
#include "lib/Sodaq_wdt.h"


/*******************************************************************************
 * Public methods
 ******************************************************************************/

Modem::Modem() : BaseModem(true) {}

void Modem::executeAtInit()
{
  uint32_t timeout;

  enableDiag();
  CONSOLE_STREAM.println();

  char buffer[256];

  for (int i = 1; i <= 100; i++) {
    if (!putAtInit(buffer, i)) { break; }

    if (strncasecmp(buffer, "AT+COPS=", 8) == 0) {
      CONSOLE_STREAM.println("Initializing module, this may take 3 minutes...");
      timeout = COPS_TIMEOUT;
    } else {
      timeout = MODEM_DEFAULT_TIMEOUT;
    }

    execCommand(buffer, timeout);
  }

  CONSOLE_STREAM.println("Finished...\r\n");

  sodaq_wdt_safe_delay(2000);
}


/*******************************************************************************
 * Protected methods
 ******************************************************************************/

bool Modem::putAtCommand(char* buffer, uint16_t index)
{
  switch (index) {
    case 1:
      strcpy(buffer, "ATE0");
      return true;
    case 2:
      sprintf(buffer, "AT+URAT=%d", params.getUrat());
      return true;
    case 3:
      strcpy(buffer, "AT+CCID");
      return true;
    case 4:
      strcpy(buffer, "AT+CGSN");
      return true;
    case 5:
      strcpy(buffer, "AT+CIMI");
      return true;
    case 6:
      strcpy(buffer, "AT+CGMR");
      return true;
    case 7:
      strcpy(buffer, "AT+UBANDMASK?");
      return true;
    case 8:
      strcpy(buffer, "AT+UMNOPROF?");
      return true;
    case 9:
      strcpy(buffer, "AT+URAT?");
      return true;
    case 10:
      strcpy(buffer, "AT+CFUN=0");
      return true;
    case 11:
      sprintf(buffer, "AT+CGDCONT=1,\"IP\",\"%s\"", params.getApn());
      return true;
    case 12:
      strcpy(buffer, "AT+CFUN=1");
      return true;
    case 13:
      strcpy(buffer, "AT+CGDCONT?");
      return true;
    case 14:
      strcpy(buffer, "AT+COPS=0");
      return true;
    case 15:
      strcpy(buffer, "AT+CSQ");
      return true;
    case 16:
      strcpy(buffer, "AT+CREG=2");
      return true;
    case 17:
      strcpy(buffer, "AT+CREG?");
      return true;
    case 18:
      strcpy(buffer, "AT+CGPADDR");
      return true;
    case 19:
      strcpy(buffer, "AT+UDCONF=1,0");
      return true;
    case 20:
      strcpy(buffer, "AT+USOCR=17");
      return true;
    case 21:
      sprintf(buffer, "AT+USOST=0,\"%s\",%d,4,\"Data\"", params.getEndpointUrl(), params.getEndpointPort());
      return true;
    case 22:
      strcpy(buffer, "AT+UDCONF=1,1");
      return true;
    case 23:
      sprintf(buffer, "AT+USOST=0,\"%s\",%d,4,\"44617461\"", params.getEndpointUrl(), params.getEndpointPort());
      return true;
    case 24:
      strcpy(buffer, "AT+USOCL=0");
      return true;
    case 25:
      strcpy(buffer, "AT+URAT=7");
      return true;
    case 26:
      strcpy(buffer, "AT+CFUN=15");
      return true;
  }

  return false;
}


/*******************************************************************************
 * Private methods
 ******************************************************************************/

bool Modem::putAtInit(char* buffer, uint16_t index)
{
  switch (index) {
    case 1:
      strcpy(buffer, "ATE0");
      return true;
    case 2:
      sprintf(buffer, "AT+URAT=%d", params.getUrat());
      return true;
    case 3:
      strcpy(buffer, "AT+CFUN=0");
      return true;
    case 4:
      sprintf(buffer, "AT+CGDCONT=1,\"IP\",\"%s\"", params.getApn());
      return true;
    case 5:
      strcpy(buffer, "AT+CFUN=1");
      return true;
    case 6:
      strcpy(buffer, "AT+COPS=0");
      return true;
    case 7:
      strcpy(buffer, "AT+CSQ");
      return true;
    case 8:
      strcpy(buffer, "AT+CFUN=15");
      return true;
  }

  return false;
}
