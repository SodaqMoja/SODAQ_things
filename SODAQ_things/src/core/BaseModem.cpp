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

#include "../lib/LedColor.h"
#include "../lib/Sodaq_nbIOT.h"
#include "../lib/Sodaq_wdt.h"
#include "../Globals.h"


/*******************************************************************************
 * State
 ******************************************************************************/

static Sodaq_nbIOT nbiot;


/*******************************************************************************
 * Public methods
 ******************************************************************************/

BaseModem::BaseModem(bool isR4XX) : isR4XX(isR4XX) {}

bool BaseModem::handleAtCommand(char *buffer)
{
  uint32_t timeout;

  int i = atoi(buffer);
  if (i > 0) { putAtCommand(buffer, i); }

  if (strncasecmp(buffer, "AT", 2) == 0) {
    if (strncasecmp(buffer, "AT+COPS=", 8) == 0) {
      CONSOLE_STREAM.println("Initializing module, this may take 3 minutes...");
      timeout = COPS_TIMEOUT;
    } else {
      timeout = MODEM_DEFAULT_TIMEOUT;
    }

    execCommand(buffer, timeout);

    return false;
  }

  return true;
}

void BaseModem::initModem()
{
  isConnected = false;

  nbiot.setInputBufferSize(1024);

  if (isR4XX) {
    MODEM_STREAM.begin(nbiot.getSaraR4Baudrate());
    nbiot.init(MODEM_STREAM, SARA_ENABLE, SARA_TX_ENABLE, SARA_R4XX_TOGGLE, DEFAULT_CID);
  } else {
    MODEM_STREAM.begin(nbiot.getSaraN2Baudrate());
    nbiot.init(MODEM_STREAM, SARA_ENABLE, SARA_TX_ENABLE, -1, DEFAULT_CID);
    nbiot.overrideNconfigParam("CR_0354_0338_SCRAMBLING", true);
  }
}

void BaseModem::readIMEI()
{
  if (nbiot.getIMEI(imei, sizeof(imei))) {
    cachedImei = atoll(imei);
    return;
  }

  imei[0] = '\0';
  cachedImei = 0;
  CONSOLE_STREAM.println("Failed to get IMEI!");
}

void BaseModem::showMenuInfo()
{
  enableDiag();

  CONSOLE_STREAM.println("\r\nAT Command Mode\r\n");

  char buffer[256];

  for (int i = 1; i <= 100; i++) {
    if (!putAtCommand(buffer, i)) { break; }

    CONSOLE_STREAM.print(i);
    CONSOLE_STREAM.print(i <= 9 ? "   " : "  ");
    CONSOLE_STREAM.println(buffer);
  }

  CONSOLE_STREAM.println("\r\nYou can use one of the above numbers, or type 'exit' for quit AT Command Mode\r\n");
}

bool BaseModem::transmit(const uint8_t* buffer, uint8_t size)
{
  sodaq_wdt_reset();

  enableDiag();

  isConnected = isConnected && nbiot.isConnected();

  if (!isConnected) {
    isConnected = connect();
    if (!isConnected) {
      off();
      sodaq_wdt_safe_delay(450);
      on();
      sodaq_wdt_safe_delay(450);
      isConnected = connect();
    }
  }

  if (!isConnected) { return false; }

  return params.getIsUDP() ? transmitUDP(buffer, size) : transmitCOAP(buffer, size);
}


/*******************************************************************************
 * Public static methods
 ******************************************************************************/

void BaseModem::disableDiag() { nbiot.setDiag(NULL); }

void BaseModem::enableDiag() { nbiot.setDiag(DEBUG_STREAM); }

void BaseModem::off() { nbiot.off(); }

void BaseModem::on()
{
  nbiot.on();
  nbiot.purgeAllResponsesRead();
}


/*******************************************************************************
 * Protected methods
 ******************************************************************************/

bool BaseModem::execCommand(const char* command, uint32_t timeout, char* buffer, size_t size)
{
  bool b = nbiot.execCommand(command, timeout, buffer, size);

  if (!b && buffer != NULL && size > 0) { buffer[0] = 0; }

  while (MODEM_STREAM.available()) { CONSOLE_STREAM.print(MODEM_STREAM.read()); }

  return b;
}


/*******************************************************************************
 * Private methods
 ******************************************************************************/

bool BaseModem::connect()
{
  debugPrintln("Connecting...");

  setLedColor(BLUE);

  bool b = isR4XX ? connectR4XX() : connectN211();

  if (b) {
    debugPrintln("Connected!");
  } else {
    uint8_t simStatus = isR4XX ? nbiot.getSimStatus() : 0;

    if (simStatus == 1) {
      debugPrintln("Please insert the SIM Card and restart the device");
      display.printLine("Please insert", ErrorMessageType);
      display.printLine("the SIM Card",  ErrorMessageType);
      display.printLine("and restart",   ErrorMessageType);
      display.printLine("the device",    ErrorMessageType);
    } else if (simStatus == 2) {
      debugPrintln("Please configure the SIM pin");
      display.printLine("Please configure", ErrorMessageType);
      display.printLine("the SIM pin",      ErrorMessageType);
    } else {
      debugPrintln("Could not connect to the network");
      display.printLine("Could not",   ErrorMessageType);
      display.printLine("connect to",  ErrorMessageType);
      display.printLine("the network", ErrorMessageType);
    }

    setLedColor(RED);
    sodaq_wdt_safe_delay(5000);
  }

  setLedColor(NONE);

  return b;
}

bool BaseModem::transmitCOAP(const uint8_t* buffer, uint8_t size)
{
  debugPrintln("Sending message through COAP...");

  return nbiot.setIndicationsActive(true) && nbiot.sendMessage(buffer, size);
}

bool BaseModem::transmitUDP(const uint8_t* buffer, uint8_t size)
{
  debugPrintln("Sending message through UDP...");

  int socketID = isR4XX ? nbiot.createSocket() : nbiot.createSocket(UDP_LOCALPORT);

  if (socketID < 0) {
    debugPrintln("Error: failed to create socket!");
    return false;
  }

  size_t lengthSent = nbiot.socketSend(socketID, params.getEndpointUrl(), params.getEndpointPort(), buffer, size);
  debugPrint("Sent ");
  debugPrint(lengthSent);
  debugPrintln(" bytes");

  nbiot.closeSocket(socketID);

  return lengthSent == size;
}


/*******************************************************************************
 * Private methods - N211
 ******************************************************************************/

bool BaseModem::connectN211()
{
  const char *cdp = params.getIsUDP() ? NULL : params.getEndpointUrl();

  return nbiot.connect(params.getApn(), cdp, params.getForceOperator(), params.getBand());
}


/*******************************************************************************
 * Private methods - R4XX
 ******************************************************************************/

bool BaseModem::attach()
{
  uint32_t tm = millis();

  if (!nbiot.attachGprs(ATTACH_TIMEOUT)) { return false; }

  if (abs(millis() - tm) > ATTACH_NEED_REBOOT) {
    if (!execCommand("AT+CFUN=15", ATTACH_TIMEOUT)) { return false; }
    sodaq_wdt_safe_delay(REBOOT_DELAY);
  }

  return true;
}

int8_t BaseModem::checkApn()
{
  char buffer[256];

  if (!execCommand("AT+CGDCONT?", SODAQ_AT_DEVICE_DEFAULT_READ_MS, buffer, sizeof(buffer))) { return -1; }

  if (strncmp(buffer, "+CGDCONT: 1,\"IP\"", 16) == 0 && strncmp(buffer + 16, ",\"\"", 3) != 0) {
    char apn[64];
    char ip[32];

    if (sscanf(buffer + 16, ",\"%[^\"]\",\"%[^\"]\",0,0,0,0", apn, ip) != 2) { return -1; }

    if (strlen(ip) >= 7 && strcmp(ip, "0.0.0.0") != 0) { return 1; }

    if (strcmp(apn, params.getApn()) == 0) { return 0; }
  }

  return nbiot.setApn(params.getApn()) ? 0 : 1;
}

bool BaseModem::checkCFUN()
{
  char buffer[64];

  if (!execCommand("AT+CFUN?", SODAQ_AT_DEVICE_DEFAULT_READ_MS, buffer, sizeof(buffer))) { return false; }

  if (strncmp(buffer, "+CFUN: 1", 8) == 0) { return true; }

  return execCommand("AT+FUN=1");
}

bool BaseModem::checkCOPS()
{
  char buffer[1024];

  if (!execCommand("AT+COPS?", COPS_TIMEOUT, buffer, sizeof(buffer))) { return false; }

  if (strncmp(buffer, "+COPS: 0", 8) == 0) { return true; }

  return execCommand("AT+COPS=0", COPS_TIMEOUT);
}

bool BaseModem::checkBandMask()
{
  char buffer[256];

  if (!execCommand("AT+UBANDMASK?", SODAQ_AT_DEVICE_DEFAULT_READ_MS, buffer, sizeof(buffer))) { return false; }

  char bm0[32];
  char bm1[32];
  if (sscanf(buffer, "+UBANDMASK: 0,%[^,],1,%s", bm0, bm1) != 2) { return false; }

  if (strcmp(bm1, params.getBandMask()) == 0) { return true; }

  sprintf(buffer, "AT+UBANDMASK=1,%s", params.getBandMask());

  return execCommand(buffer);
}

bool BaseModem::checkUrat()
{
  char buffer[256];

  if (!execCommand("AT+URAT?", SODAQ_AT_DEVICE_DEFAULT_READ_MS, buffer, sizeof(buffer))) { return false; }

  int urat;
  if (sscanf(buffer, "+URAT: %d", &urat) != 1 || urat == 0) { return false; }

  if (urat == params.getUrat()) { return true; }

  sprintf(buffer, "AT+URAT=%d", params.getUrat());

  return execCommand(buffer);
}

bool BaseModem::connectR4XX()
{
  on();

  if (!nbiot.setVerboseErrors(true)) { return false; }

  if (!execCommand("ATE0")) { return false; }

  if (!checkCFUN()) { return false; }

  if (!checkCOPS()) { return false; }

  if (!checkUrat()) { return false; }

  if (!checkBandMask()) { return false; }

  int8_t i = checkApn();
  if (i < 0) { return false; }

  if (i == 0 && !attach() && !attach()) { return false; } // two tries

  return execCommand("AT+UDCONF=1,1");
}
