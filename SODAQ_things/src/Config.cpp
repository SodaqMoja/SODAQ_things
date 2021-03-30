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


/*******************************************************************************
 * Public methods
 ******************************************************************************/

Config::Config(Command *args, uint16_t argsLength) : BaseConfig(args, argsLength) {}

bool Config::isValid()
{
  bool fail = !BaseConfig::isValid();

  if (params._urat < 7 || params._urat > 8) {
    CONSOLE_STREAM.println("Network code must be 7 or 8");
    fail = true;
  }

  return !fail;
}

void Config::reset()
{
  BaseConfig::reset();

  params._temperatureHigh = DEFAULT_TEMPERATURE_HIGH;
  params._temperatureLow  = DEFAULT_TEMPERATURE_LOW;

  resetConfigNetwork();
}


/*******************************************************************************
 * Private static methods
 ******************************************************************************/

void Config::resetConfigNetwork()
{
  BUILD_BUG_ON(sizeof(VFN_APN)              > sizeof(params._apn));
  BUILD_BUG_ON(sizeof(VFN_BANDMASK)         > sizeof(params._bandMask));
  BUILD_BUG_ON(sizeof(VFN_FORCE_OPERATOR)   > sizeof(params._forceOperator));
  BUILD_BUG_ON(sizeof(VFN_UDP_ENDPOINT_URL) > sizeof(params._endpointUrl));

  strcpy(params._apn,           VFN_APN);
  strcpy(params._bandMask,      VFN_BANDMASK);
  strcpy(params._endpointUrl,   VFN_UDP_ENDPOINT_URL);
  strcpy(params._forceOperator, VFN_FORCE_OPERATOR);

  params._endpointPort = VFN_UDP_ENDPOINT_PORT;
  params._urat         = VFN_URAT;
}
