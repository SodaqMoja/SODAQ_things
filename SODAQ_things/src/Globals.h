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

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "core/Common.h"
#include "core/Display.h"
#include "BootMenu.h"
#include "Config.h"
#include "Params.h"
#include "Maker.h"
#include "Modem.h"

#define PROJECT_VERSION           "SODAQ things 20201008.1"
#define ENABLE_DEEP_SLEEP         0
#define ENABLE_DISPLAY            1

#define VFN_APN                   "iot.t-mobile.nl"
#define VFN_BANDMASK              "524420"
#define VFN_FORCE_OPERATOR        "0"
#define VFN_UDP_ENDPOINT_PORT     8891
#define VFN_UDP_ENDPOINT_URL      "136.144.235.234"
#define VFN_URAT                  8

#define DEFAULT_ISGPS_ENABLED     0
#define DEFAULT_TEMPERATURE_HIGH  29
#define DEFAULT_TEMPERATURE_LOW   10

#define LIGHT_SENSOR_DELTA        10000

extern BootMenu menu;
extern Config   config;
extern Display  display;
extern Maker    maker;
extern Modem    modem;
extern Params   params;

#endif /* GLOBALS_H_ */
