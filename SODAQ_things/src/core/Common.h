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

#ifndef COMMON_H_
#define COMMON_H_

#include <Arduino.h>

#define CONSOLE_STREAM                   SerialUSB
#define DEBUG_STREAM                     SerialUSB
#define MODEM_STREAM                     Serial1

#define BOOT_MENU_OLED_TIMEOUT           (15 * 1000)
#define BOOT_MENU_TIMEOUT                (30 * 1000)
#define STARTUP_DELAY                    5000
#define SWITCH_SENSOR_DELAY              5000

#define CONSOLE_BAUD                     115200
#define DEBUG_BAUD                       115200 // only used when CONSOLE is different than debug

#define DEFAULT_CID                      1
#define UDP_LOCALPORT                    16666

#define ATTACH_TIMEOUT                   180000
#define ATTACH_NEED_REBOOT               40000
#define COPS_TIMEOUT                     180000
#define MODEM_DEFAULT_TIMEOUT            5000
#define REBOOT_DELAY                     15000

#define GPS_TIME_VALIDITY                0b00000011 // date and time (but not fully resolved)
#define GPS_FIX_FLAGS                    0b00000001 // just gnssFixOK
#define GPS_COMM_CHECK_TIMEOUT           3 // seconds
#define GPS_SCALING                      (float)pow(10, 7)
#define MAX_RTC_EPOCH_OFFSET             25

// default configuration parameters
#define DEFAULT_ACCELERATION_DURATION    0
#define DEFAULT_ACCELERATION_PERCENTAGE  25
#define DEFAULT_ACCELERATION_SENSITIVITY 10
#define DEFAULT_BAND                     8
#define DEFAULT_FIX_TIMEOUT              120
#define DEFAULT_ISDEBUG_ON               1
#define DEFAULT_ISATT_HEADER_ON          1
#define DEFAULT_ISUDP_ON                 1
#define DEFAULT_MIN_SATELLITE_COUNT      6
#define DEFAULT_ONTHEMOVE_BACKOFF_TIME   10
#define DEFAULT_PAYLOAD_TYPE             0
#define DEFAULT_SENSOR_READ_INTERVAL     5
#define DEFAULT_SENSOR_TYPE              0
#define DEFAULT_TEMPERATURE_OFFSET       0
#define DEFAULT_URAT                     8

// debug mode macro
#define debugPrint(x)   if (params.getIsDebugOn()) { DEBUG_STREAM.print(x);   }
#define debugPrintln(x) if (params.getIsDebugOn()) { DEBUG_STREAM.println(x); }

// macro to do compile time sanity checks / assertions
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

#endif /* COMMON_H_ */
