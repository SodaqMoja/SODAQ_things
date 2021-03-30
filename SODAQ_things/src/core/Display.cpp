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

#include "../lib/SeeedOLED.h"
#include "../Globals.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/

#define SCREEN_HORIZONTAL_CHARS_COUNT 16


/*******************************************************************************
 * State
 ******************************************************************************/

static uint8_t cursorX = 0;
static uint8_t cursorY = 0;

static LogMessageType lastLogMessageType = UnknownMessageType;

const char* ErrorHeader = "* Error *";


/*******************************************************************************
 * Public methods
 ******************************************************************************/

void Display::clear()
{
  if (!ENABLE_DISPLAY) { return; }

  SeeedOled.clearDisplay();
  cursorX = 0;
  cursorY = 0;
}

void Display::init()
{
  if (!ENABLE_DISPLAY) { return; }

  SeeedOled.init();
  clear(); // clear the screen and set start position to top left corner
}

void Display::printBitmap(unsigned char* bitmap, uint16_t size)
{
  if (!ENABLE_DISPLAY) { return; }

  SeeedOled.drawBitmap(bitmap, size);
}

void Display::printFooter(const char* message)
{
  if (!ENABLE_DISPLAY) { return; }

  const char* emptyString = "                ";

  cursorX = 6;
  cursorY = 0;
  SeeedOled.setTextXY(cursorX, cursorY);
  SeeedOled.putString(emptyString);

  cursorX = 6;
  cursorY = (SCREEN_HORIZONTAL_CHARS_COUNT - strlen(message)) / 2;
  SeeedOled.setTextXY(cursorX, cursorY);
  SeeedOled.putString(message);
}

void Display::printLine(const char* message, LogMessageType messageType)
{
  if (!ENABLE_DISPLAY) { return; }

  // Note regarding X,Y: Seeed Studio seems to assume screen in portrait orientation
  if (messageType == NormalMessageType) {
    if ((lastLogMessageType != NormalMessageType)) {
      clear();
      SeeedOled.setNormalDisplay();
      SeeedOled.setPageMode();
    }

    cursorY = 0;

    SeeedOled.setTextXY(cursorX, cursorY);
    SeeedOled.putString(message);

    cursorX += 1; // leave 1 line empty before any other messages
  } else if (messageType == ErrorMessageType) {
    if ((lastLogMessageType != ErrorMessageType)) {
      SeeedOled.clearDisplay();
      SeeedOled.setInverseDisplay();
      SeeedOled.setPageMode();

      // skip a line before writing the header
      cursorX = 1;
      cursorY = (SCREEN_HORIZONTAL_CHARS_COUNT - strlen(ErrorHeader)) / 2;

      // write the header
      SeeedOled.setTextXY(cursorX, cursorY);
      SeeedOled.putString(ErrorHeader);

      cursorX += 2; // skip 2 lines after header
    }

    cursorY = (SCREEN_HORIZONTAL_CHARS_COUNT - strlen(message)) / 2; // write to center

    SeeedOled.setTextXY(cursorX, cursorY);
    SeeedOled.putString(message);

    cursorX += 1; // leave 2 lines empty before any other messages
  }

  lastLogMessageType = messageType;
}

void Display::printSensorValue(const char* header, const char* sensorValue, const char* gpsLat, const char* gpsLon)
{
  if (!ENABLE_DISPLAY) { return; }

  SeeedOled.clearDisplay();
  SeeedOled.setInverseDisplay();
  SeeedOled.setPageMode();

  // skip a line before writing the header
  cursorX = 1;
  cursorY = (SCREEN_HORIZONTAL_CHARS_COUNT - strlen(header)) / 2;

  // write the header
  SeeedOled.setTextXY(cursorX, cursorY);
  SeeedOled.putString(header);

  cursorX += 1;
  cursorY = (SCREEN_HORIZONTAL_CHARS_COUNT - strlen(sensorValue)) / 2;

  SeeedOled.setTextXY(cursorX, cursorY);
  SeeedOled.putString(sensorValue);

  if (gpsLat) {
      cursorX += 1;
      cursorY = (SCREEN_HORIZONTAL_CHARS_COUNT - strlen(gpsLat)) / 2;

      SeeedOled.setTextXY(cursorX, cursorY);
      SeeedOled.putString(gpsLat);
  }

  if (gpsLon) {
      cursorX += 1;
      cursorY = (SCREEN_HORIZONTAL_CHARS_COUNT - strlen(gpsLon)) / 2;

      SeeedOled.setTextXY(cursorX, cursorY);
      SeeedOled.putString(gpsLon);
  }

  lastLogMessageType = FullScreenMessageType;
}
