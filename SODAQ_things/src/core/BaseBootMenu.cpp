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

#include "../Globals.h"


/*******************************************************************************
 * State
 ******************************************************************************/

bool BaseBootMenu::isCommandMode = false;


/*******************************************************************************
 * Public methods
 ******************************************************************************/

BaseBootMenu::BaseBootMenu(Command *args, uint16_t argsLength) : args(args), argsLength(argsLength) {}

void BaseBootMenu::showBootMenu()
{
  char buffer[200 + 1];

  uint32_t ts_max    = getExtendedTimeout();
  uint8_t  nrPrompts = 0;

  bool needPrompt = true;

  while (true) {
    if (isTimedOut(ts_max) || nrPrompts >= 250) {
      if (!isCommandMode) { break; }
      needPrompt = true;
    }

    if (needPrompt && isCommandMode) {
      modem.showMenuInfo();
      ts_max = getExtendedTimeout();
      needPrompt = false;
      continue;
    }

    if (checkCommand()) {
      ts_max = getExtendedTimeout();
      needPrompt = false;
      continue;
    }

    if (!isCommandMode && maker.checkAccelerometer()) {
      ts_max = getExtendedTimeout();
      needPrompt = true;
    }

    if (needPrompt) {
      showCommandPrompt();
      needPrompt = false;
      nrPrompts++;
    }

    // ignore initial NUL bytes on input
    while (CONSOLE_STREAM.available() && CONSOLE_STREAM.peek() == 0) { CONSOLE_STREAM.read(); }

    int size = -1;
    if (CONSOLE_STREAM.available()) { size = readLine(buffer, sizeof(buffer), ts_max); }

    if (size < 0) { continue; }

    needPrompt = true;
    if (size == 0) { continue; }

    if (isCommandMode) {
      if (strcasecmp(buffer, "exit") == 0) {
        isCommandMode = false;
        continue;
      }

      needPrompt = modem.handleAtCommand(buffer);
      continue;
    }

    if (strcasecmp(buffer, "ok") == 0) { break; }

    // Is this a command for us?
    bool seenCommand = Command::execCommand(args, argsLength, buffer);

    // Is this a command for config?
    if (!seenCommand && config.execCommand(buffer)) {
      seenCommand = true;
      CONSOLE_STREAM.println("Configuration changed");
    }

    if (seenCommand) { ts_max = getExtendedTimeout(); }
  }

  CONSOLE_STREAM.println();
}

void BaseBootMenu::showBootMenuOled()
{
  uint32_t ts_max = millis() + BOOT_MENU_OLED_TIMEOUT;
  uint8_t nrPrompts = 0;

  while (!isTimedOut(ts_max) && nrPrompts < 100) {
    if (maker.checkAccelerometer()) {
      ts_max = millis() + BOOT_MENU_OLED_TIMEOUT;
      nrPrompts++;
    }
  }
}


/*******************************************************************************
 * Public static methods
 ******************************************************************************/

bool BaseBootMenu::setCommandMode(const Command *a, const char *line)
{
  isCommandMode = true;
  return false;
}


/*******************************************************************************
 * Private methods
 ******************************************************************************/

void BaseBootMenu::showCommandPrompt()
{
  showCommands();
  config.showConfig();
  CONSOLE_STREAM.print("\r\nEnter command: ");
}

void BaseBootMenu::showCommands()
{
  if (argsLength == 0) { return; }

  CONSOLE_STREAM.println("\r\nCommands:");

  for (size_t i = 0; i < argsLength; ++i) {
    const Command* a = &args[i];
    if (a->show_func) { a->show_func(a); }
  }
}


/*******************************************************************************
 * Private static methods
 ******************************************************************************/

uint32_t BaseBootMenu::getExtendedTimeout() { return millis() + BOOT_MENU_TIMEOUT; }

bool BaseBootMenu::isTimedOut(uint32_t ts) { return (long)(millis() - ts) >= 0; }

int BaseBootMenu::readLine(char line[], size_t size, uint32_t &ts_max)
{
  int c;
  size_t len = 0;
  bool seenCR = false;
  uint32_t ts_waitLF = 0;
  while (!isTimedOut(ts_max)) {
    if (seenCR) {
      c = CONSOLE_STREAM.peek();
      // ts_waitLF is guaranteed to be non-zero
      if ((c == -1 && isTimedOut(ts_waitLF)) || (c != -1 && c != '\n')) {
        goto end;
      }
      // Only \n should fall through
    }

    c = CONSOLE_STREAM.read();
    // Ignore NUL bytes too
    if (c <= 0) { continue; }

    // There is input, so extend the timeout
    ts_max = getExtendedTimeout();

    seenCR = c == '\r';
    if (c == '\r') {
      CONSOLE_STREAM.write((char)c);
      ts_waitLF = millis() + 50; // Wait another .05 sec for an optional LF
    }
    else if (c == '\n') {
      CONSOLE_STREAM.write((char)c);
      goto end;
    }
    else if (c == 0x08 || c == 0x7f) {
        // Erase the last character
      if (len > 0) {
        CONSOLE_STREAM.write("\b \b");
        --len;
      }
    }
    else {
      // Any "normal" character is stored in the line buffer
      if (len < size - 1 && c >= ' ' && c < 0x7f) {
        CONSOLE_STREAM.write((char)c);
        line[len++] = c;
      }
    }
  }
  // Timed out. Ignore the input.
  line[0] = '\0';
  return -1;

end:
  line[len] = '\0';
  return len;
}
