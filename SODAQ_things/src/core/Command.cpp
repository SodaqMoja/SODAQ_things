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

#include "Command.h"
#include "Common.h"


/*******************************************************************************
 * Public methods
 ******************************************************************************/

bool Command::execCommand(const Command args[], uint8_t nr, const char *line)
{
  int ix = findCommand(args, nr, line);
  if (ix >= 0) {
    const Command *s = &args[ix];
    if (s->exec_func) {
      return s->exec_func(s, line + strlen(s->cmd_prefix));
    }
  }
  return false;
}

int Command::findCommand(const Command args[], uint8_t nr, const char *line)
{
  for (uint8_t i = 0; i < nr; i++) {
    const Command *a = &args[i];
    if (strncasecmp(line, a->cmd_prefix, strlen(a->cmd_prefix)) == 0) {
      return i;
    }
  }
  return -1;
}

bool Command::set_int8(const Command *a, const char *line)
{
  int8_t *ptr = (int8_t *)a->value;
  if (ptr) {
    int8_t i = strtol(line, NULL, 0);
    if (*ptr != i) {
      *ptr = i;
      return true;
    }
  }
  return false;
}

bool Command::set_string(const Command *a, const char *line)
{
  char *ptr = (char *)a->value;
  if (ptr && strncmp(ptr, line, a->param_size) != 0) {
    strncpy(ptr, line, a->param_size);
    ptr[a->param_size - 1] = '\0';
    return true;
  }
  return false;
}

bool Command::set_uint8(const Command *a, const char *line)
{
  uint8_t *ptr = (uint8_t *)a->value;
  if (ptr) {
    uint8_t i = strtoul(line, NULL, 0);
    if (*ptr != i) {
      *ptr = i;
      return true;
    }
  }
  return false;
}

bool Command::set_uint16(const Command *a, const char *line)
{
  uint16_t *ptr = (uint16_t *)a->value;
  if (ptr) {
    uint16_t i = strtoul(line, NULL, 0);
    if (*ptr != i) {
      *ptr = i;
      return true;
    }
  }
  return false;
}

bool Command::set_uint32(const Command *a, const char *line)
{
  uint32_t *ptr = (uint32_t *)a->value;
  if (ptr) {
    uint32_t i = strtoul(line, NULL, 0);
    if (*ptr != i) {
      *ptr = i;
      return true;
    }
  }
  return false;
}

void Command::show_int8(const Command *a)
{
  int8_t *ptr = (int8_t *)a->value;
  if (ptr) {
    show_name(a);
    CONSOLE_STREAM.println(*ptr);
  }
}

void Command::show_string(const Command *a)
{
  char *ptr = (char *)a->value;
  show_name(a);
  if (ptr) {
    CONSOLE_STREAM.println(ptr);
  } else {
    CONSOLE_STREAM.println();
  }
}

void Command::show_text(const Command *a)
{
  CONSOLE_STREAM.print("  ");
  CONSOLE_STREAM.println(a->name);
}

void Command::show_title(const Command *a)
{
  CONSOLE_STREAM.println();
  CONSOLE_STREAM.println(a->name);
}

void Command::show_uint8(const Command *a)
{
  uint8_t *ptr = (uint8_t *)a->value;
  if (ptr) {
    show_name(a);
    CONSOLE_STREAM.println(*ptr);
  }
}

void Command::show_uint16(const Command *a)
{
  uint16_t *ptr = (uint16_t *)a->value;
  if (ptr) {
    show_name(a);
    CONSOLE_STREAM.println(*ptr);
  }
}

void Command::show_uint32(const Command *a)
{
  uint32_t *ptr = (uint32_t *)a->value;
  if (ptr) {
    show_name(a);
    CONSOLE_STREAM.println(*ptr);
  }
}


/*******************************************************************************
 * Private methods
 ******************************************************************************/

void Command::show_name(const Command *a)
{
  CONSOLE_STREAM.print("  ");
  CONSOLE_STREAM.print(a->name);

  if (a->cmd_prefix) {
    CONSOLE_STREAM.print(" (");
    CONSOLE_STREAM.print(a->cmd_prefix);
    CONSOLE_STREAM.print("): ");
  } else {
    CONSOLE_STREAM.print("       : ");
  }
}
