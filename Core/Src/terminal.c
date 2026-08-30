#include "terminal.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static char command_buffer[TERMINAL_COMMAND_SIZE];
static volatile uint8_t command_ready;
static uint8_t command_length;
static uint8_t echo_enabled;
static char tx_buffer[256];

void Terminal_Init(void)
{
  command_ready = 0U;
  command_length = 0U;
  echo_enabled = 0U;
  command_buffer[0] = '\0';
}

void Terminal_RxData(const uint8_t *data, uint32_t length)
{
  uint32_t i;
  for (i = 0U; i < length; ++i)
  {
    char c = (char)data[i];
    if (command_ready) continue;
    if (c == '\r' || c == '\n')
    {
      if (command_length > 0U)
      {
        command_buffer[command_length] = '\0';
        command_ready = 1U;
      }
    }
    else if (c == '\b' || c == 127)
    {
      if (command_length > 0U) --command_length;
    }
    else if ((unsigned char)c >= 32U && (unsigned char)c <= 126U)
    {
      if (command_length < (TERMINAL_COMMAND_SIZE - 1U))
      {
        command_buffer[command_length++] = c;
      }
    }
  }
}

uint8_t Terminal_GetCommand(char *destination, uint32_t size)
{
  if (!command_ready || destination == NULL || size == 0U) return 0U;
  strncpy(destination, command_buffer, size - 1U);
  destination[size - 1U] = '\0';
  command_length = 0U;
  command_buffer[0] = '\0';
  command_ready = 0U;
  CDC_Receive_Rearm_FS();
  return 1U;
}

uint8_t Terminal_CommandPending(void) { return command_ready; }

void Terminal_Send(const char *text)
{
  uint16_t length;
  if (text == NULL) return;
  length = (uint16_t)strlen(text);
  if (length > APP_TX_DATA_SIZE) length = APP_TX_DATA_SIZE;
  (void)CDC_Transmit_FS((uint8_t *)text, length);
}

void Terminal_Sendf(const char *format, ...)
{
  va_list args;
  int length;
  va_start(args, format);
  length = vsnprintf(tx_buffer, sizeof(tx_buffer), format, args);
  va_end(args);
  if (length < 0) return;
  if ((size_t)length >= sizeof(tx_buffer)) length = (int)sizeof(tx_buffer) - 1;
  (void)CDC_Transmit_FS((uint8_t *)tx_buffer, (uint16_t)length);
}

uint8_t Terminal_GetEcho(void) { return echo_enabled; }
void Terminal_SetEcho(uint8_t enabled) { echo_enabled = enabled ? 1U : 0U; }