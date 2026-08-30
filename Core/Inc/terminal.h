#ifndef TERMINAL_H
#define TERMINAL_H

#include "main.h"

#define TERMINAL_COMMAND_SIZE 64U

void Terminal_Init(void);
void Terminal_RxData(const uint8_t *data, uint32_t length);
uint8_t Terminal_GetCommand(char *destination, uint32_t size);
uint8_t Terminal_CommandPending(void);
void Terminal_Send(const char *text);
void Terminal_Sendf(const char *format, ...);
uint8_t Terminal_GetEcho(void);
void Terminal_SetEcho(uint8_t enabled);

#endif