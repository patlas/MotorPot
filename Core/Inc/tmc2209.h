#ifndef TMC2209_DRIVER_H
#define TMC2209_DRIVER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef TMC2209_Init(void);
uint8_t TMC2209_IsReady(void);
// HAL_StatusTypeDef TMC2209_WriteRegister(uint8_t address, uint32_t value);
// HAL_StatusTypeDef TMC2209_ReadRegister(uint8_t address, uint32_t *value);
void TMC2209_Enable(void);
void TMC2209_Disable(void);
void TMC2209_SetDirectionInverted(uint8_t inverted);

#ifdef __cplusplus
}
#endif

#endif
