#ifndef STEP_ENGINE_H
#define STEP_ENGINE_H

#include "main.h"

typedef enum
{
  STEP_DIRECTION_LEFT = 0,
  STEP_DIRECTION_RIGHT = 1
} StepDirection;

typedef enum
{
  STEP_STOP_NONE = 0,
  STEP_STOP_COMPLETE,
  STEP_STOP_STALL,
  STEP_STOP_LIMIT,
  STEP_STOP_ABORTED
} StepStopReason;

extern volatile uint8_t g_stall_detected;

void StepEngine_Init(void);
HAL_StatusTypeDef StepEngine_Start(uint32_t steps, StepDirection direction,
                                   uint32_t edge_interval_us, uint8_t stop_on_stall);
void StepEngine_Stop(void);
void StepEngine_AbortFromISR(void);
void StepEngine_TIM2_IRQHandler(void);
uint8_t StepEngine_IsBusy(void);
uint8_t StepEngine_IsDone(void);
uint32_t StepEngine_GetExecutedSteps(void);
StepStopReason StepEngine_GetStopReason(void);
void StepEngine_SetStepTimeUs(uint32_t edge_interval_us);
void StallGuard_Masking_Process(void);
void PWM_DynamicRamp_Process(volatile uint8_t *p_trigger_flag);


#endif
