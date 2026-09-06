#include "app.h"
#include "adc.h"
#include "step_engine.h"
#include "stm32f4xx_hal.h"
#include "terminal.h"
#include "tmc2209.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define RELAY_DELAY_MS 100U
#define CALIBRATION_MAX_STEPS 10000000UL
#define CALIBRATION_TIMEOUT_MS 90000UL
#define STEP_PULSE_DELAY_US 80U // TO JEST TARGET SPEED ale się nie ustawia -> set pwm ramp TOOD PATLAS
#define COARSE_DELAY_US 600U
#define FINE_DELAY_US 15U
#define COARSE_MARGIN_STEPS 15UL
#define LOAD_SETTLE_DELAY_MS 100U
#define FINE_PROBE_STEPS 10UL
#define FINE_NEAR_TARGET_MV 10U
#define FINE_BULK_RATIO_PCT 70U
#define FINE_TUNE_TOLERANCE_PCT 2U

typedef enum
{
  APP_IDLE = 0,
  APP_CAL_RIGHT,
  APP_CAL_LEFT,
  APP_CAL_SETTLE_RIGHT,
  APP_CAL_SETTLE_LEFT,
  APP_SET_PREPARE,
  APP_SET_REHO_PREPARE,
  APP_SET_REHO_PROBE_WAIT,
  APP_SET_START,
  APP_SET_LOADED_START,
  APP_SET_COARSE,
  APP_SET_SETTLE,
  APP_SET_PRELOAD,
  APP_SET_LOAD_SETTLE,
  APP_SET_FINE,
  APP_SET_FINISH,
  APP_ERROR
} AppState;

static AppState app_state;
static uint8_t operation_setv;
static uint8_t debug_mode;
static uint32_t dbg_timeout=0;
static uint8_t calibrated;
static uint8_t output_is_on;
static int32_t requested_voltage;
static uint32_t current_steps;
static uint32_t total_steps;
static uint16_t left_voltage;
static uint16_t right_voltage;
static uint32_t settle_deadline;
static uint32_t operation_deadline;
static uint32_t fine_step_steps;
static uint16_t fine_reference_voltage;
static uint8_t fine_reference_valid;
static uint8_t fine_probe_done;
static StepDirection last_move_direction;
static uint8_t fine_move_pending;
static uint8_t reho_mode;
static char command_line[TERMINAL_COMMAND_SIZE];

static uint8_t elapsed(uint32_t deadline)
{
  return ((int32_t)(HAL_GetTick() - deadline) >= 0) ? 1U : 0U;
}

static void relay_pulse(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  HAL_Delay(RELAY_DELAY_MS);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void adc_switch(uint8_t on)
{
  HAL_GPIO_WritePin(ADC_ON_GPIO_Port, ADC_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_OFF_GPIO_Port, ADC_OFF_Pin, GPIO_PIN_RESET);
  relay_pulse(on ? ADC_ON_GPIO_Port : ADC_OFF_GPIO_Port,
              on ? ADC_ON_Pin : ADC_OFF_Pin);
}

static void output_switch(uint8_t on)
{
  HAL_GPIO_WritePin(OUT_ON_GPIO_Port, OUT_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT_OFF_GPIO_Port, OUT_OFF_Pin, GPIO_PIN_RESET);
  relay_pulse(on ? OUT_ON_GPIO_Port : OUT_OFF_GPIO_Port,
              on ? OUT_ON_Pin : OUT_OFF_Pin);
  output_is_on = on ? 1U : 0U;
}

static void reverse_switch(uint8_t reverse)
{
  HAL_GPIO_WritePin(REV_ON_GPIO_Port, REV_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(REV_OFF_GPIO_Port, REV_OFF_Pin, GPIO_PIN_RESET);
  relay_pulse(reverse ? REV_ON_GPIO_Port : REV_OFF_GPIO_Port,
              reverse ? REV_ON_Pin : REV_OFF_Pin);
}

static uint32_t target_voltage(void)
{
  int64_t target = requested_voltage;
  if (target < 0) target = -target;
  if ((uint32_t)target < left_voltage) return left_voltage;
  if ((uint32_t)target > right_voltage) return right_voltage;
  return (uint32_t)target;
}

static uint32_t calibration_steps_per_mv(void)
{
  uint32_t span = (uint32_t)(right_voltage - left_voltage);
  uint32_t steps;

  if (span == 0U || total_steps == 0U) return 1U;
  steps = (total_steps + span / 2U) / span;
  return steps == 0U ? 1U : steps;
}

static uint32_t bounded_move_steps(StepDirection direction, uint32_t requested_steps)
{
  uint32_t available = (direction == STEP_DIRECTION_RIGHT) ?
                       (total_steps - current_steps) : current_steps;
  return requested_steps < available ? requested_steps : available;
}

static void update_current_position(uint32_t executed)
{
  if (last_move_direction == STEP_DIRECTION_RIGHT)
    current_steps = (current_steps + executed <= total_steps) ?
                    current_steps + executed : total_steps;
  else
    current_steps = (executed <= current_steps) ? current_steps - executed : 0U;
}

static void begin_loaded_tuning(void)
{
  output_switch(1U);
  settle_deadline = HAL_GetTick() + LOAD_SETTLE_DELAY_MS;
  fine_step_steps = calibration_steps_per_mv();
  fine_reference_valid = 0U;
  fine_probe_done = 0U;
  fine_move_pending = 0U;
  app_state = APP_SET_LOAD_SETTLE;
}

static void clear_driver_diag(void)
{
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  g_stall_detected = 0U;
}

static void stop_operation(uint8_t send_reply)
{
  StepEngine_Stop();
  TMC2209_Disable();
  output_switch(0U);
  adc_switch(0U);
  app_state = APP_IDLE;
  operation_setv = 0U;
  // reho_mode = 0U; // PATLAS
  if (send_reply) Terminal_Send("OK\r\n");
}

static void fail_operation(void)
{
  StepEngine_Stop();
  TMC2209_Disable();
  output_switch(0U);
  adc_switch(0U);
  app_state = APP_IDLE;
  operation_setv = 0U;
  Terminal_Send("ERROR\r\n");
}

static uint8_t start_calibration(uint8_t for_setv)
{
  if (!TMC2209_IsReady() && TMC2209_Init() != HAL_OK) return 0U; // PATLAS
  operation_setv = for_setv;
  adc_switch(1);
  output_switch(0U);
  current_steps = 0U;
  total_steps = 0U;
  operation_deadline = HAL_GetTick() + CALIBRATION_TIMEOUT_MS; // PATLAS
  TMC2209_Enable(); // PATLAS
  clear_driver_diag();
  if (StepEngine_Start(CALIBRATION_MAX_STEPS, STEP_DIRECTION_RIGHT,
                       STEP_PULSE_DELAY_US, 1U) != HAL_OK) return 0U;
  app_state = APP_CAL_RIGHT;
  return 1U;
}

static uint8_t start_set_position(void)
{
  uint32_t span, target_position, distance;
  StepDirection direction;
  uint32_t target;
  if (!calibrated || total_steps == 0U) return 0U;
  target = target_voltage();
  span = (uint32_t)(right_voltage - left_voltage);
  if (span == 0U) span = 1U;
  target_position = ((uint32_t)target - left_voltage) * total_steps / span;
  if (target_position >= current_steps)
  {
    direction = STEP_DIRECTION_RIGHT;
    distance = target_position - current_steps;
    if (distance > COARSE_MARGIN_STEPS) distance -= COARSE_MARGIN_STEPS;
    else distance = 0U;
  }
  else
  {
    direction = STEP_DIRECTION_LEFT;
    distance = current_steps - target_position;
    if (distance > COARSE_MARGIN_STEPS) distance -= COARSE_MARGIN_STEPS;
    else distance = 0U;
  }
  TMC2209_Enable();
  last_move_direction = direction;
  if (distance == 0U)
  {
    fine_move_pending = 0U;
    settle_deadline = HAL_GetTick() + 50U;
    app_state = APP_SET_SETTLE;
    return 1U;
  }
  if (StepEngine_Start(distance, direction, COARSE_DELAY_US, 1U) != HAL_OK) return 0U;
  app_state = APP_SET_COARSE;
  return 1U;
}

static void handle_command(char *line)
{
  char *command = strtok(line, " ");
  char *argument;
  if (command == NULL) return;
  if (app_state != APP_IDLE && strcmp(command, "STOP") != 0)
  {
    Terminal_Send("BUSY\r\n");
    return;
  }
  if (strcmp(command, "HELP") == 0)
  {
    Terminal_Send("Available commands:\r\nINIT\r\nREV <0|1>\r\nON <0|1>\r\nADC <0|1>\r\nSETV <mV>\r\nGETV\r\nDBG <0|1>\r\nCAL\r\nREHO <0|1>\r\nSETECHO <0|1>\r\nSTOP\r\n");
  }
  else if (strcmp(command, "STOP") == 0)
  {
    stop_operation(1U);
  }
  else if (strcmp(command, "INIT") == 0)
  {
    Terminal_Send(TMC2209_Init() == HAL_OK ? "OK\r\n" : "ERROR\r\n");
  }
  else if (strcmp(command, "CAL") == 0)
  {
    if (!start_calibration(0U)) Terminal_Send("ERROR\r\n");
  }
  else if (strcmp(command, "SETV") == 0)
  {
    argument = strtok(NULL, " ");
    if (argument == NULL) { Terminal_Send("ERROR\r\n"); return; }
    requested_voltage = strtol(argument, NULL, 10);
    if (!calibrated)
    {
      if (!start_calibration(1U)) Terminal_Send("ERROR\r\n");
    }
    else
    {
      app_state = APP_SET_PREPARE;
      settle_deadline = HAL_GetTick() + 1000U;
    }
  }
  else if (strcmp(command, "GETV") == 0)
  {
    adc_switch(1U);
    Terminal_Sendf("OK:%u\r\n", g_adc_mv);
    adc_switch(0U);
  }
  else if (strcmp(command, "REV") == 0)
  {
    argument = strtok(NULL, " ");
    if (argument == NULL) { Terminal_Send("ERROR: NO ARGUMENT\r\n"); return; }
    reverse_switch((uint8_t)(strtol(argument, NULL, 10) != 0));
    Terminal_Sendf("OUT REV = %u\r\n", (unsigned)(strtol(argument, NULL, 10) != 0));
  }
  else if (strcmp(command, "ON") == 0 || strcmp(command, "ADC") == 0)
  {
    argument = strtok(NULL, " ");
    if (argument == NULL) { Terminal_Send("ERROR: NO ARGUMENT\r\n"); return; }
    if (strcmp(command, "ON") == 0)
    {
      uint8_t on = (uint8_t)(strtol(argument, NULL, 10) != 0);
      output_switch(on);
      Terminal_Send(on ? "OUT ON\r\n" : "OUT OFF\r\n");
    }
    else
    {
      uint8_t on = (uint8_t)(strtol(argument, NULL, 10) != 0);
      adc_switch(on);
      Terminal_Send(on ? "ADC ON\r\n" : "ADC OFF\r\n");
    }
  }
  else if (strcmp(command, "DBG") == 0)
  {
    argument = strtok(NULL, " ");
    if (argument == NULL) { Terminal_Send("ERROR: NO ARGUMENT\r\n"); return; }
    debug_mode = (uint8_t)(strtol(argument, NULL, 10) != 0);
    Terminal_Sendf("DBG = %u\r\n", debug_mode);
  }
  else if (strcmp(command, "REHO") == 0)
  {
    argument = strtok(NULL, " ");
    if (argument == NULL) { Terminal_Send("ERROR: NO ARGUMENT\r\n"); return; }
    reho_mode = (uint8_t)(strtol(argument, NULL, 10) != 0);
    Terminal_Sendf("REHO = %u\r\n", reho_mode);
  }
  else if (strcmp(command, "SETECHO") == 0)
  {
    argument = strtok(NULL, " ");
    if (argument == NULL) { Terminal_Send("ERROR: NO ARGUMENT\r\n"); return; }
    Terminal_SetEcho((uint8_t)(strtol(argument, NULL, 10) != 0));
    Terminal_Send("OK\r\n");
  }
  else
  {
    Terminal_Send("Unknown command\r\n");
  }
}

void App_Init(void)
{
  Terminal_Init();
  StepEngine_Init();
  app_state = APP_IDLE;
  calibrated = 0U;
  output_is_on = 0U;
  output_switch(0U);
  debug_mode = 0U;
  operation_setv = 0U;
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
}

void App_Process(void)
{
  uint32_t executed;
  uint32_t target;
  uint32_t error;
  uint32_t distance;
  uint32_t delta_voltage;
  uint32_t measured_steps_per_mv;
  uint32_t requested_steps;
  uint64_t requested_steps64;
  StepDirection direction;
  if (Terminal_GetCommand(command_line, sizeof(command_line))) handle_command(command_line);

  // if (app_state == APP_IDLE) return; // PATLAS
    if (app_state == APP_IDLE)
    {
        if (debug_mode)
        {
            if (elapsed(dbg_timeout))
            {
                Terminal_Sendf("ADC:%u\r\n", g_adc_mv);
                dbg_timeout = HAL_GetTick() + 500U;
            }
        }
        return;
    }

  if (elapsed(operation_deadline) &&
      (app_state == APP_CAL_RIGHT || app_state == APP_CAL_LEFT))
  {
    fail_operation();
    return;
  }

  if (app_state == APP_CAL_RIGHT && StepEngine_IsDone())
  {
    if (StepEngine_GetStopReason() != STEP_STOP_STALL) { fail_operation(); return; }
    right_voltage = g_adc_mv;
    current_steps = CALIBRATION_MAX_STEPS;
    operation_deadline = HAL_GetTick() + CALIBRATION_TIMEOUT_MS;
    clear_driver_diag();
    if (StepEngine_Start(CALIBRATION_MAX_STEPS, STEP_DIRECTION_LEFT,
                         STEP_PULSE_DELAY_US, 1U) != HAL_OK) { fail_operation(); return; }
    app_state = APP_CAL_LEFT;
  }
  else if (app_state == APP_CAL_LEFT && StepEngine_IsDone())
  {
    if (StepEngine_GetStopReason() != STEP_STOP_STALL) { fail_operation(); return; }
    executed = StepEngine_GetExecutedSteps();
    total_steps = executed;
    left_voltage = g_adc_mv;
    current_steps = 0U;
    calibrated = (total_steps > 0U && right_voltage > left_voltage) ? 1U : 0U;
    TMC2209_Disable();
    adc_switch(0U);
    if (!calibrated) { fail_operation(); return; }
    if (!operation_setv) { app_state = APP_IDLE; Terminal_Send("OK\r\n"); }
    else { app_state = APP_SET_PREPARE; settle_deadline = HAL_GetTick() + 1000U; }
  }
  else if (app_state == APP_SET_PREPARE && elapsed(settle_deadline))
  {
    adc_switch(1U);
    reverse_switch(requested_voltage < 0);
    settle_deadline = HAL_GetTick() + LOAD_SETTLE_DELAY_MS;
    if (reho_mode)
    {
      output_switch(1U);
      app_state = APP_SET_REHO_PREPARE;
    }
    else if (output_is_on)
      app_state = APP_SET_LOADED_START;
    else
    {
      output_switch(0U);
      app_state = APP_SET_START;
    }
  }
  else if (app_state == APP_SET_REHO_PREPARE && elapsed(settle_deadline))
  {
    target = target_voltage();
    error = (g_adc_mv > target) ? (g_adc_mv - target) : (target - g_adc_mv);
    if ((target == 0U && error == 0U) ||
        (target > 0U && error * 100U <= target * FINE_TUNE_TOLERANCE_PCT))
    {
      settle_deadline = HAL_GetTick() + 50U;
      app_state = APP_SET_FINISH;
    }
    else
    {
      direction = g_adc_mv < target ? STEP_DIRECTION_RIGHT : STEP_DIRECTION_LEFT;
      uint32_t max_probe = total_steps * 5UL / 100UL;
      if (max_probe < 10UL) max_probe = 10UL;
      distance = bounded_move_steps(direction, max_probe);
      if (distance == 0U)
      {
        settle_deadline = HAL_GetTick() + 50U;
        app_state = APP_SET_FINISH;
        return;
      }
      TMC2209_Enable();
      last_move_direction = direction;
      fine_reference_voltage = g_adc_mv;
      fine_reference_valid = 1U;
      if (StepEngine_Start(distance, direction, FINE_DELAY_US, 0U) != HAL_OK)
        fail_operation();
      else
      {
        fine_move_pending = 1U;
        app_state = APP_SET_REHO_PROBE_WAIT;
      }
    }
  }
  else if (app_state == APP_SET_REHO_PROBE_WAIT)
  {
    if (!StepEngine_IsDone()) return;
    executed = StepEngine_GetExecutedSteps();
    update_current_position(executed);
    fine_move_pending = 0U;

    if (fine_reference_valid)
    {
      delta_voltage = (g_adc_mv > fine_reference_voltage) ?
                      (g_adc_mv - fine_reference_voltage) :
                      (fine_reference_voltage - g_adc_mv);
      if (delta_voltage > 0U && executed > 0U)
      {
        measured_steps_per_mv = (executed + delta_voltage / 2U) / delta_voltage;
        if (measured_steps_per_mv == 0U) measured_steps_per_mv = 1U;
        fine_step_steps = measured_steps_per_mv;
      }
      else
      {
        fine_step_steps = calibration_steps_per_mv();
      }
      fine_reference_valid = 0U;
      fine_probe_done = 1U;
    }
    settle_deadline = HAL_GetTick() + LOAD_SETTLE_DELAY_MS;
    app_state = APP_SET_FINE;
  }
  else if (app_state == APP_SET_START && elapsed(settle_deadline))
  {
    if (!start_set_position()) fail_operation();
  }
  else if (app_state == APP_SET_LOADED_START && elapsed(settle_deadline))
  {
    target = target_voltage();
    error = (g_adc_mv > target) ? (g_adc_mv - target) : (target - g_adc_mv);
    fine_step_steps = calibration_steps_per_mv();
    fine_reference_valid = 0U;
    fine_probe_done = 0U;
    fine_move_pending = 0U;

    if ((target == 0U && error == 0U) ||
        (target > 0U && error * 100U <= target * FINE_TUNE_TOLERANCE_PCT))
    {
      settle_deadline = HAL_GetTick() + 50U;
      app_state = APP_SET_FINISH;
    }
    else
    {
      direction = g_adc_mv < target ? STEP_DIRECTION_RIGHT : STEP_DIRECTION_LEFT;
      requested_steps64 = (uint64_t)error * fine_step_steps * FINE_BULK_RATIO_PCT / 100U;
      requested_steps = requested_steps64 > UINT32_MAX ? UINT32_MAX :
                        (uint32_t)requested_steps64;
      if (requested_steps == 0U) requested_steps = 1U;
      distance = bounded_move_steps(direction, requested_steps);
      if (distance == 0U)
      {
        settle_deadline = HAL_GetTick() + 50U;
        app_state = APP_SET_FINISH;
        return;
      }

      TMC2209_Enable();
      last_move_direction = direction;
      fine_reference_voltage = g_adc_mv;
      fine_reference_valid = 1U;
      if (StepEngine_Start(distance, direction, FINE_DELAY_US, 0U) != HAL_OK)
        fail_operation();
      else
      {
        fine_move_pending = 1U;
        app_state = APP_SET_FINE;
      }
    }
  }
  else if (app_state == APP_SET_COARSE && StepEngine_IsDone())
  {
    if (StepEngine_GetStopReason() == STEP_STOP_STALL) { fail_operation(); return; }
    executed = StepEngine_GetExecutedSteps();
    if (last_move_direction == STEP_DIRECTION_RIGHT)
      current_steps = (current_steps + executed <= total_steps) ? current_steps + executed : total_steps;
    else
      current_steps = (executed <= current_steps) ? current_steps - executed : 0U;
    settle_deadline = HAL_GetTick() + 50U;
    app_state = APP_SET_SETTLE;
  }
  else if (app_state == APP_SET_SETTLE && elapsed(settle_deadline))
  {
    fine_move_pending = 0U;
    fine_step_steps = calibration_steps_per_mv();
    fine_reference_valid = 0U;
    fine_probe_done = 0U;
    app_state = APP_SET_PRELOAD;
  }
  else if (app_state == APP_SET_PRELOAD)
  {
    if (fine_move_pending)
    {
      if (!StepEngine_IsDone()) return;
      executed = StepEngine_GetExecutedSteps();
      update_current_position(executed);
      fine_move_pending = 0U;
    }

    target = target_voltage();
    if (g_adc_mv <= target)
    {
      begin_loaded_tuning();
    }
    else if (!StepEngine_IsBusy())
    {
      direction = STEP_DIRECTION_LEFT;
      distance = bounded_move_steps(direction, fine_step_steps);
      if (distance == 0U)
      {
        fail_operation();
        return;
      }
      last_move_direction = direction;
      if (StepEngine_Start(distance, direction, FINE_DELAY_US, 0U) != HAL_OK)
        fail_operation();
      else
        fine_move_pending = 1U;
    }
  }
  else if (app_state == APP_SET_LOAD_SETTLE && elapsed(settle_deadline))
  {
    app_state = APP_SET_FINE;
  }
  else if (app_state == APP_SET_FINE)
  {
    if (fine_move_pending)
    {
      if (!StepEngine_IsDone()) return;
      executed = StepEngine_GetExecutedSteps();
      update_current_position(executed);
      fine_move_pending = 0U;

      if (fine_reference_valid)
      {
        delta_voltage = (g_adc_mv > fine_reference_voltage) ?
                        (g_adc_mv - fine_reference_voltage) :
                        (fine_reference_voltage - g_adc_mv);
        if (delta_voltage > 0U && executed > 0U)
        {
          measured_steps_per_mv = (executed + delta_voltage / 2U) / delta_voltage;
          if (measured_steps_per_mv == 0U) measured_steps_per_mv = 1U;
          fine_step_steps = (fine_step_steps + measured_steps_per_mv + 1U) / 2U;
          if (fine_step_steps == 0U) fine_step_steps = 1U;
        }
        fine_reference_valid = 0U;
        fine_probe_done = 1U;
      }
    }

    target = target_voltage();
    error = (g_adc_mv > target) ? (g_adc_mv - target) : (target - g_adc_mv);
    if ((target == 0U && error == 0U) || (target > 0U && error * 100U <= target * FINE_TUNE_TOLERANCE_PCT) ||
        (g_adc_mv < target && current_steps >= total_steps) ||
        (g_adc_mv > target && current_steps == 0U))
    {
      settle_deadline = HAL_GetTick() + 50U;
      app_state = APP_SET_FINISH;
    }
    else if (!StepEngine_IsBusy())
    {
      direction = g_adc_mv < target ? STEP_DIRECTION_RIGHT : STEP_DIRECTION_LEFT;
      if (!fine_probe_done)
      {
        requested_steps = FINE_PROBE_STEPS;
      }
      else if (error > FINE_NEAR_TARGET_MV)
      {
        requested_steps64 = (uint64_t)error * fine_step_steps * FINE_BULK_RATIO_PCT / 100U;
        requested_steps = requested_steps64 > UINT32_MAX ? UINT32_MAX :
                          (uint32_t)requested_steps64;
      }
      else
      {
        requested_steps = fine_step_steps;
      }
      distance = bounded_move_steps(direction, requested_steps);
      if (distance == 0U)
      {
        settle_deadline = HAL_GetTick() + 50U;
        app_state = APP_SET_FINISH;
        return;
      }
      last_move_direction = direction;
      fine_reference_voltage = g_adc_mv;
      fine_reference_valid = 1U;
      if (StepEngine_Start(distance, direction, FINE_DELAY_US, 0U) != HAL_OK)
        fail_operation();
      else
        fine_move_pending = 1U;
    }
  }
  else if (app_state == APP_SET_FINISH && elapsed(settle_deadline))
  {
    TMC2209_Disable();
    uint16_t measV = g_adc_mv;
    adc_switch(0U);
    Terminal_Sendf("OK:%u\r\n", measV);
    app_state = APP_IDLE;
    operation_setv = 0U;
  }
}
