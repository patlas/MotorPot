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
#define FINE_DELAY_US 15U
#define LOAD_SETTLE_DELAY_MS 100U
#define SET_MOVING_SAMPLE_STEPS 1600UL /* Half a turn at 200 steps/rev, 16 microsteps. */
#define SET_FINE_STEPS 10UL
#define SET_FINE_THRESHOLD_PCT 80U
#define FINE_TUNE_TOLERANCE_PCT 2U
#define ADC_AVERAGE_SAMPLES 16U
#define ADC_SAMPLE_INTERVAL_MS 2U

typedef enum
{
  APP_IDLE = 0,
  APP_CAL_HOME,
  APP_SET_PREPARE,
  APP_SET_LOAD_SETTLE,
  APP_SET_SAMPLE,
  APP_SET_FINE,
  APP_SET_CONTINUOUS,
  APP_SET_FINISH
} AppState;

static AppState app_state;
static uint8_t operation_setv;
static uint8_t debug_mode;
static uint32_t dbg_timeout=0;
static uint8_t calibrated;
static uint8_t left_limit_known;
static int32_t requested_voltage;
static uint32_t current_steps;
static uint32_t total_steps;
static uint16_t minimum_voltage;
static uint16_t maximum_voltage;
static uint16_t measured_voltage;
static uint32_t settle_deadline;
static uint32_t operation_deadline;
static uint32_t next_moving_sample;
static StepDirection last_move_direction;
static uint8_t fine_move_pending;
static uint8_t fine_move_stalled;
static uint32_t fine_move_executed;
/* Deprecated: retained only to preserve the REHO command response. */
static uint8_t reho_mode;
static uint8_t adc_sampling;
static uint8_t adc_sample_count;
static uint32_t adc_sample_sum;
static uint32_t adc_sample_deadline;
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
  on = on ? 1U : 0U;
  HAL_GPIO_WritePin(ADC_ON_GPIO_Port, ADC_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_OFF_GPIO_Port, ADC_OFF_Pin, GPIO_PIN_RESET);
  relay_pulse(on ? ADC_ON_GPIO_Port : ADC_OFF_GPIO_Port,
              on ? ADC_ON_Pin : ADC_OFF_Pin);
}

static void output_switch(uint8_t on)
{
  on = on ? 1U : 0U;
  HAL_GPIO_WritePin(OUT_ON_GPIO_Port, OUT_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT_OFF_GPIO_Port, OUT_OFF_Pin, GPIO_PIN_RESET);
  relay_pulse(on ? OUT_ON_GPIO_Port : OUT_OFF_GPIO_Port,
              on ? OUT_ON_Pin : OUT_OFF_Pin);
}

static void reverse_switch(uint8_t reverse)
{
  reverse = reverse ? 1U : 0U;
  HAL_GPIO_WritePin(REV_ON_GPIO_Port, REV_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(REV_OFF_GPIO_Port, REV_OFF_Pin, GPIO_PIN_RESET);
  relay_pulse(reverse ? REV_ON_GPIO_Port : REV_OFF_GPIO_Port,
              reverse ? REV_ON_Pin : REV_OFF_Pin);
}

static uint32_t target_voltage(void)
{
  int64_t target = requested_voltage;
  if (target < 0) target = -target;
  if ((uint32_t)target < minimum_voltage) return minimum_voltage;
  if (left_limit_known && (uint32_t)target > maximum_voltage) return maximum_voltage;
  return (uint32_t)target;
}

static uint8_t within_target_tolerance(uint32_t target, uint32_t error)
{
  if (target == 0U) return error == 0U;
  return ((uint64_t)error * 100U <= (uint64_t)target * FINE_TUNE_TOLERANCE_PCT) ?
         1U : 0U;
}

static uint32_t bounded_move_steps(StepDirection direction, uint32_t requested_steps)
{
  uint32_t available;

  if (direction == STEP_DIRECTION_LEFT)
  {
    uint32_t left_bound = left_limit_known ? total_steps : CALIBRATION_MAX_STEPS;
    available = (current_steps < left_bound) ? left_bound - current_steps : 0U;
  }
  else
    available = current_steps;

  return requested_steps < available ? requested_steps : available;
}

static void update_current_position(uint32_t executed)
{
  if (last_move_direction == STEP_DIRECTION_LEFT)
  {
    uint32_t left_bound = left_limit_known ? total_steps : CALIBRATION_MAX_STEPS;
    current_steps = (current_steps <= left_bound &&
                     executed <= left_bound - current_steps) ?
                    current_steps + executed : left_bound;
  }
  else
    current_steps = (executed <= current_steps) ? current_steps - executed : 0U;
}

static void begin_adc_sampling(void)
{
  adc_sampling = 1U;
  adc_sample_count = 0U;
  adc_sample_sum = 0U;
  adc_sample_deadline = HAL_GetTick();
}

static uint8_t collect_adc_average(void)
{
  if (!adc_sampling || !elapsed(adc_sample_deadline)) return 0U;

  adc_sample_sum += g_adc_mv;
  adc_sample_count++;
  if (adc_sample_count >= ADC_AVERAGE_SAMPLES)
  {
    measured_voltage = (uint16_t)(adc_sample_sum / ADC_AVERAGE_SAMPLES);
    adc_sampling = 0U;
    return 1U;
  }

  adc_sample_deadline = HAL_GetTick() + ADC_SAMPLE_INTERVAL_MS;
  return 0U;
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
  if (app_state == APP_CAL_HOME)
    calibrated = 0U;

  if (fine_move_pending)
  {
    StepEngine_Stop();
    if (StepEngine_IsDone()) update_current_position(StepEngine_GetExecutedSteps());
  }
  else
    StepEngine_Stop();

  TMC2209_Disable();
  output_switch(0U);
  adc_switch(0U);
  app_state = APP_IDLE;
  operation_setv = 0U;
  fine_move_pending = 0U;
  adc_sampling = 0U;
  // reho_mode = 0U; // PATLAS
  if (send_reply) Terminal_Send("OK\r\n");
}

static void fail_operation(void)
{
  StepEngine_Stop();
  TMC2209_Disable();
  output_switch(0U);
  adc_switch(0U);
  calibrated = 0U;
  left_limit_known = 0U;
  fine_move_pending = 0U;
  adc_sampling = 0U;
  app_state = APP_IDLE;
  operation_setv = 0U;
  Terminal_Send("ERROR\r\n");
}

static uint8_t start_calibration(uint8_t for_setv)
{
  output_switch(0U);
  adc_switch(0U);
  if (!TMC2209_IsReady() && TMC2209_Init() != HAL_OK) return 0U;

  operation_setv = for_setv;
  calibrated = 0U;
  left_limit_known = 0U;
  current_steps = 0U;
  total_steps = 0U;
  minimum_voltage = 0U;
  maximum_voltage = 0U;
  fine_move_pending = 0U;
  fine_move_stalled = 0U;
  fine_move_executed = 0U;
  adc_sampling = 0U;
  operation_deadline = HAL_GetTick() + CALIBRATION_TIMEOUT_MS; // PATLAS
  clear_driver_diag();
  TMC2209_Enable();
  if (StepEngine_Start(CALIBRATION_MAX_STEPS, STEP_DIRECTION_RIGHT,
                       STEP_PULSE_DELAY_US, 1U) != HAL_OK)
  {
    TMC2209_Disable();
    operation_setv = 0U;
    return 0U;
  }
  app_state = APP_CAL_HOME;
  return 1U;
}

static uint8_t start_voltage_adjustment(uint32_t target)
{
  StepDirection direction = measured_voltage < target ? STEP_DIRECTION_LEFT : STEP_DIRECTION_RIGHT;
  uint32_t distance = bounded_move_steps(direction, SET_FINE_STEPS);

  if (distance == 0U)
    return 2U;
  last_move_direction = direction;
  fine_move_stalled = 0U;
  clear_driver_diag();
  TMC2209_Enable();
  if (StepEngine_Start(distance, direction, FINE_DELAY_US, 1U) != HAL_OK)
    return 0U;
  fine_move_pending = 1U;
  app_state = APP_SET_FINE;
  return 1U;
}

static uint8_t start_continuous_move(void)
{
  uint32_t distance = bounded_move_steps(STEP_DIRECTION_LEFT, CALIBRATION_MAX_STEPS);
  if (distance == 0U) return 0U;
  last_move_direction = STEP_DIRECTION_LEFT;
  next_moving_sample = SET_MOVING_SAMPLE_STEPS;
  clear_driver_diag();
  TMC2209_Enable();
  if (StepEngine_Start(distance, STEP_DIRECTION_LEFT, FINE_DELAY_US, 1U) != HAL_OK)
    return 0U;
  fine_move_pending = 1U;
  app_state = APP_SET_CONTINUOUS;
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
    Terminal_Send("Available commands:\r\nINIT\r\nREV <0|1>\r\nON <0|1>\r\nADC <0|1>\r\nSETV <mV>\r\nGETV\r\nDBG <0|1>\r\nCAL\r\nREHO <0|1> (deprecated)\r\nSETECHO <0|1>\r\nSTOP\r\n");
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
    if (!start_calibration(1U)) Terminal_Send("ERROR\r\n");
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
  left_limit_known = 0U;
  output_switch(0U);
  adc_switch(0U);
  reverse_switch(0U);
  debug_mode = 0U;
  operation_setv = 0U;
  reho_mode = 0U;
  current_steps = 0U;
  total_steps = 0U;
  minimum_voltage = 0U;
  maximum_voltage = 0U;
  measured_voltage = 0U;
  fine_move_pending = 0U;
  fine_move_stalled = 0U;
  fine_move_executed = 0U;
  adc_sampling = 0U;
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
}

void App_Process(void)
{
  uint32_t target;
  uint32_t error;
  uint8_t start_result;

  if (Terminal_GetCommand(command_line, sizeof(command_line))) handle_command(command_line);

  if (app_state == APP_IDLE)
  {
    if (debug_mode && elapsed(dbg_timeout))
    {
      Terminal_Sendf("ADC:%u\r\n", g_adc_mv);
      dbg_timeout = HAL_GetTick() + 500U;
    }
    return;
  }

  if (elapsed(operation_deadline))
  {
    fail_operation();
    return;
  }

  if (app_state == APP_CAL_HOME && StepEngine_IsDone())
  {
    if (StepEngine_GetStopReason() != STEP_STOP_STALL)
    {
      fail_operation();
      return;
    }

    current_steps = 0U;
    total_steps = 0U;
    left_limit_known = 0U;
    calibrated = 1U;
    TMC2209_Disable();
    adc_switch(0U);

    if (!operation_setv)
    {
      app_state = APP_IDLE;
      Terminal_Send("OK\r\n");
    }
    else
    {
      operation_deadline = HAL_GetTick() + CALIBRATION_TIMEOUT_MS;
      settle_deadline = HAL_GetTick() + 1000U;
      app_state = APP_SET_PREPARE;
    }
  }
  else if (app_state == APP_SET_PREPARE && elapsed(settle_deadline))
  {
    if (!calibrated)
    {
      fail_operation();
      return;
    }

    output_switch(0U);
    reverse_switch(requested_voltage < 0 ? 1U : 0U);
    adc_switch(1U);
    output_switch(1U);

    fine_move_pending = 0U;
    fine_move_stalled = 0U;
    fine_move_executed = 0U;
    settle_deadline = HAL_GetTick() + LOAD_SETTLE_DELAY_MS;
    app_state = APP_SET_LOAD_SETTLE;
  }
  else if (app_state == APP_SET_LOAD_SETTLE && elapsed(settle_deadline))
  {
    begin_adc_sampling();
    app_state = APP_SET_SAMPLE;
  }
  else if (app_state == APP_SET_SAMPLE)
  {
    if (!collect_adc_average()) return;

    if (current_steps == 0U) minimum_voltage = measured_voltage;
    if (fine_move_stalled)
    {
      if (last_move_direction == STEP_DIRECTION_LEFT)
      {
        total_steps = current_steps;
        maximum_voltage = measured_voltage;
        left_limit_known = 1U;
      }
      else
      {
        current_steps = 0U;
        minimum_voltage = measured_voltage;
      }
      fine_move_stalled = 0U;
    }

    target = target_voltage();
    error = (measured_voltage > target) ? (measured_voltage - target) :
                                          (target - measured_voltage);
    if (within_target_tolerance(target, error))
    {
      settle_deadline = HAL_GetTick() + 50U;
      app_state = APP_SET_FINISH;
    }
    else
    {
      if (current_steps == 0U && measured_voltage < target &&
          (uint64_t)measured_voltage * 100U < (uint64_t)target * SET_FINE_THRESHOLD_PCT)
        start_result = start_continuous_move() ? 1U : 0U;
      else
        start_result = start_voltage_adjustment(target);
      if (start_result == 2U)
      {
        settle_deadline = HAL_GetTick() + 50U;
        app_state = APP_SET_FINISH;
      }
      else if (start_result == 0U)
        fail_operation();
    }
  }
  else if (app_state == APP_SET_CONTINUOUS)
  {
    if (!StepEngine_IsDone() && StepEngine_GetProgressSteps() >= next_moving_sample)
    {
      uint32_t progress = StepEngine_GetProgressSteps();
      next_moving_sample = progress + SET_MOVING_SAMPLE_STEPS;
      target = target_voltage();
      if ((uint64_t)g_adc_mv * 100U >= (uint64_t)target * SET_FINE_THRESHOLD_PCT)
        StepEngine_Stop();
    }
    if (StepEngine_IsDone())
    {
      StepStopReason reason = StepEngine_GetStopReason();
      fine_move_pending = 0U;
      if (reason != STEP_STOP_ABORTED && reason != STEP_STOP_STALL && reason != STEP_STOP_COMPLETE)
      {
        fail_operation();
        return;
      }
      update_current_position(StepEngine_GetExecutedSteps());
      fine_move_stalled = (reason == STEP_STOP_STALL) ? 1U : 0U;
      settle_deadline = HAL_GetTick() + LOAD_SETTLE_DELAY_MS;
      app_state = APP_SET_LOAD_SETTLE;
    }
  }
  else if (app_state == APP_SET_FINE && fine_move_pending && StepEngine_IsDone())
  {
    StepStopReason stop_reason = StepEngine_GetStopReason();
    fine_move_executed = StepEngine_GetExecutedSteps();
    fine_move_pending = 0U;
    if (stop_reason != STEP_STOP_COMPLETE && stop_reason != STEP_STOP_STALL)
    {
      fail_operation();
      return;
    }

    update_current_position(fine_move_executed);
    fine_move_stalled = (stop_reason == STEP_STOP_STALL) ? 1U : 0U;
    settle_deadline = HAL_GetTick() + LOAD_SETTLE_DELAY_MS;
    app_state = APP_SET_LOAD_SETTLE;
  }
  else if (app_state == APP_SET_FINISH && elapsed(settle_deadline))
  {
    TMC2209_Disable();
    adc_switch(0U);
    Terminal_Sendf("OK:%u\r\n", measured_voltage);
    app_state = APP_IDLE;
    operation_setv = 0U;
  }
}
