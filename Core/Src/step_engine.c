#include "step_engine.h"
#include "tim.h"
#include "tmc2209.h"

// PATLAS rozważyć arr start na mniejszy?? czy zmieści się to w okreise skoku timera?
#define RAMP_ARR_START       300U  // Początkowa niska prędkość (duży okres ARR = wolne impulsy)
#define RAMP_ARR_STEP        10U    // O ile zmniejszamy ARR w jednym kroku (im większa wartość, tym ostrzejszy start)
#define RAMP_INTERVAL_MS     50U      // Co ile milisekund następuje krok przyspieszenia
#define STALLGUARD_MASK_TIME_MS   500U  // Czas ignorowania DIAG w ms (w razie potrzeby zwiększ do 200-300ms)

volatile uint8_t g_stall_detected = 0U;
volatile uint8_t g_started = 0U;
volatile uint32_t g_target_interval_us = 0U;

// Zmienne globalne sterujące mechanizmem
volatile uint8_t g_stallguard_active = 1; // 0 = ignoruj DIAG (silnik startuje), 1 = reaguj na DIAG (silnik jedzie stabilnie)

static volatile uint8_t step_busy;
static volatile uint8_t step_done;
static volatile uint8_t step_stop_on_stall;
static volatile uint32_t step_target;
static volatile uint32_t step_executed;
static volatile uint32_t step_interval_us = 16U;
static volatile StepStopReason step_reason = STEP_STOP_NONE;

static uint32_t clamp_interval(uint32_t us)
{
  if (us < 2U) return 2U;
  if (us > 65535U) return 65535U;
  return us;
}

void StepEngine_Init(void)
{
  step_busy = 0U;
  step_done = 0U;
  step_executed = 0U;
  step_reason = STEP_STOP_NONE;
  g_stall_detected = 0U;
  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
}

void StepEngine_SetStepTimeUs(uint32_t edge_interval_us)
{
  step_interval_us = clamp_interval(edge_interval_us);
}

HAL_StatusTypeDef StepEngine_Start(uint32_t steps, StepDirection direction,
                                   uint32_t edge_interval_us, uint8_t stop_on_stall)
{
  if (step_busy || steps == 0U) return HAL_BUSY;

  // HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,
  //                   direction == STEP_DIRECTION_RIGHT ? GPIO_PIN_SET : GPIO_PIN_RESET);

    TMC2209_SetDirectionInverted((~direction) & 0x01); // PATLAS

  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);

  g_stall_detected = 0U;
  step_target = steps;
  step_executed = 0U;
  step_stop_on_stall = stop_on_stall ? 1U : 0U;
  step_reason = STEP_STOP_NONE;
  step_done = 0U;
  step_busy = 1U;

g_target_interval_us = edge_interval_us;
  StepEngine_SetStepTimeUs(RAMP_ARR_START);

  /* Stop timers */
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  HAL_TIM_Base_Stop(&htim2);
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);

  /* Configure TIM4 frequency: 84 MHz timer clock */
  uint32_t total_ticks = step_interval_us * 84U;
  if (total_ticks <= 65535U)
  {
    __HAL_TIM_SET_PRESCALER(&htim4, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim4, (uint16_t)(total_ticks - 1U));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)(total_ticks / 2U));
  }
  else
  {
    /* Use prescaler 83 => 84MHz / 84 = 1 MHz (1 tick = 1 us) */
    __HAL_TIM_SET_PRESCALER(&htim4, 83U);
    __HAL_TIM_SET_AUTORELOAD(&htim4, (uint16_t)(step_interval_us - 1U));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)(step_interval_us / 2U));
  }

  /* Reset timer counters */
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);

  /* Set target step compare match on 32-bit TIM2 */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, steps);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);

    __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_3);
    g_started = 1;
  /* Start slave counter first, then master pulse generator */
  HAL_TIM_Base_Start(&htim2);
  return HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void StepEngine_Stop(void)
{
  if (step_busy)
  {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
    step_executed = __HAL_TIM_GET_COUNTER(&htim2);
    step_reason = STEP_STOP_ABORTED;
    step_busy = 0U;
    step_done = 1U;
    g_started = 0;
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
  }
}

void StepEngine_AbortFromISR(void)
{
  if (step_busy && step_stop_on_stall && g_stallguard_active)
  {
    htim4.Instance->CR1 &= ~TIM_CR1_CEN;
    htim2.Instance->CR1 &= ~TIM_CR1_CEN;
    htim2.Instance->DIER &= ~TIM_DIER_CC1IE;

    step_executed = htim2.Instance->CNT;
    step_reason = STEP_STOP_STALL;
    step_busy = 0U;
    step_done = 1U;
    g_started = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
}

void StepEngine_TIM2_IRQHandler(void)
{
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) != RESET)
  {
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    if (step_busy)
    {
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
      HAL_TIM_Base_Stop(&htim2);
      __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);

      step_executed = __HAL_TIM_GET_COUNTER(&htim2);

      if (g_stall_detected && step_stop_on_stall)
      {
        step_reason = STEP_STOP_STALL;
      }
      else
      {
        step_reason = STEP_STOP_COMPLETE;
      }
      step_busy = 0U;
      step_done = 1U;
      HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
    }
  }
}

uint8_t StepEngine_IsBusy(void) { return step_busy; }
uint8_t StepEngine_IsDone(void) { return step_done; }
uint32_t StepEngine_GetExecutedSteps(void) { return step_executed; }
StepStopReason StepEngine_GetStopReason(void) { return step_reason; }

void PWM_DynamicRamp_Process(volatile uint8_t *p_trigger_flag)
{
    static uint32_t last_ramp_time = 0;
    static uint32_t current_arr = RAMP_ARR_START;
    static uint8_t ramp_in_progress = 0;
    uint16_t ramp_arr_target = g_target_interval_us * 84;

    // 1. Sprawdzenie, czy inna funkcja właśnie zażądała uruchomienia nowej rampy
    if (*p_trigger_flag == 1 && !ramp_in_progress)
    {
        current_arr = RAMP_ARR_START;
        last_ramp_time = HAL_GetTick();
        ramp_in_progress = 1;

        // // Ustawienie wartości początkowej do rejestrów TIM
        // __HAL_TIM_SET_AUTORELOAD(&htim4, current_arr);
        // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, current_arr / 2); // Wypełnienie 50%
    }

    // 2. Jeśli proces przyspieszania trwa
    if (ramp_in_progress)
    {
        uint32_t current_time = HAL_GetTick();

        // Sprawdzamy, czy upłynął zadany czas (np. 2ms) od ostatniej modyfikacji
        if ((current_time - last_ramp_time) >= RAMP_INTERVAL_MS)
        {
            last_ramp_time = current_time;

            // Zmniejszanie ARR w STM32 oznacza zwiększanie częstotliwości (silnik przyspiesza)
            if (current_arr > ramp_arr_target)
            {
                if (current_arr > (ramp_arr_target + RAMP_ARR_STEP)) {
                    current_arr -= RAMP_ARR_STEP;
                } else {
                    current_arr = ramp_arr_target;
                }

                // Bezpieczny wpis do rejestrów w locie (wymaga włączonego Preload)
                __HAL_TIM_SET_AUTORELOAD(&htim4, current_arr);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, current_arr / 2);
                TIM4->EGR |= TIM_EGR_UG;

            }
            else
            {
                // Osiągnięto prędkość docelową: wygaszamy flagę sterującą oraz flagę wewnętrzną
                ramp_in_progress = 0;
                // *p_trigger_flag = 0; 
            }
        }
    }
}

void StallGuard_Masking_Process(void)
{
    static uint32_t movement_start_time = 0;
    static uint8_t watch_started = 0;

    // 1. Wykrycie momentu, w którym silnik dostał sygnał startu ruchu
    if (g_started == 1 && !watch_started)
    {
        g_stallguard_active = 0;        // Blokujemy StallGuard (ignorujemy pin DIAG)
        movement_start_time = HAL_GetTick(); // Zapisujemy czas w ms, w którym silnik ruszył
        watch_started = 1;              // Zapamiętujemy, że odliczanie trwa
    }

    // 2. Odliczanie czasu maskowania przy użyciu SysTick
    if (watch_started)
    {
        // Sprawdzamy czy minął bezpieczny czas na rozpędzenie i stabilizację cewek silnika
        if ((HAL_GetTick() - movement_start_time) >= STALLGUARD_MASK_TIME_MS)
        {
            g_stallguard_active = 1;    // Odblokowujemy StallGuard – teraz uderzenie w ścianę zatrzyma silnik
            watch_started = 0;          // Resetujemy wewnętrzny strażnik funkcji
        }
    }
}
