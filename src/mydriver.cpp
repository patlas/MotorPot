//----------------------------------------------------------------------------
// mydriver.cpp
//----------------------------------------------------------------------------
#include "mydriver.h"

#define PRINT(x) Serial.println(x); Serial.flush()

volatile bool g_isStall = false;
uint16_t g_left_stall_adc_mv;  
uint16_t g_right_stall_adc_mv; 

// Funkcja obsługi przerwania (ISR)
void diagPin_isr() {
  digitalWrite(LED1, HIGH);
  // Odczytujemy stan pinu natychmiast po wystąpieniu przerwania
  if (digitalRead(DIAG_PIN) == HIGH) {
    g_isStall = true;  // Zbocze narastające -> ustawiamy zmienną
    digitalWrite(LED1, HIGH);

  } else {
    g_isStall = false; // Zbocze opadające -> zerujemy zmienną
    digitalWrite(LED1, LOW);
  }
}

MyDriver::MyDriver()
{
    total_calibration_steps_ = 0;
    left_adc_mv_ = 0;
    right_adc_mv_ = 0;
    step_percent_ = 0.01; 
    
    bounds_.lower_bound = 0;
    bounds_.upper_bound = 0xFFFFFFFF; 
    bounds_.current_steps = 0;
}

void MyDriver::adcOnOff(bool on)
{
    if (on == true)
    {
        digitalWrite(ADC_EN_OFF, LOW);
        digitalWrite(ADC_EN_ON, HIGH);
        delay(RELAY_DELAY);
        digitalWrite(ADC_EN_ON, LOW);
    }
    else
    {
        digitalWrite(ADC_EN_ON, LOW);
        digitalWrite(ADC_EN_OFF, HIGH);
        delay(RELAY_DELAY);
        digitalWrite(ADC_EN_OFF, LOW);
    }
}

void MyDriver::outReverse(bool rev)
{
    if (rev == true)
    {
        digitalWrite(OUT_REV_OFF, LOW);
        digitalWrite(OUT_REV_ON, HIGH);
        delay(RELAY_DELAY);
        digitalWrite(OUT_REV_ON, LOW);
    }
    else
    {
        digitalWrite(OUT_REV_ON, LOW);
        digitalWrite(OUT_REV_OFF, HIGH);
        delay(RELAY_DELAY);
        digitalWrite(OUT_REV_OFF, LOW);
    }
}

void MyDriver::outOnOff(bool on)
{
    if (on == true)
    {
        digitalWrite(OUT_EN_OFF, LOW);
        digitalWrite(OUT_EN_ON, HIGH);
        delay(RELAY_DELAY);
        digitalWrite(OUT_EN_ON, LOW);
    }
    else
    {
        digitalWrite(OUT_EN_ON, LOW);
        digitalWrite(OUT_EN_OFF, HIGH);
        delay(RELAY_DELAY);
        digitalWrite(OUT_EN_OFF, LOW);
    }
}

bool MyDriver::begin()
{
    //pinMode(DIAG_PIN, INPUT_PULLDOWN);
    pinMode(STEP_PIN, OUTPUT); 
    digitalWrite(STEP_PIN, LOW);

    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(OUT_EN_ON, OUTPUT);
    pinMode(OUT_EN_OFF, OUTPUT);
    pinMode(OUT_REV_ON, OUTPUT);
    pinMode(OUT_REV_OFF, OUTPUT);
    pinMode(ADC_EN_ON, OUTPUT);
    pinMode(ADC_EN_OFF, OUTPUT);

    outOnOff(false);
    adcOnOff(false);
    outReverse(false);
    
    digitalWrite(ENABLE_PIN, HIGH); 
    stepper_driver_.setHardwareEnablePin(ENABLE_PIN);

    analogReadResolution(12);
    clearStallDiagPin();

    stepper_driver_.setup(MYDRIVER_SERIAL, MYDRIVER_SERIAL_BAUD);
    if (!stepper_driver_.isSetupAndCommunicating())
    {
        return false;
    }

    stepper_driver_.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver_.setHoldCurrent(HOLD_CURRENT_PERCENT);
    stepper_driver_.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
    stepper_driver_.setCoolStepDurationThreshold(TCOOLTHRS_VALUE);
    stepper_driver_.enableStealthChop();
    stepper_driver_.setMicrostepsPerStep(256); //256
    
    stepper_driver_.moveUsingStepDirInterface(); 
    
    stepper_driver_.disable();
    return true;
}

bool MyDriver::executeSteps(uint32_t steps_to_move, MotorDirection dir, StepCounterBounds& bounds, bool handle_stall, uint32_t delay_us)
{
    if (dir == MotorDirection::Right)
    {
        stepper_driver_.disableInverseMotorDirection();
    }
    else
    {
        stepper_driver_.enableInverseMotorDirection();
    }

    for (uint32_t step = 0; step < steps_to_move; ++step)
    {
        diagPin_isr();
        if (handle_stall && g_isStall)
        {
            PRINT("STALL DETECTED FROM GLOBAL VARIABLE!");
            return false;
        }

        if (dir == MotorDirection::Right)
        {
            if (bounds.current_steps >= bounds.upper_bound)
            {
                PRINT("BOUND LIMIT EXCEEDED (UPPER)!");
                return false; 
            }
            bounds.current_steps++;
        }
        else
        {
            if (bounds.current_steps <= bounds.lower_bound)
            {
                PRINT("BOUND LIMIT EXCEEDED (LOWER)!");
                return false; 
            }
            bounds.current_steps--;
        }

        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(delay_us);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(delay_us);
    }

    return true;
}

uint32_t MyDriver::moveUntilStall(MotorDirection dir)
{
    uint32_t initial_steps = bounds_.current_steps;

    // Wykorzystanie uniwersalnej funkcji kroku ze zdefiniowanym opóźnieniem bazowym
    executeSteps(0xFFFFFFFF, dir, bounds_, true, STEP_PULSE_DELAY_US);

    uint32_t steps_done = 0;
    if (dir == MotorDirection::Right) {
        steps_done = bounds_.current_steps - initial_steps;
    } else {
        steps_done = initial_steps - bounds_.current_steps;
    }

    clearStallDiagPin();
    digitalWrite(STEP_PIN, LOW);
    delay(300);

    return steps_done; 
}

bool MyDriver::calibrate()
{
    if (!stepper_driver_.isSetupAndCommunicating()) return false;

    stepper_driver_.enable();
    adcOnOff(true);
    //outOnOff(false); // PATLAS - uncomment after testing
    outOnOff(true); // PATLAS for testing - comment in release


    bounds_.lower_bound = 0;
    bounds_.upper_bound = 0xFFFFFFFF;
    bounds_.current_steps = 0; 

    moveUntilStall(MotorDirection::Right);
    delay(500);
    right_adc_mv_ = readAdcMv();
    g_right_stall_adc_mv = right_adc_mv_; 
    PRINT("RIGHT ADC (GLOBAL):"); PRINT(g_right_stall_adc_mv);

    bounds_.lower_bound = 0;
    bounds_.upper_bound = 0xFFFFFFFF;
    bounds_.current_steps = 0xFFFFFFFF;

    total_calibration_steps_ = moveUntilStall(MotorDirection::Left);
    delay(500);
    left_adc_mv_ = readAdcMv();
    g_left_stall_adc_mv = left_adc_mv_; 
    PRINT("LEFT ADC (GLOBAL):"); PRINT(g_left_stall_adc_mv);
    PRINT("TOTAL CALIBRATION STEPS:"); PRINT(total_calibration_steps_);

    bounds_.lower_bound = 0;
    bounds_.upper_bound = total_calibration_steps_;
    bounds_.current_steps = 0; 

    stepper_driver_.disable();
    adcOnOff(false);
    return true;
}

bool MyDriver::seekTarget(uint16_t target_mV, bool ultraFineTunning=false)
{
    if (!stepper_driver_.isSetupAndCommunicating()) return false;
    if (total_calibration_steps_ == 0) return false;

    if (target_mV > g_right_stall_adc_mv) target_mV = g_right_stall_adc_mv;
    if (target_mV < g_left_stall_adc_mv)  target_mV = g_left_stall_adc_mv;

    stepper_driver_.enable();
    adcOnOff(true);

    if (ultraFineTunning == false)
    {
        // =========================================================================
        // FAZA 1: MATEMATYCZNE WYLICZENIE I SZYBKI SKOK ZGRUBNY
        // =========================================================================
        uint32_t delta_voltage_total = g_right_stall_adc_mv - g_left_stall_adc_mv;
        if (delta_voltage_total == 0) delta_voltage_total = 1; 

        uint32_t target_steps_absolute = (uint32_t)(((float)(target_mV - g_left_stall_adc_mv) / delta_voltage_total) * total_calibration_steps_);

        MotorDirection main_dir;
        uint32_t steps_to_execute_coarse = 0;

        if (target_steps_absolute > bounds_.current_steps)
        {
            main_dir = MotorDirection::Right;
            uint32_t distance = target_steps_absolute - bounds_.current_steps;
            
            if (distance > COARSE_MARGIN_STEPS) {
                steps_to_execute_coarse = distance - COARSE_MARGIN_STEPS;
            } else {
                steps_to_execute_coarse = 0; 
            }
        }
        else
        {
            main_dir = MotorDirection::Left;
            uint32_t distance = bounds_.current_steps - target_steps_absolute;
            
            if (distance > COARSE_MARGIN_STEPS) {
                steps_to_execute_coarse = distance - COARSE_MARGIN_STEPS;
            } else {
                steps_to_execute_coarse = 0;
            }
        }

        if (steps_to_execute_coarse > 0)
        {
            PRINT("Faza zgrubna - szybki skok (liczba kroków):"); PRINT(steps_to_execute_coarse);
            // Używa COARSE_DELAY_US jako prędkości szybkiej z pliku nagłówkowego
            if (!executeSteps(steps_to_execute_coarse, main_dir, bounds_, true, COARSE_DELAY_US)) {
                PRINT("Zgrubny skok przerwany awaryjnie.");
            }
            delay(50); 
        }
    }
    // =========================================================================
    // FAZA 2: PRECYZYJNE POZYCJONOWANIE (Paczki po 5 kroków do uzyskania błędu < X%)
    // =========================================================================
    PRINT("Uruchamianie fazy precyzyjnej z kryterium procentowym...");

    uint16_t current_mv = readAdcMv();
    uint16_t best_mv = current_mv;
    
    // Obliczamy całkowity zakres napięcia z kalibracji, aby móc wyznaczyć błąd procentowy
    uint32_t total_voltage_span = g_right_stall_adc_mv - g_left_stall_adc_mv;
    if (total_voltage_span == 0) total_voltage_span = 1;

    for (uint16_t i = 0; i < MAX_FINE_STEPS; ++i)
    {
        // 1. Obliczanie aktualnego błędu bezwzględnego i procentowego
        uint16_t current_err = (current_mv > target_mV) ? (current_mv - target_mV) : (target_mV - current_mv);
        float current_err_pct = ((float)current_err / target_mV) * 100.0f;

        // 2. KRYTERIUM WYJŚCIA: Jeśli błąd mieści się w założonym % (np. 5%), przerywamy sukcesem
        if (current_err_pct <= FINE_TUNE_TOLERANCE_PCT)
        {
            best_mv = current_mv;
            PRINT("Cel osiągnięty! Błąd mieści się w progu procentowym.");
            PRINT(current_err);
            PRINT(total_voltage_span);
            break;
        }

        // Aktualizacja globalnie najlepszego wyniku ze wszystkich iteracji
        uint16_t best_err = (best_mv > target_mV) ? (best_mv - target_mV) : (target_mV - best_mv);
        if (current_err < best_err) {
            best_mv = current_mv;
        }

        // 3. Dynamiczny wybór kierunku dla kolejnej paczki kroków
        MotorDirection tune_dir = (current_mv < target_mV) ? MotorDirection::Right : MotorDirection::Left;

        // 4. Wykonanie paczki 5 kroków (parametr handle_stall = false, prędkość powolna)
        if (!executeSteps(50, tune_dir, bounds_, false, FINE_DELAY_US)) {
            PRINT("Osiągnięto limit sprzętowy potencjometru.");
            break;
        }

        delay(15); // Czas na ustabilizowanie odczytu po wykonaniu kroków
        uint16_t new_mv = readAdcMv();
        uint16_t new_err = (new_mv > target_mV) ? (new_mv - target_mV) : (target_mV - new_mv);

        // 5. OBSŁUGA OVERSHOOT (Przekroczenia punktu idealnego):
        // Jeżeli po wykonaniu paczki 5 kroków kierunek uległby zmianie, oznacza to, że przeszliśmy 
        // na drugą stronę wartości zadanej. Sprawdzamy, która pozycja była lepsza i tam kończymy.
        MotorDirection next_dir = (new_mv < target_mV) ? MotorDirection::Right : MotorDirection::Left;
        
        if (next_dir != tune_dir)
        {
            PRINT("Przekroczono cel (Overshot). Wybór optymalnego punktu końcowego...");
            
            // Jeśli poprzednia pozycja dawała mniejszy błąd niż nowa (po 5 krokach), wycofujemy się o te 5 kroków
            if (current_err < new_err)
            {
                PRINT("Poprzedni krok był lepszy. Wycofywanie paczki 5 kroków.");
                MotorDirection undo_dir = (tune_dir == MotorDirection::Right) ? MotorDirection::Left : MotorDirection::Right;
                executeSteps(5, undo_dir, bounds_, false, FINE_DELAY_US);
                delay(15);
                best_mv = readAdcMv();
            }
            else
            {
                PRINT("Nowy krok jest lepszy lub równy. Pozostawanie na obecnej pozycji.");
                best_mv = new_mv;
            }
            break; // Osiągnęliśmy optymalną bliskość punktu, wychodzimy z pętli
        }

        // Jeśli idziemy w dobrą stronę i nie minęliśmy celu, kontynuujemy pętlę
        current_mv = new_mv;
        best_mv = current_mv;
    }

    PRINT("Finalne napięcie ustalone:"); PRINT(best_mv);

    stepper_driver_.disable();
    adcOnOff(false);
    return true;
}

uint16_t MyDriver::readAdcMv()
{
    uint32_t sum = 0;
    const uint8_t N = 16;
    for (uint8_t i = 0; i < N; ++i)
    {
        sum += analogRead(ADC_PIN);
    }
    uint16_t raw = (uint16_t)(sum / N);
    return adcToMv(raw);
}

uint16_t MyDriver::adcToMv(uint16_t raw)
{
    return (uint16_t)(((uint32_t)raw * ADC_REF_MV) / (uint32_t)ADC_MAX_VALUE);
}

void MyDriver::clearStallDiagPin()
{
    digitalWrite(ENABLE_PIN, HIGH);
    delay(1);
    digitalWrite(ENABLE_PIN, LOW);
}

void MyDriver::enableMotor()
{
    stepper_driver_.enable();
}

void MyDriver::disableMotor()
{
    stepper_driver_.disable();
}
