// ----------------------------------------------------------------------------
// mydriver.cpp
//
// Implementacja drivera silnika krokowego z TMC2209 w trybie StallGuard.
// Silnik sprzężony z potencjometrem wieloobrotowym.
// ----------------------------------------------------------------------------

#include "mydriver.h"

#define PRINT(x) Serial.println(x); Serial.flush()

MyDriver::MyDriver()
{
    total_range_ms_         = 0;
    left_adc_mv_           = 0;
    right_adc_mv_          = 0;
    voltage_increases_right_ = true;
    step_percent_          = 0.01;   // domyślnie 1%
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



// ============================================================================
//  begin() — inicjalizacja TMC2209, ADC, pinów
// ============================================================================

bool MyDriver::begin()
{
    // --- Pin DIAG (wejście z pull-up, aktywny LOW) ---
    pinMode(DIAG_PIN, INPUT_PULLDOWN);

    // --- Opcjonalne piny DIR / STEP / EN ---
    // pinMode(DIR_PIN, OUTPUT);
    // pinMode(STEP_PIN, OUTPUT);
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

    digitalWrite(ENABLE_PIN, HIGH);   // HIGH = wyłączony (aktywny LOW)
    stepper_driver_.setHardwareEnablePin(ENABLE_PIN);

    // --- Konfiguracja ADC ---
    // Użytkownik dostosowuje ADC_MAX_VALUE / ADC_REF_MV w nagłówku.
    //pinMode(ADC_PIN, INPUT);
    analogReadResolution(12);

    clearStallDiagPin();
    // --- Inicjalizacja TMC2209 przez UART ---
    stepper_driver_.setup(MYDRIVER_SERIAL, MYDRIVER_SERIAL_BAUD);

    if (!stepper_driver_.isSetupAndCommunicating())
    {
        return false;
    }

    // --- Prąd roboczy / spoczynkowy ---
    stepper_driver_.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver_.setHoldCurrent(HOLD_CURRENT_PERCENT);

    // --- StallGuard ---
    stepper_driver_.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
    // TCOOLTHRS musi być wysokie, aby StallGuard był aktywny przy niskich prędkościach
    stepper_driver_.setCoolStepDurationThreshold(TCOOLTHRS_VALUE);

    // --- StealthChop (cichy tryb) ---
    stepper_driver_.enableStealthChop();

    // --- Mikrokroki: 256 (najpłynniej) ---
    stepper_driver_.setMicrostepsPerStep(256);//256

    // --- Wyłącz silnik na start ---
    stepper_driver_.disable();

    return true;
}

// ============================================================================
//  calibrate() — wykrycie skrajów potencjometru
// ============================================================================

bool MyDriver::calibrate()
{
    if (!stepper_driver_.isSetupAndCommunicating())
    {
        return false;
    }

    stepper_driver_.enable();
    adcOnOff(true);
    outOnOff(false);

    // --- Krok 1: jedź w prawo (dodatnia prędkość) do stalla ---
    moveUntilStall(CALIBRATION_VELOCITY);
    delay(500);
    right_adc_mv_ = readAdcMv();
    PRINT("RIGHT ADC:"); PRINT(right_adc_mv_);

    // --- Krok 2: jedź w lewo (ujemna prędkość) do stalla, mierz czas ---
    uint32_t start_ms = millis();
    moveUntilStall(-CALIBRATION_VELOCITY);
    uint32_t end_ms = millis();
    total_range_ms_ = end_ms - start_ms;

    delay(500);
    left_adc_mv_ = readAdcMv();
    PRINT("LEFT ADC:"); PRINT(left_adc_mv_);


    // --- Krok 3: określ kierunek wzrostu napięcia ---
    voltage_increases_right_ = (right_adc_mv_ >= left_adc_mv_);
    PRINT("RIGHT INCREASE:"); PRINT(voltage_increases_right_);

    // --- Wyłącz silnik po kalibracji ---
    stepper_driver_.moveAtVelocity(0);
    stepper_driver_.disable();

    adcOnOff(false);
    return true;
}

// ============================================================================
//  seekTarget() — znajdź pozycję potencjometru z napięciem ~ target_mV
// ============================================================================

bool MyDriver::seekTarget(uint16_t target_mV)
{
    if (!stepper_driver_.isSetupAndCommunicating())
    {
        PRINT("Communication error");
        return false;
    }

    if (total_range_ms_ == 0)
    {
        // Brak kalibracji — nie można wyliczyć kroku
        PRINT("No calibration");
        return false;
    }

    stepper_driver_.enable();
    adcOnOff(true);

    // --- Oblicz czas jednego kroku nastawy (step_percent_ % całego zakresu) ---
    uint32_t step_ms = (uint32_t)(total_range_ms_ * step_percent_);
    PRINT("Step ms:");
    PRINT(step_ms);
    if (step_ms == 0) step_ms = 1;

    // --- Odczytaj aktualne napięcie ---
    uint16_t current_mv = readAdcMv();

    // --- Wybierz kierunek szukania ---
    // Jeśli cel > aktualne → jedź w kierunku rosnącego napięcia
    // Jeśli cel < aktualne → jedź w kierunku malejącego napięcia
    int32_t velocity;
    if (current_mv < target_mV)
    {
        PRINT("Voltage lower than expected");
        // Jedź w kierunku rosnącego napięcia
        velocity = voltage_increases_right_ ? SEEK_VELOCITY :
            -SEEK_VELOCITY;
    }
    else
    {
        PRINT("Voltage higher than expected");
        // Jedź w kierunku malejącego napięcia
        velocity = voltage_increases_right_ ? -SEEK_VELOCITY :
            SEEK_VELOCITY;
    }

    // --- Pętla nastawy ---
    uint16_t best_mv = current_mv;
    int32_t  best_velocity = velocity;   // kierunek, w którym znaleziono najlepsze
    bool     overshot = false;

    PRINT(velocity);
    for (uint16_t i = 0; i < MAX_SEEK_STEPS; ++i)
    {
        // Wykonaj krok (ruch przez step_ms)
        stepper_driver_.moveAtVelocity(velocity);
        delay(step_ms);
        stepper_driver_.moveAtVelocity(0);
        delay(250);  // 50 czas na ustalenie napięcia // PATLAS

        // Odczytaj napięcie
        current_mv = readAdcMv();

        // Sprawdź czy przekroczyliśmy wartość zadaną
        bool was_below = (best_mv < target_mV);
        bool now_above = (current_mv >= target_mV);

        // Aktualizuj najlepsze przybliżenie
        uint16_t best_err   = (best_mv > target_mV) ? (best_mv - target_mV) : (target_mV - best_mv);
        uint16_t current_err = (current_mv > target_mV) ? (current_mv - target_mV) : (target_mV - current_mv);
        PRINT(current_mv);
        PRINT(current_err);
        PRINT(best_err);
        PRINT(i);

        if (current_err < best_err)
        {
            best_mv = current_mv;
            best_velocity = velocity;
        }

        // Czy przekroczyliśmy cel?
        // Jeśli szliśmy w górę i teraz jesteśmy powyżej celu → overshot
        // Jeśli szliśmy w dół i teraz jesteśmy poniżej celu → overshot
        if (velocity > 0 && voltage_increases_right_ && current_mv >= target_mV)
        {
            overshot = true;
        }
        else if (velocity > 0 && !voltage_increases_right_ && current_mv <= target_mV)
        {
            overshot = true;
        }
        else if (velocity < 0 && voltage_increases_right_ && current_mv <= target_mV)
        {
            overshot = true;
        }
        else if (velocity < 0 && !voltage_increases_right_ && current_mv >= target_mV)
        {
            overshot = true;
        }

        if (overshot)
        {
            // Cofnij o jeden krok w przeciwnym kierunku
            stepper_driver_.moveAtVelocity(-velocity);
            delay(step_ms);
            stepper_driver_.moveAtVelocity(0);
            delay(50);

            // Odczytaj po cofnięciu
            current_mv = readAdcMv();
            uint16_t err_after = (current_mv > target_mV) ? (current_mv - target_mV) : (target_mV - current_mv);

            // Wybierz lepszą pozycję: przed cofnięciem czy po
            if (err_after < best_err)
            {
                best_mv = current_mv;
            }
            PRINT("Voltage set:");
            PRINT(best_mv);
            break;  // znaleźliśmy najlepsze przybliżenie
        }

        // Sprawdź stall (zabezpieczenie)
        if (isStalled())
        {
            PRINT("STALLED!");
            break;
        }
    }

    // --- Wyłącz silnik po znalezieniu pozycji ---
    stepper_driver_.moveAtVelocity(0);
    stepper_driver_.disable();
    adcOnOff(false);

    return true;
}

//============================================================================
//  moveUntilStall() — kręć silnikiem aż DIAG = LOW (stall)
//============================================================================

uint32_t MyDriver::moveUntilStall(int32_t velocity)
{
    stepper_driver_.moveAtVelocity(velocity);

    // Czas rozbiegu — ignoruj DIAG przez pierwsze STALL_STARTUP_SKIP_MS
    uint32_t start_ms = millis();
    while ((millis() - start_ms) < STALL_STARTUP_SKIP_MS)
    {
        // czekaj na rozbieg
    }

    // Czekaj na stall (DIAG = HIGH)
    uint32_t stall_start = 0;
    bool stalled = false;

    while (true)
    {
        // PRINT(stepper_driver_.getStallGuardResult());
        int dp = digitalRead(DIAG_PIN);
        // PRINT("DIAG PIN: ");
        // PRINT(dp);
        // delay(500);
        if (dp == HIGH)
        {
            //PRINT("DIAG LOW");
            if (stall_start == 0)
            {
                stall_start = millis();
                clearStallDiagPin();
            }
            else if ((millis() - stall_start) >= STALL_DEBOUNCE_MS)
            {
                stalled = true;
                break;
            }
            clearStallDiagPin();
        }
        else
        {
            stall_start = 0;  // reset debounce
        }

        // Zabezpieczenie timeout (max 60 s)
        if ((millis() - start_ms) > 60000)
        {
            break;
        }
    }
    clearStallDiagPin();
    uint32_t ret = (millis() - start_ms);
    stepper_driver_.moveAtVelocity(0);
    delay(300);

    return ret;
}

//============================================================================
//  readAdcMv() — odczyt napięcia z ADC [mV], uśredniony
//============================================================================

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
    return (uint16_t)(((uint32_t)raw * ADC_REF_MV) /
            (uint32_t)ADC_MAX_VALUE);
}

// ============================================================================
//  isStalled() — sprawdź pin DIAG
// ============================================================================

bool MyDriver::isStalled()
{
    return (digitalRead(DIAG_PIN) == LOW);
}

void MyDriver::clearStallDiagPin()
{
    digitalWrite(ENABLE_PIN, HIGH);
    delay(1);
    digitalWrite(ENABLE_PIN, LOW);
}

// ============================================================================
//  Sterowanie silnikiem — publiczne wrapper-y
// ============================================================================

void MyDriver::moveAtVelocity(int32_t velocity)
{
    digitalWrite(ENABLE_PIN, LOW);
    stepper_driver_.moveAtVelocity(velocity);
}

void MyDriver::stop()
{
    stepper_driver_.moveAtVelocity(0);
}

void MyDriver::enableMotor()
{
    stepper_driver_.enable();
}

void MyDriver::disableMotor()
{
    stepper_driver_.moveAtVelocity(0);
    stepper_driver_.disable();
}
