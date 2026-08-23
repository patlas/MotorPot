//----------------------------------------------------------------------------
// mydriver.h
//
// Driver do sterowania silnikiem krokowym przez TMC2209 w trybie StallGuard.
// Silnik sprzężony jest z potencjometrem wieloobrotowym.
// Procedura: kalibracja (wykrycie skrajów) → nastawa (szukanie pozycji
// potencjometru dającej napięcie ADC najbliższe wartości zadanej w mV).
//----------------------------------------------------------------------------

#ifndef MYDRIVER_H
#define MYDRIVER_H

#include <Arduino.h>
#include <TMC2209.h>

//============================================================================
//  KONFIGURACJA UŻYTKOWNIKA  —  podmień wartości pod swój procesor /sprzęt
//============================================================================

// --- UART do TMC2209 ---
#define MYDRIVER_SERIAL          Serial1
#define MYDRIVER_SERIAL_BAUD     115200

// --- Pin DIAG TMC2209 (open-drain, aktywny LOW gdy stall) ---
//   Połącz DIAG TMC2209 → ten GPIO. Wewn. pull-up włączony w kodzie.
#define DIAG_PIN                2 //P0.8
#define STEP_PIN                6 //P2.0
#define OUT_EN_OFF              3 //P0.9
#define OUT_EN_ON               1 //P0.7
#define OUT_REV_OFF             9 //P0.0 
#define OUT_REV_ON              8 //P0.5
#define ADC_EN_ON               4 //P0.14
#define ADC_EN_OFF              5 //P0.15
#define ENABLE_PIN              0 //P0.6
#define ADC_PIN                  A0

#define ADC_MAX_VALUE            4095.0
#define ADC_REF_MV               3300

// --- Parametry silnika ---
#define HOLD_CURRENT_PERCENT    10       // prąd spoczynkowy (%)
#define STALL_GUARD_THRESHOLD   40      //15 próg SGTHRS (0-255); wyżej =czulej
#define TCOOLTHRS_VALUE         0xFFFFE  //0xFFFFF// musi być wysokie, być StallGuard aktywny

// --- Konfiguracja Prędkości (Opóźnienia impulsów w mikrosekundach) ---
/* IF PRINT enabled
#define STEP_PULSE_DELAY_US      2//5//33  // Domyślna prędkość bazowa (np. dla kalibracji)
#define COARSE_DELAY_US          5//33  // Prędkość fazy zgrubnej (mniejsza wartość = szybszy ruch)
#define FINE_DELAY_US            15//100 // Prędkość fazy precyzyjnej (większa wartość = wolniejszy ruch)
#define STALL_GUARD_THRESHOLD   15      //15 próg SGTHRS (0-255); wyżej =czulej

*/

#define STEP_PULSE_DELAY_US      3//5//33  // Domyślna prędkość bazowa (np. dla kalibracji)
#define COARSE_DELAY_US          6//33  // Prędkość fazy zgrubnej (mniejsza wartość = szybszy ruch)
#define FINE_DELAY_US            15//100 // Prędkość fazy precyzyjnej (większa wartość = wolniejszy ruch)

// --- Parametry Algorytmu Pozycjonowania (Liczba kroków i limity) ---
#define COARSE_MARGIN_STEPS      15  // Margines bezpieczeństwa (10-20 kroków) przed celem liniowym
#define MAX_FINE_STEPS           1500 // Maksymalna liczba pojedynczych kroków w fazie precyzyjnej

// --- Konfiguracja Przetwornika ADC i Przekaźników ---
#define RELAY_DELAY              100  // Czas podtrzymania cewki przekaźnika impulsowego [ms]

// --- Parametry Sprzętowe Sterownika TMC2209 ---
#define MYDRIVER_SERIAL          Serial1
#define MYDRIVER_SERIAL_BAUD     115200
#define RUN_CURRENT_PERCENT      90

#define FINE_TUNE_TOLERANCE_PCT  2.0f // Dopuszczalny błąd końcowy w procentach (np. 5%)

// =========================================================================

enum class MotorDirection {
    Left,
    Right
};

struct StepCounterBounds {
    uint32_t current_steps;
    uint32_t lower_bound;
    uint32_t upper_bound;
};

class MyDriver {
public:
    MyDriver();
    bool begin();
    bool calibrate();
    //bool seekTarget(uint16_t target_mV);
    int16_t seekTarget(uint16_t target_mV, bool ultraFineTunning=false);

    
    uint32_t getTotalCalibrationSteps() const { return total_calibration_steps_; }
    uint32_t getCurrentPositionSteps() const { return bounds_.current_steps; }

    void enableMotor();
    void disableMotor();

    void adcOnOff(bool on);
    void outReverse(bool rev);
    void outOnOff(bool on);
    uint16_t readAdcMv();

private:
    TMC2209 stepper_driver_;
    
    uint32_t total_calibration_steps_;
    uint16_t left_adc_mv_;
    uint16_t right_adc_mv_;
    float    step_percent_;
    bool isCallibrated;

    StepCounterBounds bounds_; 

    uint32_t moveUntilStall(MotorDirection dir);
    bool     executeSteps(uint32_t steps_to_move, MotorDirection dir, StepCounterBounds& bounds, bool handle_stall, uint32_t delay_us);
    

    uint16_t adcToMv(uint16_t raw);
    void     clearStallDiagPin();
};

#endif // MYDRIVER_H
