#include "tmc2209.h"
#include "TMC2209.h"
#include "stm32f4xx_hal_def.h"
// --- Parametry silnika ---
#define HOLD_CURRENT_PERCENT    10       // prąd spoczynkowy (%)
#define STALL_GUARD_THRESHOLD   100      //15 próg SGTHRS (0-255); wyżej =czulej
#define TCOOLTHRS_VALUE         0xFFFFE  //0xFFFFF// musi być wysokie, być StallGuard aktywny
// #define TCOOLTHRS_VALUE         0x20  //0xFFFFF// musi być wysokie, być StallGuard aktywny

// --- Parametry Sprzętowe Sterownika TMC2209 ---
#define MYDRIVER_SERIAL_BAUD     115200
#define RUN_CURRENT_PERCENT      90

TMC2209 stepper_driver_;

HAL_StatusTypeDef TMC2209_Init(void)
{
    stepper_driver_.setup(MYDRIVER_SERIAL_BAUD, TMC2209::SERIAL_ADDRESS_0);
    if (!stepper_driver_.isSetupAndCommunicating())
    {
        stepper_driver_.isReady_ = 0;
        return HAL_ERROR;
    }
    stepper_driver_.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver_.setHoldCurrent(HOLD_CURRENT_PERCENT);
    stepper_driver_.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
    stepper_driver_.setCoolStepDurationThreshold(TCOOLTHRS_VALUE);
    stepper_driver_.enableStealthChop();
    // stepper_driver_.disableStealthChop();
    stepper_driver_.setMicrostepsPerStep(16); //256
    stepper_driver_.moveUsingStepDirInterface(); 
    stepper_driver_.setHoldDelay(1); 
    stepper_driver_.disableCoolStep();
    stepper_driver_.setPowerDownDelay(20);
    stepper_driver_.setStealthChopDurationThreshold(0);
    stepper_driver_.setStandstillMode(TMC2209::FREEWHEELING);
    stepper_driver_.clearDriveError();
    // stepper_driver_.disable();

    stepper_driver_.isReady_ = 1;
    return HAL_OK;
}

uint8_t TMC2209_IsReady(void) { return stepper_driver_.isReady_;}

void TMC2209_Enable(void)
{
    stepper_driver_.enable();
}

void TMC2209_Disable(void)
{
    stepper_driver_.disable();
}

void TMC2209_SetDirectionInverted(uint8_t inverted)
{
    if(inverted)
    {
        stepper_driver_.enableInverseMotorDirection();
    }
    else
    {
        stepper_driver_.disableInverseMotorDirection();
    }
}
