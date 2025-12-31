#include "stepperMotor.h"
#include <TeensyThreads.h>

namespace {
    // Helpers
    static uint8_t rawToPercent(uint8_t raw0_31)
    {
        return (uint8_t)lroundf((raw0_31 / 31.0f) * 100.0f);
    }

    auto diff(int a, int b) 
    { 
        return abs(a - b); 
    }
}

StepperMotor::StepperMotor(
    const uint8_t step_pin,
    const uint8_t dir_pin,
    const uint8_t endstop_pin,
    const uint8_t en_pin,
    const char *name,
    const float gear_ratio,
    HardwareSerial &uart,
    TMC2209::SerialAddress serial_addr)
    : stepper(AccelStepper::DRIVER, step_pin, dir_pin),
      gear_ratio(gear_ratio),
      driver_(),
      uart_(uart),
      serial_addr_(serial_addr),
      name_(name),
      step_pin_(step_pin),
      dir_pin_(dir_pin),
      endstop_pin_(endstop_pin),
      en_pin_(en_pin)
{}

void StepperMotor::init()
{
    pinMode(step_pin_, OUTPUT);
    pinMode(dir_pin_, OUTPUT);

    // TODO: support for EN pin tied LOW
    if (en_pin_ != 0xFF)
        pinMode(en_pin_, OUTPUT);

    if (endstop_pin_ != 0xFF)
    {
        pinMode(endstop_pin_, INPUT_PULLUP);  
    }

    stepper.setMinPulseWidth(3);
    stepper.setMaxSpeed(static_cast<float>(cfg::speed.load(std::memory_order_relaxed))); // steps/s
    stepper.setAcceleration(20.0f * cfg::STEPS_PER_REV);
    stepper.setCurrentPosition(0);

    driver_.setup(uart_, cfg::BAUD_RATE, serial_addr_);
    driver_.setReplyDelay(5);                           // adjust to 3–5 if wires are long/noisy
    driver_.moveUsingStepDirInterface();                // external STEP/DIR mode

    applyConfig();
    startEndstopMonitor();
}

void StepperMotor::applyConfig()
{
    found_ = driver_.isSetupAndCommunicating();
    Serial.print(F("[TMC2209] "));
    Serial.print(found_ ? F("") : F("Error 001! "));
    Serial.print(name_);
    Serial.println(found_ ? F(" - detected.") : F(" - NOT found."));

    driver_.moveAtVelocity(0);                      // keep internal generator idle

    driver_.setMicrostepsPerStep(cfg::MICROSTEPS);
    driver_.setRunCurrent(cfg::RUN_CURRENT_PCT);    // %
    driver_.setHoldCurrent(cfg::HOLD_CURRENT_PCT);  // %
    if (cfg::STEALTHCHOP_ENABLED)
    {
        driver_.enableStealthChop();
    }
    else
    {
        driver_.disableStealthChop(); 
    }
    driver_.enable();

    delay(3);
}

bool StepperMotor::settingsMismatch()
{
    // Compare current driver settings vs desired ones (with tolerance for percent rounding)
    const uint8_t tolerance = 3;

    uint16_t ms = driver_.getMicrostepsPerStep();
    if (ms != cfg::MICROSTEPS)
        return true;

    TMC2209::Settings s = driver_.getSettings();
    uint8_t irun_pct = rawToPercent(s.irun_register_value);
    uint8_t ihold_pct = rawToPercent(s.ihold_register_value);

    if (diff(irun_pct, cfg::RUN_CURRENT_PCT) > tolerance)
        return true;
    if (diff(ihold_pct, cfg::HOLD_CURRENT_PCT) > tolerance)
        return true;

    return false;
}

void StepperMotor::refreshConfigIfNeeded()
{
    if (!driver_.isSetupAndCommunicating())
    {
        Serial.println(F("[TMC2209] "));
        Serial.print(name_);
        Serial.println(F(" - Error 002! Communication lost or driver reset. Reinitializing..."));
        
        driver_.setup(uart_, cfg::BAUD_RATE, serial_addr_);
        driver_.setReplyDelay(5);
        applyConfig();

        return;
    }

    // Snapshot BEFORE (for printout), then decide
    uint16_t ms_before = driver_.getMicrostepsPerStep();
    TMC2209::Settings s_before = driver_.getSettings();

    if (settingsMismatch())
    {

        Serial.print(F("[TMC2209] "));
        Serial.print(name_);
        Serial.println(F(" - Error 003! Detected default/changed settings. Re-applying..."));

        Serial.print(F("Before -> microsteps="));
        Serial.print(ms_before);
        Serial.print(F(", IRUN%~"));
        Serial.print(rawToPercent(s_before.irun_register_value));
        Serial.print(F(", IHOLD%~"));
        Serial.println(rawToPercent(s_before.ihold_register_value));

        applyConfig();

        // Read AFTER and print
        uint16_t ms_after = driver_.getMicrostepsPerStep();
        TMC2209::Settings s_after = driver_.getSettings();
        Serial.print(F("After  -> microsteps="));
        Serial.print(ms_after);
        Serial.print(F(", IRUN%~"));
        Serial.print(rawToPercent(s_after.irun_register_value));
        Serial.print(F(", IHOLD%~"));
        Serial.println(rawToPercent(s_after.ihold_register_value));
        Serial.println();
    }
}

void StepperMotor::printTelemetry()
{
    // For some reason getting [.isSetupAndCommunicating(), tstep, sg or itc] stoping [irun, ihold and standstill]
    // Probably a bug in the TMC2209 library or bus is busy/blocked

    uint16_t microsteps = driver_.getMicrostepsPerStep();
    // uint32_t tstep = driver_.getInterstepDuration();        // smaller = faster
    // uint16_t sg = driver_.getStallGuardResult();
    // uint8_t itc = driver_.getInterfaceTransmissionCounter();

    TMC2209::Settings s = driver_.getSettings(); 
    TMC2209::Status st = driver_.getStatus();
    Serial.print(F("[TMC2209] "));
    Serial.print(name_);
    Serial.print(F(" - found: "));
    Serial.println(found_ ? F("yes") : F("no"));

    Serial.print("Endstop triggered: ");
    Serial.println(endstop_triggered_);

    Serial.print(F("Microsteps/step: "));
    Serial.println(microsteps);

    Serial.print(F("IRUN raw="));
    Serial.print(s.irun_register_value);
    Serial.print(F(" (~"));
    Serial.print(rawToPercent(s.irun_register_value));
    Serial.println(F("%)"));

    Serial.print(F("IHOLD raw="));
    Serial.print(s.ihold_register_value);
    Serial.print(F(" (~"));
    Serial.print(rawToPercent(s.ihold_register_value));
    Serial.println(F("%)"));

    // Serial.print(F("tSTEP: "));
    // Serial.println(tstep);
    // Serial.print(F("SG_RESULT: "));
    // Serial.println(sg);
    // Serial.print(F("TX counter: "));
    // Serial.println(itc);

    Serial.print(F("Status: standstill="));
    Serial.print(st.standstill);
    Serial.print(F(", stealthChopMode="));
    Serial.print(st.stealth_chop_mode);
    Serial.print(F(", overTempWarn="));
    Serial.println(st.over_temperature_warning);
}

void StepperMotor::startEndstopMonitor(uint32_t sample_period_ms)
{
    // When endstop is turned off OR endstop has already started
    if (endstop_pin_ == 0xFF || endstop_thread_id_ >= 0)
        return;

    // Check in case of 0 and assign minimum
    endstop_sample_period_ms_ = sample_period_ms ? sample_period_ms : 1;
    // Start thread to monitor endstop state and pass 'this' pointer as argument so static thunk can call member function (method)
    endstop_thread_id_ = threads.addThread(endstopMonitorThunk_, this);
}

// Static thunk to call member function (method) from thread. They need to be static or free functions. Thats why we pass 'this' pointer as argument.
// A thunk is a small adapter function that receives a generic argument (void*), converts it back to the correct object type, and then calls object logic.
// Thread needs this type of function becouse memberfunctions have hidden 'this' pointer as first argument.
void StepperMotor::endstopMonitorThunk_(void *arg)
{

    // 'arg' is actually 'this' pointer - StepperMotor*
    // 'static_cast' is used to convert 'void*' back to 'StepperMotor*'
    // 'self' is now a pointer to the StepperMotor instance
    auto *self = static_cast<StepperMotor *>(arg);

    constexpr uint8_t DEBOUNCE_COUNT = 5;  // Must read same value 5 times in a row
    uint8_t stable_count = 0;
    bool last_reading = false;
    
    while (true)
    {
        bool current_reading = (digitalRead(self->endstop_pin_) == HIGH);     // Active LOW
        
        if (current_reading == last_reading)
        {
            stable_count++;
            if (stable_count >= DEBOUNCE_COUNT)
            {
                self->endstop_triggered_ = current_reading;
                stable_count = DEBOUNCE_COUNT;  // Prevent overflow
            }
        }
        else
        {
            stable_count = 0;
            last_reading = current_reading;
        }

        threads.delay(self->endstop_sample_period_ms_);
    }
}

void StepperMotor::home()
{
    Serial.print(F("[TMC2209] "));
    Serial.print(name_);
    Serial.println(F(" - Starting homing procedure..."));

    stepper.moveTo(-cfg::ONE_FULL_ROTATION_STEPS * 1);  

    while (stepper.distanceToGo() != 0)
    {
        if (endstop_triggered_)
        {
            stepper.stop();
            stepper.setCurrentPosition(0);
            hommed_ = true;
            
            Serial.print(F("[TMC2209] "));
            Serial.print(name_);
            Serial.println(F(" - Homing complete."));
            
            delay(100);
            return;  
        }
        stepper.run();
        threads.yield();  
    }

    Serial.print(F("[TMC2209] "));
    Serial.print(name_);
    Serial.println(F(" - Error 004. Homing failed, endstop not triggered."));
}