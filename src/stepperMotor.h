#pragma once
#include <cstdint>
#include <Arduino.h>
#include <TMC2209.h>
#include <AccelStepper.h>
#include "config.h"

class StepperMotor
{
public:
    explicit StepperMotor(
        const uint8_t step_pin,
        const uint8_t dir_pin,
        const uint8_t en_pin = 0xFF,    // 0xFF => EN tied LOW 
        const char *name = "xyz_stepper_motor",
        HardwareSerial &uart = Serial1,
        TMC2209::SerialAddress serial_addr = TMC2209::SERIAL_ADDRESS_0);

    AccelStepper stepper;

    void init();            
    void applyConfig();
    bool settingsMismatch();
    void refreshConfigIfNeeded();
    void printTelemetry();
    bool isFound() const { return found_; }

private:
    TMC2209 driver_;
    HardwareSerial &uart_;
    TMC2209::SerialAddress serial_addr_;
    const char *name_;
    bool found_ = false;
    const uint8_t step_pin_, dir_pin_, en_pin_;
};
