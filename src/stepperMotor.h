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
        const uint8_t endstop_pin = 0xFF,        // 0xFF => no endstop
        const uint8_t en_pin = 0xFF,             // 0xFF => EN tied LOW
        const char *name = "xyz_stepper_motor",
        const float gear_ratio = 1.0f,
        HardwareSerial &uart = Serial1,
        TMC2209::SerialAddress serial_addr = TMC2209::SERIAL_ADDRESS_0);

    AccelStepper stepper;
    const float gear_ratio; 

    void init();            
    void applyConfig();
    bool settingsMismatch();
    void refreshConfigIfNeeded();
    void printTelemetry();
    bool isFound() const { return found_; }
    void startEndstopMonitor(uint32_t sample_period_ms = 2);
    uint16_t endstopBool() const { return endstop_triggered_; }

private:
    TMC2209 driver_;
    HardwareSerial &uart_;
    TMC2209::SerialAddress serial_addr_;
    const char *name_;
    bool found_ = false;
    const uint8_t step_pin_, dir_pin_, endstop_pin_, en_pin_;

    volatile uint16_t endstop_value_ = 0;
    volatile bool endstop_triggered_ = false;
    uint32_t endstop_sample_period_ms_ = 2;
    int endstop_thread_id_ = -1;
    static void endstopMonitorThunk_(void *arg);
};
