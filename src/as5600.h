#pragma once
#include <Wire.h>
#include <cstdint>
#include <Arduino.h>
#include "config.h"

class AS5600 
{
public:
    explicit AS5600(uint8_t channel=0, const char *name="xyz_sensor");

    bool init();
    uint16_t readAbsPosition() const;
    uint16_t calcMappedAbsPosition() const;
    float readPositionDeg() const;
    int16_t deltaMicrosteps(uint16_t start_steps, uint16_t end_steps) const;
    float measureRPM(int16_t delta_microsteps, uint32_t delta_time_ms) const;
    void printTelemetry(float gear_ratio = 1.0f) const;
    bool isFound() const { return found_; }

private:
    void selectChannelTCA9548A(uint8_t channel) const;

    const char *name_;
    const uint8_t channel_;
    bool found_ = false;
};
