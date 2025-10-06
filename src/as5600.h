#pragma once
#include <Wire.h>
#include <cstdint>
#include <Arduino.h>
#include "config.h"

class AS5600 
{
public:
    explicit AS5600(uint8_t i2c_addr = 0x36, const char* name = "xyz_sensor");

    const char* name;

    bool init();
    uint16_t readAbsPosition() const;
    uint16_t calcMappedAbsPosition() const;
    int16_t deltaMicrosteps(uint16_t start_steps, uint16_t end_steps);
    bool isFound() const { return found_; }

private:
    
    const uint8_t address_;
    bool found_ = false;
};
