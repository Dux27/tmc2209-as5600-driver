#include "as5600.h"

namespace {
    constexpr uint8_t ANGLE_REG_HIGH = 0x0C; // Register addresses for angle high byte
    constexpr uint8_t ANGLE_REG_LOW = 0x0D;  // Register addresses for angle low byte
}

AS5600::AS5600(uint8_t i2c_addr, const char* name)
    : name(name), address_(i2c_addr) {}

bool AS5600::init() 
{
    // Detecting device 
    Wire.beginTransmission(address_);
    if (Wire.endTransmission() == 0) 
    {
        Serial.print(F("[AS5600] "));
        Serial.print(name);
        Serial.println(F(" detected."));
        found_ = true;
    }
    else 
    {
        Serial.print(F("[AS5600] "));
        Serial.print(F("Error 001! "));
        Serial.print(name);
        Serial.println(F(" NOT found."));
        found_ = false;
    }

    return found_;
}

uint16_t AS5600::readAbsPosition() const 
{
    // 1. Tell the AS5600 we want to read starting at register ANGLE_REG_HIGH
    Wire.beginTransmission(address_);
    Wire.write(ANGLE_REG_HIGH);                 // Register pointer = high byte of angle
    Wire.endTransmission(false);                // 'false' = send a RESTART, not a STOP
    // 2. Ask for 2 bytes - 16 bits of data from that register onwards
    Wire.requestFrom(int(address_), 2);
    // 3. If we got exactly 2 bytes, combine them into a 16-bit value
    if (Wire.available() == 2)
    {
        uint8_t high = Wire.read();
        uint8_t low = Wire.read();
        return (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low); // combine high+low
    }
    // 4. If something failed, return an error marker
    return 0xFFFF;
}

uint16_t AS5600::calcMappedAbsPosition() const 
{
    uint16_t raw_abs_pos = readAbsPosition();
    if (raw_abs_pos != 0xFFFF) 
    {
        // AS5600 gives 16 bits but only 12 bits are important
        uint16_t raw_abs_pos_12b = raw_abs_pos & 0x0FFF;    // ANDing with 0x0FFF keeps the lower 12 bits and sets the upper 4 bits to 0.
        uint16_t mapped_abs_pos = (static_cast<uint32_t>(raw_abs_pos_12b) * cfg::STEPS_PER_REV + (4096u / 2u)) / 4096u; // Mapping with rounding to nearest trick
        return mapped_abs_pos;
    }
    return 0xFFFF;
}
