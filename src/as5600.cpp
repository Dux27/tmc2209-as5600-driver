#include "as5600.h"

namespace {
    constexpr uint8_t ANGLE_REG_HIGH = 0x0C; // Register addresses for angle high byte
    constexpr uint8_t ANGLE_REG_LOW = 0x0D;  // Register addresses for angle low byte

    auto diff(int a, int b)
    {
        return abs(a - b);
    }
}

AS5600::AS5600(uint8_t channel, const char *name)
    : name_(name), channel_(channel) {}

bool AS5600::init() 
{
    // Detecting device
    selectChannelTCA9548A(channel_);
    Wire.beginTransmission(cfg::AS5600_ADDR);

    if (Wire.endTransmission() == 0) 
    {
        Serial.print(F("[AS5600] "));
        Serial.print(name_);
        Serial.println(F(" - detected."));
        found_ = true;
    }
    else 
    {
        Serial.print(F("[AS5600] "));
        Serial.print(F("Error 001! "));
        Serial.print(name_);
        Serial.println(F(" - NOT found."));
        found_ = false;
    }

    return found_;
}

void AS5600::selectChannelTCA9548A(uint8_t channel) const
{
    if (channel > 7) 
    {
        Serial.print(F("[AS5600] "));
        Serial.print(F("Error 003! "));
        Serial.print(name_);
        Serial.println(F(" - Invalid TCA9548A channel selected."));
        return;
    }

    Wire.beginTransmission(cfg::TCA9548A_ADDR);     // Talk to the multiplexer
    Wire.write(1 << channel);                       // Choose one channel (0–7)
    Wire.endTransmission();
}

uint16_t AS5600::readAbsPosition() const 
{
    // 0. Select the correct channel on the TCA9548A multiplexer
    selectChannelTCA9548A(channel_);
    // 1. Tell the AS5600 we want to read starting at register ANGLE_REG_HIGH
    Wire.beginTransmission(cfg::AS5600_ADDR);
    Wire.write(ANGLE_REG_HIGH);                 // Register pointer = high byte of angle
    Wire.endTransmission(false);                // 'false' = send a RESTART, not a STOP
    // 2. Ask for 2 bytes - 16 bits of data from that register onwards
    Wire.requestFrom(int(cfg::AS5600_ADDR), 2);
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

float AS5600::readPositionDeg() const
{
    uint16_t raw = readAbsPosition();
    if (raw == 0xFFFF)
        return -999.0f;     // Error code

    float encoder_deg = (raw & 0x0FFF) * (360.0f / 4096.0f);
    return encoder_deg / cfg::GEAR_RATIO;
}

int16_t AS5600::deltaMicrosteps(uint16_t start_steps, uint16_t end_steps) const
{
    int32_t delta = static_cast<int32_t>(end_steps) - static_cast<int32_t>(start_steps);
    const int32_t half_rev = cfg::STEPS_PER_REV / 2;

    // Handle wrap-around
    // If delta is more than a half turn negative, it was a small positive turn
    if (delta < -half_rev) {
        delta += cfg::STEPS_PER_REV;
    }
    // If delta is more than a half turn positive, it was a small negative turn
    else if (delta > half_rev) {
        delta -= cfg::STEPS_PER_REV;
    }

    return static_cast<int16_t>(delta);
}

float AS5600::measureRPM(int16_t delta_microsteps, uint32_t delta_time_ms) const
{
    if (delta_time_ms == 0)
        return 0.0f; // Prevent division by zero

    float revolutions = static_cast<float>(delta_microsteps) / static_cast<float>(cfg::STEPS_PER_REV);
    float minutes = static_cast<float>(delta_time_ms) / 60000.0f; // Convert ms to minutes
    float RPM = revolutions / minutes;
    return RPM > 0 ? RPM : 0;                
}

void AS5600::printTelemetry(float gear_ratio) const
{
    uint16_t raw_abs_pos = readAbsPosition();
    uint16_t mapped_abs_pos_before = calcMappedAbsPosition();
    uint32_t time_before = millis();

    if (raw_abs_pos != 0xFFFF) 
    {
        Serial.print(F("[AS5600] "));
        Serial.print(name_);
        Serial.print(F(" - found: "));
        Serial.println(found_ ? F("yes") : F("no"));
        Serial.print(F("Raw absolute position: "));
        Serial.println(raw_abs_pos);

        Serial.print(F("Scaled absolute position: "));
        Serial.println(mapped_abs_pos_before);
    } 
    else 
    {
        Serial.print(F("[AS5600] "));
        Serial.print(F("Error 002! "));
        Serial.print(name_);
        Serial.println(F(" - Unable to read position."));
    }

    // calculate RPM
    delay(50);
    uint32_t time_after = millis();
    uint16_t mapped_abs_pos_after = calcMappedAbsPosition();

    uint16_t delta_microsteps = deltaMicrosteps(mapped_abs_pos_before, mapped_abs_pos_after);
    uint32_t delta_time = diff(time_before, time_after);
    float RPM = measureRPM(delta_microsteps, delta_time);

    Serial.print(F("Measured RPM"));
    Serial.print(F(" (delta_time = "));
    Serial.print(delta_time);
    Serial.println(F(" ms):"));
    Serial.print(F("Without gear reduction: "));
    Serial.print(RPM);
    Serial.println(F(" RPM"));
    Serial.print(F("With gear ratio = "));
    Serial.print(gear_ratio);
    Serial.print(F(": "));
    Serial.print(RPM / gear_ratio);
    Serial.println(F(" RPM"));
}
