#pragma once
#include <cstdint>
#include <atomic>

namespace cfg {
    constexpr uint8_t TCA9548A_ADDR = 0x70;      // Address of the I2C multiplexer. A0-A2 grounded
    constexpr uint8_t AS5600_ADDR = 0x36;        // Address of the AS5600 encoder chip

    constexpr uint32_t GEAR_RATIO = 94.0f / 3.0f;  // 31 + 1/3
    constexpr uint8_t  MICROSTEPS = 8;
    constexpr bool STEALTHCHOP_ENABLED = false;
    constexpr uint8_t FULL_STEPS_PER_REV = 200;
    constexpr uint16_t STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS;
    constexpr uint32_t ONE_FULL_ROTATION_STEPS = STEPS_PER_REV * GEAR_RATIO;
    constexpr uint32_t MICROSTEPS_PER_DEGREE = STEPS_PER_REV * GEAR_RATIO / 360.0f;
    constexpr uint32_t BAUD_RATE = 115200;

    constexpr uint8_t RUN_CURRENT_PCT = 100;
    constexpr uint8_t HOLD_CURRENT_PCT = 20;

    // 'atomic' used to allow safe concurrent access from multiple threads
    inline std::atomic<std::uint16_t> speed{5000};    // microsteps per second
}