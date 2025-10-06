#pragma once
#include <cstdint>
#include <atomic>

namespace cfg {
    constexpr uint8_t  MICROSTEPS = 8;
    constexpr uint8_t FULL_STEPS_PER_REV = 200;
    constexpr uint16_t STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS;
    constexpr uint32_t BAUD_RATE = 115200;

    constexpr uint8_t RUN_CURRENT_PCT = 100;
    constexpr uint8_t HOLD_CURRENT_PCT = 20;

    inline std::atomic<std::uint16_t> speed{5000};    // microsteps per second

    // Tiny helpers (optional)
    // inline void set_speed(std::uint32_t s) {
    //     if (s > SPEED_MAX)
    //         s = SPEED_MAX;
     //     speed.store(s, std::memory_order_relaxed);
    // }
    // inline std::uint32_t get_speed()
    // {
    //     return speed.load(std::memory_order_relaxed);
    // }
    // inline void inc_speed(std::uint32_t delta)
    // {
    //     speed.fetch_add(delta, std::memory_order_relaxed);
    // }
    // inline void dec_speed(std::uint32_t delta)
    // {
    //     speed.fetch_sub(delta, std::memory_order_relaxed);
    // }
}