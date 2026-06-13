#pragma once

#include <chrono>
#include <cstdint>

static inline auto
current_timestamp_ms () -> uint64_t
{
    return static_cast<uint64_t> (
        std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now ().time_since_epoch ())
            .count ());
}
