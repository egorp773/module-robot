#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>

uint32_t millis();

template <typename T>
inline T max(T a, T b) {
    return std::max(a, b);
}

template <typename T>
inline T min(T a, T b) {
    return std::min(a, b);
}

// Minimal host-side Serial surface used by diagnostics inside otherwise
// deterministic estimator tests.  It deliberately has no buffering or
// timing behaviour; firmware UART behaviour is outside these host tests.
struct HostSerialStub {
    template <typename... Args>
    int printf(const char* format, Args... args) {
        return std::printf(format, args...);
    }
};

inline HostSerialStub Serial;
