#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

uint32_t millis();

template <typename T>
inline T max(T a, T b) {
    return std::max(a, b);
}
