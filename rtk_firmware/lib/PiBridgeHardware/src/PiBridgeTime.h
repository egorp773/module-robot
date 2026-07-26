#pragma once

#include <stdint.h>

namespace pibridge {

// Standard modulo-uint32 elapsed time. Callers must snapshot `then_ms` before
// `now_ms`; an ordering violation deliberately remains a huge/stale age.
constexpr uint32_t elapsedAgeMs(uint32_t now_ms, uint32_t then_ms) {
    return now_ms - then_ms;
}

static_assert(elapsedAgeMs(101u, 100u) == 1u, "normal age failed");
static_assert(elapsedAgeMs(100u, 101u) == 0xFFFFFFFFu,
              "out-of-order age must fail stale");
static_assert(elapsedAgeMs(3u, 0xFFFFFFFEu) == 5u, "millis wrap failed");
static_assert(elapsedAgeMs(0x80000001u, 0u) == 0x80000001u,
              "ancient feedback must remain stale");

}  // namespace pibridge
