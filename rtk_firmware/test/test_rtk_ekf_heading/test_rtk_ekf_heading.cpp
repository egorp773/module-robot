#include <cassert>
#include <cmath>
#include <cstdio>

#include "../../lib/StateEstimator/src/RtkEkf.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDt = 0.1f;
constexpr float kDistance = 0.1f;
constexpr float kEps = 1e-4f;

void assertNear(float actual, float expected) {
    assert(std::fabs(actual - expected) < kEps);
}

void testHeadingZeroMovesNorth() {
    RtkEkf ekf;
    ekf.begin();
    ekf.reset(0.0f, 0.0f, 0.0f);

    ekf.predict(kDt, 1.0f, 0.0f, 0.0f);

    assertNear(ekf.x(), 0.0f);
    assertNear(ekf.y(), kDistance);
}

void testHeadingNinetyMovesEast() {
    RtkEkf ekf;
    ekf.begin();
    ekf.reset(0.0f, 0.0f, 0.5f * kPi);

    ekf.predict(kDt, 1.0f, 0.0f, 0.0f);

    assertNear(ekf.x(), kDistance);
    assertNear(ekf.y(), 0.0f);
}

}  // namespace

int main() {
    testHeadingZeroMovesNorth();
    testHeadingNinetyMovesEast();
    std::puts("RtkEkf heading convention tests passed");
    return 0;
}
