#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "../../lib/StateEstimator/src/StateEstimator.h"

namespace {

uint32_t fakeMillis = 0;

void expectNear(float actual, float expected, float tolerance) {
    assert(std::fabs(actual - expected) <= tolerance);
}

}  // namespace

uint32_t millis() {
    return fakeMillis;
}

int main() {
    RtkEkf ekf;
    ekf.begin();
    ekf.reset(0.0f, 0.0f, 179.0f * 3.14159265358979323846f / 180.0f);
    for (int i = 0; i < 20; ++i) {
        assert(ekf.updateHeading(
            -179.0f * 3.14159265358979323846f / 180.0f,
            0.01f,
            true));
        assert(std::isfinite(ekf.covHead()));
        assert(ekf.covHead() > 0.0f);
    }
    expectNear(
        StateEstimator::normalizeDeg360(
            ekf.heading() * 180.0f / 3.14159265358979323846f),
        181.0f,
        0.2f);

    StateEstimator estimator;
    estimator.begin();
    estimator.seedHeadingDeg(0.0f);

    fakeMillis = 100;
    estimator.onImu(
        fakeMillis, 0.0f, true, 90.0f, true, 0.03f,
        ImuYawSource::ROTATION_VECTOR, true);
    assert(estimator.get().headingValid);
    assert(estimator.get().yawSource == ImuYawSource::ROTATION_VECTOR);
    assert(estimator.get().yawIsAbsolute);
    assert(estimator.get().absYawValid);
    assert(estimator.get().headingUsedByEstimator);
    expectNear(estimator.get().headingFiltDeg, 90.0f, 2.0f);

    const float beforeGpsCourse = estimator.get().headingFiltDeg;
    fakeMillis = 200;
    estimator.onPvt(
        fakeMillis,
        557512440, 376184230, 0,
        10, 20, 300, 27000000,
        3, 2, true, 15, 1.0f);
    expectNear(estimator.get().headingFiltDeg, beforeGpsCourse, 0.1f);

    StateEstimator invalidAbsoluteYaw;
    fakeMillis = 0;
    invalidAbsoluteYaw.begin();
    invalidAbsoluteYaw.seedHeadingDeg(0.0f);
    fakeMillis = 100;
    invalidAbsoluteYaw.onImu(
        fakeMillis, 0.0f, true, 90.0f, false, 0.03f,
        ImuYawSource::ROTATION_VECTOR, true);
    expectNear(invalidAbsoluteYaw.get().headingFiltDeg, 0.0f, 0.1f);
    assert(!invalidAbsoluteYaw.get().headingUsedByEstimator);

    StateEstimator badAccuracyYaw;
    fakeMillis = 0;
    badAccuracyYaw.begin();
    badAccuracyYaw.seedHeadingDeg(0.0f);
    fakeMillis = 100;
    badAccuracyYaw.onImu(
        fakeMillis, 0.0f, true, 90.0f, true, 0.6f,
        ImuYawSource::ROTATION_VECTOR, true);
    expectNear(badAccuracyYaw.get().headingFiltDeg, 0.0f, 0.1f);
    assert(!badAccuracyYaw.get().absYawValid);
    assert(!badAccuracyYaw.get().headingUsedByEstimator);

    StateEstimator gameYaw;
    fakeMillis = 0;
    gameYaw.begin();
    gameYaw.seedHeadingDeg(0.0f);
    fakeMillis = 100;
    gameYaw.onImu(
        fakeMillis, 0.0f, true, 123.0f, false, 0.5f,
        ImuYawSource::GAME_ROTATION_VECTOR, false);
    expectNear(gameYaw.get().headingFiltDeg, 0.0f, 0.1f);
    assert(gameYaw.get().yawSource == ImuYawSource::GAME_ROTATION_VECTOR);
    assert(!gameYaw.get().yawIsAbsolute);
    assert(!gameYaw.get().absYawValid);
    assert(!gameYaw.get().headingUsedByEstimator);

    StateEstimator geomagAbsolute;
    fakeMillis = 0;
    geomagAbsolute.begin();
    geomagAbsolute.seedHeadingDeg(10.0f);
    fakeMillis = 100;
    geomagAbsolute.onImu(
        fakeMillis, 0.0f, true, 12.0f, true, 0.04f,
        ImuYawSource::GEOMAGNETIC_ROTATION_VECTOR, true);
    assert(geomagAbsolute.get().absYawValid);
    assert(geomagAbsolute.get().headingUsedByEstimator);

    std::puts("StateEstimator absolute-yaw and GPS-course tests passed");
    return 0;
}
