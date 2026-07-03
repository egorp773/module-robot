#include <cassert>
#include <cmath>
#include <cstdio>

#include "../../lib/Imu/src/ImuMath.h"

namespace {

void expectNear(float actual, float expected, float tolerance) {
    assert(std::fabs(actual - expected) <= tolerance);
}

}  // namespace

int main() {
    expectNear(ImuMath::normalizeDeg360(-1.0f), 359.0f, 0.0001f);
    expectNear(ImuMath::normalizeDeg360(361.0f), 1.0f, 0.0001f);
    expectNear(ImuMath::wrapDeg180(181.0f), -179.0f, 0.0001f);
    expectNear(ImuMath::wrapDeg180(-181.0f), 179.0f, 0.0001f);

    expectNear(ImuMath::imuRawToRobotHeadingDeg(0.0f, 1.0f, 0.0f, 0.0f), 0.0f, 0.0001f);
    expectNear(ImuMath::imuRawToRobotHeadingDeg(90.0f, 1.0f, 0.0f, 0.0f), 90.0f, 0.0001f);
    expectNear(ImuMath::imuRawToRobotHeadingDeg(90.0f, -1.0f, 0.0f, 0.0f), 270.0f, 0.0001f);
    expectNear(ImuMath::imuRawToRobotHeadingDeg(0.0f, -1.0f, 90.0f, 0.0f), 90.0f, 0.0001f);
    expectNear(ImuMath::imuRawToRobotHeadingDeg(359.0f, 1.0f, 0.0f, 0.0f), 359.0f, 0.0001f);
    expectNear(ImuMath::imuRawToRobotHeadingDeg(361.0f, 1.0f, 0.0f, 0.0f), 1.0f, 0.0001f);

    assert(ImuMath::yawStateIsAbsolute(ImuHeadingState::IMU_ABSOLUTE_OK));
    assert(!ImuMath::yawStateIsAbsolute(ImuHeadingState::IMU_RELATIVE_ONLY));
    assert(!ImuMath::yawStateIsAbsolute(ImuHeadingState::IMU_MAG_DISTURBED));
    assert(ImuMath::yawSourceIsAbsolute(ImuYawSource::ROTATION_VECTOR));
    assert(ImuMath::yawSourceIsAbsolute(ImuYawSource::GEOMAGNETIC_ROTATION_VECTOR));
    assert(!ImuMath::yawSourceIsAbsolute(ImuYawSource::GAME_ROTATION_VECTOR));
    assert(ImuMath::canUseAbsoluteYawForNav(
        ImuHeadingState::IMU_ABSOLUTE_OK, true, 0.03f, 100u, 200u));
    assert(!ImuMath::canUseAbsoluteYawForNav(
        ImuHeadingState::IMU_RELATIVE_ONLY, false, 0.03f, 100u, 200u));
    assert(!ImuMath::canUseAbsoluteYawForNav(
        ImuHeadingState::IMU_ABSOLUTE_OK, true, 0.6f, 100u, 200u));
    assert(!ImuMath::canUseAbsoluteYawForNav(
        ImuHeadingState::IMU_ABSOLUTE_OK, true, 0.03f, 250u, 200u));
    assert(!ImuMath::canUseAbsoluteYawForNav(
        ImuHeadingState::IMU_MAG_DISTURBED, true, 0.03f, 100u, 200u));
    assert(!ImuMath::canUseAbsoluteYawForNav(
        ImuHeadingState::IMU_STALE, true, 0.03f, 100u, 200u));

    std::puts("ImuMath heading transform tests passed");
    return 0;
}
