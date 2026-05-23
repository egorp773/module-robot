#include <gtest/gtest.h>

#include "NavigationCore.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <random>

namespace {

using navcore::LocalCoords;
using navcore::NavConfig;
using navcore::NavInput;
using navcore::NavOutput;
using navcore::NavQuality;
using navcore::NavState;
using navcore::NavigationCore;

constexpr float kDt = 0.05f;
constexpr float kWheelBaseM = 0.38f;
constexpr float kMaxSpeedMps = 0.5f;
constexpr int kMaxCommand = 70;
constexpr float kPi = 3.14159265358979323846f;

float normalize360(float value) {
  while (value >= 360.0f) value -= 360.0f;
  while (value < 0.0f) value += 360.0f;
  return value;
}

class GpsMock {
public:
  explicit GpsMock(std::mt19937& rng) : rng_(rng), rtkNoise_(-0.015f, 0.015f), jumpNoise_(-0.20f, 0.20f) {}

  LocalCoords read(const LocalCoords& truth) {
    LocalCoords measured{
      truth.x + rtkNoise_(rng_),
      truth.y + rtkNoise_(rng_),
    };

    if (jumpCountdown_ == 0) {
      measured.x += jumpNoise_(rng_);
      measured.y += jumpNoise_(rng_);
      jumpCountdown_ = 97;
    } else {
      jumpCountdown_--;
    }

    return measured;
  }

private:
  std::mt19937& rng_;
  std::uniform_real_distribution<float> rtkNoise_;
  std::uniform_real_distribution<float> jumpNoise_;
  int jumpCountdown_ = 47;
};

class ImuMock {
public:
  explicit ImuMock(std::mt19937& rng) : rng_(rng), noise_(-0.25f, 0.25f) {}

  float read(float trueHeadingDeg, float dt) {
    driftDeg_ += driftRateDegPerS * dt;
    return normalize360(trueHeadingDeg + driftDeg_ + noise_(rng_));
  }

  float driftRateDegPerS = 0.1f;

private:
  std::mt19937& rng_;
  std::uniform_real_distribution<float> noise_;
  float driftDeg_ = 0.0f;
};

class MotorMock {
public:
  void sendCommand(int16_t left, int16_t right) {
    lastLeftCmd = left;
    lastRightCmd = right;
    lastMeasuredLeft = left;
    lastMeasuredRight = right;
  }

  int16_t lastLeftCmd = 0;
  int16_t lastRightCmd = 0;
  int16_t lastMeasuredLeft = 0;
  int16_t lastMeasuredRight = 0;
};

class RobotPhysics {
public:
  LocalCoords position;
  float headingDeg = 0.0f;
  float speedMps = 0.0f;

  void update(int16_t leftCmd, int16_t rightCmd, float dt) {
    const float left = applyMotorImperfections(leftCmd);
    const float right = applyMotorImperfections(rightCmd);
    const float leftVelocity = (left / kMaxCommand) * kMaxSpeedMps;
    const float rightVelocity = (right / kMaxCommand) * kMaxSpeedMps;
    const float velocity = (leftVelocity + rightVelocity) * 0.5f;
    const float omegaRadPerS = (leftVelocity - rightVelocity) / kWheelBaseM;

    const float headingRad = headingDeg * kPi / 180.0f;
    position.x += velocity * sinf(headingRad) * dt;
    position.y += velocity * cosf(headingRad) * dt;
    headingDeg = normalize360(headingDeg + omegaRadPerS * dt * 180.0f / kPi);
    speedMps = fabsf(velocity);
  }

private:
  static float applyMotorImperfections(int16_t command) {
    if (std::abs(command) < 3) return 0.0f;
    return static_cast<float>(command) * 0.98f;
  }
};

struct RunResult {
  LocalCoords finalPosition;
  float finalErrorM = 0.0f;
  float maxHeadingErrorDeg = 0.0f;
  float maxCrossTrackM = 0.0f;
  int steps = 0;
  bool arrived = false;
};

NavigationCore makeCore(float targetX, float targetY) {
  NavigationCore core;
  NavConfig config;
  config.enableArrivalAdvance = true;
  config.arrivalDistanceM = 0.12f;
  config.arrivalSpeedMps = 0.08f;
  core.setConfig(config);
  core.clearRoute();
  core.addWaypoint(targetX, targetY);
  core.reset();
  return core;
}

RunResult runStraightLine(int seed, float targetX = 5.0f, float targetY = 0.0f) {
  std::mt19937 rng(seed);
  GpsMock gps(rng);
  ImuMock imu(rng);
  MotorMock motor;
  RobotPhysics robot;
  NavigationCore core = makeCore(targetX, targetY);

  RunResult result;

  for (int step = 0; step < 2400; step++) {
    const LocalCoords gpsPosition = gps.read(robot.position);
    const float imuHeading = imu.read(robot.headingDeg, kDt);

    NavInput input;
    input.position = gpsPosition;
    input.headingDeg = imuHeading;
    input.speedMps = robot.speedMps;
    input.quality = NavQuality::Fixed;

    const NavOutput output = core.update(input, kDt);
    motor.sendCommand(output.leftCmd, output.rightCmd);
    robot.update(motor.lastMeasuredLeft, motor.lastMeasuredRight, kDt);

    result.maxHeadingErrorDeg = std::max(result.maxHeadingErrorDeg, fabsf(output.headingErrorDeg));
    result.maxCrossTrackM = std::max(result.maxCrossTrackM, fabsf(output.crossTrackErrorM));
    result.steps = step + 1;

    if (core.state() == NavState::Arrived) {
      result.arrived = true;
      break;
    }
  }

  result.finalPosition = robot.position;
  result.finalErrorM = hypotf(targetX - robot.position.x, targetY - robot.position.y);
  return result;
}

}  // namespace

TEST(NavigationCore, DrivesFromOriginToFiveMetersEastFiftyRuns) {
  float totalError = 0.0f;
  float worstError = 0.0f;
  int arrivedCount = 0;

  for (int run = 0; run < 50; run++) {
    const RunResult result = runStraightLine(1000 + run * 17);
    totalError += result.finalErrorM;
    worstError = std::max(worstError, result.finalErrorM);
    if (result.arrived) arrivedCount++;
  }

  const float averageError = totalError / 50.0f;
  std::printf("NavigationCore 50-run average miss: %.3f m (%.1f cm), worst: %.3f m, arrived: %d/50\n",
              averageError, averageError * 100.0f, worstError, arrivedCount);

  EXPECT_EQ(arrivedCount, 50);
  EXPECT_LT(averageError, 0.18f);
  EXPECT_LT(worstError, 0.28f);
}

TEST(NavigationCore, CrossTrackCorrectionSignForEastboundSegment) {
  NavigationCore core;
  NavConfig config;
  config.enableArrivalAdvance = false;
  core.setConfig(config);
  core.clearRoute();
  core.addWaypoint(0.0f, 0.0f);
  core.addWaypoint(5.0f, 0.0f);
  core.setCurrentWaypointIndex(1);

  NavInput northOfLine;
  northOfLine.position = {2.0f, 0.5f};
  northOfLine.headingDeg = 90.0f;
  northOfLine.speedMps = 0.25f;
  northOfLine.quality = NavQuality::Fixed;

  const NavOutput northOutput = core.update(northOfLine, kDt);
  EXPECT_GT(northOutput.crossTrackErrorM, 0.0f);
  EXPECT_GT(northOutput.crossTrackCorrection, 0.0f);
  EXPECT_GT(northOutput.leftCmd, northOutput.rightCmd)
    << "Positive cross-track on an eastbound segment should command a clockwise/right correction.";

  NavInput southOfLine = northOfLine;
  southOfLine.position = {2.0f, -0.5f};
  const NavOutput southOutput = core.update(southOfLine, kDt);
  EXPECT_LT(southOutput.crossTrackErrorM, 0.0f);
  EXPECT_LT(southOutput.crossTrackCorrection, 0.0f);
  EXPECT_LT(southOutput.leftCmd, southOutput.rightCmd)
    << "Negative cross-track on an eastbound segment should command a counter-clockwise/left correction.";

  std::printf("Cross-track sign: north err=%.2f corr=%.2f cmds=(%d,%d), south err=%.2f corr=%.2f cmds=(%d,%d)\n",
              northOutput.crossTrackErrorM, northOutput.crossTrackCorrection,
              northOutput.leftCmd, northOutput.rightCmd,
              southOutput.crossTrackErrorM, southOutput.crossTrackCorrection,
              southOutput.leftCmd, southOutput.rightCmd);
}

TEST(NavigationCore, HoldsWithImuDriftAtPointOneDegreePerSecond) {
  const RunResult result = runStraightLine(4242);
  std::printf("IMU drift run miss: %.3f m (%.1f cm), max heading error: %.1f deg\n",
              result.finalErrorM, result.finalErrorM * 100.0f, result.maxHeadingErrorDeg);

  EXPECT_TRUE(result.arrived);
  EXPECT_LT(result.finalErrorM, 0.25f);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
