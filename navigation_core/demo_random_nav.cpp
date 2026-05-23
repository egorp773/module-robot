#include "NavigationCore.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

float motorOutput(int16_t command) {
  if (std::abs(command) < 3) return 0.0f;
  return static_cast<float>(command) * 0.98f;
}

struct RobotPhysics {
  LocalCoords position;
  float headingDeg = 0.0f;
  float speedMps = 0.0f;

  void update(int16_t leftCmd, int16_t rightCmd) {
    const float leftVelocity = (motorOutput(leftCmd) / kMaxCommand) * kMaxSpeedMps;
    const float rightVelocity = (motorOutput(rightCmd) / kMaxCommand) * kMaxSpeedMps;
    const float velocity = (leftVelocity + rightVelocity) * 0.5f;
    const float omegaRadPerS = (leftVelocity - rightVelocity) / kWheelBaseM;
    const float headingRad = headingDeg * kPi / 180.0f;

    position.x += velocity * sinf(headingRad) * kDt;
    position.y += velocity * cosf(headingRad) * kDt;
    headingDeg = normalize360(headingDeg + omegaRadPerS * kDt * 180.0f / kPi);
    speedMps = fabsf(velocity);
  }
};

struct SensorMocks {
  explicit SensorMocks(int seed)
    : rng(seed),
      rtkNoise(-0.015f, 0.015f),
      imuNoise(-0.25f, 0.25f),
      jumpNoise(-0.20f, 0.20f) {}

  LocalCoords gps(const LocalCoords& truth) {
    LocalCoords measured(truth.x + rtkNoise(rng), truth.y + rtkNoise(rng));
    if (jumpCountdown == 0) {
      measured.x += jumpNoise(rng);
      measured.y += jumpNoise(rng);
      jumpCountdown = 97;
    } else {
      jumpCountdown--;
    }
    return measured;
  }

  float imu(float truthHeadingDeg) {
    imuDriftDeg += 0.1f * kDt;
    return normalize360(truthHeadingDeg + imuDriftDeg + imuNoise(rng));
  }

  std::mt19937 rng;
  std::uniform_real_distribution<float> rtkNoise;
  std::uniform_real_distribution<float> imuNoise;
  std::uniform_real_distribution<float> jumpNoise;
  float imuDriftDeg = 0.0f;
  int jumpCountdown = 47;
};

void printUsage(const char* exe) {
  std::printf("Usage:\n");
  std::printf("  %s --random [seed] [--csv path]\n", exe);
  std::printf("  %s --target <x_m> <y_m> [seed] [--csv path]\n", exe);
}

}  // namespace

int main(int argc, char** argv) {
  int seed = 42;
  float targetX = 0.0f;
  float targetY = 0.0f;
  const char* csvPath = nullptr;

  if (argc >= 2 && std::strcmp(argv[1], "--random") == 0) {
    if (argc >= 3) seed = std::atoi(argv[2]);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> targetDist(-5.0f, 5.0f);
    do {
      targetX = targetDist(rng);
      targetY = targetDist(rng);
    } while (hypotf(targetX, targetY) < 1.0f);
  } else if (argc >= 4 && std::strcmp(argv[1], "--target") == 0) {
    targetX = static_cast<float>(std::atof(argv[2]));
    targetY = static_cast<float>(std::atof(argv[3]));
    if (argc >= 5) seed = std::atoi(argv[4]);
  } else {
    printUsage(argv[0]);
    return 2;
  }

  for (int i = 2; i + 1 < argc; i++) {
    if (std::strcmp(argv[i], "--csv") == 0) {
      csvPath = argv[i + 1];
      break;
    }
  }

  NavigationCore nav;
  NavConfig config;
  config.enableArrivalAdvance = true;
  config.arrivalDistanceM = 0.12f;
  config.arrivalSpeedMps = 0.08f;
  nav.setConfig(config);
  nav.addWaypoint(targetX, targetY);
  nav.reset();

  RobotPhysics robot;
  SensorMocks sensors(seed);
  FILE* csv = nullptr;
  if (csvPath != nullptr) {
    csv = std::fopen(csvPath, "w");
    if (csv == nullptr) {
      std::printf("Failed to open CSV output: %s\n", csvPath);
      return 3;
    }
    std::fprintf(csv, "time_s,true_x,true_y,gps_x,gps_y,heading_deg,dist_m,heading_error_deg,cross_track_m,left_cmd,right_cmd,state,target_x,target_y\n");
  }

  std::printf("Random navigation demo\n");
  std::printf("Target: x=%.2f m, y=%.2f m, seed=%d\n", targetX, targetY, seed);
  std::printf("Sensors: GPS RTK noise +/-15mm + occasional jumps, IMU drift 0.1 deg/s\n\n");
  std::printf(" t[s] | true_x true_y | gps_x gps_y | heading | dist | err | xtk | cmdL cmdR\n");
  std::printf("------+---------------+-------------+---------+------+-----+-----+----------\n");

  NavOutput output;
  for (int step = 0; step < 2400; step++) {
    const LocalCoords gpsPosition = sensors.gps(robot.position);
    const float imuHeading = sensors.imu(robot.headingDeg);

    NavInput input;
    input.position = gpsPosition;
    input.headingDeg = imuHeading;
    input.speedMps = robot.speedMps;
    input.quality = NavQuality::Fixed;

    output = nav.update(input, kDt);
    robot.update(output.leftCmd, output.rightCmd);

    if (csv != nullptr) {
      std::fprintf(csv, "%.3f,%.6f,%.6f,%.6f,%.6f,%.3f,%.6f,%.3f,%.6f,%d,%d,%s,%.6f,%.6f\n",
                   step * kDt,
                   robot.position.x,
                   robot.position.y,
                   gpsPosition.x,
                   gpsPosition.y,
                   robot.headingDeg,
                   output.distanceToTargetM,
                   output.headingErrorDeg,
                   output.crossTrackErrorM,
                   output.leftCmd,
                   output.rightCmd,
                   nav.state() == NavState::Arrived ? "ARRIVED" : "MOVING",
                   targetX,
                   targetY);
    }

    if (step % 20 == 0 || nav.state() == NavState::Arrived) {
      std::printf("%5.1f | %6.2f %6.2f | %5.2f %5.2f | %7.1f | %4.2f | %+4.0f | %+4.2f | %4d %4d\n",
                  step * kDt,
                  robot.position.x, robot.position.y,
                  gpsPosition.x, gpsPosition.y,
                  robot.headingDeg,
                  output.distanceToTargetM,
                  output.headingErrorDeg,
                  output.crossTrackErrorM,
                  output.leftCmd,
                  output.rightCmd);
    }

    if (nav.state() == NavState::Arrived) break;
  }

  const float miss = hypotf(targetX - robot.position.x, targetY - robot.position.y);
  if (csv != nullptr) {
    std::fclose(csv);
  }
  std::printf("\nFinal: x=%.3f m, y=%.3f m, miss=%.3f m (%.1f cm), state=%s\n",
              robot.position.x,
              robot.position.y,
              miss,
              miss * 100.0f,
              nav.state() == NavState::Arrived ? "ARRIVED" : "NOT_ARRIVED");
  return nav.state() == NavState::Arrived ? 0 : 1;
}
