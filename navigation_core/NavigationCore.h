#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace navcore {

struct LocalCoords {
  float x = 0.0f;
  float y = 0.0f;
};

enum class NavQuality : uint8_t { None, Float, Fixed };
enum class NavState : uint8_t { Idle, Running, Arrived, Error };

struct NavConfig {
  bool enableArrivalAdvance = true;
  float arrivalDistanceM = 0.12f;
  float arrivalSpeedMps = 0.08f;
  float maxSpeedMps = 0.25f;
  float wheelBaseM = 0.38f;
  int maxCommand = 70;
};

struct NavInput {
  LocalCoords position;
  float headingDeg = 0.0f;
  float speedMps = 0.0f;
  NavQuality quality = NavQuality::None;
};

struct NavOutput {
  int leftCmd = 0;
  int rightCmd = 0;
  float headingErrorDeg = 0.0f;
  float crossTrackErrorM = 0.0f;
  float crossTrackCorrection = 0.0f;
  float distanceToTargetM = 0.0f;
};

class NavigationCore {
public:
  void setConfig(const NavConfig& config) { config_ = config; }

  void clearRoute() {
    waypoints_.clear();
    currentWaypoint_ = 0;
    state_ = NavState::Idle;
  }

  bool addWaypoint(float x, float y) {
    if (waypoints_.size() >= kMaxWaypoints) return false;
    waypoints_.push_back({x, y});
    return true;
  }

  void reset() {
    currentWaypoint_ = 0;
    state_ = waypoints_.empty() ? NavState::Idle : NavState::Running;
  }

  void setCurrentWaypointIndex(int index) {
    if (index < 0 || index >= static_cast<int>(waypoints_.size())) return;
    currentWaypoint_ = static_cast<size_t>(index);
    state_ = NavState::Running;
  }

  NavState state() const { return state_; }

  NavOutput update(const NavInput& input, float dt) {
    (void)dt;
    NavOutput out;
    if (waypoints_.empty() || input.quality == NavQuality::None) {
      state_ = waypoints_.empty() ? NavState::Idle : NavState::Error;
      return out;
    }

    if (currentWaypoint_ >= waypoints_.size()) {
      state_ = NavState::Arrived;
      return out;
    }

    const LocalCoords target = waypoints_[currentWaypoint_];
    const float dx = target.x - input.position.x;
    const float dy = target.y - input.position.y;
    const float dist = std::sqrt(dx * dx + dy * dy);
    out.distanceToTargetM = dist;

    if (dist <= config_.arrivalDistanceM &&
        (!config_.enableArrivalAdvance || input.speedMps <= config_.arrivalSpeedMps ||
         currentWaypoint_ + 1 >= waypoints_.size())) {
      currentWaypoint_++;
      if (currentWaypoint_ >= waypoints_.size()) {
        state_ = NavState::Arrived;
        return out;
      }
    }

    state_ = NavState::Running;
    const LocalCoords activeTarget = waypoints_[currentWaypoint_];
    const LocalCoords previous = currentWaypoint_ == 0 ? input.position : waypoints_[currentWaypoint_ - 1];
    const float lineDx = activeTarget.x - previous.x;
    const float lineDy = activeTarget.y - previous.y;
    const float lineLen = std::sqrt(lineDx * lineDx + lineDy * lineDy);

    float desiredHeadingDeg = std::atan2(activeTarget.x - input.position.x,
                                         activeTarget.y - input.position.y) *
                              180.0f / kPi;
    if (lineLen > 0.05f) {
      desiredHeadingDeg = std::atan2(lineDx, lineDy) * 180.0f / kPi;
      const float rx = input.position.x - previous.x;
      const float ry = input.position.y - previous.y;
      out.crossTrackErrorM = (lineDx * ry - lineDy * rx) / lineLen;
    }
    desiredHeadingDeg = normalize360(desiredHeadingDeg);

    out.headingErrorDeg = wrap180(desiredHeadingDeg - normalize360(input.headingDeg));
    out.crossTrackCorrection =
        std::atan2(kStanleyK * out.crossTrackErrorM, kStanleySoftSpeed + std::fabs(input.speedMps));

    const float headingTerm = kHeadingK * out.headingErrorDeg * kPi / 180.0f;
    float turn = headingTerm + out.crossTrackCorrection;
    turn = clamp(turn, -kMaxTurn, kMaxTurn);

    float forward = config_.maxSpeedMps;
    if (input.quality == NavQuality::Float) forward *= 0.5f;
    forward *= clamp(1.0f - std::fabs(out.headingErrorDeg) / 120.0f, 0.0f, 1.0f);
    if (dist < 0.5f) forward *= 0.45f;

    if (std::fabs(out.headingErrorDeg) > 55.0f) {
      forward = 0.0f;
      turn = clamp(headingTerm, -kMaxTurn, kMaxTurn);
    }

    const float leftMps = forward + turn * config_.wheelBaseM * 0.5f;
    const float rightMps = forward - turn * config_.wheelBaseM * 0.5f;
    out.leftCmd = speedToCommand(leftMps);
    out.rightCmd = speedToCommand(rightMps);
    return out;
  }

private:
  static constexpr size_t kMaxWaypoints = 254;
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kHeadingK = 1.5f;
  static constexpr float kStanleyK = 0.35f;
  static constexpr float kStanleySoftSpeed = 0.08f;
  static constexpr float kMaxTurn = 1.2f;

  static float clamp(float value, float lo, float hi) {
    return std::max(lo, std::min(hi, value));
  }

  static float normalize360(float value) {
    while (value >= 360.0f) value -= 360.0f;
    while (value < 0.0f) value += 360.0f;
    return value;
  }

  static float wrap180(float value) {
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
  }

  int speedToCommand(float speedMps) const {
    const float normalized = speedMps / std::max(0.001f, config_.maxSpeedMps);
    const float cmd = clamp(normalized, -1.0f, 1.0f) * config_.maxCommand;
    return static_cast<int>(std::lround(cmd));
  }

  NavConfig config_;
  std::vector<LocalCoords> waypoints_;
  size_t currentWaypoint_ = 0;
  NavState state_ = NavState::Idle;
};

}  // namespace navcore
