/**
 * NavigationCore - platform independent rover navigation.
 *
 * Heading convention matches rover.cpp:
 *   0 degrees = north, positive clockwise, x = east, y = north.
 */
#ifndef RTK_NAVIGATION_CORE_H
#define RTK_NAVIGATION_CORE_H

#include <math.h>
#include <stdint.h>

namespace navcore {

struct LocalCoords {
  LocalCoords() : x(0.0f), y(0.0f) {}
  LocalCoords(float xValue, float yValue) : x(xValue), y(yValue) {}

  float x;
  float y;
};

struct Waypoint {
  LocalCoords pos;
  bool received = false;
};

enum class NavState : uint8_t {
  Idle,
  Moving,
  Approaching,
  Arrived,
  Error,
};

enum class NavQuality : uint8_t {
  None,
  GpsHold,
  Degraded,
  FloatOk,
  Fixed,
};

struct NavConfig {
  float maxSpeedMps = 0.25f;
  float floatSpeedMps = 0.12f;
  float degradedSpeedMps = 0.07f;
  float holdSpeedMps = 0.03f;
  int16_t maxSpeedPercent = 70;

  float arrivalDistanceM = 0.1f;
  float arrivalApproachDistanceM = 0.3f;
  float arrivalSpeedMps = 0.03f;
  bool enableArrivalAdvance = true;

  float lookaheadMinM = 0.22f;
  float lookaheadMaxM = 0.60f;
  float lookaheadSpeedGain = 0.55f;
  float headingGain = 0.85f;
  float crossTrackGain = 30.0f;
  bool alignFirst = true;
  float alignFirstThresholdDeg = 7.0f;

  float forwardScale = 1.0f;
  float turnScale = 1.0f;
  bool invertForward = false;
  bool invertSteering = false;
};

struct NavInput {
  LocalCoords position;
  float headingDeg = 0.0f;
  float speedMps = 0.0f;
  NavQuality quality = NavQuality::Fixed;
  bool goingWrong = false;
};

struct NavOutput {
  int16_t leftCmd = 0;
  int16_t rightCmd = 0;
  float desiredHeadingDeg = 0.0f;
  float headingErrorDeg = 0.0f;
  float crossTrackErrorM = 0.0f;
  float distanceToTargetM = 0.0f;
  float commandSpeedMps = 0.0f;
  float forwardCmd = 0.0f;
  float turnCmd = 0.0f;
  float crossTrackCorrection = 0.0f;
  bool alignOnly = false;
  LocalCoords lookaheadTarget;
  NavState state = NavState::Idle;
  uint8_t waypointIndex = 0;
};

class NavigationCore {
public:
  static constexpr uint8_t kMaxWaypoints = 254;

  NavigationCore() {
    clearRoute();
  }

  void setConfig(const NavConfig& newConfig) {
    config_ = newConfig;
  }

  NavConfig& config() {
    return config_;
  }

  const NavConfig& config() const {
    return config_;
  }

  void reset() {
    state_ = NavState::Idle;
    currentWaypointIndex_ = 0;
    lastOutput_ = NavOutput{};
  }

  void clearRoute() {
    waypointCount_ = 0;
    currentWaypointIndex_ = 0;
    for (uint8_t i = 0; i < kMaxWaypoints; i++) {
      waypoints_[i] = Waypoint{};
    }
  }

  bool addWaypoint(float x, float y) {
    if (waypointCount_ >= kMaxWaypoints) return false;
    waypoints_[waypointCount_].pos = LocalCoords(x, y);
    waypoints_[waypointCount_].received = true;
    waypointCount_++;
    return true;
  }

  bool setWaypoint(uint8_t index, float x, float y, bool received = true) {
    if (index >= kMaxWaypoints) return false;
    waypoints_[index].pos = LocalCoords(x, y);
    waypoints_[index].received = received;
    if (index >= waypointCount_) waypointCount_ = index + 1;
    return true;
  }

  void setWaypointCount(uint8_t count) {
    waypointCount_ = count > kMaxWaypoints ? kMaxWaypoints : count;
    if (currentWaypointIndex_ > waypointCount_) currentWaypointIndex_ = waypointCount_;
  }

  void setCurrentWaypointIndex(uint8_t index) {
    currentWaypointIndex_ = index;
  }

  uint8_t currentWaypointIndex() const {
    return currentWaypointIndex_;
  }

  uint8_t waypointCount() const {
    return waypointCount_;
  }

  NavState state() const {
    return state_;
  }

  const NavOutput& lastOutput() const {
    return lastOutput_;
  }

  bool isRouteReady() const {
    if (waypointCount_ == 0) return false;
    for (uint8_t i = 0; i < waypointCount_; i++) {
      if (!waypoints_[i].received) return false;
    }
    return true;
  }

  float remainingRouteDistance(const LocalCoords& position) const {
    if (currentWaypointIndex_ >= waypointCount_) return 0.0f;
    float distance = distanceBetween(position, waypoints_[currentWaypointIndex_].pos);
    for (uint8_t i = currentWaypointIndex_ + 1; i < waypointCount_; i++) {
      distance += distanceBetween(waypoints_[i - 1].pos, waypoints_[i].pos);
    }
    return distance;
  }

  NavOutput update(const NavInput& input, float /*dtSeconds*/) {
    NavOutput output{};
    output.state = state_;
    output.waypointIndex = currentWaypointIndex_;

    if (!isRouteReady()) {
      state_ = NavState::Idle;
      output.state = state_;
      lastOutput_ = output;
      return output;
    }

    if (currentWaypointIndex_ >= waypointCount_) {
      state_ = NavState::Arrived;
      output.state = state_;
      lastOutput_ = output;
      return output;
    }

    const LocalCoords target = waypoints_[currentWaypointIndex_].pos;
    const float distanceToTarget = distanceBetween(input.position, target);
    output.distanceToTargetM = distanceToTarget;

    if (config_.enableArrivalAdvance &&
        distanceToTarget < config_.arrivalDistanceM &&
        input.speedMps < config_.arrivalSpeedMps) {
      currentWaypointIndex_++;
      state_ = currentWaypointIndex_ >= waypointCount_ ? NavState::Arrived : NavState::Moving;
      output.state = state_;
      output.waypointIndex = currentWaypointIndex_;
      lastOutput_ = output;
      return output;
    }

    state_ = distanceToTarget < config_.arrivalApproachDistanceM ? NavState::Approaching : NavState::Moving;
    output.state = state_;

    const LocalCoords lookaheadTarget = selectLookaheadTarget(
      input.position,
      input.speedMps,
      target,
      distanceToTarget
    );
    output.lookaheadTarget = lookaheadTarget;

    const float desiredHeading = headingTo(input.position, lookaheadTarget);
    const float headingError = normalizeAngle(desiredHeading - input.headingDeg);
    output.desiredHeadingDeg = desiredHeading;
    output.headingErrorDeg = headingError;

    const float crossTrackError = calculateCrossTrackError(input.position, target);
    output.crossTrackErrorM = crossTrackError;

    float commandSpeed = speedForQuality(input.quality);
    const float absHeadingError = fabsf(headingError);
    // FIXED: Allow slow forward motion during turns up to 150 degrees.
    // The original 120 degree threshold caused stuck detection when robot
    // tried to turn in place but had zero forward speed.
    if (absHeadingError > 150.0f && distanceToTarget > config_.arrivalApproachDistanceM) {
      commandSpeed *= 0.15f;  // Very slow forward during extreme turns
    } else if (absHeadingError > 120.0f) {
      commandSpeed *= 0.2f;   // Slow during big turns
    } else if (absHeadingError > 90.0f) {
      commandSpeed *= 0.3f;
    } else if (absHeadingError > 60.0f) {
      commandSpeed *= 0.5f;
    } else if (absHeadingError > 30.0f) {
      commandSpeed *= 0.7f;
    }

    if (state_ == NavState::Approaching) {
      commandSpeed *= 0.5f;
      if (distanceToTarget < 0.2f) commandSpeed *= 0.3f;
    }

    if (fabsf(crossTrackError) > 0.5f) commandSpeed *= 0.8f;
    if (fabsf(crossTrackError) > 1.0f) commandSpeed *= 0.6f;
    if (input.goingWrong) commandSpeed *= 0.3f;

    float forwardCmd = config_.maxSpeedMps > 0.0f
      ? (commandSpeed / config_.maxSpeedMps) * config_.maxSpeedPercent * config_.forwardScale
      : 0.0f;
    float turnCmd = config_.headingGain * headingError * config_.turnScale;
    const float crossTrackCorrection = config_.crossTrackGain * crossTrackError * config_.turnScale;
    const bool offLine = fabsf(crossTrackError) > 0.45f;
    if (config_.alignFirst && !offLine && absHeadingError > config_.alignFirstThresholdDeg) {
      commandSpeed = 0.0f;
      forwardCmd = 0.0f;
      output.alignOnly = true;
    } else {
      turnCmd += crossTrackCorrection;
    }

    output.commandSpeedMps = commandSpeed;

    const float maxTurn = config_.maxSpeedPercent * (output.alignOnly ? 0.7f : 0.6f);
    turnCmd = clamp(turnCmd, -maxTurn, maxTurn);

    if (config_.invertForward) forwardCmd = -forwardCmd;
    if (config_.invertSteering) turnCmd = -turnCmd;

    const float left = clamp(forwardCmd + turnCmd, -config_.maxSpeedPercent, config_.maxSpeedPercent);
    const float right = clamp(forwardCmd - turnCmd, -config_.maxSpeedPercent, config_.maxSpeedPercent);

    output.forwardCmd = forwardCmd;
    output.turnCmd = turnCmd;
    output.crossTrackCorrection = crossTrackCorrection;
    output.leftCmd = static_cast<int16_t>(left);
    output.rightCmd = static_cast<int16_t>(right);
    output.waypointIndex = currentWaypointIndex_;

    lastOutput_ = output;
    return output;
  }

  static float normalizeAngle(float angleDeg) {
    while (angleDeg > 180.0f) angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
  }

  static float normalizeAngle360(float angleDeg) {
    while (angleDeg >= 360.0f) angleDeg -= 360.0f;
    while (angleDeg < 0.0f) angleDeg += 360.0f;
    return angleDeg;
  }

private:
  static float clamp(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
  }

  static float distanceBetween(const LocalCoords& a, const LocalCoords& b) {
    return hypotf(b.x - a.x, b.y - a.y);
  }

  static float headingTo(const LocalCoords& from, const LocalCoords& to) {
    float heading = atan2f(to.x - from.x, to.y - from.y) * 180.0f / kPi;
    if (heading < 0.0f) heading += 360.0f;
    return heading;
  }

  float speedForQuality(NavQuality quality) const {
    switch (quality) {
      case NavQuality::Fixed: return config_.maxSpeedMps;
      case NavQuality::FloatOk: return config_.floatSpeedMps;
      case NavQuality::Degraded: return config_.degradedSpeedMps;
      case NavQuality::GpsHold: return config_.holdSpeedMps;
      case NavQuality::None:
      default: return 0.0f;
    }
  }

  LocalCoords selectLookaheadTarget(
    const LocalCoords& position,
    float speedMps,
    const LocalCoords& target,
    float distanceToTarget
  ) const {
    float lookahead = config_.lookaheadMinM + speedMps * config_.lookaheadSpeedGain;
    lookahead = clamp(lookahead, config_.lookaheadMinM, config_.lookaheadMaxM);

    LocalCoords lookaheadTarget = target;
    if (currentWaypointIndex_ > 0 && currentWaypointIndex_ < waypointCount_) {
      const LocalCoords a = waypoints_[currentWaypointIndex_ - 1].pos;
      const LocalCoords b = target;
      const float dx = b.x - a.x;
      const float dy = b.y - a.y;
      const float segLen = distanceBetween(a, b);
      if (segLen > 0.05f) {
        const float progress = clamp(
          ((position.x - a.x) * dx + (position.y - a.y) * dy) / segLen,
          0.0f,
          segLen
        );
        const float ahead = clamp(progress + lookahead, 0.0f, segLen);
        lookaheadTarget.x = a.x + dx * (ahead / segLen);
        lookaheadTarget.y = a.y + dy * (ahead / segLen);
        if (ahead < segLen - 0.01f || currentWaypointIndex_ + 1 >= waypointCount_) {
          return lookaheadTarget;
        }
      }
    }

    if (distanceToTarget >= lookahead || currentWaypointIndex_ + 1 >= waypointCount_) {
      return lookaheadTarget;
    }

    float remaining = lookahead - distanceToTarget;
    LocalCoords a = target;
    for (uint8_t segIdx = currentWaypointIndex_ + 1; segIdx < waypointCount_; segIdx++) {
      const LocalCoords b = waypoints_[segIdx].pos;
      const float segmentLength = distanceBetween(a, b);
      if (segmentLength <= 0.01f) continue;
      if (remaining <= segmentLength) {
        const float t = remaining / segmentLength;
        lookaheadTarget.x = a.x + (b.x - a.x) * t;
        lookaheadTarget.y = a.y + (b.y - a.y) * t;
        break;
      }
      remaining -= segmentLength;
      a = b;
    }

    return lookaheadTarget;
  }

  float calculateCrossTrackError(const LocalCoords& position, const LocalCoords& target) const {
    if (currentWaypointIndex_ == 0 || currentWaypointIndex_ >= waypointCount_) return 0.0f;

    const LocalCoords a = waypoints_[currentWaypointIndex_ - 1].pos;
    const LocalCoords b = target;
    const float segmentLength = distanceBetween(a, b);
    if (segmentLength <= 0.1f) return 0.0f;

    const float dxSegment = b.x - a.x;
    const float dySegment = b.y - a.y;
    return (dxSegment * (position.y - a.y) - dySegment * (position.x - a.x)) / segmentLength;
  }

  static constexpr float kPi = 3.14159265358979323846f;

  NavConfig config_;
  NavState state_ = NavState::Idle;
  Waypoint waypoints_[kMaxWaypoints];
  uint8_t waypointCount_ = 0;
  uint8_t currentWaypointIndex_ = 0;
  NavOutput lastOutput_;
};

}  // namespace navcore

#endif  // RTK_NAVIGATION_CORE_H
