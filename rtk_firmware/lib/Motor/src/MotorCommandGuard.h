#pragma once

#include <cstdint>

namespace motorcmd {

struct GuardedMotorCommand {
    uint32_t motorCommandSequence = 0u;
    uint32_t authorizationEpoch = 0u;
    int routeRequestedLeft = 0;
    int routeRequestedRight = 0;
    int motorAppliedLeft = 0;
    int motorAppliedRight = 0;
    int uartSpeed = 0;
    int uartSteer = 0;
    bool zeroLatchActive = true;
};

// Pure command-path interlock shared by firmware and native tests.  A hard
// zero advances the authorization epoch, invalidating commands calculated by
// another task before STOP/BRAKE/ESTOP.  Only an explicit authorize() call can
// open a new epoch; ordinary command writes never clear the latch.
class MotorCommandGuard {
public:
    uint32_t authorize() {
        ++state_.authorizationEpoch;
        if (state_.authorizationEpoch == 0u) ++state_.authorizationEpoch;
        state_.zeroLatchActive = false;
        ++state_.motorCommandSequence;
        return state_.authorizationEpoch;
    }

    bool apply(uint32_t authorization,
               int requestedLeft, int requestedRight,
               int appliedLeft, int appliedRight) {
        if (state_.zeroLatchActive || authorization == 0u ||
            authorization != state_.authorizationEpoch) {
            return false;
        }
        state_.routeRequestedLeft = requestedLeft;
        state_.routeRequestedRight = requestedRight;
        state_.motorAppliedLeft = appliedLeft;
        state_.motorAppliedRight = appliedRight;
        ++state_.motorCommandSequence;
        return true;
    }

    void hardZero() {
        if (state_.zeroLatchActive && state_.routeRequestedLeft == 0 &&
            state_.routeRequestedRight == 0 &&
            state_.motorAppliedLeft == 0 && state_.motorAppliedRight == 0 &&
            state_.uartSpeed == 0 && state_.uartSteer == 0) {
            return;
        }
        ++state_.authorizationEpoch;
        if (state_.authorizationEpoch == 0u) ++state_.authorizationEpoch;
        state_.zeroLatchActive = true;
        state_.routeRequestedLeft = 0;
        state_.routeRequestedRight = 0;
        state_.motorAppliedLeft = 0;
        state_.motorAppliedRight = 0;
        state_.uartSpeed = 0;
        state_.uartSteer = 0;
        ++state_.motorCommandSequence;
    }

    void recordUartFrame(int speed, int steer) {
        if (state_.zeroLatchActive) {
            state_.uartSpeed = 0;
            state_.uartSteer = 0;
        } else {
            state_.uartSpeed = speed;
            state_.uartSteer = steer;
        }
    }

    int uartSpeedForFrame(int proposedSpeed) const {
        return state_.zeroLatchActive ? 0 : proposedSpeed;
    }

    int uartSteerForFrame(int proposedSteer) const {
        return state_.zeroLatchActive ? 0 : proposedSteer;
    }

    const GuardedMotorCommand& state() const { return state_; }

private:
    GuardedMotorCommand state_{};
};

}  // namespace motorcmd
