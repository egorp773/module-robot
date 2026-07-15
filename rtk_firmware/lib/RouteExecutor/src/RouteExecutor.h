#pragma once

#include "RouteGeometry.h"
#include "RouteTypes.h"

#include <cstddef>
#include <cstdint>

namespace routeexec {

enum class ExecutorState : uint8_t {
    IDLE = 0,
    VALIDATE_ROUTE,
    ACQUIRE_SEGMENT,
    FOLLOW_SEGMENT,
    APPROACH_TRANSITION,
    BRAKE,
    WAIT_PHYSICAL_STOP,
    TURN_TO_NEXT,
    HEADING_STABLE,
    INTERCEPT_NEXT_LINE,
    CORNER_MISSED_INTERCEPT,
    TERMINAL_APPROACH,
    RECOVERY_PLAN,
    RECOVERY_REVERSE,
    RECOVERY_TURN,
    RECOVERY_APPROACH,
    RECOVERY_EVALUATE,
    FINAL_STOP,
    COMPLETE,
    FAULT,
};

enum class ExecutorResult : uint8_t {
    NONE = 0,
    ARRIVED,
    STOPPED,
    TIMEOUT,
    FAULT,
};

enum class MotionKind : uint8_t {
    STOP = 0,
    DRIVE,
    TURN_IN_PLACE,
    REVERSE,
};

enum class RouteTransitionPurpose : uint8_t {
    NONE = 0,
    PLANNED_CORNER,
    CORNER_MISSED_INTERCEPT,
    STEERING_REACQUIRE,
    RECOVERY,
    FINAL_ARRIVAL,
    STEERING_RESPONSE_FAULT,
};

struct RouteExecutorConfig {
    float stanleyHeadingGain;
    float stanleyCrossTrackGain;
    float stanleySoftSpeedMps;
    float yawRateDamping;
    float maxAngularRadps;
    float degradedSpeedMps;
    float lookaheadM;
    float rampUpM;
    float rampMinFactor;

    float turnEnterDeg;
    float turnExitDeg;
    float turnToleranceDeg;
    float turnRateMaxRadps;
    float turnRateMidRadps;
    float turnRateLowRadps;
    float turnRateMinRadps;
    float turnSettleYawRateDps;
    uint32_t turnTimeoutMs;

    int physicalStopMotorThreshold;
    float physicalStopYawRateDps;
    float physicalStopPvtDisplacementM;
    float physicalStopHeadingDriftDeg;
    uint32_t feedbackFreshMs;
    uint32_t imuFreshMs;
    uint32_t pvtFreshMs;
    uint32_t physicalStopStableMs;
    uint32_t physicalStopTimeoutMs;

    float transitionPlaneMarginM;
    float interceptEnterCrossM;
    float interceptEnterHeadingDeg;
    float interceptLookaheadM;
    float interceptMaxCrossM;
    float interceptSpeedMps;
    uint32_t interceptTimeoutMs;
    float progressMinM;
    uint32_t progressTimeoutMs;

    // A DRIVE steering command and the estimator use one sign contract:
    // positive angularRadps must produce positive (clockwise) heading/yawRate.
    // The watchdog only evaluates settled, moving, fresh-data samples and
    // requires several consecutive opposite responses before stopping.
    float steeringResponseMinAngularRadps;
    float steeringResponseMinYawRateDps;
    float steeringResponseMinHeadingDeltaDeg;
    float steeringResponseMinSpeedMps;
    int steeringResponseMinCommandPercent;
    uint32_t steeringResponseSettleMs;
    uint32_t steeringResponseObservationMinMs;
    uint8_t steeringResponseWrongDirectionCount;

    float terminalApproachDistanceM;
    float terminalApproachSpeedMps;
    float finalPosePreApproachM;

    float recoveryReverseSpeedMps;
    float recoveryReverseMaxM;
    float recoveryApproachSpeedMps;
    float recoveryProgressMinM;
    float recoveryGoalToleranceM;
    float recoveryApproachMaxM;
    uint8_t recoveryMaxAttempts;
    uint32_t recoveryApproachTimeoutMs;

    RouteExecutorConfig();
};

struct RouteExecutorInput {
    uint32_t nowMs;
    uint32_t pvtId;
    // Local route coordinates are x=East, y=North.
    LocalPoint position;
    // Navigation sign contract: 0 deg = North, 90 deg = East, increasing
    // heading and positive yawRate are clockwise when viewed from above.
    float headingDeg;
    float yawRateDps;
    float estimatedSpeedMps;
    // Applied legacy hoverboard virtual channels and raw, side-labelled
    // feedback.  These are deliberately not advertised as calibrated
    // physical track velocities; Motor owns the mode-aware DRIVE/TURN map
    // and no unproved feedback sign inversion is applied here.
    int commandLeft;
    int commandRight;
    int measuredLeft;
    int measuredRight;
    uint32_t feedbackAgeMs;
    uint32_t imuAgeMs;
    uint32_t pvtAgeMs;
    bool motionAllowed;
    bool degraded;
    bool currentFootprintAllowed;
    bool forwardPathAllowed;
    bool nextSegmentPathAllowed;
    bool recoveryPathAllowed;
    bool reversePathAllowed;
    bool turnPathAllowed;
    bool reverseEnabled;

    RouteExecutorInput();
};

struct RouteExecutorOutput {
    MotionKind motion;
    float linearMps;
    // Positive angularRadps requests the same clockwise-positive rotation as
    // positive heading/yawRate, for DRIVE and TURN_IN_PLACE alike.
    float angularRadps;
    ExecutorState state;
    ExecutorState oldState;
    bool stateChanged;
    ExecutorResult result;
    const char* faultReason;
    uint32_t routeId;
    size_t segmentIndex;
    size_t segmentCount;
    WaypointType waypointType;
    LocalPoint plannedStart;
    LocalPoint plannedEnd;
    float plannedHeadingDeg;
    float alongTrackM;
    float crossTrackM;
    float distanceToWaypointM;
    LocalPoint interceptTarget;
    float steeringTargetDeg;
    float steeringErrorDeg;
    float computedRecoveryBearingDeg;
    float latchedTurnTargetDeg;
    LocalPoint recoveryGoal;
    uint8_t recoveryAttempt;
    uint32_t physicalStableMs;
    bool physicalStopReady;
    bool finalArrivalPending;
    bool workActionPending;
    size_t workActionPointIndex;
    RouteTransitionPurpose transitionPurpose;
    float steeringResponseRequestedAngularRadps;
    float steeringResponseHeadingDeltaDeg;
    uint8_t steeringResponseObservationCount;
    bool steeringResponseWrongDirection;
    bool invariantViolation;

    RouteExecutorOutput();
};

const char* executorStateName(ExecutorState state);
const char* executorResultName(ExecutorResult result);
const char* waypointTypeName(WaypointType type);

class RouteExecutor {
public:
    RouteExecutor();

    bool start(const RoutePlan& plan,
               const RouteExecutorConfig& config,
               const RouteExecutorInput& input);
    RouteExecutorOutput update(const RouteExecutorInput& input);
    void stop(const char* reason = "stopped");
    void expire(const char* reason = "timeout");
    void pause(uint32_t nowMs = 0u);
    void resume(uint32_t nowMs = 0u);
    void requestRecovery(const char* reason = "recovery_requested");

    bool active() const;
    bool paused() const { return paused_; }
    bool arrived() const { return result_ == ExecutorResult::ARRIVED; }
    bool routeFinished() const;
    ExecutorState state() const { return state_; }
    ExecutorResult result() const { return result_; }
    const char* faultReason() const { return faultReason_; }
    size_t segmentIndex() const { return segmentIndex_; }
    const RoutePlan& plan() const { return plan_; }
    // Reuse the executor's fixed-capacity storage while it is inactive; a
    // second 254-point staging plan costs about 21 KiB of ESP32 DRAM.
    RoutePlan& planBufferForBuild() { return plan_; }
    const RouteExecutorOutput& lastOutput() const { return output_; }

private:
    enum class StopNext : uint8_t {
        NONE = 0,
        START_TURN,
        TURN_NEXT,
        TURN_SETTLE,
        START_INTERCEPT,
        FINAL_COMPLETE,
        RECOVERY_REVERSE,
        RECOVERY_TURN,
        RECOVERY_EVALUATE,
        CORNER_MISSED_INTERCEPT,
        CORNER_MISSED_FAULT,
        STEERING_RESPONSE_FAULT,
    };
    enum class StableNext : uint8_t {
        NONE = 0,
        NEXT_SEGMENT_INTERCEPT,
        RECOVERY_APPROACH,
        RESUME_INTERCEPT,
    };

    void setState(ExecutorState state, uint32_t nowMs);
    void enterBrake(StopNext next,
                    const RouteExecutorInput& input,
                    ExecutorState visibleState = ExecutorState::BRAKE);
    bool physicalStopInstant(const RouteExecutorInput& input) const;
    bool updatePhysicalStop(const RouteExecutorInput& input);
    void startTurn(float targetDeg,
                   ExecutorState turnState,
                   StableNext stableNext,
                   const RouteExecutorInput& input,
                   bool resetCorrections = true);
    void updateTurn(const RouteExecutorInput& input,
                    ExecutorState turnState);
    void updateHeadingStable(const RouteExecutorInput& input);
    void beginRecovery(const RouteExecutorInput& input,
                       const char* reason);
    void updateRecoveryPlan(const RouteExecutorInput& input);
    void updateRecoveryApproach(const RouteExecutorInput& input);
    void updateRecoveryEvaluate(const RouteExecutorInput& input);
    void scheduleTurn(float targetDeg,
                      ExecutorState turnState,
                      StableNext stableNext,
                      const RouteExecutorInput& input);
    void resetProgress(uint32_t nowMs);
    bool updateProgress(const RouteExecutorInput& input);
    void resetSteeringResponseWatchdog();
    bool updateSteeringResponseWatchdog(
        const RouteExecutorInput& input,
        float requestedAngularRadps,
        float headingErrorDeg);
    void completeArrived(uint32_t nowMs);
    void fail(const char* reason, uint32_t nowMs,
              ExecutorResult result = ExecutorResult::FAULT);
    void acquireSegment(uint32_t nowMs, bool intercept);
    void updateLineMotion(const RouteExecutorInput& input,
                          bool interceptMode,
                          bool terminalMode,
                          bool recoveryMode);
    void refreshGeometry(const RouteExecutorInput& input);
    void populateOutput(const RouteExecutorInput& input);
    bool finalRadiusSatisfied() const;
    bool isTerminalSegment() const;

    RoutePlan plan_;
    RouteExecutorConfig config_;
    ExecutorState state_;
    ExecutorState previousState_;
    ExecutorResult result_;
    const char* faultReason_;
    const char* recoveryReason_;
    bool recoveryInitializationPending_;
    bool paused_;
    uint32_t pausedAtMs_;
    size_t segmentIndex_;
    uint32_t stateSinceMs_;

    SegmentProjection projection_;
    LocalPoint interceptTarget_;
    float steeringTargetDeg_;
    float steeringTargetPreviousDeg_;
    uint32_t steeringTargetAtMs_;

    StopNext stopNext_;
    StableNext stableNext_;
    uint32_t stopStartedMs_;
    uint32_t physicalStableSinceMs_;
    LocalPoint physicalStopAnchor_;
    float physicalStopHeadingDeg_;
    uint32_t physicalStopPvtId_;

    float turnTargetDeg_;
    bool turnTargetLatched_;
    int turnDirection_;
    int pendingTurnDirection_;
    uint8_t turnCorrections_;
    float turnStableReferenceDeg_;
    uint32_t turnStartedMs_;
    float scheduledTurnTargetDeg_;
    ExecutorState scheduledTurnState_;
    StableNext scheduledStableNext_;

    uint32_t progressSinceMs_;
    uint32_t progressPvtId_;
    float progressBestAlongM_;
    float progressBestDistanceM_;
    float progressBestCrossM_;

    uint8_t recoveryAttempt_;
    size_t recoverySegmentIndex_;
    bool recoveryActive_;
    LocalPoint recoveryGoal_;
    float computedRecoveryBearingDeg_;
    float recoveryBaselineDistanceM_;
    float recoveryBaselineCrossM_;
    LocalPoint recoveryMotionStart_;
    uint32_t recoveryMotionStartedMs_;
    bool recoveryGoalReached_;

    int steeringResponseAngularSign_;
    uint32_t steeringResponseCommandSinceMs_;
    uint32_t steeringResponseLastObservationMs_;
    uint32_t steeringResponseLastPvtId_;
    float steeringResponseLastHeadingDeg_;
    float steeringResponseRequestedAngularRadps_;
    float steeringResponseHeadingErrorDeg_;
    float steeringResponseHeadingDeltaDeg_;
    uint8_t steeringResponseObservationCount_;
    bool steeringResponseWrongDirection_;
    RouteTransitionPurpose transitionPurpose_;

    RouteExecutorOutput output_;
};

}  // namespace routeexec
