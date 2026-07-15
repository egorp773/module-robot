#include "RouteExecutor.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace routeexec {

namespace {

const float kDegToRad = 0.01745329251994329577f;
const float kRadToDeg = 57.295779513082320876f;

bool finiteInput(const RouteExecutorInput& input) {
    return finitePoint(input.position) &&
           std::isfinite(input.headingDeg) &&
           std::isfinite(input.yawRateDps) &&
           std::isfinite(input.estimatedSpeedMps);
}

bool stateNeedsForwardPath(ExecutorState state) {
    return state == ExecutorState::FOLLOW_SEGMENT ||
           state == ExecutorState::APPROACH_TRANSITION ||
           state == ExecutorState::INTERCEPT_NEXT_LINE ||
           state == ExecutorState::TERMINAL_APPROACH ||
           state == ExecutorState::RECOVERY_APPROACH;
}

bool stateTracksRouteProgress(ExecutorState state) {
    return state == ExecutorState::FOLLOW_SEGMENT ||
           state == ExecutorState::APPROACH_TRANSITION ||
           state == ExecutorState::INTERCEPT_NEXT_LINE ||
           state == ExecutorState::TERMINAL_APPROACH;
}

bool hasValidFinalPosePreApproach(const RoutePlan& plan) {
    if (!plan.valid() || plan.pointCount() < 2u) return false;
    const RoutePoint& finalPoint = plan.point(plan.pointCount() - 1u);
    if (finalPoint.type != WaypointType::FINAL_POSE) return true;
    // FINAL_POSE is accepted only when the planner supplied an explicit
    // pre-approach corner and a final short segment already aligned with the
    // requested pose. This prevents an improvised large turn at the endpoint.
    if (plan.segmentCount() < 2u) return false;
    const RoutePoint& preApproach = plan.point(plan.pointCount() - 2u);
    const RouteSegment& finalSegment = plan.segment(plan.segmentCount() - 1u);
    return preApproach.type == WaypointType::CORNER &&
           finalSegment.type == SegmentType::TERMINAL_APPROACH &&
           std::fabs(wrapHeadingErrorDeg(
               finalSegment.plannedHeadingDeg - finalPoint.finalHeadingDeg)) <=
               finalPoint.headingToleranceDeg;
}

float smoothTarget(float current, float requested,
                   float maxRateDegPerS, float dtS) {
    const float error = wrapHeadingErrorDeg(requested - current);
    const float step = clampFloat(error,
                                  -maxRateDegPerS * dtS,
                                  maxRateDegPerS * dtS);
    return normalizeHeadingDeg(current + step);
}

}  // namespace

RouteExecutorConfig::RouteExecutorConfig()
    : stanleyHeadingGain(1.0f),
      stanleyCrossTrackGain(0.60f),
      stanleySoftSpeedMps(0.30f),
      yawRateDamping(0.10f),
      maxAngularRadps(0.35f),
      degradedSpeedMps(0.07f),
      lookaheadM(0.70f),
      rampUpM(0.15f),
      rampMinFactor(0.45f),
      turnEnterDeg(20.0f),
      turnExitDeg(5.0f),
      turnToleranceDeg(7.0f),
      turnRateMaxRadps(0.6333333f),
      turnRateMidRadps(0.5277778f),
      turnRateLowRadps(0.4222222f),
      turnRateMinRadps(0.3166667f),
      turnSettleYawRateDps(3.0f),
      turnTimeoutMs(18000u),
      turnEstimatedAngularDecelDegps2(45.0f),
      turnBrakeMarginDeg(2.0f),
      turnPredictedStopMinDeg(2.0f),
      turnPredictedStopMaxDeg(25.0f),
      turnSlowdownEnterDeg(35.0f),
      turnAngularPercentPerRadps(9.12f),
      turnFarCommandPercent(7),
      turnSlowCommandPercent(6),
      turnBreakawayPercent(6),
      turnBreakawayBoostPercent(7),
      turnBreakawayBoostAfterMs(350u),
      turnBreakawayMaxMs(900u),
      turnBreakawayYawRateThresholdDps(4.0f),
      turnBreakawayResponseStableMs(120u),
      turnMaxCorrectionAttempts(4u),
      turnCorrectionMinImprovementDeg(1.0f),
      physicalStopMotorThreshold(3),
      physicalStopYawRateDps(3.0f),
      physicalStopPvtDisplacementM(0.040f),
      physicalStopHeadingDriftDeg(2.0f),
      feedbackFreshMs(200u),
      imuFreshMs(200u),
      pvtFreshMs(500u),
      physicalStopStableMs(650u),
      physicalStopTimeoutMs(6000u),
      transitionPlaneMarginM(0.02f),
      interceptEnterCrossM(0.08f),
      interceptEnterHeadingDeg(12.0f),
      interceptLookaheadM(0.45f),
      interceptMaxCrossM(0.60f),
      interceptSpeedMps(0.07f),
      interceptTimeoutMs(10000u),
      progressMinM(0.04f),
      progressTimeoutMs(15000u),
      steeringResponseMinAngularRadps(0.12f),
      steeringResponseMinYawRateDps(1.0f),
      steeringResponseMinHeadingDeltaDeg(0.20f),
      steeringResponseMinSpeedMps(0.02f),
      steeringResponseMinCommandPercent(5),
      steeringResponseSettleMs(350u),
      steeringResponseObservationMinMs(120u),
      steeringResponseWrongDirectionCount(3u),
      terminalApproachDistanceM(0.45f),
      terminalApproachSpeedMps(0.055f),
      finalPosePreApproachM(0.40f),
      recoveryReverseSpeedMps(0.04f),
      recoveryReverseMaxM(0.35f),
      recoveryApproachSpeedMps(0.04f),
      recoveryProgressMinM(0.025f),
      recoveryGoalToleranceM(0.10f),
      recoveryApproachMaxM(0.50f),
      recoveryMaxAttempts(3u),
      recoveryApproachTimeoutMs(8000u) {}

RouteExecutorInput::RouteExecutorInput()
    : nowMs(0u), pvtId(0u), position(), headingDeg(0.0f), yawRateDps(0.0f),
      estimatedSpeedMps(0.0f), commandLeft(0), commandRight(0),
      measuredLeft(0), measuredRight(0), feedbackAgeMs(UINT32_MAX),
      imuAgeMs(UINT32_MAX), pvtAgeMs(UINT32_MAX), motionAllowed(false),
      degraded(false), currentFootprintAllowed(false),
      forwardPathAllowed(false), nextSegmentPathAllowed(false),
      recoveryPathAllowed(false), reversePathAllowed(false),
      turnPathAllowed(false), reverseEnabled(false) {}

RouteExecutorOutput::RouteExecutorOutput()
    : motion(MotionKind::STOP), linearMps(0.0f), angularRadps(0.0f),
      state(ExecutorState::IDLE), oldState(ExecutorState::IDLE),
      stateChanged(false), result(ExecutorResult::NONE), faultReason(nullptr),
      routeId(0u), segmentIndex(0u), segmentCount(0u),
      waypointType(WaypointType::PASS_THROUGH), plannedStart(), plannedEnd(),
      plannedHeadingDeg(0.0f), alongTrackM(0.0f), crossTrackM(0.0f),
      distanceToWaypointM(0.0f), interceptTarget(), steeringTargetDeg(0.0f),
      steeringErrorDeg(0.0f), computedRecoveryBearingDeg(0.0f),
      latchedTurnTargetDeg(0.0f), recoveryGoal(), recoveryAttempt(0u),
      physicalStableMs(0u),
      physicalStopReady(false), finalArrivalPending(false),
      workActionPending(false), workActionPointIndex(static_cast<size_t>(-1)),
      transitionPurpose(RouteTransitionPurpose::NONE),
      steeringResponseRequestedAngularRadps(0.0f),
      steeringResponseHeadingDeltaDeg(0.0f),
      steeringResponseObservationCount(0u),
      steeringResponseWrongDirection(false),
      turnPhase(TurnPhase::NONE), turnTargetDeg(NAN), turnErrorDeg(NAN),
      turnDirection(0), turnPredictedStopAngleDeg(0.0f),
      turnCommandPercent(0), turnCorrectionAttempt(0u),
      turnBreakawayActive(false), turnStartPosition(),
      turnStartHeadingDeg(NAN), turnFirstBrakeHeadingDeg(NAN),
      turnFirstBrakeYawRateDps(NAN), turnFirstPhysicalStopPosition(),
      turnFirstPhysicalStopHeadingDeg(NAN),
      turnLastCorrectionStartHeadingDeg(NAN),
      turnLastPhysicalStopPosition(),
      turnLastPhysicalStopHeadingDeg(NAN), turnFinalErrorDeg(NAN),
      invariantViolation(false) {}

const char* executorStateName(ExecutorState state) {
    switch (state) {
        case ExecutorState::IDLE: return "IDLE";
        case ExecutorState::VALIDATE_ROUTE: return "VALIDATE_ROUTE";
        case ExecutorState::ACQUIRE_SEGMENT: return "ACQUIRE_SEGMENT";
        case ExecutorState::FOLLOW_SEGMENT: return "FOLLOW_SEGMENT";
        case ExecutorState::APPROACH_TRANSITION: return "APPROACH_TRANSITION";
        case ExecutorState::BRAKE: return "BRAKE";
        case ExecutorState::WAIT_PHYSICAL_STOP: return "WAIT_PHYSICAL_STOP";
        case ExecutorState::TURN_BREAKAWAY: return "TURN_BREAKAWAY";
        case ExecutorState::TURN_ROTATE: return "TURN_ROTATE";
        case ExecutorState::TURN_BRAKE: return "TURN_BRAKE";
        case ExecutorState::TURN_EVALUATE: return "TURN_EVALUATE";
        case ExecutorState::TURN_CORRECTION_BREAKAWAY:
            return "TURN_CORRECTION_BREAKAWAY";
        case ExecutorState::TURN_CORRECTION: return "TURN_CORRECTION";
        case ExecutorState::TURN_TO_NEXT: return "TURN_TO_NEXT";
        case ExecutorState::HEADING_STABLE: return "HEADING_STABLE";
        case ExecutorState::INTERCEPT_NEXT_LINE: return "INTERCEPT_NEXT_LINE";
        case ExecutorState::CORNER_MISSED_INTERCEPT:
            return "CORNER_MISSED_INTERCEPT";
        case ExecutorState::TERMINAL_APPROACH: return "TERMINAL_APPROACH";
        case ExecutorState::RECOVERY_PLAN: return "RECOVERY_PLAN";
        case ExecutorState::RECOVERY_REVERSE: return "RECOVERY_REVERSE";
        case ExecutorState::RECOVERY_TURN: return "RECOVERY_TURN";
        case ExecutorState::RECOVERY_APPROACH: return "RECOVERY_APPROACH";
        case ExecutorState::RECOVERY_EVALUATE: return "RECOVERY_EVALUATE";
        case ExecutorState::FINAL_STOP: return "FINAL_STOP";
        case ExecutorState::COMPLETE: return "COMPLETE";
        case ExecutorState::FAULT: return "FAULT";
    }
    return "IDLE";
}

const char* executorResultName(ExecutorResult result) {
    switch (result) {
        case ExecutorResult::ARRIVED: return "arrived";
        case ExecutorResult::STOPPED: return "stopped";
        case ExecutorResult::TIMEOUT: return "timeout";
        case ExecutorResult::FAULT: return "fault";
        default: return "none";
    }
}

const char* waypointTypeName(WaypointType type) {
    switch (type) {
        case WaypointType::PASS_THROUGH: return "PASS_THROUGH";
        case WaypointType::CORNER: return "CORNER";
        case WaypointType::FINAL_POSITION: return "FINAL_POSITION";
        case WaypointType::FINAL_POSE: return "FINAL_POSE";
        case WaypointType::WORK_ACTION: return "WORK_ACTION";
    }
    return "PASS_THROUGH";
}

const char* turnPhaseName(TurnPhase phase) {
    switch (phase) {
        case TurnPhase::BREAKAWAY: return "TURN_BREAKAWAY";
        case TurnPhase::ROTATE: return "TURN_ROTATE";
        case TurnPhase::BRAKE: return "TURN_BRAKE";
        case TurnPhase::WAIT_PHYSICAL_STOP: return "WAIT_PHYSICAL_STOP";
        case TurnPhase::EVALUATE: return "TURN_EVALUATE";
        case TurnPhase::CORRECTION_BREAKAWAY:
            return "TURN_CORRECTION_BREAKAWAY";
        case TurnPhase::CORRECTION: return "TURN_CORRECTION";
        case TurnPhase::HEADING_STABLE: return "HEADING_STABLE";
        default: return "NONE";
    }
}

RouteExecutor::RouteExecutor()
    : state_(ExecutorState::IDLE), previousState_(ExecutorState::IDLE),
      result_(ExecutorResult::NONE), faultReason_(nullptr),
      recoveryReason_(nullptr), recoveryInitializationPending_(false),
      paused_(false), pausedAtMs_(0u), segmentIndex_(0u),
      stateSinceMs_(0u), projection_(), interceptTarget_(),
      steeringTargetDeg_(0.0f), steeringTargetPreviousDeg_(0.0f),
      steeringTargetAtMs_(0u), stopNext_(StopNext::NONE),
      stableNext_(StableNext::NONE), stopStartedMs_(0u),
      physicalStableSinceMs_(0u), physicalStopAnchor_(),
      physicalStopHeadingDeg_(0.0f), physicalStopPvtId_(0u),
      turnTargetDeg_(0.0f), turnTargetLatched_(false), turnDirection_(0),
      turnCorrections_(0u),
      turnStableReferenceDeg_(0.0f), turnStartedMs_(0u),
      turnPhaseStartedMs_(0u), turnBreakawayResponseSinceMs_(0u),
      turnContextState_(ExecutorState::TURN_TO_NEXT),
      turnCycleIsCorrection_(false), turnErrorBeforeCycleDeg_(0.0f),
      turnPredictedStopAngleDeg_(0.0f), turnCommandPercent_(0),
      turnStartPosition_(), turnStartHeadingDeg_(NAN),
      turnFirstBrakeValid_(false), turnFirstBrakeHeadingDeg_(NAN),
      turnFirstBrakeYawRateDps_(NAN),
      turnFirstPhysicalStopValid_(false), turnFirstPhysicalStopPosition_(),
      turnFirstPhysicalStopHeadingDeg_(NAN),
      turnLastCorrectionStartHeadingDeg_(NAN),
      turnLastPhysicalStopPosition_(),
      turnLastPhysicalStopHeadingDeg_(NAN), turnFinalErrorDeg_(NAN),
      scheduledTurnTargetDeg_(0.0f),
      scheduledTurnState_(ExecutorState::TURN_TO_NEXT),
      scheduledStableNext_(StableNext::NONE),
      progressSinceMs_(0u), progressPvtId_(0u),
      progressBestAlongM_(0.0f), progressBestDistanceM_(0.0f),
      progressBestCrossM_(0.0f), recoveryAttempt_(0u),
      recoverySegmentIndex_(static_cast<size_t>(-1)),
      recoveryActive_(false), recoveryGoal_(),
      computedRecoveryBearingDeg_(0.0f), recoveryBaselineDistanceM_(0.0f),
      recoveryBaselineCrossM_(0.0f), recoveryMotionStart_(),
      recoveryMotionStartedMs_(0u), recoveryGoalReached_(false),
      steeringResponseAngularSign_(0),
      steeringResponseCommandSinceMs_(0u),
      steeringResponseLastObservationMs_(0u),
      steeringResponseLastPvtId_(0u),
      steeringResponseLastHeadingDeg_(0.0f),
      steeringResponseRequestedAngularRadps_(0.0f),
      steeringResponseHeadingErrorDeg_(0.0f),
      steeringResponseHeadingDeltaDeg_(0.0f),
      steeringResponseObservationCount_(0u),
      steeringResponseWrongDirection_(false),
      transitionPurpose_(RouteTransitionPurpose::NONE), output_() {}

bool RouteExecutor::start(const RoutePlan& plan,
                          const RouteExecutorConfig& config,
                          const RouteExecutorInput& input) {
    if (!plan.valid() || !finiteInput(input) ||
        !hasValidFinalPosePreApproach(plan)) return false;
    plan_ = plan;
    config_ = config;
    result_ = ExecutorResult::NONE;
    faultReason_ = nullptr;
    recoveryReason_ = nullptr;
    recoveryInitializationPending_ = false;
    paused_ = false;
    pausedAtMs_ = 0u;
    segmentIndex_ = 0u;
    projection_ = SegmentProjection();
    interceptTarget_ = plan_.segment(0u).plannedStart;
    steeringTargetDeg_ = plan_.segment(0u).plannedHeadingDeg;
    steeringTargetPreviousDeg_ = steeringTargetDeg_;
    steeringTargetAtMs_ = input.nowMs;
    stopNext_ = StopNext::NONE;
    stableNext_ = StableNext::NONE;
    physicalStableSinceMs_ = 0u;
    turnTargetLatched_ = false;
    turnDirection_ = 0;
    turnCorrections_ = 0u;
    turnStartedMs_ = 0u;
    turnPhaseStartedMs_ = 0u;
    turnBreakawayResponseSinceMs_ = 0u;
    turnCycleIsCorrection_ = false;
    turnErrorBeforeCycleDeg_ = 0.0f;
    turnPredictedStopAngleDeg_ = 0.0f;
    turnCommandPercent_ = 0;
    turnStartPosition_ = input.position;
    turnStartHeadingDeg_ = NAN;
    turnFirstBrakeValid_ = false;
    turnFirstBrakeHeadingDeg_ = NAN;
    turnFirstBrakeYawRateDps_ = NAN;
    turnFirstPhysicalStopValid_ = false;
    turnFirstPhysicalStopHeadingDeg_ = NAN;
    turnLastCorrectionStartHeadingDeg_ = NAN;
    turnLastPhysicalStopPosition_ = LocalPoint();
    turnLastPhysicalStopHeadingDeg_ = NAN;
    turnFinalErrorDeg_ = NAN;
    scheduledTurnTargetDeg_ = steeringTargetDeg_;
    scheduledTurnState_ = ExecutorState::TURN_TO_NEXT;
    scheduledStableNext_ = StableNext::NONE;
    resetProgress(input.nowMs);
    recoveryAttempt_ = 0u;
    recoverySegmentIndex_ = static_cast<size_t>(-1);
    recoveryActive_ = false;
    recoveryGoal_ = plan_.segment(0u).plannedEnd;
    computedRecoveryBearingDeg_ = 0.0f;
    recoveryBaselineDistanceM_ = 0.0f;
    recoveryBaselineCrossM_ = 0.0f;
    recoveryMotionStart_ = input.position;
    recoveryMotionStartedMs_ = input.nowMs;
    recoveryGoalReached_ = false;
    transitionPurpose_ = RouteTransitionPurpose::NONE;
    resetSteeringResponseWatchdog();
    steeringResponseLastHeadingDeg_ = input.headingDeg;
    steeringResponseLastPvtId_ = input.pvtId;
    steeringResponseLastObservationMs_ = input.nowMs;
    output_ = RouteExecutorOutput();
    previousState_ = ExecutorState::IDLE;
    state_ = ExecutorState::VALIDATE_ROUTE;
    stateSinceMs_ = input.nowMs;
    return true;
}

bool RouteExecutor::active() const {
    return state_ != ExecutorState::IDLE &&
           state_ != ExecutorState::COMPLETE &&
           state_ != ExecutorState::FAULT;
}

bool RouteExecutor::routeFinished() const {
    return state_ == ExecutorState::COMPLETE || state_ == ExecutorState::FAULT;
}

void RouteExecutor::stop(const char* reason) {
    (void)reason;
    previousState_ = state_;
    state_ = ExecutorState::IDLE;
    result_ = ExecutorResult::STOPPED;
    faultReason_ = nullptr;
    paused_ = false;
    recoveryActive_ = false;
    recoveryInitializationPending_ = false;
    recoveryReason_ = nullptr;
    segmentIndex_ = 0u;
    stopNext_ = StopNext::NONE;
    stableNext_ = StableNext::NONE;
    physicalStableSinceMs_ = 0u;
    turnTargetLatched_ = false;
    turnDirection_ = 0;
    turnCorrections_ = 0u;
    turnPhaseStartedMs_ = 0u;
    turnBreakawayResponseSinceMs_ = 0u;
    turnCycleIsCorrection_ = false;
    turnCommandPercent_ = 0;
    recoveryAttempt_ = 0u;
    recoverySegmentIndex_ = static_cast<size_t>(-1);
    recoveryGoalReached_ = false;
    transitionPurpose_ = RouteTransitionPurpose::NONE;
    resetSteeringResponseWatchdog();
    plan_.clear();
    output_ = RouteExecutorOutput();
    output_.result = result_;
}

void RouteExecutor::expire(const char* reason) {
    const bool finalArrivalPending =
        state_ == ExecutorState::FINAL_STOP ||
        (state_ == ExecutorState::WAIT_PHYSICAL_STOP &&
         stopNext_ == StopNext::FINAL_COMPLETE);
    if (result_ == ExecutorResult::ARRIVED ||
        state_ == ExecutorState::COMPLETE || finalArrivalPending)
        return;
    fail(reason ? reason : "timeout", stateSinceMs_, ExecutorResult::TIMEOUT);
}

void RouteExecutor::pause(uint32_t nowMs) {
    if (!active() || paused_) return;
    paused_ = true;
    pausedAtMs_ = nowMs;
}

void RouteExecutor::resume(uint32_t nowMs) {
    if (!paused_) return;
    if (nowMs == 0u) nowMs = pausedAtMs_;
    const uint32_t pausedDuration = elapsedMillis(nowMs, pausedAtMs_);
    // State deadlines measure active execution time, not operator pause time.
    stateSinceMs_ += pausedDuration;
    stopStartedMs_ += pausedDuration;
    if (physicalStableSinceMs_ != 0u)
        physicalStableSinceMs_ += pausedDuration;
    if (steeringTargetAtMs_ != 0u)
        steeringTargetAtMs_ += pausedDuration;
    if (turnStartedMs_ != 0u) turnStartedMs_ += pausedDuration;
    if (turnPhaseStartedMs_ != 0u) turnPhaseStartedMs_ += pausedDuration;
    if (turnBreakawayResponseSinceMs_ != 0u)
        turnBreakawayResponseSinceMs_ += pausedDuration;
    if (recoveryMotionStartedMs_ != 0u)
        recoveryMotionStartedMs_ += pausedDuration;
    if (progressSinceMs_ != 0u) progressSinceMs_ += pausedDuration;
    paused_ = false;
    pausedAtMs_ = 0u;
}

void RouteExecutor::requestRecovery(const char* reason) {
    if (!active()) return;
    recoveryReason_ = reason ? reason : "recovery_requested";
    recoveryInitializationPending_ = true;
}

void RouteExecutor::setState(ExecutorState state, uint32_t nowMs) {
    if (state_ == state) return;
    previousState_ = state_;
    state_ = state;
    stateSinceMs_ = nowMs;
}

bool RouteExecutor::isTerminalSegment() const {
    return plan_.valid() && segmentIndex_ + 1u == plan_.segmentCount();
}

bool RouteExecutor::finalRadiusSatisfied() const {
    if (!isTerminalSegment()) return false;
    const RoutePoint& point = plan_.point(segmentIndex_ + 1u);
    return projection_.distanceToEndM <= point.positionToleranceM;
}

void RouteExecutor::refreshGeometry(const RouteExecutorInput& input) {
    if (!plan_.valid() || segmentIndex_ >= plan_.segmentCount()) return;
    const RouteSegment& segment = plan_.segment(segmentIndex_);
    projection_ = projectToSegment(segment, input.position);
    interceptTarget_ = computeInterceptTarget(
        segment, input.position, config_.interceptLookaheadM).target;
}

void RouteExecutor::enterBrake(StopNext next,
                               const RouteExecutorInput& input,
                               ExecutorState visibleState) {
    stopNext_ = next;
    stopStartedMs_ = input.nowMs;
    physicalStableSinceMs_ = 0u;
    physicalStopAnchor_ = input.position;
    physicalStopHeadingDeg_ = input.headingDeg;
    physicalStopPvtId_ = input.pvtId;
    if (next != StopNext::STEERING_RESPONSE_FAULT)
        resetSteeringResponseWatchdog();
    setState(visibleState, input.nowMs);
}

void RouteExecutor::scheduleTurn(float targetDeg,
                                 ExecutorState turnState,
                                 StableNext stableNext,
                                 const RouteExecutorInput& input) {
    scheduledTurnTargetDeg_ = normalizeHeadingDeg(targetDeg);
    scheduledTurnState_ = turnState;
    scheduledStableNext_ = stableNext;
    enterBrake(StopNext::START_TURN, input);
}

void RouteExecutor::resetProgress(uint32_t nowMs) {
    progressSinceMs_ = nowMs;
    progressPvtId_ = 0u;
    progressBestAlongM_ = -std::numeric_limits<float>::infinity();
    progressBestDistanceM_ = std::numeric_limits<float>::infinity();
    progressBestCrossM_ = std::numeric_limits<float>::infinity();
}

bool RouteExecutor::updateProgress(const RouteExecutorInput& input) {
    if (input.pvtId != progressPvtId_) {
        progressPvtId_ = input.pvtId;
        const float cross = std::fabs(projection_.crossTrackM);
        const bool first = !std::isfinite(progressBestAlongM_) ||
                           !std::isfinite(progressBestDistanceM_) ||
                           !std::isfinite(progressBestCrossM_);
        const bool improved = first ||
            projection_.alongTrackM >=
                progressBestAlongM_ + config_.progressMinM ||
            projection_.distanceToEndM <=
                progressBestDistanceM_ - config_.progressMinM ||
            cross <= progressBestCrossM_ - config_.progressMinM;
        if (improved) {
            progressBestAlongM_ = projection_.alongTrackM;
            progressBestDistanceM_ = projection_.distanceToEndM;
            progressBestCrossM_ = cross;
            progressSinceMs_ = input.nowMs;
        }
    }
    if (elapsedMillis(input.nowMs, progressSinceMs_) >
        config_.progressTimeoutMs) {
        fail("route_not_progressing", input.nowMs);
        return false;
    }
    return true;
}

void RouteExecutor::resetSteeringResponseWatchdog() {
    steeringResponseAngularSign_ = 0;
    steeringResponseCommandSinceMs_ = 0u;
    steeringResponseLastObservationMs_ = 0u;
    steeringResponseLastPvtId_ = 0u;
    steeringResponseLastHeadingDeg_ = 0.0f;
    steeringResponseRequestedAngularRadps_ = 0.0f;
    steeringResponseHeadingErrorDeg_ = 0.0f;
    steeringResponseHeadingDeltaDeg_ = 0.0f;
    steeringResponseObservationCount_ = 0u;
    steeringResponseWrongDirection_ = false;
}

bool RouteExecutor::updateSteeringResponseWatchdog(
        const RouteExecutorInput& input,
        float requestedAngularRadps,
        float headingErrorDeg) {
    const int requestedSign =
        requestedAngularRadps >= config_.steeringResponseMinAngularRadps ? 1 :
        (requestedAngularRadps <= -config_.steeringResponseMinAngularRadps
             ? -1 : 0);
    steeringResponseRequestedAngularRadps_ = requestedAngularRadps;
    steeringResponseHeadingErrorDeg_ = headingErrorDeg;

    const bool commandEffective =
        std::max(std::abs(input.commandLeft), std::abs(input.commandRight)) >=
            config_.steeringResponseMinCommandPercent;
    const bool fresh = input.feedbackAgeMs <= config_.feedbackFreshMs &&
                       input.imuAgeMs <= config_.imuFreshMs &&
                       input.pvtAgeMs <= config_.pvtFreshMs;
    const bool moving = std::fabs(input.estimatedSpeedMps) >=
                        config_.steeringResponseMinSpeedMps;

    if (requestedSign == 0 || !commandEffective || !fresh || !moving) {
        steeringResponseAngularSign_ = 0;
        steeringResponseObservationCount_ = 0u;
        steeringResponseWrongDirection_ = false;
        steeringResponseLastHeadingDeg_ = input.headingDeg;
        steeringResponseLastPvtId_ = input.pvtId;
        steeringResponseLastObservationMs_ = input.nowMs;
        return false;
    }

    if (requestedSign != steeringResponseAngularSign_) {
        steeringResponseAngularSign_ = requestedSign;
        steeringResponseCommandSinceMs_ = input.nowMs;
        steeringResponseObservationCount_ = 0u;
        steeringResponseWrongDirection_ = false;
        steeringResponseLastHeadingDeg_ = input.headingDeg;
        steeringResponseLastPvtId_ = input.pvtId;
        steeringResponseLastObservationMs_ = input.nowMs;
        return false;
    }

    if (!isNewPvtId(input.pvtId, steeringResponseLastPvtId_) ||
        elapsedMillis(input.nowMs, steeringResponseLastObservationMs_) <
            config_.steeringResponseObservationMinMs) {
        return false;
    }

    const float headingDelta = wrapHeadingErrorDeg(
        input.headingDeg - steeringResponseLastHeadingDeg_);
    steeringResponseHeadingDeltaDeg_ = headingDelta;
    steeringResponseLastHeadingDeg_ = input.headingDeg;
    steeringResponseLastPvtId_ = input.pvtId;
    steeringResponseLastObservationMs_ = input.nowMs;

    if (elapsedMillis(input.nowMs, steeringResponseCommandSinceMs_) <
        config_.steeringResponseSettleMs) {
        steeringResponseObservationCount_ = 0u;
        steeringResponseWrongDirection_ = false;
        return false;
    }

    const bool headingOpposite =
        std::fabs(headingDelta) >=
            config_.steeringResponseMinHeadingDeltaDeg &&
        headingDelta * static_cast<float>(requestedSign) < 0.0f;
    const bool yawOpposite =
        std::fabs(input.yawRateDps) >=
            config_.steeringResponseMinYawRateDps &&
        input.yawRateDps * static_cast<float>(requestedSign) < 0.0f;
    steeringResponseWrongDirection_ = headingOpposite && yawOpposite;
    if (steeringResponseWrongDirection_) {
        if (steeringResponseObservationCount_ < UINT8_MAX)
            ++steeringResponseObservationCount_;
    } else {
        steeringResponseObservationCount_ = 0u;
    }

    if (config_.steeringResponseWrongDirectionCount > 0u &&
        steeringResponseObservationCount_ >=
            config_.steeringResponseWrongDirectionCount) {
        transitionPurpose_ =
            RouteTransitionPurpose::STEERING_RESPONSE_FAULT;
        enterBrake(StopNext::STEERING_RESPONSE_FAULT, input);
        return true;
    }
    return false;
}

bool RouteExecutor::physicalStopInstant(const RouteExecutorInput& input) const {
    return input.commandLeft == 0 && input.commandRight == 0 &&
           input.feedbackAgeMs <= config_.feedbackFreshMs &&
           input.imuAgeMs <= config_.imuFreshMs &&
           input.pvtAgeMs <= config_.pvtFreshMs &&
           std::abs(input.measuredLeft) <= config_.physicalStopMotorThreshold &&
           std::abs(input.measuredRight) <= config_.physicalStopMotorThreshold &&
           std::fabs(input.yawRateDps) <= config_.physicalStopYawRateDps;
}

bool RouteExecutor::updatePhysicalStop(const RouteExecutorInput& input) {
    const float displacement = distance(input.position, physicalStopAnchor_);
    const float headingDrift = std::fabs(wrapHeadingErrorDeg(
        input.headingDeg - physicalStopHeadingDeg_));
    const bool instant = physicalStopInstant(input);
    if (!instant || displacement > config_.physicalStopPvtDisplacementM ||
        headingDrift > config_.physicalStopHeadingDriftDeg) {
        physicalStableSinceMs_ = 0u;
        physicalStopAnchor_ = input.position;
        physicalStopHeadingDeg_ = input.headingDeg;
        physicalStopPvtId_ = input.pvtId;
        return false;
    }
    if (physicalStableSinceMs_ == 0u) {
        physicalStableSinceMs_ = input.nowMs;
        physicalStopAnchor_ = input.position;
        physicalStopHeadingDeg_ = input.headingDeg;
        physicalStopPvtId_ = input.pvtId;
        return false;
    }
    return elapsedMillis(input.nowMs, physicalStableSinceMs_) >=
           config_.physicalStopStableMs;
}

void RouteExecutor::startTurn(float targetDeg,
                              ExecutorState turnState,
                              StableNext stableNext,
                              const RouteExecutorInput& input,
                              bool resetCorrections) {
    turnTargetDeg_ = normalizeHeadingDeg(targetDeg);
    turnTargetLatched_ = true;
    const float error = wrapHeadingErrorDeg(turnTargetDeg_ - input.headingDeg);
    turnDirection_ = error >= 0.0f ? 1 : -1;
    turnContextState_ = turnState;
    if (resetCorrections) {
        turnCorrections_ = 0u;
        turnStartedMs_ = input.nowMs;
        turnStartPosition_ = input.position;
        turnStartHeadingDeg_ = input.headingDeg;
        turnFirstBrakeValid_ = false;
        turnFirstPhysicalStopValid_ = false;
        turnFirstBrakeHeadingDeg_ = NAN;
        turnFirstBrakeYawRateDps_ = NAN;
        turnFirstPhysicalStopHeadingDeg_ = NAN;
        turnLastCorrectionStartHeadingDeg_ = NAN;
        turnLastPhysicalStopHeadingDeg_ = NAN;
        turnFinalErrorDeg_ = error;
    }
    turnCycleIsCorrection_ = !resetCorrections;
    turnErrorBeforeCycleDeg_ = std::fabs(error);
    turnPhaseStartedMs_ = input.nowMs;
    turnBreakawayResponseSinceMs_ = 0u;
    physicalStableSinceMs_ = 0u;
    turnPredictedStopAngleDeg_ = 0.0f;
    turnCommandPercent_ = 0;
    stableNext_ = stableNext;
    setState(resetCorrections ? ExecutorState::TURN_BREAKAWAY
                              : ExecutorState::TURN_CORRECTION_BREAKAWAY,
             input.nowMs);
    updateTurnBreakaway(input);
}

float RouteExecutor::turnAngularForPercent(int percent) const {
    const float scale = config_.turnAngularPercentPerRadps > 0.01f
        ? config_.turnAngularPercentPerRadps : 1.0f;
    return static_cast<float>(percent) / scale;
}

float RouteExecutor::predictedTurnStopAngleDeg(
        const RouteExecutorInput& input, float errorDeg) const {
    (void)errorDeg;
    const float decel = config_.turnEstimatedAngularDecelDegps2 > 1.0f
        ? config_.turnEstimatedAngularDecelDegps2 : 1.0f;
    const float yaw = std::fabs(input.yawRateDps);
    const float prediction = yaw * yaw / (2.0f * decel) +
                             config_.turnBrakeMarginDeg;
    return clampFloat(prediction, config_.turnPredictedStopMinDeg,
                      config_.turnPredictedStopMaxDeg);
}

void RouteExecutor::updateTurnBreakaway(const RouteExecutorInput& input) {
    if (elapsedMillis(input.nowMs, turnStartedMs_) > config_.turnTimeoutMs) {
        fail("turn_not_converging", input.nowMs);
        return;
    }
    const uint32_t elapsed = elapsedMillis(input.nowMs, turnPhaseStartedMs_);
    if (elapsed > config_.turnBreakawayMaxMs) {
        fail("turn_motor_no_response", input.nowMs);
        return;
    }
    const bool response = input.yawRateDps *
            static_cast<float>(turnDirection_) >=
        config_.turnBreakawayYawRateThresholdDps;
    if (response) {
        if (turnBreakawayResponseSinceMs_ == 0u)
            turnBreakawayResponseSinceMs_ = input.nowMs;
        if (elapsedMillis(input.nowMs, turnBreakawayResponseSinceMs_) >=
            config_.turnBreakawayResponseStableMs) {
            setState(turnCycleIsCorrection_ ? ExecutorState::TURN_CORRECTION
                                            : ExecutorState::TURN_ROTATE,
                     input.nowMs);
            updateTurn(input, state_);
            return;
        }
    } else {
        turnBreakawayResponseSinceMs_ = 0u;
    }
    turnCommandPercent_ = elapsed >= config_.turnBreakawayBoostAfterMs
        ? config_.turnBreakawayBoostPercent : config_.turnBreakawayPercent;
    output_.motion = MotionKind::TURN_IN_PLACE;
    output_.linearMps = 0.0f;
    output_.angularRadps = static_cast<float>(turnDirection_) *
                           turnAngularForPercent(turnCommandPercent_);
}

void RouteExecutor::beginTurnBrake(const RouteExecutorInput& input) {
    if (!turnFirstBrakeValid_) {
        turnFirstBrakeValid_ = true;
        turnFirstBrakeHeadingDeg_ = input.headingDeg;
        turnFirstBrakeYawRateDps_ = input.yawRateDps;
    }
    turnCommandPercent_ = 0;
    enterBrake(StopNext::TURN_EVALUATE, input, ExecutorState::TURN_BRAKE);
}

void RouteExecutor::updateTurn(const RouteExecutorInput& input,
                               ExecutorState turnState) {
    if (elapsedMillis(input.nowMs, turnStartedMs_) >
        config_.turnTimeoutMs) {
        fail("turn_not_converging", input.nowMs);
        return;
    }
    const float error = wrapHeadingErrorDeg(turnTargetDeg_ - input.headingDeg);
    const float absError = std::fabs(error);
    turnPredictedStopAngleDeg_ = predictedTurnStopAngleDeg(input, error);
    const int errorDirection = error >= 0.0f ? 1 : -1;
    const bool yawTowardTarget = input.yawRateDps *
            static_cast<float>(errorDirection) >
        config_.turnSettleYawRateDps;
    const bool predictedBrake = yawTowardTarget &&
        absError <= turnPredictedStopAngleDeg_;
    const bool overshot = errorDirection != turnDirection_;
    if (absError <= config_.turnToleranceDeg || predictedBrake || overshot) {
        beginTurnBrake(input);
        return;
    }
    turnCommandPercent_ = absError > config_.turnSlowdownEnterDeg
        ? config_.turnFarCommandPercent : config_.turnSlowCommandPercent;
    output_.motion = MotionKind::TURN_IN_PLACE;
    output_.linearMps = 0.0f;
    output_.angularRadps = static_cast<float>(turnDirection_) *
                           turnAngularForPercent(turnCommandPercent_);
    (void)turnState;
}

void RouteExecutor::beginTurnCorrection(const RouteExecutorInput& input,
                                        float errorDeg) {
    turnDirection_ = errorDeg >= 0.0f ? 1 : -1;
    turnCycleIsCorrection_ = true;
    turnErrorBeforeCycleDeg_ = std::fabs(errorDeg);
    turnLastCorrectionStartHeadingDeg_ = input.headingDeg;
    turnPhaseStartedMs_ = input.nowMs;
    turnBreakawayResponseSinceMs_ = 0u;
    turnPredictedStopAngleDeg_ = 0.0f;
    turnCommandPercent_ = 0;
    setState(ExecutorState::TURN_CORRECTION_BREAKAWAY, input.nowMs);
}

void RouteExecutor::updateTurnEvaluate(const RouteExecutorInput& input) {
    const float stoppedHeading = std::isfinite(turnLastPhysicalStopHeadingDeg_)
        ? turnLastPhysicalStopHeadingDeg_ : input.headingDeg;
    const float error = wrapHeadingErrorDeg(turnTargetDeg_ - stoppedHeading);
    const float absError = std::fabs(error);
    turnFinalErrorDeg_ = error;
    if (turnCycleIsCorrection_) {
        if (turnCorrections_ < UINT8_MAX) ++turnCorrections_;
        const bool improved = absError <= config_.turnToleranceDeg ||
            absError <= turnErrorBeforeCycleDeg_ -
                            config_.turnCorrectionMinImprovementDeg;
        if (!improved) {
            fail("turn_not_converging", input.nowMs);
            return;
        }
    }
    if (absError <= config_.turnToleranceDeg) {
        turnCycleIsCorrection_ = false;
        turnStableReferenceDeg_ = input.headingDeg;
        physicalStableSinceMs_ = 0u;
        physicalStopAnchor_ = input.position;
        physicalStopHeadingDeg_ = input.headingDeg;
        setState(ExecutorState::HEADING_STABLE, input.nowMs);
        return;
    }
    if (turnCorrections_ >= config_.turnMaxCorrectionAttempts) {
        fail("turn_not_converging", input.nowMs);
        return;
    }
    beginTurnCorrection(input, error);
    updateTurnBreakaway(input);
}

void RouteExecutor::updateHeadingStable(const RouteExecutorInput& input) {
    if (!updatePhysicalStop(input)) return;
    const float error = wrapHeadingErrorDeg(turnTargetDeg_ - input.headingDeg);
    const float drift = std::fabs(wrapHeadingErrorDeg(
        input.headingDeg - turnStableReferenceDeg_));
    if (std::fabs(error) > config_.turnToleranceDeg ||
        drift > config_.physicalStopHeadingDriftDeg) {
        if (turnCorrections_ >= config_.turnMaxCorrectionAttempts) {
            fail("turn_not_converging", input.nowMs);
            return;
        }
        beginTurnCorrection(input, error);
        return;
    }
    switch (stableNext_) {
        case StableNext::NEXT_SEGMENT_INTERCEPT:
            if (++segmentIndex_ >= plan_.segmentCount()) {
                fail("route_state_critical", input.nowMs);
                return;
            }
            acquireSegment(input.nowMs, true);
            break;
        case StableNext::RECOVERY_APPROACH:
            recoveryMotionStart_ = input.position;
            recoveryMotionStartedMs_ = input.nowMs;
            recoveryGoalReached_ = false;
            steeringTargetDeg_ = turnTargetDeg_;
            steeringTargetAtMs_ = input.nowMs;
            setState(ExecutorState::RECOVERY_APPROACH, input.nowMs);
            break;
        case StableNext::RESUME_INTERCEPT:
            acquireSegment(input.nowMs, true);
            break;
        default:
            fail("route_state_critical", input.nowMs);
            break;
    }
}

void RouteExecutor::acquireSegment(uint32_t nowMs, bool intercept) {
    if (!plan_.valid() || segmentIndex_ >= plan_.segmentCount()) {
        fail("route_state_critical", nowMs);
        return;
    }
    steeringTargetDeg_ = plan_.segment(segmentIndex_).plannedHeadingDeg;
    steeringTargetPreviousDeg_ = steeringTargetDeg_;
    steeringTargetAtMs_ = nowMs;
    turnTargetLatched_ = false;
    recoveryActive_ = false;
    resetProgress(nowMs);
    setState(intercept ? ExecutorState::INTERCEPT_NEXT_LINE
                       : ExecutorState::FOLLOW_SEGMENT,
             nowMs);
}

void RouteExecutor::beginRecovery(const RouteExecutorInput& input,
                                  const char* reason) {
    recoveryReason_ = reason;
    recoveryInitializationPending_ = false;
    // Attempts are bounded per planned segment, including a later re-entry
    // after an apparently successful pre-approach.  Otherwise a repeated
    // miss could restart the counter forever.
    if (recoverySegmentIndex_ != segmentIndex_) {
        recoverySegmentIndex_ = segmentIndex_;
        recoveryAttempt_ = 0u;
    } else if (recoveryAttempt_ >= config_.recoveryMaxAttempts) {
        fail(isTerminalSegment() ? "recovery_not_converging"
                                 : "transition_missed",
             input.nowMs);
        return;
    }
    recoveryActive_ = true;
    transitionPurpose_ = RouteTransitionPurpose::RECOVERY;
    recoveryBaselineDistanceM_ = projection_.distanceToEndM;
    recoveryBaselineCrossM_ = std::fabs(projection_.crossTrackM);
    const RouteSegment& segment = plan_.segment(segmentIndex_);
    const RoutePoint& targetPoint = plan_.point(segmentIndex_ + 1u);
    if (isTerminalSegment() &&
        targetPoint.type == WaypointType::FINAL_POSITION) {
        // FINAL_POSITION has no pose constraint. Recover toward the fixed
        // endpoint itself; never manufacture FINAL_POSE pre-approach geometry
        // or a turn merely to regain the line heading near the endpoint.
        recoveryGoal_ = segment.plannedEnd;
    } else {
        const float approachAlong = clampFloat(
            segment.segmentLengthM - config_.finalPosePreApproachM,
            0.0f, segment.segmentLengthM);
        const float u = segment.segmentLengthM > 0.001f
            ? approachAlong / segment.segmentLengthM : 0.0f;
        recoveryGoal_.x = segment.plannedStart.x +
            (segment.plannedEnd.x - segment.plannedStart.x) * u;
        recoveryGoal_.y = segment.plannedStart.y +
            (segment.plannedEnd.y - segment.plannedStart.y) * u;
    }
    computedRecoveryBearingDeg_ = bearingDeg(input.position, recoveryGoal_);
    turnTargetLatched_ = false;
    recoveryGoalReached_ = false;
    setState(ExecutorState::RECOVERY_PLAN, input.nowMs);
}

void RouteExecutor::updateRecoveryPlan(const RouteExecutorInput& input) {
    computedRecoveryBearingDeg_ = bearingDeg(input.position, recoveryGoal_);
    const bool shouldReverse = input.reverseEnabled &&
        input.reversePathAllowed &&
        projection_.alongTrackM >
            plan_.segment(segmentIndex_).segmentLengthM;
    enterBrake(shouldReverse ? StopNext::RECOVERY_REVERSE
                             : StopNext::RECOVERY_TURN,
               input);
}

void RouteExecutor::updateRecoveryApproach(const RouteExecutorInput& input) {
    if (finalRadiusSatisfied()) {
        const RoutePoint& finalPoint = plan_.point(segmentIndex_ + 1u);
        const bool finalHeadingSatisfied =
            finalPoint.type == WaypointType::FINAL_POSITION ||
            std::fabs(wrapHeadingErrorDeg(
                finalPoint.finalHeadingDeg - input.headingDeg)) <=
                finalPoint.headingToleranceDeg;
        if (finalHeadingSatisfied) {
            enterBrake(StopNext::FINAL_COMPLETE, input,
                       ExecutorState::FINAL_STOP);
            return;
        }
    }
    const float goalDistance = distance(input.position, recoveryGoal_);
    const float travelled = distance(input.position, recoveryMotionStart_);
    if (goalDistance <= config_.recoveryGoalToleranceM) {
        recoveryGoalReached_ = true;
        enterBrake(StopNext::RECOVERY_EVALUATE, input);
        return;
    }
    if (travelled >= config_.recoveryApproachMaxM ||
        elapsedMillis(input.nowMs, recoveryMotionStartedMs_) >=
            config_.recoveryApproachTimeoutMs) {
        enterBrake(StopNext::RECOVERY_EVALUATE, input);
        return;
    }
    updateLineMotion(input, false, false, true);
}

void RouteExecutor::updateRecoveryEvaluate(const RouteExecutorInput& input) {
    ++recoveryAttempt_;
    const float distNow = projection_.distanceToEndM;
    const float crossNow = std::fabs(projection_.crossTrackM);
    const bool improved = recoveryGoalReached_ ||
        distNow <= recoveryBaselineDistanceM_ - config_.recoveryProgressMinM ||
        crossNow <= recoveryBaselineCrossM_ - config_.recoveryProgressMinM;
    const RoutePoint& targetPoint = plan_.point(segmentIndex_ + 1u);
    const bool finalPositionRecovery = isTerminalSegment() &&
        targetPoint.type == WaypointType::FINAL_POSITION;
    if (recoveryGoalReached_ && !finalPositionRecovery) {
        startTurn(plan_.segment(segmentIndex_).plannedHeadingDeg,
                  ExecutorState::RECOVERY_TURN,
                  StableNext::RESUME_INTERCEPT, input);
        return;
    }
    if (recoveryAttempt_ >= config_.recoveryMaxAttempts) {
        fail(isTerminalSegment() ? "recovery_not_converging"
                                 : "transition_missed",
             input.nowMs);
        return;
    }
    if (improved) {
        recoveryBaselineDistanceM_ = distNow;
        recoveryBaselineCrossM_ = crossNow;
    }
    computedRecoveryBearingDeg_ = bearingDeg(input.position, recoveryGoal_);
    setState(ExecutorState::RECOVERY_PLAN, input.nowMs);
}

void RouteExecutor::completeArrived(uint32_t nowMs) {
    result_ = ExecutorResult::ARRIVED;
    faultReason_ = nullptr;
    recoveryActive_ = false;
    setState(ExecutorState::COMPLETE, nowMs);
}

void RouteExecutor::fail(const char* reason, uint32_t nowMs,
                         ExecutorResult result) {
    if (result_ == ExecutorResult::ARRIVED) return;
    result_ = result;
    faultReason_ = reason ? reason : "route_fault";
    recoveryActive_ = false;
    setState(ExecutorState::FAULT, nowMs);
}

void RouteExecutor::updateLineMotion(const RouteExecutorInput& input,
                                     bool interceptMode,
                                     bool terminalMode,
                                     bool recoveryMode) {
    const RouteSegment& segment = plan_.segment(segmentIndex_);
    const float lineHeading = segment.plannedHeadingDeg;
    float requestedHeading = lineHeading;
    float crossTerm = std::atan(
        config_.stanleyCrossTrackGain * projection_.crossTrackM /
        (config_.stanleySoftSpeedMps +
         std::fabs(input.estimatedSpeedMps)));

    if (interceptMode) {
        const float interceptAngle = clampFloat(
            std::atan(projection_.crossTrackM / 0.60f) * kRadToDeg,
            -35.0f, 35.0f);
        requestedHeading = normalizeHeadingDeg(lineHeading + interceptAngle);
        crossTerm = 0.0f;
    }
    if (recoveryMode) {
        requestedHeading = bearingDeg(input.position, recoveryGoal_);
        crossTerm = 0.0f;
    }

    const float dt = steeringTargetAtMs_ == 0u ? 0.0f :
        clampFloat(elapsedMillis(input.nowMs, steeringTargetAtMs_) * 0.001f,
                   0.0f, 0.20f);
    if (steeringTargetAtMs_ == 0u) steeringTargetDeg_ = requestedHeading;
    else steeringTargetDeg_ = smoothTarget(
        steeringTargetDeg_, requestedHeading, 60.0f, dt);
    steeringTargetAtMs_ = input.nowMs;
    steeringTargetPreviousDeg_ = requestedHeading;

    const float headingError = wrapHeadingErrorDeg(
        steeringTargetDeg_ - input.headingDeg);
    const float absHeadingError = std::fabs(headingError);
    if (recoveryMode && absHeadingError > config_.turnEnterDeg) {
        // A recovery turn target is latched only at the beginning of an
        // attempt.  If the approach geometry changes enough to require a new
        // turn, finish this attempt with a full stop/evaluation; never create
        // an unlatched turn-in-place through a DRIVE command.
        enterBrake(StopNext::RECOVERY_EVALUATE, input);
        return;
    }
    if (!recoveryMode && absHeadingError > config_.turnEnterDeg) {
        transitionPurpose_ = RouteTransitionPurpose::STEERING_REACQUIRE;
        scheduleTurn(steeringTargetDeg_,
                     ExecutorState::TURN_TO_NEXT,
                     StableNext::RESUME_INTERCEPT, input);
        return;
    }

    float speed = segment.speedLimitMps;
    if (input.degraded && speed > config_.degradedSpeedMps)
        speed = config_.degradedSpeedMps;
    if (interceptMode && speed > config_.interceptSpeedMps)
        speed = config_.interceptSpeedMps;
    if (terminalMode && speed > config_.terminalApproachSpeedMps)
        speed = config_.terminalApproachSpeedMps;
    if (recoveryMode) speed = config_.recoveryApproachSpeedMps;

    const float errNorm = absHeadingError / 45.0f;
    const float slowFactor = errNorm >= 1.0f ? 0.0f
        : 1.0f - errNorm * errNorm;
    float rampFactor = 1.0f;
    if (!recoveryMode && projection_.alongTrackM < config_.rampUpM) {
        const float u = clampFloat(projection_.alongTrackM /
                                   config_.rampUpM, 0.0f, 1.0f);
        rampFactor = u * u * (3.0f - 2.0f * u);
        if (rampFactor < config_.rampMinFactor)
            rampFactor = config_.rampMinFactor;
    }
    speed *= slowFactor * rampFactor;
    if (speed <= 0.0f && absHeadingError < config_.turnEnterDeg)
        speed = recoveryMode ? config_.recoveryApproachSpeedMps :
            (segment.speedLimitMps < 0.06f ? segment.speedLimitMps : 0.06f);

    float angular = config_.stanleyHeadingGain *
        (headingError * kDegToRad) + crossTerm -
        config_.yawRateDamping * (input.yawRateDps * kDegToRad);
    angular = clampFloat(angular, -config_.maxAngularRadps,
                         config_.maxAngularRadps);

    if (updateSteeringResponseWatchdog(input, angular, headingError))
        return;

    output_.motion = MotionKind::DRIVE;
    output_.linearMps = speed;
    output_.angularRadps = angular;
}

void RouteExecutor::populateOutput(const RouteExecutorInput& input) {
    output_.state = state_;
    output_.oldState = previousState_;
    output_.stateChanged = state_ != previousState_;
    output_.result = result_;
    output_.faultReason = faultReason_;
    output_.routeId = plan_.routeId();
    output_.segmentIndex = segmentIndex_;
    output_.segmentCount = plan_.segmentCount();
    output_.computedRecoveryBearingDeg = computedRecoveryBearingDeg_;
    output_.latchedTurnTargetDeg = turnTargetLatched_ ? turnTargetDeg_ : NAN;
    output_.recoveryGoal = recoveryGoal_;
    output_.recoveryAttempt = recoveryAttempt_;
    output_.physicalStableMs = physicalStableSinceMs_ == 0u ? 0u :
        elapsedMillis(input.nowMs, physicalStableSinceMs_);
    output_.physicalStopReady = physicalStableSinceMs_ != 0u &&
        output_.physicalStableMs >= config_.physicalStopStableMs;
    output_.transitionPurpose = transitionPurpose_;
    output_.steeringResponseRequestedAngularRadps =
        steeringResponseRequestedAngularRadps_;
    output_.steeringResponseHeadingDeltaDeg =
        steeringResponseHeadingDeltaDeg_;
    output_.steeringResponseObservationCount =
        steeringResponseObservationCount_;
    output_.steeringResponseWrongDirection =
        steeringResponseWrongDirection_;
    switch (state_) {
        case ExecutorState::TURN_BREAKAWAY:
            output_.turnPhase = TurnPhase::BREAKAWAY;
            break;
        case ExecutorState::TURN_ROTATE:
            output_.turnPhase = TurnPhase::ROTATE;
            break;
        case ExecutorState::TURN_BRAKE:
            output_.turnPhase = TurnPhase::BRAKE;
            break;
        case ExecutorState::TURN_EVALUATE:
            output_.turnPhase = TurnPhase::EVALUATE;
            break;
        case ExecutorState::TURN_CORRECTION_BREAKAWAY:
            output_.turnPhase = TurnPhase::CORRECTION_BREAKAWAY;
            break;
        case ExecutorState::TURN_CORRECTION:
            output_.turnPhase = TurnPhase::CORRECTION;
            break;
        case ExecutorState::HEADING_STABLE:
            output_.turnPhase = TurnPhase::HEADING_STABLE;
            break;
        case ExecutorState::WAIT_PHYSICAL_STOP:
            if (stopNext_ == StopNext::TURN_EVALUATE)
                output_.turnPhase = TurnPhase::WAIT_PHYSICAL_STOP;
            break;
        case ExecutorState::FAULT:
            if (turnTargetLatched_ && std::isfinite(turnLastPhysicalStopHeadingDeg_))
                output_.turnPhase = TurnPhase::EVALUATE;
            break;
        default:
            break;
    }
    output_.turnTargetDeg = turnTargetLatched_ ? turnTargetDeg_ : NAN;
    output_.turnErrorDeg = turnTargetLatched_
        ? wrapHeadingErrorDeg(turnTargetDeg_ - input.headingDeg) : NAN;
    output_.turnDirection = turnDirection_;
    output_.turnPredictedStopAngleDeg = turnPredictedStopAngleDeg_;
    output_.turnCommandPercent = turnCommandPercent_;
    output_.turnCorrectionAttempt = turnCorrections_;
    output_.turnBreakawayActive =
        output_.turnPhase == TurnPhase::BREAKAWAY ||
        output_.turnPhase == TurnPhase::CORRECTION_BREAKAWAY;
    output_.turnStartPosition = turnStartPosition_;
    output_.turnStartHeadingDeg = turnStartHeadingDeg_;
    output_.turnFirstBrakeHeadingDeg = turnFirstBrakeHeadingDeg_;
    output_.turnFirstBrakeYawRateDps = turnFirstBrakeYawRateDps_;
    output_.turnFirstPhysicalStopPosition = turnFirstPhysicalStopPosition_;
    output_.turnFirstPhysicalStopHeadingDeg = turnFirstPhysicalStopHeadingDeg_;
    output_.turnLastCorrectionStartHeadingDeg =
        turnLastCorrectionStartHeadingDeg_;
    output_.turnLastPhysicalStopPosition = turnLastPhysicalStopPosition_;
    output_.turnLastPhysicalStopHeadingDeg = turnLastPhysicalStopHeadingDeg_;
    output_.turnFinalErrorDeg = turnFinalErrorDeg_;
    output_.finalArrivalPending =
        state_ == ExecutorState::FINAL_STOP ||
        (state_ == ExecutorState::WAIT_PHYSICAL_STOP &&
         stopNext_ == StopNext::FINAL_COMPLETE);
    if (plan_.valid() && segmentIndex_ < plan_.segmentCount()) {
        const RouteSegment& segment = plan_.segment(segmentIndex_);
        const RoutePoint& point = plan_.point(segmentIndex_ + 1u);
        output_.waypointType = point.type;
        output_.plannedStart = segment.plannedStart;
        output_.plannedEnd = segment.plannedEnd;
        output_.plannedHeadingDeg = segment.plannedHeadingDeg;
        output_.alongTrackM = projection_.alongTrackM;
        output_.crossTrackM = projection_.crossTrackM;
        output_.distanceToWaypointM = projection_.distanceToEndM;
        output_.interceptTarget = interceptTarget_;
        output_.steeringTargetDeg = steeringTargetDeg_;
        output_.steeringErrorDeg = wrapHeadingErrorDeg(
            steeringTargetDeg_ - input.headingDeg);
    }
    output_.invariantViolation =
        (result_ == ExecutorResult::ARRIVED && state_ != ExecutorState::COMPLETE) ||
        (state_ == ExecutorState::COMPLETE && result_ != ExecutorResult::ARRIVED) ||
        (result_ == ExecutorResult::ARRIVED && faultReason_ != nullptr);
    previousState_ = state_;
}

RouteExecutorOutput RouteExecutor::update(const RouteExecutorInput& input) {
    const size_t segmentAtUpdateStart = segmentIndex_;
    output_ = RouteExecutorOutput();
    output_.state = state_;
    output_.oldState = previousState_;
    if (state_ == ExecutorState::IDLE || state_ == ExecutorState::COMPLETE ||
        state_ == ExecutorState::FAULT) {
        populateOutput(input);
        return output_;
    }
    // Keep the purpose visible for the complete stop/turn/intercept handoff,
    // then clear it only after FOLLOW has been observed for a full update.
    // This avoids stale PLANNED_CORNER labels on a later unrelated brake.
    if (state_ == ExecutorState::FOLLOW_SEGMENT &&
        previousState_ == ExecutorState::FOLLOW_SEGMENT) {
        transitionPurpose_ = RouteTransitionPurpose::NONE;
    }
    if (!finiteInput(input)) {
        fail("internal_nonfinite", input.nowMs);
        populateOutput(input);
        return output_;
    }
    if (paused_ || !input.motionAllowed) {
        populateOutput(input);
        return output_;
    }
    if (!input.currentFootprintAllowed) {
        fail("footprint_blocked", input.nowMs);
        populateOutput(input);
        return output_;
    }
    refreshGeometry(input);
    const bool finalStopPending =
        state_ == ExecutorState::FINAL_STOP ||
        (state_ == ExecutorState::WAIT_PHYSICAL_STOP &&
         stopNext_ == StopNext::FINAL_COMPLETE);
    if (isTerminalSegment() && finalRadiusSatisfied() &&
        !finalStopPending) {
        const RoutePoint& finalPoint = plan_.point(segmentIndex_ + 1u);
        const float finalHeadingError = wrapHeadingErrorDeg(
            finalPoint.finalHeadingDeg - input.headingDeg);
        if (finalPoint.type == WaypointType::FINAL_POSITION ||
            (finalPoint.type == WaypointType::FINAL_POSE &&
             std::fabs(finalHeadingError) <= finalPoint.headingToleranceDeg)) {
            transitionPurpose_ = RouteTransitionPurpose::FINAL_ARRIVAL;
            enterBrake(StopNext::FINAL_COMPLETE, input,
                       ExecutorState::FINAL_STOP);
        } else if (!recoveryActive_) {
            beginRecovery(input, "final_pose_heading_missed");
        }
    }
    const bool finalStopNowPending =
        state_ == ExecutorState::FINAL_STOP ||
        (state_ == ExecutorState::WAIT_PHYSICAL_STOP &&
         stopNext_ == StopNext::FINAL_COMPLETE);
    if (recoveryInitializationPending_ && !finalStopNowPending) {
        beginRecovery(input, recoveryReason_);
    }
    const bool activeForwardPathAllowed =
        state_ == ExecutorState::RECOVERY_APPROACH
            ? input.recoveryPathAllowed : input.forwardPathAllowed;
    if (!activeForwardPathAllowed && stateNeedsForwardPath(state_)) {
        fail("route_segment_blocked", input.nowMs);
        populateOutput(input);
        return output_;
    }
    if ((state_ == ExecutorState::TURN_BREAKAWAY ||
         state_ == ExecutorState::TURN_ROTATE ||
         state_ == ExecutorState::TURN_CORRECTION_BREAKAWAY ||
         state_ == ExecutorState::TURN_CORRECTION) &&
        !input.turnPathAllowed) {
        fail("turn_footprint_blocked", input.nowMs);
        populateOutput(input);
        return output_;
    }
    if (stateTracksRouteProgress(state_) && !updateProgress(input)) {
        populateOutput(input);
        return output_;
    }

    switch (state_) {
        case ExecutorState::VALIDATE_ROUTE:
            if (!plan_.valid()) fail("route_invalid", input.nowMs);
            else setState(ExecutorState::ACQUIRE_SEGMENT, input.nowMs);
            break;
        case ExecutorState::ACQUIRE_SEGMENT:
            acquireSegment(input.nowMs, segmentIndex_ > 0u);
            break;
        case ExecutorState::FOLLOW_SEGMENT:
        case ExecutorState::APPROACH_TRANSITION: {
            const RoutePoint& point = plan_.point(segmentIndex_ + 1u);
            const RouteSegment& segment = plan_.segment(segmentIndex_);
            const bool insideCorridor =
                std::fabs(projection_.crossTrackM) <= segment.corridorHalfWidthM;
            if (point.type == WaypointType::FINAL_POSITION ||
                point.type == WaypointType::FINAL_POSE) {
                if (projection_.distanceToEndM <=
                    config_.terminalApproachDistanceM) {
                    setState(ExecutorState::TERMINAL_APPROACH, input.nowMs);
                } else if (projection_.crossedTransitionPlane) {
                    beginRecovery(input, "waypoint_missed");
                } else {
                    updateLineMotion(input, false, false, false);
                }
                break;
            }
            const bool passThrough =
                point.type == WaypointType::PASS_THROUGH ||
                point.type == WaypointType::WORK_ACTION;
            const bool reached = passThrough
                ? projection_.crossedTransitionPlane
                : (projection_.crossedTransitionPlane ||
                   projection_.distanceToEndM <= point.positionToleranceM);
            if (reached) {
                if (point.type == WaypointType::CORNER) {
                    if (insideCorridor) {
                        transitionPurpose_ =
                            RouteTransitionPurpose::PLANNED_CORNER;
                        enterBrake(StopNext::TURN_NEXT, input);
                    } else {
                        // Crossing the corner plane well beside the planned
                        // corner is not a successful corner. Stop first, then
                        // either enter the next fixed line through a bounded,
                        // footprint-gated intercept or report transition_missed.
                        const bool candidateForNextLineIntercept =
                            std::fabs(projection_.crossTrackM) <=
                                config_.interceptMaxCrossM;
                        transitionPurpose_ =
                            RouteTransitionPurpose::CORNER_MISSED_INTERCEPT;
                        enterBrake(candidateForNextLineIntercept
                                       ? StopNext::CORNER_MISSED_INTERCEPT
                                       : StopNext::CORNER_MISSED_FAULT,
                                   input);
                    }
                } else if (!insideCorridor &&
                           std::fabs(projection_.crossTrackM) >
                               config_.interceptMaxCrossM) {
                    fail("transition_missed", input.nowMs);
                } else {
                    if (point.type == WaypointType::WORK_ACTION) {
                        output_.workActionPending = true;
                        output_.workActionPointIndex = segmentIndex_ + 1u;
                    }
                    if (++segmentIndex_ >= plan_.segmentCount()) {
                        fail("route_state_critical", input.nowMs);
                    } else if (insideCorridor) {
                        acquireSegment(input.nowMs, false);
                    } else {
                        enterBrake(StopNext::START_INTERCEPT, input);
                    }
                }
            } else {
                if (projection_.distanceToEndM <=
                    config_.terminalApproachDistanceM)
                    setState(ExecutorState::APPROACH_TRANSITION, input.nowMs);
                updateLineMotion(input, false, false, false);
            }
            break;
        }
        case ExecutorState::INTERCEPT_NEXT_LINE: {
            const float lineError = std::fabs(wrapHeadingErrorDeg(
                plan_.segment(segmentIndex_).plannedHeadingDeg -
                input.headingDeg));
            const bool nearTerminalEndpoint = isTerminalSegment() &&
                projection_.distanceToEndM <=
                    config_.terminalApproachDistanceM;
            if (std::fabs(projection_.crossTrackM) <=
                    config_.interceptEnterCrossM &&
                lineError <= config_.interceptEnterHeadingDeg) {
                setState(nearTerminalEndpoint
                             ? ExecutorState::TERMINAL_APPROACH
                             : ExecutorState::FOLLOW_SEGMENT,
                          input.nowMs);
            } else if (std::fabs(projection_.crossTrackM) >
                           config_.interceptMaxCrossM ||
                       elapsedMillis(input.nowMs, stateSinceMs_) >
                           config_.interceptTimeoutMs) {
                beginRecovery(input, "intercept_not_converging");
            } else {
                updateLineMotion(input, true, nearTerminalEndpoint, false);
            }
            break;
        }
        case ExecutorState::TERMINAL_APPROACH: {
            const RouteSegment& segment = plan_.segment(segmentIndex_);
            const float remaining = segment.segmentLengthM -
                                    projection_.alongTrackM;
            const float requiredInterceptRun =
                std::fabs(projection_.crossTrackM) /
                std::tan(35.0f * kDegToRad);
            const bool approachUnreachable =
                std::fabs(projection_.crossTrackM) >
                    segment.corridorHalfWidthM &&
                remaining < requiredInterceptRun + 0.05f;
            if (projection_.crossedTransitionPlane && !finalRadiusSatisfied())
                beginRecovery(input, "waypoint_missed");
            else if (approachUnreachable)
                beginRecovery(input, "terminal_approach_unreachable");
            else
                updateLineMotion(input, false, true, false);
            break;
        }
        case ExecutorState::BRAKE:
        case ExecutorState::FINAL_STOP:
        case ExecutorState::TURN_BRAKE:
            setState(ExecutorState::WAIT_PHYSICAL_STOP, input.nowMs);
            break;
        case ExecutorState::WAIT_PHYSICAL_STOP:
            if (elapsedMillis(input.nowMs, stopStartedMs_) >
                config_.physicalStopTimeoutMs) {
                fail(stopNext_ == StopNext::FINAL_COMPLETE
                         ? "final_stop_timeout" : "physical_stop_timeout",
                     input.nowMs);
                break;
            }
            if (!updatePhysicalStop(input)) break;
            switch (stopNext_) {
                case StopNext::START_TURN:
                    if (!input.turnPathAllowed)
                        fail("turn_footprint_blocked", input.nowMs);
                    else
                        startTurn(scheduledTurnTargetDeg_,
                                  scheduledTurnState_,
                                  scheduledStableNext_, input);
                    break;
                case StopNext::TURN_NEXT:
                    if (segmentIndex_ + 1u >= plan_.segmentCount())
                        fail("route_state_critical", input.nowMs);
                    else if (!input.turnPathAllowed)
                        fail("turn_footprint_blocked", input.nowMs);
                    else
                        startTurn(plan_.segment(segmentIndex_ + 1u).plannedHeadingDeg,
                                  ExecutorState::TURN_TO_NEXT,
                                  StableNext::NEXT_SEGMENT_INTERCEPT, input);
                    break;
                case StopNext::TURN_EVALUATE:
                    turnLastPhysicalStopPosition_ = input.position;
                    turnLastPhysicalStopHeadingDeg_ = input.headingDeg;
                    turnFinalErrorDeg_ = wrapHeadingErrorDeg(
                        turnTargetDeg_ - input.headingDeg);
                    if (!turnFirstPhysicalStopValid_) {
                        turnFirstPhysicalStopValid_ = true;
                        turnFirstPhysicalStopPosition_ = input.position;
                        turnFirstPhysicalStopHeadingDeg_ = input.headingDeg;
                    }
                    setState(ExecutorState::TURN_EVALUATE, input.nowMs);
                    break;
                case StopNext::START_INTERCEPT:
                    acquireSegment(input.nowMs, true);
                    break;
                case StopNext::FINAL_COMPLETE:
                    completeArrived(input.nowMs);
                    break;
                case StopNext::RECOVERY_REVERSE:
                    recoveryMotionStart_ = input.position;
                    recoveryMotionStartedMs_ = input.nowMs;
                    setState(ExecutorState::RECOVERY_REVERSE, input.nowMs);
                    break;
                case StopNext::RECOVERY_TURN:
                    if (!input.turnPathAllowed) {
                        fail("turn_footprint_blocked", input.nowMs);
                    } else {
                        computedRecoveryBearingDeg_ = bearingDeg(
                            input.position, recoveryGoal_);
                        startTurn(computedRecoveryBearingDeg_,
                                  ExecutorState::RECOVERY_TURN,
                                  StableNext::RECOVERY_APPROACH, input);
                    }
                    break;
                case StopNext::RECOVERY_EVALUATE:
                    setState(ExecutorState::RECOVERY_EVALUATE,
                             input.nowMs);
                    break;
                case StopNext::CORNER_MISSED_INTERCEPT: {
                    if (segmentIndex_ + 1u >= plan_.segmentCount() ||
                        !input.nextSegmentPathAllowed) {
                        fail("transition_missed", input.nowMs);
                        break;
                    }
                    if (!input.turnPathAllowed) {
                        fail("turn_footprint_blocked", input.nowMs);
                        break;
                    }
                    const size_t nextIndex = segmentIndex_ + 1u;
                    const RouteSegment& nextSegment = plan_.segment(nextIndex);
                    const SegmentProjection nextProjection =
                        projectToSegment(nextSegment, input.position);
                    if (std::fabs(nextProjection.crossTrackM) >
                        config_.interceptMaxCrossM) {
                        fail("transition_missed", input.nowMs);
                        break;
                    }
                    segmentIndex_ = nextIndex;
                    startTurn(nextSegment.plannedHeadingDeg,
                              ExecutorState::CORNER_MISSED_INTERCEPT,
                              StableNext::RESUME_INTERCEPT, input);
                    break;
                }
                case StopNext::CORNER_MISSED_FAULT:
                    fail("transition_missed", input.nowMs);
                    break;
                case StopNext::STEERING_RESPONSE_FAULT:
                    transitionPurpose_ =
                        RouteTransitionPurpose::STEERING_RESPONSE_FAULT;
                    fail("steering_response_wrong_direction", input.nowMs);
                    break;
                default:
                    fail("route_state_critical", input.nowMs);
                    break;
            }
            break;
        case ExecutorState::TURN_BREAKAWAY:
        case ExecutorState::TURN_CORRECTION_BREAKAWAY:
            updateTurnBreakaway(input);
            break;
        case ExecutorState::TURN_ROTATE:
        case ExecutorState::TURN_CORRECTION:
            updateTurn(input, state_);
            break;
        case ExecutorState::TURN_EVALUATE:
            updateTurnEvaluate(input);
            break;
        case ExecutorState::TURN_TO_NEXT:
        case ExecutorState::CORNER_MISSED_INTERCEPT:
        case ExecutorState::RECOVERY_TURN:
            fail("route_state_critical", input.nowMs);
            break;
        case ExecutorState::HEADING_STABLE:
            updateHeadingStable(input);
            break;
        case ExecutorState::RECOVERY_PLAN:
            updateRecoveryPlan(input);
            break;
        case ExecutorState::RECOVERY_REVERSE:
            if (!input.reverseEnabled || !input.reversePathAllowed) {
                enterBrake(StopNext::RECOVERY_TURN, input);
            } else if (distance(input.position, recoveryMotionStart_) >=
                           config_.recoveryReverseMaxM ||
                       elapsedMillis(input.nowMs, recoveryMotionStartedMs_) >=
                           config_.recoveryApproachTimeoutMs) {
                enterBrake(StopNext::RECOVERY_TURN, input);
            } else {
                output_.motion = MotionKind::REVERSE;
                output_.linearMps = -config_.recoveryReverseSpeedMps;
                output_.angularRadps = 0.0f;
            }
            break;
        case ExecutorState::RECOVERY_APPROACH:
            updateRecoveryApproach(input);
            break;
        case ExecutorState::RECOVERY_EVALUATE:
            updateRecoveryEvaluate(input);
            break;
        case ExecutorState::IDLE:
        case ExecutorState::COMPLETE:
        case ExecutorState::FAULT:
            break;
    }

    // State-only transitions must not inject a one-loop brake pulse. In
    // particular PASS_THROUGH handoff and FOLLOW<->TERMINAL/INTERCEPT remain
    // continuous. Geometry is refreshed because segmentIndex_ may have
    // changed inside the switch.
    if (output_.motion == MotionKind::STOP) {
        const bool linePathAllowed =
            state_ == ExecutorState::RECOVERY_APPROACH
                ? input.recoveryPathAllowed
                : (segmentIndex_ == segmentAtUpdateStart
                       ? input.forwardPathAllowed
                       : input.nextSegmentPathAllowed);
        if (stateNeedsForwardPath(state_) && !linePathAllowed) {
            fail("route_segment_blocked", input.nowMs);
        }
        switch (state_) {
            case ExecutorState::FOLLOW_SEGMENT:
            case ExecutorState::APPROACH_TRANSITION:
                refreshGeometry(input);
                updateLineMotion(input, false, false, false);
                break;
            case ExecutorState::INTERCEPT_NEXT_LINE:
                refreshGeometry(input);
                updateLineMotion(
                    input, true,
                    isTerminalSegment() &&
                        projection_.distanceToEndM <=
                            config_.terminalApproachDistanceM,
                    false);
                break;
            case ExecutorState::TERMINAL_APPROACH:
                refreshGeometry(input);
                updateLineMotion(input, false, true, false);
                break;
            case ExecutorState::RECOVERY_APPROACH:
                refreshGeometry(input);
                updateLineMotion(input, false, false, true);
                break;
            default:
                break;
        }
    }

    populateOutput(input);
    return output_;
}

}  // namespace routeexec
