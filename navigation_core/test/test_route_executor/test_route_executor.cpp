#include <gtest/gtest.h>

#include "CoveragePlanner.h"
#include "FootprintGeometry.h"
#include "MotorCommandMath.h"
#include "RouteExecutor.h"
#include "RouteGeometry.h"
#include "RouteTypes.h"

#include <cmath>
#include <cstdint>
#include <limits>

namespace {

using namespace routeexec;

RoutePoint point(float x, float y, WaypointType type,
                 float tolerance = 0.10f,
                 float finalHeadingDeg = 0.0f,
                 float headingToleranceDeg = 7.0f) {
    RoutePoint result;
    result.position = LocalPoint(x, y);
    result.type = type;
    result.positionToleranceM = tolerance;
    result.finalHeadingRequired = type == WaypointType::FINAL_POSE;
    result.finalHeadingDeg = finalHeadingDeg;
    result.headingToleranceDeg = headingToleranceDeg;
    return result;
}

RoutePlan straightPlan(WaypointType terminalType = WaypointType::FINAL_POSITION,
                       float finalHeadingDeg = 90.0f) {
    RoutePlan plan;
    plan.setRouteId(17u);
    EXPECT_TRUE(plan.appendPoint(point(0.0f, 0.0f,
                                       WaypointType::PASS_THROUGH)));
    EXPECT_TRUE(plan.appendPoint(point(1.0f, 0.0f, terminalType, 0.09f,
                                       finalHeadingDeg),
                                 SegmentType::LINE, 0.15f, 0.11f));
    EXPECT_TRUE(plan.finalize());
    return plan;
}

RoutePlan passThroughPlan() {
    RoutePlan plan;
    plan.setRouteId(23u);
    EXPECT_TRUE(plan.appendPoint(point(0.0f, 0.0f,
                                       WaypointType::PASS_THROUGH)));
    EXPECT_TRUE(plan.appendPoint(point(1.0f, 0.0f,
                                       WaypointType::PASS_THROUGH),
                                 SegmentType::LINE, 0.15f, 0.11f));
    EXPECT_TRUE(plan.appendPoint(point(2.0f, 0.0f,
                                       WaypointType::FINAL_POSITION, 0.09f),
                                 SegmentType::LINE, 0.15f, 0.11f));
    EXPECT_TRUE(plan.finalize());
    return plan;
}

RoutePlan cornerPlan() {
    RoutePlan plan;
    plan.setRouteId(31u);
    EXPECT_TRUE(plan.appendPoint(point(0.0f, 0.0f,
                                       WaypointType::PASS_THROUGH)));
    EXPECT_TRUE(plan.appendPoint(point(1.0f, 0.0f,
                                       WaypointType::CORNER, 0.11f),
                                 SegmentType::LINE, 0.15f, 0.11f));
    EXPECT_TRUE(plan.appendPoint(point(1.0f, 1.0f,
                                       WaypointType::FINAL_POSITION, 0.09f),
                                 SegmentType::LINE, 0.15f, 0.11f));
    EXPECT_TRUE(plan.finalize());
    return plan;
}

RoutePlan finalPosePlanWithPreApproach() {
    RoutePlan plan;
    plan.setRouteId(41u);
    EXPECT_TRUE(plan.appendPoint(point(0.0f, 0.0f,
                                       WaypointType::PASS_THROUGH)));
    EXPECT_TRUE(plan.appendPoint(point(0.60f, 0.0f,
                                       WaypointType::CORNER, 0.10f),
                                 SegmentType::TERMINAL_APPROACH,
                                 0.12f, 0.055f));
    EXPECT_TRUE(plan.appendPoint(point(1.0f, 0.0f,
                                       WaypointType::FINAL_POSE,
                                       0.09f, 90.0f, 7.0f),
                                 SegmentType::TERMINAL_APPROACH,
                                 0.12f, 0.055f));
    EXPECT_TRUE(plan.finalize());
    return plan;
}

RouteExecutorConfig fastConfig() {
    RouteExecutorConfig config;
    config.physicalStopStableMs = 2u;
    config.physicalStopTimeoutMs = 100u;
    config.interceptTimeoutMs = 1000u;
    config.recoveryApproachTimeoutMs = 100u;
    config.recoveryReverseMaxM = 0.10f;
    config.recoveryApproachMaxM = 0.50f;
    config.recoveryMaxAttempts = 3u;
    config.turnBreakawayResponseStableMs = 1u;
    config.turnBreakawayBoostAfterMs = 5u;
    config.turnBreakawayMaxMs = 20u;
    config.turnTimeoutMs = 1000u;
    return config;
}

RouteExecutorInput goodInput(float x = 0.0f, float y = 0.0f,
                             float headingDeg = 90.0f,
                             uint32_t nowMs = 100u) {
    RouteExecutorInput input;
    input.nowMs = nowMs;
    input.pvtId = 1u;
    input.position = LocalPoint(x, y);
    input.headingDeg = headingDeg;
    input.yawRateDps = 0.0f;
    input.estimatedSpeedMps = 0.05f;
    input.commandLeft = 0;
    input.commandRight = 0;
    input.measuredLeft = 0;
    input.measuredRight = 0;
    input.feedbackAgeMs = 0u;
    input.imuAgeMs = 0u;
    input.pvtAgeMs = 0u;
    input.motionAllowed = true;
    input.currentFootprintAllowed = true;
    input.forwardPathAllowed = true;
    input.nextSegmentPathAllowed = true;
    input.recoveryPathAllowed = true;
    input.reversePathAllowed = true;
    input.turnPathAllowed = true;
    input.reverseEnabled = false;
    return input;
}

RouteExecutorOutput tick(RouteExecutor& executor, RouteExecutorInput& input,
                         uint32_t advanceMs = 1u) {
    input.nowMs += advanceMs;
    ++input.pvtId;
    return executor.update(input);
}

void startFollowing(RouteExecutor& executor, const RoutePlan& plan,
                    const RouteExecutorConfig& config,
                    RouteExecutorInput& input) {
    ASSERT_TRUE(executor.start(plan, config, input));
    EXPECT_EQ(tick(executor, input).state, ExecutorState::ACQUIRE_SEGMENT);
    EXPECT_EQ(tick(executor, input).state, ExecutorState::FOLLOW_SEGMENT);
}

RouteExecutorOutput finishPhysicalStop(RouteExecutor& executor,
                                       RouteExecutorInput& input,
                                       const RouteExecutorConfig& config) {
    RouteExecutorOutput output = executor.lastOutput();
    if (executor.state() == ExecutorState::BRAKE ||
        executor.state() == ExecutorState::FINAL_STOP ||
        executor.state() == ExecutorState::TURN_BRAKE) {
        output = tick(executor, input);
    }
    EXPECT_EQ(executor.state(), ExecutorState::WAIT_PHYSICAL_STOP);
    output = tick(executor, input);
    EXPECT_EQ(executor.state(), ExecutorState::WAIT_PHYSICAL_STOP);
    output = tick(executor, input, config.physicalStopStableMs + 1u);
    return output;
}

RouteExecutorOutput brakeTurnAtTarget(RouteExecutor& executor,
                                      RouteExecutorInput& input,
                                      const RouteExecutorConfig& config,
                                      float targetHeadingDeg) {
    RouteExecutorOutput output = executor.lastOutput();
    EXPECT_TRUE(executor.state() == ExecutorState::TURN_BREAKAWAY ||
                executor.state() ==
                    ExecutorState::TURN_CORRECTION_BREAKAWAY);
    if (executor.state() != ExecutorState::TURN_BREAKAWAY &&
        executor.state() != ExecutorState::TURN_CORRECTION_BREAKAWAY) {
        return output;
    }
    input.yawRateDps = output.turnDirection * 5.0f;
    output = tick(executor, input);
    input.headingDeg = targetHeadingDeg;
    output = tick(executor, input,
                  config.turnBreakawayResponseStableMs + 1u);
    EXPECT_EQ(output.state, ExecutorState::TURN_BRAKE);
    input.yawRateDps = 0.0f;
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.state, ExecutorState::TURN_EVALUATE);
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::HEADING_STABLE);
    return output;
}

void advanceCornerToIntercept(RouteExecutor& executor,
                              RouteExecutorInput& input,
                              const RouteExecutorConfig& config,
                              float nextHeadingDeg) {
    ASSERT_EQ(executor.state(), ExecutorState::BRAKE);
    RouteExecutorOutput output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    output = brakeTurnAtTarget(executor, input, config, nextHeadingDeg);
    ASSERT_EQ(output.state, ExecutorState::HEADING_STABLE);
    output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::HEADING_STABLE);
    output = tick(executor, input, config.physicalStopStableMs + 1u);
    ASSERT_EQ(output.state, ExecutorState::INTERCEPT_NEXT_LINE);
}

TEST(RoutePlan, PlannedGeometryIsImmutableAfterActualDisplacement) {
    const RoutePlan plan = cornerPlan();
    const RouteSegment originalSecond = plan.segment(1u);
    EXPECT_FALSE(const_cast<RoutePlan&>(plan).appendPoint(
        point(9.0f, 9.0f, WaypointType::FINAL_POSITION)));

    RouteExecutor executor;
    RouteExecutorInput input = goodInput(-0.20f, 0.30f, 90.0f);
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.08f, 0.14f);
    tick(executor, input);

    const RouteSegment& stored = executor.plan().segment(1u);
    EXPECT_FLOAT_EQ(stored.plannedStart.x, originalSecond.plannedStart.x);
    EXPECT_FLOAT_EQ(stored.plannedStart.y, originalSecond.plannedStart.y);
    EXPECT_FLOAT_EQ(stored.plannedEnd.x, originalSecond.plannedEnd.x);
    EXPECT_FLOAT_EQ(stored.plannedEnd.y, originalSecond.plannedEnd.y);
    EXPECT_FLOAT_EQ(stored.plannedHeadingDeg,
                    originalSecond.plannedHeadingDeg);
}

TEST(RouteTransition, PlaneCrossingInsideCorridorAdvancesWithoutOldPointChase) {
    const RoutePlan plan = passThroughPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);

    input.position = LocalPoint(1.02f, 0.03f);
    const RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.segmentIndex, 1u);
    EXPECT_EQ(output.state, ExecutorState::FOLLOW_SEGMENT);
    EXPECT_FLOAT_EQ(output.plannedStart.x, 1.0f);
    EXPECT_FLOAT_EQ(output.plannedStart.y, 0.0f);
    EXPECT_NE(output.motion, MotionKind::REVERSE);
    EXPECT_EQ(output.motion, MotionKind::DRIVE);
}

TEST(RouteTransition, PassThroughRequiresSafeNextSegmentBeforeContinuousHandoff) {
    const RoutePlan plan = passThroughPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.02f, 0.01f);
    input.nextSegmentPathAllowed = false;
    const RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    EXPECT_STREQ(output.faultReason, "route_segment_blocked");
}

TEST(RouteTransition, WorkActionEmitsOneTransitionEventAndKeepsDriving) {
    RoutePlan plan;
    plan.setRouteId(24u);
    ASSERT_TRUE(plan.appendPoint(point(0.0f, 0.0f,
                                       WaypointType::PASS_THROUGH)));
    ASSERT_TRUE(plan.appendPoint(point(1.0f, 0.0f,
                                       WaypointType::WORK_ACTION),
                                 SegmentType::LINE, 0.15f, 0.11f));
    ASSERT_TRUE(plan.appendPoint(point(2.0f, 0.0f,
                                       WaypointType::FINAL_POSITION, 0.09f),
                                 SegmentType::LINE, 0.15f, 0.11f));
    ASSERT_TRUE(plan.finalize());
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.02f, 0.0f);
    const RouteExecutorOutput output = tick(executor, input);
    EXPECT_TRUE(output.workActionPending);
    EXPECT_EQ(output.workActionPointIndex, 1u);
    EXPECT_EQ(output.segmentIndex, 1u);
    EXPECT_EQ(output.motion, MotionKind::DRIVE);
}

TEST(RouteTransition, MissedIntermediatePointInterceptsNextFixedLine) {
    const RoutePlan plan = passThroughPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);

    input.position = LocalPoint(1.05f, 0.20f);
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::BRAKE);
    ASSERT_EQ(output.segmentIndex, 1u);
    output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::INTERCEPT_NEXT_LINE);
    output = tick(executor, input);
    EXPECT_FLOAT_EQ(output.plannedStart.x, 1.0f);
    EXPECT_FLOAT_EQ(output.plannedStart.y, 0.0f);
    EXPECT_FLOAT_EQ(output.plannedEnd.x, 2.0f);
    EXPECT_GT(output.interceptTarget.x, 1.05f);
    EXPECT_NEAR(output.interceptTarget.y, 0.0f, 1e-6f);
    EXPECT_GT(distance(output.interceptTarget, plan.point(1u).position), 0.05f);
}

TEST(RouteTransition, CornerStopsTurnsThenInterceptsNextPlannedSegment) {
    const RoutePlan plan = cornerPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.0f, 0.02f);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    advanceCornerToIntercept(executor, input, config, 0.0f);

    const RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.segmentIndex, 1u);
    EXPECT_FLOAT_EQ(output.plannedStart.x, 1.0f);
    EXPECT_FLOAT_EQ(output.plannedStart.y, 0.0f);
    EXPECT_NEAR(output.plannedHeadingDeg, 0.0f, 1e-5f);
}

TEST(RouteTransition, MissedCornerStopsBeforeBoundedNextLineIntercept) {
    const RoutePlan plan = cornerPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);

    // The first segment is eastbound: y is its signed cross-track. This pose
    // has crossed the endpoint plane but is well outside the 0.15 m corridor.
    input.position = LocalPoint(1.02f, 0.27f);
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::BRAKE);
    EXPECT_EQ(output.segmentIndex, 0u);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    EXPECT_EQ(output.transitionPurpose,
              RouteTransitionPurpose::CORNER_MISSED_INTERCEPT);

    // Only after a physical stop may the executor turn toward/intercept the
    // next immutable line. It never treats the missed corner as reached.
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    EXPECT_EQ(output.segmentIndex, 1u);
    EXPECT_EQ(output.transitionPurpose,
              RouteTransitionPurpose::CORNER_MISSED_INTERCEPT);
    EXPECT_FLOAT_EQ(output.plannedStart.x, 1.0f);
    EXPECT_FLOAT_EQ(output.plannedStart.y, 0.0f);
}

TEST(RouteTransition, MissedCornerFaultsAfterStopWhenNextLineIsUnsafe) {
    const RoutePlan plan = cornerPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.02f, 0.27f);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    input.nextSegmentPathAllowed = false;
    const RouteExecutorOutput output =
        finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_STREQ(output.faultReason, "transition_missed");
}

void enterAutomaticEndpointRecovery(RouteExecutor& executor,
                                    RouteExecutorInput& input,
                                    const RouteExecutorConfig& config) {
    const RoutePlan plan = straightPlan();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.20f, 0.15f);
    EXPECT_EQ(tick(executor, input).state, ExecutorState::TERMINAL_APPROACH);
    EXPECT_EQ(tick(executor, input).state, ExecutorState::RECOVERY_PLAN);
    EXPECT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
}

TEST(RouteRecovery, TurnTargetIsLatchedForWholeAttempt) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    enterAutomaticEndpointRecovery(executor, input, config);
    RouteExecutorOutput output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    ASSERT_TRUE(std::isfinite(output.latchedTurnTargetDeg));
    const float latched = output.latchedTurnTargetDeg;

    input.position = LocalPoint(1.25f, 0.28f);
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    EXPECT_FLOAT_EQ(output.latchedTurnTargetDeg, latched);
}

TEST(RouteRecovery, AttemptCounterChangesOnlyAfterStopTurnStopApproachEvaluate) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    enterAutomaticEndpointRecovery(executor, input, config);
    RouteExecutorOutput output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    ASSERT_EQ(output.recoveryAttempt, 0u);

    output = brakeTurnAtTarget(executor, input, config,
                               output.latchedTurnTargetDeg);
    EXPECT_EQ(output.recoveryAttempt, 0u);
    ASSERT_EQ(output.state, ExecutorState::HEADING_STABLE);
    output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::HEADING_STABLE);
    output = tick(executor, input, config.physicalStopStableMs + 1u);
    ASSERT_EQ(output.state, ExecutorState::RECOVERY_APPROACH);
    EXPECT_EQ(output.recoveryAttempt, 0u);

    input.position = LocalPoint(0.60f, 0.0f);
    output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::BRAKE);
    EXPECT_EQ(output.recoveryAttempt, 0u);
    output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::RECOVERY_EVALUATE);
    EXPECT_EQ(output.recoveryAttempt, 0u);
    output = tick(executor, input);
    EXPECT_EQ(output.recoveryAttempt, 1u);
}

TEST(RouteTurn, LargeHeadingErrorBrakesBeforeTurnAndTimesOutBoundedly) {
    const RoutePlan plan = straightPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    RouteExecutorConfig config = fastConfig();
    config.turnTimeoutMs = 5u;
    ASSERT_TRUE(executor.start(plan, config, input));
    EXPECT_EQ(tick(executor, input).state, ExecutorState::ACQUIRE_SEGMENT);
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::BRAKE);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    output = tick(executor, input, 6u);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_STREQ(output.faultReason, "turn_not_converging");
}

TEST(RouteTurn, BlockedSweptFootprintFaultsBeforeTurnCommand) {
    const RoutePlan plan = straightPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    ASSERT_TRUE(executor.start(plan, config, input));
    EXPECT_EQ(tick(executor, input).state, ExecutorState::ACQUIRE_SEGMENT);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    input.turnPathAllowed = false;
    output = tick(executor, input, config.physicalStopStableMs + 1u);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    EXPECT_STREQ(output.faultReason, "turn_footprint_blocked");
}

RouteExecutorOutput enterHeadingReacquireTurn(
        RouteExecutor& executor, RouteExecutorInput& input,
        const RouteExecutorConfig& config) {
    const RoutePlan plan = straightPlan();
    EXPECT_TRUE(executor.start(plan, config, input));
    tick(executor, input);
    RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::BRAKE);
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    return output;
}

RouteExecutorOutput advanceBreakawayToRotate(
        RouteExecutor& executor, RouteExecutorInput& input,
        const RouteExecutorConfig& config) {
    RouteExecutorOutput output = executor.lastOutput();
    input.yawRateDps = output.turnDirection * 5.0f;
    output = tick(executor, input);
    output = tick(executor, input,
                  config.turnBreakawayResponseStableMs + 1u);
    EXPECT_TRUE(output.state == ExecutorState::TURN_ROTATE ||
                output.state == ExecutorState::TURN_CORRECTION);
    return output;
}

RouteExecutorOutput stopMainTurnWithFieldOvershoot(
        RouteExecutor& executor, RouteExecutorInput& input,
        const RouteExecutorConfig& config) {
    RouteExecutorOutput output = advanceBreakawayToRotate(
        executor, input, config);
    input.headingDeg = 78.8f;
    input.yawRateDps = 37.0f;
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::TURN_BRAKE);
    input.headingDeg = 98.9f;
    input.yawRateDps = 0.0f;
    tick(executor, input);  // TURN_BRAKE -> WAIT_PHYSICAL_STOP.
    tick(executor, input);  // Coast changed heading; establish stop anchor.
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.state, ExecutorState::TURN_EVALUATE);
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::TURN_CORRECTION_BREAKAWAY);
    return output;
}

TEST(RouteTurnFieldControl, HighYawRateBrakesBeforeHeadingTolerance) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    enterHeadingReacquireTurn(executor, input, config);
    advanceBreakawayToRotate(executor, input, config);

    input.headingDeg = 78.8f;  // 11.2 deg remains to the 90 deg target.
    input.yawRateDps = 37.0f;
    const RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::TURN_BRAKE);
    EXPECT_GT(std::fabs(output.turnErrorDeg), config.turnToleranceDeg);
    EXPECT_GT(output.turnPredictedStopAngleDeg,
              std::fabs(output.turnErrorDeg));
    EXPECT_NEAR(output.turnPredictedStopAngleDeg,
                37.0f * 37.0f /
                        (2.0f * config.turnEstimatedAngularDecelDegps2) +
                    config.turnBrakeMarginDeg,
                1e-4f);
}

TEST(RouteTurnFieldControl, LowYawRateAndLargeErrorKeepsRotating) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    enterHeadingReacquireTurn(executor, input, config);
    advanceBreakawayToRotate(executor, input, config);
    input.headingDeg = 20.0f;
    input.yawRateDps = 1.0f;
    const RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::TURN_ROTATE);
    EXPECT_EQ(output.motion, MotionKind::TURN_IN_PLACE);
    EXPECT_EQ(output.turnCommandPercent, config.turnFarCommandPercent);
}

TEST(RouteTurnFieldControl, BrakeCannotCompleteBeforePhysicalStop) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    enterHeadingReacquireTurn(executor, input, config);
    advanceBreakawayToRotate(executor, input, config);
    input.headingDeg = 86.0f;
    input.yawRateDps = 20.0f;
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::TURN_BRAKE);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    EXPECT_NE(output.state, ExecutorState::HEADING_STABLE);
}

TEST(RouteTurnFieldControl, OvershootStartsBoundedReverseBreakawayAfterStop) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    enterHeadingReacquireTurn(executor, input, config);
    const RouteExecutorOutput output = stopMainTurnWithFieldOvershoot(
        executor, input, config);
    EXPECT_EQ(output.turnDirection, -1);
    EXPECT_TRUE(output.turnBreakawayActive);
    EXPECT_EQ(output.turnCorrectionAttempt, 0u);
    EXPECT_EQ(output.motion, MotionKind::TURN_IN_PLACE);
    EXPECT_LT(output.angularRadps, 0.0f);
    EXPECT_EQ(output.turnCommandPercent, config.turnBreakawayPercent);
}

TEST(RouteTurnFieldControl, FivePercentWithoutYawResponseIsNotSuccess) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    enterHeadingReacquireTurn(executor, input, config);
    input.commandLeft = -5;
    input.commandRight = 5;
    input.yawRateDps = 0.0f;
    const RouteExecutorOutput output = tick(executor, input, 4u);
    EXPECT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    EXPECT_EQ(output.turnCommandPercent, config.turnBreakawayPercent);
    EXPECT_FALSE(output.physicalStopReady);
}

TEST(RouteTurnFieldControl, SixPercentBreakawayEndsOnSustainedYawResponse) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    RouteExecutorOutput output = enterHeadingReacquireTurn(
        executor, input, config);
    EXPECT_EQ(output.turnCommandPercent, config.turnBreakawayPercent);
    input.yawRateDps = output.turnDirection *
                       config.turnBreakawayYawRateThresholdDps;
    tick(executor, input);
    output = tick(executor, input,
                  config.turnBreakawayResponseStableMs + 1u);
    EXPECT_EQ(output.state, ExecutorState::TURN_ROTATE);
    EXPECT_FALSE(output.turnBreakawayActive);
}

TEST(RouteTurnFieldControl, MissingBreakawayResponseHasDedicatedFault) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    RouteExecutorConfig config = fastConfig();
    config.turnBreakawayMaxMs = 10u;
    enterHeadingReacquireTurn(executor, input, config);
    input.yawRateDps = 0.0f;
    const RouteExecutorOutput output = tick(executor, input, 11u);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_STREQ(output.faultReason, "turn_motor_no_response");
}

TEST(RouteTurnFieldControl, DirectionCannotReverseBeforePhysicalStop) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    enterHeadingReacquireTurn(executor, input, config);
    advanceBreakawayToRotate(executor, input, config);
    input.headingDeg = 98.9f;
    input.yawRateDps = 20.0f;
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::TURN_BRAKE);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    EXPECT_EQ(output.turnDirection, 1);
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    EXPECT_EQ(output.turnDirection, 1);
}

TEST(RouteTurnFieldControl, CorrectionCountChangesOnlyAfterEvaluate) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    const RouteExecutorConfig config = fastConfig();
    enterHeadingReacquireTurn(executor, input, config);
    RouteExecutorOutput output = stopMainTurnWithFieldOvershoot(
        executor, input, config);
    EXPECT_EQ(output.turnCorrectionAttempt, 0u);
    advanceBreakawayToRotate(executor, input, config);
    input.headingDeg = 91.0f;
    input.yawRateDps = -10.0f;
    output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::TURN_BRAKE);
    input.yawRateDps = 0.0f;
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.turnCorrectionAttempt, 0u);
    output = tick(executor, input);
    EXPECT_EQ(output.turnCorrectionAttempt, 1u);
    EXPECT_EQ(output.state, ExecutorState::HEADING_STABLE);
}

TEST(RouteTurnFieldControl, CorrectionsAreBoundedAndKeepFailureSnapshots) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 0.0f);
    RouteExecutorConfig config = fastConfig();
    config.turnMaxCorrectionAttempts = 1u;
    enterHeadingReacquireTurn(executor, input, config);
    RouteExecutorOutput output = stopMainTurnWithFieldOvershoot(
        executor, input, config);
    advanceBreakawayToRotate(executor, input, config);
    input.headingDeg = 97.5f;
    input.yawRateDps = -30.0f;
    output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::TURN_BRAKE);
    input.yawRateDps = 0.0f;
    output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_EVALUATE);
    EXPECT_NEAR(output.turnLastPhysicalStopHeadingDeg, 97.5f, 1e-5f);
    input.headingDeg = 96.0f;  // EVALUATE must use the latched stop snapshot.
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_STREQ(output.faultReason, "turn_not_converging");
    EXPECT_EQ(output.turnCorrectionAttempt, 1u);
    EXPECT_TRUE(std::isfinite(output.turnStartHeadingDeg));
    EXPECT_TRUE(std::isfinite(output.turnFirstBrakeHeadingDeg));
    EXPECT_TRUE(std::isfinite(output.turnFirstPhysicalStopHeadingDeg));
    EXPECT_TRUE(std::isfinite(output.turnLastPhysicalStopHeadingDeg));
    EXPECT_TRUE(std::isfinite(output.turnFinalErrorDeg));
    EXPECT_NEAR(output.turnLastPhysicalStopHeadingDeg, 97.5f, 1e-5f);
    EXPECT_NEAR(output.turnFinalErrorDeg, -7.5f, 1e-5f);
}

TEST(RouteTurnFieldControl, TurnErrorIsNotOldLineHeadingError) {
    const RoutePlan plan = cornerPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.0f, 0.0f);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    const RouteExecutorOutput output = finishPhysicalStop(
        executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    EXPECT_NEAR(output.turnTargetDeg, 0.0f, 1e-5f);
    EXPECT_NEAR(output.turnErrorDeg, -90.0f, 1e-5f);
    EXPECT_NEAR(output.steeringErrorDeg, 0.0f, 1e-5f);
    EXPECT_NE(output.turnErrorDeg, output.steeringErrorDeg);
}

TEST(RouteSteeringWatchdog,
     RequiresConsecutiveFreshOppositeResponsesThenStopsBeforeFault) {
    const RoutePlan plan = straightPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.10f, 0.08f, 90.0f);
    RouteExecutorConfig config = fastConfig();
    config.steeringResponseMinAngularRadps = 0.05f;
    config.steeringResponseMinYawRateDps = 1.0f;
    config.steeringResponseMinHeadingDeltaDeg = 0.20f;
    config.steeringResponseMinSpeedMps = 0.02f;
    config.steeringResponseMinCommandPercent = 5;
    config.steeringResponseSettleMs = 0u;
    config.steeringResponseObservationMinMs = 1u;
    config.steeringResponseWrongDirectionCount = 3u;
    startFollowing(executor, plan, config, input);

    // Positive cross-track requests positive angular. Simulate fresh raw
    // board feedback plus normalized IMU/heading showing a sustained response
    // in the opposite physical direction.
    input.commandLeft = 5;
    input.commandRight = 8;
    input.measuredLeft = 5;
    input.measuredRight = 8;
    input.yawRateDps = -3.0f;

    input.headingDeg -= 0.50f;  // establishes settled command baseline
    RouteExecutorOutput output = tick(executor, input, 200u);
    ASSERT_EQ(output.state, ExecutorState::FOLLOW_SEGMENT);
    EXPECT_EQ(output.steeringResponseObservationCount, 0u);

    for (uint8_t expected = 1u; expected <= 2u; ++expected) {
        input.headingDeg -= 0.50f;
        output = tick(executor, input, 200u);
        ASSERT_EQ(output.state, ExecutorState::FOLLOW_SEGMENT);
        EXPECT_EQ(output.steeringResponseObservationCount, expected);
        EXPECT_TRUE(output.steeringResponseWrongDirection);
    }

    input.headingDeg -= 0.50f;
    output = tick(executor, input, 200u);
    ASSERT_EQ(output.state, ExecutorState::BRAKE);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    EXPECT_EQ(output.steeringResponseObservationCount, 3u);
    EXPECT_EQ(output.transitionPurpose,
              RouteTransitionPurpose::STEERING_RESPONSE_FAULT);

    // The dedicated fault is terminal only after a verified physical stop.
    input.commandLeft = input.commandRight = 0;
    input.measuredLeft = input.measuredRight = 0;
    input.yawRateDps = 0.0f;
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_STREQ(output.faultReason,
                 "steering_response_wrong_direction");
}

TEST(RouteSteeringWatchdog, OneOppositeSampleCannotFaultRoute) {
    const RoutePlan plan = straightPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.10f, 0.08f, 90.0f);
    RouteExecutorConfig config = fastConfig();
    config.steeringResponseMinAngularRadps = 0.05f;
    config.steeringResponseSettleMs = 0u;
    config.steeringResponseObservationMinMs = 1u;
    config.steeringResponseWrongDirectionCount = 3u;
    startFollowing(executor, plan, config, input);
    input.commandLeft = 5;
    input.commandRight = 8;
    input.measuredLeft = 5;
    input.measuredRight = 8;
    input.yawRateDps = -3.0f;
    input.headingDeg -= 0.50f;
    ASSERT_EQ(tick(executor, input, 200u).state,
              ExecutorState::FOLLOW_SEGMENT);
    input.headingDeg -= 0.50f;
    const RouteExecutorOutput output = tick(executor, input, 200u);
    EXPECT_EQ(output.state, ExecutorState::FOLLOW_SEGMENT);
    EXPECT_EQ(output.steeringResponseObservationCount, 1u);
    EXPECT_EQ(output.result, ExecutorResult::NONE);
}

TEST(RouteSteering, YawRateDampingOpposesClockwisePositiveRotation) {
    const RoutePlan plan = straightPlan();
    const RouteExecutorConfig config = fastConfig();

    RouteExecutor undampedSample;
    RouteExecutorInput zeroYaw = goodInput(0.10f, 0.05f, 90.0f);
    startFollowing(undampedSample, plan, config, zeroYaw);
    const RouteExecutorOutput zeroYawOutput =
        tick(undampedSample, zeroYaw);
    ASSERT_GT(zeroYawOutput.angularRadps, 0.0f);

    RouteExecutor clockwiseSample;
    RouteExecutorInput clockwiseYaw = goodInput(0.10f, 0.05f, 90.0f);
    clockwiseYaw.yawRateDps = 5.0f;
    startFollowing(clockwiseSample, plan, config, clockwiseYaw);
    const RouteExecutorOutput clockwiseOutput =
        tick(clockwiseSample, clockwiseYaw);

    EXPECT_GT(clockwiseOutput.angularRadps, 0.0f);
    EXPECT_LT(clockwiseOutput.angularRadps, zeroYawOutput.angularRadps);
    EXPECT_NEAR(zeroYawOutput.angularRadps - clockwiseOutput.angularRadps,
                config.yawRateDamping * 5.0f *
                    0.01745329251994329577f,
                1e-5f);
}

TEST(RouteRecovery, FinalPoseWrongHeadingDoesNotRestartRecoveryEveryTick) {
    const RoutePlan plan = finalPosePlanWithPreApproach();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(0.60f, 0.0f);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    advanceCornerToIntercept(executor, input, config, 90.0f);

    input.position = LocalPoint(0.95f, 0.0f);
    input.headingDeg = 180.0f;
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::BRAKE);
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    EXPECT_EQ(output.recoveryAttempt, 0u);
}

TEST(RouteRecovery, LargeApproachHeadingChangeStopsAndEvaluatesAttempt) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    enterAutomaticEndpointRecovery(executor, input, config);
    RouteExecutorOutput output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    output = brakeTurnAtTarget(executor, input, config,
                               output.latchedTurnTargetDeg);
    ASSERT_EQ(output.state, ExecutorState::HEADING_STABLE);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::HEADING_STABLE);
    output = tick(executor, input, config.physicalStopStableMs + 1u);
    ASSERT_EQ(output.state, ExecutorState::RECOVERY_APPROACH);

    // Moving the estimated pose to the other side of the pre-approach goal
    // makes the new bearing differ materially from the latched attempt turn.
    input.position = LocalPoint(1.20f, -0.50f);
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::BRAKE);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    EXPECT_EQ(output.recoveryAttempt, 0u);
}

TEST(RouteRecovery, AttemptLimitSurvivesReentryOnSamePlannedSegment) {
    // Use a non-terminal segment here. FINAL_POSITION recovery now correctly
    // targets the endpoint and completes immediately on radius entry.
    const RoutePlan plan = passThroughPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    RouteExecutorConfig config = fastConfig();
    config.recoveryMaxAttempts = 1u;
    startFollowing(executor, plan, config, input);

    executor.requestRecovery("test_recovery");
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    RouteExecutorOutput output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    output = brakeTurnAtTarget(executor, input, config,
                               output.latchedTurnTargetDeg);
    ASSERT_EQ(output.state, ExecutorState::HEADING_STABLE);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::HEADING_STABLE);
    output = tick(executor, input, config.physicalStopStableMs + 1u);
    ASSERT_EQ(output.state, ExecutorState::RECOVERY_APPROACH);

    input.position = output.recoveryGoal;
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::RECOVERY_EVALUATE);
    output = tick(executor, input);
    ASSERT_EQ(output.recoveryAttempt, 1u);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);

    output = brakeTurnAtTarget(executor, input, config,
                               output.latchedTurnTargetDeg);
    ASSERT_EQ(output.state, ExecutorState::HEADING_STABLE);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::HEADING_STABLE);
    output = tick(executor, input, config.physicalStopStableMs + 1u);
    ASSERT_EQ(output.state, ExecutorState::INTERCEPT_NEXT_LINE);

    executor.requestRecovery("test_reentry");
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_STREQ(output.faultReason, "transition_missed");
}

TEST(RouteRecovery, FinalRadiusWinsOverBlockedObsoleteRecoveryPath) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    enterAutomaticEndpointRecovery(executor, input, config);
    RouteExecutorOutput output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.state, ExecutorState::TURN_BREAKAWAY);
    output = brakeTurnAtTarget(executor, input, config,
                               output.latchedTurnTargetDeg);
    ASSERT_EQ(output.state, ExecutorState::HEADING_STABLE);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::HEADING_STABLE);
    output = tick(executor, input, config.physicalStopStableMs + 1u);
    ASSERT_EQ(output.state, ExecutorState::RECOVERY_APPROACH);

    input.position = LocalPoint(0.95f, 0.0f);
    input.recoveryPathAllowed = false;
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    EXPECT_EQ(output.result, ExecutorResult::NONE);
    EXPECT_EQ(output.faultReason, nullptr);
    EXPECT_TRUE(output.finalArrivalPending);
}

TEST(RouteLifecycle, FollowProgressWatchdogAndPauseUseActiveTime) {
    const RoutePlan plan = straightPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    RouteExecutorConfig config = fastConfig();
    config.progressTimeoutMs = 10u;
    startFollowing(executor, plan, config, input);

    executor.pause(input.nowMs);
    input.nowMs += 1000u;
    RouteExecutorOutput output = executor.update(input);
    EXPECT_EQ(output.state, ExecutorState::FOLLOW_SEGMENT);
    executor.resume(input.nowMs);
    output = tick(executor, input, 5u);
    EXPECT_EQ(output.state, ExecutorState::FOLLOW_SEGMENT);
    output = tick(executor, input, 6u);
    EXPECT_EQ(output.state, ExecutorState::FOLLOW_SEGMENT);
    output = tick(executor, input, 5u);
    EXPECT_EQ(output.state, ExecutorState::FAULT);
    EXPECT_STREQ(output.faultReason, "route_not_progressing");
}

TEST(RouteTerminal, FinalPositionCompletesWithoutHeadingTurn) {
    const RoutePlan plan = straightPlan(WaypointType::FINAL_POSITION);
    RouteExecutor executor;
    RouteExecutorInput input = goodInput(0.0f, 0.0f, 90.0f);
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(0.95f, 0.0f);
    input.headingDeg = 270.0f;
    RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    EXPECT_EQ(output.motion, MotionKind::STOP);
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.state, ExecutorState::COMPLETE);
    EXPECT_EQ(output.result, ExecutorResult::ARRIVED);
    EXPECT_FALSE(output.invariantViolation);
}

TEST(RouteTerminal, FarTerminalSegmentLeavesInterceptForNormalFollow) {
    const RoutePlan plan = cornerPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.0f, 0.0f);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    advanceCornerToIntercept(executor, input, config, 0.0f);

    // The final endpoint is still 1.0 m away. A successful fixed-line
    // intercept must resume normal FOLLOW, without terminal speed capping.
    RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::FOLLOW_SEGMENT);
    EXPECT_NEAR(output.distanceToWaypointM, 1.0f, 1e-5f);

    input.position = LocalPoint(1.0f, 0.60f);
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::TERMINAL_APPROACH);
    EXPECT_LT(output.distanceToWaypointM,
              config.terminalApproachDistanceM);
    EXPECT_LE(output.linearMps, config.terminalApproachSpeedMps);
}

TEST(RouteRecovery, FinalPositionUsesEndpointNotFinalPosePreApproach) {
    const RoutePlan plan = straightPlan(WaypointType::FINAL_POSITION);
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(1.20f, 0.15f);
    ASSERT_EQ(tick(executor, input).state,
              ExecutorState::TERMINAL_APPROACH);
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::RECOVERY_PLAN);
    EXPECT_FLOAT_EQ(output.recoveryGoal.x, plan.segment(0u).plannedEnd.x);
    EXPECT_FLOAT_EQ(output.recoveryGoal.y, plan.segment(0u).plannedEnd.y);
    EXPECT_NEAR(distance(output.recoveryGoal, plan.segment(0u).plannedEnd),
                0.0f, 1e-6f);
}

TEST(RouteRecovery, FinalPoseRetainsExplicitAlignedPreApproachGoal) {
    const RoutePlan plan = finalPosePlanWithPreApproach();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(0.60f, 0.0f);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    advanceCornerToIntercept(executor, input, config, 90.0f);
    ASSERT_EQ(tick(executor, input).state,
              ExecutorState::TERMINAL_APPROACH);

    input.position = LocalPoint(1.20f, 0.15f);
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::RECOVERY_PLAN);
    EXPECT_NEAR(output.recoveryGoal.x,
                plan.segment(1u).plannedStart.x, 1e-6f);
    EXPECT_NEAR(output.recoveryGoal.y,
                plan.segment(1u).plannedStart.y, 1e-6f);
    EXPECT_GT(distance(output.recoveryGoal,
                       plan.segment(1u).plannedEnd),
              0.30f);
}

TEST(RouteTerminal, FinalPoseUsesExplicitPreApproachBeforeFinalDrive) {
    const RoutePlan plan = finalPosePlanWithPreApproach();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);

    input.position = LocalPoint(0.60f, 0.0f);
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    advanceCornerToIntercept(executor, input, config, 90.0f);
    RouteExecutorOutput output = tick(executor, input);
    EXPECT_EQ(output.segmentIndex, 1u);
    EXPECT_EQ(output.state, ExecutorState::TERMINAL_APPROACH);
    EXPECT_NEAR(output.plannedStart.x, 0.60f, 1e-6f);
    EXPECT_NEAR(output.plannedHeadingDeg, 90.0f, 1e-5f);

    input.position = LocalPoint(0.95f, 0.0f);
    input.headingDeg = 90.0f;
    output = tick(executor, input);
    EXPECT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.result, ExecutorResult::ARRIVED);
}

struct AllowedBox {
    float minX;
    float maxX;
    float minY;
    float maxY;
};

bool pointInsideBox(const LocalPoint& p, void* context) {
    const AllowedBox& box = *static_cast<AllowedBox*>(context);
    return p.x >= box.minX && p.x <= box.maxX &&
           p.y >= box.minY && p.y <= box.maxY;
}

TEST(FootprintGeometry, SweptFootprintBlocksUnsafeReverse) {
    AllowedBox box = {-1.0f, 1.0f, -1.0f, 1.0f};
    FootprintConfig footprint;
    footprint.body = RobotFootprint(0.275f, 0.275f, 0.275f, 0.275f);
    footprint.toolConfigured = true;
    footprint.tool = RobotFootprint(0.60f, 0.10f, 0.30f, 0.30f);
    const Pose2D pose(LocalPoint(0.0f, 0.0f), 0.0f);

    EXPECT_EQ(checkReverseSweptFootprint(
                  pose, 0.50f, 0.05f, footprint, true,
                  pointInsideBox, &box),
              FootprintCheckResult::CLEAR);
    EXPECT_EQ(checkReverseSweptFootprint(
                  pose, 0.90f, 0.05f, footprint, true,
                  pointInsideBox, &box),
              FootprintCheckResult::BLOCKED);

    footprint.toolConfigured = false;
    EXPECT_EQ(checkReverseSweptFootprint(
                  pose, 0.10f, 0.05f, footprint, true,
                  pointInsideBox, &box),
              FootprintCheckResult::TOOL_FOOTPRINT_UNCONFIGURED);
}

TEST(RouteGeometry, AngleWrapUsesShortestCompassError) {
    EXPECT_NEAR(wrapHeadingErrorDeg(1.0f - 359.0f), 2.0f, 1e-6f);
    EXPECT_NEAR(wrapHeadingErrorDeg(359.0f - 1.0f), -2.0f, 1e-6f);
    EXPECT_NEAR(normalizeHeadingDeg(-1.0f), 359.0f, 1e-6f);
}

TEST(RouteGeometry, MillisAndItowRolloverRemainMonotonic) {
    const uint32_t nearWrap = std::numeric_limits<uint32_t>::max() - 4u;
    EXPECT_EQ(elapsedMillis(5u, nearWrap), 10u);
    EXPECT_EQ(elapsedItowMillis(100u, 604799900u), 200u);
    EXPECT_TRUE(isNewPvtId(0u, std::numeric_limits<uint32_t>::max()));
    EXPECT_FALSE(isNewPvtId(42u, 42u));
}

TEST(RouteLifecycle, StopClearsRouteAndAllowsCleanNewStart) {
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    const RoutePlan first = straightPlan();
    startFollowing(executor, first, config, input);
    input.headingDeg = 0.0f;
    ASSERT_EQ(tick(executor, input).state, ExecutorState::BRAKE);
    ASSERT_EQ(executor.lastOutput().transitionPurpose,
              RouteTransitionPurpose::STEERING_REACQUIRE);
    executor.stop();
    EXPECT_EQ(executor.state(), ExecutorState::IDLE);
    EXPECT_EQ(executor.result(), ExecutorResult::STOPPED);
    EXPECT_FALSE(executor.active());
    EXPECT_FALSE(executor.plan().valid());
    EXPECT_EQ(executor.lastOutput().transitionPurpose,
              RouteTransitionPurpose::NONE);
    EXPECT_EQ(executor.lastOutput().steeringResponseObservationCount, 0u);

    RoutePlan second = passThroughPlan();
    ASSERT_TRUE(executor.start(second, config, input));
    EXPECT_EQ(tick(executor, input).state, ExecutorState::ACQUIRE_SEGMENT);
    EXPECT_EQ(executor.result(), ExecutorResult::NONE);
}

TEST(RouteLifecycle, ArrivedCannotBecomeTimeoutOrFault) {
    const RoutePlan plan = straightPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(0.95f, 0.0f);
    tick(executor, input);
    RouteExecutorOutput output = finishPhysicalStop(executor, input, config);
    ASSERT_EQ(output.result, ExecutorResult::ARRIVED);
    ASSERT_EQ(output.state, ExecutorState::COMPLETE);
    executor.expire("timeout");
    output = tick(executor, input);
    EXPECT_EQ(output.result, ExecutorResult::ARRIVED);
    EXPECT_EQ(output.state, ExecutorState::COMPLETE);
    EXPECT_EQ(output.faultReason, nullptr);
    EXPECT_FALSE(output.invariantViolation);
    EXPECT_TRUE(executor.routeFinished());
    EXPECT_FALSE(executor.active());
}

TEST(RouteLifecycle, PendingFinalStopCannotBecomeTimeout) {
    const RoutePlan plan = straightPlan();
    RouteExecutor executor;
    RouteExecutorInput input = goodInput();
    const RouteExecutorConfig config = fastConfig();
    startFollowing(executor, plan, config, input);
    input.position = LocalPoint(0.95f, 0.0f);
    RouteExecutorOutput output = tick(executor, input);
    ASSERT_EQ(output.state, ExecutorState::WAIT_PHYSICAL_STOP);
    ASSERT_TRUE(output.finalArrivalPending);
    executor.expire("timeout");
    EXPECT_EQ(executor.state(), ExecutorState::WAIT_PHYSICAL_STOP);
    EXPECT_EQ(executor.result(), ExecutorResult::NONE);
    output = finishPhysicalStop(executor, input, config);
    EXPECT_EQ(output.result, ExecutorResult::ARRIVED);
    EXPECT_EQ(output.state, ExecutorState::COMPLETE);
}

TEST(CoveragePlanner, BuildsFixedParallelLanesWithSquareTransitions) {
    CoveragePlanConfig config;
    config.start = LocalPoint(0.0f, 0.0f);
    config.firstLaneHeadingDeg = 0.0f;
    config.laneLengthM = 2.0f;
    config.laneCount = 3u;
    config.laneSpacingM = 0.42f;
    config.shiftLeft = false;
    RoutePlan plan;
    ASSERT_EQ(buildRectangularSerpentine(config, 88u, plan),
              CoveragePlanResult::OK);
    ASSERT_EQ(plan.segmentCount(), 5u);
    EXPECT_EQ(plan.segment(0u).type, SegmentType::COVERAGE_LANE);
    EXPECT_EQ(plan.segment(1u).type, SegmentType::COVERAGE_SHIFT);
    EXPECT_EQ(plan.segment(2u).type, SegmentType::COVERAGE_LANE);
    EXPECT_EQ(plan.segment(3u).type, SegmentType::COVERAGE_SHIFT);
    EXPECT_EQ(plan.segment(4u).type, SegmentType::COVERAGE_LANE);
    EXPECT_NEAR(plan.segment(0u).plannedHeadingDeg, 0.0f, 1e-5f);
    EXPECT_NEAR(plan.segment(2u).plannedHeadingDeg, 180.0f, 1e-5f);
    EXPECT_NEAR(plan.segment(4u).plannedHeadingDeg, 0.0f, 1e-5f);
    EXPECT_NEAR(plan.segment(1u).segmentLengthM, 0.42f, 1e-5f);
}

TEST(CoveragePlanner, FinalPoseGetsExplicitAlignedPreApproach) {
    CoveragePlanConfig config;
    config.start = LocalPoint(0.0f, 0.0f);
    config.firstLaneHeadingDeg = 90.0f;
    config.laneLengthM = 2.0f;
    config.laneCount = 1u;
    config.finalType = WaypointType::FINAL_POSE;
    config.finalHeadingDeg = 90.0f;
    config.finalPreApproachM = 0.40f;
    RoutePlan plan;
    ASSERT_EQ(buildRectangularSerpentine(config, 89u, plan),
              CoveragePlanResult::OK);
    ASSERT_EQ(plan.segmentCount(), 2u);
    EXPECT_EQ(plan.point(1u).type, WaypointType::CORNER);
    EXPECT_EQ(plan.point(2u).type, WaypointType::FINAL_POSE);
    EXPECT_EQ(plan.segment(1u).type, SegmentType::TERMINAL_APPROACH);
    EXPECT_NEAR(plan.segment(1u).segmentLengthM, 0.40f, 1e-5f);
    EXPECT_NEAR(plan.segment(1u).plannedHeadingDeg, 90.0f, 1e-5f);
}

struct DeadbandFixture {
    int left;
    int right;
    int expectedLeft;
    int expectedRight;
    bool forwardMode;
};

TEST(MotorCommandMath, DeadbandMatrixPreservesIntentAndDifferential) {
    const DeadbandFixture cases[] = {
        { 1,  4,   5,  8, true},
        { 8,  2,  11,  5, true},
        {-1, -4,  -5, -8, true},
        {-8, -2, -11, -5, true},
        { 0,  4,   0,  5, true},
        { 4,  0,   5,  0, true},
        { 1, -1,   1, -1, true},
        { 0,  0,   0,  0, true},
        {69,  2,  70,  3, true},
    };
    for (size_t i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const DeadbandFixture& c = cases[i];
        const motorcmd::WheelPercent output =
            motorcmd::compensateForwardDeadband(
                c.left, c.right, 5, 5, 70, c.forwardMode);
        EXPECT_EQ(output.left, c.expectedLeft) << "case " << i;
        EXPECT_EQ(output.right, c.expectedRight) << "case " << i;
        if (c.left != 0 && c.right != 0 &&
            ((c.left > 0) == (c.right > 0))) {
            EXPECT_EQ(output.right - output.left, c.right - c.left)
                << "common-mode differential case " << i;
        }
        if (c.left == 0) {
            EXPECT_EQ(output.left, 0) << "case " << i;
        }
        if (c.right == 0) {
            EXPECT_EQ(output.right, 0) << "case " << i;
        }
    }
    const motorcmd::WheelPercent turn =
        motorcmd::compensateForwardDeadband(2, -2, 5, 5, 70, false);
    EXPECT_EQ(turn.left, 2);
    EXPECT_EQ(turn.right, -2);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
