#include <gtest/gtest.h>

#include "MotorCommandMath.h"

namespace {

constexpr float kWheelBaseM = 0.38f;
constexpr float kPercentPerMps = 12.0f / 0.25f;
constexpr int kMaxPercent = 12;

motorcmd::WheelPercent mix(float linearMps, float angularRadps,
                           motorcmd::MotionMixMode mode) {
    return motorcmd::mixCompassLinearAngularPercent(
        linearMps, angularRadps, kWheelBaseM, kPercentPerMps,
        kMaxPercent, mode);
}

TEST(MotorMixSignContract, StraightForwardCommandsBothVirtualChannelsForward) {
    const motorcmd::WheelPercent wheels = mix(
        0.11f, 0.0f, motorcmd::MotionMixMode::DRIVE);
    EXPECT_GT(wheels.left, 0);
    EXPECT_EQ(wheels.left, wheels.right);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  wheels, motorcmd::MotionMixMode::DRIVE), 0);
}

TEST(MotorMixSignContract, PositiveDriveAngularIsClockwisePositive) {
    const motorcmd::WheelPercent wheels = mix(
        0.11f, 0.20f, motorcmd::MotionMixMode::DRIVE);
    EXPECT_GT(wheels.left, wheels.right);
    EXPECT_GT(wheels.right, 0);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  wheels, motorcmd::MotionMixMode::DRIVE), 1);

    const motorcmd::HoverAxes axes = motorcmd::hoverChannelsToAxes(
        wheels.left, wheels.right, 300, 70);
    EXPECT_GT(axes.speed, 0);
    EXPECT_LT(axes.steer, 0);
}

TEST(MotorMixSignContract, NegativeDriveAngularIsClockwiseNegative) {
    const motorcmd::WheelPercent wheels = mix(
        0.11f, -0.20f, motorcmd::MotionMixMode::DRIVE);
    EXPECT_LT(wheels.left, wheels.right);
    EXPECT_GT(wheels.left, 0);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  wheels, motorcmd::MotionMixMode::DRIVE), -1);

    const motorcmd::HoverAxes axes = motorcmd::hoverChannelsToAxes(
        wheels.left, wheels.right, 300, 70);
    EXPECT_GT(axes.speed, 0);
    EXPECT_GT(axes.steer, 0);
}

TEST(MotorMixSignContract, PositiveTurnUsesSameHeadingDirectionAsDrive) {
    const motorcmd::WheelPercent wheels = mix(
        0.0f, 0.20f, motorcmd::MotionMixMode::TURN_IN_PLACE);
    EXPECT_LT(wheels.left, 0);
    EXPECT_GT(wheels.right, 0);
    EXPECT_EQ(wheels.left, -wheels.right);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  wheels, motorcmd::MotionMixMode::TURN_IN_PLACE), 1);

    const motorcmd::HoverAxes axes = motorcmd::hoverChannelsToAxes(
        wheels.left, wheels.right, 300, 70);
    EXPECT_EQ(axes.speed, 0);
    EXPECT_GT(axes.steer, 0);
}

TEST(MotorMixSignContract, NegativeTurnUsesSameHeadingDirectionAsDrive) {
    const motorcmd::WheelPercent wheels = mix(
        0.0f, -0.20f, motorcmd::MotionMixMode::TURN_IN_PLACE);
    EXPECT_GT(wheels.left, 0);
    EXPECT_LT(wheels.right, 0);
    EXPECT_EQ(wheels.left, -wheels.right);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  wheels, motorcmd::MotionMixMode::TURN_IN_PLACE), -1);

    const motorcmd::HoverAxes axes = motorcmd::hoverChannelsToAxes(
        wheels.left, wheels.right, 300, 70);
    EXPECT_EQ(axes.speed, 0);
    EXPECT_LT(axes.steer, 0);
}

TEST(MotorMixSignContract, ZeroLinearAndAngularRemainStopped) {
    const motorcmd::WheelPercent channels = mix(
        0.0f, 0.0f, motorcmd::MotionMixMode::TURN_IN_PLACE);
    EXPECT_EQ(channels.left, 0);
    EXPECT_EQ(channels.right, 0);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  channels, motorcmd::MotionMixMode::TURN_IN_PLACE), 0);
    const motorcmd::HoverAxes axes = motorcmd::hoverChannelsToAxes(
        channels.left, channels.right, 300, 70);
    EXPECT_EQ(axes.speed, 0);
    EXPECT_EQ(axes.steer, 0);
}

TEST(MotorMixSignContract, TranslationalMixKeepsBothTracksMoving) {
    // This angular magnitude previously rounded the inside wheel to zero,
    // turning normal FOLLOW into prolonged one-track motion.
    const motorcmd::WheelPercent positive = mix(
        0.06f, 0.30f, motorcmd::MotionMixMode::DRIVE);
    EXPECT_GT(positive.left, 0);
    EXPECT_GT(positive.right, 0);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  positive, motorcmd::MotionMixMode::DRIVE), 1);

    const motorcmd::WheelPercent negative = mix(
        0.06f, -0.30f, motorcmd::MotionMixMode::DRIVE);
    EXPECT_GT(negative.left, 0);
    EXPECT_GT(negative.right, 0);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  negative, motorcmd::MotionMixMode::DRIVE), -1);
}

TEST(MotorMixSignContract, EffectiveFollowCommandsKeepBothTracksAboveDeadband) {
    const motorcmd::CommandMix positive =
        motorcmd::mixEffectiveCompassCommandPercent(
            0.06f, 0.30f, kWheelBaseM, kPercentPerMps,
            5, 5, kMaxPercent, motorcmd::MotionMixMode::DRIVE);
    EXPECT_GE(positive.effective.left, 5);
    EXPECT_GE(positive.effective.right, 5);
    EXPECT_LE(positive.effective.left, kMaxPercent);
    EXPECT_LE(positive.effective.right, kMaxPercent);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  positive.effective, positive.mode), 1);

    const motorcmd::CommandMix negative =
        motorcmd::mixEffectiveCompassCommandPercent(
            0.06f, -0.30f, kWheelBaseM, kPercentPerMps,
            5, 5, kMaxPercent, motorcmd::MotionMixMode::DRIVE);
    EXPECT_GE(negative.effective.left, 5);
    EXPECT_GE(negative.effective.right, 5);
    EXPECT_LE(negative.effective.left, kMaxPercent);
    EXPECT_LE(negative.effective.right, kMaxPercent);
    EXPECT_EQ(motorcmd::expectedClockwiseHeadingChange(
                  negative.effective, negative.mode), -1);
}

TEST(MotorTurnDeadband, SymmetricMinimumForBothDirections) {
    const motorcmd::WheelPercent clockwise =
        motorcmd::compensateTurnInPlaceDeadband(-3, 3, 5, 5, 12);
    EXPECT_EQ(clockwise.left, -5);
    EXPECT_EQ(clockwise.right, 5);

    const motorcmd::WheelPercent counterClockwise =
        motorcmd::compensateTurnInPlaceDeadband(3, -3, 5, 5, 12);
    EXPECT_EQ(counterClockwise.left, 5);
    EXPECT_EQ(counterClockwise.right, -5);

    const motorcmd::WheelPercent stopped =
        motorcmd::compensateTurnInPlaceDeadband(0, 0, 5, 5, 12);
    EXPECT_EQ(stopped.left, 0);
    EXPECT_EQ(stopped.right, 0);
}

TEST(MotorTurnDeadband, ForwardCompensationIsNotAppliedToTurn) {
    const motorcmd::WheelPercent requested{3, -3};
    const motorcmd::WheelPercent forward =
        motorcmd::compensateForwardDeadband(
            requested.left, requested.right, 5, 5, 12, true);
    EXPECT_EQ(forward.left, requested.left);
    EXPECT_EQ(forward.right, requested.right);

    const motorcmd::WheelPercent turn =
        motorcmd::compensateTurnInPlaceDeadband(
            requested.left, requested.right, 5, 5, 12);
    EXPECT_EQ(turn.left, 5);
    EXPECT_EQ(turn.right, -5);
}

TEST(MotorForwardDeadband, IntentionalZeroAndDifferentialRemainIntact) {
    const motorcmd::WheelPercent zeroLeft =
        motorcmd::compensateForwardDeadband(0, 4, 5, 5, 12, true);
    EXPECT_EQ(zeroLeft.left, 0);
    EXPECT_EQ(zeroLeft.right, 5);

    const motorcmd::WheelPercent zeroRight =
        motorcmd::compensateForwardDeadband(4, 0, 5, 5, 12, true);
    EXPECT_EQ(zeroRight.left, 5);
    EXPECT_EQ(zeroRight.right, 0);

    const motorcmd::WheelPercent curved =
        motorcmd::compensateForwardDeadband(1, 4, 5, 5, 12, true);
    EXPECT_EQ(curved.left, 5);
    EXPECT_EQ(curved.right, 8);
    EXPECT_EQ(curved.right - curved.left, 3);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
