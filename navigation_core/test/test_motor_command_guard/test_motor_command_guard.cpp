#include <gtest/gtest.h>

#include "MotorCommandGuard.h"

namespace {

using motorcmd::MotorCommandGuard;

TEST(MotorCommandGuard, HardZeroMakesEveryFollowingUartFrameZero) {
    MotorCommandGuard guard;
    const uint32_t token = guard.authorize();
    ASSERT_TRUE(guard.apply(token, -6, 6, -6, 6));
    guard.recordUartFrame(0, 51);
    ASSERT_NE(guard.state().uartSteer, 0);

    guard.hardZero();
    EXPECT_TRUE(guard.state().zeroLatchActive);
    for (int frame = 0; frame < 20; ++frame) {
        const int speed = guard.uartSpeedForFrame(37);
        const int steer = guard.uartSteerForFrame(-42);
        guard.recordUartFrame(speed, steer);
        EXPECT_EQ(speed, 0);
        EXPECT_EQ(steer, 0);
        EXPECT_EQ(guard.state().uartSpeed, 0);
        EXPECT_EQ(guard.state().uartSteer, 0);
    }
}

TEST(MotorCommandGuard, ConcurrentStaleCommandCannotOverwriteZeroLatch) {
    MotorCommandGuard guard;
    const uint32_t staleToken = guard.authorize();
    ASSERT_TRUE(guard.apply(staleToken, 6, 5, 6, 5));

    // Models STOP winning the race while a writer still holds its old token.
    guard.hardZero();
    EXPECT_FALSE(guard.apply(staleToken, 6, 5, 6, 5));
    EXPECT_TRUE(guard.state().zeroLatchActive);
    EXPECT_EQ(guard.state().motorAppliedLeft, 0);
    EXPECT_EQ(guard.state().motorAppliedRight, 0);

    // A genuinely new allowed command needs an explicit new epoch.
    const uint32_t freshToken = guard.authorize();
    EXPECT_NE(freshToken, staleToken);
    EXPECT_TRUE(guard.apply(freshToken, 5, 6, 5, 6));
}

TEST(MotorCommandGuard, RepeatedHardZeroIsIdempotentAndStaysLatched) {
    MotorCommandGuard guard;
    const uint32_t token = guard.authorize();
    ASSERT_TRUE(guard.apply(token, 5, 5, 5, 5));
    guard.hardZero();
    const uint32_t sequence = guard.state().motorCommandSequence;
    guard.hardZero();
    EXPECT_EQ(guard.state().motorCommandSequence, sequence);
    EXPECT_TRUE(guard.state().zeroLatchActive);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
