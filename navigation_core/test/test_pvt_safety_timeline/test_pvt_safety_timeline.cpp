#include <gtest/gtest.h>

#include "PvtSafetyTimeline.h"

namespace {

TEST(PvtSafetyTimeline,
     FreshPvtThisLoopReplacesRetainedStaleStatusBeforeAlignAbort) {
    pvtsafety::PvtPublicationTimeline timeline;
    timeline.publish(100u, 1u, 1u);
    ASSERT_GT(timeline.ageMs(1200u), 1000u);

    // A newer PVT is atomically published before this loop evaluates Safety.
    timeline.publish(1200u, 2u, 2u);
    ASSERT_EQ(timeline.ageMs(1200u), 0u);
    ASSERT_TRUE(timeline.publishedInLoop(2u));

    pvtsafety::AlignStartupRecoveryConditions conditions;
    conditions.startSettle = true;
    conditions.fixed = true;
    conditions.hAccAllowed = true;
    conditions.pvtPublishedThisLoop = timeline.publishedInLoop(2u);
    conditions.motorsZero = true;
    conditions.safetyEvaluatedPublishedPvt =
        timeline.timestampMs() == 1200u;
    const bool recoveryRequested =
        pvtsafety::requestAlignStartupFreshPvtRecovery(conditions);
    EXPECT_TRUE(recoveryRequested);

    // The old retained state was pvt_stale; the new Safety candidate is OK.
    EXPECT_TRUE(pvtsafety::applyFreshPvtStatusImmediately(
        recoveryRequested, true, true));
}

TEST(PvtSafetyTimeline, MissingNewPvtKeepsRealStaleAbort) {
    pvtsafety::PvtPublicationTimeline timeline;
    timeline.publish(100u, 1u, 1u);
    constexpr uint32_t currentLoop = 2u;
    constexpr uint32_t safetyThresholdMs = 1000u;
    ASSERT_FALSE(timeline.publishedInLoop(currentLoop));
    ASSERT_GT(timeline.ageMs(1201u), safetyThresholdMs);

    pvtsafety::AlignStartupRecoveryConditions conditions;
    conditions.startSettle = true;
    conditions.fixed = true;
    conditions.hAccAllowed = true;
    conditions.pvtPublishedThisLoop =
        timeline.publishedInLoop(currentLoop);
    conditions.motorsZero = true;
    conditions.safetyEvaluatedPublishedPvt = false;
    const bool recoveryRequested =
        pvtsafety::requestAlignStartupFreshPvtRecovery(conditions);
    EXPECT_FALSE(recoveryRequested);
    EXPECT_FALSE(pvtsafety::applyFreshPvtStatusImmediately(
        recoveryRequested, true, false));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
