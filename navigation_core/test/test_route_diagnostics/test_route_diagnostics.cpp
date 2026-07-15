#include <gtest/gtest.h>

#include "RouteRunDiagnostics.h"

#include <cmath>

namespace {

using routediag::CornerSnapshots;
using routediag::PathIntervalKind;
using routediag::SegmentPathMetrics;

TEST(RouteDiagnostics, CornerSnapshotsAreCapturedExactlyOnce) {
    CornerSnapshots snapshots;

    EXPECT_TRUE(routediag::captureOnce(
        snapshots.cornerApproachBrakeStart, 1.0f, 2.0f, 64.9f));
    EXPECT_TRUE(routediag::captureOnce(
        snapshots.cornerPhysicalStopBeforeTurn, 1.01f, 2.01f, 64.9f));
    EXPECT_TRUE(routediag::captureOnce(
        snapshots.turnStart, 1.01f, 2.01f, 64.9f));

    // A later BRAKE after the turn must not overwrite the turn start.
    EXPECT_FALSE(routediag::captureOnce(
        snapshots.turnStart, 1.06f, 2.03f, 144.6f));
    EXPECT_FLOAT_EQ(snapshots.turnStart.x, 1.01f);
    EXPECT_FLOAT_EQ(snapshots.turnStart.y, 2.01f);
    EXPECT_FLOAT_EQ(snapshots.turnStart.headingDeg, 64.9f);
}

TEST(RouteDiagnostics, TurnSummaryUsesWrappedPhysicalStopDelta) {
    CornerSnapshots snapshots;
    ASSERT_TRUE(routediag::captureOnce(
        snapshots.turnStart, 0.0f, 0.0f, 350.0f));
    ASSERT_TRUE(routediag::captureOnce(
        snapshots.turnPhysicalStop, 0.04f, 0.03f, 71.6f));

    EXPECT_NEAR(snapshots.turnActualDeg(), 81.6f, 1e-4f);
    EXPECT_NEAR(snapshots.turnPositionChordM(), 0.05f, 1e-5f);
    EXPECT_TRUE(std::isfinite(snapshots.equivalentRadiusM()));
}

TEST(RouteDiagnostics, SegmentMetricsSeparatePoweredDriveAndBrakeCoast) {
    SegmentPathMetrics metrics;
    metrics.reset(0.0f, 0.0f, 90.0f);

    EXPECT_NEAR(metrics.observePvt(
        0.50f, 0.0f, PathIntervalKind::POWERED_DRIVE), 0.50f, 1e-6f);
    EXPECT_NEAR(metrics.observePvt(
        0.90f, 0.0f, PathIntervalKind::POWERED_DRIVE), 0.40f, 1e-6f);
    EXPECT_TRUE(metrics.captureBrake(0.90f, 0.0f, 90.0f,
                                     0.90f, 0.0f));

    EXPECT_NEAR(metrics.observePvt(
        1.00f, 0.0f, PathIntervalKind::BRAKE_COAST), 0.10f, 1e-6f);
    EXPECT_NEAR(metrics.observePvt(
        1.05f, 0.0f, PathIntervalKind::BRAKE_COAST), 0.05f, 1e-6f);
    EXPECT_TRUE(metrics.capturePhysicalStop(
        1.05f, 0.0f, 90.0f, 1.05f, 0.0f));

    EXPECT_NEAR(metrics.poweredDrivePathM, 0.90f, 1e-6f);
    EXPECT_NEAR(metrics.brakeCoastPathM, 0.15f, 1e-6f);
    EXPECT_NEAR(metrics.totalTranslationalPathM(), 1.05f, 1e-6f);
    EXPECT_NEAR(metrics.plannedStartToActualStopChordM(), 1.05f, 1e-6f);
    // Powered-only path is shorter than the stop chord by definition here;
    // the consistency check must use the total translational path.
    EXPECT_LT(metrics.poweredDrivePathM,
              metrics.plannedStartToActualStopChordM());
    EXPECT_TRUE(metrics.totalPathCoversActualStopChord());
}

TEST(RouteDiagnostics, ClosedSegmentDoesNotIncludeTurnDisplacement) {
    SegmentPathMetrics metrics;
    metrics.reset(0.0f, 0.0f, 0.0f);
    metrics.observePvt(0.0f, 1.0f, PathIntervalKind::POWERED_DRIVE);
    ASSERT_TRUE(metrics.capturePhysicalStop(
        0.0f, 1.0f, 0.0f, 1.0f, 0.0f));

    metrics.observePvt(0.08f, 1.04f, PathIntervalKind::BRAKE_COAST);
    EXPECT_NEAR(metrics.totalTranslationalPathM(), 1.0f, 1e-6f);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
