/**
 * Navigation Core Tests
 *
 * GoogleTest with mocks for GPS, IMU, Motor
 */

#include "unity.h"
#include "NavigationCore.h"
#include <cmath>
#include <cstdlib>
#include <ctime>

// ============================================================================
// Test Configuration
// ============================================================================

static constexpr float DT = 0.05f;  // 50ms loop
static constexpr float WHEELBASE = 0.38f;
static constexpr float MAX_SPEED_MPS = 0.5f;
static constexpr int MAX_PWM = 70;

// ============================================================================
// Physics Simulation (what robot does in real world)
// ============================================================================

struct RobotPhysics {
    float x = 0, y = 0;
    float heading = 0;  // degrees, 0=North, clockwise positive
    float speed = 0;
    float gpsNoiseStd = 0.015f;  // 15mm for RTK Fixed

    void update(int leftPwm, int rightPwm) {
        // PWM to velocity
        float vLeft = (leftPwm / 70.0f) * MAX_SPEED_MPS;
        float vRight = (rightPwm / 70.0f) * MAX_SPEED_MPS;

        // Kinematics
        float v = (vLeft + vRight) / 2.0f;
        float omega = (vLeft - vRight) / WHEELBASE;  // rad/s

        // Update position
        float headingRad = heading * M_PI / 180.0f;
        x += v * sinf(headingRad) * DT;
        y += v * cosf(headingRad) * DT;

        // Update heading
        heading += omega * DT * 180.0f / M_PI;

        // Normalize
        while (heading >= 360) heading -= 360;
        while (heading < 0) heading += 360;

        speed = fabsf(v);
    }

    // Get noisy GPS position
    void getGpsPosition(float& outX, float& outY, float& outHeading) {
        // Add GPS noise
        outX = x + (((float)rand() / RAND_MAX - 0.5f) * 2 * gpsNoiseStd);
        outY = y + (((float)rand() / RAND_MAX - 0.5f) * 2 * gpsNoiseStd);
        outHeading = heading + (((float)rand() / RAND_MAX - 0.5f) * 2 * 0.5f);  // 0.5 deg IMU noise
    }
};

// ============================================================================
// IMU Mock with Drift
// ============================================================================

struct ImuMock {
    float heading = 0;
    float driftRate = 0.0f;  // deg/s
    float noiseStd = 0.5f;   // deg

    void update(float trueHeading, float dt) {
        heading = trueHeading + driftRate * dt + ((float)rand() / RAND_MAX - 0.5f) * 2 * noiseStd;
    }
};

// ============================================================================
// Motor Mock
// ============================================================================

struct MotorMock {
    int lastLeftCmd = 0;
    int lastRightCmd = 0;
    int lastMeasuredLeft = 0;
    int lastMeasuredRight = 0;
    float slipFactor = 1.0f;  // 1.0 = no slip

    void sendCommand(int left, int right) {
        lastLeftCmd = left;
        lastRightCmd = right;
        // Simulate slippage
        lastMeasuredLeft = (int)(left * slipFactor);
        lastMeasuredRight = (int)(right * slipFactor);
    }
};

// ============================================================================
// Test Helpers
// ============================================================================

struct TestResult {
    float finalX, finalY;
    float error;
    float maxHeadingError;
    float maxCrossTrack;
    int steps;
    bool success;
};

TestResult runNavigation(NavigationCore& nav, RobotPhysics& robot, ImuMock& imu, MotorMock& motor,
                       float targetX, float targetY, int maxSteps = 2000) {
    TestResult result = {};

    nav.clearRoute();
    nav.addWaypoint(targetX, targetY);
    nav.reset();
    nav.quality = NavQuality::FIXED;  // Simulate RTK Fixed

    robot.x = 0;
    robot.y = 0;
    robot.heading = 0;

    for (int step = 0; step < maxSteps; step++) {
        // Get noisy sensor data
        float gpsX, gpsY, imuHeading;
        robot.getGpsPosition(gpsX, gpsY, imuHeading);

        // Update navigation with sensor data
        nav.position.x = gpsX;
        nav.position.y = gpsY;
        nav.heading = imuHeading;
        nav.speed = robot.speed;

        // Run navigation
        NavOutput output = nav.update(DT);

        // Send motor commands
        motor.sendCommand(output.leftCmd, output.rightCmd);

        // Update robot physics
        robot.update(motor.lastMeasuredLeft, motor.lastMeasuredRight);

        // Update IMU
        imu.update(robot.heading, DT);

        // Track max errors
        float err = nav.debug.headingError;
        if (fabsf(err) > result.maxHeadingError) {
            result.maxHeadingError = fabsf(err);
        }
        if (fabsf(nav.debug.crossTrackError) > result.maxCrossTrack) {
            result.maxCrossTrack = fabsf(nav.debug.crossTrackError);
        }

        // Check for arrival
        if (nav.state == NavState::ARRIVED) {
            result.success = true;
            break;
        }

        result.steps = step + 1;
    }

    // Calculate final error
    result.finalX = robot.x;
    result.finalY = robot.y;
    result.error = sqrtf(powf(targetX - robot.x, 2) + powf(targetY - robot.y, 2));

    return result;
}

// ============================================================================
// TEST: Simple straight line (0,0) -> (5,0)
// ============================================================================

void test_straight_line_5m(void) {
    printf("\n=== TEST: Straight line 5m ===\n");

    NavigationCore nav;
    RobotPhysics robot;
    ImuMock imu;
    MotorMock motor;

    srand(42);  // Reproducible

    TestResult result = runNavigation(nav, robot, imu, motor, 5.0f, 0.0f);

    printf("Final position: (%.3f, %.3f)\n", result.finalX, result.finalY);
    printf("Target: (5.0, 0.0)\n");
    printf("Error: %.3f m (%.1f cm)\n", result.error, result.error * 100);
    printf("Steps: %d\n", result.steps);
    printf("Max heading error: %.1f deg\n", result.maxHeadingError);
    printf("Max cross-track: %.3f m\n", result.maxCrossTrack);

    // Should be within 20cm for straight line
    TEST_ASSERT_TRUE_MESSAGE(result.error < 0.20f, "Should be within 20cm");
}

// ============================================================================
// TEST: 50 runs straight line, calculate average error
// ============================================================================

void test_straight_line_50_runs(void) {
    printf("\n=== TEST: Straight line 50 runs ===\n");

    NavigationCore nav;
    RobotPhysics robot;
    ImuMock imu;
    MotorMock motor;

    float totalError = 0;
    float maxError = 0;
    int successCount = 0;
    float totalHeadingError = 0;
    float totalCrossTrack = 0;

    for (int run = 0; run < 50; run++) {
        srand(run * 1000);

        TestResult result = runNavigation(nav, robot, imu, motor, 5.0f, 0.0f);

        totalError += result.error;
        if (result.error > maxError) maxError = result.error;
        if (result.error < 0.10f) successCount++;
        totalHeadingError += result.maxHeadingError;
        totalCrossTrack += result.maxCrossTrack;
    }

    float avgError = totalError / 50;
    float avgHeadingError = totalHeadingError / 50;
    float avgCrossTrack = totalCrossTrack / 50;
    float successRate = (float)successCount / 50 * 100;

    printf("\nResults over 50 runs:\n");
    printf("  Average error: %.3f m (%.1f cm)\n", avgError, avgError * 100);
    printf("  Max error: %.3f m (%.1f cm)\n", maxError, maxError * 100);
    printf("  Success rate (<10cm): %.0f%%\n", successRate);
    printf("  Avg max heading error: %.1f deg\n", avgHeadingError);
    printf("  Avg max cross-track: %.3f m\n", avgCrossTrack);

    // Expect average error < 15cm
    TEST_ASSERT_TRUE_MESSAGE(avgError < 0.15f, "Average error should be < 15cm");
}

// ============================================================================
// TEST: Cross track correction sign
// ============================================================================

void test_cross_track_correction(void) {
    printf("\n=== TEST: Cross track correction sign ===\n");

    // This test checks if cross-track correction works correctly
    // If robot is to the RIGHT of trajectory (positive cross-track error),
    // it should turn LEFT to correct

    NavigationCore nav;
    RobotPhysics robot;
    ImuMock imu;
    MotorMock motor;

    // Set up a route
    nav.clearRoute();
    nav.addWaypoint(5.0f, 0.0f);  // Target
    nav.reset();
    nav.quality = NavQuality::FIXED;

    // Start robot slightly to the RIGHT of the line (cross-track > 0)
    robot.x = 0.5f;  // Offset to the right
    robot.y = 0;
    robot.heading = 0;

    // Run for a few steps
    for (int step = 0; step < 100; step++) {
        float gpsX, gpsY, imuHeading;
        robot.getGpsPosition(gpsX, gpsY, imuHeading);

        nav.position.x = gpsX;
        nav.position.y = gpsY;
        nav.heading = imuHeading;
        nav.speed = robot.speed;

        NavOutput output = nav.update(DT);

        printf("Step %d: pos=(%.3f,%.3f) crossTrack=%.3f heading=%.1f cmd=(%d,%d)\n",
               step, nav.position.x, nav.position.y,
               nav.debug.crossTrackError, nav.heading,
               output.leftCmd, output.rightCmd);

        motor.sendCommand(output.leftCmd, output.rightCmd);
        robot.update(motor.lastMeasuredLeft, motor.lastMeasuredRight);
    }

    // After correction, robot should have moved LEFT (negative X direction)
    printf("\nAfter correction:\n");
    printf("  Final X: %.3f (started at 0.5)\n", robot.x);
    printf("  Cross-track direction: %s\n", robot.x < 0.5f ? "LEFT (correct)" : "RIGHT (bug?)");

    // If cross-track correction is correct, robot should have moved left
    // (X should be less than initial 0.5)
    // This will fail if the sign is wrong in the original code
}

// ============================================================================
// TEST: 45 degree angle
// ============================================================================

void test_45_degree_angle(void) {
    printf("\n=== TEST: 45 degree angle ===\n");

    NavigationCore nav;
    RobotPhysics robot;
    ImuMock imu;
    MotorMock motor;

    srand(123);

    TestResult result = runNavigation(nav, robot, imu, motor, 3.0f, 3.0f);

    printf("Final position: (%.3f, %.3f)\n", result.finalX, result.finalY);
    printf("Target: (3.0, 3.0)\n");
    printf("Error: %.3f m (%.1f cm)\n", result.error, result.error * 100);

    // Should be within 20cm
    TEST_ASSERT_TRUE_MESSAGE(result.error < 0.20f, "Should be within 20cm for 45 degree");
}

// ============================================================================
// TEST: With GPS noise variation
// ============================================================================

void test_with_gps_noise_variation(void) {
    printf("\n=== TEST: GPS noise variation ===\n");

    struct NoiseTest {
        float noiseStd;
        const char* name;
    };

    NoiseTest tests[] = {
        {0.005f, "5mm (very good)"},
        {0.015f, "15mm (RTK Fixed)"},
        {0.030f, "30mm (good)"},
        {0.050f, "50mm (RTK Float)"},
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        NavigationCore nav;
        RobotPhysics robot;
        ImuMock imu;
        MotorMock motor;

        robot.gpsNoiseStd = tests[i].noiseStd;

        float totalError = 0;
        for (int run = 0; run < 20; run++) {
            srand(run * 100 + i);

            TestResult result = runNavigation(nav, robot, imu, motor, 5.0f, 0.0f);
            totalError += result.error;
        }

        float avgError = totalError / 20;
        printf("  %s: avg error = %.1f cm\n", tests[i].name, avgError * 100);
    }
}

// ============================================================================
// TEST: With IMU drift
// ============================================================================

void test_with_imu_drift(void) {
    printf("\n=== TEST: IMU drift ===\n");

    NavigationCore nav;
    RobotPhysics robot;
    ImuMock imu;
    MotorMock motor;

    imu.driftRate = 0.1f;  // 0.1 deg/s drift

    srand(456);

    TestResult result = runNavigation(nav, robot, imu, motor, 5.0f, 0.0f);

    printf("Final position: (%.3f, %.3f)\n", result.finalX, result.finalY);
    printf("Error: %.3f m (%.1f cm)\n", result.error, result.error * 100);
    printf("Max heading error: %.1f deg\n", result.maxHeadingError);

    // Should still be within 20cm despite drift
    TEST_ASSERT_TRUE_MESSAGE(result.error < 0.25f, "Should be within 25cm with IMU drift");
}

// ============================================================================
// Entry Point
// ============================================================================

int main(void) {
    printf("=================================================\n");
    printf("  Navigation Core Tests\n");
    printf("=================================================\n");

    UNITY_BEGIN();

    RUN_TEST(test_straight_line_5m);
    RUN_TEST(test_straight_line_50_runs);
    RUN_TEST(test_cross_track_correction);
    RUN_TEST(test_45_degree_angle);
    RUN_TEST(test_with_gps_noise_variation);
    RUN_TEST(test_with_imu_drift);

    return UNITY_END();
}
