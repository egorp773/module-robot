/**
 * Navigation Core Standalone Test
 * Compile: g++ -std=c++11 -o nav_test nav_test_standalone.cpp -lm
 * Run: ./nav_test
 */

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <ctime>

// ============================================================================
// Constants from rover.cpp
// ============================================================================

static constexpr float MAX_SPEED = 0.5f;
static constexpr float FLOAT_SPEED = 0.25f;
static constexpr float DEGRADED_SPEED = 0.15f;
static constexpr float HOLD_SPEED = 0.05f;
static constexpr int MAX_SPEED_PERCENT = 70;
static constexpr float ARRIVAL_DIST_M = 0.1f;
static constexpr float ARRIVAL_CONFIRM_TIME_S = 2.0f;
static constexpr float ARRIVAL_APPROACH_DIST_M = 0.3f;
static constexpr float LOOKAHEAD_MIN_M = 0.5f;
static constexpr float LOOKAHEAD_MAX_M = 1.2f;
static constexpr float LOOKAHEAD_SPEED_GAIN = 1.5f;
static constexpr float K_HEADING = 0.5f;
static constexpr float K_CROSSTRACK = 15.0f;
static constexpr float TURN_THRESHOLD_1 = 30.0f;
static constexpr float TURN_THRESHOLD_2 = 60.0f;
static constexpr float TURN_THRESHOLD_3 = 90.0f;
static constexpr float TURN_THRESHOLD_4 = 120.0f;

// Physics
static constexpr float WHEELBASE = 0.38f;
static constexpr float GPS_NOISE_STD = 0.015f;  // 15mm
static constexpr float IMU_NOISE_STD = 0.5f;   // 0.5 deg
static constexpr float DT = 0.05f;

// ============================================================================
// Navigation Core (simplified from NavigationCore.h)
// ============================================================================

struct LocalCoords {
    float x, y;
};

enum class NavState { IDLE, MOVING, APPROACHING, ARRIVED, ERROR };
enum class NavQuality { NONE, DEGRADED, FLOAT_OK, FIXED };

struct NavOutput {
    int leftCmd, rightCmd;
    float headingError, crossTrackError, distToTarget, speed;
    NavState state;
};

struct NavDebug {
    float posX, posY, heading, desiredHeading;
    float lookaheadX, lookaheadY;
    float forwardCmd, turnCmd, crossTrackCorrection;
    float crossTrackError;
    uint8_t currentWpIndex, wpCount;
};

class NavigationCore {
public:
    NavState state = NavState::IDLE;
    NavQuality quality = NavQuality::FIXED;
    float positionX = 0, positionY = 0;
    float heading = 0;
    float speed = 0;

    static constexpr int MAX_WAYPOINTS = 160;
    LocalCoords waypoints[MAX_WAYPOINTS];
    int waypointCount = 0;
    int currentWaypointIndex = 0;

    NavDebug debug;

    float forwardScale = 1.0f;
    float turnScale = 1.0f;
    bool invertForward = false;
    bool invertSteering = false;

    void reset() {
        state = NavState::IDLE;
        currentWaypointIndex = 0;
        speed = 0;
    }

    void clearRoute() {
        waypointCount = 0;
        currentWaypointIndex = 0;
    }

    bool addWaypoint(float x, float y) {
        if (waypointCount >= MAX_WAYPOINTS) return false;
        waypoints[waypointCount++] = {x, y};
        return true;
    }

    bool isRouteReady() const {
        return waypointCount > 0;
    }

    static float normalizeAngle(float a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    NavOutput update(float dt) {
        NavOutput output = {};

        if (!isRouteReady()) {
            state = NavState::IDLE;
            output.state = state;
            return output;
        }

        if (currentWaypointIndex >= waypointCount) {
            state = NavState::ARRIVED;
            output.state = state;
            return output;
        }

        LocalCoords target = waypoints[currentWaypointIndex];
        float dx = target.x - positionX;
        float dy = target.y - positionY;
        float distToTarget = hypotf(dx, dy);
        debug.posX = positionX;
        debug.posY = positionY;
        output.distToTarget = distToTarget;

        // Check arrival
        if (distToTarget < ARRIVAL_DIST_M && speed < 0.03f) {
            currentWaypointIndex++;
            if (currentWaypointIndex >= waypointCount) {
                state = NavState::ARRIVED;
            } else {
                state = NavState::MOVING;
            }
            output.state = state;
            return output;
        }

        // Update state
        if (distToTarget < ARRIVAL_APPROACH_DIST_M) {
            state = NavState::APPROACHING;
        } else {
            state = NavState::MOVING;
        }

        // Pure pursuit lookahead
        float lookahead = LOOKAHEAD_MIN_M + speed * LOOKAHEAD_SPEED_GAIN;
        if (lookahead > LOOKAHEAD_MAX_M) lookahead = LOOKAHEAD_MAX_M;

        LocalCoords lookaheadTarget = target;

        // Look past current waypoint
        if (distToTarget < lookahead && currentWaypointIndex + 1 < waypointCount) {
            float remaining = lookahead - distToTarget;
            LocalCoords a = target;
            for (int segIdx = currentWaypointIndex + 1; segIdx < waypointCount; segIdx++) {
                LocalCoords b = waypoints[segIdx];
                float segLen = hypotf(b.x - a.x, b.y - a.y);
                if (segLen <= 0.01f) continue;
                if (remaining <= segLen) {
                    float t = remaining / segLen;
                    lookaheadTarget.x = a.x + (b.x - a.x) * t;
                    lookaheadTarget.y = a.y + (b.y - a.y) * t;
                    break;
                }
                remaining -= segLen;
                a = b;
            }
        }
        debug.lookaheadX = lookaheadTarget.x;
        debug.lookaheadY = lookaheadTarget.y;

        // Desired heading
        float dxLook = lookaheadTarget.x - positionX;
        float dyLook = lookaheadTarget.y - positionY;
        float desiredHeading = atan2f(dxLook, dyLook) * 180.0f / M_PI;
        if (desiredHeading < 0) desiredHeading += 360.0f;
        debug.desiredHeading = desiredHeading;

        // Heading error
        float headingError = normalizeAngle(desiredHeading - heading);
        debug.headingError = headingError;
        output.headingError = headingError;

        // Cross track error
        float crossTrackError = 0.0f;
        if (currentWaypointIndex > 0) {
            LocalCoords a = waypoints[currentWaypointIndex - 1];
            LocalCoords b = target;
            float segLen = hypotf(b.x - a.x, b.y - a.y);
            if (segLen > 0.1f) {
                float dxSeg = b.x - a.x;
                float dySeg = b.y - a.y;
                crossTrackError = (dxSeg * (positionY - a.y) - dySeg * (positionX - a.x)) / segLen;
            }
        }
        debug.crossTrackError = crossTrackError;
        output.crossTrackError = crossTrackError;

        // Speed selection
        float baseSpeed = MAX_SPEED;
        switch (quality) {
            case NavQuality::FIXED: baseSpeed = MAX_SPEED; break;
            case NavQuality::FLOAT_OK: baseSpeed = FLOAT_SPEED; break;
            case NavQuality::DEGRADED: baseSpeed = DEGRADED_SPEED; break;
            default: baseSpeed = 0; break;
        }

        // Speed reduction for turns
        if (fabsf(headingError) > TURN_THRESHOLD_4 && distToTarget > ARRIVAL_APPROACH_DIST_M) {
            baseSpeed = 0.0f;
        } else if (fabsf(headingError) > TURN_THRESHOLD_3) {
            baseSpeed *= 0.3f;
        } else if (fabsf(headingError) > TURN_THRESHOLD_2) {
            baseSpeed *= 0.5f;
        } else if (fabsf(headingError) > TURN_THRESHOLD_1) {
            baseSpeed *= 0.7f;
        }

        // Approach mode
        if (state == NavState::APPROACHING) {
            baseSpeed *= 0.5f;
            if (distToTarget < 0.2f) baseSpeed *= 0.3f;
        }

        // Cross track speed reduction
        if (fabsf(crossTrackError) > 0.5f) baseSpeed *= 0.8f;
        if (fabsf(crossTrackError) > 1.0f) baseSpeed *= 0.6f;

        debug.speed = baseSpeed;
        output.speed = baseSpeed;

        // Motor commands
        float forwardCmd = (baseSpeed / MAX_SPEED) * MAX_SPEED_PERCENT * forwardScale;
        float turnCmd = K_HEADING * headingError * turnScale;

        // Cross track correction - THIS IS THE POTENTIAL BUG
        // Original code: turnCmd += K_CROSSTRACK * crossTrackError
        turnCmd += K_CROSSTRACK * crossTrackError * turnScale;

        debug.forwardCmd = forwardCmd;
        debug.turnCmd = turnCmd;
        debug.crossTrackCorrection = K_CROSSTRACK * crossTrackError * turnScale;

        // Limit turn
        float maxTurn = MAX_SPEED_PERCENT * 0.6f;
        if (turnCmd > maxTurn) turnCmd = maxTurn;
        if (turnCmd < -maxTurn) turnCmd = -maxTurn;

        // Inversions
        if (invertForward) forwardCmd = -forwardCmd;
        if (invertSteering) turnCmd = -turnCmd;

        // Motor output
        float left = forwardCmd + turnCmd;
        float right = forwardCmd - turnCmd;
        if (left > MAX_SPEED_PERCENT) left = MAX_SPEED_PERCENT;
        if (left < -MAX_SPEED_PERCENT) left = -MAX_SPEED_PERCENT;
        if (right > MAX_SPEED_PERCENT) right = MAX_SPEED_PERCENT;
        if (right < -MAX_SPEED_PERCENT) right = -MAX_SPEED_PERCENT;

        output.leftCmd = (int)left;
        output.rightCmd = (int)right;
        output.state = state;

        debug.currentWpIndex = currentWaypointIndex;
        debug.wpCount = waypointCount;

        return output;
    }
};

// ============================================================================
// Physics Simulation
// ============================================================================

struct RobotPhysics {
    float x = 0, y = 0;
    float heading = 0;
    float speed = 0;

    void update(int leftPwm, int rightPwm) {
        float vLeft = (leftPwm / 70.0f) * MAX_SPEED;
        float vRight = (rightPwm / 70.0f) * MAX_SPEED;
        float v = (vLeft + vRight) / 2.0f;
        float omega = (vLeft - vRight) / WHEELBASE;

        float headingRad = heading * M_PI / 180.0f;
        x += v * sinf(headingRad) * DT;
        y += v * cosf(headingRad) * DT;
        heading += omega * DT * 180.0f / M_PI;

        while (heading >= 360) heading -= 360;
        while (heading < 0) heading += 360;

        speed = fabsf(v);
    }

    void getNoisyPosition(float& outX, float& outY, float& outHeading) {
        outX = x + (((float)rand() / RAND_MAX - 0.5f) * 2 * GPS_NOISE_STD);
        outY = y + (((float)rand() / RAND_MAX - 0.5f) * 2 * GPS_NOISE_STD);
        outHeading = heading + (((float)rand() / RAND_MAX - 0.5f) * 2 * IMU_NOISE_STD);
    }
};

// ============================================================================
// Test Runner
// ============================================================================

struct TestResult {
    float finalX, finalY;
    float error;
    float maxHeadingError;
    float maxCrossTrack;
    int steps;
    bool success;
};

TestResult runTest(float targetX, float targetY, int seed = 42) {
    srand(seed);

    NavigationCore nav;
    RobotPhysics robot;

    nav.clearRoute();
    nav.addWaypoint(targetX, targetY);
    nav.reset();
    nav.quality = NavQuality::FIXED;

    TestResult result = {};

    for (int step = 0; step < 2000; step++) {
        float gpsX, gpsY, imuHeading;
        robot.getNoisyPosition(gpsX, gpsY, imuHeading);

        nav.positionX = gpsX;
        nav.positionY = gpsY;
        nav.heading = imuHeading;
        nav.speed = robot.speed;

        NavOutput output = nav.update(DT);
        robot.update(output.leftCmd, output.rightCmd);

        if (fabsf(nav.debug.headingError) > result.maxHeadingError) {
            result.maxHeadingError = fabsf(nav.debug.headingError);
        }
        if (fabsf(nav.debug.crossTrackError) > result.maxCrossTrack) {
            result.maxCrossTrack = fabsf(nav.debug.crossTrackError);
        }

        if (nav.state == NavState::ARRIVED) {
            result.success = true;
            break;
        }
        result.steps = step + 1;
    }

    result.finalX = robot.x;
    result.finalY = robot.y;
    result.error = hypotf(targetX - robot.x, targetY - robot.y);

    return result;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    printf("\n");
    printf("==================================================\n");
    printf("  Navigation Core Standalone Test\n");
    printf("==================================================\n");
    printf("\n");

    // Test 1: Single run 5m straight
    printf("=== Test 1: Straight line 5m ===\n");
    TestResult r1 = runTest(5.0f, 0.0f);
    printf("  Final: (%.3f, %.3f)\n", r1.finalX, r1.finalY);
    printf("  Target: (5.0, 0.0)\n");
    printf("  Error: %.3f m (%.1f cm)\n", r1.error, r1.error * 100);
    printf("  Steps: %d\n", r1.steps);
    printf("  Success: %s\n", r1.success ? "YES" : "NO");
    printf("  Max heading error: %.1f deg\n", r1.maxHeadingError);
    printf("  Max cross-track: %.3f m\n", r1.maxCrossTrack);
    printf("\n");

    // Test 2: 50 runs
    printf("=== Test 2: 50 runs (5m straight) ===\n");
    float totalError = 0;
    float maxError = 0;
    int successCount = 0;

    for (int i = 0; i < 50; i++) {
        TestResult r = runTest(5.0f, 0.0f, i * 1000);
        totalError += r.error;
        if (r.error > maxError) maxError = r.error;
        if (r.error < 0.10f) successCount++;
    }

    printf("  Average error: %.3f m (%.1f cm)\n", totalError / 50, totalError / 50 * 100);
    printf("  Max error: %.3f m (%.1f cm)\n", maxError, maxError * 100);
    printf("  Success rate (<10cm): %.0f%% (%d/50)\n", (float)successCount / 50 * 100, successCount);
    printf("\n");

    // Test 3: Cross-track correction check
    printf("=== Test 3: Cross-track correction sign ===\n");
    printf("  Testing if robot corrects correctly when offset...\n");

    // Run multiple times with different starting positions
    float avgOffsetCorrection = 0;
    for (int i = 0; i < 20; i++) {
        srand(i * 500);

        NavigationCore nav;
        RobotPhysics robot;
        robot.x = 0.3f;  // Start offset to the right
        robot.y = 0;
        robot.heading = 0;

        nav.clearRoute();
        nav.addWaypoint(5.0f, 0.0f);
        nav.reset();
        nav.quality = NavQuality::FIXED;

        for (int step = 0; step < 500; step++) {
            float gpsX, gpsY, imuHeading;
            robot.getNoisyPosition(gpsX, gpsY, imuHeading);
            nav.positionX = gpsX;
            nav.positionY = gpsY;
            nav.heading = imuHeading;

            NavOutput output = nav.update(DT);
            robot.update(output.leftCmd, output.rightCmd);
        }

        // If cross-track is positive (robot right of line),
        // turnCmd should be negative (turn left to correct)
        printf("  Run %2d: start offset=%.2f, final x=%.3f, crossTrack=%.3f, cmd turn=%.1f\n",
               i, 0.3f, robot.x, nav.debug.crossTrackError, nav.debug.turnCmd);
        avgOffsetCorrection += nav.debug.turnCmd;
    }
    printf("  Avg turn command (robot right of line): %.2f\n", avgOffsetCorrection / 20);
    printf("  Note: If positive, cross-track correction has wrong sign!\n");
    printf("\n");

    // Test 4: 45 degree angle
    printf("=== Test 4: 45 degree angle ===\n");
    TestResult r4 = runTest(3.0f, 3.0f, 123);
    printf("  Final: (%.3f, %.3f)\n", r4.finalX, r4.finalY);
    printf("  Target: (3.0, 3.0)\n");
    printf("  Error: %.3f m (%.1f cm)\n", r4.error, r4.error * 100);
    printf("\n");

    // Summary
    printf("==================================================\n");
    printf("  Summary\n");
    printf("==================================================\n");
    printf("  5m straight: %.1f cm avg error\n", totalError / 50 * 100);
    printf("  45 deg: %.1f cm error\n", r4.error * 100);
    printf("  Success rate: %.0f%%\n", (float)successCount / 50 * 100);
    printf("\n");

    return 0;
}
