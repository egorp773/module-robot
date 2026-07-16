#include <cassert>

#include "PiBridgeSafety.h"

using namespace pibridge;

namespace {

MotorHealth healthyStoppedMotor() {
    MotorHealth health;
    health.feedback_available = true;
    health.feedback_fresh = true;
    health.feedback_at_zero = true;
    health.controller_fault = false;
    return health;
}

ArmPayload manualArm(uint32_t nonce) {
    ArmPayload command{};
    command.arm_nonce = nonce;
    command.requested_mode = static_cast<uint8_t>(ControlMode::MANUAL);
    return command;
}

CmdVelPayload forwardCommand() {
    CmdVelPayload command{};
    command.linear_mm_s = 100;
    command.angular_mrad_s = 0;
    command.command_timeout_ms = 300u;
    command.control_mode = static_cast<uint8_t>(ControlMode::MANUAL);
    return command;
}

}  // namespace

int main() {
    const MotorHealth healthy = healthyStoppedMotor();
    SafetyMachine safety;
    SafetySnapshot snapshot = safety.snapshot(10u);
    assert(snapshot.state == EspState::BOOT);
    assert(snapshot.applied_left_percent == 0);
    assert(snapshot.applied_right_percent == 0);

    safety.completeBoot(10u);
    snapshot = safety.snapshot(10u);
    assert(snapshot.state == EspState::DISCONNECTED);
    assert(safety.arm(manualArm(1u), healthy, 11u).result != AckResult::OK);

    safety.acceptHello(100u);
    assert(safety.snapshot(100u).state == EspState::DISARMED);
    assert(safety.arm(manualArm(1u), healthy, 110u).result == AckResult::OK);
    assert(safety.snapshot(110u).state == EspState::ARMED);
    assert(safety.snapshot(110u).applied_left_percent == 0);

    CmdVelPayload forward = forwardCommand();
    assert(safety.commandVelocity(10u, forward, 120u).result == AckResult::OK);
    safety.tick(220u, healthy);
    snapshot = safety.snapshot(220u);
    assert(snapshot.applied_left_percent > 0);
    assert(snapshot.applied_right_percent > 0);

    const uint32_t before_stop_generation = snapshot.hard_zero_generation;
    assert(safety.stop().result == AckResult::OK);
    snapshot = safety.snapshot(221u);
    assert(snapshot.state == EspState::ARMED);
    assert(snapshot.applied_left_percent == 0);
    assert(snapshot.applied_right_percent == 0);
    assert(snapshot.hard_zero_generation != before_stop_generation);

    // The default (still TODO_CALIBRATE) mapping implements the requested
    // standard differential formula: positive angular velocity makes the
    // left target negative and the right target positive at zero linear speed.
    SafetyMachine mixer;
    mixer.completeBoot(100u);
    mixer.acceptHello(101u);
    assert(mixer.arm(manualArm(40u), healthy, 102u).result == AckResult::OK);
    CmdVelPayload rotate = forward;
    rotate.linear_mm_s = 0;
    rotate.angular_mrad_s = 600;
    assert(mixer.commandVelocity(1u, rotate, 110u).result == AckResult::OK);
    mixer.tick(210u, healthy);
    const SafetySnapshot rotate_snapshot = mixer.snapshot(210u);
    assert(rotate_snapshot.applied_left_percent < 0);
    assert(rotate_snapshot.applied_right_percent > 0);

    SafetyReply stale = safety.commandVelocity(10u, forward, 222u);
    assert(stale.result == AckResult::STALE_SEQUENCE);
    assert(stale.detail == AckDetail::DUPLICATE_SEQUENCE);
    stale = safety.commandVelocity(9u, forward, 223u);
    assert(stale.result == AckResult::STALE_SEQUENCE);
    assert(stale.detail == AckDetail::OUT_OF_ORDER_SEQUENCE);
    assert(safety.commandVelocity(11u, forward, 224u).result == AckResult::OK);

    // At exactly 300 ms without a valid receive, watchdog is a latched fault
    // and a hard zero (not one scheduler tick later).
    safety.tick(524u, healthy);
    snapshot = safety.snapshot(524u);
    assert(snapshot.state == EspState::FAULT);
    assert(snapshot.fault == FaultCode::CMD_VEL_TIMEOUT);
    assert(snapshot.watchdog_trips == 1u);
    assert(snapshot.applied_left_percent == 0);
    assert(safety.commandVelocity(12u, forward, 526u).result ==
           AckResult::FAULT_LATCHED);
    FaultTransition fault;
    assert(safety.consumeFaultTransition(fault));
    assert(fault.fault == FaultCode::CMD_VEL_TIMEOUT);
    assert(!safety.consumeFaultTransition(fault));

    assert(safety.resetFault(healthy).result == AckResult::OK);
    assert(safety.snapshot(527u).state == EspState::DISARMED);
    assert(safety.arm(manualArm(2u), healthy, 530u).result == AckResult::OK);
    assert(safety.estop(1u).result == AckResult::OK);
    snapshot = safety.snapshot(531u);
    assert(snapshot.state == EspState::ESTOP);
    assert(snapshot.estop_latched);
    assert(snapshot.applied_left_percent == 0);
    assert(safety.arm(manualArm(3u), healthy, 532u).result != AckResult::OK);
    EstopTransition estop;
    assert(safety.consumeEstopTransition(estop));
    assert(estop.latched);
    // Duplicate ESTOP is safe but does not spam transition telemetry.
    assert(safety.estop(1u).result == AckResult::OK);
    assert(!safety.consumeEstopTransition(estop));
    assert(safety.resetEstop(healthy).result == AckResult::OK);
    assert(safety.snapshot(533u).state == EspState::DISARMED);
    assert(safety.consumeEstopTransition(estop));
    assert(!estop.latched);

    assert(safety.arm(manualArm(4u), healthy, 540u).result == AckResult::OK);
    assert(safety.disarm().result == AckResult::OK);
    snapshot = safety.snapshot(541u);
    assert(snapshot.state == EspState::DISARMED);
    assert(snapshot.control_mode == static_cast<uint8_t>(ControlMode::INACTIVE));
    assert(snapshot.applied_left_percent == 0);

    // Sequence wrap is newer in serial-number arithmetic.
    SafetyMachine wrapping;
    wrapping.completeBoot(100u);
    wrapping.acceptHello(101u);
    assert(wrapping.arm(manualArm(10u), healthy, 102u).result == AckResult::OK);
    assert(wrapping.commandVelocity(0xFFFFFFFEu, forward, 103u).result ==
           AckResult::OK);
    assert(wrapping.commandVelocity(0xFFFFFFFFu, forward, 104u).result ==
           AckResult::OK);
    assert(wrapping.commandVelocity(0u, forward, 105u).result == AckResult::OK);

    // A fresh HELLO is a reconnect/session boundary and may only remove motion
    // authority, even while the old session was ARMED.
    wrapping.acceptHello(106u);
    snapshot = wrapping.snapshot(106u);
    assert(snapshot.state == EspState::DISARMED);
    assert(snapshot.applied_left_percent == 0);
    assert(snapshot.applied_right_percent == 0);
    assert(wrapping.arm(manualArm(11u), healthy, 107u).result == AckResult::OK);

    // Loss of feedback while armed is independently latched.
    MotorHealth lost = healthy;
    lost.feedback_fresh = false;
    wrapping.tick(108u, lost);
    snapshot = wrapping.snapshot(108u);
    assert(snapshot.state == EspState::FAULT);
    assert(snapshot.fault == FaultCode::MOTOR_FEEDBACK_LOST);
    assert(snapshot.applied_left_percent == 0);

    // Heartbeats are connection liveness only and never refresh CMD_VEL.
    SafetyMachine heartbeat_only;
    heartbeat_only.completeBoot(100u);
    heartbeat_only.acceptHello(101u);
    assert(heartbeat_only.arm(manualArm(30u), healthy, 102u).result ==
           AckResult::OK);
    assert(heartbeat_only.commandVelocity(1u, forward, 110u).result ==
           AckResult::OK);
    heartbeat_only.noteHeartbeat(350u);
    heartbeat_only.tick(410u, healthy);
    snapshot = heartbeat_only.snapshot(410u);
    assert(snapshot.state == EspState::FAULT);
    assert(snapshot.fault == FaultCode::CMD_VEL_TIMEOUT);

    // A duplicate with a valid payload is still not a valid receive for the
    // motor watchdog. It cannot keep the previous movement alive.
    SafetyMachine duplicate_only;
    duplicate_only.completeBoot(100u);
    duplicate_only.acceptHello(101u);
    assert(duplicate_only.arm(manualArm(33u), healthy, 102u).result ==
           AckResult::OK);
    assert(duplicate_only.commandVelocity(7u, forward, 110u).result ==
           AckResult::OK);
    assert(duplicate_only.commandVelocity(7u, forward, 350u).result ==
           AckResult::STALE_SEQUENCE);
    duplicate_only.tick(410u, healthy);
    snapshot = duplicate_only.snapshot(410u);
    assert(snapshot.state == EspState::FAULT);
    assert(snapshot.fault == FaultCode::CMD_VEL_TIMEOUT);

    // A sender cannot extend its authority past the compiled 300 ms ceiling.
    SafetyMachine timeout_capped;
    timeout_capped.completeBoot(100u);
    timeout_capped.acceptHello(101u);
    assert(timeout_capped.arm(manualArm(34u), healthy, 102u).result ==
           AckResult::OK);
    CmdVelPayload long_timeout = forward;
    long_timeout.command_timeout_ms = 5000u;
    assert(timeout_capped.commandVelocity(1u, long_timeout, 110u).result ==
           AckResult::OK);
    timeout_capped.tick(409u, healthy);
    assert(timeout_capped.snapshot(409u).state == EspState::ARMED);
    timeout_capped.tick(410u, healthy);
    snapshot = timeout_capped.snapshot(410u);
    assert(snapshot.state == EspState::FAULT);
    assert(snapshot.fault == FaultCode::CMD_VEL_TIMEOUT);

    // Loss of the Pi heartbeat removes the handshake and hard-zeros without
    // ever preserving ARMED authority for a reconnect.
    SafetyMachine disconnected;
    disconnected.completeBoot(100u);
    disconnected.acceptHello(101u);
    assert(disconnected.arm(manualArm(31u), healthy, 102u).result ==
           AckResult::OK);
    assert(disconnected.commandVelocity(1u, forward, 110u).result ==
           AckResult::OK);
    disconnected.tick(200u, healthy);
    assert(disconnected.snapshot(200u).applied_left_percent > 0);
    disconnected.noteHeartbeat(200u);
    assert(disconnected.commandVelocity(2u, forward, 1699u).result ==
           AckResult::OK);
    disconnected.tick(1700u, healthy);
    snapshot = disconnected.snapshot(1700u);
    assert(snapshot.state == EspState::DISCONNECTED);
    assert(!snapshot.handshake);
    assert(snapshot.applied_left_percent == 0);
    assert(snapshot.applied_right_percent == 0);

    // Timestamp zero is valid after millis() wraps. It must not be mistaken
    // for "no command" or trigger before the configured 300 ms deadline.
    SafetyMachine rollover;
    rollover.completeBoot(0xFFFFFF00u);
    rollover.acceptHello(0xFFFFFFE0u);
    assert(rollover.arm(manualArm(32u), healthy, 0xFFFFFFF0u).result ==
           AckResult::OK);
    assert(rollover.commandVelocity(1u, forward, 0u).result == AckResult::OK);
    rollover.tick(299u, healthy);
    assert(rollover.snapshot(299u).state == EspState::ARMED);
    rollover.tick(300u, healthy);
    snapshot = rollover.snapshot(300u);
    assert(snapshot.state == EspState::FAULT);
    assert(snapshot.fault == FaultCode::CMD_VEL_TIMEOUT);

    // ARM never creates motion and still requires a first fresh CMD_VEL
    // within the 300 ms watchdog window.
    SafetyMachine no_command;
    no_command.completeBoot(1000u);
    no_command.acceptHello(1001u);
    assert(no_command.arm(manualArm(20u), healthy, 1002u).result ==
           AckResult::OK);
    no_command.tick(1303u, healthy);
    snapshot = no_command.snapshot(1303u);
    assert(snapshot.state == EspState::FAULT);
    assert(snapshot.fault == FaultCode::CMD_VEL_TIMEOUT);
    assert(snapshot.last_cmd_age_ms == 0xFFFFFFFFu);

    // A newly constructed/rebooted machine can never retain movement.
    SafetyMachine rebooted;
    snapshot = rebooted.snapshot(1u);
    assert(snapshot.state == EspState::BOOT);
    assert(snapshot.applied_left_percent == 0);
    assert(snapshot.applied_right_percent == 0);
    return 0;
}
