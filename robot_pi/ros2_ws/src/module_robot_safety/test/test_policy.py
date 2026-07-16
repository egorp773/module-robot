from module_robot_safety.policy import (
    CommandSlot,
    Conditions,
    FaultLatch,
    State,
    auto_rejection,
    manual_rejection,
    selected_source,
)


def manual_ready(**changes):
    values = dict(
        connected=True,
        esp32_armed=True,
        motor_feedback_fresh=True,
        motor_fault_free=True,
        estop_clear=True,
        operator_armed=True,
    )
    values.update(changes)
    return Conditions(**values)


def test_manual_does_not_require_rtk_or_nav2():
    assert manual_rejection(manual_ready()) is None


def test_auto_is_strictly_gated():
    assert auto_rejection(manual_ready()) == "AUTO_NOT_EXPLICITLY_ARMED"


def test_auto_ready_requires_every_explicit_condition():
    conditions = manual_ready(
        explicit_auto_arm=True,
        rtk_fresh=True,
        rtk_fixed=True,
        rtk_accuracy_ok=True,
        imu_fresh=True,
        gnss_fresh=True,
        heading_initialized=True,
        localization_valid=True,
        route_valid=True,
        nav2_active=True,
    )
    assert auto_rejection(conditions) is None


def test_estop_and_stop_precede_motion_sources():
    assert selected_source(State.ESTOP, False, True, True) == "ESTOP"
    assert selected_source(State.MANUAL, True, True, True) == "STOP"


def test_commands_are_never_combined():
    assert selected_source(State.MANUAL, False, True, True) == "MANUAL"
    assert selected_source(State.AUTO, False, True, True) == "AUTO"


def test_manual_preempts_auto_but_zero_is_used_when_both_are_stale():
    assert selected_source(State.AUTO, False, True, True) == "MANUAL"
    assert selected_source(State.AUTO, False, False, False) == "ZERO"


def test_command_slot_rejects_stale_and_pre_authority_generations():
    slot = CommandSlot()
    slot.update(0.1, 0.2, received_monotonic_s=10.0, generation=4)
    assert slot.fresh(10.2, timeout_s=0.2, minimum_generation=4)
    assert not slot.fresh(10.200001, timeout_s=0.2, minimum_generation=4)
    assert not slot.fresh(10.1, timeout_s=0.2, minimum_generation=5)
    slot.invalidate()
    assert not slot.fresh(10.1, timeout_s=0.2)


def test_fault_latch_requires_explicit_reset_before_a_new_transition():
    latch = FaultLatch()
    assert latch.latch(0x8001, "FIRST")
    assert latch.active
    assert not latch.latch(0x8002, "SECOND")
    assert latch.code == 0x8001
    latch.reset()
    assert not latch.active
    assert latch.latch(0x8002, "SECOND")
    assert latch.occurrence_count == 3
