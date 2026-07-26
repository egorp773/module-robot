import rclpy

from module_robot_safety.safety_node import SafetyNode


def test_safety_deadline_classes_are_independently_schedulable():
    """High-rate commands/tick must not exclude critical telemetry."""
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()

    node = SafetyNode()
    active = []
    expected = (
        (
            node._command_input_group,
            node._manual_command_sub,
        ),
        (
            node._critical_telemetry_group,
            node._motor_sub,
        ),
        (
            node._auxiliary_state_group,
            node._imu_sub,
        ),
        (
            node._tick_callback_group,
            node._tick_timer,
        ),
    )
    try:
        assert len({id(group) for group, _ in expected}) == len(expected)

        assert node._manual_command_sub.callback_group is node._command_input_group
        assert node._auto_command_sub.callback_group is node._command_input_group
        assert node._status_sub.callback_group is node._critical_telemetry_group
        assert node._motor_sub.callback_group is node._critical_telemetry_group
        for subscription in (
            node._imu_sub,
            node._gnss_sub,
            node._rtk_sub,
            node._heading_initialized_sub,
            node._route_valid_sub,
            node._localization_valid_sub,
            node._nav2_active_sub,
        ):
            assert subscription.callback_group is node._auxiliary_state_group
        assert node._tick_timer.callback_group is node._tick_callback_group

        # Occupy every deadline class at once. This would fail if any two
        # entities accidentally regress to one MutuallyExclusive group.
        for group, entity in expected:
            assert group.beginning_execution(entity)
            active.append((group, entity))
    finally:
        for group, entity in reversed(active):
            group.ending_execution(entity)
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()
