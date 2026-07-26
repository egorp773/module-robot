import rclpy

from module_robot_esp32_bridge.bridge_node import Esp32Bridge


def test_serial_deadline_callbacks_can_run_while_control_group_is_busy():
    """Command transmission and heartbeat must not share input exclusion."""
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()

    node = Esp32Bridge()
    control_started = False
    command_started = False
    heartbeat_started = False
    try:
        assert node._command_timer_callback_group is not node._control_callback_group
        assert node._heartbeat_callback_group is not node._control_callback_group
        assert (
            node._command_timer_callback_group
            is not node._heartbeat_callback_group
        )
        assert (
            node._cmd_vel_subscription.callback_group
            is node._control_callback_group
        )
        assert (
            node._safety_state_subscription.callback_group
            is node._control_callback_group
        )
        assert (
            node._command_timer.callback_group
            is node._command_timer_callback_group
        )
        assert node._heartbeat_timer.callback_group is node._heartbeat_callback_group

        control_started = node._control_callback_group.beginning_execution(
            node._cmd_vel_subscription
        )
        assert control_started
        command_started = node._command_timer_callback_group.beginning_execution(
            node._command_timer
        )
        assert command_started
        heartbeat_started = node._heartbeat_callback_group.beginning_execution(
            node._heartbeat_timer
        )
        assert heartbeat_started
    finally:
        if heartbeat_started:
            node._heartbeat_callback_group.ending_execution(node._heartbeat_timer)
        if command_started:
            node._command_timer_callback_group.ending_execution(
                node._command_timer
            )
        if control_started:
            node._control_callback_group.ending_execution(
                node._cmd_vel_subscription
            )
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()
