from module_robot_esp32_bridge.bridge_state import CommandGate, ReconnectBackoff


def test_nonzero_is_blocked_until_armed_confirmation():
    gate = CommandGate(stale_after_s=0.25)
    gate.update(1.0, 0.1, 0.2)
    assert gate.output(1.1) == (0.0, 0.0)
    gate.armed_confirmed = True
    assert gate.output(1.1) == (0.1, 0.2)


def test_stale_ros_command_becomes_zero():
    gate = CommandGate(stale_after_s=0.25, armed_confirmed=True)
    gate.update(1.0, 0.1, 0.0)
    assert gate.output(1.25) == (0.1, 0.0)
    assert gate.output(1.250001) == (0.0, 0.0)


def test_disconnect_invalidates_command():
    gate = CommandGate(stale_after_s=0.25, armed_confirmed=True)
    gate.update(1.0, 0.1, 0.0)
    gate.armed_confirmed = False
    gate.invalidate()
    assert gate.output(1.01) == (0.0, 0.0)


def test_reconnect_delay():
    backoff = ReconnectBackoff(2.0)
    backoff.failed(10.0)
    assert not backoff.ready(11.99)
    assert backoff.ready(12.0)
