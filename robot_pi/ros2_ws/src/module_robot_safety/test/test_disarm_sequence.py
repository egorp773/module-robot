import threading
from types import SimpleNamespace

from module_robot_msgs.srv import Disarm

from module_robot_safety.safety_node import SafetyNode


class DisarmHarness:
    def __init__(self, bridge_response):
        self._lock = threading.Lock()
        self._bridge_disarm = object()
        self._bridge_response = bridge_response
        self.events = []

    def _clear_motion_authority_locked(self):
        self.events.append("clear_authority")

    def _publish_zero_immediately(self):
        self.events.append("publish_zero")

    def _call_bridge(self, client, request):
        assert client is self._bridge_disarm
        assert isinstance(request, Disarm.Request)
        self.events.append("direct_disarm")
        return self._bridge_response

    def _request_bridge_zero(self, *, disarm):
        self.events.append(("fail_safe_zero", disarm))


def test_disarm_calls_bridge_disarm_directly_without_preliminary_stop():
    harness = DisarmHarness(SimpleNamespace(success=True))

    response = SafetyNode._on_disarm(
        harness, Disarm.Request(), Disarm.Response()
    )

    assert response.success
    assert harness.events == [
        "clear_authority",
        "publish_zero",
        "direct_disarm",
    ]


def test_disarm_requests_fail_safe_stop_and_disarm_only_after_missing_ack():
    harness = DisarmHarness(None)

    response = SafetyNode._on_disarm(
        harness, Disarm.Request(), Disarm.Response()
    )

    assert not response.success
    assert harness.events == [
        "clear_authority",
        "publish_zero",
        "direct_disarm",
        ("fail_safe_zero", True),
    ]
