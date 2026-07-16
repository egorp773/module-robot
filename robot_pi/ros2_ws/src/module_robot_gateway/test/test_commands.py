import pytest

from module_robot_gateway.commands import (
    CommandError,
    ManualDrive,
    NavigationCommand,
    Ping,
    RouteBegin,
    RouteEnd,
    RouteWaypoint,
    Stop,
    parse_command,
)


def test_ping_is_a_closed_zero_motion_command():
    assert isinstance(parse_command("PING"), Ping)
    with pytest.raises(CommandError):
        parse_command("PING,extra")


def test_manual_command_is_strict_and_finite():
    assert parse_command("M,-25,50") == ManualDrive(-25.0, 50.0)
    with pytest.raises(CommandError) as error:
        parse_command("M,nan,0")
    assert error.value.response() == "ERR,M,INVALID_LEFT"
    with pytest.raises(CommandError):
        parse_command("M,1,2,3")


def test_route_begin_required_and_legacy_forms():
    assert parse_command("ROUTE_BEGIN,3") == RouteBegin(3)
    legacy = parse_command("ROUTE_BEGIN,2,55.751244,37.618423")
    assert legacy == RouteBegin(2, 55.751244, 37.618423)
    assert legacy.legacy_local_mode


def test_waypoint_and_end():
    assert parse_command("ROUTE_WP,7,55.0,37.0") == RouteWaypoint(
        7, 55.0, 37.0
    )
    assert isinstance(parse_command("ROUTE_END"), RouteEnd)


@pytest.mark.parametrize(
    "line,expected_type",
    [
        ("STOP", Stop),
        ("NAV_START", NavigationCommand),
        ("NAV_PAUSE", NavigationCommand),
        ("NAV_RESUME", NavigationCommand),
        ("NAV_STOP", NavigationCommand),
    ],
)
def test_control_commands(line, expected_type):
    assert isinstance(parse_command(line), expected_type)


@pytest.mark.parametrize(
    "line",
    [
        "",
        "UNKNOWN",
        "stop",
        "STOP,extra",
        "ROUTE_BEGIN,1,55",
        "ROUTE_WP,1.5,55,37",
        "ROUTE_WP,0,inf,37",
        "NAV_START\nSTOP",
    ],
)
def test_malformed_commands_are_rejected(line):
    with pytest.raises(CommandError):
        parse_command(line)


def test_encoded_message_limit_is_bytes_not_characters():
    with pytest.raises(CommandError) as error:
        parse_command("Я" * 20, max_message_bytes=20)
    assert error.value.code == "MESSAGE_TOO_LARGE"
