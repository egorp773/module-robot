import pytest

from module_robot_esp32_bridge.payloads import (
    COMMAND_ACK,
    HELLO_ACK,
    IMU,
    POWER_STATUS,
    STATUS,
    PayloadError,
    pack_set_limits,
    unpack_command_ack,
    unpack_hello_ack,
    unpack_imu,
    unpack_power_status,
    unpack_status,
)


def test_wire_layout_sizes_match_firmware_static_asserts():
    assert HELLO_ACK.size == 44
    assert STATUS.size == 48
    assert IMU.size == 60
    assert POWER_STATUS.size == 20
    assert COMMAND_ACK.size == 12
    assert len(
        pack_set_limits(
            150, 150, 600, 100, 200, 1000, 1000, 1000, 1000,
            1, 1, False, 12, 30, 380, 300,
        )
    ) == 37


def test_hello_ack_rejects_nonzero_reserved_bytes():
    good = HELLO_ACK.pack(1, 1, 2, 0, 0x1F, 192, 0, b"pi-bridge\0".ljust(32, b"\0"))
    assert unpack_hello_ack(good).accepted
    bad = HELLO_ACK.pack(1, 1, 2, 0, 0x1F, 192, 1, b"pi-bridge\0".ljust(32, b"\0"))
    with pytest.raises(PayloadError):
        unpack_hello_ack(bad)


def test_status_rejects_nonzero_reserved_field():
    values = (2, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    with pytest.raises(PayloadError):
        unpack_status(STATUS.pack(*values))


def test_imu_rejects_nonzero_reserved_byte():
    values = [1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81,
              3, 0, 1, 0, 0.1, 1]
    with pytest.raises(PayloadError):
        unpack_imu(IMU.pack(*values))


def test_power_and_ack_reject_nonzero_reserved_fields():
    with pytest.raises(PayloadError):
        unpack_power_status(POWER_STATUS.pack(1, 40.0, 255, 0, 1, 0, 1))
    with pytest.raises(PayloadError):
        unpack_command_ack(COMMAND_ACK.pack(1, 4, 0, 3, 1, 0, 0))

