"""Exact packed payload layouts shared with the ESP32 pi_bridge firmware."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


HELLO = struct.Struct("<B3xI32s32s")
TIME_SYNC_REQUEST = struct.Struct("<Q")
HEARTBEAT = struct.Struct("<QB7x")
ARM = struct.Struct("<IB3x")
CMD_VEL = struct.Struct("<iiIB")
RELAY_COMMAND = struct.Struct("<BB")
SET_LIMITS = struct.Struct("<iiiIIhhhhbbBBBHH")

HELLO_ACK = struct.Struct("<BBBBIHH32s")
TIME_SYNC_RESPONSE = struct.Struct("<QQQ")
STATUS = struct.Struct("<BBBBHHIIIIIhhhhIII")
IMU = struct.Struct("<Q10fB3BfI")
GNSS = struct.Struct("<QiiiIIiiBBBBII")
MOTOR_FEEDBACK = struct.Struct("<QhhhhHHII")
POWER_STATUS = struct.Struct("<QfB3BI")
RELAY_STATUS = struct.Struct("<QBBBBI")
FAULT_EVENT = struct.Struct("<QHBBI")
ESTOP_EVENT = struct.Struct("<QBBHI")
COMMAND_ACK = struct.Struct("<IBBBBHH")
DIAGNOSTICS = struct.Struct("<Q11I")


class PayloadError(ValueError):
    pass


def _fixed_ascii(value: str, width: int, field: str) -> bytes:
    try:
        encoded = value.encode("ascii", errors="strict")
    except UnicodeEncodeError as exc:
        raise PayloadError(f"{field} must contain ASCII only") from exc
    if len(encoded) > width:
        raise PayloadError(f"{field} exceeds {width} bytes")
    return encoded.ljust(width, b"\x00")


def _text(value: bytes, field: str) -> str:
    head, separator, tail = value.partition(b"\x00")
    if separator and any(tail):
        raise PayloadError(f"{field} contains data after its NUL terminator")
    try:
        return head.decode("ascii", errors="strict")
    except UnicodeDecodeError as exc:
        raise PayloadError(f"{field} is not ASCII") from exc


def _unpack(layout: struct.Struct, payload: bytes, name: str) -> tuple:
    if len(payload) != layout.size:
        raise PayloadError(f"{name} payload is {len(payload)} bytes, expected {layout.size}")
    return layout.unpack(payload)


def require_empty(payload: bytes, name: str) -> None:
    if payload:
        raise PayloadError(f"{name} requires an empty payload")


def pack_hello(version: int, capabilities: int, client_name: str, build_id: str) -> bytes:
    return HELLO.pack(
        version,
        capabilities,
        _fixed_ascii(client_name, 32, "client_name"),
        _fixed_ascii(build_id, 32, "build_id"),
    )


def pack_time_sync_request(pi_send_monotonic_us: int) -> bytes:
    return TIME_SYNC_REQUEST.pack(pi_send_monotonic_us)


def pack_heartbeat(pi_monotonic_us: int, pi_state: int) -> bytes:
    return HEARTBEAT.pack(pi_monotonic_us, pi_state)


def pack_arm(arm_nonce: int, requested_mode: int) -> bytes:
    return ARM.pack(arm_nonce, requested_mode)


def pack_cmd_vel(
    linear_mps: float,
    angular_rad_s: float,
    command_timeout_ms: int,
    control_mode: int,
) -> bytes:
    linear_mm_s = int(round(linear_mps * 1000.0))
    angular_mrad_s = int(round(angular_rad_s * 1000.0))
    return CMD_VEL.pack(linear_mm_s, angular_mrad_s, command_timeout_ms, control_mode)


def pack_relay_command(relay_mask: int, relay_values: int) -> bytes:
    if relay_mask & ~0x03 or relay_values & ~0x03:
        raise PayloadError("only relay bits 0 and 1 are defined")
    return RELAY_COMMAND.pack(relay_mask, relay_values)


def pack_set_limits(
    max_forward_mm_s: int,
    max_reverse_mm_s: int,
    max_angular_mrad_s: int,
    linear_accel_mm_s2: int,
    angular_accel_mrad_s2: int,
    left_scale_milli: int,
    right_scale_milli: int,
    linear_scale_milli: int,
    angular_scale_milli: int,
    left_sign: int,
    right_sign: int,
    swap_left_right: bool,
    motor_deadband_percent: int,
    max_motor_percent: int,
    track_width_mm: int,
    command_timeout_ms: int,
) -> bytes:
    """Serialize SET_LIMITS exactly; ESP32 still enforces compiled hard ceilings.

    The bridge deliberately does not call this during handshake because the
    repository contains TODO_CALIBRATE values. It is available for an explicit
    commissioning path once every value has been measured and reviewed.
    """
    try:
        return SET_LIMITS.pack(
            max_forward_mm_s,
            max_reverse_mm_s,
            max_angular_mrad_s,
            linear_accel_mm_s2,
            angular_accel_mrad_s2,
            left_scale_milli,
            right_scale_milli,
            linear_scale_milli,
            angular_scale_milli,
            left_sign,
            right_sign,
            int(swap_left_right),
            motor_deadband_percent,
            max_motor_percent,
            track_width_mm,
            command_timeout_ms,
        )
    except struct.error as exc:
        raise PayloadError(f"SET_LIMITS value is outside its wire field: {exc}") from exc


@dataclass(frozen=True)
class HelloAckPayload:
    version: int
    accepted: bool
    state: int
    capabilities: int
    max_payload: int
    firmware_build: str


def unpack_hello_ack(payload: bytes) -> HelloAckPayload:
    version, accepted, state, reserved, capabilities, max_payload, reserved2, build = _unpack(
        HELLO_ACK, payload, "HELLO_ACK"
    )
    if reserved != 0 or reserved2 != 0:
        raise PayloadError("HELLO_ACK reserved field is nonzero")
    if accepted not in (0, 1) or state > 5:
        raise PayloadError("HELLO_ACK boolean or state is invalid")
    return HelloAckPayload(
        version, bool(accepted), state, capabilities, max_payload,
        _text(build, "firmware_build_id"),
    )


@dataclass(frozen=True)
class TimeSyncResponsePayload:
    echoed_pi_send_us: int
    esp_receive_us: int
    esp_transmit_us: int


def unpack_time_sync_response(payload: bytes) -> TimeSyncResponsePayload:
    return TimeSyncResponsePayload(*_unpack(TIME_SYNC_RESPONSE, payload, "TIME_SYNC_RESPONSE"))


@dataclass(frozen=True)
class StatusPayload:
    state: int
    armed: bool
    estop: bool
    reset_reason: int
    fault_code: int
    cmd_age_ms: int
    heartbeat_age_ms: int
    gnss_age_ms: int
    imu_age_ms: int
    motor_age_ms: int
    applied_left: int
    applied_right: int
    uart_speed: int
    uart_steer: int
    watchdog_trips: int
    boot_counter: int
    uptime_ms: int


def unpack_status(payload: bytes) -> StatusPayload:
    values = _unpack(STATUS, payload, "STATUS")
    if values[5] != 0:
        raise PayloadError("STATUS reserved field is nonzero")
    if values[0] > 5 or values[1] not in (0, 1) or values[2] not in (0, 1):
        raise PayloadError("STATUS state or boolean is invalid")
    return StatusPayload(
        values[0], bool(values[1]), bool(values[2]), values[3], values[4], *values[6:]
    )


@dataclass(frozen=True)
class ImuPayload:
    sensor_monotonic_us: int
    quaternion_xyzw: tuple[float, float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    linear_acceleration_xyz: tuple[float, float, float]
    calibration_status: int
    accuracy_rad: float
    sequence: int


def unpack_imu(payload: bytes) -> ImuPayload:
    values = _unpack(IMU, payload, "IMU")
    floats = values[1:11]
    if any(values[index] != 0 for index in (12, 13, 14)):
        raise PayloadError("IMU reserved field is nonzero")
    if not all(math.isfinite(value) for value in floats) or not math.isfinite(values[15]):
        raise PayloadError("IMU contains non-finite values")
    return ImuPayload(values[0], tuple(floats[:4]), tuple(floats[4:7]), tuple(floats[7:]), values[11], values[15], values[16])


@dataclass(frozen=True)
class GnssPayload:
    sensor_monotonic_us: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    horizontal_accuracy_m: float
    vertical_accuracy_m: float
    ground_speed_mps: float
    motion_heading_rad: float
    fix_type: int
    carrier_solution: int
    satellites: int
    rtcm_age_ms: int
    sequence: int


def unpack_gnss(payload: bytes) -> GnssPayload:
    values = _unpack(GNSS, payload, "GNSS")
    reserved = values[11]
    if reserved != 0:
        raise PayloadError("GNSS reserved byte is nonzero")
    if values[8] > 5 or values[9] > 2:
        raise PayloadError("GNSS fix or carrier state is invalid")
    latitude = values[1] * 1.0e-7
    longitude = values[2] * 1.0e-7
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise PayloadError("GNSS coordinates are out of range")
    return GnssPayload(
        values[0], latitude, longitude, values[3] * 1.0e-3,
        values[4] * 1.0e-3, values[5] * 1.0e-3, values[6] * 1.0e-3,
        math.radians(values[7] * 1.0e-5), values[8], values[9], values[10], values[12], values[13]
    )


@dataclass(frozen=True)
class MotorFeedbackPayload:
    sensor_monotonic_us: int
    left: int
    right: int
    battery_voltage: float
    board_temperature_c: float
    fault: int
    valid_frames: int
    invalid_frames: int


def unpack_motor_feedback(payload: bytes) -> MotorFeedbackPayload:
    timestamp, left, right, battery_cv, temperature_deci_c, fault, reserved, valid, invalid = _unpack(
        MOTOR_FEEDBACK, payload, "MOTOR_FEEDBACK"
    )
    if reserved != 0:
        raise PayloadError("MOTOR_FEEDBACK reserved field is nonzero")
    return MotorFeedbackPayload(timestamp, left, right, battery_cv * 0.01, temperature_deci_c * 0.1, fault, valid, invalid)


@dataclass(frozen=True)
class PowerStatusPayload:
    sensor_monotonic_us: int
    voltage: float
    battery_percent: int
    flags: int


def unpack_power_status(payload: bytes) -> PowerStatusPayload:
    values = _unpack(POWER_STATUS, payload, "POWER_STATUS")
    if not math.isfinite(values[1]):
        raise PayloadError("POWER_STATUS voltage is non-finite")
    if any(values[index] != 0 for index in (3, 4, 5)):
        raise PayloadError("POWER_STATUS reserved field is nonzero")
    if values[2] > 100 and values[2] != 255:
        raise PayloadError("POWER_STATUS battery percent is invalid")
    return PowerStatusPayload(values[0], values[1], values[2], values[6])


@dataclass(frozen=True)
class RelayStatusPayload:
    sensor_monotonic_us: int
    attachment_active: bool
    mount_active: bool
    allowed_mask: int
    state: int
    sequence: int


def unpack_relay_status(payload: bytes) -> RelayStatusPayload:
    timestamp, attachment, mount, allowed, state, sequence = _unpack(RELAY_STATUS, payload, "RELAY_STATUS")
    if attachment not in (0, 1) or mount not in (0, 1) or allowed & ~0x03 or state > 5:
        raise PayloadError("RELAY_STATUS contains invalid boolean, mask or state")
    return RelayStatusPayload(timestamp, bool(attachment), bool(mount), allowed, state, sequence)


@dataclass(frozen=True)
class FaultEventPayload:
    event_monotonic_us: int
    fault_code: int
    from_state: int
    to_state: int
    occurrence_count: int


def unpack_fault_event(payload: bytes) -> FaultEventPayload:
    values = _unpack(FAULT_EVENT, payload, "FAULT_EVENT")
    if values[2] > 5 or values[3] > 5:
        raise PayloadError("FAULT_EVENT state is invalid")
    return FaultEventPayload(*values)


@dataclass(frozen=True)
class EstopEventPayload:
    event_monotonic_us: int
    latched: bool
    source: int
    occurrence_count: int


def unpack_estop_event(payload: bytes) -> EstopEventPayload:
    timestamp, latched, source, reserved, count = _unpack(ESTOP_EVENT, payload, "ESTOP_EVENT")
    if reserved != 0:
        raise PayloadError("ESTOP_EVENT reserved field is nonzero")
    if latched not in (0, 1):
        raise PayloadError("ESTOP_EVENT latched boolean is invalid")
    return EstopEventPayload(timestamp, bool(latched), source, count)


@dataclass(frozen=True)
class CommandAckPayload:
    request_sequence: int
    request_type: int
    result: int
    state: int
    detail: int


def unpack_command_ack(payload: bytes) -> CommandAckPayload:
    request_sequence, request_type, result, state, reserved, detail, reserved2 = _unpack(COMMAND_ACK, payload, "COMMAND_ACK")
    if reserved != 0 or reserved2 != 0:
        raise PayloadError("COMMAND_ACK reserved field is nonzero")
    if state > 5 or result > 8:
        raise PayloadError("COMMAND_ACK result or state is invalid")
    return CommandAckPayload(request_sequence, request_type, result, state, detail)


@dataclass(frozen=True)
class DiagnosticsPayload:
    sensor_monotonic_us: int
    rx_frames: int
    tx_frames: int
    crc_errors: int
    cobs_errors: int
    length_errors: int
    oversized_frames: int
    unknown_types: int
    duplicate_sequences: int
    out_of_order_sequences: int
    rx_overflows: int
    tx_errors: int


def unpack_diagnostics(payload: bytes) -> DiagnosticsPayload:
    return DiagnosticsPayload(*_unpack(DIAGNOSTICS, payload, "DIAGNOSTICS"))
