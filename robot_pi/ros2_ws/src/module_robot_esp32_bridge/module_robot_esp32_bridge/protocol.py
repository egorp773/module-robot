"""Version 1 of the bounded ESP32 serial wire protocol.

The wire representation is COBS(header || payload || CRC16) followed by a zero
delimiter.  All integers are little-endian.  This module deliberately has no
ROS dependency so the exact same vectors can be tested on a development host.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Iterable, Optional


PROTOCOL_VERSION = 1
MAX_PAYLOAD_LENGTH = 192
HEADER = struct.Struct("<BBHIQ")
CRC = struct.Struct("<H")
MIN_RAW_FRAME_LENGTH = HEADER.size + CRC.size
# COBS adds at most one code byte for each 254 raw bytes plus the initial byte.
MAX_RAW_FRAME_LENGTH = HEADER.size + MAX_PAYLOAD_LENGTH + CRC.size
MAX_ENCODED_FRAME_LENGTH = MAX_RAW_FRAME_LENGTH + (MAX_RAW_FRAME_LENGTH // 254) + 1


class MessageType(IntEnum):
    HELLO = 0x01
    TIME_SYNC_REQUEST = 0x02
    HEARTBEAT = 0x03
    ARM = 0x04
    DISARM = 0x05
    CMD_VEL = 0x06
    STOP = 0x07
    ESTOP = 0x08
    RESET_ESTOP = 0x09
    RESET_FAULT = 0x0A
    RELAY_COMMAND = 0x0B
    SET_LIMITS = 0x0C
    REQUEST_STATUS = 0x0D

    HELLO_ACK = 0x81
    TIME_SYNC_RESPONSE = 0x82
    STATUS = 0x83
    IMU = 0x84
    GNSS = 0x85
    MOTOR_FEEDBACK = 0x86
    POWER_STATUS = 0x87
    RELAY_STATUS = 0x88
    FAULT_EVENT = 0x89
    ESTOP_EVENT = 0x8A
    COMMAND_ACK = 0x8B
    DIAGNOSTICS = 0x8C


KNOWN_MESSAGE_TYPES = frozenset(int(item) for item in MessageType)


class FrameError(ValueError):
    """Base class for a rejected frame."""


class CobsError(FrameError):
    pass


class CrcError(FrameError):
    pass


class LengthError(FrameError):
    pass


class VersionError(FrameError):
    pass


class UnknownMessageError(FrameError):
    pass


@dataclass(frozen=True)
class Frame:
    protocol_version: int
    message_type: MessageType
    sequence: int
    sender_monotonic_us: int
    payload: bytes


@dataclass
class DecoderStats:
    valid_frames: int = 0
    empty_delimiters: int = 0
    cobs_errors: int = 0
    crc_errors: int = 0
    length_errors: int = 0
    version_errors: int = 0
    unknown_message_types: int = 0
    oversized_frames: int = 0


def crc16_ccitt_false(data: bytes | bytearray | memoryview) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff, xorout 0."""
    crc = 0xFFFF
    for octet in data:
        crc ^= octet << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(raw: bytes) -> bytes:
    """Encode one non-delimited COBS frame without allocations proportional to values."""
    output = bytearray(1)
    code_index = 0
    code = 1
    for value in raw:
        if value == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(value)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(encoded: bytes) -> bytes:
    """Decode a single COBS frame and reject zero/truncated code blocks."""
    if not encoded:
        raise CobsError("empty COBS frame")
    output = bytearray()
    index = 0
    encoded_length = len(encoded)
    while index < encoded_length:
        code = encoded[index]
        if code == 0:
            raise CobsError("zero byte inside COBS frame")
        index += 1
        block_end = index + code - 1
        if block_end > encoded_length:
            raise CobsError("COBS block exceeds frame")
        output.extend(encoded[index:block_end])
        index = block_end
        if code != 0xFF and index < encoded_length:
            output.append(0)
        if len(output) > MAX_RAW_FRAME_LENGTH:
            raise LengthError("decoded frame exceeds maximum")
    return bytes(output)


def encode_frame(
    message_type: MessageType | int,
    sequence: int,
    sender_monotonic_us: int,
    payload: bytes = b"",
    protocol_version: int = PROTOCOL_VERSION,
) -> bytes:
    """Return a complete delimited wire frame."""
    if not 0 <= protocol_version <= 0xFF:
        raise ValueError("protocol_version does not fit uint8")
    message_value = int(message_type)
    if message_value not in KNOWN_MESSAGE_TYPES:
        raise ValueError("unknown message_type")
    if len(payload) > MAX_PAYLOAD_LENGTH:
        raise ValueError("payload exceeds maximum")
    if not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError("sequence does not fit uint32")
    if not 0 <= sender_monotonic_us <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("sender_monotonic_us does not fit uint64")
    header = HEADER.pack(protocol_version, message_value, len(payload), sequence, sender_monotonic_us)
    body = header + payload
    return cobs_encode(body + CRC.pack(crc16_ccitt_false(body))) + b"\x00"


def decode_frame(encoded: bytes, expected_version: int = PROTOCOL_VERSION) -> Frame:
    """Validate and decode one frame without its trailing delimiter."""
    raw = cobs_decode(encoded)
    if len(raw) < MIN_RAW_FRAME_LENGTH:
        raise LengthError("frame shorter than header and CRC")
    version, message_value, payload_length, sequence, sender_us = HEADER.unpack_from(raw)
    if payload_length > MAX_PAYLOAD_LENGTH:
        raise LengthError("declared payload exceeds maximum")
    expected_length = HEADER.size + payload_length + CRC.size
    if len(raw) != expected_length:
        raise LengthError("declared payload length does not match frame")
    expected_crc = CRC.unpack_from(raw, len(raw) - CRC.size)[0]
    actual_crc = crc16_ccitt_false(raw[:-CRC.size])
    if actual_crc != expected_crc:
        raise CrcError(f"CRC mismatch: received 0x{expected_crc:04x}")
    if version != expected_version:
        raise VersionError(f"protocol version {version} is not {expected_version}")
    try:
        message_type = MessageType(message_value)
    except ValueError as exc:
        raise UnknownMessageError(f"unknown message type 0x{message_value:02x}") from exc
    payload = raw[HEADER.size : HEADER.size + payload_length]
    return Frame(version, message_type, sequence, sender_us, payload)


def sequence_is_newer(candidate: int, previous: Optional[int]) -> bool:
    """RFC-1982 style uint32 comparison, including wraparound."""
    if previous is None:
        return True
    delta = (candidate - previous) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


class FrameDecoder:
    """Bounded delimiter-oriented stream parser.

    On overflow bytes are discarded until the next delimiter.  A malformed
    packet never escapes as a Frame and therefore cannot refresh watchdogs.
    """

    def __init__(self, expected_version: int = PROTOCOL_VERSION) -> None:
        self.expected_version = expected_version
        self.stats = DecoderStats()
        self._buffer = bytearray()
        self._discard_until_delimiter = False

    def reset(self) -> None:
        self._buffer.clear()
        self._discard_until_delimiter = False

    def feed(self, data: bytes | bytearray | memoryview) -> list[Frame]:
        frames: list[Frame] = []
        for value in data:
            if value != 0:
                if self._discard_until_delimiter:
                    continue
                if len(self._buffer) >= MAX_ENCODED_FRAME_LENGTH:
                    self._buffer.clear()
                    self._discard_until_delimiter = True
                    self.stats.oversized_frames += 1
                else:
                    self._buffer.append(value)
                continue

            if self._discard_until_delimiter:
                self._discard_until_delimiter = False
                continue
            if not self._buffer:
                self.stats.empty_delimiters += 1
                continue
            encoded = bytes(self._buffer)
            self._buffer.clear()
            try:
                frame = decode_frame(encoded, self.expected_version)
            except CrcError:
                self.stats.crc_errors += 1
            except VersionError:
                self.stats.version_errors += 1
            except UnknownMessageError:
                self.stats.unknown_message_types += 1
            except LengthError:
                self.stats.length_errors += 1
            except CobsError:
                self.stats.cobs_errors += 1
            else:
                self.stats.valid_frames += 1
                frames.append(frame)
        return frames


def frames_from_chunks(chunks: Iterable[bytes]) -> list[Frame]:
    decoder = FrameDecoder()
    frames: list[Frame] = []
    for chunk in chunks:
        frames.extend(decoder.feed(chunk))
    return frames
