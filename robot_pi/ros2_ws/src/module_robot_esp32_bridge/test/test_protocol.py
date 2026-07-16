import itertools
import json
from pathlib import Path

import pytest

from module_robot_esp32_bridge.protocol import (
    CRC,
    HEADER,
    MAX_ENCODED_FRAME_LENGTH,
    FrameDecoder,
    MessageType,
    cobs_decode,
    cobs_encode,
    crc16_ccitt_false,
    decode_frame,
    encode_frame,
    sequence_is_newer,
)


@pytest.mark.parametrize(
    "raw",
    [b"", b"\x00", b"abc", b"a\x00b", bytes(range(128)), b"\xff" * 190],
)
def test_cobs_round_trip(raw):
    encoded = cobs_encode(raw)
    assert b"\x00" not in encoded
    assert cobs_decode(encoded) == raw


def test_crc_standard_check_vector():
    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_frame_round_trip():
    wire = encode_frame(MessageType.CMD_VEL, 0xAABBCCDD, 0x0102030405060708, b"\x00payload")
    frame = decode_frame(wire[:-1])
    assert frame.message_type == MessageType.CMD_VEL
    assert frame.sequence == 0xAABBCCDD
    assert frame.sender_monotonic_us == 0x0102030405060708
    assert frame.payload == b"\x00payload"


def test_fragmented_frame():
    wire = encode_frame(MessageType.HELLO_ACK, 7, 42, b"ack")
    decoder = FrameDecoder()
    result = list(itertools.chain.from_iterable(decoder.feed(bytes([value])) for value in wire))
    assert len(result) == 1
    assert result[0].payload == b"ack"


def test_multiple_frames_in_one_read():
    data = encode_frame(MessageType.STATUS, 1, 1) + encode_frame(MessageType.IMU, 2, 2)
    frames = FrameDecoder().feed(data)
    assert [frame.message_type for frame in frames] == [MessageType.STATUS, MessageType.IMU]


def test_corrupted_frame_is_not_emitted():
    wire = bytearray(encode_frame(MessageType.STATUS, 1, 1, b"status"))
    wire[-3] ^= 0x55
    decoder = FrameDecoder()
    assert decoder.feed(wire) == []
    assert decoder.stats.crc_errors + decoder.stats.cobs_errors == 1


def test_oversized_fragment_is_discarded_until_delimiter():
    decoder = FrameDecoder()
    assert decoder.feed(b"\x01" * (MAX_ENCODED_FRAME_LENGTH + 20)) == []
    assert decoder.feed(b"\x00") == []
    assert decoder.stats.oversized_frames == 1
    good = encode_frame(MessageType.STATUS, 2, 3)
    assert len(decoder.feed(good)) == 1


def test_declared_payload_length_mismatch_is_rejected():
    # A CRC-valid packet with an inconsistent bounded length must still never
    # escape the stream decoder as a Frame.
    body = HEADER.pack(1, int(MessageType.STATUS), 2, 7, 11) + b"x"
    wire = cobs_encode(body + CRC.pack(crc16_ccitt_false(body))) + b"\x00"
    decoder = FrameDecoder()
    assert decoder.feed(wire) == []
    assert decoder.stats.length_errors == 1


def test_unknown_message_type_is_counted_and_dropped():
    body = HEADER.pack(1, 0x7F, 0, 8, 12)
    wire = cobs_encode(body + CRC.pack(crc16_ccitt_false(body))) + b"\x00"
    decoder = FrameDecoder()
    assert decoder.feed(wire) == []
    assert decoder.stats.unknown_message_types == 1


def test_sequence_wrap_and_reject_duplicate_or_old():
    assert sequence_is_newer(0, 0xFFFFFFFF)
    assert sequence_is_newer(10, 9)
    assert not sequence_is_newer(9, 10)
    assert not sequence_is_newer(10, 10)
    assert not sequence_is_newer(0x80000000, 0)


def test_shared_firmware_golden_vectors():
    vectors = json.loads((Path(__file__).parent / "data" / "golden_vectors.json").read_text(encoding="utf-8"))
    crc_vector = vectors["crc16_ccitt_false"]
    assert crc16_ccitt_false(crc_vector["input_ascii"].encode("ascii")) == int(crc_vector["crc_hex"], 16)
    for vector in vectors["frames"]:
        wire = encode_frame(
            vector["message_type"],
            vector["sequence"],
            vector["sender_monotonic_us"],
            bytes.fromhex(vector["payload_hex"]),
        )
        assert wire.hex() == vector["wire_hex"]
        assert CRC.unpack(cobs_decode(wire[:-1])[-CRC.size :])[0] == int(vector["crc_hex"], 16)
