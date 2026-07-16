#!/usr/bin/env python3
"""Non-motion serial commissioning check for the ESP32 pi_bridge image.

Despite the historical filename, this is an end-to-end protocol round trip,
not an electrical TX/RX wire loopback. It performs HELLO and REQUEST_STATUS,
requires DISARMED/hard-zero, and sends STOP+DISARM before closing. It never
sends ARM, CMD_VEL, ESTOP reset, fault reset or relay commands.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


SCRIPT_DIR = Path(__file__).resolve().parent
BRIDGE_SOURCE = SCRIPT_DIR.parent / "ros2_ws" / "src" / "module_robot_esp32_bridge"
sys.path.insert(0, str(BRIDGE_SOURCE))

try:
    import serial
    from module_robot_esp32_bridge.payloads import pack_hello, unpack_hello_ack, unpack_status
    from module_robot_esp32_bridge.protocol import (
        FrameDecoder,
        MessageType,
        PROTOCOL_VERSION,
        decode_frame,
        encode_frame,
    )
except ImportError as exc:  # pragma: no cover - environment-specific message
    raise SystemExit(
        f"Missing dependency ({exc}). Run install_pi.sh or install requirements.txt."
    ) from exc


DISARMED = 2


def monotonic_us() -> int:
    return time.monotonic_ns() // 1_000


def next_sequence(value: int) -> int:
    return (value + 1) & 0xFFFFFFFF


def write_frame(port: serial.Serial, message_type: MessageType, sequence: int, payload: bytes = b"") -> None:
    port.write(encode_frame(message_type, sequence, monotonic_us(), payload))
    port.flush()


def receive_until(port: serial.Serial, decoder: FrameDecoder, wanted: set[MessageType], deadline: float):
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if not chunk:
            continue
        for frame in decoder.feed(chunk):
            if frame.message_type in wanted:
                return frame
    names = ", ".join(item.name for item in sorted(wanted, key=int))
    raise TimeoutError(f"timed out waiting for {names}")


def self_test() -> None:
    payload = pack_hello(PROTOCOL_VERSION, 0, "serial-loopback", "self-test")
    wire = encode_frame(MessageType.HELLO, 0xFFFFFFFE, 0x0102030405060708, payload)
    frame = decode_frame(wire[:-1])
    if frame.message_type != MessageType.HELLO or frame.payload != payload:
        raise RuntimeError("in-memory encode/decode mismatch")
    print("Protocol in-memory round trip: PASS")


def run(args: argparse.Namespace) -> None:
    decoder = FrameDecoder(expected_version=args.protocol_version)
    sequence = int(time.monotonic_ns()) & 0xFFFFFFFF
    hello_seen = False
    port = serial.Serial(
        port=args.device,
        baudrate=args.baud,
        timeout=0.1,
        write_timeout=0.5,
        exclusive=True,
    )
    try:
        port.reset_input_buffer()
        hello_payload = pack_hello(
            args.protocol_version,
            0,
            "serial-loopback",
            args.build_id,
        )
        write_frame(port, MessageType.HELLO, sequence, hello_payload)
        sequence = next_sequence(sequence)
        ack_frame = receive_until(
            port,
            decoder,
            {MessageType.HELLO_ACK},
            time.monotonic() + args.timeout,
        )
        ack = unpack_hello_ack(ack_frame.payload)
        if not ack.accepted:
            raise RuntimeError("ESP32 rejected HELLO")
        if ack.version != args.protocol_version:
            raise RuntimeError(f"negotiated protocol {ack.version}, expected {args.protocol_version}")
        if ack.state != DISARMED:
            raise RuntimeError(f"HELLO_ACK state is {ack.state}, expected DISARMED ({DISARMED})")
        hello_seen = True
        print(
            f"HELLO_ACK: PASS, firmware={ack.firmware_build!r}, "
            f"state=DISARMED, max_payload={ack.max_payload}"
        )

        write_frame(port, MessageType.REQUEST_STATUS, sequence)
        sequence = next_sequence(sequence)
        status_frame = receive_until(
            port,
            decoder,
            {MessageType.STATUS},
            time.monotonic() + args.timeout,
        )
        status = unpack_status(status_frame.payload)
        if status.state != DISARMED or status.armed or status.estop:
            raise RuntimeError(
                f"unsafe status: state={status.state}, armed={status.armed}, estop={status.estop}"
            )
        if any((status.applied_left, status.applied_right, status.uart_speed, status.uart_steer)):
            raise RuntimeError(
                "non-zero output in DISARMED: "
                f"left={status.applied_left}, right={status.applied_right}, "
                f"speed={status.uart_speed}, steer={status.uart_steer}"
            )
        print(
            "REQUEST_STATUS: PASS, hard-zero confirmed; "
            f"watchdog_trips={status.watchdog_trips}, boot_counter={status.boot_counter}"
        )
        print(f"Decoder stats: {decoder.stats}")
    finally:
        # Both messages can only remove motion authority. They are best effort
        # because unplug/error paths must still close promptly.
        if hello_seen and port.is_open:
            try:
                write_frame(port, MessageType.STOP, sequence)
                sequence = next_sequence(sequence)
                write_frame(port, MessageType.DISARM, sequence)
            except (OSError, serial.SerialException):
                pass
        port.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/module-esp32")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--protocol-version", type=int, default=PROTOCOL_VERSION)
    parser.add_argument("--build-id", default="manual-commissioning")
    parser.add_argument("--self-test", action="store_true", help="only test in-memory framing")
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    if arguments.self_test:
        self_test()
    else:
        try:
            run(arguments)
        except (OSError, TimeoutError, RuntimeError, serial.SerialException) as exc:
            raise SystemExit(f"serial protocol check FAILED: {exc}") from exc
