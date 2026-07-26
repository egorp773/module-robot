#!/usr/bin/env python3
"""Wake the hoverboard controller through a bounded ESP32 zero-first reset."""

from __future__ import annotations

import argparse
import os
import stat
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Hold ESP32 EN low briefly, then boot pi_bridge with IO0 high. "
            "Run only while no ROS bridge owns the serial device."
        )
    )
    parser.add_argument("--device", default="/dev/module-esp32")
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    args = parser.parse_args()
    if not 1.0 <= args.hold_seconds <= 5.0:
        parser.error("--hold-seconds must be in [1, 5]")

    device_stat = os.stat(args.device)
    if not stat.S_ISCHR(device_stat.st_mode):
        parser.error(f"serial device is not a character device: {args.device}")

    port = serial.Serial(
        args.device,
        460800,
        timeout=0.1,
        write_timeout=0.5,
        exclusive=True,
    )
    try:
        # On the deployed CH340 auto-reset circuit RTS drives EN low. DTR is
        # held false so IO0 remains high and the application, not the ROM
        # bootloader, starts when EN is released.
        port.setDTR(False)
        port.setRTS(True)
        time.sleep(args.hold_seconds)
        port.setRTS(False)
        time.sleep(0.2)
    finally:
        # Release reset even if sleep or a serial operation is interrupted.
        try:
            port.setRTS(False)
            port.setDTR(False)
        finally:
            port.close()

    print(
        f"motor wake reset completed on {args.device}: "
        f"EN low {args.hold_seconds:.1f}s, IO0 high"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
