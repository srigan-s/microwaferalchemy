#!/usr/bin/env python3
"""Stationary Yahboom Raspbot V2 tape-edge test. Never commands a motor.

Pi:  python3 scripts/test_line_edge.py --edge black-left
Mac: python3 scripts/test_line_edge.py --demo --samples 12

Protocol comes from the existing vendor/yahboom/project_demo Raspbot_Lib.py
and four-way line patrol notebooks: bus 1, address 0x2B, sensor register 0x0A.
The notebooks suggest S1..S4 bit order 2,3,1,0 and raw 0 = black. Confirm the
order by covering each sensor individually; both settings can be overridden.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Sequence

SENSOR_REGISTER = 0x0A
DEFAULT_BITS = (2, 3, 1, 0)


def parse_bits(text: str) -> tuple[int, ...]:
    try:
        bits = tuple(int(part.strip()) for part in text.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("use four bit numbers, e.g. 2,3,1,0") from exc
    if sorted(bits) != [0, 1, 2, 3]:
        raise argparse.ArgumentTypeError("bits must contain 0,1,2,3 exactly once")
    return bits


def decode_frame(
    frame: Sequence[int],
    bits: tuple[int, ...] = DEFAULT_BITS,
    black_value: int = 0,
) -> tuple[int, tuple[int, ...], tuple[int, ...], str]:
    """Return packed byte, raw S1..S4, normalized S1..S4 and middle-pair edge."""
    if len(frame) != 1 or type(frame[0]) is not int or not 0 <= frame[0] <= 255:
        raise ValueError(f"expected one sensor byte, received {frame!r}")
    if sorted(bits) != [0, 1, 2, 3] or black_value not in (0, 1):
        raise ValueError("invalid sensor bit order or polarity")
    packed = frame[0]
    raw = tuple((packed >> bit) & 1 for bit in bits)
    normalized = tuple(int(value == black_value) for value in raw)
    edge = {
        (1, 0): "BLACK_LEFT",
        (0, 1): "BLACK_RIGHT",
        (1, 1): "BOTH_BLACK",
        (0, 0): "BOTH_WHITE",
    }[normalized[1:3]]
    return packed, raw, normalized, edge


def read_frame(bus, address: int) -> list[int]:
    # No motor registers, motor imports, arming, or motor writes in this script.
    return bus.read_i2c_block_data(address, SENSOR_REGISTER, 1)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--edge", choices=("black-left", "black-right"), default="black-left")
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x2B)
    parser.add_argument("--bits", type=parse_bits, default=DEFAULT_BITS,
                        help="physical S1,S2,S3,S4 bit order (default 2,3,1,0)")
    parser.add_argument("--black-value", type=int, choices=(0, 1), default=0,
                        help="raw value on black tape (vendor default: 0)")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--stable-samples", type=int, default=3)
    parser.add_argument("--samples", type=int, default=0,
                        help="number of readings; 0 runs until Ctrl+C")
    parser.add_argument("--demo", action="store_true",
                        help="synthetic readings for testing on a Mac; no hardware")
    args = parser.parse_args(argv)
    if args.bus < 0 or not 0x03 <= args.address <= 0x77:
        parser.error("use a nonnegative bus and a 7-bit address between 0x03 and 0x77")
    if not math.isfinite(args.hz) or not 0 < args.hz <= 100:
        parser.error("--hz must be finite and between 0 and 100")
    if args.stable_samples < 1 or args.samples < 0:
        parser.error("--stable-samples must be positive; --samples must be nonnegative")

    target = args.edge.upper().replace("-", "_")
    bus = None
    try:
        if not args.demo:
            try:
                from smbus2 import SMBus
            except ImportError:
                print("Install the sensor dependency: python3 -m pip install smbus2", file=sys.stderr)
                return 1
            bus = SMBus(args.bus)
        print(f"{'DEMO (synthetic)' if args.demo else 'PHYSICAL sensor read'} | target={target}")
        print("Stationary test: this script does not command motors. Move tape by hand.")
        print("Normalized: BLACK=1, WHITE=0. S1..S4 are left to right, viewed from behind.")
        print("time     byte  raw S1 S2 S3 S4 | normalized S1 S2 S3 S4 | edge         | status", flush=True)
        started = time.monotonic()
        previous_edge = None
        consecutive = 0
        index = 0
        while args.samples == 0 or index < args.samples:
            cycle = time.monotonic()
            if args.demo:
                # Show the requested edge first, then each other middle-pair state.
                pair = (1, 0) if target == "BLACK_LEFT" else (0, 1)
                pairs = (pair, (1, 1), (0, 0), tuple(reversed(pair)))
                s2, s3 = pairs[(index // args.stable_samples) % len(pairs)]
                black = (0, s2, s3, 0)
                packed = sum((args.black_value if value else 1 - args.black_value) << bit
                             for value, bit in zip(black, args.bits))
                frame = [packed]
            else:
                frame = read_frame(bus, args.address)
            packed, raw, normalized, edge = decode_frame(frame, args.bits, args.black_value)
            consecutive = consecutive + 1 if edge == previous_edge else 1
            previous_edge = edge
            if edge in ("BOTH_BLACK", "BOTH_WHITE"):
                status = "AMBIGUOUS — move tape slightly"
            elif consecutive < args.stable_samples:
                status = f"SETTLING {consecutive}/{args.stable_samples}"
            else:
                status = "EDGE_OK" if edge == target else "OTHER_EDGE"
            print(f"{time.monotonic() - started:6.2f}s  0x{packed:02X}  "
                  f"{' '.join(map(str, raw))}          | {' '.join(map(str, normalized))}                 "
                  f"| {edge:12} | {status}", flush=True)
            index += 1
            if args.samples == 0 or index < args.samples:
                time.sleep(max(0.0, 1.0 / args.hz - (time.monotonic() - cycle)))
        return 0
    except KeyboardInterrupt:
        print("\nSensor test stopped.")
        return 0
    except (OSError, ValueError) as exc:
        print(f"Sensor test failed: {exc}\nCheck I2C is enabled, bus/address, and board power.", file=sys.stderr)
        return 1
    finally:
        if bus is not None:
            bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
