"""Hardware-free tests of the standalone stationary sensor diagnostic."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "test_line_edge.py"
spec = importlib.util.spec_from_file_location("standalone_line_edge", SCRIPT)
diagnostic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostic)


class LineEdgeTest(unittest.TestCase):
    def test_all_middle_pair_states(self):
        for byte, edge in [(7, "BLACK_LEFT"), (13, "BLACK_RIGHT"),
                           (5, "BOTH_BLACK"), (15, "BOTH_WHITE")]:
            with self.subTest(edge=edge):
                self.assertEqual(diagnostic.decode_frame([byte])[3], edge)

    def test_polarity_and_channel_order(self):
        self.assertEqual(diagnostic.decode_frame([8], black_value=1)[3], "BLACK_LEFT")
        self.assertEqual(diagnostic.decode_frame([11], bits=(3, 2, 1, 0))[3], "BLACK_LEFT")

    def test_bad_frames_fail(self):
        for frame in ([], [1, 2], [256], [-1], [True], ["7"]):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                diagnostic.decode_frame(frame)

    def test_physical_path_only_reads_sensor_register_and_closes(self):
        class ReadOnlyBus:
            def __init__(self):
                self.reads = []
                self.closed = False
            def read_i2c_block_data(self, address, register, count):
                self.reads.append((address, register, count))
                return [7]
            def close(self):
                self.closed = True
            # There is intentionally no write method.
        bus = ReadOnlyBus()
        output = io.StringIO()
        module = SimpleNamespace(SMBus=lambda number: bus)
        with patch.dict(sys.modules, {"smbus2": module}), \
             patch.object(diagnostic.time, "sleep"), contextlib.redirect_stdout(output):
            self.assertEqual(diagnostic.main(["--samples", "3"]), 0)
        self.assertEqual(bus.reads, [(0x2B, 0x0A, 1)] * 3)
        self.assertTrue(bus.closed)
        self.assertIn("EDGE_OK", output.getvalue())

    def test_sensor_failure_is_reported_and_bus_closed(self):
        class FailedBus:
            closed = False
            def read_i2c_block_data(self, *args):
                raise OSError("no controller")
            def close(self):
                self.closed = True
        bus = FailedBus()
        with patch.dict(sys.modules, {"smbus2": SimpleNamespace(SMBus=lambda n: bus)}), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(diagnostic.main(["--samples", "1"]), 1)
        self.assertTrue(bus.closed)

    def test_demo_shows_target_and_ambiguity_without_importing_hardware(self):
        output = io.StringIO()
        with patch.dict(sys.modules, {"smbus2": None}), \
             patch.object(diagnostic.time, "sleep"), contextlib.redirect_stdout(output):
            self.assertEqual(diagnostic.main(["--demo", "--edge", "black-right", "--samples", "12"]), 0)
        for state in ("EDGE_OK", "AMBIGUOUS", "OTHER_EDGE"):
            self.assertIn(state, output.getvalue())


if __name__ == "__main__":
    unittest.main()
