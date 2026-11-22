#!/usr/bin/env python3
"""
Tests for TelemetryLogger — CSV flight logging with timestamps.
"""

import csv
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

# Stub cv2 before any source module that transitively imports it is loaded;
# src/utils/__init__.py re-exports VideoRecorder which pulls in OpenCV.
# The telemetry logger itself has no cv2 dependency.
class _FakeCV2:
    def __getattr__(self, name):
        return None
    def __call__(self, *a, **kw):
        return None
sys.modules.setdefault('cv2', _FakeCV2())

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.telemetry_logger import TelemetryLogger


class TestTelemetryLogger(unittest.TestCase):
    """Unit tests for the TelemetryLogger class."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.logger = TelemetryLogger(log_dir=self.tmpdir)

    def tearDown(self):
        # Make sure any open file handles are closed.
        if self.logger.is_logging():
            self.logger.stop_logging()
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    # -- filename generation -------------------------------------------------

    def test_filename_format(self):
        """Filenaem must follow flight_YYYYMMDD_HHMMSS.csv pattern."""
        path = self.logger._generate_filename()
        name = path.name
        self.assertTrue(name.startswith("flight_"))
        self.assertTrue(name.endswith(".csv"))
        # Strip the extension and check date-time portion length (15 chars).
        dt_part = name[len("flight_"):-4]
        self.assertEqual(len(dt_part), 15)

    def test_collision_resistance(self):
        """Two rapid calls to start_logging produce distinct files (different counters)."""
        self.logger.start_logging()
        first = self.logger.get_current_log_file()
        self.logger.stop_logging()

        # Start again - should get a new file with counter suffix
        self.logger.start_logging()
        second = self.logger.get_current_log_file()
        self.logger.stop_logging()

        self.assertNotEqual(first, second)
        self.assertIn("_1", second.name)

    def test_skips_existing(self):
        """If a file already exists, we skip past it."""
        existing = Path(self.tmpdir) / "flight_20260101_000000.csv"
        existing.write_text("stub\n")
        path = self.logger._generate_filename()
        self.assertNotEqual(path, existing)

    # -- start / stop idempotency ------------------------------------------

    def test_start_logging_creates_file(self):
        self.logger.start_logging()
        self.assertTrue(self.logger.is_logging())
        self.assertIsNotNone(self.logger.get_current_log_file())
        self.assertTrue(self.logger.get_current_log_file().exists())

    def test_stop_logging_closes_file(self):
        self.logger.start_logging()
        self.logger.stop_logging()
        self.assertFalse(self.logger.is_logging())
        self.assertIsNone(self.logger.get_current_log_file())

    def test_start_twice_is_idempotent(self):
        """Calling start_logging() twice does not create two files."""
        self.logger.start_logging()
        first = self.logger.get_current_log_file()
        self.logger.start_logging()
        self.assertTrue(self.logger.is_logging())
        self.assertEqual(self.logger.get_current_log_file(), first)

    def test_stop_twice_is_idempotent(self):
        self.logger.stop_logging()
        self.assertFalse(self.logger.is_logging())
        self.logger.stop_logging()
        self.assertFalse(self.logger.is_logging())

    def test_stop_without_start_succeeds(self):
        """stop_logging when nothing is logged returns True, no error."""
        result = self.logger.stop_logging()
        self.assertTrue(result)

    # -- CSV structure ------------------------------------------------------

    def test_header_written_on_start(self):
        """Header row must be present when logging stops."""
        self.logger.start_logging()
        self.logger.stop_logging()
        csv_files = sorted(Path(self.tmpdir).glob("flight_*.csv"))
        self.assertEqual(len(csv_files), 1)
        with open(csv_files[0], newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
        self.assertEqual(header, self.logger.CSV_COLUMNS)

    def test_csv_columns_match_spec(self):
        expected = [
            'timestamp_iso', 'timestamp_epoch', 'battery', 'altitude',
            'speed_x', 'speed_y', 'speed_z', 'speed_total',
            'pitch', 'roll', 'yaw',
            'acceleration_x', 'acceleration_y', 'acceleration_z',
            'barometer', 'time_of_flight', 'motor_time',
            'temperature', 'temperature_low',
        ]
        self.assertEqual(self.logger.CSV_COLUMNS, expected)

    def test_row_count_matches_columns(self):
        self.logger.start_logging()
        sample = {'battery': 85, 'altitude': 120, 'timestamp': time.time()}
        self.logger.log_telemetry(sample)
        self.logger.stop_logging()
        filepath = Path(self.tmpdir)
        csv_files = sorted(filepath.glob("flight_*.csv"))
        self.assertEqual(len(csv_files), 1)
        with open(csv_files[0], newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            row = next(reader)
        self.assertEqual(len(header), len(self.logger.CSV_COLUMNS))
        self.assertEqual(len(row), len(self.logger.CSV_COLUMNS))

    def test_empty_telemetry_produces_empty_fields(self):
        """log_telemetry with a bare dict writes a row of empty strings (except timestamp)."""
        self.logger.start_logging()
        self.logger.log_telemetry({})
        self.logger.stop_logging()
        csv_files = sorted(Path(self.tmpdir).glob("flight_*.csv"))
        with open(csv_files[0], newline="") as f:
            reader = csv.DictReader(f)
            row = next(reader)
        self.assertNotEqual(row['timestamp_iso'], '')
        self.assertNotEqual(row['timestamp_epoch'], '')
        for col in self.logger.CSV_COLUMNS[2:]:
            self.assertEqual(row[col], '')

    # -- no-op when not logging --------------------------------------------

    def test_log_before_start_returns_true(self):
        """log_telemetry before start_logging is a no-op that returns True."""
        result = self.logger.log_telemetry({'battery': 90})
        self.assertTrue(result)
        csv_files = list(Path(self.tmpdir).glob("flight_*.csv"))
        self.assertEqual(len(csv_files), 0)

    # -- flight duration ----------------------------------------------------

    def test_flight_duration_none_when_not_logging(self):
        self.assertIsNone(self.logger.get_flight_duration())

    def test_flight_duration_increases(self):
        self.logger.start_logging()
        time.sleep(0.05)
        dur = self.logger.get_flight_duration()
        self.logger.stop_logging()
        self.assertIsNotNone(dur)
        self.assertGreater(dur, 0.0)

    # -- context manager ---------------------------------------------------

    def test_context_manager_starts_and_stops(self):
        with self.logger:
            self.assertTrue(self.logger.is_logging())
        self.assertFalse(self.logger.is_logging())
        # File path is cleared after stop, but file itself persists on disk
        # Verify the file exists by checking the log directory
        csv_files = sorted(Path(self.tmpdir).glob("flight_*.csv"))
        self.assertEqual(len(csv_files), 1)

    # -- thread safety -----------------------------------------------------

    def test_concurrent_writes(self):
        """Multiple threads writing simultaneously should not raise."""
        self.logger.start_logging()
        errors = []

        def writer(tid):
            try:
                for _ in range(50):
                    self.logger.log_telemetry({
                        'battery': 80 + tid,
                        'altitude': 100 + tid * 10,
                        'timestamp': time.time(),
                    })
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(i,)) for i in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.logger.stop_logging()
        self.assertEqual(len(errors), 0)

        csv_files = sorted(Path(self.tmpdir).glob("flight_*.csv"))
        self.assertEqual(len(csv_files), 1)
        with open(csv_files[0], newline="") as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            rows = list(reader)
        self.assertEqual(len(rows), 200)  # 4 threads × 50 writes


if __name__ == "__main__":
    unittest.main()
