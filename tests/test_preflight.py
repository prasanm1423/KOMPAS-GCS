#!/usr/bin/env python3
"""Pre-flight checklist tests: unit checks, then real faults in the simulator."""

import os
import socket
import sys
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.preflight import PreflightChecker, CheckStatus   # noqa: E402

CONFIG = {
    'safety': {'min_battery_level': 20},
    'network': {'connection_timeout': 5.0},
    'preflight': {'max_tilt_deg': 10, 'battery_warning_margin': 15,
                  'warn_temperature': 70, 'max_temperature': 80, 'min_telemetry_rate_hz': 2},
}


def ready_aircraft(**telemetry):
    """Mocks for an aircraft that should pass every check."""
    now = time.time()
    controller = Mock()
    controller.is_flying = False
    controller.last_telemetry_time = now
    controller.last_telemetry = {'battery': 90, 'pitch': 1, 'roll': -1,
                                 'temperature': 45, 'altitude': 0, **telemetry}
    connection = Mock()
    connection.is_connected = True
    connection.last_command_reply = now
    connection.keepalive_interval = 5.0
    connection.send_command.return_value = "90"
    safety = Mock()
    safety.monitoring_active = True
    safety.safety_enabled = True
    return controller, connection, safety


def checker_for(controller, connection, safety):
    checker = PreflightChecker(controller, connection, safety, CONFIG)
    now = time.time()
    for i in range(20):                      # 10 Hz for the last 2 s
        checker.record_telemetry(now - i * 0.1)
    return checker


def status(report, name):
    return next(r for r in report.results if r.name == name).status


class PreflightUnitTest(unittest.TestCase):

    def test_ready_aircraft_is_cleared(self):
        report = checker_for(*ready_aircraft()).run(probe=True)
        self.assertEqual(report.failures, [])
        self.assertEqual(report.warnings, [])
        self.assertTrue(report.cleared)
        self.assertEqual(report.summary(), "READY FOR TAKEOFF")

    def test_low_battery_blocks(self):
        report = checker_for(*ready_aircraft(battery=15)).run()
        self.assertEqual(status(report, "Battery"), CheckStatus.FAIL)
        self.assertFalse(report.cleared)

    def test_marginal_battery_warns(self):
        report = checker_for(*ready_aircraft(battery=30)).run()
        self.assertEqual(status(report, "Battery"), CheckStatus.WARN)
        self.assertTrue(report.cleared)       # allowed, after confirmation

    def test_tilted_ground_blocks(self):
        report = checker_for(*ready_aircraft(roll=14)).run()
        self.assertEqual(status(report, "Level"), CheckStatus.FAIL)

    def test_temperature(self):
        self.assertEqual(status(checker_for(*ready_aircraft(temperature=72)).run(), "Temperature"),
                         CheckStatus.WARN)
        self.assertEqual(status(checker_for(*ready_aircraft(temperature=85)).run(), "Temperature"),
                         CheckStatus.FAIL)

    def test_already_flying_blocks(self):
        controller, connection, safety = ready_aircraft()
        controller.is_flying = True
        self.assertEqual(status(checker_for(controller, connection, safety).run(), "On the ground"),
                         CheckStatus.FAIL)

    def test_disabled_safety_monitor_blocks(self):
        controller, connection, safety = ready_aircraft()
        safety.safety_enabled = False
        self.assertEqual(status(checker_for(controller, connection, safety).run(), "Safety monitor"),
                         CheckStatus.FAIL)

    def test_stale_telemetry_is_not_trusted(self):
        # Old telemetry saying 90% must not pass the battery check.
        controller, connection, safety = ready_aircraft()
        controller.last_telemetry_time = time.time() - 30
        report = checker_for(controller, connection, safety).run()
        self.assertEqual(status(report, "Telemetry link"), CheckStatus.FAIL)
        self.assertEqual(status(report, "Battery"), CheckStatus.FAIL)

    def test_low_telemetry_rate_blocks(self):
        checker = PreflightChecker(*ready_aircraft(), CONFIG)
        checker.record_telemetry()            # a single packet in the window
        self.assertEqual(status(checker.run(), "Telemetry link"), CheckStatus.FAIL)

    def test_probe_detects_deaf_aircraft(self):
        controller, connection, safety = ready_aircraft()
        connection.send_command.return_value = None
        report = checker_for(controller, connection, safety).run(probe=True)
        self.assertEqual(status(report, "Command link"), CheckStatus.FAIL)

    def test_not_connected(self):
        controller, connection, safety = ready_aircraft()
        connection.is_connected = False
        report = checker_for(controller, connection, safety).run(probe=True)
        self.assertEqual(status(report, "Connection"), CheckStatus.FAIL)
        connection.send_command.assert_not_called()


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class PreflightSimulatorTest(unittest.TestCase):
    """Real faults injected into the simulator, checked through the real stack."""

    def setUp(self):
        from simulator import SimulatedDrone
        from communication.wifi_connection import DroneConnection
        from control.drone_controller import DroneController
        from utils.safety_manager import SafetyManager
        from utils.telemetry_parser import TelemetryParser

        state_port = free_udp_port()
        self.sim = SimulatedDrone(command_port=0, state_port=state_port, video_enabled=False,
                                  telemetry_rate_hz=10, time_scale=4.0, seed=1).start()
        self.addCleanup(self.sim.stop)
        self.connection = DroneConnection("127.0.0.1", self.sim.command_port, state_port)
        self.connection.keepalive_interval = 0.5
        self.addCleanup(self.connection.disconnect)
        self.controller = DroneController(self.connection)
        safety = SafetyManager(self.controller, CONFIG)
        safety.start_monitoring()
        self.addCleanup(safety.stop_monitoring)
        self.checker = PreflightChecker(self.controller, self.connection, safety, CONFIG)

        parser = TelemetryParser()
        def on_telemetry(line):   # as the GUI does
            self.controller.update_from_telemetry(parser.parse(line))
            self.checker.record_telemetry()
        self.connection.set_telemetry_callback(on_telemetry)
        self.assertTrue(self.connection.connect())
        time.sleep(2.2)           # fill the telemetry-rate window

    def test_healthy_simulator_is_cleared(self):
        report = self.checker.run(probe=True)
        self.assertTrue(report.cleared, report.summary())
        self.assertEqual(report.warnings, [])

    def test_one_way_link_is_caught(self):
        # Telemetry keeps flowing, so the status panel looks perfect...
        self.sim.drop_commands = True
        time.sleep(0.5)
        report = self.checker.run(probe=True)
        self.assertEqual(status(report, "Telemetry link"), CheckStatus.PASS)
        # ...but the aircraft can't hear us.
        self.assertEqual(status(report, "Command link"), CheckStatus.FAIL)
        self.assertFalse(report.cleared)
        # The passive check catches it too once keepalives go unanswered.
        time.sleep(2.5)
        self.assertEqual(status(self.checker.run(), "Command link"), CheckStatus.FAIL)

    def test_tilted_ground_blocks_takeoff(self):
        self.sim.ground_tilt = (3.0, 14.0)
        time.sleep(0.5)
        report = self.checker.run()
        self.assertEqual(status(report, "Level"), CheckStatus.FAIL)

    def test_low_battery(self):
        self.sim.set_battery(28)
        time.sleep(0.5)
        self.assertEqual(status(self.checker.run(), "Battery"), CheckStatus.WARN)
        self.sim.set_battery(12)
        time.sleep(0.5)
        self.assertEqual(status(self.checker.run(), "Battery"), CheckStatus.FAIL)

    def test_lost_link_blocks(self):
        self.sim.set_link(False)
        time.sleep(2.5)           # rate window empties
        report = self.checker.run()
        self.assertEqual(status(report, "Telemetry link"), CheckStatus.FAIL)
        self.assertFalse(report.cleared)


if __name__ == '__main__':
    unittest.main()
