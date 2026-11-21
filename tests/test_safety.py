#!/usr/bin/env python3
"""
SafetyManager tests.

Unit tests drive check() by hand against a mock controller. Scenario tests
fly the simulator over real UDP with the ground station's real connection,
controller, parser and safety manager, wired the way the GUI wires them.

The rule under test throughout: automation may land the aircraft, but
never cuts the motors.
"""

import os
import socket
import sys
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.safety_manager import SafetyManager, SafetyLevel   # noqa: E402

CONFIG = {
    'safety': {'min_battery_level': 20, 'auto_land_battery_threshold': 10,
               'auto_land_low_battery': True, 'max_flight_time': 1200,
               'connection_timeout_action': 'land'},
    'network': {'connection_timeout': 5.0},
    'flight': {'max_altitude': 500},
}


def make_config(**overrides):
    config = {k: dict(v) for k, v in CONFIG.items()}
    for dotted, value in overrides.items():
        section, key = dotted.split('.')
        config[section][key] = value
    return config


def flying_controller(battery=80, altitude=100):
    controller = Mock()
    controller.is_flying = True
    controller.current_battery = battery
    controller.current_altitude = altitude
    controller.connection.is_connected = True
    controller.last_telemetry_time = time.time()

    def land():
        controller.is_flying = False
        return True
    controller.land.side_effect = land
    return controller


def wait_until(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


class SafetyUnitTest(unittest.TestCase):

    def setUp(self):
        self.events = []

    def manager(self, controller, **overrides):
        safety = SafetyManager(controller, make_config(**overrides))
        safety.set_safety_alert_callback(self.events.append)
        safety.LAND_RETRY_INTERVAL = 0.05
        self.addCleanup(safety.stop_monitoring)
        return safety

    def types(self):
        return [e['type'] for e in self.events]

    def test_critical_battery_lands_and_never_cuts_motors(self):
        controller = flying_controller(battery=4)   # old code cut motors at <=5%
        safety = self.manager(controller)
        safety.check()
        self.assertTrue(wait_until(lambda: controller.land.called))
        controller.emergency_stop.assert_not_called()

    def test_warnings_are_raised_once_per_flight(self):
        controller = flying_controller(battery=18)
        safety = self.manager(controller)
        for _ in range(20):
            safety.check()
        self.assertEqual(self.types().count('warning'), 1)

    def test_unknown_battery_is_not_an_error(self):
        controller = flying_controller(battery=None)
        safety = self.manager(controller)
        safety.check()   # must not raise
        controller.land.assert_not_called()

    def test_auto_land_can_be_disabled(self):
        controller = flying_controller(battery=8)
        safety = self.manager(controller, **{'safety.auto_land_low_battery': False})
        safety.check()
        time.sleep(0.2)
        controller.land.assert_not_called()
        self.assertTrue(any(e['level'] == SafetyLevel.CRITICAL for e in self.events))

    def test_flight_time_limit_lands_once(self):
        controller = flying_controller()
        controller.land.side_effect = None          # landing stays "in progress"
        controller.land.return_value = False
        safety = self.manager(controller, **{'safety.max_flight_time': 1})
        safety.check()                              # detects takeoff
        safety.flight_start_time -= 2
        for _ in range(10):
            safety.check()
        self.assertEqual(self.types().count('auto_land'), 1)
        self.assertTrue(wait_until(lambda: controller.land.call_count >= 2))  # retries...
        self.assertIsNotNone(safety._land_thread)                            # ...in one thread
        controller.emergency_stop.assert_not_called()

    def test_fresh_telemetry_is_not_a_lost_link(self):
        # The old code measured a heartbeat nothing updated, so every flight
        # "lost its link" 10 s after takeoff.
        controller = flying_controller()
        safety = self.manager(controller)
        safety.check()
        safety.last_heartbeat_time -= 60
        safety.check()
        self.assertNotIn('connection_loss', self.types())

    def test_link_loss_waits_for_link_before_landing(self):
        controller = flying_controller()
        controller.last_telemetry_time = time.time() - 30
        safety = self.manager(controller)
        safety.check()
        self.assertIn('connection_loss', self.types())
        time.sleep(0.3)
        controller.land.assert_not_called()         # pointless while the link is down

        controller.last_telemetry_time = time.time()   # link back
        safety.check()
        self.assertIn('connection_restored', self.types())
        self.assertTrue(wait_until(lambda: controller.land.called))
        controller.emergency_stop.assert_not_called()

    def test_link_loss_hover_action_only_alerts(self):
        controller = flying_controller()
        controller.last_telemetry_time = time.time() - 30
        safety = self.manager(controller, **{'safety.connection_timeout_action': 'hover'})
        safety.check()
        controller.last_telemetry_time = time.time()
        safety.check()
        time.sleep(0.2)
        controller.land.assert_not_called()

    def test_operator_emergency_is_never_swallowed(self):
        controller = flying_controller()
        safety = self.manager(controller)
        safety.trigger_emergency_stop("first press")
        safety.trigger_emergency_stop("second press")
        self.assertEqual(controller.emergency_stop.call_count, 2)

    def test_flight_start_and_end_detected_without_hooks(self):
        controller = flying_controller()
        safety = self.manager(controller)
        safety.check()
        self.assertIsNotNone(safety.flight_start_time)
        controller.is_flying = False
        safety.check()
        self.assertIsNone(safety.flight_start_time)
        self.assertEqual(self.types(), ['takeoff', 'landing'])


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class SafetyScenarioTest(unittest.TestCase):
    """Simulated flights with the full ground-station stack."""

    def fly(self, safety_overrides=None, **sim_kwargs):
        from simulator import SimulatedDrone
        from communication.wifi_connection import DroneConnection
        from control.drone_controller import DroneController
        from utils.telemetry_parser import TelemetryParser

        state_port = free_udp_port()
        sim_kwargs.setdefault('time_scale', 4.0)
        self.sim = SimulatedDrone(command_port=0, state_port=state_port, video_enabled=False,
                                  telemetry_rate_hz=20, seed=1, **sim_kwargs).start()
        self.addCleanup(self.sim.stop)

        self.connection = DroneConnection("127.0.0.1", self.sim.command_port, state_port)
        self.connection.keepalive_interval = 0.5
        self.addCleanup(self.connection.disconnect)
        self.controller = DroneController(self.connection)
        self.safety = SafetyManager(self.controller, make_config(**(safety_overrides or {})))
        self.safety.CHECK_INTERVAL = 0.1
        self.safety.LAND_RETRY_INTERVAL = 0.2
        self.events = []
        self.safety.set_safety_alert_callback(self.events.append)
        self.addCleanup(self.safety.stop_monitoring)

        parser = TelemetryParser()
        def on_telemetry(line):   # as DroneControllerGUI._update_telemetry_display does
            parsed = parser.parse(line)
            self.controller.update_from_telemetry(parsed)
            self.safety.on_telemetry_update(parsed)
        self.connection.set_telemetry_callback(on_telemetry)

        self.assertTrue(self.connection.connect())
        self.safety.start_monitoring()
        self.assertTrue(wait_until(lambda: self.controller.current_battery is not None, 3))
        self.assertTrue(self.controller.takeoff())

    def types(self):
        return [e['type'] for e in self.events]

    def test_low_battery_ground_station_lands_before_firmware_does(self):
        self.fly(start_battery=22, battery_drain_per_min=30, command_watchdog_s=0)
        self.assertTrue(wait_until(lambda: self.sim.state.phase == "landed", timeout=20))
        self.assertIn('auto_land', self.types())
        self.assertEqual(self.types().count('warning'), 1)        # low-battery warning, once
        self.assertGreater(self.sim.state.battery, 5)             # before the firmware's 5% auto-land
        self.assertIn('land', self.sim.command_log)
        self.assertNotIn('emergency', self.sim.command_log)

    def test_link_loss_long_outage_firmware_lands_session_survives(self):
        self.fly({'network.connection_timeout': 0.5}, command_watchdog_s=8)
        self.sim.set_link(False)
        self.assertTrue(wait_until(lambda: 'connection_loss' in self.types(), timeout=3))
        # Link stays down: the aircraft's own watchdog brings it down.
        self.assertTrue(wait_until(lambda: self.sim.state.phase == "landed", timeout=10))
        self.sim.set_link(True)
        self.assertTrue(wait_until(lambda: not self.controller.is_flying, timeout=6))
        self.assertIn('connection_restored', self.types())
        self.assertTrue(self.connection.is_connected)             # no forced disconnect
        self.assertNotIn('emergency', self.sim.command_log)

    def test_link_loss_short_outage_ground_station_lands_on_return(self):
        self.fly({'network.connection_timeout': 0.5}, command_watchdog_s=0)
        self.sim.set_link(False)
        self.assertTrue(wait_until(lambda: 'connection_loss' in self.types(), timeout=3))
        time.sleep(1.0)
        self.assertEqual(self.sim.state.phase, "flying")         # still up there, link down
        self.sim.set_link(True)
        self.assertTrue(wait_until(lambda: self.sim.state.phase == "landed", timeout=10))
        self.assertIn('land', self.sim.command_log)
        self.assertNotIn('emergency', self.sim.command_log)

    def test_flight_time_limit_lands(self):
        self.fly({'safety.max_flight_time': 2}, command_watchdog_s=0)
        self.assertTrue(wait_until(lambda: self.sim.state.phase == "landed", timeout=10))
        self.assertIn('auto_land', self.types())
        self.assertIn('landing_complete', self.types())

    def test_takeoff_uses_configured_battery_minimum(self):
        # Controller limit wired from config the way the GUI does it.
        from simulator import SimulatedDrone  # noqa: F401  (same stack, lower battery)
        self.fly(start_battery=50, command_watchdog_s=0)
        self.assertTrue(self.controller.land())
        self.assertTrue(wait_until(lambda: not self.controller.is_flying, 3))
        self.controller.min_battery = 60
        self.assertFalse(self.controller.takeoff())


if __name__ == '__main__':
    unittest.main()
