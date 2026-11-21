#!/usr/bin/env python3
"""
Flight Simulator Mode tests.

The physics tests step the simulator by hand, so they're deterministic.
The integration tests run the ground station's real DroneConnection and
DroneController against the simulator over real UDP sockets, so they
exercise the actual telemetry loop end to end.
"""

import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from simulator import SimulatedDrone                      # noqa: E402
from utils.telemetry_parser import TelemetryParser        # noqa: E402


def fly(drone, command, max_seconds=30.0, dt=0.02):
    """Send a command to a hand-stepped simulator; step until it's answered."""
    reply = drone.handle_command(command)
    if reply is not None:
        return reply
    maneuver = drone._maneuver
    for _ in range(int(max_seconds / dt)):
        drone.step(dt)
        if maneuver.done.is_set():
            return maneuver.result
    return "NOT FINISHED"


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class TestSimPhysics(unittest.TestCase):

    def setUp(self):
        self.drone = SimulatedDrone(seed=1, command_watchdog_s=0)
        self.parser = TelemetryParser()

    def state(self):
        return self.parser.parse(self.drone.telemetry_string())

    def test_requires_sdk_mode(self):
        self.assertEqual(fly(self.drone, "takeoff"), "error Not in SDK mode")
        self.assertEqual(fly(self.drone, "command"), "ok")

    def test_takeoff_climbs_and_lands(self):
        fly(self.drone, "command")
        self.assertEqual(fly(self.drone, "takeoff"), "ok")
        self.assertAlmostEqual(self.state()['altitude'], 80, delta=4)
        self.assertEqual(fly(self.drone, "land"), "ok")
        self.assertEqual(self.state()['altitude'], 0)
        self.assertEqual(self.drone.state.phase, "landed")

    def test_moves_follow_heading(self):
        fly(self.drone, "command")
        fly(self.drone, "takeoff")
        self.assertEqual(fly(self.drone, "cw 90"), "ok")        # now facing east
        self.assertEqual(fly(self.drone, "forward 100"), "ok")
        self.assertAlmostEqual(self.drone.state.x, 100, delta=3)  # moved east...
        self.assertAlmostEqual(self.drone.state.y, 0, delta=3)    # ...not north
        self.assertEqual(fly(self.drone, "up 50"), "ok")
        self.assertAlmostEqual(self.drone.state.z, 130, delta=3)

    def test_telemetry_shows_motion_while_moving(self):
        fly(self.drone, "command")
        fly(self.drone, "takeoff")
        self.drone.handle_command("forward 300")
        for _ in range(50):          # 1 s into the move
            self.drone.step(0.02)
        state = self.state()
        self.assertGreater(state['speed_x'], 20)   # flying forward
        self.assertLess(state['pitch'], -3)        # nose down to accelerate

    def test_rejects_invalid_commands(self):
        fly(self.drone, "command")
        self.assertEqual(fly(self.drone, "forward 50"), "error Not flying")
        fly(self.drone, "takeoff")
        self.assertEqual(fly(self.drone, "forward 5"), "error Out of range")
        self.assertEqual(fly(self.drone, "speed 500"), "error Out of range")
        self.assertEqual(fly(self.drone, "barrelroll"), "error Unknown command")

    def test_emergency_drops_aircraft(self):
        fly(self.drone, "command")
        fly(self.drone, "takeoff")
        fly(self.drone, "up 100")
        self.drone.handle_command("forward 300")
        self.assertEqual(self.drone.handle_command("emergency"), "ok")
        for _ in range(100):
            self.drone.step(0.02)
        self.assertEqual(self.drone.state.phase, "landed")
        self.assertEqual(self.state()['altitude'], 0)
        self.assertFalse(self.drone.state.motors_on)

    def test_low_battery_forces_landing(self):
        drone = SimulatedDrone(seed=1, command_watchdog_s=0, start_battery=12,
                               battery_drain_per_min=60)  # 1 %/s
        fly(drone, "command")
        self.assertEqual(fly(drone, "takeoff"), "ok")
        for _ in range(int(15 / 0.02)):
            drone.step(0.02)
        self.assertEqual(drone.state.phase, "landed")

    def test_takeoff_refused_on_low_battery(self):
        drone = SimulatedDrone(seed=1, start_battery=8)
        fly(drone, "command")
        self.assertEqual(fly(drone, "takeoff"), "error Low battery")

    def test_command_watchdog_auto_lands(self):
        drone = SimulatedDrone(seed=1, command_watchdog_s=5)
        fly(drone, "command")
        fly(drone, "takeoff")
        for _ in range(int(10 / 0.02)):   # 10 s with no commands
            drone.step(0.02)
        self.assertEqual(drone.state.phase, "landed")

    def test_queries(self):
        fly(self.drone, "command")
        self.assertEqual(fly(self.drone, "battery?"), "100")
        self.assertEqual(fly(self.drone, "height?"), "0dm")


class TestTelemetryParserTelloFormat(unittest.TestCase):

    def test_temph_is_not_read_as_altitude(self):
        line = ("pitch:-3;roll:2;yaw:45;vgx:30;vgy:-40;vgz:0;templ:58;temph:61;"
                "tof:92;h:80;bat:87;baro:12.34;time:15;agx:-12.00;agy:4.00;agz:-998.00;\r\n")
        parsed = TelemetryParser().parse(line)
        self.assertEqual(parsed['altitude'], 80)
        self.assertEqual(parsed['temperature'], 61)
        self.assertEqual((parsed['pitch'], parsed['roll'], parsed['yaw']), (-3, 2, 45))
        self.assertEqual(parsed['speed_total'], 50.0)
        self.assertEqual(parsed['battery'], 87)


class TestTelemetryLoopOverUDP(unittest.TestCase):
    """Real sockets, real ground-station classes, simulated aircraft."""

    def setUp(self):
        from communication.wifi_connection import DroneConnection
        from control.drone_controller import DroneController

        self.state_port = free_udp_port()
        self.drone = SimulatedDrone(command_port=0, state_port=self.state_port,
                                    video_enabled=False, telemetry_rate_hz=20,
                                    time_scale=4.0, seed=1).start()
        self.connection = DroneConnection("127.0.0.1", self.drone.command_port, self.state_port)
        self.parser = TelemetryParser()
        self.telemetry = []
        self.connection.set_telemetry_callback(self.telemetry.append)
        self.controller = DroneController(self.connection)

    def tearDown(self):
        self.connection.disconnect()
        self.drone.stop()

    def latest(self):
        return self.parser.parse(self.telemetry[-1])

    def wait_for(self, predicate, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.telemetry and predicate(self.latest()):
                return True
            time.sleep(0.05)
        return False

    def feed_controller(self):
        """Stand-in for the GUI: route telemetry into the controller."""
        def route(line):
            self.telemetry.append(line)
            self.controller.update_from_telemetry(self.parser.parse(line))
        self.connection.set_telemetry_callback(route)

    def test_connect_and_receive_telemetry(self):
        self.assertTrue(self.connection.connect())
        self.assertTrue(self.connection.is_connected)
        self.assertTrue(self.wait_for(lambda s: 'battery' in s, timeout=3))
        time.sleep(1.0)
        self.assertGreaterEqual(len(self.telemetry), 10)   # ~20 Hz

    def test_full_flight_through_controller(self):
        self.feed_controller()
        self.assertTrue(self.connection.connect())
        self.assertTrue(self.controller.initialize())
        self.assertTrue(self.wait_for(lambda s: 'battery' in s, timeout=3))
        self.assertEqual(self.controller.current_battery, self.latest()['battery'])

        self.assertTrue(self.controller.takeoff())
        self.assertTrue(self.wait_for(lambda s: s['altitude'] >= 70))
        self.assertTrue(self.controller.move_up(50))
        self.assertTrue(self.wait_for(lambda s: s['altitude'] >= 120))
        self.assertTrue(self.controller.rotate_clockwise(90))
        self.assertTrue(self.wait_for(lambda s: abs(s['yaw'] - 90) <= 2))
        self.assertTrue(self.controller.land())
        self.assertTrue(self.wait_for(lambda s: s['altitude'] == 0))

    def test_takeoff_refused_before_battery_is_known(self):
        self.assertTrue(self.connection.connect())
        # Telemetry isn't routed to the controller, so battery stays unknown.
        self.assertIsNone(self.controller.current_battery)
        self.assertFalse(self.controller.takeoff())

    def test_emergency_not_blocked_by_move_in_progress(self):
        self.feed_controller()
        self.assertTrue(self.connection.connect())
        self.assertTrue(self.wait_for(lambda s: 'battery' in s, timeout=3))
        self.assertTrue(self.controller.takeoff())
        self.connection.send_command("speed 10")

        results = {}
        mover = threading.Thread(
            target=lambda: results.setdefault("move", self.connection.send_command("forward 500", timeout=60)))
        mover.start()
        time.sleep(0.5)                    # the move now holds the command lock

        started = time.time()
        ack = self.connection.send_command("emergency")
        self.assertLess(time.time() - started, 1.5)   # didn't wait behind the move
        self.assertEqual(ack, "ok")

        mover.join(timeout=5)
        self.assertEqual(results.get("move"), "error Motor stop")
        self.assertTrue(self.wait_for(lambda s: s['altitude'] == 0, timeout=5))

    def test_keepalive_prevents_watchdog_landing(self):
        self.drone.command_watchdog_s = 8    # sim seconds = 2 s real at 4x
        self.connection.keepalive_interval = 0.5
        self.feed_controller()
        self.assertTrue(self.connection.connect())
        self.assertTrue(self.wait_for(lambda s: 'battery' in s, timeout=3))
        self.assertTrue(self.controller.takeoff())
        time.sleep(4.0)                      # 16 sim-seconds with no pilot input
        self.assertEqual(self.drone.state.phase, "flying")
        self.assertGreater(self.latest()['altitude'], 60)

    def test_controller_notices_auto_landing(self):
        self.drone.command_watchdog_s = 4
        self.connection.keepalive_interval = 1000   # no keepalive: watchdog fires
        self.feed_controller()
        self.assertTrue(self.connection.connect())
        self.assertTrue(self.wait_for(lambda s: 'battery' in s, timeout=3))
        self.assertTrue(self.controller.takeoff())
        self.assertTrue(self.wait_for(lambda s: s['altitude'] == 0, timeout=10))
        deadline = time.time() + 5
        while self.controller.is_flying and time.time() < deadline:
            time.sleep(0.1)
        self.assertFalse(self.controller.is_flying)


if __name__ == '__main__':
    unittest.main()
