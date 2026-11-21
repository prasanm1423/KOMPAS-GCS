"""
Simulated drone for KOMPAS's Flight Simulator Mode.

The simulator is a stand-in for the aircraft, not a mock inside the GUI. It
listens for the same text commands on UDP, answers them the way the firmware
does, and streams state and video back over UDP. The ground station's real
connection, parser, controller and GUI code therefore run unchanged against
it, so bugs in any of them show up in simulation instead of in the field.

Protocol (Tello-SDK style, which is what DroneConnection speaks):

  command port  <- "command", "takeoff", "forward 50", "cw 90", "battery?" ...
                -> "ok" / "error <reason>" / query value
  state port    -> one line per tick, e.g.
                   pitch:-3;roll:1;yaw:45;vgx:30;vgy:0;vgz:0;templ:58;temph:61;
                   tof:92;h:80;bat:87;baro:12.34;time:15;agx:-12.00;agy:4.00;
                   agz:-998.00;
  video port    -> JPEG frames split into <=1400-byte datagrams

State and video go to the IP address that last sent a command, on the
ground station's state/video ports, the same way the real firmware does it.

Units in the state string: distances and heights in cm, velocities in cm/s
(vgx = forward, vgy = right, vgz = up, all in the drone's body frame),
angles in degrees, acceleration in thousandths of g, barometer in metres.
Real Tello firmware reports velocities in dm/s; this simulator uses cm/s
because that's what the ground station displays.

Motion commands are answered when the manoeuvre finishes, as on the real
aircraft, so the ground station's timeouts are exercised realistically.
"""

from __future__ import annotations

import logging
import math
import random
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

TAKEOFF_HEIGHT = 80.0        # cm, height the aircraft climbs to on "takeoff"
MIN_FLIGHT_HEIGHT = 30.0     # cm, "down" won't descend below this
CEILING = 1000.0             # cm, firmware altitude limit
MAX_ACCEL = 200.0            # cm/s^2, horizontal and vertical
MAX_CLIMB_RATE = 60.0        # cm/s
LANDING_RATE = 50.0          # cm/s
YAW_RATE = 90.0              # deg/s
POSITION_GAIN = 2.0          # 1/s, proportional gain of the position loop
ATTITUDE_GAIN = 0.2          # deg of tilt per cm/s of body velocity
MAX_TILT = 25.0              # deg
GRAVITY = 981.0              # cm/s^2
FIRMWARE_AUTO_LAND_BATTERY = 5.0   # % the "firmware" forces a landing at
MANEUVER_TIMEOUT = 30.0      # s, give up on a manoeuvre that never converges
VIDEO_CHUNK = 1400           # bytes per video datagram
ARRIVAL_POSITION_TOL = 2.0   # cm
ARRIVAL_SPEED_TOL = 3.0      # cm/s


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _wrap_deg(angle: float) -> float:
    """Wrap to (-180, 180]."""
    angle = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if angle == -180.0 else angle


@dataclass
class SimState:
    """Everything the physics tick reads and writes. World frame: x east,
    y north, z up (cm). Yaw is compass heading, clockwise from north (deg)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    battery: float = 100.0
    temp_high: float = 45.0
    motor_time: float = 0.0
    motors_on: bool = False
    phase: str = "landed"   # landed | takeoff | flying | landing | falling
    # Position/heading setpoints for the flying phases
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    yaw_remaining: float = 0.0
    speed: float = 50.0     # cm/s, set by the "speed" command

    @property
    def flying(self) -> bool:
        return self.phase in ("takeoff", "flying", "landing", "falling")

    def body_velocity(self) -> Tuple[float, float]:
        """(forward, right) components of horizontal velocity, cm/s."""
        yaw = math.radians(self.yaw)
        forward = self.vx * math.sin(yaw) + self.vy * math.cos(yaw)
        right = self.vx * math.cos(yaw) - self.vy * math.sin(yaw)
        return forward, right


@dataclass
class _Maneuver:
    command: str
    reply_to: Tuple[str, int]
    done: threading.Event = field(default_factory=threading.Event)
    result: str = "ok"


class SimulatedDrone:
    """A simulated aircraft reachable over UDP. Use start()/stop(), or call
    step() directly for deterministic tests."""

    def __init__(self,
                 bind_host: str = "127.0.0.1",
                 command_port: int = 8889,
                 state_port: int = 8890,
                 video_port: int = 11111,
                 telemetry_rate_hz: float = 10.0,
                 video_enabled: bool = True,
                 video_fps: float = 15.0,
                 start_battery: float = 100.0,
                 battery_drain_per_min: float = 7.0,
                 command_watchdog_s: float = 15.0,
                 time_scale: float = 1.0,
                 seed: Optional[int] = None):
        self.bind_host = bind_host
        self.command_port = command_port
        self.state_port = state_port
        self.video_port = video_port
        self.telemetry_rate_hz = telemetry_rate_hz
        self.video_enabled = video_enabled
        self.video_fps = video_fps
        self.battery_drain_per_min = battery_drain_per_min
        self.command_watchdog_s = command_watchdog_s
        self.time_scale = time_scale

        self.state = SimState(battery=float(start_battery))
        self._rng = random.Random(seed)
        self._lock = threading.RLock()
        self._maneuver: Optional[_Maneuver] = None

        self.sdk_mode = False
        self.streaming = False
        self.client_ip: Optional[str] = None
        self._last_command_time = 0.0
        self._sim_time = 0.0

        # Fault injection: with the link down, commands are silently dropped and
        # no state or video is sent, as when the aircraft is out of radio
        # range. The aircraft itself keeps flying and its watchdog keeps running.
        self.link_up = True
        self.drop_commands = False             # one-way link: telemetry flows, commands don't arrive
        self.ground_tilt = (0.0, 0.0)          # (pitch, roll) of the surface it's sitting on
        self.command_log = deque(maxlen=500)   # every command received, for inspection

        self._running = False
        self._threads = []
        self._cmd_sock: Optional[socket.socket] = None
        self._out_sock: Optional[socket.socket] = None
        self.packets_sent = 0
        self.frames_sent = 0

    # ------------------------------------------------------------------ run

    def start(self) -> "SimulatedDrone":
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.bind((self.bind_host, self.command_port))
        self._cmd_sock.settimeout(0.5)
        self.command_port = self._cmd_sock.getsockname()[1]  # resolves port 0
        self._out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._running = True
        targets = [self._receive_loop, self._physics_loop]
        if self.video_enabled:
            targets.append(self._video_loop)
        for target in targets:
            thread = threading.Thread(target=target, name=f"sim-{target.__name__}", daemon=True)
            thread.start()
            self._threads.append(thread)

        logger.info("Simulated drone listening on %s:%d (state -> :%d, video -> :%d)",
                    self.bind_host, self.command_port, self.state_port, self.video_port)
        return self

    def stop(self):
        self._running = False
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads.clear()
        for sock in (self._cmd_sock, self._out_sock):
            if sock:
                sock.close()
        self._cmd_sock = self._out_sock = None
        logger.info("Simulated drone stopped")

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # ------------------------------------------------------------- commands

    def _receive_loop(self):
        while self._running:
            try:
                data, addr = self._cmd_sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            command = data.decode("utf-8", errors="replace").strip()
            if not command or not self.link_up or self.drop_commands:
                continue
            self.command_log.append(command)
            reply = self.handle_command(command, addr)
            if reply is not None:
                self._reply(reply, addr)

    def _reply(self, text: str, addr: Tuple[str, int]):
        sock = self._cmd_sock
        if sock is None or not self.link_up:
            return
        try:
            sock.sendto(text.encode("utf-8"), addr)
        except OSError as e:
            logger.debug("Reply to %s failed: %s", addr, e)

    def handle_command(self, command: str, addr: Tuple[str, int] = ("127.0.0.1", 0)) -> Optional[str]:
        """Process one command. Returns the immediate reply, or None when a
        manoeuvre was started and will be answered once it finishes."""
        parts = command.lower().split()
        name, args = parts[0], parts[1:]
        logger.debug("SIM <- %s", command)

        with self._lock:
            self._last_command_time = self._sim_time

            if name == "command":
                self.sdk_mode = True
                self.client_ip = addr[0]
                return "ok"
            if not self.sdk_mode:
                return "error Not in SDK mode"
            self.client_ip = addr[0]

            if name == "emergency":
                self._emergency()
                return "ok"

            if name.endswith("?"):
                return self._query(name)

            if name in ("streamon", "streamoff"):
                self.streaming = name == "streamon"
                return "ok"
            if name in ("setbitrate", "setresolution", "setfps"):
                return "ok" if len(args) == 1 else "error Bad argument"
            if name == "speed":
                value = self._int_arg(args, 10, 100)
                if value is None:
                    return "error Out of range"
                self.state.speed = float(value)
                return "ok"

            if name in ("takeoff", "land", "up", "down", "left", "right",
                        "forward", "back", "cw", "ccw"):
                return self._start_maneuver(name, args, command, addr)

        return "error Unknown command"

    @staticmethod
    def _int_arg(args, lo: int, hi: int) -> Optional[int]:
        if len(args) != 1:
            return None
        try:
            value = int(float(args[0]))
        except ValueError:
            return None
        return value if lo <= value <= hi else None

    def _query(self, name: str) -> str:
        s = self.state
        return {
            "battery?": f"{int(s.battery)}",
            "speed?": f"{s.speed:.1f}",
            "time?": f"{int(s.motor_time)}s",
            "height?": f"{int(round(s.z / 10))}dm",
            "temp?": f"{int(s.temp_high) - 2}~{int(s.temp_high)}C",
            "attitude?": f"pitch:{int(s.pitch)};roll:{int(s.roll)};yaw:{int(s.yaw)};",
            "baro?": f"{self._baro():.2f}",
            "tof?": f"{int(self._tof() * 10)}mm",
            "wifi?": "90",
            "sdk?": "30",
            "sn?": "KOMPASSIM0001",
        }.get(name, "error Unknown command")

    def _start_maneuver(self, name: str, args, command: str, addr) -> Optional[str]:
        s = self.state
        if self._maneuver is not None:
            return "error Busy"

        if name == "takeoff":
            if s.flying:
                return "error Already flying"
            if s.battery < 10:
                return "error Low battery"
            s.motors_on = True
            s.phase = "takeoff"
            s.target_x, s.target_y, s.target_z = s.x, s.y, TAKEOFF_HEIGHT
        elif name == "land":
            if s.phase not in ("flying", "takeoff"):
                return "error Not flying"
            s.phase = "landing"
        else:
            if s.phase != "flying":
                return "error Not flying"
            if name in ("cw", "ccw"):
                degrees = self._int_arg(args, 1, 360)
                if degrees is None:
                    return "error Out of range"
                s.yaw_remaining = degrees if name == "cw" else -degrees
            else:
                distance = self._int_arg(args, 20, 500)
                if distance is None:
                    return "error Out of range"
                self._set_move_target(name, distance)

        self._maneuver = _Maneuver(command, addr)
        threading.Thread(target=self._await_maneuver, args=(self._maneuver,), daemon=True).start()
        return None

    def _set_move_target(self, direction: str, distance: float):
        s = self.state
        if direction == "up":
            s.target_z = min(CEILING, s.target_z + distance)
            return
        if direction == "down":
            s.target_z = max(MIN_FLIGHT_HEIGHT, s.target_z - distance)
            return
        yaw = math.radians(s.yaw)
        fwd = (math.sin(yaw), math.cos(yaw))
        right = (math.cos(yaw), -math.sin(yaw))
        vec = {"forward": fwd, "back": (-fwd[0], -fwd[1]),
               "right": right, "left": (-right[0], -right[1])}[direction]
        s.target_x += vec[0] * distance
        s.target_y += vec[1] * distance

    def _await_maneuver(self, maneuver: _Maneuver):
        """Reply to a motion command once physics reports it finished."""
        finished = maneuver.done.wait(MANEUVER_TIMEOUT / max(self.time_scale, 1e-6))
        with self._lock:
            if not finished:
                maneuver.result = "error Timeout"
            if self._maneuver is maneuver:
                self._maneuver = None
        logger.debug("SIM -> %s: %s", maneuver.command, maneuver.result)
        self._reply(maneuver.result, maneuver.reply_to)

    def _finish_maneuver(self, result: str = "ok"):
        if self._maneuver is not None:
            self._maneuver.result = result
            self._maneuver.done.set()
            self._maneuver = None

    def _emergency(self):
        s = self.state
        self._finish_maneuver("error Motor stop")
        s.motors_on = False
        s.yaw_remaining = 0.0
        s.phase = "falling" if s.z > 0.5 else "landed"
        logger.warning("SIM: emergency motor stop at %.0f cm", s.z)

    def _auto_land(self, reason: str):
        s = self.state
        if s.phase in ("landing", "falling", "landed"):
            return
        self._finish_maneuver(f"error Auto land ({reason})")
        s.yaw_remaining = 0.0
        s.phase = "landing"
        logger.warning("SIM: auto-landing (%s)", reason)

    # -------------------------------------------------------------- physics

    def step(self, dt: float):
        """Advance the simulation by dt seconds of simulated time."""
        with self._lock:
            self._sim_time += dt
            s = self.state
            prev_v = (s.vx, s.vy, s.vz)

            self._update_battery(dt)
            if (s.flying and self.command_watchdog_s > 0 and s.phase != "falling"
                    and self._sim_time - self._last_command_time > self.command_watchdog_s):
                self._auto_land(f"no command for {self.command_watchdog_s:.0f} s")

            if s.phase == "falling":
                self._step_falling(dt)
            elif s.phase in ("takeoff", "flying"):
                self._step_position(dt, max_climb=MAX_CLIMB_RATE)
                self._step_yaw(dt)
                if self._arrived():
                    if s.phase == "takeoff":
                        s.phase = "flying"
                    self._finish_maneuver("ok")
            elif s.phase == "landing":
                s.target_x, s.target_y = s.x, s.y
                s.target_z = 0.0
                self._step_position(dt, max_climb=LANDING_RATE)
                if s.z <= 0.5:
                    self._touch_down()
                    self._finish_maneuver("ok")
            else:  # landed
                s.vx = s.vy = s.vz = 0.0

            s.ax = (s.vx - prev_v[0]) / dt
            s.ay = (s.vy - prev_v[1]) / dt
            s.az = (s.vz - prev_v[2]) / dt
            self._update_attitude()
            self._update_thermal(dt)
            if s.motors_on:
                s.motor_time += dt

    def _update_battery(self, dt: float):
        s = self.state
        if s.motors_on:
            moving = math.hypot(s.vx, s.vy) > 5 or abs(s.vz) > 5
            per_min = self.battery_drain_per_min * (1.3 if moving else 1.0)
        else:
            per_min = 0.3  # electronics only
        s.battery = max(0.0, s.battery - per_min / 60.0 * dt)

        if s.battery <= 0.0 and s.motors_on:
            logger.warning("SIM: battery depleted")
            self._emergency()
        elif s.battery < FIRMWARE_AUTO_LAND_BATTERY and s.flying:
            self._auto_land("battery critical")

    def _step_position(self, dt: float, max_climb: float):
        s = self.state
        # Horizontal: P-controller, vector-limited to the commanded speed
        dvx = POSITION_GAIN * (s.target_x - s.x)
        dvy = POSITION_GAIN * (s.target_y - s.y)
        mag = math.hypot(dvx, dvy)
        if mag > s.speed:
            dvx, dvy = dvx * s.speed / mag, dvy * s.speed / mag
        dvz = _clamp(POSITION_GAIN * (s.target_z - s.z), -max_climb, max_climb)

        step = MAX_ACCEL * dt
        s.vx += _clamp(dvx - s.vx, -step, step)
        s.vy += _clamp(dvy - s.vy, -step, step)
        s.vz += _clamp(dvz - s.vz, -step, step)

        s.x += s.vx * dt
        s.y += s.vy * dt
        s.z = max(0.0, s.z + s.vz * dt)

    def _step_yaw(self, dt: float):
        s = self.state
        if s.yaw_remaining:
            turn = math.copysign(min(YAW_RATE * dt, abs(s.yaw_remaining)), s.yaw_remaining)
            s.yaw = _wrap_deg(s.yaw + turn)
            s.yaw_remaining -= turn
            if abs(s.yaw_remaining) < 1e-6:
                s.yaw_remaining = 0.0

    def _step_falling(self, dt: float):
        s = self.state
        s.vz -= GRAVITY * dt
        s.vx *= max(0.0, 1.0 - 0.5 * dt)  # air drag
        s.vy *= max(0.0, 1.0 - 0.5 * dt)
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.z += s.vz * dt
        if s.z <= 0.0:
            logger.warning("SIM: impact at %.0f cm/s", -s.vz)
            self._touch_down()

    def _touch_down(self):
        s = self.state
        s.z = 0.0
        s.vx = s.vy = s.vz = 0.0
        s.motors_on = False
        s.phase = "landed"
        s.target_x, s.target_y, s.target_z = s.x, s.y, 0.0

    def _arrived(self) -> bool:
        s = self.state
        position_error = math.dist((s.x, s.y, s.z), (s.target_x, s.target_y, s.target_z))
        speed = math.sqrt(s.vx ** 2 + s.vy ** 2 + s.vz ** 2)
        return (position_error < ARRIVAL_POSITION_TOL and speed < ARRIVAL_SPEED_TOL
                and s.yaw_remaining == 0.0)

    def _update_attitude(self):
        s = self.state
        if s.phase == "landed":
            s.pitch, s.roll = self.ground_tilt
            return
        if s.phase == "falling":
            s.pitch = self._rng.gauss(0.0, 3.0)
            s.roll = self._rng.gauss(0.0, 3.0)
            return
        forward, right = s.body_velocity()
        s.pitch = _clamp(-ATTITUDE_GAIN * forward, -MAX_TILT, MAX_TILT) + self._rng.gauss(0.0, 0.6)
        s.roll = _clamp(ATTITUDE_GAIN * right, -MAX_TILT, MAX_TILT) + self._rng.gauss(0.0, 0.6)

    def _update_thermal(self, dt: float):
        s = self.state
        ambient_target = 68.0 if s.motors_on else 45.0
        s.temp_high += (ambient_target - s.temp_high) * min(1.0, dt / 120.0)

    def _baro(self) -> float:
        return 12.0 + self.state.z / 100.0 + self._rng.gauss(0.0, 0.03)

    def _tof(self) -> float:
        """Downward range sensor, cm. Reads ~10 cm when sitting on the ground."""
        return self.state.z + 10.0

    # ------------------------------------------------------------ telemetry

    def telemetry_string(self) -> str:
        """Current state as a Tello-SDK state line."""
        with self._lock:
            s = self.state
            forward, right = s.body_velocity()
            yaw = math.radians(s.yaw)
            a_forward = s.ax * math.sin(yaw) + s.ay * math.cos(yaw)
            a_right = s.ax * math.cos(yaw) - s.ay * math.sin(yaw)
            flying = s.flying and s.phase != "landed"
            hover_noise = self._rng.gauss(0.0, 0.7) if flying else 0.0
            height = max(0, int(round(s.z + hover_noise)))
            return (
                f"pitch:{int(round(s.pitch))};roll:{int(round(s.roll))};yaw:{int(round(s.yaw))};"
                f"vgx:{int(round(forward))};vgy:{int(round(right))};vgz:{int(round(s.vz))};"
                f"templ:{int(s.temp_high) - 2};temph:{int(s.temp_high)};"
                f"tof:{int(round(self._tof()))};h:{height};bat:{int(s.battery)};"
                f"baro:{self._baro():.2f};time:{int(s.motor_time)};"
                f"agx:{a_forward / GRAVITY * 1000:.2f};agy:{a_right / GRAVITY * 1000:.2f};"
                f"agz:{-1000 + s.az / GRAVITY * 1000:.2f};\r\n"
            )

    def _physics_loop(self):
        tick = 1.0 / 50.0
        telemetry_period = 1.0 / self.telemetry_rate_hz
        since_telemetry = 0.0
        last = time.monotonic()
        while self._running:
            time.sleep(tick)
            now = time.monotonic()
            dt = min(now - last, 0.1) * self.time_scale  # cap: don't jump after a stall
            last = now
            self.step(dt)

            since_telemetry += dt / self.time_scale
            if since_telemetry >= telemetry_period:
                since_telemetry = 0.0
                self._send_state()

    # ------------------------------------------------------ fault injection

    def set_link(self, up: bool):
        """Simulate losing (False) or regaining (True) the radio link."""
        self.link_up = up
        logger.warning("SIM: radio link %s", "restored" if up else "LOST")

    def set_battery(self, percent: float):
        """Jump the battery to a level, e.g. to rehearse low-battery handling."""
        with self._lock:
            self.state.battery = float(_clamp(percent, 0.0, 100.0))
        logger.warning("SIM: battery set to %.0f%%", percent)

    def _send_state(self):
        if not (self.link_up and self.sdk_mode and self.client_ip and self._out_sock):
            return
        try:
            self._out_sock.sendto(self.telemetry_string().encode("utf-8"),
                                  (self.client_ip, self.state_port))
            self.packets_sent += 1
        except OSError as e:
            logger.debug("State send failed: %s", e)

    # ---------------------------------------------------------------- video

    def _video_loop(self):
        try:
            import cv2
            from simulator.sim_video import render_frame
        except ImportError as e:
            logger.warning("Simulated video disabled: %s", e)
            return

        period = 1.0 / self.video_fps
        while self._running:
            started = time.monotonic()
            if self.link_up and self.streaming and self.client_ip and self._out_sock:
                with self._lock:
                    snapshot = SimState(**vars(self.state))
                frame = render_frame(snapshot)
                ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    self._send_video(jpeg.tobytes())
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _send_video(self, data: bytes):
        target = (self.client_ip, self.video_port)
        try:
            for i in range(0, len(data), VIDEO_CHUNK):
                self._out_sock.sendto(data[i:i + VIDEO_CHUNK], target)
            self.frames_sent += 1
        except OSError as e:
            logger.debug("Video send failed: %s", e)
