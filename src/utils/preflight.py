"""
Pre-flight checklist for KOMPAS.

Takeoff is only allowed when every check passes. Warnings allow takeoff
after the operator explicitly confirms them; failures block it outright,
with no override.

Checks
  Connection      a session with the aircraft is open
  Telemetry link  state packets are fresh and arriving at a usable rate
  Command link    the aircraft answers commands. Telemetry only proves the
                  downlink works; this proves the aircraft can hear us. The
                  passive check uses the most recent reply (keepalives answer
                  every few seconds); the takeoff check sends a live probe.
  Battery         known, at or above safety.min_battery_level; a warning
                  within preflight.battery_warning_margin of it
  Level           pitch and roll within preflight.max_tilt_deg (tilted
                  ground makes a takeoff drift or flip)
  Temperature     below preflight.max_temperature; warning above
                  preflight.warn_temperature
  On the ground   not already flying
  Safety monitor  running and enabled, so limits are enforced in flight

run() is cheap and passive (safe to call every second from the UI);
run(probe=True) additionally sends one "battery?" query and is what the
takeoff path uses.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class CheckStatus(Enum):
    PASS = "pass"
    WARN = "warn"
    FAIL = "fail"


@dataclass
class CheckResult:
    name: str
    status: CheckStatus
    detail: str


@dataclass
class PreflightReport:
    results: List[CheckResult] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

    @property
    def failures(self) -> List[CheckResult]:
        return [r for r in self.results if r.status == CheckStatus.FAIL]

    @property
    def warnings(self) -> List[CheckResult]:
        return [r for r in self.results if r.status == CheckStatus.WARN]

    @property
    def cleared(self) -> bool:
        """True when nothing blocks takeoff (warnings still need confirming)."""
        return not self.failures

    def summary(self) -> str:
        if self.failures:
            return "NOT READY: " + "; ".join(f"{r.name} - {r.detail}" for r in self.failures)
        if self.warnings:
            return "READY WITH WARNINGS: " + "; ".join(f"{r.name} - {r.detail}" for r in self.warnings)
        return "READY FOR TAKEOFF"


DEFAULTS = {
    "max_tilt_deg": 10,
    "battery_warning_margin": 15,
    "warn_temperature": 70,
    "max_temperature": 80,
    "min_telemetry_rate_hz": 2,
}


class PreflightChecker:

    RATE_WINDOW = 2.0   # s over which the telemetry rate is measured

    def __init__(self, controller, connection, safety, config: Dict[str, Any]):
        self.controller = controller
        self.connection = connection
        self.safety = safety

        pre = {**DEFAULTS, **config.get("preflight", {})}
        self.max_tilt = pre["max_tilt_deg"]
        self.battery_margin = pre["battery_warning_margin"]
        self.warn_temperature = pre["warn_temperature"]
        self.max_temperature = pre["max_temperature"]
        self.min_rate = pre["min_telemetry_rate_hz"]
        self.min_battery = config.get("safety", {}).get("min_battery_level", 20)
        self.link_timeout = config.get("network", {}).get("connection_timeout", 5.0)

        self._telemetry_times = deque()

    def record_telemetry(self, now: Optional[float] = None):
        """Call for every telemetry packet received (feeds the rate check)."""
        self._telemetry_times.append(time.time() if now is None else now)

    def telemetry_rate(self, now: Optional[float] = None) -> float:
        now = time.time() if now is None else now
        while self._telemetry_times and now - self._telemetry_times[0] > self.RATE_WINDOW:
            self._telemetry_times.popleft()
        return len(self._telemetry_times) / self.RATE_WINDOW

    # ------------------------------------------------------------------ run

    def run(self, probe: bool = False) -> PreflightReport:
        now = time.time()
        report = PreflightReport(timestamp=now)
        connected = bool(self.connection and self.connection.is_connected)
        report.results.append(self._check_connection(connected))
        report.results.append(self._check_telemetry(connected, now))
        report.results.append(self._check_commands(connected, now, probe))

        telemetry = self._fresh_telemetry(now)
        report.results.append(self._check_battery(telemetry))
        report.results.append(self._check_level(telemetry))
        report.results.append(self._check_temperature(telemetry))
        report.results.append(self._check_on_ground(telemetry))
        report.results.append(self._check_safety_monitor())
        return report

    def _fresh_telemetry(self, now: float) -> Dict[str, Any]:
        """Latest telemetry, or {} if it's too old to trust."""
        last = getattr(self.controller, "last_telemetry_time", 0.0) or 0.0
        if now - last > self.link_timeout:
            return {}
        return getattr(self.controller, "last_telemetry", {}) or {}

    # --------------------------------------------------------------- checks

    @staticmethod
    def _check_connection(connected: bool) -> CheckResult:
        if connected:
            return CheckResult("Connection", CheckStatus.PASS, "session open")
        return CheckResult("Connection", CheckStatus.FAIL, "not connected")

    def _check_telemetry(self, connected: bool, now: float) -> CheckResult:
        name = "Telemetry link"
        if not connected:
            return CheckResult(name, CheckStatus.FAIL, "not connected")
        last = getattr(self.controller, "last_telemetry_time", 0.0) or 0.0
        if not last:
            return CheckResult(name, CheckStatus.FAIL, "no telemetry received")
        age = now - last
        if age > self.link_timeout:
            return CheckResult(name, CheckStatus.FAIL, f"silent for {age:.0f}s")
        rate = self.telemetry_rate(now)
        if rate < self.min_rate:
            return CheckResult(name, CheckStatus.FAIL, f"{rate:.1f} Hz (need {self.min_rate} Hz)")
        return CheckResult(name, CheckStatus.PASS, f"{rate:.0f} Hz")

    def _check_commands(self, connected: bool, now: float, probe: bool) -> CheckResult:
        name = "Command link"
        if not connected:
            return CheckResult(name, CheckStatus.FAIL, "not connected")
        if probe:
            started = time.time()
            reply = self.connection.send_command("battery?", timeout=1.5)
            if reply is None:
                return CheckResult(name, CheckStatus.FAIL, "aircraft did not answer a command")
            rtt_ms = (time.time() - started) * 1000
            if not reply.strip().isdigit():
                return CheckResult(name, CheckStatus.FAIL, f"unexpected reply {reply!r}")
            return CheckResult(name, CheckStatus.PASS, f"answered in {rtt_ms:.0f} ms")

        last = getattr(self.connection, "last_command_reply", 0.0) or 0.0
        window = getattr(self.connection, "keepalive_interval", 5.0) * 2 + 1.0
        if not last:
            return CheckResult(name, CheckStatus.FAIL, "no command answered yet")
        if now - last > window:
            return CheckResult(name, CheckStatus.FAIL, f"no reply for {now - last:.0f}s")
        return CheckResult(name, CheckStatus.PASS, f"last reply {now - last:.0f}s ago")

    def _check_battery(self, telemetry: Dict[str, Any]) -> CheckResult:
        name = "Battery"
        battery = telemetry.get("battery")
        if battery is None:
            return CheckResult(name, CheckStatus.FAIL, "unknown (no fresh telemetry)")
        if battery < self.min_battery:
            return CheckResult(name, CheckStatus.FAIL, f"{battery}% (minimum {self.min_battery}%)")
        if battery < self.min_battery + self.battery_margin:
            return CheckResult(name, CheckStatus.WARN,
                               f"{battery}% - short flight before the low-battery landing")
        return CheckResult(name, CheckStatus.PASS, f"{battery}%")

    def _check_level(self, telemetry: Dict[str, Any]) -> CheckResult:
        name = "Level"
        pitch, roll = telemetry.get("pitch"), telemetry.get("roll")
        if pitch is None or roll is None:
            return CheckResult(name, CheckStatus.FAIL, "attitude unknown")
        tilt = max(abs(pitch), abs(roll))
        if tilt > self.max_tilt:
            return CheckResult(name, CheckStatus.FAIL,
                               f"tilted {tilt}° (pitch {pitch}°, roll {roll}°; max {self.max_tilt}°)")
        return CheckResult(name, CheckStatus.PASS, f"pitch {pitch}°, roll {roll}°")

    def _check_temperature(self, telemetry: Dict[str, Any]) -> CheckResult:
        name = "Temperature"
        temperature = telemetry.get("temperature")
        if temperature is None:
            return CheckResult(name, CheckStatus.WARN, "not reported")
        if temperature >= self.max_temperature:
            return CheckResult(name, CheckStatus.FAIL, f"{temperature}°C (max {self.max_temperature}°C)")
        if temperature >= self.warn_temperature:
            return CheckResult(name, CheckStatus.WARN, f"{temperature}°C - running hot")
        return CheckResult(name, CheckStatus.PASS, f"{temperature}°C")

    def _check_on_ground(self, telemetry: Dict[str, Any]) -> CheckResult:
        name = "On the ground"
        if getattr(self.controller, "is_flying", False):
            return CheckResult(name, CheckStatus.FAIL, "already flying")
        altitude = telemetry.get("altitude")
        if altitude is not None and altitude > 10:
            return CheckResult(name, CheckStatus.FAIL, f"reports {altitude} cm altitude")
        return CheckResult(name, CheckStatus.PASS, "landed")

    def _check_safety_monitor(self) -> CheckResult:
        name = "Safety monitor"
        if not self.safety:
            return CheckResult(name, CheckStatus.FAIL, "not available")
        if not self.safety.monitoring_active:
            return CheckResult(name, CheckStatus.FAIL, "not running")
        if not self.safety.safety_enabled:
            return CheckResult(name, CheckStatus.FAIL, "disabled")
        return CheckResult(name, CheckStatus.PASS, "active")
