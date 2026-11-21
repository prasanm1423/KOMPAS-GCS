"""
Safety Manager for KOMPAS

Watches the aircraft while it flies and takes automatic action when a limit
is crossed. Two rules govern every automatic action:

  1. Automation may only ever LAND the aircraft. Cutting the motors
     ("emergency") drops it from whatever height it is at, so that is
     reserved for the operator's EMERGENCY STOP button.
  2. Each condition is reported once per flight (latched), not every tick,
     so a real alert is never buried under a stream of repeats.

Checks (every 0.5 s while flying):
  battery      warning at safety.min_battery_level, auto-land at
               safety.auto_land_battery_threshold (if auto_land_low_battery)
  flight time  warning at 80% of safety.max_flight_time, auto-land at 100%
  telemetry    link considered lost after network.connection_timeout s of
               silence; then safety.connection_timeout_action:
               "land"  - keep trying to land whenever the link is back
               "hover" - alert only
               Either way the firmware's own command watchdog lands the
               aircraft if the link stays down.
  altitude     warning above flight.max_altitude (re-arms 20 cm below it)

Flight start/end is detected from the controller's is_flying flag, so the
flight timer can't be skipped by a code path that forgets to call a hook.
"""

import threading
import time
import logging
from typing import Optional, Callable, Dict, Any
from enum import Enum

class SafetyLevel(Enum):
    """Safety alert levels"""
    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"

class SafetyManager:
    """Manages drone safety features and monitoring"""
    
    CHECK_INTERVAL = 0.5          # s between checks
    LAND_RETRY_INTERVAL = 1.0     # s between automatic landing attempts
    LAND_GIVE_UP_AFTER = 60.0     # s of failed attempts before handing over to the operator
    ALTITUDE_HYSTERESIS = 20      # cm
    
    def __init__(self, controller, config: Dict[str, Any]):
        """
        Initialize safety manager
        
        Args:
            controller: DroneController instance
            config: Configuration dictionary
        """
        self.controller = controller
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        safety = config.get('safety', {})
        self.min_battery_flight = safety.get('min_battery_level', 20)
        self.critical_battery = safety.get('auto_land_battery_threshold', 10)
        self.auto_land_low_battery = safety.get('auto_land_low_battery', True)
        self.max_flight_time = safety.get('max_flight_time', 1200)
        self.connection_timeout_action = safety.get('connection_timeout_action', 'land')
        self.connection_timeout = config.get('network', {}).get('connection_timeout', 5.0)
        self.max_altitude = config.get('flight', {}).get('max_altitude', 500)
        
        # Safety state
        self.safety_enabled = True
        self.emergency_stop_triggered = False
        self.auto_landing_triggered = False
        self.link_lost = False
        
        # Monitoring threads
        self.monitor_thread: Optional[threading.Thread] = None
        self.monitoring_active = False
        self._shutdown = False
        self._land_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        # Monitoring data
        self.flight_start_time: Optional[float] = None
        self.last_telemetry_time = 0.0
        self.last_heartbeat_time = time.time()
        self._was_flying = False
        self._alerts_raised = set()      # latched condition keys for this flight
        
        # Callbacks
        self.safety_alert_callback: Optional[Callable] = None
        self.emergency_callback: Optional[Callable] = None
        
        # Safety history
        self.safety_events = []
        
        self.logger.info("Safety manager initialized")
    
    # ------------------------------------------------------------ lifecycle
    
    def start_monitoring(self):
        """Start safety monitoring"""
        if self.monitoring_active:
            return
        
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._safety_monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        self.logger.info("Safety monitoring started")
    
    def stop_monitoring(self):
        """Stop safety monitoring"""
        self.monitoring_active = False
        self._shutdown = True   # also ends any automatic-landing retries
        
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        
        self.logger.info("Safety monitoring stopped")
    
    def _safety_monitor_loop(self):
        """Main safety monitoring loop"""
        while self.monitoring_active:
            try:
                self.check()
            except Exception as e:
                self.logger.error(f"Safety monitor error: {e}")
            time.sleep(self.CHECK_INTERVAL)
    
    def check(self):
        """Run every check once. Called by the monitor loop; callable directly in tests."""
        if not self.controller:
            return
        self._track_flight_state()
        if not self.safety_enabled or not self.controller.is_flying:
            return
        self._check_battery_safety()
        self._check_flight_time_safety()
        self._check_connection_safety()
        self._check_altitude_safety()
    
    def _track_flight_state(self):
        flying = bool(self.controller.is_flying)
        if flying and not self._was_flying:
            self.on_takeoff()
        elif not flying and self._was_flying:
            self.on_landing()
    
    # --------------------------------------------------------------- checks
    
    def _check_battery_safety(self):
        """Check battery levels and take action if needed"""
        battery_level = self.controller.current_battery
        if battery_level is None:
            return
        
        if battery_level <= self.critical_battery:
            if self.auto_land_low_battery:
                if self._raise_once('battery_critical', SafetyLevel.EMERGENCY,
                                    f"Critical battery level: {battery_level}% - landing now",
                                    "auto_land"):
                    self._request_landing("Critical battery level")
            else:
                self._raise_once('battery_critical', SafetyLevel.CRITICAL,
                                 f"Critical battery level: {battery_level}% - LAND NOW "
                                 f"(automatic landing is disabled)", "warning")
        elif battery_level <= self.min_battery_flight:
            self._raise_once('battery_low', SafetyLevel.WARNING,
                             f"Low battery warning: {battery_level}%", "warning")
    
    def _check_flight_time_safety(self):
        """Check flight time limits"""
        if not self.flight_start_time:
            return
        
        flight_duration = time.time() - self.flight_start_time
        
        if flight_duration >= self.max_flight_time:
            if self._raise_once('flight_time_limit', SafetyLevel.CRITICAL,
                                f"Maximum flight time reached: {flight_duration:.0f}s - landing now",
                                "auto_land"):
                self._request_landing("Flight time limit reached")
        elif flight_duration >= self.max_flight_time * 0.8:  # 80% warning
            self._raise_once('flight_time_warning', SafetyLevel.WARNING,
                             f"Flight time warning: {flight_duration:.0f}s of {self.max_flight_time}s",
                             "warning")
    
    def _check_connection_safety(self):
        """Detect a lost link and apply the configured action."""
        connection = getattr(self.controller, 'connection', None)
        connected = bool(connection and connection.is_connected)
        last = self._last_link_activity()
        silent_for = time.time() - last if last else float('inf')
        lost = (not connected) or silent_for > self.connection_timeout
        
        if lost and not self.link_lost:
            self.link_lost = True
            action = self.connection_timeout_action
            detail = ("landing as soon as the link returns" if action == 'land'
                      else "aircraft left to hover")
            self._trigger_safety_event(
                SafetyLevel.CRITICAL,
                f"Telemetry link lost ({silent_for:.0f}s silent) - {detail}; "
                f"firmware watchdog will land it if the link stays down",
                "connection_loss")
            if action == 'land':
                self._request_landing("Connection lost")
        elif not lost and self.link_lost:
            self.link_lost = False
            self._trigger_safety_event(SafetyLevel.INFO, "Telemetry link restored", "connection_restored")
    
    def _check_altitude_safety(self):
        """Check altitude safety limits"""
        current_altitude = self.controller.current_altitude or 0
        
        if current_altitude > self.max_altitude:
            self._raise_once('altitude_limit', SafetyLevel.WARNING,
                             f"Altitude limit exceeded: {current_altitude}cm > {self.max_altitude}cm",
                             "altitude_limit")
        elif current_altitude < self.max_altitude - self.ALTITUDE_HYSTERESIS:
            self._alerts_raised.discard('altitude_limit')
    
    def _check_emergency_conditions(self):
        """
        Kept for API compatibility. Deliberately takes no action: the old
        version cut the motors at 5% battery, which drops the aircraft.
        Low battery is handled by landing in _check_battery_safety().
        """
    
    def _last_link_activity(self) -> float:
        """Most recent evidence the aircraft is talking to us."""
        return max(self.last_telemetry_time,
                   getattr(self.controller, 'last_telemetry_time', 0.0) or 0.0)
    
    # -------------------------------------------------------------- actions
    
    def _request_landing(self, reason: str):
        """Start an automatic landing unless one is already under way."""
        with self._lock:
            if self._land_thread and self._land_thread.is_alive():
                return
            self.auto_landing_triggered = True
            self._land_thread = threading.Thread(target=self._emergency_land, args=(reason,), daemon=True)
            self._land_thread.start()
        if self.emergency_callback:
            try:
                self.emergency_callback(reason)
            except Exception as e:
                self.logger.error(f"Emergency callback error: {e}")
    
    def _emergency_land(self, reason: str):
        """
        Land, retrying while the aircraft is still airborne.
        
        Attempts are only made while the link is up (a command sent into a
        dead link just burns the controller's 15 s timeout). This never
        escalates to a motor cut: if landing keeps failing, the operator is
        told to take over.
        """
        self.logger.warning(f"Initiating automatic landing: {reason}")
        deadline = time.time() + self.LAND_GIVE_UP_AFTER
        
        while not self._shutdown and self.controller and self.controller.is_flying:
            if time.time() > deadline:
                self._trigger_safety_event(
                    SafetyLevel.EMERGENCY,
                    f"Automatic landing ({reason}) has not succeeded - TAKE MANUAL CONTROL",
                    "landing_failed")
                return
            if not self.link_lost:
                try:
                    if self.controller.land():
                        self._trigger_safety_event(SafetyLevel.INFO,
                                                   f"Automatic landing completed ({reason})",
                                                   "landing_complete")
                        return
                    self.logger.error("Automatic landing attempt failed - retrying")
                except Exception as e:
                    self.logger.error(f"Automatic landing error: {e}")
            time.sleep(self.LAND_RETRY_INTERVAL)
    
    def trigger_emergency_stop(self, reason: str = "Emergency stop activated") -> bool:
        """
        OPERATOR emergency stop: cut the motors now.
        
        Always sends, even if pressed before; a second press must not be
        swallowed. Nothing in this class calls it automatically.
        
        Returns:
            bool: True if the emergency command was issued
        """
        self.emergency_stop_triggered = True
        self._trigger_safety_event(SafetyLevel.EMERGENCY, reason, "emergency_stop")
        
        try:
            if self.controller:
                return bool(self.controller.emergency_stop())
            self.logger.error("No controller available for emergency stop")
            return False
        except Exception as e:
            self.logger.error(f"Emergency stop error: {e}")
            return False
    
    # --------------------------------------------------------------- events
    
    def _raise_once(self, key: str, level: SafetyLevel, message: str, event_type: str) -> bool:
        """Raise an alert the first time a condition occurs this flight.
        Returns True if it was raised now (so the caller acts only once)."""
        if key in self._alerts_raised:
            return False
        self._alerts_raised.add(key)
        self._trigger_safety_event(level, message, event_type)
        return True
    
    def _trigger_safety_event(self, level: SafetyLevel, message: str, event_type: str):
        """
        Trigger safety event and notify callbacks
        
        Args:
            level: Safety level
            message: Event message
            event_type: Type of safety event
        """
        event = {
            'timestamp': time.time(),
            'level': level,
            'message': message,
            'type': event_type
        }
        
        self.safety_events.append(event)
        
        # Limit event history
        if len(self.safety_events) > 100:
            self.safety_events = self.safety_events[-50:]
        
        # Log the event
        log_method = {
            SafetyLevel.INFO: self.logger.info,
            SafetyLevel.WARNING: self.logger.warning,
            SafetyLevel.CRITICAL: self.logger.error,
            SafetyLevel.EMERGENCY: self.logger.critical
        }.get(level, self.logger.info)
        
        log_method(f"SAFETY {level.value.upper()}: {message}")
        
        # Notify callback
        if self.safety_alert_callback:
            try:
                self.safety_alert_callback(event)
            except Exception as e:
                self.logger.error(f"Safety callback error: {e}")
    
    # ---------------------------------------------------------------- hooks
    
    def on_takeoff(self):
        """Called when drone takes off (also detected automatically)"""
        if self._was_flying:
            return
        self._was_flying = True
        self.flight_start_time = time.time()
        self.emergency_stop_triggered = False
        self.auto_landing_triggered = False
        self.link_lost = False
        self._alerts_raised.clear()
        
        self._trigger_safety_event(
            SafetyLevel.INFO,
            "Flight started - safety monitoring active",
            "takeoff"
        )
    
    def on_landing(self):
        """Called when drone lands (also detected automatically)"""
        if not self._was_flying:
            return
        self._was_flying = False
        duration = time.time() - self.flight_start_time if self.flight_start_time else 0
        self.flight_start_time = None
        self._alerts_raised.clear()
        
        self._trigger_safety_event(
            SafetyLevel.INFO,
            f"Flight ended after {duration:.0f}s - safety monitoring standby",
            "landing"
        )
    
    def on_telemetry_update(self, telemetry_data: Dict[str, Any]):
        """
        Called when telemetry data is received
        
        Args:
            telemetry_data: Parsed telemetry data
        """
        if telemetry_data:
            self.last_telemetry_time = time.time()
    
    def on_heartbeat(self):
        """Called when heartbeat is received"""
        self.last_heartbeat_time = time.time()
    
    def set_safety_alert_callback(self, callback: Callable):
        """Set callback for safety alerts (called from a background thread)"""
        self.safety_alert_callback = callback
    
    def set_emergency_callback(self, callback: Callable):
        """Set callback for when an automatic landing starts"""
        self.emergency_callback = callback
    
    def get_safety_status(self) -> Dict[str, Any]:
        """
        Get current safety status
        
        Returns:
            dict: Safety status information
        """
        return {
            'safety_enabled': self.safety_enabled,
            'monitoring_active': self.monitoring_active,
            'emergency_stop_triggered': self.emergency_stop_triggered,
            'auto_landing_triggered': self.auto_landing_triggered,
            'auto_landing_in_progress': bool(self._land_thread and self._land_thread.is_alive()),
            'link_lost': self.link_lost,
            'flight_time': time.time() - self.flight_start_time if self.flight_start_time else 0,
            'max_flight_time': self.max_flight_time,
            'battery_critical_threshold': self.critical_battery,
            'recent_events': self.safety_events[-10:] if self.safety_events else []
        }
    
    def enable_safety(self):
        """Enable safety monitoring"""
        self.safety_enabled = True
        self.logger.info("Safety monitoring enabled")
    
    def disable_safety(self):
        """Disable safety monitoring (use with caution!)"""
        self.safety_enabled = False
        self.logger.warning("Safety monitoring DISABLED - use with extreme caution!")
    
    def reset_safety_state(self):
        """Reset safety state (after landing)"""
        self.emergency_stop_triggered = False
        self.auto_landing_triggered = False
        self.link_lost = False
        self.flight_start_time = None
        self._alerts_raised.clear()
        
        self.logger.info("Safety state reset")
