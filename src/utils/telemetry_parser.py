"""
Telemetry Parser for the drone
Parses incoming telemetry data and extracts relevant information
"""

import re
import time
import logging
from typing import Dict, Any, Optional

class TelemetryParser:
    """Parses drone telemetry data into structured format"""
    
    # Accepts both the Tello-SDK state format
    #   "pitch:0;roll:0;yaw:0;vgx:0;...;h:80;bat:87;baro:12.34;time:5;agx:1.00;..."
    # and the space-separated form used in the docs/tests ("bat:85 h:120 temp:32").
    # Every "key:number" token is read independently, so field order and
    # separators don't matter, and "temph:" can never be mistaken for "h:".
    _TOKEN = re.compile(r'([A-Za-z_]+):(-?\d+(?:\.\d+)?)')

    # raw key -> (output key, type)
    _FIELDS = {
        'bat': ('battery', int),
        'h': ('altitude', int),
        'vgx': ('speed_x', int),
        'vgy': ('speed_y', int),
        'vgz': ('speed_z', int),
        'pitch': ('pitch', int),
        'roll': ('roll', int),
        'yaw': ('yaw', int),
        'agx': ('acceleration_x', float),
        'agy': ('acceleration_y', float),
        'agz': ('acceleration_z', float),
        'baro': ('barometer', float),
        'tof': ('time_of_flight', int),
        'time': ('motor_time', int),
        'temp': ('temperature', int),
        'templ': ('temperature_low', int),
        'temph': ('temperature', int),   # report the hotter sensor
    }

    def __init__(self):
        self.logger = logging.getLogger(__name__)

    def parse(self, telemetry_string: str) -> Dict[str, Any]:
        """
        Parse telemetry string into structured data

        Args:
            telemetry_string: Raw telemetry data from drone

        Returns:
            dict: Parsed telemetry data (empty if nothing recognisable)
        """
        parsed_data: Dict[str, Any] = {}

        try:
            for key, value in self._TOKEN.findall(telemetry_string):
                field = self._FIELDS.get(key.lower())
                if field is None:
                    continue
                name, cast = field
                parsed_data[name] = cast(float(value))

            if 'speed_x' in parsed_data and 'speed_y' in parsed_data:
                total_speed = (parsed_data['speed_x'] ** 2 + parsed_data['speed_y'] ** 2) ** 0.5
                parsed_data['speed_total'] = round(total_speed, 2)

            if parsed_data:
                parsed_data['timestamp'] = time.time()

            return parsed_data

        except Exception as e:
            self.logger.error(f"Error parsing telemetry: {e}")
            return {}

    def format_for_display(self, parsed_data: Dict[str, Any]) -> str:
        """
        Format parsed data for human-readable display
        
        Args:
            parsed_data: Parsed telemetry dictionary
            
        Returns:
            str: Formatted string for display
        """
        lines = []
        
        # Essential flight data
        if 'battery' in parsed_data:
            lines.append(f"Battery: {parsed_data['battery']}%")
        
        if 'altitude' in parsed_data:
            lines.append(f"Altitude: {parsed_data['altitude']} cm")
        
        if 'speed_total' in parsed_data:
            lines.append(f"Speed: {parsed_data['speed_total']} cm/s")
        
        # Attitude information
        if all(key in parsed_data for key in ['pitch', 'roll', 'yaw']):
            lines.append(f"Attitude: P:{parsed_data['pitch']}° R:{parsed_data['roll']}° Y:{parsed_data['yaw']}°")
        
        # Temperature
        if 'temperature' in parsed_data:
            lines.append(f"Temperature: {parsed_data['temperature']}°C")
        
        # Time of flight sensor
        if 'time_of_flight' in parsed_data:
            lines.append(f"Distance below: {parsed_data['time_of_flight']} cm")
        
        return " | ".join(lines) if lines else "No telemetry data"
    
    def get_critical_alerts(self, parsed_data: Dict[str, Any]) -> list:
        """
        Check for critical conditions that require attention
        
        Args:
            parsed_data: Parsed telemetry dictionary
            
        Returns:
            list: List of alert messages
        """
        alerts = []
        
        # Low battery warning
        if 'battery' in parsed_data and parsed_data['battery'] < 20:
            alerts.append(f"LOW BATTERY: {parsed_data['battery']}%")
        
        # Very low battery critical
        if 'battery' in parsed_data and parsed_data['battery'] < 10:
            alerts.append(f"CRITICAL BATTERY: {parsed_data['battery']}% - LAND IMMEDIATELY")
        
        # High temperature warning
        if 'temperature' in parsed_data and parsed_data['temperature'] > 80:
            alerts.append(f"HIGH TEMPERATURE: {parsed_data['temperature']}°C")
        
        # Extreme attitude warnings
        if 'pitch' in parsed_data and abs(parsed_data['pitch']) > 45:
            alerts.append(f"EXTREME PITCH: {parsed_data['pitch']}°")
        
        if 'roll' in parsed_data and abs(parsed_data['roll']) > 45:
            alerts.append(f"EXTREME ROLL: {parsed_data['roll']}°")
        
        # Low altitude warning (too close to ground)
        if 'time_of_flight' in parsed_data and parsed_data['time_of_flight'] < 20:
            alerts.append(f"LOW ALTITUDE: {parsed_data['time_of_flight']} cm")
        
        return alerts