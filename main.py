#!/usr/bin/env python3
"""
KOMPAS ground control station
Main application entry point for controlling WiFi RC drones
"""

import argparse
import sys
import os
import logging
from pathlib import Path

# Add src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from gui.main_window import DroneControllerGUI
from utils.logger import setup_logging
from config.settings import load_config

SIM_ENV_VAR = "KOMPAS_SIMULATOR"


def parse_args():
    parser = argparse.ArgumentParser(description="KOMPAS ground station")
    parser.add_argument("--sim", action="store_true",
                        help="Flight Simulator Mode: fly a simulated drone on this machine")
    parser.add_argument("--config", help="Path to a config YAML (overrides KOMPAS_CONFIG)")
    return parser.parse_args()


def simulator_requested(args, config) -> bool:
    env = os.environ.get(SIM_ENV_VAR, "").strip().lower() in ("1", "true", "yes", "on")
    return args.sim or env or config["simulator"]["enabled"]


def start_simulator(config):
    """Start the simulated drone and point the ground station at it."""
    from simulator import SimulatedDrone

    network, sim = config["network"], config["simulator"]
    drone = SimulatedDrone(
        bind_host="127.0.0.1",
        command_port=network["command_port"],
        state_port=network["state_port"],
        video_port=network["video_port"],
        telemetry_rate_hz=sim["telemetry_rate_hz"],
        video_enabled=sim["video_enabled"],
        video_fps=sim["video_fps"],
        start_battery=sim["start_battery"],
        battery_drain_per_min=sim["battery_drain_per_min"],
        command_watchdog_s=sim["command_watchdog_s"],
    ).start()

    # In-memory only: the config file on disk keeps the real drone's address.
    config["simulator"]["enabled"] = True
    network["drone_ip"] = "127.0.0.1"
    return drone


def main():
    """Main application entry point"""
    args = parse_args()

    # Setup logging
    setup_logging()
    logger = logging.getLogger(__name__)
    
    logger.info("Starting KOMPAS ground control station")
    
    sim_drone = None
    try:
        # Load configuration
        config = load_config(args.config)
        logger.info("Configuration loaded successfully")
        
        if simulator_requested(args, config):
            sim_drone = start_simulator(config)
            logger.warning("FLIGHT SIMULATOR MODE - no real aircraft is being controlled")
        
        # Create and run GUI application
        app = DroneControllerGUI(config, sim_drone=sim_drone)
        app.run()
        
    except Exception as e:
        logger.error(f"Failed to start application: {e}")
        sys.exit(1)
    finally:
        if sim_drone:
            sim_drone.stop()
    
    logger.info("KOMPAS ground control station shutdown complete")

if __name__ == "__main__":
    main()