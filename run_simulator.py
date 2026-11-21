#!/usr/bin/env python3
"""
Run KOMPAS's simulated drone on its own, without the GUI.

Usually you want `python main.py --sim`, which starts the simulator and the
ground station together. Run it standalone when the ground station is on
another machine or container, or to point a different client at it:

    python run_simulator.py --bind 0.0.0.0
    # then, on the ground-station machine, set network.drone_ip to this host
"""

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config.settings import load_config  # noqa: E402
from simulator import SimulatedDrone     # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bind", default="127.0.0.1",
                        help="address to listen for commands on (default 127.0.0.1; 0.0.0.0 for remote)")
    parser.add_argument("--config", help="config YAML to take ports and simulator settings from")
    parser.add_argument("--battery", type=float, help="starting battery %% (overrides config)")
    parser.add_argument("--no-video", action="store_true", help="don't stream simulated video")
    parser.add_argument("-v", "--verbose", action="store_true", help="log every command")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")

    config = load_config(args.config)
    network, sim = config["network"], config["simulator"]
    drone = SimulatedDrone(
        bind_host=args.bind,
        command_port=network["command_port"],
        state_port=network["state_port"],
        video_port=network["video_port"],
        telemetry_rate_hz=sim["telemetry_rate_hz"],
        video_enabled=sim["video_enabled"] and not args.no_video,
        video_fps=sim["video_fps"],
        start_battery=args.battery if args.battery is not None else sim["start_battery"],
        battery_drain_per_min=sim["battery_drain_per_min"],
        command_watchdog_s=sim["command_watchdog_s"],
    ).start()

    print("Simulated drone running. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        drone.stop()


if __name__ == "__main__":
    main()
