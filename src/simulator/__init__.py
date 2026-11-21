"""Flight Simulator Mode: a simulated drone speaking the ground station's UDP protocol."""

from .sim_drone import SimulatedDrone, SimState

__all__ = ["SimulatedDrone", "SimState"]
