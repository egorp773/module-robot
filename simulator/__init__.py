"""
RTK Robot HITL Simulator
========================

Python = virtual world (physics, GPS emulator, visualization)
ESP32 = real brain (navigation, state machine, motor control)

Usage:
    from simulator import HitlSimulator, SitlSimulator
    sim = SitlSimulator()
    sim.set_waypoints([(5.0, 0.0)])
    while sim.state.running:
        sim.step()
"""

from .physics import RobotPhysics, pwm_to_velocity, compute_kinematics, update_position
from .gps_emulator import GpsEmulator, create_gps_emulator
from .simulator import HitlSimulator, SitlSimulator, NavState, Telemetry, Waypoint

__all__ = [
    'RobotPhysics',
    'pwm_to_velocity',
    'compute_kinematics',
    'update_position',
    'GpsEmulator',
    'create_gps_emulator',
    'HitlSimulator',
    'SitlSimulator',
    'NavState',
    'Telemetry',
    'Waypoint',
]
