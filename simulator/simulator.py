#!/usr/bin/env python3
"""
HITL Simulator for RTK Robot
============================

Python = virtual world (physics, GPS emulator, visualization)
ESP32 = real brain (navigation, state machine, motor control)

Usage:
    python simulator.py --hitl --ip 192.168.31.222 --gps-port 8888
    python simulator.py --sitl --waypoints 0,0,5,0,5,5

Architecture:
    ESP32 firmware
    ↓ reads fake GPS/NMEA
    ↓ runs navigation algorithm
    ↓ outputs motor PWM commands
    ↓ Python reads PWM
    ↓ Python updates physics
    ↓ Python generates new fake GPS
    ↓ Python sends GPS back to ESP32
    ↓ repeat
"""

import argparse
import asyncio
import json
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional, Callable
from enum import Enum

# Local imports
from physics import RobotPhysics, pwm_to_velocity, update_position, normalize_angle_360, normalize_angle_180
from gps_emulator import GpsEmulator, create_gps_emulator


# Robot parameters (must match ESP32 firmware)
WHEELBASE = 0.38  # meters
MAX_PWM = 70  # Maximum PWM percentage
DT = 0.05  # Simulation timestep (50ms, matches NAV_LOOP_MS in firmware)


class NavState(Enum):
    IDLE = "IDLE"
    CALIBRATING = "CALIBRATING"
    MOVING = "MOVING"
    APPROACHING = "APPROACHING"
    ARRIVED = "ARRIVED"
    RECOVERING = "RECOVERING"
    ERROR = "ERROR"


@dataclass
class Waypoint:
    x: float
    y: float


@dataclass
class Telemetry:
    """Robot telemetry data received from ESP32"""
    lat: float = 0.0
    lon: float = 0.0
    heading: float = 0.0
    fix_type: int = 0
    carrier: int = 0
    h_acc: float = 999999
    speed: float = 0.0
    state: NavState = NavState.IDLE
    wp_index: int = 0
    wp_count: int = 0
    dist_to_end: float = 0.0
    heading_error: float = 0.0
    progress: float = 0.0
    cross_track: float = 0.0
    status: str = ""
    motor_left: int = 0
    motor_right: int = 0
    bat_voltage: int = 0
    temp: int = 0
    timestamp: float = 0.0


@dataclass
class SimState:
    """Simulation state"""
    running: bool = True
    paused: bool = False
    time: float = 0.0
    waypoints: List[Waypoint] = None
    current_wp_index: int = 0

    def __post_init__(self):
        if self.waypoints is None:
            self.waypoints = []


class HitlSimulator:
    """
    HITL Simulator - connects to real ESP32 firmware.

    Data flow:
    1. Receive motor commands from ESP32 (via WebSocket)
    2. Update physics simulation
    3. Generate fake GPS data
    4. Send fake GPS to ESP32 (via TCP/Serial)
    5. Visualize results
    """

    def __init__(self, esp32_ip: str = "192.168.31.222",
                 esp32_gps_port: int = 8888,
                 esp32_ws_port: int = 81,
                 origin_lat: float = 55.0,
                 origin_lon: float = 37.0,
                 add_gps_noise: bool = True):
        self.esp32_ip = esp32_ip
        self.esp32_gps_port = esp32_gps_port
        self.esp32_ws_port = esp32_ws_port

        # Simulation components
        self.physics = RobotPhysics()
        self.gps = create_gps_emulator(origin_lat, origin_lon)
        self.gps.add_noise = add_gps_noise

        # ESP32 connection
        self.gps_socket = None
        self.ws_connected = False
        self.ws_thread = None

        # State
        self.state = SimState()
        self.telemetry = Telemetry()
        self.telemetry_history = []

        # WebSocket client for ESP32
        self.ws_reader = None
        self.ws_writer = None

        # Motor commands from ESP32
        self.last_motor_left = 0
        self.last_motor_right = 0
        self.last_motor_time = 0

        # Timing
        self.last_update = None
        self.loop_hz = 20  # Physics update rate

    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """Set navigation waypoints"""
        self.state.waypoints = [Waypoint(x, y) for x, y in waypoints]
        self.state.current_wp_index = 0
        print(f"Set {len(waypoints)} waypoints")

    def connect(self) -> bool:
        """Connect to ESP32"""
        print(f"Connecting to ESP32 at {self.esp32_ip}...")

        # Connect GPS TCP
        try:
            self.gps_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.gps_socket.connect((self.esp32_ip, self.esp32_gps_port))
            self.gps_socket.settimeout(1.0)
            print(f"GPS TCP connected on port {self.esp32_gps_port}")
        except Exception as e:
            print(f"GPS connection failed: {e}")
            self.gps_socket = None

        # Connect WebSocket (for telemetry reading)
        try:
            import websockets
            # We'll implement a simple WebSocket client ourselves
            self.ws_connected = self._ws_connect()
        except Exception as e:
            print(f"WebSocket not available: {e}")
            print("Telemetry will be simulated")
            self.ws_connected = False

        return self.gps_socket is not None

    def _ws_connect(self) -> bool:
        """Connect to ESP32 WebSocket for telemetry"""
        # Simple WebSocket implementation
        # In production, use: websockets library
        try:
            import websockets
            uri = f"ws://{self.esp32_ip}:{self.esp32_ws_port}/ws"
            # This would be async in production
            print(f"WebSocket: would connect to {uri}")
            return False
        except:
            return False

    def send_gps_data(self, x: float, y: float, heading: float, speed: float):
        """Send fake GPS NMEA data to ESP32"""
        if self.gps_socket is None:
            return

        try:
            sentences = self.gps.generate_all_sentences(
                x, y, heading, speed,
                self.physics.vel_e, self.physics.vel_n
            )

            for sentence in sentences:
                self.gps_socket.sendall(sentence.encode('ascii'))

        except Exception as e:
            print(f"GPS send error: {e}")

    def read_motor_commands(self) -> Tuple[int, int]:
        """
        Read motor commands from ESP32.
        Returns (left_pwm, right_pwm) as integers -70 to +70.
        In real mode, this reads from ESP32.
        In simulation mode, we calculate from WebSocket telemetry.
        """
        # In HITL mode, ESP32 sends motor telemetry via WebSocket
        # We parse it in the telemetry loop
        return self.last_motor_left, self.last_motor_right

    def update_physics(self, dt: float = DT):
        """Update robot physics based on motor commands"""
        left, right = self.read_motor_commands()

        # Update position
        update_position(self.physics, left, right, dt, add_noise=self.gps.add_noise)

        # Store for later
        self.last_motor_time = time.time()

    def parse_telemetry(self, data: str):
        """Parse telemetry message from ESP32 WebSocket"""
        parts = data.split(',')

        if len(parts) < 2:
            return

        msg_type = parts[0]

        try:
            if msg_type == "TEL":
                # GPS telemetry
                self.telemetry.lat = float(parts[1])
                self.telemetry.lon = float(parts[2])
                self.telemetry.heading = float(parts[4])
                self.telemetry.fix_type = int(parts[5])
                self.telemetry.carrier = 2 if parts[6] == "fixed" else (1 if parts[6] == "float" else 0)
                self.telemetry.h_acc = float(parts[9])
                self.telemetry.speed = float(parts[11])

            elif msg_type == "NAV":
                # Navigation telemetry
                state_map = {
                    'IDLE': NavState.IDLE,
                    'CALIBRATING': NavState.CALIBRATING,
                    'MOVING': NavState.MOVING,
                    'APPROACHING': NavState.APPROACHING,
                    'ARRIVED': NavState.ARRIVED,
                    'RECOVERING': NavState.RECOVERING,
                    'ERROR': NavState.ERROR,
                }
                self.telemetry.state = state_map.get(parts[1], NavState.IDLE)
                self.telemetry.wp_index = int(parts[2])
                self.telemetry.wp_count = int(parts[3])
                self.telemetry.dist_to_end = float(parts[4])
                self.telemetry.heading_error = float(parts[5])
                self.telemetry.progress = float(parts[7])
                self.telemetry.cross_track = float(parts[8])
                self.telemetry.status = parts[9]

            elif msg_type == "MOTOR":
                # Motor telemetry
                self.telemetry.motor_left = int(parts[1])
                self.telemetry.motor_right = int(parts[2])
                self.last_motor_left = self.telemetry.motor_left
                self.last_motor_right = self.telemetry.motor_right

                if len(parts) >= 7:
                    self.telemetry.bat_voltage = int(parts[5])
                    self.telemetry.temp = int(parts[6])

            elif msg_type == "STATE":
                # Connection state
                print(f"ESP32 state: {parts[1]}")

        except (ValueError, IndexError) as e:
            print(f"Telemetry parse error: {e} in '{data}'")

        self.telemetry.timestamp = time.time()
        self.telemetry_history.append(asdict(self.telemetry))

    def run_telemetry_loop(self):
        """Background thread to receive telemetry from ESP32"""
        if not self.ws_connected:
            return

        while self.state.running:
            try:
                # In production, use WebSocket library
                # data = await ws.recv()
                # self.parse_telemetry(data)
                time.sleep(0.1)
            except Exception as e:
                print(f"Telemetry error: {e}")
                time.sleep(1)

    def send_waypoints(self):
        """Send waypoints to ESP32 via WebSocket"""
        if not self.ws_connected:
            return

        try:
            # ROUTE_BEGIN,count,originLat,originLon
            origin_lat = self.gps.origin_lat
            origin_lon = self.gps.origin_lon
            self._ws_send(f"ROUTE_BEGIN,{len(self.state.waypoints)},{origin_lat},{origin_lon}")

            # ROUTE_WP,idx,x,y
            for i, wp in enumerate(self.state.waypoints):
                # Convert local to geodetic
                lat, lon = self.gps.local_to_geodetic(wp.x, wp.y)
                self._ws_send(f"ROUTE_WP,{i},{wp.x},{wp.y}")

            self._ws_send("ROUTE_END")

        except Exception as e:
            print(f"Waypoint send error: {e}")

    def _ws_send(self, message: str):
        """Send message via WebSocket"""
        # Placeholder - implement with websockets library
        pass

    def step(self) -> bool:
        """
        Execute one simulation step.
        Returns True if simulation should continue.
        """
        if not self.state.running or self.state.paused:
            return True

        # Calculate delta time
        now = time.time()
        if self.last_update is None:
            dt = DT
        else:
            dt = now - self.last_update
        self.last_update = now

        # Cap dt to prevent spiral of death, ensure minimum dt
        dt = min(dt, 0.2)
        if dt <= 0:
            dt = DT

        # Update physics
        self.update_physics(dt)

        # Send fake GPS to ESP32
        self.send_gps_data(
            self.physics.x, self.physics.y,
            self.physics.heading, self.physics.vel_n  # speed
        )

        # Update simulation time
        self.state.time += dt

        return self.state.running

    def reset(self):
        """Reset simulation"""
        self.physics = RobotPhysics()
        self.state.time = 0.0
        self.state.current_wp_index = 0
        self.telemetry_history = []

    def stop(self):
        """Stop simulation"""
        self.state.running = False
        if self.gps_socket:
            self.gps_socket.close()
        if self.ws_writer:
            self.ws_writer.close()

    def get_stats(self) -> dict:
        """Get simulation statistics"""
        return {
            'time': self.state.time,
            'position': (self.physics.x, self.physics.y),
            'heading': self.physics.heading,
            'state': self.telemetry.state.value,
            'motor_left': self.last_motor_left,
            'motor_right': self.last_motor_right,
            'heading_error': self.telemetry.heading_error,
            'cross_track': self.telemetry.cross_track,
            'dist_to_end': self.telemetry.dist_to_end,
        }


class SitlSimulator(HitlSimulator):
    """
    SITL Simulator - standalone simulation without ESP32.
    Uses same physics but simulates motor commands internally.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # SITL-specific: no ESP32 needed
        self.gps_socket = None
        self.ws_connected = False

        # Navigation simulation (simplified - for testing without ESP32)
        self.current_target = None
        self.nav_loop_counter = 0

        # For SITL: simulate motor control loop
        self._sim_motor_left = 0
        self._sim_motor_right = 0
        self._sim_ramp_rate = 5  # PWM change per step

    def set_target(self, x: float, y: float):
        """Set single target point and start navigation"""
        self.state.waypoints = [Waypoint(x, y)]
        self.state.current_wp_index = 0
        self.current_target = Waypoint(x, y)
        # Start navigation (simulates ESP32 starting)
        self.telemetry.state = NavState.MOVING

    def set_waypoints(self, waypoints):
        """Override to also set current_target for SITL"""
        # Call parent (converts tuples to Waypoints)
        self.state.waypoints = waypoints  # Parent expects tuples, but we pass Waypoints
        self.state.waypoints = [Waypoint(x, y) for x, y in waypoints] if waypoints else []
        self.state.current_wp_index = 0
        if waypoints:
            self.current_target = self.state.waypoints[-1]
        print(f"Set {len(waypoints)} waypoints")
        self.telemetry.state = NavState.MOVING

    def update_physics(self, dt: float = DT):
        """Update physics with simulated navigation"""
        # Simple proportional controller for SITL testing
        if self.current_target is None:
            left, right = 0, 0
        else:
            dx = self.current_target.x - self.physics.x
            dy = self.current_target.y - self.physics.y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < 0.1:
                # Arrived
                left, right = 0, 0
                self.telemetry.state = NavState.ARRIVED
            else:
                # Calculate heading error
                desired_heading = math.degrees(math.atan2(dx, dy)) % 360
                heading_error = normalize_angle_180(desired_heading - self.physics.heading)

                # Simple steering
                K_HEADING = 0.5
                K_CROSSTRACK = 0.0  # No cross-track for single point

                forward = 35  # 50% speed
                turn = K_HEADING * heading_error

                left = forward + turn
                right = forward - turn

                # Clamp
                left = max(-MAX_PWM, min(MAX_PWM, left))
                right = max(-MAX_PWM, min(MAX_PWM, right))

                # Update telemetry
                self.telemetry.heading_error = heading_error
                self.telemetry.dist_to_end = dist

        # Store motor commands
        self.last_motor_left = left
        self.last_motor_right = right

        # Update physics
        from physics import update_position
        update_position(self.physics, left, right, dt, add_noise=self.gps.add_noise)


def run_headless(scenario: dict) -> dict:
    """
    Run headless simulation for automated testing.
    Returns results as JSON-serializable dict.
    """
    sim = SitlSimulator(add_gps_noise=scenario.get('add_gps_noise', True))

    # Set waypoints
    waypoints = scenario.get('waypoints', [(5.0, 0.0)])
    sim.set_waypoints(waypoints)

    # Run simulation
    max_time = scenario.get('max_time', 60.0)
    start_time = time.time()

    while sim.state.running and sim.state.time < max_time:
        sim.step()

        # Check if arrived
        if sim.telemetry.state == NavState.ARRIVED:
            break

    # Calculate results
    final_x, final_y = sim.physics.x, sim.physics.y
    target = waypoints[-1] if waypoints else (0, 0)
    error = math.sqrt((final_x - target[0])**2 + (final_y - target[1])**2)

    # Calculate oscillation count (direction changes)
    oscillation_count = 0
    if len(sim.telemetry_history) > 10:
        prev_heading = None
        for t in sim.telemetry_history:
            h = t.get('heading_error', 0)
            if prev_heading is not None and h is not None:
                if prev_heading * h < 0 and abs(prev_heading) > 5 and abs(h) > 5:
                    oscillation_count += 1
            prev_heading = h

    return {
        'success': error < 0.1,  # Within 10cm
        'final_position': (final_x, final_y),
        'target_position': target,
        'error_m': error,
        'time_s': sim.state.time,
        'oscillation_count': oscillation_count,
        'final_state': sim.telemetry.state.value,
    }


def main():
    parser = argparse.ArgumentParser(description='RTK Robot HITL Simulator')
    parser.add_argument('--sitl', action='store_true', help='SITL mode (no ESP32)')
    parser.add_argument('--hitl', action='store_true', help='HITL mode (with ESP32)')
    parser.add_argument('--ip', default='192.168.31.222', help='ESP32 IP address')
    parser.add_argument('--gps-port', type=int, default=8888, help='GPS TCP port')
    parser.add_argument('--ws-port', type=int, default=81, help='WebSocket port')
    parser.add_argument('--waypoints', type=str, default='', help='Waypoints: x1,y1,x2,y2,...')
    parser.add_argument('--lat', type=float, default=55.0, help='Origin latitude')
    parser.add_argument('--lon', type=float, default=37.0, help='Origin longitude')
    parser.add_argument('--no-noise', action='store_true', help='Disable GPS noise')
    parser.add_argument('--headless', action='store_true', help='Run without visualization')
    parser.add_argument('--max-time', type=float, default=60.0, help='Max simulation time (s)')

    args = parser.parse_args()

    # Create simulator
    if args.sitl:
        sim = SitlSimulator(
            origin_lat=args.lat,
            origin_lon=args.lon,
            add_gps_noise=not args.no_noise
        )
    else:
        sim = HitlSimulator(
            esp32_ip=args.ip,
            esp32_gps_port=args.gps_port,
            esp32_ws_port=args.ws_port,
            origin_lat=args.lat,
            origin_lon=args.lon,
            add_gps_noise=not args.no_noise
        )

    # Parse waypoints
    if args.waypoints:
        try:
            coords = [float(x) for x in args.waypoints.split(',')]
            waypoints = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]
            sim.set_waypoints(waypoints)
        except (ValueError, IndexError) as e:
            print(f"Invalid waypoints: {e}")
            return 1

    if args.headless:
        # Headless mode
        result = run_headless({
            'waypoints': [(5.0, 0.0)],  # Default test
            'add_gps_noise': not args.no_noise,
            'max_time': args.max_time
        })
        print(json.dumps(result, indent=2))
        return 0

    # Interactive mode (requires matplotlib)
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        HAS_MATPLOTLIB = True
    except ImportError:
        print("matplotlib not available, running headless")
        HAS_MATPLOTLIB = False
        args.headless = True

    if HAS_MATPLOTLIB:
        run_interactive(sim)
    else:
        result = run_headless({
            'waypoints': [(5.0, 0.0)],
            'add_gps_noise': not args.no_noise,
            'max_time': args.max_time
        })
        print(json.dumps(result, indent=2))

    return 0


def run_interactive(sim):
    """Run interactive visualization"""
    import matplotlib.pyplot as plt

    fig, (ax_map, ax_info) = plt.subplots(1, 2, figsize=(14, 7))

    # Map setup
    ax_map.set_aspect('equal')
    ax_map.grid(True, alpha=0.3)
    ax_map.set_xlabel('X (meters)')
    ax_map.set_ylabel('Y (meters)')

    # Data lines
    trajectory, = ax_map.plot([], [], 'b-', alpha=0.5, linewidth=1)
    robot_marker = ax_map.scatter([], [], c='green', s=200, marker='o', zorder=10)
    waypoint_markers = ax_map.scatter([], [], c='red', s=100, marker='^', zorder=5)
    target_marker = ax_map.scatter([], [], c='orange', s=150, marker='*', zorder=8)

    # Info panel
    ax_info.axis('off')
    info_text = ax_info.text(0.05, 0.95, '', transform=ax_info.transAxes,
                            verticalalignment='top', fontfamily='monospace',
                            fontsize=10)

    plt.tight_layout()

    # State
    position_history = []
    running = True

    def on_key(event):
        nonlocal running
        if event.key == ' ':
            sim.state.paused = not sim.state.paused
        elif event.key == 'r':
            sim.reset()
            position_history.clear()
        elif event.key == 'q':
            running = False
        elif event.key == 's':
            # Start navigation
            if sim.ws_connected or isinstance(sim, SitlSimulator):
                pass  # Would send START command

    fig.canvas.mpl_connect('key_press_event', on_key)

    def on_click(event):
        if event.inaxes == ax_map and event.button == 1:
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                sim.set_target(x, y)

    fig.canvas.mpl_connect('button_press_event', on_click)

    print("Controls: SPACE=pause, R=reset, Q=quit, Click=set target")

    while running and sim.state.running:
        sim.step()

        # Update visualization
        position_history.append((sim.physics.x, sim.physics.y))

        if len(position_history) > 2:
            xs, ys = zip(*position_history)
            trajectory.set_data(xs, ys)

        robot_marker.set_offsets([[sim.physics.x, sim.physics.y]])

        # Update waypoints
        if sim.state.waypoints:
            wpx = [wp.x for wp in sim.state.waypoints]
            wpy = [wp.y for wp in sim.state.waypoints]
            waypoint_markers.set_offsets([[x, y] for x, y in zip(wpx, wpy)])

            if sim.state.current_wp_index < len(sim.state.waypoints):
                target = sim.state.waypoints[sim.state.current_wp_index]
                target_marker.set_offsets([[target.x, target.y]])

        # Update info
        stats = sim.get_stats()
        info = f"""Time: {stats['time']:.1f}s
Position: ({stats['position'][0]:.2f}, {stats['position'][1]:.2f}) m
Heading: {stats['heading']:.1f}°
State: {stats['state']}

Motor: L={stats['motor_left']}% R={stats['motor_right']}%

Heading Error: {stats['heading_error']:.1f}°
Cross Track: {stats['cross_track']:.2f} m
Dist to End: {stats['dist_to_end']:.2f} m

SPACE=pause R=reset Q=quit"""

        if sim.state.paused:
            info = "[PAUSED]\n" + info

        info_text.set_text(info)

        # Auto-scale
        if position_history:
            all_x = [p[0] for p in position_history] + [wp.x for wp in sim.state.waypoints]
            all_y = [p[1] for p in position_history] + [wp.y for wp in sim.state.waypoints]
            margin = 2.0
            ax_map.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax_map.set_ylim(min(all_y) - margin, max(all_y) + margin)

        plt.pause(0.05)

    plt.close()
    sim.stop()


if __name__ == '__main__':
    sys.exit(main())
