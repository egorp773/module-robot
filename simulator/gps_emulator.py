#!/usr/bin/env python3
"""
GPS emulator - generates fake NMEA sentences for ESP32.
ESP32 reads these as if from real GPS module.
"""

import math
import random
from dataclasses import dataclass
from typing import Tuple, Optional


# RTK GPS parameters
RTK_FIXED_HACC_MM = 50  # mm, RTK Fixed accuracy
RTK_FLOAT_HACC_MM = 300  # mm, RTK Float accuracy

# GPS simulation parameters
GPS_NOISE_RTK_FIXED = 0.015  # meters, ~15mm 1-sigma
GPS_NOISE_RTK_FLOAT = 0.050  # meters, ~50mm 1-sigma
GPS_NOISE_DEGRADED = 0.200  # meters, degraded

# Update rate
GPS_UPDATE_HZ = 10  # 10 Hz (100ms intervals)


@dataclass
class GpsEmulator:
    """
    Generates fake NMEA/GPS data from robot physics position.
    Simulates RTK GPS with fix quality levels.
    """
    # Reference origin (in degrees)
    origin_lat: float = 55.0
    origin_lon: float = 37.0
    origin_heading: float = 0.0  # Heading at origin for calibration

    # Current state
    fix_quality: int = 0  # 0=none, 1=GPS, 2=SBAS, 4=RTK Fixed, 5=RTK Float
    carrier: int = 0  # 0=none, 1=RTK Float, 2=RTK Fixed
    num_sv: int = 15  # Number of satellites
    h_acc: float = 50.0  # Horizontal accuracy in mm
    v_acc: float = 100.0  # Vertical accuracy in mm

    # Latency simulation
    position_buffer: list = None
    buffer_size: int = 2  # 2 updates = 200ms latency at 10Hz

    # Noise state
    noise_seed: int = 0

    def __post_init__(self):
        self.position_buffer = []

    def set_origin(self, lat: float, lon: float):
        """Set GPS origin (reference point for local coordinates)"""
        self.origin_lat = lat
        self.origin_lon = lon

    def set_fix_quality(self, quality: int, carrier: int = 0):
        """Set GPS fix quality: 0=none, 4=RTK Fixed, 5=RTK Float"""
        self.fix_quality = quality
        self.carrier = carrier
        if quality == 4:  # RTK Fixed
            self.h_acc = RTK_FIXED_HACC_MM
            self.carrier = 2
        elif quality == 5:  # RTK Float
            self.h_acc = RTK_FLOAT_HACC_MM
            self.carrier = 1
        else:
            self.h_acc = 2000.0
            self.carrier = 0

    def update(self, x: float, y: float, heading: float,
                speed: float = 0.0, vel_e: float = 0.0, vel_n: float = 0.0,
                add_noise: bool = True) -> Tuple[float, float]:
        """
        Update GPS with new position. Returns (lat, lon) with noise.
        """
        # Convert local coords to lat/lon
        lat, lon = self.local_to_geodetic(x, y)

        # Add position noise based on fix quality
        if add_noise:
            if self.fix_quality == 4:  # RTK Fixed
                noise = GPS_NOISE_RTK_FIXED
            elif self.fix_quality == 5:  # RTK Float
                noise = GPS_NOISE_RTK_FLOAT
            else:
                noise = GPS_NOISE_DEGRADED

            noise_x = random.gauss(0, noise)
            noise_y = random.gauss(0, noise)

            # Convert noise to lat/lon
            lat += noise_y / 111132.92  # ~111km per degree
            lon += noise_x / (111132.92 * math.cos(math.radians(self.origin_lat)))

        # Add to position buffer for latency simulation
        self.position_buffer.append((lat, lon, heading, speed, vel_e, vel_n, self.h_acc))
        if len(self.position_buffer) > self.buffer_size:
            self.position_buffer.pop(0)

        return lat, lon

    def get_delayed_position(self) -> Optional[Tuple[float, float, float, float, float, float, float]]:
        """
        Get delayed position (simulates GPS latency).
        Returns (lat, lon, heading, speed, vel_e, vel_n, h_acc) or None.
        """
        if not self.position_buffer:
            return None
        return self.position_buffer[0]  # Oldest entry

    def local_to_geodetic(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert local ENU coordinates (meters from origin) to geodetic (lat/lon).
        x = East (+x is East)
        y = North (+y is North)
        """
        # Meters per degree at this latitude
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(self.origin_lat))
        m_per_deg_lon = 111412.84 * math.cos(math.radians(self.origin_lat))

        lat = self.origin_lat + y / m_per_deg_lat
        lon = self.origin_lon + x / m_per_deg_lon

        return lat, lon

    def generate_gga(self, lat: float, lon: float) -> str:
        """Generate GPGGA NMEA sentence"""
        # Format: $GPGGA,time,lat,N/S,lon,E/W,quality,numSV,hDOP,alt,M,sep,M,diffAge,diffStation*checksum

        # Quality field
        quality_str = str(self.fix_quality) if self.fix_quality else '0'

        # Number of satellites
        num_sv_str = str(self.num_sv) if self.num_sv else '0'

        # hDOP (horizontal dilution of precision)
        if self.fix_quality == 4:
            hdop = '0.5'
        elif self.fix_quality == 5:
            hdop = '1.0'
        else:
            hdop = '2.5'

        # Altitude (assume 0 for flat terrain)
        alt = '100.0'

        # Format lat/lon as NMEA requires
        lat_nmea = self._format_nmea_coord(lat, 'N', 'S')
        lon_nmea = self._format_nmea_coord(lon, 'E', 'W')

        # Time (UTC)
        time_str = '120000.00'

        sentence = f"$GPGGA,{time_str},{lat_nmea},{lon_nmea},{quality_str},{num_sv_str},{hdop},1.5,{alt},M,0.0,M,0.0,0000"
        checksum = self._nmea_checksum(sentence[1:])
        return f"{sentence}*{checksum}\r\n"

    def generate_rmc(self, lat: float, lon: float, speed_knots: float = 0.0,
                    heading_deg: float = 0.0) -> str:
        """Generate GPRMC NMEA sentence"""
        # Format: $GPRMC,time,status,lat,N/S,lon,E/W,speedKnots,headingDeg,date,magVar*checksum

        status = 'A' if self.fix_quality else 'V'  # A=active, V=void

        # Format lat/lon
        lat_nmea = self._format_nmea_coord(lat, 'N', 'S')
        lon_nmea = self._format_nmea_coord(lon, 'E', 'W')

        # Speed in knots
        speed_knots = speed_knots * 1.94384  # m/s to knots

        # Heading
        heading_deg = heading_deg % 360

        # Time and date
        time_str = '120000.00'
        date_str = '010124'  # Jan 1, 2024

        sentence = f"$GPRMC,{time_str},{status},{lat_nmea},{lon_nmea},{speed_knots:.1f},{heading_deg:.1f},{date_str},,W"
        checksum = self._nmea_checksum(sentence[1:])
        return f"{sentence}*{checksum}\r\n"

    def generate_vtg(self, heading_deg: float = 0.0, speed_knots: float = 0.0) -> str:
        """Generate GPVTG NMEA sentence"""
        # Format: $GPVTG,headingT,headingM,speedKnots,speedKmh*checksum

        speed_kmh = speed_knots * 1.852  # knots to km/h

        sentence = f"$GPVTG,{heading_deg:.1f},T,,M,{speed_knots:.1f},N,{speed_kmh:.1f},K"
        checksum = self._nmea_checksum(sentence[1:])
        return f"{sentence}*{checksum}\r\n"

    def generate_velned(self, vel_n: float, vel_e: float) -> Tuple[str, str]:
        """
        Generate pseudo-Ublox NAV-VELNED data as hex strings.
        Actually returns GPNTR (custom) for our ESP32 parser.
        """
        # For UBX protocol simulation, we need to generate binary NAV-VELNED
        # For simplicity, we'll use NMEA-style sentences that the ESP32 can parse

        speed_3d = math.sqrt(vel_n**2 + vel_e**2)
        heading_2d = math.degrees(math.atan2(vel_e, vel_n)) if speed_3d > 0.01 else 0
        if heading_2d < 0:
            heading_2d += 360

        return speed_3d, heading_2d

    def _format_nmea_coord(self, coord: float, pos_dir: str, neg_dir: str) -> str:
        """Format coordinate for NMEA (DDMM.MMMMM format)"""
        if coord < 0:
            coord = -coord
            dir_char = neg_dir
        else:
            dir_char = pos_dir

        degrees = int(coord)
        minutes = (coord - degrees) * 60

        return f"{degrees:02d}{minutes:07.4f},{dir_char}"

    def _nmea_checksum(self, sentence: str) -> str:
        """Calculate NMEA checksum"""
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return f"{checksum:02X}"

    def generate_all_sentences(self, x: float, y: float, heading: float,
                               speed: float, vel_e: float, vel_n: float) -> list:
        """
        Generate all NMEA sentences for one update cycle.
        Returns list of strings ready to send to ESP32.
        """
        lat, lon = self.update(x, y, heading, speed, vel_e, vel_n)

        sentences = []
        sentences.append(self.generate_gga(lat, lon))
        sentences.append(self.generate_rmc(lat, lon, speed, heading))
        sentences.append(self.generate_vtg(heading, speed))

        return sentences


def create_gps_emulator(lat: float = 55.0, lon: float = 37.0) -> GpsEmulator:
    """Factory function to create GPS emulator with default settings"""
    emulator = GpsEmulator()
    emulator.set_origin(lat, lon)
    emulator.set_fix_quality(4)  # RTK Fixed by default
    return emulator
