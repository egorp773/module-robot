"""Atomic, owner-scoped route upload and validation.

No ROS imports live here. A completed route contains both geographic and local
map coordinates; the ROS node is responsible only for converting it to a Path.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import threading
from typing import Dict, Optional, Tuple


EARTH_MEAN_RADIUS_M = 6_371_008.8


class RouteMode(str, Enum):
    GEODETIC = "GEODETIC"
    LEGACY_LOCAL = "LEGACY_LOCAL"


class RouteError(ValueError):
    def __init__(self, code: str, detail: str = "") -> None:
        super().__init__(code)
        self.code = code
        self.detail = detail

    def response(self, command: str) -> str:
        suffix = f",{self.detail}" if self.detail else ""
        return f"ERR,{command},{self.code}{suffix}"


@dataclass(frozen=True)
class RoutePoint:
    index: int
    latitude: float
    longitude: float
    x_m: float
    y_m: float


@dataclass(frozen=True)
class StoredRoute:
    version: int
    mode: RouteMode
    origin_latitude: float
    origin_longitude: float
    points: Tuple[RoutePoint, ...]
    total_length_m: float


@dataclass(frozen=True)
class RouteSnapshot:
    status: str
    valid: bool
    expected_count: int
    received_count: int
    version: int
    error: str


@dataclass
class _Upload:
    owner: str
    mode: RouteMode
    expected_count: int
    origin: Optional[Tuple[float, float]]
    points: Dict[int, Tuple[float, float]]


def _validate_lat_lon(latitude: float, longitude: float) -> None:
    if not math.isfinite(latitude) or not math.isfinite(longitude):
        raise RouteError("NON_FINITE_COORDINATE")
    if latitude < -90.0 or latitude > 90.0:
        raise RouteError("LATITUDE_RANGE")
    if longitude < -180.0 or longitude > 180.0:
        raise RouteError("LONGITUDE_RANGE")


def _wrapped_longitude_delta_rad(from_lon: float, to_lon: float) -> float:
    delta = math.radians(to_lon - from_lon)
    return (delta + math.pi) % (2.0 * math.pi) - math.pi


def haversine_distance_m(
    first_lat: float, first_lon: float, second_lat: float, second_lon: float
) -> float:
    """Return great-circle distance with anti-meridian-safe longitude delta."""

    lat1 = math.radians(first_lat)
    lat2 = math.radians(second_lat)
    dlat = lat2 - lat1
    dlon = _wrapped_longitude_delta_rad(first_lon, second_lon)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * EARTH_MEAN_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def geodetic_to_local_m(
    origin_lat: float, origin_lon: float, latitude: float, longitude: float
) -> Tuple[float, float]:
    """Small-area ENU approximation used only as a conversion skeleton."""

    latitude_delta = math.radians(latitude - origin_lat)
    longitude_delta = _wrapped_longitude_delta_rad(origin_lon, longitude)
    x_m = EARTH_MEAN_RADIUS_M * longitude_delta * math.cos(math.radians(origin_lat))
    y_m = EARTH_MEAN_RADIUS_M * latitude_delta
    return x_m, y_m


def local_to_geodetic(
    origin_lat: float, origin_lon: float, x_m: float, y_m: float
) -> Tuple[float, float]:
    """Inverse of the stage-1 small-area ENU approximation."""

    cosine = math.cos(math.radians(origin_lat))
    if abs(cosine) < 1e-6:
        raise RouteError("ORIGIN_TOO_CLOSE_TO_POLE")
    latitude = origin_lat + math.degrees(y_m / EARTH_MEAN_RADIUS_M)
    longitude = origin_lon + math.degrees(x_m / (EARTH_MEAN_RADIUS_M * cosine))
    longitude = (longitude + 180.0) % 360.0 - 180.0
    _validate_lat_lon(latitude, longitude)
    return latitude, longitude


class RouteStore:
    """Thread-safe route transaction store with a single upload owner."""

    def __init__(
        self,
        *,
        min_waypoints: int,
        max_waypoints: int,
        min_segment_m: float,
        max_segment_m: float,
        max_abs_local_m: float,
    ) -> None:
        if min_waypoints < 1 or max_waypoints < min_waypoints:
            raise ValueError("invalid waypoint limits")
        if (
            not math.isfinite(min_segment_m)
            or not math.isfinite(max_segment_m)
            or min_segment_m < 0.0
            or max_segment_m <= min_segment_m
        ):
            raise ValueError("invalid segment limits")
        if not math.isfinite(max_abs_local_m) or max_abs_local_m <= 0.0:
            raise ValueError("invalid local coordinate limit")
        self._min_waypoints = min_waypoints
        self._max_waypoints = max_waypoints
        self._min_segment_m = min_segment_m
        self._max_segment_m = max_segment_m
        self._max_abs_local_m = max_abs_local_m
        self._lock = threading.RLock()
        self._upload: Optional[_Upload] = None
        self._route: Optional[StoredRoute] = None
        self._status = "EMPTY"
        self._error = ""
        self._version = 0

    @property
    def route(self) -> Optional[StoredRoute]:
        with self._lock:
            return self._route

    def snapshot(self) -> RouteSnapshot:
        with self._lock:
            expected = self._upload.expected_count if self._upload else 0
            received = len(self._upload.points) if self._upload else 0
            if self._route is not None and self._upload is None:
                expected = len(self._route.points)
                received = expected
            return RouteSnapshot(
                status=self._status,
                valid=self._route is not None and self._status == "READY",
                expected_count=expected,
                received_count=received,
                version=self._version,
                error=self._error,
            )

    def begin(
        self,
        owner: str,
        expected_count: int,
        origin: Optional[Tuple[float, float]] = None,
    ) -> RouteMode:
        with self._lock:
            if self._upload is not None and self._upload.owner != owner:
                raise RouteError("UPLOAD_BUSY")
            if expected_count < self._min_waypoints:
                raise RouteError("TOO_FEW_WAYPOINTS")
            if expected_count > self._max_waypoints:
                raise RouteError("TOO_MANY_WAYPOINTS")

            if origin is None:
                mode = RouteMode.GEODETIC
            else:
                _validate_lat_lon(*origin)
                if abs(math.cos(math.radians(origin[0]))) < 1e-6:
                    raise RouteError("ORIGIN_TOO_CLOSE_TO_POLE")
                mode = RouteMode.LEGACY_LOCAL

            # Starting an upload explicitly invalidates the old route. This
            # prevents a failed replacement upload from leaving stale autonomy
            # data marked ready.
            self._route = None
            self._upload = _Upload(owner, mode, expected_count, origin, {})
            self._status = "UPLOADING"
            self._error = ""
            return mode

    def add_waypoint(self, owner: str, index: int, first: float, second: float) -> int:
        with self._lock:
            upload = self._require_owner(owner)
            if index < 0 or index >= upload.expected_count:
                raise RouteError("INDEX_RANGE", str(index))
            if index in upload.points:
                raise RouteError("DUPLICATE_INDEX", str(index))
            if not math.isfinite(first) or not math.isfinite(second):
                raise RouteError("NON_FINITE_COORDINATE", str(index))
            if upload.mode is RouteMode.GEODETIC:
                _validate_lat_lon(first, second)
            elif abs(first) > self._max_abs_local_m or abs(second) > self._max_abs_local_m:
                raise RouteError("LOCAL_COORDINATE_RANGE", str(index))
            upload.points[index] = (first, second)
            return len(upload.points)

    def finish(self, owner: str) -> StoredRoute:
        with self._lock:
            upload = self._require_owner(owner)
            missing = [
                index
                for index in range(upload.expected_count)
                if index not in upload.points
            ]
            if missing:
                preview = ":".join(str(index) for index in missing[:8])
                raise RouteError("INCOMPLETE", preview)

            try:
                route = self._build_route(upload)
            except RouteError as exc:
                self._upload = None
                self._route = None
                self._status = "INVALID"
                self._error = exc.code
                raise

            self._version += 1
            route = StoredRoute(
                version=self._version,
                mode=route.mode,
                origin_latitude=route.origin_latitude,
                origin_longitude=route.origin_longitude,
                points=route.points,
                total_length_m=route.total_length_m,
            )
            self._route = route
            self._upload = None
            self._status = "READY"
            self._error = ""
            return route

    def abort_owner(self, owner: str, reason: str = "CLIENT_DISCONNECTED") -> bool:
        with self._lock:
            if self._upload is None or self._upload.owner != owner:
                return False
            self._upload = None
            self._route = None
            self._status = "INVALID"
            self._error = reason
            return True

    def _require_owner(self, owner: str) -> _Upload:
        if self._upload is None:
            raise RouteError("NO_UPLOAD")
        if self._upload.owner != owner:
            raise RouteError("UPLOAD_BUSY")
        return self._upload

    def _build_route(self, upload: _Upload) -> StoredRoute:
        raw_points = [upload.points[index] for index in range(upload.expected_count)]
        if upload.mode is RouteMode.GEODETIC:
            origin_lat, origin_lon = raw_points[0]
            if abs(math.cos(math.radians(origin_lat))) < 1e-6:
                raise RouteError("ORIGIN_TOO_CLOSE_TO_POLE")
            canonical = []
            for index, (latitude, longitude) in enumerate(raw_points):
                x_m, y_m = geodetic_to_local_m(
                    origin_lat, origin_lon, latitude, longitude
                )
                canonical.append(RoutePoint(index, latitude, longitude, x_m, y_m))
        else:
            if upload.origin is None:  # Defensive invariant.
                raise RouteError("MISSING_ORIGIN")
            origin_lat, origin_lon = upload.origin
            canonical = []
            for index, (x_m, y_m) in enumerate(raw_points):
                latitude, longitude = local_to_geodetic(
                    origin_lat, origin_lon, x_m, y_m
                )
                canonical.append(RoutePoint(index, latitude, longitude, x_m, y_m))

        # Reject exact duplicate points even when they aren't adjacent. Near
        # duplicates on an active segment are handled by the minimum segment
        # check below without an O(N^2) distance pass over large routes.
        seen_coordinates: Dict[Tuple[float, float], int] = {}
        for point in canonical:
            coordinate = (
                (point.x_m, point.y_m)
                if upload.mode is RouteMode.LEGACY_LOCAL
                else (
                    point.latitude,
                    (point.longitude + 180.0) % 360.0 - 180.0,
                )
            )
            previous_index = seen_coordinates.get(coordinate)
            if previous_index is not None:
                raise RouteError(
                    "DUPLICATE_WAYPOINT", f"{previous_index}:{point.index}"
                )
            seen_coordinates[coordinate] = point.index

        total_length_m = 0.0
        for index in range(1, len(canonical)):
            previous = canonical[index - 1]
            current = canonical[index]
            if upload.mode is RouteMode.LEGACY_LOCAL:
                distance = math.hypot(
                    current.x_m - previous.x_m, current.y_m - previous.y_m
                )
            else:
                distance = haversine_distance_m(
                    previous.latitude,
                    previous.longitude,
                    current.latitude,
                    current.longitude,
                )
            if distance < self._min_segment_m:
                raise RouteError("DUPLICATE_WAYPOINT", f"{index - 1}:{index}")
            if distance > self._max_segment_m:
                raise RouteError("SEGMENT_TOO_LONG", f"{index - 1}:{index}")
            total_length_m += distance

        return StoredRoute(
            version=0,
            mode=upload.mode,
            origin_latitude=origin_lat,
            origin_longitude=origin_lon,
            points=tuple(canonical),
            total_length_m=total_length_m,
        )
