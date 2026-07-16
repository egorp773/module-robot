"""Static autonomous preflight checks used before Nav2 processes are created."""

from __future__ import annotations

import argparse
import math
import os
from typing import Any, Dict, Iterable, Tuple

import yaml


DIMENSION_ALIASES = {
    "track_width_m": ("track_width_m", "TODO_TRACK_WIDTH_M"),
    "robot_length_m": ("robot_length_m", "TODO_ROBOT_LENGTH_M"),
    "robot_width_m": ("robot_width_m", "TODO_ROBOT_WIDTH_M"),
    "robot_height_m": ("robot_height_m", "TODO_ROBOT_HEIGHT_M"),
    "gps_x_m": ("gps_x_m", "TODO_GPS_X_M"),
    "gps_y_m": ("gps_y_m", "TODO_GPS_Y_M"),
    "gps_z_m": ("gps_z_m", "TODO_GPS_Z_M"),
    "imu_x_m": ("imu_x_m", "TODO_IMU_X_M"),
    "imu_y_m": ("imu_y_m", "TODO_IMU_Y_M"),
    "imu_z_m": ("imu_z_m", "TODO_IMU_Z_M"),
    "imu_roll_rad": ("imu_roll_rad", "TODO_IMU_ROLL"),
    "imu_pitch_rad": ("imu_pitch_rad", "TODO_IMU_PITCH"),
    "imu_yaw_rad": ("imu_yaw_rad", "TODO_IMU_YAW"),
}
POSITIVE_DIMENSIONS = {
    "track_width_m",
    "robot_length_m",
    "robot_width_m",
    "robot_height_m",
}


class PreflightError(RuntimeError):
    """Autonomy configuration is incomplete or physically invalid."""


def _walk(value: Any, path: str = "") -> Iterable[Tuple[str, Any]]:
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = f"{path}.{key}" if path else str(key)
            yield from _walk(child, child_path)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _walk(child, f"{path}[{index}]")
    else:
        yield path, value


def _key_values(value: Any) -> Dict[str, Any]:
    found: Dict[str, Any] = {}
    if isinstance(value, dict):
        for key, child in value.items():
            found[str(key)] = child
            found.update(_key_values(child))
    elif isinstance(value, list):
        for child in value:
            found.update(_key_values(child))
    return found


def resolve_dimensions(document: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize canonical lowercase and description-package placeholder keys."""
    values = _key_values(document)
    resolved = {}
    missing = []
    for logical_name, aliases in DIMENSION_ALIASES.items():
        matching = [values[alias] for alias in aliases if alias in values]
        if not matching:
            missing.append(logical_name)
        else:
            resolved[logical_name] = matching[0]
    if missing:
        raise PreflightError("missing required measurements: " + ", ".join(missing))
    return resolved


def validate_dimensions(path: str) -> Dict[str, Any]:
    """Return parsed YAML or raise if any measurement gate is unresolved."""
    expanded = os.path.abspath(os.path.expanduser(path))
    if not os.path.isfile(expanded):
        raise PreflightError(f"dimensions file does not exist: {expanded}")
    try:
        with open(expanded, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise PreflightError(f"cannot read dimensions file {expanded}: {exc}") from exc
    if not isinstance(document, dict):
        raise PreflightError("dimensions YAML must contain a mapping")

    unresolved = [
        location
        for location, value in _walk(document)
        if isinstance(value, str) and value.strip().upper().startswith("TODO")
    ]
    if unresolved:
        raise PreflightError("unresolved measurements: " + ", ".join(sorted(unresolved)))

    values = _key_values(document)
    resolved = resolve_dimensions(document)

    for key, value in sorted(resolved.items()):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise PreflightError(f"{key} must be a finite numeric measurement")
        if not math.isfinite(float(value)):
            raise PreflightError(f"{key} must be finite")
        if key in POSITIVE_DIMENSIONS and float(value) <= 0.0:
            raise PreflightError(f"{key} must be greater than zero")

    if "measurement_status" in values and str(values["measurement_status"]).lower() != "measured":
        raise PreflightError("measurement_status must be 'measured'")
    if "footprint_status" in values and str(values["footprint_status"]).lower() != "measured":
        raise PreflightError("footprint_status must be 'measured'")
    return document


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Validate robot dimensions without starting any ROS nodes"
    )
    parser.add_argument("dimensions_file")
    args = parser.parse_args(argv)
    validate_dimensions(args.dimensions_file)
    print("Autonomy dimensions preflight passed")


if __name__ == "__main__":
    main()
