from pathlib import Path

import pytest
import yaml

from module_robot_bringup.preflight import PreflightError, validate_dimensions


MEASURED = {
    "robot_dimensions": {
        "ros__parameters": {
            "measurement_status": "measured",
            "footprint_status": "measured",
            "TODO_TRACK_WIDTH_M": 0.4,
            "TODO_ROBOT_LENGTH_M": 0.8,
            "TODO_ROBOT_WIDTH_M": 0.5,
            "TODO_ROBOT_HEIGHT_M": 0.3,
            "TODO_GPS_X_M": 0.0,
            "TODO_GPS_Y_M": 0.0,
            "TODO_GPS_Z_M": 0.2,
            "TODO_IMU_X_M": 0.0,
            "TODO_IMU_Y_M": 0.0,
            "TODO_IMU_Z_M": 0.1,
            "TODO_IMU_ROLL": 0.0,
            "TODO_IMU_PITCH": 0.0,
            "TODO_IMU_YAW": 0.0,
        }
    }
}


def _write_yaml(path: Path, document) -> Path:
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_preflight_rejects_any_unresolved_measurement(tmp_path):
    document = yaml.safe_load(yaml.safe_dump(MEASURED))
    document["robot_dimensions"]["ros__parameters"]["TODO_TRACK_WIDTH_M"] = (
        "TODO_MEASURE"
    )
    with pytest.raises(PreflightError, match="unresolved measurements"):
        validate_dimensions(str(_write_yaml(tmp_path / "dimensions.yaml", document)))


def test_preflight_rejects_nonpositive_physical_dimensions(tmp_path):
    document = yaml.safe_load(yaml.safe_dump(MEASURED))
    document["robot_dimensions"]["ros__parameters"]["TODO_ROBOT_LENGTH_M"] = 0.0
    with pytest.raises(PreflightError, match="robot_length_m must be greater than zero"):
        validate_dimensions(str(_write_yaml(tmp_path / "dimensions.yaml", document)))


def test_preflight_accepts_only_complete_finite_measured_dimensions(tmp_path):
    path = _write_yaml(tmp_path / "dimensions.yaml", MEASURED)
    assert validate_dimensions(str(path)) == MEASURED
