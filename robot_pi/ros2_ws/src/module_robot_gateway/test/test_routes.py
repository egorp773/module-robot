import pytest

from module_robot_gateway.routes import RouteError, RouteMode, RouteStore


def make_store(max_segment_m=100.0):
    return RouteStore(
        min_waypoints=1,
        max_waypoints=16,
        min_segment_m=0.01,
        max_segment_m=max_segment_m,
        max_abs_local_m=1000.0,
    )


def test_geodetic_route_is_complete_and_converted_to_local_path():
    store = make_store()
    assert store.begin("phone-a", 3) is RouteMode.GEODETIC
    store.add_waypoint("phone-a", 2, 55.00002, 37.00000)
    store.add_waypoint("phone-a", 0, 55.00000, 37.00000)
    store.add_waypoint("phone-a", 1, 55.00001, 37.00000)

    route = store.finish("phone-a")

    assert route.mode is RouteMode.GEODETIC
    assert route.points[0].x_m == pytest.approx(0.0)
    assert route.points[0].y_m == pytest.approx(0.0)
    assert route.points[2].y_m > route.points[1].y_m > 0.0
    assert route.total_length_m == pytest.approx(2.22, abs=0.05)
    assert store.snapshot().valid


def test_current_flutter_legacy_origin_and_local_xy_are_supported():
    store = make_store()
    assert store.begin("phone", 2, (55.751244, 37.618423)) is RouteMode.LEGACY_LOCAL
    store.add_waypoint("phone", 0, 0.0, 0.0)
    store.add_waypoint("phone", 1, 1.0, 2.0)

    route = store.finish("phone")

    assert route.points[1].x_m == 1.0
    assert route.points[1].y_m == 2.0
    assert route.points[1].latitude > route.origin_latitude
    assert route.points[1].longitude > route.origin_longitude
    assert route.total_length_m == pytest.approx(5**0.5)


def test_upload_is_owned_by_one_client():
    store = make_store()
    store.begin("phone-a", 1)
    with pytest.raises(RouteError) as error:
        store.begin("phone-b", 1)
    assert error.value.code == "UPLOAD_BUSY"
    with pytest.raises(RouteError) as error:
        store.add_waypoint("phone-b", 0, 55.0, 37.0)
    assert error.value.code == "UPLOAD_BUSY"


def test_duplicate_index_is_rejected():
    store = make_store()
    store.begin("phone", 2)
    store.add_waypoint("phone", 0, 55.0, 37.0)
    with pytest.raises(RouteError) as error:
        store.add_waypoint("phone", 0, 55.1, 37.1)
    assert error.value.code == "DUPLICATE_INDEX"


def test_missing_index_is_reported_and_can_then_be_completed():
    store = make_store()
    store.begin("phone", 2)
    store.add_waypoint("phone", 0, 55.0, 37.0)
    with pytest.raises(RouteError) as error:
        store.finish("phone")
    assert error.value.code == "INCOMPLETE"
    assert error.value.detail == "1"

    store.add_waypoint("phone", 1, 55.00001, 37.0)
    assert store.finish("phone").points[1].index == 1


def test_duplicate_coordinates_in_adjacent_points_are_rejected():
    store = make_store()
    store.begin("phone", 2)
    store.add_waypoint("phone", 0, 55.0, 37.0)
    store.add_waypoint("phone", 1, 55.0, 37.0)
    with pytest.raises(RouteError) as error:
        store.finish("phone")
    assert error.value.code == "DUPLICATE_WAYPOINT"
    assert not store.snapshot().valid


def test_non_adjacent_duplicate_coordinates_are_rejected():
    store = make_store()
    store.begin("phone", 3)
    store.add_waypoint("phone", 0, 55.0, 37.0)
    store.add_waypoint("phone", 1, 55.00001, 37.0)
    store.add_waypoint("phone", 2, 55.0, 37.0)
    with pytest.raises(RouteError) as error:
        store.finish("phone")
    assert error.value.code == "DUPLICATE_WAYPOINT"


def test_segment_upper_bound_is_enforced():
    store = make_store(max_segment_m=5.0)
    store.begin("phone", 2, (55.0, 37.0))
    store.add_waypoint("phone", 0, 0.0, 0.0)
    store.add_waypoint("phone", 1, 6.0, 0.0)
    with pytest.raises(RouteError) as error:
        store.finish("phone")
    assert error.value.code == "SEGMENT_TOO_LONG"


@pytest.mark.parametrize(
    "latitude,longitude,code",
    [
        (91.0, 37.0, "LATITUDE_RANGE"),
        (-91.0, 37.0, "LATITUDE_RANGE"),
        (55.0, 181.0, "LONGITUDE_RANGE"),
        (55.0, -181.0, "LONGITUDE_RANGE"),
    ],
)
def test_geographic_bounds(latitude, longitude, code):
    store = make_store()
    store.begin("phone", 1)
    with pytest.raises(RouteError) as error:
        store.add_waypoint("phone", 0, latitude, longitude)
    assert error.value.code == code


def test_non_finite_coordinate_is_rejected_by_store_defence_in_depth():
    store = make_store()
    store.begin("phone", 1)
    with pytest.raises(RouteError) as error:
        store.add_waypoint("phone", 0, float("nan"), 37.0)
    assert error.value.code == "NON_FINITE_COORDINATE"


def test_disconnect_aborts_only_the_owners_upload():
    store = make_store()
    store.begin("phone-a", 1)
    assert not store.abort_owner("phone-b")
    assert store.abort_owner("phone-a")
    snapshot = store.snapshot()
    assert snapshot.status == "INVALID"
    assert snapshot.error == "CLIENT_DISCONNECTED"
