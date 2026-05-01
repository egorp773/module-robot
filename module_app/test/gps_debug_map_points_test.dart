import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/gps_perimeter_storage.dart';
import 'package:hello_flutter/features/gps/gps_debug_screen.dart';

void main() {
  test('opened perimeter with 4 points is passed to map points', () {
    final opened = [
      GpsPerimeterPoint(
        lat: 55.0,
        lon: 37.0,
        at: DateTime.utc(2026, 5),
      ),
      GpsPerimeterPoint(
        lat: 55.0,
        lon: 37.0001,
        at: DateTime.utc(2026, 5, 1, 0, 0, 1),
      ),
      GpsPerimeterPoint(
        lat: 55.0001,
        lon: 37.0001,
        at: DateTime.utc(2026, 5, 1, 0, 0, 2),
      ),
      GpsPerimeterPoint(
        lat: 55.0001,
        lon: 37.0,
        at: DateTime.utc(2026, 5, 1, 0, 0, 3),
      ),
    ];

    final visible = gpsDebugVisibleMapPoints(
      currentPerimeter: const [],
      openedPerimeter: opened,
      track: const [],
      currentPosition: null,
    );

    expect(visible, hasLength(4));
    expect(visible, orderedEquals(opened));
  });
}
