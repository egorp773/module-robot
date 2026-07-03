import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/gps_display_math.dart';
import 'package:hello_flutter/core/nav_run_tracker.dart';
import 'package:hello_flutter/core/wifi_connection.dart';

WifiConnectionState _sampleState({
  required double lat,
  required double lon,
  required double headingDeg,
}) {
  return WifiConnectionState.initial().copyWith(
    gpsLat: lat,
    gpsLon: lon,
    imuYaw: headingDeg,
  );
}

void main() {
  test('NavRunTracker summarizes a forward-left-forward route', () {
    const originLat = 55.0;
    const originLon = 37.0;
    final geo = GpsDisplayGeometry(originLat: originLat, originLon: originLon);
    final tracker = NavRunTracker(originLat: originLat, originLon: originLon)..begin();

    final samples = <({double x, double y, double heading})>[
      (x: 0.0, y: 0.0, heading: 0.0),
      (x: 0.0, y: 0.8, heading: 0.0),
      (x: 0.0, y: 1.6, heading: 0.0),
      (x: 0.0, y: 1.6, heading: 350.0),
      (x: 0.0, y: 1.6, heading: 320.0),
      (x: 0.0, y: 1.6, heading: 285.0),
      (x: -0.8, y: 1.6, heading: 270.0),
      (x: -1.6, y: 1.6, heading: 270.0),
    ];

    for (final sample in samples) {
      tracker.observe(
        _sampleState(
          lat: originLat + sample.y / geo.metersPerDegreeLat,
          lon: originLon + sample.x / geo.metersPerDegreeLon,
          headingDeg: sample.heading,
        ),
      );
    }

    final summary = tracker.finish();
    expect(summary, isNotNull);
    expect(summary!.steps.length, greaterThanOrEqualTo(2));
    expect(summary.steps.first, startsWith('вперед'));
    expect(summary.steps.any((step) => step.startsWith('налево')), isTrue);
    expect(summary.shortText, contains('м'));
    expect(summary.sampleCount, greaterThan(2));
    expect(summary.totalDistanceM, greaterThan(1.5));
  });
}
