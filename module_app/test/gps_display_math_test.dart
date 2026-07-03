import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/gps_display_math.dart';
import 'package:hello_flutter/core/gps_projection.dart';

void main() {
  test('distance and bearing stay usable for monitor display', () {
    final distance = GpsDisplayGeometry.distanceMeters(
      55.751244,
      37.618423,
      55.751244,
      37.618523,
    );
    final bearing = GpsDisplayGeometry.bearingDegrees(
      55.751244,
      37.618423,
      55.751244,
      37.618523,
    );

    expect(distance, closeTo(6.27, 0.25));
    expect(bearing, closeTo(90, 0.2));
  });

  test('heading error is normalized to signed shortest turn', () {
    expect(GpsDisplayGeometry.headingErrorDegrees(350, 10), 20);
    expect(GpsDisplayGeometry.headingErrorDegrees(10, 350), -20);
  });

  test('local projection returns east as x and north as y', () {
    final geometry = GpsDisplayGeometry(
      originLat: 55.751244,
      originLon: 37.618423,
    );

    final point = geometry.toLocal(55.751344, 37.618523);

    // Same fixture and expected values are used by firmware NavMath tests.
    expect(point.x, closeTo(6.2793, 0.01));
    expect(point.y, closeTo(11.132, 0.01));

    final legacyProjection = GpsProjection(
      refLat: 55.751244,
      refLon: 37.618423,
    ).toLocal(55.751344, 37.618523);
    expect(legacyProjection.dx, closeTo(point.x, 1e-6));
    expect(legacyProjection.dy, closeTo(point.y, 1e-6));
  });
}
