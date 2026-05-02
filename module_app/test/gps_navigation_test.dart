import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/gps_navigation.dart';

void main() {
  test('lat/lon converts to local meters', () {
    final geometry = GpsLocalGeometry(originLat: 55.0, originLon: 37.0);

    final north = geometry.toLocal(55.00001, 37.0);
    final east = geometry.toLocal(55.0, 37.00001);

    expect(north.y, closeTo(1.113, 0.02));
    expect(north.x, closeTo(0, 0.001));
    expect(east.x, closeTo(0.64, 0.03));
    expect(east.y, closeTo(0, 0.001));
  });

  test('distance returns meters', () {
    final distance = GpsLocalGeometry.distanceMeters(
      55.0,
      37.0,
      55.00001,
      37.0,
    );

    expect(distance, closeTo(1.11, 0.03));
  });

  test('bearing returns navigation azimuth', () {
    expect(
      GpsLocalGeometry.bearingDegrees(55.0, 37.0, 55.00001, 37.0),
      closeTo(0, 0.5),
    );
    expect(
      GpsLocalGeometry.bearingDegrees(55.0, 37.0, 55.0, 37.00001),
      closeTo(90, 0.5),
    );
  });

  test('heading error is shortest signed turn', () {
    expect(GpsLocalGeometry.headingErrorDegrees(350, 10), closeTo(20, 0.001));
    expect(GpsLocalGeometry.headingErrorDegrees(10, 350), closeTo(-20, 0.001));
    expect(GpsLocalGeometry.headingErrorDegrees(0, 270), closeTo(-90, 0.001));
  });

  test('navigation stops when RTK is not fixed', () {
    final result = const GpsNavigationController().evaluate(
      currentLat: 55.0,
      currentLon: 37.0,
      targetLat: 55.0001,
      targetLon: 37.0,
      headingDegrees: 0,
      rtkFixed: false,
      rtcmAgeMs: 200,
      hAccMm: 16,
      originLat: 55.0,
      originLon: 37.0,
    );

    expect(result.command, NavigationCommand.stop);
  });

  test('navigation arrives inside 0.3 m', () {
    final result = const GpsNavigationController().evaluate(
      currentLat: 55.0,
      currentLon: 37.0,
      targetLat: 55.000001,
      targetLon: 37.0,
      headingDegrees: 0,
      rtkFixed: true,
      rtcmAgeMs: 200,
      hAccMm: 16,
      originLat: 55.0,
      originLon: 37.0,
    );

    expect(result.command, NavigationCommand.arrived);
    expect(result.distanceMeters, lessThan(0.3));
  });
}
