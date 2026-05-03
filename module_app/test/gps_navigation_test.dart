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

  test('navigation stops on stale GPS data', () {
    final now = DateTime.utc(2026, 5, 3, 12);
    final result = const GpsNavigationController().evaluate(
      currentLat: 55.0,
      currentLon: 37.0,
      targetLat: 55.0001,
      targetLon: 37.0,
      headingDegrees: 0,
      rtkFixed: true,
      rtcmAgeMs: 200,
      hAccMm: 16,
      originLat: 55.0,
      originLon: 37.0,
      gpsFixType: 3,
      gpsReceivedAt: now.subtract(const Duration(seconds: 31)),
      now: now,
    );

    expect(result.command, NavigationCommand.stop);
    expect(result.reason, 'GPS-данные устарели');
  });

  test('navigation stops on weak GPS fix', () {
    final result = const GpsNavigationController().evaluate(
      currentLat: 55.0,
      currentLon: 37.0,
      targetLat: 55.0001,
      targetLon: 37.0,
      headingDegrees: 0,
      rtkFixed: true,
      rtcmAgeMs: 200,
      hAccMm: 16,
      originLat: 55.0,
      originLon: 37.0,
      gpsFixType: 2,
    );

    expect(result.command, NavigationCommand.stop);
    expect(result.reason, 'GPS fix слабый');
  });

  test('navigation stops on stale rover GPS age', () {
    final result = const GpsNavigationController().evaluate(
      currentLat: 55.0,
      currentLon: 37.0,
      targetLat: 55.0001,
      targetLon: 37.0,
      headingDegrees: 0,
      rtkFixed: true,
      rtcmAgeMs: 200,
      hAccMm: 16,
      originLat: 55.0,
      originLon: 37.0,
      gpsFixType: 3,
      gpsAgeMs: 6000,
    );

    expect(result.command, NavigationCommand.stop);
    expect(result.reason, 'GPS rover stale');
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

  test('motor mapper keeps route commands conservative', () {
    const mapper = NavigationMotorMapper();

    expect(
      mapper.toMotorCommand(NavigationCommand.forward).protocol,
      'M,22,22',
    );
    expect(
      mapper.toMotorCommand(NavigationCommand.turnLeft).protocol,
      'M,-18,18',
    );
    expect(
      mapper.toMotorCommand(NavigationCommand.turnRight).protocol,
      'M,18,-18',
    );
    expect(
      mapper.toMotorCommand(NavigationCommand.stop).protocol,
      'STOP',
    );
    expect(
      mapper.toMotorCommand(NavigationCommand.arrived).protocol,
      'STOP',
    );
  });

  test('heading calibration aligns raw IMU yaw to target bearing', () {
    const calibration = HeadingCalibration();

    final aligned = calibration.alignRawToTarget(
      rawDegrees: 103,
      targetDegrees: 0,
    );

    expect(aligned.offsetDegrees, closeTo(257, 0.001));
    expect(aligned.apply(103), closeTo(0, 0.001));
  });

  test('heading calibration corrects gradually toward GPS course', () {
    const calibration = HeadingCalibration(offsetDegrees: 10);

    final corrected = calibration.correctTowardObservedHeading(
      rawDegrees: 80,
      observedHeadingDegrees: 100,
      gain: 0.25,
    );

    expect(calibration.apply(80), closeTo(90, 0.001));
    expect(corrected.offsetDegrees, closeTo(12.5, 0.001));
    expect(corrected.apply(80), closeTo(92.5, 0.001));
  });

  test('heading calibration ignores impossible GPS course jumps', () {
    const calibration = HeadingCalibration(offsetDegrees: 10);

    final corrected = calibration.correctTowardObservedHeading(
      rawDegrees: 80,
      observedHeadingDegrees: 180,
      gain: 0.25,
    );

    expect(corrected.offsetDegrees, calibration.offsetDegrees);
  });

  test('motor mapper can invert forward and steering direction', () {
    const mapper = NavigationMotorMapper();

    expect(
      mapper
          .toMotorCommand(
            NavigationCommand.forward,
            invertForward: true,
          )
          .protocol,
      'M,-22,-22',
    );
    expect(
      mapper
          .toMotorCommand(
            NavigationCommand.turnLeft,
            invertSteering: true,
          )
          .protocol,
      'M,18,-18',
    );
  });
}
