import 'dart:ui';

import 'gps_display_math.dart';
import 'wifi_connection.dart';

class NavRunSample {
  final DateTime at;
  final Offset local;
  final double headingDeg;

  const NavRunSample({
    required this.at,
    required this.local,
    required this.headingDeg,
  });
}

class NavRunSummary {
  final int sampleCount;
  final double totalDistanceM;
  final Duration duration;
  final Offset start;
  final Offset end;
  final double startHeadingDeg;
  final double endHeadingDeg;
  final List<String> steps;

  const NavRunSummary({
    required this.sampleCount,
    required this.totalDistanceM,
    required this.duration,
    required this.start,
    required this.end,
    required this.startHeadingDeg,
    required this.endHeadingDeg,
    required this.steps,
  });

  String get shortText {
    final stepText = steps.isEmpty ? 'без уверенных сегментов' : steps.join(' -> ');
    return 'Факт: $stepText. '
        'Пройдено ~${totalDistanceM.toStringAsFixed(1)} м за '
        '${duration.inSeconds}s, точек: $sampleCount.';
  }
}

class NavRunTracker {
  static const double _minSampleDistanceM = 0.10;
  static const double _minMotionSegmentM = 0.20;
  static const double _turnThresholdDeg = 25.0;
  static const double _turnSampleThresholdDeg = 6.0;

  final GpsDisplayGeometry _geometry;
  final List<NavRunSample> _samples = <NavRunSample>[];
  bool _active = false;

  NavRunTracker({
    required double originLat,
    required double originLon,
  }) : _geometry = GpsDisplayGeometry(originLat: originLat, originLon: originLon);

  bool get isActive => _active;
  int get sampleCount => _samples.length;

  void begin() {
    _samples.clear();
    _active = true;
  }

  void reset() {
    _samples.clear();
    _active = false;
  }

  void observe(WifiConnectionState wifi) {
    if (!_active) return;

    final lat = wifi.gpsLat;
    final lon = wifi.gpsLon;
    if (lat == null || lon == null) return;
    if (lat.abs() > 90 || lon.abs() > 180) return;

    final heading = _sampleHeadingDeg(wifi);
    if (heading == null || !heading.isFinite) return;

    final local = _geometry.toLocal(lat, lon);
    final sample = NavRunSample(
      at: DateTime.now(),
      local: Offset(local.x, local.y),
      headingDeg: _normalizeDeg360(heading),
    );

    if (_samples.isNotEmpty) {
      final last = _samples.last;
      final dist = (sample.local - last.local).distance;
      final headingDelta = _wrapDeg180(sample.headingDeg - last.headingDeg).abs();
      if (dist < _minSampleDistanceM && headingDelta < 10.0) {
        return;
      }
    }

    _samples.add(sample);
  }

  NavRunSummary? finish() {
    if (_samples.length < 2) {
      reset();
      return null;
    }

    final summary = NavRunSummary(
      sampleCount: _samples.length,
      totalDistanceM: _pathDistance(_samples.map((s) => s.local).toList(growable: false)),
      duration: _samples.last.at.difference(_samples.first.at),
      start: _samples.first.local,
      end: _samples.last.local,
      startHeadingDeg: _samples.first.headingDeg,
      endHeadingDeg: _samples.last.headingDeg,
      steps: _summarizeSteps(_samples),
    );
    reset();
    return summary;
  }

  static double? _sampleHeadingDeg(WifiConnectionState wifi) {
    final imu = wifi.imuYaw;
    if (imu != null && imu.isFinite) return imu;
    final gpsHeading = wifi.gpsHeading;
    if (gpsHeading != null && gpsHeading.isFinite) return gpsHeading;
    return null;
  }

  static List<String> _summarizeSteps(List<NavRunSample> samples) {
    final steps = <String>[];
    double motionDistance = 0.0;
    double turnAccumulatorDeg = 0.0;
    bool turnActive = false;

    void flushMotion() {
      if (motionDistance >= _minMotionSegmentM) {
        steps.add('вперед ~${motionDistance.toStringAsFixed(1)} м');
      }
      motionDistance = 0.0;
    }

    void flushTurn() {
      if (turnAccumulatorDeg.abs() >= _turnThresholdDeg) {
        final dir = turnAccumulatorDeg > 0 ? 'направо' : 'налево';
        steps.add('$dir ~${turnAccumulatorDeg.abs().toStringAsFixed(0)}°');
      }
      turnAccumulatorDeg = 0.0;
    }

    for (int i = 1; i < samples.length; i++) {
      final prev = samples[i - 1];
      final curr = samples[i];
      final segmentDistance = (curr.local - prev.local).distance;
      final headingDelta = _wrapDeg180(curr.headingDeg - prev.headingDeg);

      final turnish =
          headingDelta.abs() >= _turnSampleThresholdDeg || segmentDistance < 0.12;
      if (turnish) {
        turnActive = true;
        turnAccumulatorDeg += headingDelta;
        continue;
      }

      if (turnActive) {
        flushMotion();
        flushTurn();
        turnActive = false;
      }

      motionDistance += segmentDistance;
    }

    if (turnActive) {
      flushMotion();
      flushTurn();
    }

    flushMotion();
    return steps;
  }

  static double _pathDistance(List<Offset> points) {
    var total = 0.0;
    for (int i = 1; i < points.length; i++) {
      total += (points[i] - points[i - 1]).distance;
    }
    return total;
  }

  static double _normalizeDeg360(double value) {
    var out = value % 360.0;
    if (out < 0) out += 360.0;
    return out;
  }

  static double _wrapDeg180(double value) {
    var out = _normalizeDeg360(value);
    if (out > 180.0) out -= 360.0;
    return out;
  }
}
