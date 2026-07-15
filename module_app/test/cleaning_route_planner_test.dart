import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/cleaning_route_planner.dart';
import 'package:hello_flutter/features/manual/manual_control_screen.dart';

void main() {
  test('coverage spacing defaults match the configurable tool range', () {
    expect(CleaningRoutePlanner.defaultLineStepMeters, 0.42);
    expect(CleaningRoutePlanner.minRecommendedLineStepMeters, 0.38);
    expect(CleaningRoutePlanner.maxRecommendedLineStepMeters, 0.45);
  });

  test('builds perimeter plus snake from an outside start', () {
    final zone = _rect(0, 0, 6, 2.4);
    final route = CleaningRoutePlanner.planRoute(
      _map(zone),
      lineStep: 0.35,
      startOverride: const Offset(-1, 1.2),
    );

    expect(route, isNotNull);
    expect(route!.path.first, const Offset(-1, 1.2));
    expect(route.path.length, lessThanOrEqualTo(160));
    expect(route.cleaningSegments, greaterThan(4));
    expect(route.totalDistance, greaterThan(12));
    expect(_allAfterStartInside(route.path, zone), isTrue);
    expect(
      route.path.any((p) => _distanceToPolygon(p, zone) < 0.02),
      isTrue,
      reason:
          'route should include the recorded perimeter, not only inset rows',
    );
  });

  test('rejects arbitrary starts that are too far from the mapped zone', () {
    final route = CleaningRoutePlanner.planRoute(
      _map(_rect(0, 0, 6, 2.4)),
      lineStep: 0.35,
      startOverride: const Offset(-10, 1.2),
    );

    expect(route, isNull);
  });

  test('can build from a far GPS start when automatic mode allows approach',
      () {
    final zone = _rect(0, 0, 6, 2.4);
    final route = CleaningRoutePlanner.planRoute(
      _map(zone),
      lineStep: 0.35,
      startOverride: const Offset(-10, 1.2),
      maxStartDistanceMeters: double.infinity,
    );

    expect(route, isNotNull);
    expect(route!.path.first, const Offset(-10, 1.2));
    expect(route.path[1], const Offset(0, 1.2));
    expect(_allAfterStartInside(route.path, zone), isTrue);
  });

  test('routes around a forbidden island without entering the red polygon', () {
    final zone = _rect(0, 0, 6, 3);
    final forbidden = _rect(2.5, 1, 3.5, 2);
    final route = CleaningRoutePlanner.planRoute(
      _map(zone, forbiddens: [forbidden]),
      lineStep: 0.35,
      startOverride: const Offset(-1, 1.5),
    );

    expect(route, isNotNull);
    expect(route!.path.length, lessThanOrEqualTo(160));
    expect(_allAfterStartInside(route.path, zone), isTrue);
    expect(_pathEntersPolygon(route.path, forbidden), isFalse);
  });

  test('chooses vertical snake lines for tall zones', () {
    final zone = _rect(0, 0, 2.4, 6);
    final route = CleaningRoutePlanner.planRoute(
      _map(zone),
      lineStep: 0.35,
      startOverride: const Offset(1.2, -1),
    );

    expect(route, isNotNull);
    expect(route!.path.length, lessThanOrEqualTo(160));
    expect(_allAfterStartInside(route.path, zone), isTrue);
    expect(route.totalDistance, greaterThan(14));
  });

  test('accepts starts from multiple nearby positions around the same map', () {
    final zone = _rect(0, 0, 5, 3);
    final starts = [
      const Offset(-1, 1.5),
      const Offset(6, 1.5),
      const Offset(2.5, -1),
      const Offset(2.5, 1.5),
    ];

    for (final start in starts) {
      final route = CleaningRoutePlanner.planRoute(
        _map(zone),
        lineStep: 0.35,
        startOverride: start,
      );

      expect(route, isNotNull, reason: 'start=$start');
      expect(route!.path.first, start);
      expect(route.path.length, lessThanOrEqualTo(160));
      expect(_allAfterStartInside(route.path, zone), isTrue);
    }
  });

  test('uses transition line as travel-only connector between zones', () {
    final zoneA = _rect(0, 0, 2, 2);
    final zoneB = _rect(5, 0, 7, 2);
    final transition = [const Offset(2, 1), const Offset(5, 1)];

    final route = CleaningRoutePlanner.planRoute(
      _map(zoneA, zones: [zoneA, zoneB], transitions: [transition]),
      lineStep: 0.35,
      startOverride: const Offset(-1, 1),
    );

    expect(route, isNotNull);
    expect(route!.path.any((p) => _pointInOrOnPolygon(p, zoneA)), isTrue);
    expect(route.path.any((p) => _pointInOrOnPolygon(p, zoneB)), isTrue);
    expect(
      route.path.any((p) =>
          _distanceToSegment(
            p,
            transition.first,
            transition.last,
          ) <
          0.03),
      isTrue,
      reason: 'route should travel along the stored transition line',
    );
  });

  test('keeps reachable zone route when another zone has no transition', () {
    final zoneA = _rect(0, 0, 2, 2);
    final zoneB = _rect(6, 0, 8, 2);

    final route = CleaningRoutePlanner.planRoute(
      _map(zoneA, zones: [zoneA, zoneB]),
      lineStep: 0.35,
      startOverride: const Offset(-1, 1),
    );

    expect(route, isNotNull);
    expect(route!.path.any((p) => _pointInOrOnPolygon(p, zoneA)), isTrue);
    expect(route.path.any((p) => _pointInOrOnPolygon(p, zoneB)), isFalse);
  });
}

ManualMapState _map(
  List<Offset> zone, {
  List<List<Offset>>? zones,
  List<List<Offset>> forbiddens = const [],
  List<List<Offset>> transitions = const [],
}) {
  return ManualMapState.initial().copyWith(
    zones: (zones ?? [zone]).map(PolyShape.new).toList(),
    forbiddens: forbiddens.map(PolyShape.new).toList(),
    transitions: transitions,
    coordinateType: 'gps',
    refLat: 55.751244,
    refLon: 37.618423,
  );
}

List<Offset> _rect(double left, double top, double right, double bottom) {
  return [
    Offset(left, top),
    Offset(right, top),
    Offset(right, bottom),
    Offset(left, bottom),
  ];
}

bool _allAfterStartInside(List<Offset> path, List<Offset> polygon) {
  for (var i = 1; i < path.length; i++) {
    if (!_pointInOrOnPolygon(path[i], polygon)) return false;
  }
  return true;
}

bool _pathEntersPolygon(List<Offset> path, List<Offset> polygon) {
  for (var i = 1; i < path.length; i++) {
    final a = path[i - 1];
    final b = path[i];
    final length = (b - a).distance;
    final samples = math.max(2, (length / 0.04).ceil());
    for (var s = 0; s <= samples; s++) {
      final t = s / samples;
      final p = Offset(
        a.dx + (b.dx - a.dx) * t,
        a.dy + (b.dy - a.dy) * t,
      );
      if (_pointStrictlyInPolygon(p, polygon)) return true;
    }
  }
  return false;
}

bool _pointInOrOnPolygon(Offset p, List<Offset> polygon) {
  if (_distanceToPolygon(p, polygon) < 1e-5) return true;
  return _pointStrictlyInPolygon(p, polygon);
}

bool _pointStrictlyInPolygon(Offset p, List<Offset> polygon) {
  var inside = false;
  for (var i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    final a = polygon[i];
    final b = polygon[j];
    final crosses = ((a.dy > p.dy) != (b.dy > p.dy)) &&
        (p.dx < (b.dx - a.dx) * (p.dy - a.dy) / (b.dy - a.dy) + a.dx);
    if (crosses) inside = !inside;
  }
  return inside;
}

double _distanceToPolygon(Offset p, List<Offset> polygon) {
  var best = double.infinity;
  for (var i = 0; i < polygon.length; i++) {
    final a = polygon[i];
    final b = polygon[(i + 1) % polygon.length];
    best = math.min(best, _distanceToSegment(p, a, b));
  }
  return best;
}

double _distanceToSegment(Offset p, Offset a, Offset b) {
  final ab = b - a;
  final len2 = ab.dx * ab.dx + ab.dy * ab.dy;
  if (len2 <= 1e-12) return (p - a).distance;
  final rawT = ((p.dx - a.dx) * ab.dx + (p.dy - a.dy) * ab.dy) / len2;
  final t = rawT.clamp(0.0, 1.0).toDouble();
  final projection = Offset(a.dx + ab.dx * t, a.dy + ab.dy * t);
  return (p - projection).distance;
}
