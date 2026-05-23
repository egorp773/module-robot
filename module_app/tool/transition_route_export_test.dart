import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/cleaning_route_planner.dart';
import 'package:hello_flutter/features/manual/manual_control_screen.dart';

void main() {
  test('exports a multi-zone transition route for plotting', () {
    final zoneA = [
      const Offset(0.0, 0.0),
      const Offset(4.2, 0.2),
      const Offset(4.0, 2.7),
      const Offset(2.7, 3.5),
      const Offset(0.2, 3.1),
    ];
    final zoneB = [
      const Offset(7.0, -0.2),
      const Offset(12.0, 0.0),
      const Offset(12.6, 2.7),
      const Offset(11.0, 5.2),
      const Offset(7.3, 4.8),
      const Offset(6.4, 2.3),
    ];
    final forbiddenA = _rect(1.6, 1.1, 2.3, 1.8);
    final forbiddenB = [
      const Offset(8.7, 1.4),
      const Offset(10.1, 1.7),
      const Offset(10.0, 3.0),
      const Offset(8.6, 3.1),
      const Offset(8.1, 2.2),
    ];
    final transition = [
      const Offset(4.0, 2.0),
      const Offset(5.0, 2.0),
      const Offset(5.6, 0.8),
      const Offset(6.5, 0.8),
      const Offset(7.0, 1.8),
    ];
    final map = ManualMapState.initial().copyWith(
      zones: [PolyShape(zoneA), PolyShape(zoneB)],
      forbiddens: [PolyShape(forbiddenA), PolyShape(forbiddenB)],
      transitions: [transition],
      coordinateType: 'gps',
      refLat: 55.751244,
      refLon: 37.618423,
    );
    final route = CleaningRoutePlanner.planRoute(
      map,
      lineStep: 0.35,
      startOverride: const Offset(-1, 1.3),
    );

    expect(route, isNotNull);
    expect(route!.path.length, lessThanOrEqualTo(254));
    expect(route!.path.any((p) => _distanceToPolyline(p, transition) < 0.03),
        isTrue);
    expect(
      _coveragePercent([zoneA, zoneB], [forbiddenA, forbiddenB], route.path),
      greaterThan(95),
    );

    final exported = {
      'name':
          'complex two-zone map with dogleg transition and forbidden islands',
      'zones': [zoneA, zoneB].map(_encodePoints).toList(),
      'forbiddens': [forbiddenA, forbiddenB].map(_encodePoints).toList(),
      'transitions': [_encodePoints(transition)],
      'start': _encodePoint(const Offset(-1, 1.3)),
      'path': _encodePoints(route.path),
      'pointCount': route.path.length,
      'distanceM': route.totalDistance,
      'coveragePercent': _coveragePercent(
          [zoneA, zoneB], [forbiddenA, forbiddenB], route.path),
    };

    final outDir = Directory('../.pio/build_root/transition_route');
    outDir.createSync(recursive: true);
    File('${outDir.path}/transition_route.json').writeAsStringSync(
        const JsonEncoder.withIndent('  ').convert(exported));
  });
}

List<Map<String, double>> _encodePoints(List<Offset> points) =>
    points.map(_encodePoint).toList();

Map<String, double> _encodePoint(Offset p) => {'x': p.dx, 'y': p.dy};

List<Offset> _rect(double left, double top, double right, double bottom) {
  return [
    Offset(left, top),
    Offset(right, top),
    Offset(right, bottom),
    Offset(left, bottom),
  ];
}

double _distanceToPolyline(Offset p, List<Offset> path) {
  var best = double.infinity;
  for (var i = 1; i < path.length; i++) {
    best = math.min(best, _distanceToSegment(p, path[i - 1], path[i]));
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

double _coveragePercent(
  List<List<Offset>> zones,
  List<List<Offset>> forbiddens,
  List<Offset> path, {
  double robotRadius = 0.25,
  double grid = 0.10,
}) {
  final allPoints = [for (final zone in zones) ...zone];
  final minX = allPoints.map((p) => p.dx).reduce(math.min);
  final maxX = allPoints.map((p) => p.dx).reduce(math.max);
  final minY = allPoints.map((p) => p.dy).reduce(math.min);
  final maxY = allPoints.map((p) => p.dy).reduce(math.max);
  var total = 0;
  var covered = 0;
  for (var x = minX; x <= maxX; x += grid) {
    for (var y = minY; y <= maxY; y += grid) {
      final p = Offset(x, y);
      if (!zones.any((zone) => _pointStrictlyInPolygon(p, zone))) continue;
      if (forbiddens.any((f) => _pointStrictlyInPolygon(p, f))) continue;
      total++;
      if (_distanceToPath(p, path) <= robotRadius) covered++;
    }
  }
  return total == 0 ? 0 : covered * 100.0 / total;
}

double _distanceToPath(Offset p, List<Offset> path) {
  var best = double.infinity;
  for (var i = 1; i < path.length; i++) {
    best = math.min(best, _distanceToSegment(p, path[i - 1], path[i]));
  }
  return best;
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
