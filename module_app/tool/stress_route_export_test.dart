import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/cleaning_route_planner.dart';
import 'package:hello_flutter/features/manual/manual_control_screen.dart';

void main() {
  test('exports stress snake routes for plotting', () {
    const robotRadius = 0.25;
    final cases = _cases();
    final exported = <Map<String, Object?>>[];

    for (final c in cases) {
      final route = CleaningRoutePlanner.planRoute(
        _map(c.zone, forbiddens: c.forbiddens),
        lineStep: 0.35,
        startOverride: c.start,
      );
      expect(route, isNotNull, reason: c.name);

      exported.add({
        'name': c.name,
        'sizeM': _sizeMeters(c.zone),
        'zone': _encodePoints(c.zone),
        'forbiddens': c.forbiddens.map(_encodePoints).toList(),
        'start': _encodePoint(c.start),
        'path': _encodePoints(route!.path),
        'pointCount': route.path.length,
        'distanceM': route.totalDistance,
        'turnCount': _turnCount(route.path),
        'sharpTurnCount': _sharpTurnCount(route.path),
        'estimatedRunTimeS': _estimatedRunTimeSeconds(route.totalDistance),
        'estimatedRunTimeMin':
            _estimatedRunTimeSeconds(route.totalDistance) / 60.0,
        'coveragePercent': _coveragePercent(
          c.zone,
          c.forbiddens,
          route.path,
          robotRadius: robotRadius,
          grid: 0.10,
        ),
        'minForbiddenClearanceM': c.forbiddens.isEmpty
            ? null
            : c.forbiddens
                .map((f) => _minPathDistanceToPolygon(route.path, f))
                .reduce(math.min),
      });
    }

    final outDir = Directory('../.pio/build_root/stress_routes');
    outDir.createSync(recursive: true);
    File('${outDir.path}/stress_routes.json').writeAsStringSync(
        const JsonEncoder.withIndent('  ').convert(exported));
  });
}

List<_Case> _cases() {
  return [
    const _Case(
      name: 'rotated diamond',
      zone: [
        Offset(4, 0),
        Offset(8, 4),
        Offset(4, 8),
        Offset(0, 4),
      ],
      start: Offset(-1, 4),
    ),
    const _Case(
      name: 'l shaped polygon',
      zone: [
        Offset(0, 0),
        Offset(8, 0),
        Offset(8, 3),
        Offset(3, 3),
        Offset(3, 8),
        Offset(0, 8),
      ],
      start: Offset(-1, 1.5),
    ),
    const _Case(
      name: 'u shaped polygon',
      zone: [
        Offset(0, 0),
        Offset(8, 0),
        Offset(8, 8),
        Offset(5.5, 8),
        Offset(5.5, 2.5),
        Offset(2.5, 2.5),
        Offset(2.5, 8),
        Offset(0, 8),
      ],
      start: Offset(-1, 1.2),
    ),
    _Case(
      name: 'skewed trapezoid with two islands',
      zone: const [
        Offset(0, 0),
        Offset(9, 0.8),
        Offset(7.5, 7),
        Offset(1.2, 8.2),
      ],
      forbiddens: [
        _rect(2.2, 2.2, 3.4, 3.6),
        _rect(5.4, 4.1, 6.5, 5.4),
      ],
      start: const Offset(-1, 3.5),
    ),
    const _Case(
      name: 'narrow waist remains navigable',
      zone: [
        Offset(0, 0),
        Offset(9, 0),
        Offset(9, 3.5),
        Offset(5.2, 3.5),
        Offset(5.2, 4.7),
        Offset(9, 4.7),
        Offset(9, 8),
        Offset(0, 8),
        Offset(0, 4.7),
        Offset(3.8, 4.7),
        Offset(3.8, 3.5),
        Offset(0, 3.5),
      ],
      start: Offset(-1, 1.8),
    ),
    _Case(
      name: 'wavy amoeba zone with triangle and diamond forbidden',
      zone: _wavyBlob(
        center: const Offset(5, 5),
        baseRadius: 4.2,
        count: 34,
        wave1: 0.55,
        wave2: 0.28,
      ),
      forbiddens: [
        const [
          Offset(3.0, 3.2),
          Offset(4.3, 3.7),
          Offset(3.5, 4.8),
        ],
        const [
          Offset(6.2, 5.0),
          Offset(7.0, 5.8),
          Offset(6.2, 6.6),
          Offset(5.4, 5.8),
        ],
      ],
      start: const Offset(0.2, 5),
    ),
    _Case(
      name: 'gear shaped cleaning zone with wavy forbidden island',
      zone: _gearZone(),
      forbiddens: [
        _wavyBlob(
          center: const Offset(5, 5),
          baseRadius: 0.9,
          count: 18,
          wave1: 0.18,
          wave2: 0.10,
        ),
      ],
      start: const Offset(-0.7, 4.8),
    ),
    _Case(
      name: 'crooked c corridor with two forbidden pockets',
      zone: const [
        Offset(0, 0),
        Offset(9, 0.4),
        Offset(9.4, 2.2),
        Offset(3.0, 2.0),
        Offset(2.6, 3.4),
        Offset(8.9, 3.8),
        Offset(8.4, 5.6),
        Offset(2.5, 5.3),
        Offset(2.2, 6.7),
        Offset(9.1, 7.4),
        Offset(8.2, 9.0),
        Offset(0.4, 8.3),
        Offset(0.8, 5.0),
        Offset(0.1, 2.2),
      ],
      forbiddens: [
        _rotatedRect(const Offset(4.8, 4.45), 1.1, 0.55, 24),
        const [
          Offset(6.5, 6.0),
          Offset(7.3, 6.35),
          Offset(6.9, 7.05),
        ],
      ],
      start: const Offset(-1.0, 1.2),
    ),
    _Case(
      name: 'saw blade boundary and star forbidden',
      zone: _sawBladeZone(),
      forbiddens: [
        _star(center: const Offset(4.8, 4.4), outer: 1.0, inner: 0.45)
      ],
      start: const Offset(-1, 4.2),
    ),
    _Case(
      name: 'large 20x15 production field with three forbidden islands',
      zone: const [
        Offset(0, 0),
        Offset(20, 0),
        Offset(20, 15),
        Offset(0, 15),
      ],
      forbiddens: [
        _rect(4.0, 3.2, 6.1, 5.6),
        _rotatedRect(const Offset(12.0, 7.5), 2.4, 1.2, 18),
        _rect(15.5, 11.0, 17.8, 13.3),
      ],
      start: const Offset(-1.2, 1.0),
    ),
    _Case(
      name: 'large dogleg 24x14 with transition neck and red service pad',
      zone: const [
        Offset(0, 0),
        Offset(10, 0),
        Offset(10, 4.2),
        Offset(24, 4.2),
        Offset(24, 14),
        Offset(4, 14),
        Offset(4, 9.4),
        Offset(0, 9.4),
      ],
      forbiddens: [
        _rect(6.0, 1.4, 8.2, 3.4),
        _rect(13.5, 6.5, 16.2, 9.0),
        _rotatedRect(const Offset(20.5, 11.2), 2.4, 1.4, -25),
      ],
      start: const Offset(-1.0, 1.0),
    ),
    _Case(
      name: 'large 20x20 square with two small forbidden zones',
      zone: const [
        Offset(0, 0),
        Offset(20, 0),
        Offset(20, 20),
        Offset(0, 20),
      ],
      forbiddens: [
        _rect(5.0, 5.0, 6.4, 6.2),
        _rotatedRect(const Offset(14.0, 13.0), 1.6, 1.1, -22),
      ],
      start: const Offset(-1.0, 1.0),
    ),
  ];
}

ManualMapState _map(
  List<Offset> zone, {
  List<List<Offset>> forbiddens = const [],
}) {
  return ManualMapState.initial().copyWith(
    zones: [PolyShape(zone)],
    forbiddens: forbiddens.map(PolyShape.new).toList(),
    coordinateType: 'gps',
    refLat: 55.751244,
    refLon: 37.618423,
  );
}

List<Map<String, double>> _encodePoints(List<Offset> points) =>
    points.map(_encodePoint).toList();

Map<String, double> _encodePoint(Offset p) => {'x': p.dx, 'y': p.dy};

Map<String, double> _sizeMeters(List<Offset> points) {
  var minX = points.first.dx;
  var maxX = points.first.dx;
  var minY = points.first.dy;
  var maxY = points.first.dy;
  for (final p in points.skip(1)) {
    minX = math.min(minX, p.dx);
    maxX = math.max(maxX, p.dx);
    minY = math.min(minY, p.dy);
    maxY = math.max(maxY, p.dy);
  }
  return {'width': maxX - minX, 'height': maxY - minY};
}

double _estimatedRunTimeSeconds(double distanceM) {
  // Field estimate: average speed below max because turns, segment alignment,
  // RTK filtering and forbidden-zone slowdowns take time on the physical robot.
  return distanceM / 0.28 + 120.0;
}

int _turnCount(List<Offset> path) {
  var count = 0;
  for (var i = 2; i < path.length; i++) {
    final angle = _turnAngleDeg(path[i - 2], path[i - 1], path[i]);
    if (angle >= 20.0) count++;
  }
  return count;
}

int _sharpTurnCount(List<Offset> path) {
  var count = 0;
  for (var i = 2; i < path.length; i++) {
    final angle = _turnAngleDeg(path[i - 2], path[i - 1], path[i]);
    if (angle >= 100.0) count++;
  }
  return count;
}

double _turnAngleDeg(Offset a, Offset b, Offset c) {
  final ab = b - a;
  final bc = c - b;
  final abLen = ab.distance;
  final bcLen = bc.distance;
  if (abLen < 1e-6 || bcLen < 1e-6) return 0.0;
  final dot = (ab.dx * bc.dx + ab.dy * bc.dy) / (abLen * bcLen);
  final clamped = dot.clamp(-1.0, 1.0).toDouble();
  return math.acos(clamped) * 180.0 / math.pi;
}

class _Case {
  final String name;
  final List<Offset> zone;
  final List<List<Offset>> forbiddens;
  final Offset start;

  const _Case({
    required this.name,
    required this.zone,
    this.forbiddens = const [],
    required this.start,
  });
}

List<Offset> _wavyBlob({
  required Offset center,
  required double baseRadius,
  required int count,
  required double wave1,
  required double wave2,
}) {
  final points = <Offset>[];
  for (var i = 0; i < count; i++) {
    final a = math.pi * 2 * i / count;
    final r = baseRadius +
        math.sin(a * 3.0 + 0.4) * wave1 +
        math.cos(a * 7.0 - 0.2) * wave2;
    points
        .add(Offset(center.dx + math.cos(a) * r, center.dy + math.sin(a) * r));
  }
  return points;
}

List<Offset> _gearZone() {
  final points = <Offset>[];
  const center = Offset(5, 5);
  for (var i = 0; i < 28; i++) {
    final a = math.pi * 2 * i / 28;
    final r = i.isEven ? 4.5 : 3.65;
    points
        .add(Offset(center.dx + math.cos(a) * r, center.dy + math.sin(a) * r));
  }
  return points;
}

List<Offset> _sawBladeZone() {
  return const [
    Offset(0.0, 0.8),
    Offset(1.2, 0.0),
    Offset(2.0, 0.7),
    Offset(3.0, 0.0),
    Offset(4.2, 0.8),
    Offset(5.4, 0.1),
    Offset(6.3, 0.9),
    Offset(7.5, 0.3),
    Offset(8.8, 1.2),
    Offset(8.3, 2.1),
    Offset(9.0, 3.2),
    Offset(8.2, 4.0),
    Offset(8.9, 5.2),
    Offset(7.8, 5.9),
    Offset(8.4, 7.2),
    Offset(7.0, 7.8),
    Offset(5.8, 7.2),
    Offset(4.7, 8.1),
    Offset(3.5, 7.3),
    Offset(2.2, 8.0),
    Offset(1.2, 7.0),
    Offset(0.4, 7.6),
    Offset(0.0, 6.2),
    Offset(0.7, 5.2),
    Offset(0.0, 4.1),
    Offset(0.8, 3.0),
    Offset(0.1, 1.9),
  ];
}

List<Offset> _rect(double left, double top, double right, double bottom) {
  return [
    Offset(left, top),
    Offset(right, top),
    Offset(right, bottom),
    Offset(left, bottom),
  ];
}

List<Offset> _rotatedRect(
    Offset center, double width, double height, double deg) {
  final a = deg * math.pi / 180;
  final ca = math.cos(a);
  final sa = math.sin(a);
  final hw = width / 2;
  final hh = height / 2;
  return [
    _rotate(center, -hw, -hh, ca, sa),
    _rotate(center, hw, -hh, ca, sa),
    _rotate(center, hw, hh, ca, sa),
    _rotate(center, -hw, hh, ca, sa),
  ];
}

Offset _rotate(Offset center, double x, double y, double ca, double sa) {
  return Offset(center.dx + x * ca - y * sa, center.dy + x * sa + y * ca);
}

List<Offset> _star({
  required Offset center,
  required double outer,
  required double inner,
}) {
  final points = <Offset>[];
  for (var i = 0; i < 10; i++) {
    final a = -math.pi / 2 + math.pi * 2 * i / 10;
    final r = i.isEven ? outer : inner;
    points
        .add(Offset(center.dx + math.cos(a) * r, center.dy + math.sin(a) * r));
  }
  return points;
}

double _coveragePercent(
  List<Offset> zone,
  List<List<Offset>> forbiddens,
  List<Offset> path, {
  required double robotRadius,
  required double grid,
}) {
  final box = _bbox(zone);
  var safe = 0;
  var covered = 0;
  for (var y = box.minY + grid * 0.5; y < box.maxY; y += grid) {
    for (var x = box.minX + grid * 0.5; x < box.maxX; x += grid) {
      final p = Offset(x, y);
      if (!_pointStrictlyInPolygon(p, zone)) continue;
      if (forbiddens.any((f) => _pointStrictlyInPolygon(p, f))) continue;
      safe++;
      if (_distanceToPolyline(p, path) <= robotRadius + grid * 0.75) {
        covered++;
      }
    }
  }
  return safe == 0 ? 0 : covered * 100 / safe;
}

double _minPathDistanceToPolygon(List<Offset> path, List<Offset> polygon) {
  var best = double.infinity;
  for (var i = 1; i < path.length; i++) {
    final a = path[i - 1];
    final b = path[i];
    final samples = math.max(2, ((b - a).distance / 0.03).ceil());
    for (var s = 0; s <= samples; s++) {
      final t = s / samples;
      final p = Offset(
        a.dx + (b.dx - a.dx) * t,
        a.dy + (b.dy - a.dy) * t,
      );
      best = math.min(best, _distanceToPolygon(p, polygon));
      if (_pointStrictlyInPolygon(p, polygon)) return 0;
    }
  }
  return best;
}

double _distanceToPolyline(Offset p, List<Offset> path) {
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

double _distanceToPolygon(Offset p, List<Offset> polygon) {
  var best = double.infinity;
  for (var i = 0; i < polygon.length; i++) {
    best = math.min(
      best,
      _distanceToSegment(p, polygon[i], polygon[(i + 1) % polygon.length]),
    );
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

_Box _bbox(List<Offset> points) {
  var minX = points.first.dx;
  var maxX = points.first.dx;
  var minY = points.first.dy;
  var maxY = points.first.dy;
  for (final p in points.skip(1)) {
    minX = math.min(minX, p.dx);
    maxX = math.max(maxX, p.dx);
    minY = math.min(minY, p.dy);
    maxY = math.max(maxY, p.dy);
  }
  return _Box(minX, maxX, minY, maxY);
}

class _Box {
  final double minX;
  final double maxX;
  final double minY;
  final double maxY;
  const _Box(this.minX, this.maxX, this.minY, this.maxY);
}
