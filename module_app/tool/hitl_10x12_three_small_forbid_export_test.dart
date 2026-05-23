import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/cleaning_route_planner.dart';
import 'package:hello_flutter/features/manual/manual_control_screen.dart';

void main() {
  test('exports 10x12 route with three 0.5x0.3 forbidden pockets', () {
    final zone = _rect(0, 0, 10, 12);
    final forbiddens = [
      _rect(2.0, 2.0, 2.5, 2.3),
      _rect(6.5, 5.0, 7.0, 5.3),
      _rect(4.0, 9.0, 4.5, 9.3),
    ];

    final route = CleaningRoutePlanner.planRoute(
      _map(zone, forbiddens: forbiddens),
      lineStep: 0.35,
      forbiddenMarginMeters: 0.05,
      startOverride: const Offset(-1, 0.5),
    );

    expect(route, isNotNull);
    expect(route!.path.length, lessThanOrEqualTo(254));

    final outDir = Directory('../.pio/build_root/ten_by_twelve_three_small_forbid');
    outDir.createSync(recursive: true);
    final data = {
      'name': '10x12 with three 0.5x0.3 forbidden pockets',
      'zone': _encodePoints(zone),
      'forbiddens': forbiddens.map(_encodePoints).toList(),
      'path': _encodePoints(route.path),
      'pointCount': route.path.length,
      'distanceM': route.totalDistance,
      'planningForbiddenMarginM': 0.05,
      'minForbiddenClearanceM': forbiddens
          .map((f) => _minPathDistanceToPolygon(route.path, f))
          .reduce(math.min),
    };
    File('${outDir.path}/route.json').writeAsStringSync(
      const JsonEncoder.withIndent('  ').convert(data),
    );
    File('${outDir.path}/route_arg.txt').writeAsStringSync(
      route.path.map((p) => '${p.dx.toStringAsFixed(3)},${p.dy.toStringAsFixed(3)}').join(';'),
    );
    File('${outDir.path}/forbid_args.txt').writeAsStringSync(
      forbiddens
          .map((poly) => poly.map((p) => '${p.dx.toStringAsFixed(3)},${p.dy.toStringAsFixed(3)}').join(';'))
          .join('\n'),
    );
  });
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

List<Offset> _rect(double left, double top, double right, double bottom) {
  return [
    Offset(left, top),
    Offset(right, top),
    Offset(right, bottom),
    Offset(left, bottom),
  ];
}

List<Map<String, double>> _encodePoints(List<Offset> points) =>
    points.map((p) => {'x': p.dx, 'y': p.dy}).toList();

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
    }
  }
  return best;
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
