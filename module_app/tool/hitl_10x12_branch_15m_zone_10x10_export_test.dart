import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/cleaning_route_planner.dart';
import 'package:hello_flutter/features/manual/manual_control_screen.dart';

void main() {
  test('exports 10x12 route with curved 15m transition and second 10x10 zone', () {
    final zoneA = _rect(0, 0, 10, 12);
    final zoneB = _rect(25, 1, 35, 11);
    final transition = _curvedTransition();
    final forbiddens = [
      _rect(2.0, 2.0, 2.5, 2.3),
      _rect(6.5, 5.0, 7.0, 5.3),
      _rect(4.0, 9.0, 4.5, 9.3),
    ];

    final route = CleaningRoutePlanner.planRoute(
      _map([zoneA, zoneB], forbiddens: forbiddens, transitions: [transition]),
      lineStep: 0.35,
      forbiddenMarginMeters: 0.05,
      startOverride: const Offset(0.5, 0.5),
    );

    expect(route, isNotNull);
    expect(route!.path.length, lessThanOrEqualTo(254));
    expect(
      route.path.any((p) => _distanceToPolyline(p, transition) < 0.03),
      isTrue,
      reason: 'route should travel through the stored curved transition line',
    );

    final outDir =
        Directory('../.pio/build_root/ten_by_twelve_branch_15m_zone_10x10');
    outDir.createSync(recursive: true);
    final data = {
      'name': '10x12 with three 0.5x0.3 forbidden pockets, curved 15m transition, and second 10x10 zone',
      'mainZoneM': {'left': 0, 'top': 0, 'right': 10, 'bottom': 12},
      'transitionM': {
        'kind': 'single curved polyline',
        'start': {'x': 10, 'y': 6},
        'end': {'x': 25, 'y': 6},
        'straightSpanM': 15,
        'pointCount': transition.length,
      },
      'secondZoneM': {'left': 25, 'top': 1, 'right': 35, 'bottom': 11},
      'zones': [zoneA, zoneB].map(_encodePoints).toList(),
      'transitions': [_encodePoints(transition)],
      'forbiddens': forbiddens.map(_encodePoints).toList(),
      'path': _encodePoints(route.path),
      'pointCount': route.path.length,
      'distanceM': route.totalDistance,
      'transitionLengthM': _pathDistance(transition),
      'planningForbiddenMarginM': 0.05,
      'minForbiddenClearanceM': forbiddens
          .map((f) => _minPathDistanceToPolygon(route.path, f))
          .reduce(math.min),
    };
    File('${outDir.path}/route.json').writeAsStringSync(
      const JsonEncoder.withIndent('  ').convert(data),
    );
    File('${outDir.path}/route_arg.txt').writeAsStringSync(
      route.path
          .map((p) => '${p.dx.toStringAsFixed(3)},${p.dy.toStringAsFixed(3)}')
          .join(';'),
    );
    File('${outDir.path}/forbid_args.txt').writeAsStringSync(
      forbiddens
          .map((poly) => poly
              .map((p) => '${p.dx.toStringAsFixed(3)},${p.dy.toStringAsFixed(3)}')
              .join(';'))
          .join('\n'),
    );
    File('${outDir.path}/transition_arg.txt').writeAsStringSync(
      transition
          .map((p) => '${p.dx.toStringAsFixed(3)},${p.dy.toStringAsFixed(3)}')
          .join(';'),
    );
  });
}

ManualMapState _map(
  List<List<Offset>> zones, {
  List<List<Offset>> forbiddens = const [],
  List<List<Offset>> transitions = const [],
}) {
  return ManualMapState.initial().copyWith(
    zones: zones.map(PolyShape.new).toList(),
    forbiddens: forbiddens.map(PolyShape.new).toList(),
    transitions: transitions,
    coordinateType: 'gps',
    refLat: 55.751244,
    refLon: 37.618423,
  );
}

List<Offset> _curvedTransition() {
  const anchors = [
    Offset(10.0, 6.0),
    Offset(11.6, 8.8),
    Offset(14.3, 8.2),
    Offset(16.0, 3.1),
    Offset(18.8, 2.8),
    Offset(20.8, 7.8),
    Offset(23.3, 8.6),
    Offset(25.0, 6.0),
  ];
  final out = <Offset>[anchors.first];
  for (var i = 0; i < anchors.length - 1; i++) {
    final p0 = anchors[math.max(0, i - 1)];
    final p1 = anchors[i];
    final p2 = anchors[i + 1];
    final p3 = anchors[math.min(anchors.length - 1, i + 2)];
    for (var s = 1; s <= 8; s++) {
      out.add(_catmullRom(p0, p1, p2, p3, s / 8));
    }
  }
  return out;
}

Offset _catmullRom(Offset p0, Offset p1, Offset p2, Offset p3, double t) {
  final t2 = t * t;
  final t3 = t2 * t;
  final x = 0.5 *
      ((2 * p1.dx) +
          (-p0.dx + p2.dx) * t +
          (2 * p0.dx - 5 * p1.dx + 4 * p2.dx - p3.dx) * t2 +
          (-p0.dx + 3 * p1.dx - 3 * p2.dx + p3.dx) * t3);
  final y = 0.5 *
      ((2 * p1.dy) +
          (-p0.dy + p2.dy) * t +
          (2 * p0.dy - 5 * p1.dy + 4 * p2.dy - p3.dy) * t2 +
          (-p0.dy + 3 * p1.dy - 3 * p2.dy + p3.dy) * t3);
  return Offset(x, y);
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

double _distanceToPolyline(Offset p, List<Offset> path) {
  var best = double.infinity;
  for (var i = 1; i < path.length; i++) {
    best = math.min(best, _distanceToSegment(p, path[i - 1], path[i]));
  }
  return best;
}

double _pathDistance(List<Offset> path) {
  var total = 0.0;
  for (var i = 1; i < path.length; i++) {
    total += (path[i] - path[i - 1]).distance;
  }
  return total;
}
