import 'dart:convert';
import 'dart:io';

import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/cleaning_route_planner.dart';
import 'package:hello_flutter/features/manual/manual_control_screen.dart';

void main() {
  test('export 10x10 snake HITL route', () {
    const start = Offset(-2, 5);
    final zone = [
      const Offset(0, 0),
      const Offset(10, 0),
      const Offset(10, 10),
      const Offset(0, 10),
    ];
    final forbidden = [
      const Offset(4, 4),
      const Offset(6, 4),
      const Offset(6, 6),
      const Offset(4, 6),
    ];
    final map = ManualMapState.initial().copyWith(
      zones: [PolyShape(zone)],
      forbiddens: [PolyShape(forbidden)],
      coordinateType: 'gps',
      refLat: 55.751244,
      refLon: 37.618423,
    );

    final route = CleaningRoutePlanner.planRoute(
      map,
      lineStep: 0.35,
      borderPasses: 1,
      startOverride: start,
      debugPrint: true,
    );
    expect(route, isNotNull);

    final points = route!.path
        .map((p) => {
              'x': double.parse(p.dx.toStringAsFixed(3)),
              'y': double.parse(p.dy.toStringAsFixed(3)),
            })
        .toList(growable: false);
    final routeSpec = points.map((p) => '${p['x']},${p['y']}').join(';');
    final outDir = Directory('../.pio/build_root')..createSync(recursive: true);
    File('${outDir.path}/hitl_snake_10x10_route.json').writeAsStringSync(
      const JsonEncoder.withIndent('  ').convert({
        'scenario': '10x10 zone, 2x2 forbidden center, start outside',
        'trackStepM': 0.35,
        'start': {'x': start.dx, 'y': start.dy},
        'zone': zone.map((p) => {'x': p.dx, 'y': p.dy}).toList(),
        'forbidden': forbidden.map((p) => {'x': p.dx, 'y': p.dy}).toList(),
        'pointCount': points.length,
        'totalDistanceM': double.parse(route.totalDistance.toStringAsFixed(3)),
        'cleaningSegments': route.cleaningSegments,
        'routeSpec': routeSpec,
        'points': points,
      }),
    );
    File('${outDir.path}/hitl_snake_10x10_route.txt').writeAsStringSync(
      routeSpec,
    );

    print('ROUTE_POINT_COUNT=${points.length}');
    print('ROUTE_TOTAL_DISTANCE_M=${route.totalDistance.toStringAsFixed(3)}');
    print('ROUTE_CLEANING_SEGMENTS=${route.cleaningSegments}');
    print('ROUTE_SPEC=$routeSpec');
  });
}
