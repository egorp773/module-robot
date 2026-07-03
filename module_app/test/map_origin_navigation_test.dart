import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/gps_display_math.dart';
import 'package:hello_flutter/core/map_storage.dart';
import 'package:hello_flutter/core/wifi_connection.dart';

Map<String, dynamic> gpsMapJson({
  required double robotX,
  required double robotY,
}) {
  return <String, dynamic>{
    'id': 'zone-1',
    'name': 'RTK zone',
    'coordinateType': 'gps',
    'mapOriginLat': 55.751244,
    'mapOriginLon': 37.618423,
    // Old aliases deliberately disagree: canonical mapOrigin* must win.
    'refLat': 1.0,
    'refLon': 2.0,
    'robot': <String, dynamic>{'x': robotX, 'y': robotY},
    'zones': <dynamic>[],
    'forbiddens': <dynamic>[],
    'transitions': <dynamic>[],
  };
}

void main() {
  test('saved map origin is canonical and independent of robot start', () {
    final firstStart =
        MapStorage.mapFromJson(gpsMapJson(robotX: 0.2, robotY: 0.3));
    final secondStart =
        MapStorage.mapFromJson(gpsMapJson(robotX: 2.1, robotY: -0.4));

    expect(firstStart.refLat, 55.751244);
    expect(firstStart.refLon, 37.618423);
    expect(secondStart.refLat, firstStart.refLat);
    expect(secondStart.refLon, firstStart.refLon);
    expect(secondStart.robot, isNot(firstStart.robot));

    final firstProjection = GpsDisplayGeometry(
      originLat: firstStart.refLat!,
      originLon: firstStart.refLon!,
    ).toLocal(55.751344, 37.618523);
    final secondProjection = GpsDisplayGeometry(
      originLat: secondStart.refLat!,
      originLon: secondStart.refLon!,
    ).toLocal(55.751344, 37.618523);

    expect(firstProjection.x, closeTo(6.2793, 0.01));
    expect(firstProjection.y, closeTo(11.132, 0.01));
    expect(secondProjection.x, firstProjection.x);
    expect(secondProjection.y, firstProjection.y);

    final commandFromFirstStart = buildRouteBeginCommand(
      4,
      mapOriginLat: firstStart.refLat!,
      mapOriginLon: firstStart.refLon!,
    );
    final commandFromSecondStart = buildRouteBeginCommand(
      4,
      mapOriginLat: secondStart.refLat!,
      mapOriginLon: secondStart.refLon!,
    );
    expect(commandFromSecondStart, commandFromFirstStart);
    expect(
      commandFromFirstStart,
      'ROUTE_BEGIN,4,55.75124400,37.61842300',
    );
  });

  test('legacy ref origin still loads', () {
    final json = gpsMapJson(robotX: 0, robotY: 0)
      ..remove('mapOriginLat')
      ..remove('mapOriginLon')
      ..['refLat'] = 55.0
      ..['refLon'] = 37.0;
    final map = MapStorage.mapFromJson(json);
    expect(map.refLat, 55.0);
    expect(map.refLon, 37.0);
  });
}
