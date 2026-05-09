import 'dart:math' as math;

class LocalPointMeters {
  final double x;
  final double y;

  const LocalPointMeters({required this.x, required this.y});
}

class GpsDisplayGeometry {
  final double originLat;
  final double originLon;
  late final double metersPerDegreeLat;
  late final double metersPerDegreeLon;

  GpsDisplayGeometry({required this.originLat, required this.originLon}) {
    final latRad = _degToRad(originLat);
    metersPerDegreeLat = 111132.92 -
        559.82 * math.cos(2 * latRad) +
        1.175 * math.cos(4 * latRad) -
        0.0023 * math.cos(6 * latRad);
    metersPerDegreeLon = 111412.84 * math.cos(latRad) -
        93.5 * math.cos(3 * latRad) +
        0.118 * math.cos(5 * latRad);
  }

  LocalPointMeters toLocal(double lat, double lon) {
    return LocalPointMeters(
      x: (lon - originLon) * metersPerDegreeLon,
      y: (lat - originLat) * metersPerDegreeLat,
    );
  }

  static double distanceMeters(
    double lat1,
    double lon1,
    double lat2,
    double lon2,
  ) {
    const earthRadiusM = 6371008.8;
    final dLat = _degToRad(lat2 - lat1);
    final dLon = _degToRad(lon2 - lon1);
    final rLat1 = _degToRad(lat1);
    final rLat2 = _degToRad(lat2);
    final a = math.sin(dLat / 2) * math.sin(dLat / 2) +
        math.cos(rLat1) *
            math.cos(rLat2) *
            math.sin(dLon / 2) *
            math.sin(dLon / 2);
    final c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));
    return earthRadiusM * c;
  }

  static double bearingDegrees(
    double fromLat,
    double fromLon,
    double toLat,
    double toLon,
  ) {
    final fromLatRad = _degToRad(fromLat);
    final toLatRad = _degToRad(toLat);
    final dLon = _degToRad(toLon - fromLon);
    final y = math.sin(dLon) * math.cos(toLatRad);
    final x = math.cos(fromLatRad) * math.sin(toLatRad) -
        math.sin(fromLatRad) * math.cos(toLatRad) * math.cos(dLon);
    return normalizeDegrees(_radToDeg(math.atan2(y, x)));
  }

  static double headingErrorDegrees(
    double currentHeadingDegrees,
    double targetBearingDegrees,
  ) {
    final error = normalizeDegrees(targetBearingDegrees) -
        normalizeDegrees(currentHeadingDegrees);
    if (error > 180) return error - 360;
    if (error < -180) return error + 360;
    return error;
  }

  static double normalizeDegrees(double degrees) {
    var value = degrees % 360;
    if (value < 0) value += 360;
    return value;
  }

  static double _degToRad(double value) => value * math.pi / 180.0;

  static double _radToDeg(double value) => value * 180.0 / math.pi;
}
