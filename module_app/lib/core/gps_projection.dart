import 'dart:ui';
import 'dart:math' as math;

/// GPS projection: converts GPS coordinates to/from local meters
/// Uses flat Earth approximation (accurate <1mm for areas up to 1 km²)
class GpsProjection {
  final double refLat;
  final double refLon;
  late final double mPerDegLat;
  late final double mPerDegLon;

  GpsProjection({required this.refLat, required this.refLon}) {
    // Meters per degree of latitude (constant)
    mPerDegLat = 111132.92 - 559.82 * math.cos(2 * refLat * math.pi / 180) +
                 1.175 * math.cos(4 * refLat * math.pi / 180) -
                 0.0023 * math.cos(6 * refLat * math.pi / 180);

    // Meters per degree of longitude (depends on latitude)
    mPerDegLon = 111412.84 * math.cos(refLat * math.pi / 180) -
                 93.5 * math.cos(3 * refLat * math.pi / 180) +
                 0.118 * math.cos(5 * refLat * math.pi / 180);
  }

  /// Convert GPS coordinates to local meters
  /// Returns Offset(east, north) in meters relative to reference point
  Offset toLocal(double lat, double lon) {
    final dLat = lat - refLat;
    final dLon = lon - refLon;

    final north = dLat * mPerDegLat;
    final east = dLon * mPerDegLon;

    return Offset(east, north);
  }

  /// Convert local meters to GPS coordinates
  /// Takes Offset(east, north) in meters and returns (lat, lon)
  (double, double) toGps(Offset local) {
    final dLat = local.dy / mPerDegLat;
    final dLon = local.dx / mPerDegLon;

    final lat = refLat + dLat;
    final lon = refLon + dLon;

    return (lat, lon);
  }

  /// Calculate distance between two GPS points in meters
  static double distance(double lat1, double lon1, double lat2, double lon2) {
    const R = 6371000.0; // Earth radius in meters
    final dLat = (lat2 - lat1) * math.pi / 180;
    final dLon = (lon2 - lon1) * math.pi / 180;

    final a = math.sin(dLat / 2) * math.sin(dLat / 2) +
              math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) *
              math.sin(dLon / 2) * math.sin(dLon / 2);

    final c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));
    return R * c;
  }

  /// Calculate bearing from point 1 to point 2 in degrees (0-360)
  static double bearing(double lat1, double lon1, double lat2, double lon2) {
    final dLon = (lon2 - lon1) * math.pi / 180;
    final y = math.sin(dLon) * math.cos(lat2 * math.pi / 180);
    final x = math.cos(lat1 * math.pi / 180) * math.sin(lat2 * math.pi / 180) -
              math.sin(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.cos(dLon);

    var brng = math.atan2(y, x) * 180 / math.pi;
    if (brng < 0) brng += 360;
    return brng;
  }
}
