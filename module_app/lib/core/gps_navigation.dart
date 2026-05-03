import 'dart:math' as math;

class LocalPointMeters {
  final double x;
  final double y;

  const LocalPointMeters({required this.x, required this.y});
}

class GpsLocalGeometry {
  final double originLat;
  final double originLon;
  late final double metersPerDegreeLat;
  late final double metersPerDegreeLon;

  GpsLocalGeometry({required this.originLat, required this.originLon}) {
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

class HeadingCalibration {
  final double offsetDegrees;
  final bool invertYaw;

  const HeadingCalibration({
    this.offsetDegrees = 0,
    this.invertYaw = false,
  });

  double apply(double rawDegrees) {
    final yaw = invertYaw ? -rawDegrees : rawDegrees;
    return GpsLocalGeometry.normalizeDegrees(yaw + offsetDegrees);
  }

  HeadingCalibration alignRawToTarget({
    required double rawDegrees,
    required double targetDegrees,
  }) {
    final yaw = invertYaw ? -rawDegrees : rawDegrees;
    return HeadingCalibration(
      offsetDegrees: GpsLocalGeometry.normalizeDegrees(targetDegrees - yaw),
      invertYaw: invertYaw,
    );
  }
}

enum NavigationCommand {
  stop,
  turnLeft,
  turnRight,
  forward,
  arrived,
}

extension NavigationCommandText on NavigationCommand {
  String get wireName {
    switch (this) {
      case NavigationCommand.stop:
        return 'STOP';
      case NavigationCommand.turnLeft:
        return 'TURN_LEFT';
      case NavigationCommand.turnRight:
        return 'TURN_RIGHT';
      case NavigationCommand.forward:
        return 'FORWARD';
      case NavigationCommand.arrived:
        return 'ARRIVED';
    }
  }
}

class NavigationResult {
  final NavigationCommand command;
  final String reason;
  final LocalPointMeters? currentLocal;
  final LocalPointMeters? targetLocal;
  final double? distanceMeters;
  final double? bearingDegrees;
  final double? headingErrorDegrees;

  const NavigationResult({
    required this.command,
    required this.reason,
    this.currentLocal,
    this.targetLocal,
    this.distanceMeters,
    this.bearingDegrees,
    this.headingErrorDegrees,
  });

  static const noTarget = NavigationResult(
    command: NavigationCommand.stop,
    reason: 'Нет цели',
  );
}

class MotorDriveCommand {
  final int left;
  final int right;
  final String label;

  const MotorDriveCommand({
    required this.left,
    required this.right,
    required this.label,
  });

  bool get isStop => left == 0 && right == 0;

  String get protocol => isStop ? 'STOP' : 'M,$left,$right';
}

class NavigationMotorMapper {
  const NavigationMotorMapper();

  MotorDriveCommand toMotorCommand(
    NavigationCommand command, {
    int forwardPercent = 22,
    int turnPercent = 18,
    bool invertForward = false,
    bool invertSteering = false,
  }) {
    final forward = forwardPercent.clamp(0, 45) * (invertForward ? -1 : 1);
    final turn = turnPercent.clamp(0, 40) * (invertSteering ? -1 : 1);
    switch (command) {
      case NavigationCommand.forward:
        return MotorDriveCommand(
          left: forward,
          right: forward,
          label: 'ехать вперед',
        );
      case NavigationCommand.turnLeft:
        return MotorDriveCommand(
          left: -turn,
          right: turn,
          label: 'поворот влево',
        );
      case NavigationCommand.turnRight:
        return MotorDriveCommand(
          left: turn,
          right: -turn,
          label: 'поворот вправо',
        );
      case NavigationCommand.stop:
        return const MotorDriveCommand(left: 0, right: 0, label: 'стоп');
      case NavigationCommand.arrived:
        return const MotorDriveCommand(
            left: 0, right: 0, label: 'точка достигнута');
    }
  }
}

class GpsNavigationController {
  static const double maxRtcmAgeMs = 1500;
  static const int maxRoverGpsAgeMs = 1500;
  static const int maxHorizontalAccuracyMm = 50;
  static const double arrivedDistanceM = 0.3;
  static const double turnThresholdDeg = 20;
  static const Duration maxGpsAge = Duration(seconds: 5);

  const GpsNavigationController();

  NavigationResult evaluate({
    required double? currentLat,
    required double? currentLon,
    required double? targetLat,
    required double? targetLon,
    required double? headingDegrees,
    required bool rtkFixed,
    required int? rtcmAgeMs,
    required int? hAccMm,
    required double? originLat,
    required double? originLon,
    int? gpsFixType,
    int? gpsAgeMs,
    DateTime? gpsReceivedAt,
    DateTime? now,
  }) {
    if (currentLat == null ||
        currentLon == null ||
        targetLat == null ||
        targetLon == null) {
      return NavigationResult.noTarget;
    }
    if (currentLat.abs() < 0.000001 && currentLon.abs() < 0.000001) {
      return const NavigationResult(
        command: NavigationCommand.stop,
        reason: 'Нет валидных координат',
      );
    }
    if (gpsReceivedAt != null &&
        (now ?? DateTime.now()).difference(gpsReceivedAt) > maxGpsAge) {
      return const NavigationResult(
        command: NavigationCommand.stop,
        reason: 'GPS-данные устарели',
      );
    }
    if (gpsFixType != null && gpsFixType < 3) {
      return const NavigationResult(
        command: NavigationCommand.stop,
        reason: 'GPS fix слабый',
      );
    }

    if (gpsAgeMs != null && gpsAgeMs > maxRoverGpsAgeMs) {
      return const NavigationResult(
        command: NavigationCommand.stop,
        reason: 'GPS rover stale',
      );
    }

    final geometry = GpsLocalGeometry(
      originLat: originLat ?? currentLat,
      originLon: originLon ?? currentLon,
    );
    final currentLocal = geometry.toLocal(currentLat, currentLon);
    final targetLocal = geometry.toLocal(targetLat, targetLon);
    final distance = GpsLocalGeometry.distanceMeters(
      currentLat,
      currentLon,
      targetLat,
      targetLon,
    );
    final bearing = GpsLocalGeometry.bearingDegrees(
      currentLat,
      currentLon,
      targetLat,
      targetLon,
    );
    final headingError = headingDegrees == null
        ? null
        : GpsLocalGeometry.headingErrorDegrees(headingDegrees, bearing);

    NavigationResult result(NavigationCommand command, String reason) {
      return NavigationResult(
        command: command,
        reason: reason,
        currentLocal: currentLocal,
        targetLocal: targetLocal,
        distanceMeters: distance,
        bearingDegrees: bearing,
        headingErrorDegrees: headingError,
      );
    }

    if (!rtkFixed) {
      return result(NavigationCommand.stop, 'RTK не fixed');
    }
    if (rtcmAgeMs == null || rtcmAgeMs > maxRtcmAgeMs) {
      return result(NavigationCommand.stop, 'RTCM старый');
    }
    if (hAccMm == null || hAccMm > maxHorizontalAccuracyMm) {
      return result(NavigationCommand.stop, 'hAcc больше 50 мм');
    }
    if (distance < arrivedDistanceM) {
      return result(NavigationCommand.arrived, 'Цель достигнута');
    }
    if (headingError == null) {
      return result(NavigationCommand.stop, 'Нет курса');
    }
    if (headingError.abs() > turnThresholdDeg) {
      return result(
        headingError > 0
            ? NavigationCommand.turnRight
            : NavigationCommand.turnLeft,
        'Повернуть на курс',
      );
    }
    return result(NavigationCommand.forward, 'Ехать вперед');
  }
}
