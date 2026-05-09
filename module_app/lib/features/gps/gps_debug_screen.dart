import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:go_router/go_router.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../../core/gps_navigation.dart';
import '../../core/gps_perimeter_storage.dart';
import '../../core/gps_projection.dart';
import '../../core/wifi_connection.dart';

@visibleForTesting
List<GpsPerimeterPoint> gpsDebugVisibleMapPoints({
  required List<GpsPerimeterPoint> currentPerimeter,
  required List<GpsPerimeterPoint> openedPerimeter,
  required List<GpsPerimeterPoint> track,
  required GpsPerimeterPoint? currentPosition,
}) {
  if (currentPerimeter.isNotEmpty) {
    return List<GpsPerimeterPoint>.unmodifiable(currentPerimeter);
  }
  if (openedPerimeter.isNotEmpty) {
    return List<GpsPerimeterPoint>.unmodifiable(openedPerimeter);
  }
  if (track.isNotEmpty) {
    return List<GpsPerimeterPoint>.unmodifiable(track);
  }
  if (currentPosition != null) {
    return List<GpsPerimeterPoint>.unmodifiable([currentPosition]);
  }
  return const [];
}

class GpsDebugScreen extends ConsumerStatefulWidget {
  const GpsDebugScreen({super.key});

  @override
  ConsumerState<GpsDebugScreen> createState() => _GpsDebugScreenState();
}

class _GpsDebugScreenState extends ConsumerState<GpsDebugScreen> {
  static const double _autoPointMinDistanceM = 0.20;
  static const double _trailMinDistanceM = 0.10;
  static const int _trailMaxPoints = 900;
  static const int _roverMaxWaypoints = 128;
  static const String _prefsHeadingOffsetKey = 'gps_nav_heading_offset';
  static const String _prefsInvertYawKey = 'gps_nav_invert_yaw';
  static const String _prefsInvertForwardKey = 'gps_nav_invert_forward';
  static const String _prefsInvertSteeringKey = 'gps_nav_invert_steering';
  static const String _prefsForwardPercentKey = 'gps_nav_forward_percent';
  static const String _prefsTurnPercentKey = 'gps_nav_turn_percent';
  static const String _prefsHeadingCalibratedKey = 'gps_nav_heading_calibrated';

  final List<GpsPerimeterPoint> _points = [];
  final List<GpsPerimeterPoint> _trail = [];
  GpsPerimeter? _activeSaved;
  GpsPerimeterPoint? _navigationOrigin;
  GpsPerimeterPoint? _navigationTarget;
  int? _navigationTargetIndex;
  int _roverRouteStartIndex = 0;
  NavigationCommand? _lastLoggedNavigationCommand;
  String? _lastRoverNavState;
  DateTime? _lastNavigationLogAt;
  Timer? _motorDriveTimer;
  bool _motorsEnabled = false;
  bool _routeRunning = false;
  int _forwardPercent = 22;
  int _turnPercent = 18;
  HeadingCalibration _headingCalibration = const HeadingCalibration();
  bool _headingCalibrated = false;
  bool _invertForward = false;
  bool _invertSteering = false;
  double? _lastStableHeadingDegrees;
  DateTime? _lastStableHeadingAt;
  double? _smoothedHeadingDegrees;
  DateTime? _lastHeadingSampleAt;
  final TextEditingController _nameCtrl = TextEditingController();
  final TextEditingController _hostCtrl = TextEditingController();
  final FocusNode _hostFocus = FocusNode();
  Timer? _recordTimer;
  bool _recording = false;
  String? _notice;
  Future<List<GpsPerimeter>>? _savedFuture;

  @override
  void initState() {
    super.initState();
    _savedFuture = GpsPerimeterStorage.list();
    _hostCtrl.text = WifiRobotHostNotifier.defaultHost;
    final stamp = DateTime.now();
    _nameCtrl.text =
        'GPS ${stamp.year}-${_two(stamp.month)}-${_two(stamp.day)} ${_two(stamp.hour)}-${_two(stamp.minute)}';
    _loadNavigationSettings();
  }

  @override
  void dispose() {
    _recordTimer?.cancel();
    _motorDriveTimer?.cancel();
    _nameCtrl.dispose();
    _hostCtrl.dispose();
    _hostFocus.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final wifi = ref.watch(wifiConnectionProvider);
    final robotHost = ref.watch(wifiRobotHostProvider);
    final ctrl = ref.read(wifiConnectionProvider.notifier);
    final fixOk = _fixOk(wifi);
    final rtkOk = _rtkOk(wifi);
    final precisionOk = _precisionOk(wifi);
    final rtcmText = _rtcmStatusText(wifi.rtcmAgeMs);
    final rtcmFresh = _rtcmFresh(wifi.rtcmAgeMs);
    final accM = wifi.gpsAccuracy == null ? null : wifi.gpsAccuracy! / 1000.0;
    final currentPoint = _currentPoint(wifi);
    final openedPoints = _activeSaved?.points ?? const <GpsPerimeterPoint>[];
    final visibleMapPoints = gpsDebugVisibleMapPoints(
      currentPerimeter: _points,
      openedPerimeter: openedPoints,
      track: _trail,
      currentPosition: currentPoint,
    );
    final navigationPoints = _points.isNotEmpty ? _points : openedPoints;
    final navigationOrigin =
        _navigationOrigin ?? currentPoint ?? _firstPoint(navigationPoints);
    final rawNavigationHeading =
        _hasFreshImu(wifi) ? wifi.imuYaw : wifi.gpsHeading;
    final navigationHeading = _navigationHeadingFor(wifi);
    final navigationHeadingSource = _navigationHeadingSourceFor(wifi);
    final navigation = const GpsNavigationController().evaluate(
      currentLat: wifi.gpsLat,
      currentLon: wifi.gpsLon,
      targetLat: _navigationTarget?.lat,
      targetLon: _navigationTarget?.lon,
      headingDegrees: navigationHeading,
      rtkFixed: _rtkOkForNavigation(wifi),
      rtcmAgeMs: wifi.rtcmAgeMs,
      hAccMm: wifi.gpsAccuracy,
      originLat: navigationOrigin?.lat,
      originLon: navigationOrigin?.lon,
      gpsFixType: wifi.gpsFixType,
      gpsAgeMs: wifi.gpsAgeMs,
      gpsReceivedAt: wifi.gpsReceivedAt,
    );
    final movementBearing = _movementBearing();
    final movementTargetError =
        movementBearing == null || navigation.bearingDegrees == null
            ? null
            : GpsLocalGeometry.headingErrorDegrees(
                movementBearing,
                navigation.bearingDegrees!,
              );
    final motorPreview = const NavigationMotorMapper().toMotorCommandForResult(
      navigation,
      forwardPercent: _forwardPercent,
      turnPercent: _turnPercent,
      invertForward: _invertForward,
      invertSteering: _invertSteering,
    );
    final routeLabel = _navigationTargetIndex == null
        ? 'ручная цель'
        : 'точка ${_navigationTargetIndex! + 1}/${navigationPoints.length}';
    final mapSpanM = _mapSpanM(currentPoint);
    ref.listen<WifiConnectionState>(wifiConnectionProvider, (_, next) {
      _appendTrail(next);
      _maybeLogNavigation(next);
      _syncRoverNavigation(next);
      _handleAutoRouteAdvance(next);
    });
    if (!_hostFocus.hasFocus && _hostCtrl.text != robotHost) {
      _hostCtrl.text = robotHost;
    }

    return Scaffold(
      backgroundColor: const Color(0xFF050608),
      body: SafeArea(
        child: Column(
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(12, 10, 12, 8),
              child: Row(
                children: [
                  _IconButton(
                    icon: Icons.arrow_back_rounded,
                    onTap: () => context.go('/'),
                  ),
                  const SizedBox(width: 10),
                  const Expanded(
                    child: Text(
                      'ZED-F9P GPS Отладка',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w900,
                      ),
                    ),
                  ),
                  _IconButton(
                    icon: wifi.isConnected
                        ? Icons.link_off_rounded
                        : Icons.wifi_rounded,
                    onTap: wifi.isConnecting
                        ? null
                        : () {
                            if (wifi.isConnected) {
                              ctrl.disconnect();
                            } else {
                              ctrl.connect(skipPreflight: true);
                            }
                          },
                  ),
                ],
              ),
            ),
            Expanded(
              child: ListView(
                padding: const EdgeInsets.fromLTRB(12, 0, 12, 14),
                children: [
                  if (_notice != null)
                    _Notice(
                      text: _notice!,
                      onClose: () => setState(() => _notice = null),
                    ),
                  _StatusStrip(
                    connected: wifi.isConnected,
                    connecting: wifi.isConnecting,
                    fixOk: fixOk,
                    rtkOk: rtkOk,
                    carrier: wifi.gpsCarrier,
                    rtcmText: rtcmText,
                  ),
                  const SizedBox(height: 10),
                  _Panel(
                    child: Row(
                      children: [
                        Expanded(
                          child: TextField(
                            controller: _hostCtrl,
                            focusNode: _hostFocus,
                            keyboardType: TextInputType.url,
                            decoration: InputDecoration(
                              labelText: 'IP ровера',
                              hintText: WifiRobotHostNotifier.defaultHost,
                              border: OutlineInputBorder(
                                borderRadius: BorderRadius.circular(8),
                              ),
                              isDense: true,
                            ),
                            onSubmitted: _saveRobotHost,
                          ),
                        ),
                        const SizedBox(width: 8),
                        _IconButton(
                          icon: Icons.save_rounded,
                          onTap: () => _saveRobotHost(_hostCtrl.text),
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 10),
                  _Panel(
                    child: Column(
                      children: [
                        Row(
                          children: [
                            Expanded(
                              child: _Metric(
                                label: 'Фикс',
                                value: _fixLabel(wifi.gpsFixType),
                                good: fixOk,
                              ),
                            ),
                            Expanded(
                              child: _Metric(
                                label: 'RTK',
                                value: _carrierLabel(wifi.gpsCarrier),
                                good: rtkOk,
                              ),
                            ),
                            Expanded(
                              child: _Metric(
                                label: 'Спутн.',
                                value: wifi.gpsSatellites?.toString() ?? '-',
                                good: (wifi.gpsSatellites ?? 0) >= 12,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 10),
                        Row(
                          children: [
                            Expanded(
                              child: _Metric(
                                label: 'Точн.',
                                value: accM == null
                                    ? '-'
                                    : '${accM.toStringAsFixed(2)} m',
                                good: accM != null && accM <= 0.03,
                              ),
                            ),
                            Expanded(
                              child: _Metric(
                                label: 'Скор.',
                                value: wifi.gpsSpeedMps == null
                                    ? '-'
                                    : '${wifi.gpsSpeedMps!.toStringAsFixed(2)} m/s',
                              ),
                            ),
                            Expanded(
                              child: _Metric(
                                label: 'PDOP',
                                value: wifi.gpsPDop?.toStringAsFixed(2) ?? '-',
                                good: wifi.gpsPDop != null &&
                                    wifi.gpsPDop! <= 2.0,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 10),
                        Row(
                          children: [
                            Expanded(
                              child: _Metric(
                                label: '2 см режим',
                                value: precisionOk ? 'готов' : 'ждем fixed',
                                good: precisionOk,
                              ),
                            ),
                            Expanded(
                              child: _Metric(
                                label: 'hAcc',
                                value: wifi.gpsAccuracy == null
                                    ? '-'
                                    : '${wifi.gpsAccuracy} mm',
                                good: wifi.gpsAccuracy != null &&
                                    wifi.gpsAccuracy! <= 30,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 10),
                        Row(
                          children: [
                            Expanded(
                              child: _Metric(
                                label: 'RTCM',
                                value: wifi.rtcmBytes == null
                                    ? '-'
                                    : '${wifi.rtcmBytes} B',
                                good: (wifi.rtcmBytes ?? 0) > 0 && rtcmFresh,
                              ),
                            ),
                            Expanded(
                              child: _Metric(
                                label: 'RTCM возраст',
                                value: _rtcmMetricValue(wifi.rtcmAgeMs),
                                good: rtcmFresh,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 12),
                        _CoordLine(
                          label: 'Lat',
                          value: wifi.gpsLat?.toStringAsFixed(8) ?? '-',
                        ),
                        _CoordLine(
                          label: 'Lon',
                          value: wifi.gpsLon?.toStringAsFixed(8) ?? '-',
                        ),
                        _CoordLine(
                          label: 'Высота',
                          value: wifi.gpsHeightM == null
                              ? '-'
                              : '${wifi.gpsHeightM!.toStringAsFixed(2)} m',
                        ),
                        _CoordLine(
                          label: 'Курс',
                          value: wifi.gpsHeading == null
                              ? '-'
                              : '${wifi.gpsHeading!.toStringAsFixed(1)} град',
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 10),
                  _Panel(
                    child: _NavigationPanel(
                      current: currentPoint,
                      origin: navigationOrigin,
                      target: _navigationTarget,
                      targetIndex: _navigationTargetIndex,
                      routeLabel: routeLabel,
                      routePoints: navigationPoints,
                      result: navigation,
                      motorPreview: motorPreview,
                      motorsEnabled: _motorsEnabled,
                      routeRunning: _routeRunning,
                      roverNavState: wifi.navState,
                      roverNavWpIndex: wifi.navWpIndex,
                      roverNavWpTotal: wifi.navWpTotal,
                      roverNavDistToWp: wifi.navDistToWp,
                      forwardPercent: _forwardPercent,
                      turnPercent: _turnPercent,
                      rawHeadingDegrees: rawNavigationHeading,
                      headingDegrees: navigationHeading,
                      headingSource: navigationHeadingSource,
                      headingOffsetDegrees: _headingCalibration.offsetDegrees,
                      invertYaw: _headingCalibration.invertYaw,
                      invertForward: _invertForward,
                      invertSteering: _invertSteering,
                      movementBearingDegrees: movementBearing,
                      movementTargetErrorDegrees: movementTargetError,
                      onToggleMotors: _toggleMotors,
                      onStartRoute: navigationPoints.isEmpty
                          ? null
                          : () => _startRoute(navigationPoints),
                      onPauseRoute: _pauseRoute,
                      onCalibrateHeading: _navigationTarget == null ||
                              wifi.imuYaw == null ||
                              navigation.bearingDegrees == null
                          ? null
                          : () => _calibrateHeadingToTarget(wifi),
                      onResetImu:
                          wifi.isConnected ? _resetImuCalibration : null,
                      onInvertYawChanged: _setInvertYaw,
                      onInvertForwardChanged: _setInvertForward,
                      onInvertSteeringChanged: _setInvertSteering,
                      onSetCurrentTarget: currentPoint == null
                          ? null
                          : () => _setNavigationTargetToCurrent(wifi),
                      onSelectRoutePoint: navigationPoints.isEmpty
                          ? null
                          : _setNavigationTargetByIndex,
                      onNextRoutePoint: navigationPoints.isEmpty
                          ? null
                          : _setNextNavigationTarget,
                      onClearTarget: _navigationTarget == null
                          ? null
                          : _clearNavigationTarget,
                      onForwardChanged: _setForwardPercent,
                      onTurnChanged: _setTurnPercent,
                    ),
                  ),
                  const SizedBox(height: 10),
                  _Panel(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        Row(
                          children: [
                            const Expanded(
                              child: Text(
                                'Карта участка',
                                style: TextStyle(
                                  fontSize: 16,
                                  fontWeight: FontWeight.w900,
                                ),
                              ),
                            ),
                            Text(
                              mapSpanM == null
                                  ? 'нет позиции'
                                  : '~${mapSpanM.toStringAsFixed(0)} м',
                              style: TextStyle(
                                color: Colors.white.withValues(alpha: 0.68),
                                fontWeight: FontWeight.w800,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 10),
                        _MapDebugInfo(
                          currentPoints: _points.length,
                          openedPoints: openedPoints.length,
                          trackPoints: _trail.length,
                          currentPosition: currentPoint,
                        ),
                        const SizedBox(height: 10),
                        SizedBox(
                          height: 300,
                          child: ClipRRect(
                            borderRadius: BorderRadius.circular(8),
                            child: CustomPaint(
                              painter: _SiteMapPainter(
                                points: visibleMapPoints,
                                current: currentPoint,
                                target: _navigationTarget,
                                targetIndex: _navigationTargetIndex,
                              ),
                              child: const SizedBox.expand(),
                            ),
                          ),
                        ),
                        const SizedBox(height: 10),
                        Wrap(
                          spacing: 8,
                          runSpacing: 8,
                          children: [
                            _Action(
                              icon: Icons.my_location_rounded,
                              label: 'Я',
                              color: const Color(0xFF7AA2FF),
                              onTap: currentPoint == null
                                  ? null
                                  : () => _appendTrail(wifi, force: true),
                            ),
                            _Action(
                              icon: Icons.delete_sweep_rounded,
                              label: 'Стереть след',
                              onTap: _trail.isEmpty
                                  ? null
                                  : () => setState(() => _trail.clear()),
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 10),
                  _Panel(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        Row(
                          children: [
                            const Expanded(
                              child: Text(
                                'Запись периметра',
                                style: TextStyle(
                                  fontSize: 16,
                                  fontWeight: FontWeight.w900,
                                ),
                              ),
                            ),
                            Text(
                              '${_points.length} точек',
                              style: TextStyle(
                                color: Colors.white.withValues(alpha: 0.70),
                                fontWeight: FontWeight.w800,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 10),
                        SizedBox(
                          height: 230,
                          child: ClipRRect(
                            borderRadius: BorderRadius.circular(8),
                            child: CustomPaint(
                              painter: _PerimeterPainter(_points),
                              child: const SizedBox.expand(),
                            ),
                          ),
                        ),
                        const SizedBox(height: 10),
                        TextField(
                          controller: _nameCtrl,
                          decoration: InputDecoration(
                            labelText: 'Имя записи',
                            border: OutlineInputBorder(
                              borderRadius: BorderRadius.circular(8),
                            ),
                            isDense: true,
                          ),
                        ),
                        const SizedBox(height: 10),
                        Wrap(
                          spacing: 8,
                          runSpacing: 8,
                          children: [
                            _Action(
                              icon: _recording
                                  ? Icons.pause_rounded
                                  : Icons.fiber_manual_record_rounded,
                              label: _recording ? 'Пауза' : 'Запись',
                              color: _recording
                                  ? const Color(0xFFFFD166)
                                  : const Color(0xFFFF4D6D),
                              onTap: () => _toggleRecording(wifi),
                            ),
                            _Action(
                              icon: Icons.add_location_alt_rounded,
                              label: 'Точка',
                              onTap: () => _addCurrentPoint(wifi, force: true),
                            ),
                            _Action(
                              icon: Icons.undo_rounded,
                              label: 'Назад',
                              onTap: _points.isEmpty
                                  ? null
                                  : () => setState(() => _points.removeLast()),
                            ),
                            _Action(
                              icon: Icons.save_rounded,
                              label: 'Сохранить',
                              onTap: _save,
                            ),
                            _Action(
                              icon: Icons.copy_rounded,
                              label: 'JSON',
                              onTap: _points.isEmpty ? null : _copyJson,
                            ),
                            _Action(
                              icon: Icons.delete_outline_rounded,
                              label: 'Очистить',
                              onTap: _points.isEmpty ? null : _clearPoints,
                            ),
                          ],
                        ),
                        const SizedBox(height: 8),
                        Text(
                          'Кнопка "Точка" добавляет вручную. Автозапись добавляет точку каждые ${_autoPointMinDistanceM.toStringAsFixed(1)} м, когда координаты идут с ровера.',
                          style: TextStyle(
                            color: Colors.white.withValues(alpha: 0.58),
                            fontSize: 12,
                            fontWeight: FontWeight.w700,
                          ),
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 10),
                  _Panel(
                    child: FutureBuilder<List<GpsPerimeter>>(
                      future: _savedFuture,
                      builder: (context, snapshot) {
                        final saved = snapshot.data ?? const <GpsPerimeter>[];
                        return Column(
                          crossAxisAlignment: CrossAxisAlignment.stretch,
                          children: [
                            const Text(
                              'Сохраненные периметры',
                              style: TextStyle(
                                fontSize: 16,
                                fontWeight: FontWeight.w900,
                              ),
                            ),
                            const SizedBox(height: 8),
                            if (saved.isEmpty)
                              Text(
                                'Сохраненных GPS-периметров пока нет.',
                                style: TextStyle(
                                  color: Colors.white.withValues(alpha: 0.58),
                                  fontWeight: FontWeight.w700,
                                ),
                              )
                            else
                              ...saved.map(
                                (p) => _SavedPerimeterCard(
                                  perimeter: p,
                                  selected: _activeSaved?.id == p.id,
                                  onOpen: _openSaved,
                                  onCopyJson: _copySavedJson,
                                  onDelete: _deleteSaved,
                                ),
                              ),
                          ],
                        );
                      },
                    ),
                  ),
                  const SizedBox(height: 10),
                  _Panel(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        const Text(
                          'Журнал приема',
                          style: TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.w900,
                          ),
                        ),
                        const SizedBox(height: 8),
                        SizedBox(
                          height: 140,
                          child: ListView(
                            reverse: true,
                            children: wifi.rxLog.reversed
                                .take(40)
                                .map(
                                  (line) => Text(
                                    line,
                                    style: const TextStyle(
                                      fontFamily: 'monospace',
                                      fontSize: 11,
                                    ),
                                  ),
                                )
                                .toList(),
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  GpsPerimeterPoint? _currentPoint(WifiConnectionState wifi) {
    final lat = wifi.gpsLat;
    final lon = wifi.gpsLon;
    if (lat == null || lon == null) return null;
    if (lat.abs() < 0.000001 && lon.abs() < 0.000001) return null;
    return GpsPerimeterPoint(
      lat: lat,
      lon: lon,
      hAccM: wifi.gpsAccuracy == null ? null : wifi.gpsAccuracy! / 1000.0,
      at: DateTime.now(),
    );
  }

  void _appendTrail(WifiConnectionState wifi, {bool force = false}) {
    if (!_hasUsableGps(wifi)) return;
    final point = _currentPoint(wifi);
    if (point == null) return;

    if (!force && _trail.isNotEmpty) {
      final last = _trail.last;
      final dist = _distanceM(last.lat, last.lon, point.lat, point.lon);
      if (dist < _trailMinDistanceM) return;
    }

    if (!mounted) return;
    setState(() {
      _trail.add(point);
      if (_trail.length > _trailMaxPoints) {
        _trail.removeRange(0, _trail.length - _trailMaxPoints);
      }
    });
  }

  double? _mapSpanM(GpsPerimeterPoint? current) {
    final all = <GpsPerimeterPoint>[..._trail, ..._points];
    if (current != null) all.add(current);
    if (all.isEmpty) return null;
    if (all.length == 1) return 10.0;

    final lat0 = all.map((p) => p.lat).reduce((a, b) => a + b) / all.length;
    final lon0 = all.map((p) => p.lon).reduce((a, b) => a + b) / all.length;
    const latScale = 111320.0;
    final lonScale = 111320.0 * math.cos(lat0 * math.pi / 180.0);

    double minX = double.infinity;
    double maxX = -double.infinity;
    double minY = double.infinity;
    double maxY = -double.infinity;
    for (final p in all) {
      final x = (p.lon - lon0) * lonScale;
      final y = (p.lat - lat0) * latScale;
      minX = math.min(minX, x);
      maxX = math.max(maxX, x);
      minY = math.min(minY, y);
      maxY = math.max(maxY, y);
    }
    return math.max(10.0, math.max(maxX - minX, maxY - minY));
  }

  static GpsPerimeterPoint? _firstPoint(List<GpsPerimeterPoint> points) {
    return points.isEmpty ? null : points.first;
  }

  void _setNavigationTargetToCurrent(WifiConnectionState wifi) {
    final point = _currentPoint(wifi);
    if (point == null) {
      setState(() => _notice = 'Нет текущей GPS-позиции для цели.');
      return;
    }
    setState(() {
      _navigationOrigin ??= point;
      _navigationTarget = point;
      _navigationTargetIndex = null;
      _roverRouteStartIndex = 0;
      _notice = 'Цель навигации: текущая позиция.';
    });
    _maybeLogNavigation(wifi, force: true);
  }

  void _setNavigationTargetByIndex(int index) {
    final routePoints =
        _points.isNotEmpty ? _points : (_activeSaved?.points ?? const []);
    if (index < 0 || index >= routePoints.length) return;
    final current = _currentPoint(ref.read(wifiConnectionProvider));
    setState(() {
      _navigationOrigin ??= current ?? routePoints.first;
      _navigationTarget = routePoints[index];
      _navigationTargetIndex = index;
      _roverRouteStartIndex = index;
      _notice = 'Цель навигации: точка ${index + 1}.';
    });
    _maybeLogNavigation(ref.read(wifiConnectionProvider), force: true);
  }

  void _setNextNavigationTarget() {
    final routePoints =
        _points.isNotEmpty ? _points : (_activeSaved?.points ?? const []);
    if (routePoints.isEmpty) return;
    final nextIndex = _navigationTargetIndex == null
        ? 0
        : (_navigationTargetIndex! + 1) % routePoints.length;
    _setNavigationTargetByIndex(nextIndex);
  }

  double? _movementBearing() {
    if (_trail.length < 2) return null;
    for (var i = _trail.length - 2; i >= 0; i--) {
      final from = _trail[i];
      final to = _trail.last;
      final distance =
          GpsLocalGeometry.distanceMeters(from.lat, from.lon, to.lat, to.lon);
      if (distance >= 0.35) {
        return GpsLocalGeometry.bearingDegrees(
          from.lat,
          from.lon,
          to.lat,
          to.lon,
        );
      }
    }
    return null;
  }

  void _setInvertYaw(bool value) {
    final wifi = ref.read(wifiConnectionProvider);
    final targetBearing = _targetBearing(wifi);
    setState(() {
      _headingCalibration = HeadingCalibration(
        offsetDegrees: _headingCalibration.offsetDegrees,
        invertYaw: value,
      );
      if (wifi.imuYaw != null && targetBearing != null) {
        _headingCalibration = _headingCalibration.alignRawToTarget(
          rawDegrees: wifi.imuYaw!,
          targetDegrees: targetBearing,
        );
        _headingCalibrated = true;
      } else {
        _headingCalibrated = false;
      }
      _notice = 'IMU invert: ${value ? 'ON' : 'OFF'}.';
    });
    _saveNavigationSettings();
    _sendRoverNavConfig();
  }

  Future<void> _loadNavigationSettings() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      if (!mounted) return;
      setState(() {
        _headingCalibration = HeadingCalibration(
          offsetDegrees: prefs.getDouble(_prefsHeadingOffsetKey) ?? 0,
          invertYaw: prefs.getBool(_prefsInvertYawKey) ?? false,
        );
        _headingCalibrated = prefs.getBool(_prefsHeadingCalibratedKey) ?? false;
        _invertForward = prefs.getBool(_prefsInvertForwardKey) ?? false;
        _invertSteering = prefs.getBool(_prefsInvertSteeringKey) ?? false;
        _forwardPercent =
            (prefs.getInt(_prefsForwardPercentKey) ?? 22).clamp(8, 35).toInt();
        _turnPercent =
            (prefs.getInt(_prefsTurnPercentKey) ?? 18).clamp(8, 35).toInt();
      });
    } catch (e) {
      if (!mounted) return;
      setState(() => _notice = 'Не удалось загрузить настройки навигации: $e');
    }
  }

  Future<void> _saveNavigationSettings() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setDouble(
        _prefsHeadingOffsetKey,
        _headingCalibration.offsetDegrees,
      );
      await prefs.setBool(_prefsHeadingCalibratedKey, _headingCalibrated);
      await prefs.setBool(_prefsInvertYawKey, _headingCalibration.invertYaw);
      await prefs.setBool(_prefsInvertForwardKey, _invertForward);
      await prefs.setBool(_prefsInvertSteeringKey, _invertSteering);
      await prefs.setInt(_prefsForwardPercentKey, _forwardPercent);
      await prefs.setInt(_prefsTurnPercentKey, _turnPercent);
    } catch (e) {
      if (!mounted) return;
      setState(() => _notice = 'Не удалось сохранить настройки навигации: $e');
    }
  }

  void _setInvertForward(bool value) {
    setState(() {
      _invertForward = value;
      _notice = 'Forward invert: ${value ? 'ON' : 'OFF'}.';
    });
    _saveNavigationSettings();
    _sendRoverNavConfig();
  }

  void _setInvertSteering(bool value) {
    setState(() {
      _invertSteering = value;
      _notice = 'Steering invert: ${value ? 'ON' : 'OFF'}.';
    });
    _saveNavigationSettings();
    _sendRoverNavConfig();
  }

  void _setForwardPercent(double value) {
    setState(() => _forwardPercent = value.round());
    _saveNavigationSettings();
    _sendRoverNavConfig();
  }

  void _setTurnPercent(double value) {
    setState(() => _turnPercent = value.round());
    _saveNavigationSettings();
    _sendRoverNavConfig();
  }

  double? _targetBearing(WifiConnectionState wifi) {
    final target = _navigationTarget;
    final lat = wifi.gpsLat;
    final lon = wifi.gpsLon;
    if (target == null || lat == null || lon == null) return null;
    return GpsLocalGeometry.bearingDegrees(lat, lon, target.lat, target.lon);
  }

  void _calibrateHeadingToTarget(WifiConnectionState wifi) {
    final rawYaw = wifi.imuYaw;
    final targetBearing = _targetBearing(wifi);
    if (rawYaw == null || targetBearing == null) {
      setState(() => _notice = 'Нет IMU yaw или цели для калибровки.');
      return;
    }
    setState(() {
      _headingCalibration = _headingCalibration.alignRawToTarget(
        rawDegrees: rawYaw,
        targetDegrees: targetBearing,
      );
      _headingCalibrated = true;
      _notice =
          'IMU offset: ${_headingCalibration.offsetDegrees.toStringAsFixed(1)} deg. Нос робота должен смотреть на цель.';
    });
    ref.read(wifiConnectionProvider.notifier).addLocalLog(
          'NAV IMU calibrate raw=${rawYaw.toStringAsFixed(1)} target=${targetBearing.toStringAsFixed(1)} offset=${_headingCalibration.offsetDegrees.toStringAsFixed(1)} invert=${_headingCalibration.invertYaw ? 1 : 0}',
        );
    _saveNavigationSettings();
    _sendRoverNavConfig();
  }

  static bool _hasFreshImu(WifiConnectionState wifi) {
    final receivedAt = wifi.imuReceivedAt;
    final roverFresh = wifi.imuFresh ?? true;
    final roverAge = wifi.imuAgeMs;
    return wifi.imuYaw != null &&
        roverFresh &&
        (roverAge == null || roverAge < 5000) &&
        receivedAt != null &&
        DateTime.now().difference(receivedAt).inMilliseconds < 6000;
  }

  double? _navigationHeadingFor(WifiConnectionState wifi) {
    double? heading;
    if (_hasFreshImu(wifi)) {
      heading = _headingCalibrated
          ? _headingCalibration.apply(wifi.imuYaw!)
          : wifi.imuYaw;
    }
    final speed = wifi.gpsSpeedMps;
    if (heading == null &&
        wifi.gpsHeading != null &&
        speed != null &&
        speed >= 0.35) {
      heading = wifi.gpsHeading;
    }
    if (heading != null) {
      heading = _smoothHeading(heading);
      _lastStableHeadingDegrees = heading;
      _lastStableHeadingAt = DateTime.now();
      return heading;
    }
    final last = _lastStableHeadingAt;
    if (last != null &&
        DateTime.now().difference(last) < const Duration(seconds: 8)) {
      return _lastStableHeadingDegrees;
    }
    return null;
  }

  double _smoothHeading(double rawDegrees) {
    final now = DateTime.now();
    final previous = _smoothedHeadingDegrees;
    final previousAt = _lastHeadingSampleAt;
    _lastHeadingSampleAt = now;
    if (previous == null ||
        previousAt == null ||
        now.difference(previousAt) > const Duration(seconds: 3)) {
      _smoothedHeadingDegrees = rawDegrees;
      return rawDegrees;
    }

    final error = GpsLocalGeometry.headingErrorDegrees(previous, rawDegrees);
    final gain = error.abs() > 45 ? 0.30 : 0.45;
    final next = GpsLocalGeometry.normalizeDegrees(previous + error * gain);
    _smoothedHeadingDegrees = next;
    return next;
  }

  String _navigationHeadingSourceFor(WifiConnectionState wifi) {
    if (_headingCalibrated && _hasFreshImu(wifi)) return 'BNO085 + offset';
    if (!_headingCalibrated && _hasFreshImu(wifi)) return 'BNO085 raw';
    final speed = wifi.gpsSpeedMps;
    if (wifi.gpsHeading != null && speed != null && speed >= 0.35) {
      return 'GPS fallback';
    }
    final last = _lastStableHeadingAt;
    if (last != null &&
        DateTime.now().difference(last) < const Duration(seconds: 8)) {
      return 'heading hold';
    }
    return 'нет свежего курса';
  }

  void _resetImuCalibration() {
    ref.read(wifiConnectionProvider.notifier).sendRaw('IMU_RESET');
    setState(() {
      _headingCalibrated = false;
      _notice = 'IMU reset requested. Повтори калибровку носом на цель.';
    });
    _saveNavigationSettings();
    _sendRoverNavConfig();
  }

  void _startRoute(List<GpsPerimeterPoint> routePoints) {
    if (routePoints.isEmpty) {
      setState(() => _notice = 'Нет точек маршрута.');
      return;
    }
    final current = _currentPoint(ref.read(wifiConnectionProvider));
    setState(() {
      _navigationOrigin = current ?? routePoints.first;
      _navigationTarget = routePoints.first;
      _navigationTargetIndex = 0;
      _roverRouteStartIndex = 0;
      _routeRunning = true;
      _lastLoggedNavigationCommand = null;
      _notice = 'Маршрут выбран: точка 1/${routePoints.length}.';
    });
    _maybeLogNavigation(ref.read(wifiConnectionProvider), force: true);
  }

  void _sendRoverNavConfig() {
    ref.read(wifiConnectionProvider.notifier).sendNavConfig(
          forwardPercent: _forwardPercent,
          turnPercent: _turnPercent,
          invertForward: _invertForward,
          invertSteering: _invertSteering,
          headingOffsetDegrees:
              _headingCalibrated ? _headingCalibration.offsetDegrees : 0,
          invertYaw: _headingCalibrated && _headingCalibration.invertYaw,
        );
  }

  List<GpsPerimeterPoint> _roverRouteForCurrentTarget() {
    final target = _navigationTarget;
    final routePoints = _currentRoutePoints();
    if (target == null) return const [];
    if (_navigationTargetIndex == null || routePoints.isEmpty) {
      _roverRouteStartIndex = 0;
      return <GpsPerimeterPoint>[target];
    }

    final start =
        _navigationTargetIndex!.clamp(0, routePoints.length - 1).toInt();
    _roverRouteStartIndex = start;
    return routePoints.skip(start).take(_roverMaxWaypoints).toList();
  }

  void _startRoverNavigation(List<GpsPerimeterPoint> routePoints) {
    final notifier = ref.read(wifiConnectionProvider.notifier);
    final origin = routePoints.first;
    final projection = GpsProjection(refLat: origin.lat, refLon: origin.lon);
    notifier.sendNavStop();
    _sendRoverNavConfig();
    notifier.sendRouteBegin(
      routePoints.length,
      originLat: origin.lat,
      originLon: origin.lon,
    );
    for (var i = 0; i < routePoints.length; i++) {
      final point = routePoints[i];
      final local = projection.toLocal(point.lat, point.lon);
      notifier.sendRouteWaypoint(i, local.dx, local.dy);
    }
    notifier.sendRouteEnd();
    notifier.sendNavStart();
    notifier.addLocalLog('ROVER NAV uploaded ${routePoints.length} points');
  }

  void _syncRoverNavigation(WifiConnectionState wifi) {
    if (!_motorsEnabled) return;
    final navState = wifi.navState;
    var changed = false;
    String? notice;

    if (navState != null && navState != _lastRoverNavState) {
      _lastRoverNavState = navState;
      notice = 'Rover NAV: $navState';
      changed = true;
    }

    final routePoints = _currentRoutePoints();
    final roverIndex = wifi.navWpIndex;
    if (roverIndex != null && routePoints.isNotEmpty) {
      final appIndex = (_roverRouteStartIndex + roverIndex)
          .clamp(0, routePoints.length - 1)
          .toInt();
      if (_navigationTargetIndex != appIndex ||
          _navigationTarget != routePoints[appIndex]) {
        _navigationTargetIndex = appIndex;
        _navigationTarget = routePoints[appIndex];
        changed = true;
      }
    }

    if (navState == 'DONE' || navState == 'ERROR') {
      _motorsEnabled = false;
      _routeRunning = false;
      changed = true;
    }

    if (!changed || !mounted) return;
    setState(() {
      if (notice != null) _notice = notice;
    });
  }

  void _pauseRoute() {
    _sendStopIfConnected();
    _motorDriveTimer?.cancel();
    _motorDriveTimer = null;
    setState(() {
      _motorsEnabled = false;
      _routeRunning = false;
      _notice = 'Маршрут остановлен, моторы выключены.';
    });
  }

  void _toggleMotors() {
    if (_motorsEnabled) {
      _sendStopIfConnected();
      _motorDriveTimer?.cancel();
      _motorDriveTimer = null;
      setState(() {
        _motorsEnabled = false;
        _notice = 'Моторы выключены.';
      });
      return;
    }

    final wifi = ref.read(wifiConnectionProvider);
    if (!wifi.isConnected) {
      setState(() => _notice = 'Нет WebSocket-связи с роботом.');
      return;
    }
    if (_navigationTarget == null) {
      final routePoints = _currentRoutePoints();
      if (routePoints.isEmpty) {
        setState(() => _notice = 'Сначала выбери цель или маршрут.');
        return;
      }
      _startRoute(routePoints);
    }

    final roverRoute = _roverRouteForCurrentTarget();
    if (roverRoute.isEmpty) {
      setState(() => _notice = 'No target for rover NAV.');
      return;
    }
    _startRoverNavigation(roverRoute);
    _motorDriveTimer?.cancel();
    _motorDriveTimer = null;

    setState(() {
      _motorsEnabled = true;
      _routeRunning = _navigationTargetIndex != null || _routeRunning;
      _lastRoverNavState = null;
      _notice =
          'Rover NAV started locally: ${roverRoute.length} point(s). App motor loop is off.';
    });
  }

  void _sendStopIfConnected() {
    if (ref.read(wifiConnectionProvider).isConnected) {
      final notifier = ref.read(wifiConnectionProvider.notifier);
      notifier.sendNavStop();
      notifier.sendStop();
    }
  }

  List<GpsPerimeterPoint> _currentRoutePoints() {
    return _points.isNotEmpty ? _points : (_activeSaved?.points ?? const []);
  }

  void _handleAutoRouteAdvance(WifiConnectionState wifi) {
    if (_motorsEnabled) return;
    if (!_routeRunning || _navigationTarget == null) return;
    final origin =
        _navigationOrigin ?? _currentPoint(wifi) ?? _navigationTarget!;
    final result = const GpsNavigationController().evaluate(
      currentLat: wifi.gpsLat,
      currentLon: wifi.gpsLon,
      targetLat: _navigationTarget!.lat,
      targetLon: _navigationTarget!.lon,
      headingDegrees: _navigationHeadingFor(wifi),
      rtkFixed: _rtkOkForNavigation(wifi),
      rtcmAgeMs: wifi.rtcmAgeMs,
      hAccMm: wifi.gpsAccuracy,
      originLat: origin.lat,
      originLon: origin.lon,
      gpsFixType: wifi.gpsFixType,
      gpsAgeMs: wifi.gpsAgeMs,
      gpsReceivedAt: wifi.gpsReceivedAt,
    );
    if (result.command == NavigationCommand.arrived) {
      _advanceRouteAfterArrival();
    }
  }

  bool _advanceRouteAfterArrival() {
    if (!_routeRunning || _navigationTargetIndex == null) return false;
    final routePoints = _currentRoutePoints();
    final currentIndex = _navigationTargetIndex!;
    final nextIndex = currentIndex + 1;
    if (nextIndex >= routePoints.length) {
      _motorDriveTimer?.cancel();
      _motorDriveTimer = null;
      if (mounted) {
        setState(() {
          _routeRunning = false;
          _motorsEnabled = false;
          _navigationTargetIndex = null;
          _roverRouteStartIndex = 0;
          _notice = 'Маршрут завершен, моторы остановлены.';
        });
      }
      ref.read(wifiConnectionProvider.notifier).addLocalLog(
            'NAV route complete points=${routePoints.length}',
          );
      return true;
    }

    if (mounted) {
      setState(() {
        _navigationTarget = routePoints[nextIndex];
        _navigationTargetIndex = nextIndex;
        _roverRouteStartIndex = nextIndex;
        _lastLoggedNavigationCommand = null;
        _notice =
            'Следующая цель: точка ${nextIndex + 1}/${routePoints.length}.';
      });
    }
    ref.read(wifiConnectionProvider.notifier).addLocalLog(
          'NAV next target ${nextIndex + 1}/${routePoints.length}',
        );
    return true;
  }

  void _clearNavigationTarget() {
    _sendStopIfConnected();
    _motorDriveTimer?.cancel();
    _motorDriveTimer = null;
    setState(() {
      _motorsEnabled = false;
      _routeRunning = false;
      _navigationTarget = null;
      _navigationTargetIndex = null;
      _roverRouteStartIndex = 0;
      _lastLoggedNavigationCommand = null;
      _notice = 'Цель навигации очищена.';
    });
  }

  void _maybeLogNavigation(WifiConnectionState wifi, {bool force = false}) {
    final target = _navigationTarget;
    if (target == null) return;
    final current = _currentPoint(wifi);
    final origin = _navigationOrigin ?? current ?? target;
    final heading = _navigationHeadingFor(wifi);
    final result = const GpsNavigationController().evaluate(
      currentLat: wifi.gpsLat,
      currentLon: wifi.gpsLon,
      targetLat: target.lat,
      targetLon: target.lon,
      headingDegrees: heading,
      rtkFixed: _rtkOkForNavigation(wifi),
      rtcmAgeMs: wifi.rtcmAgeMs,
      hAccMm: wifi.gpsAccuracy,
      originLat: origin.lat,
      originLon: origin.lon,
      gpsFixType: wifi.gpsFixType,
      gpsAgeMs: wifi.gpsAgeMs,
      gpsReceivedAt: wifi.gpsReceivedAt,
    );

    final now = DateTime.now();
    final changed = result.command != _lastLoggedNavigationCommand;
    final last = _lastNavigationLogAt;
    final due =
        last == null || now.difference(last) >= const Duration(seconds: 2);
    if (!force && !changed && !due) return;

    _lastLoggedNavigationCommand = result.command;
    _lastNavigationLogAt = now;
    final distance = result.distanceMeters?.toStringAsFixed(2) ?? '-';
    final error = result.headingErrorDegrees?.toStringAsFixed(1) ?? '-';
    ref.read(wifiConnectionProvider.notifier).addLocalLog(
          'NAV dry ${result.command.wireName} dist=${distance}m err=${error}deg ${result.reason}',
        );
  }

  void _toggleRecording(WifiConnectionState wifi) {
    if (_recording) {
      _recordTimer?.cancel();
      _recordTimer = null;
      setState(() => _recording = false);
      return;
    }

    if (!_precisionOk(wifi)) {
      setState(() => _notice = _precisionBlockedReason(wifi));
      return;
    }

    _addCurrentPoint(wifi, force: true);
    _recordTimer = Timer.periodic(const Duration(milliseconds: 700), (_) {
      _addCurrentPoint(ref.read(wifiConnectionProvider));
    });
    setState(() => _recording = true);
  }

  Future<void> _saveRobotHost(String value) async {
    await ref.read(wifiRobotHostProvider.notifier).setHost(value);
    if (mounted) {
      final saved = ref.read(wifiRobotHostProvider);
      _hostCtrl.text = saved;
      FocusScope.of(context).unfocus();
      setState(() => _notice = 'IP ровера сохранен: $saved');
    }
  }

  void _addCurrentPoint(WifiConnectionState wifi, {bool force = false}) {
    final lat = wifi.gpsLat;
    final lon = wifi.gpsLon;
    if (!_precisionOk(wifi) || lat == null || lon == null) {
      setState(() => _notice = _precisionBlockedReason(wifi));
      return;
    }

    final point = GpsPerimeterPoint(
      lat: lat,
      lon: lon,
      hAccM: wifi.gpsAccuracy == null ? null : wifi.gpsAccuracy! / 1000.0,
      at: DateTime.now(),
    );

    if (!force && _points.isNotEmpty) {
      final last = _points.last;
      final dist = _distanceM(last.lat, last.lon, point.lat, point.lon);
      if (dist < _autoPointMinDistanceM) return;
    }

    setState(() {
      _points.add(point);
      _activeSaved = null;
      _notice = 'Точка добавлена: ${_points.length}.';
    });
  }

  Future<void> _save() async {
    if (_points.length < 3) {
      const message = 'Нужно минимум 3 точки, чтобы сохранить периметр.';
      setState(() => _notice = message);
      _showSnack(message);
      return;
    }

    try {
      final saved = await GpsPerimeterStorage.save(_nameCtrl.text, _points);
      if (!mounted) return;
      setState(() {
        _activeSaved = saved;
        _savedFuture = GpsPerimeterStorage.list();
        _notice = 'Периметр сохранён: ${saved.points.length} точек';
      });
      _showSnack('Периметр сохранён: ${saved.points.length} точек');
    } catch (error) {
      if (!mounted) return;
      final message = 'Ошибка сохранения периметра: $error';
      setState(() => _notice = message);
      _showSnack(message);
    }
  }

  Future<void> _copyJson() async {
    await Clipboard.setData(
      ClipboardData(text: GpsPerimeterStorage.toExportJson(_points)),
    );
    setState(() => _notice = 'JSON периметра скопирован.');
  }

  void _clearPoints() {
    _recordTimer?.cancel();
    _motorDriveTimer?.cancel();
    _recordTimer = null;
    _motorDriveTimer = null;
    _sendStopIfConnected();
    setState(() {
      _recording = false;
      _motorsEnabled = false;
      _routeRunning = false;
      _points.clear();
      _activeSaved = null;
      _navigationTarget = null;
      _navigationTargetIndex = null;
      _roverRouteStartIndex = 0;
      _notice = 'Текущий периметр очищен.';
    });
  }

  Future<void> _openSaved(GpsPerimeter perimeter) async {
    try {
      final loaded = await GpsPerimeterStorage.load(perimeter.id);
      if (!mounted) return;
      final selected = loaded ?? perimeter;
      setState(() {
        _points
          ..clear()
          ..addAll(selected.points);
        _activeSaved = selected;
        _nameCtrl.text = selected.name;
        _notice = 'Открыт периметр: ${selected.points.length} точек.';
      });
      _showSnack('Открыт периметр: ${selected.points.length} точек');
    } catch (error) {
      if (!mounted) return;
      final message = 'Ошибка открытия периметра: $error';
      setState(() => _notice = message);
      _showSnack(message);
    }
  }

  Future<void> _copySavedJson(GpsPerimeter perimeter) async {
    try {
      final loaded = await GpsPerimeterStorage.load(perimeter.id);
      final selected = loaded ?? perimeter;
      await Clipboard.setData(
        ClipboardData(
            text: GpsPerimeterStorage.perimeterToExportJson(selected)),
      );
      if (!mounted) return;
      setState(() => _notice = 'JSON сохраненного периметра скопирован.');
      _showSnack('JSON сохраненного периметра скопирован');
    } catch (error) {
      if (!mounted) return;
      final message = 'Ошибка экспорта JSON: $error';
      setState(() => _notice = message);
      _showSnack(message);
    }
  }

  Future<void> _deleteSaved(GpsPerimeter perimeter) async {
    try {
      await GpsPerimeterStorage.delete(perimeter.id);
      if (!mounted) return;
      setState(() {
        if (_activeSaved?.id == perimeter.id) {
          _activeSaved = null;
        }
        _savedFuture = GpsPerimeterStorage.list();
        _notice = 'Периметр удален: ${perimeter.name}.';
      });
      _showSnack('Периметр удален');
    } catch (error) {
      if (!mounted) return;
      final message = 'Ошибка удаления периметра: $error';
      setState(() => _notice = message);
      _showSnack(message);
    }
  }

  void _showSnack(String message) {
    if (!mounted) return;
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(SnackBar(content: Text(message)));
  }

  static bool _fixOk(WifiConnectionState wifi) {
    return (wifi.gpsFixType ?? 0) >= 3 &&
        wifi.gpsLat != null &&
        wifi.gpsLon != null;
  }

  static bool _rtkOk(WifiConnectionState wifi) {
    final accurateDiff =
        (wifi.gpsDiff ?? false) && (wifi.gpsAccuracy ?? 999999) <= 50;
    return (wifi.gpsCarrier == 'fixed' || accurateDiff) &&
        _rtcmFresh(wifi.rtcmAgeMs);
  }

  static bool _rtkOkForNavigation(WifiConnectionState wifi) {
    final hAcc = wifi.gpsAccuracy ?? 999999;
    final accurateDiff = (wifi.gpsDiff ?? false) && hAcc <= 300;
    final rtcmAge = wifi.rtcmAgeMs;
    final rtcmUsable =
        rtcmAge != null && rtcmAge <= GpsNavigationController.maxRtcmHoldAgeMs;
    return wifi.gpsCarrier == 'fixed' ||
        wifi.gpsCarrier == 'float' ||
        accurateDiff ||
        rtcmUsable ||
        hAcc <= GpsNavigationController.maxDegradedAccuracyMm;
  }

  static bool _precisionOk(WifiConnectionState wifi) {
    return _hasUsableGps(wifi) &&
        _rtkOk(wifi) &&
        wifi.gpsAccuracy != null &&
        wifi.gpsAccuracy! <= 30;
  }

  static bool _rtcmFresh(int? ageMs) {
    return ageMs != null && ageMs <= GpsNavigationController.maxRtcmAgeMs;
  }

  static String _rtcmStatusText(int? ageMs) {
    if (ageMs == null) return 'RTCM нет';
    if (ageMs > 10000) return 'RTCM потерян';
    if (ageMs > GpsNavigationController.maxRtcmAgeMs) return 'RTCM stale';
    return 'RTCM свежий';
  }

  static String _rtcmMetricValue(int? ageMs) {
    if (ageMs == null) return 'нет данных';
    if (ageMs > 10000) return '$ageMs ms\nпотерян';
    if (ageMs > GpsNavigationController.maxRtcmAgeMs) {
      return '$ageMs ms\nstale';
    }
    return '$ageMs ms\nсвежий';
  }

  static bool _hasUsableGps(WifiConnectionState wifi) {
    final lat = wifi.gpsLat;
    final lon = wifi.gpsLon;
    if (lat == null || lon == null) return false;
    if (lat.abs() < 0.000001 && lon.abs() < 0.000001) return false;

    final receivedAt = wifi.gpsReceivedAt;
    if (receivedAt == null) return true;
    return DateTime.now().difference(receivedAt) <
        GpsNavigationController.maxGpsAge;
  }

  static String _gpsBlockedReason(WifiConnectionState wifi) {
    if (!wifi.isConnected) return 'Нет связи с ровером.';
    if (wifi.gpsLat == null || wifi.gpsLon == null) {
      return 'Нет координат от ровера.';
    }
    final receivedAt = wifi.gpsReceivedAt;
    if (receivedAt != null &&
        DateTime.now().difference(receivedAt) >=
            GpsNavigationController.maxGpsAge) {
      return 'GPS-данные устарели, подожди новое сообщение от ровера.';
    }
    return 'Координаты пока не готовы.';
  }

  static String _precisionBlockedReason(WifiConnectionState wifi) {
    final usableReason = _gpsBlockedReason(wifi);
    if (!_hasUsableGps(wifi)) return usableReason;
    if (wifi.gpsCarrier != 'fixed') {
      return 'Для 2 см нужна RTK FIXED. Сейчас: ${_carrierLabel(wifi.gpsCarrier)}.';
    }
    if (!_rtcmFresh(wifi.rtcmAgeMs)) {
      return 'RTCM не свежий: ${_rtcmStatusText(wifi.rtcmAgeMs)}.';
    }
    final hAcc = wifi.gpsAccuracy;
    if (hAcc == null) return 'Нет hAcc от ровера.';
    if (hAcc > 30) {
      return 'Точность пока $hAcc мм. Для записи нужно 30 мм или лучше.';
    }
    return 'Ждем точный RTK FIXED.';
  }

  static String _fixLabel(int? fixType) {
    switch (fixType) {
      case 0:
        return 'нет';
      case 2:
        return '2D';
      case 3:
        return '3D';
      case 4:
        return 'GNSS+DR';
      case 5:
        return 'время';
      default:
        return fixType?.toString() ?? '-';
    }
  }

  static String _carrierLabel(String? carrier) {
    switch (carrier) {
      case 'fixed':
        return 'fixed';
      case 'float':
        return 'float';
      case 'none':
        return 'нет';
      default:
        return carrier ?? 'нет';
    }
  }

  static double _distanceM(
    double lat1,
    double lon1,
    double lat2,
    double lon2,
  ) {
    const r = 6371000.0;
    final p1 = lat1 * math.pi / 180.0;
    final p2 = lat2 * math.pi / 180.0;
    final dp = (lat2 - lat1) * math.pi / 180.0;
    final dl = (lon2 - lon1) * math.pi / 180.0;
    final a = math.sin(dp / 2) * math.sin(dp / 2) +
        math.cos(p1) * math.cos(p2) * math.sin(dl / 2) * math.sin(dl / 2);
    return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));
  }

  static String _two(int v) => v.toString().padLeft(2, '0');
}

class _Panel extends StatelessWidget {
  final Widget child;

  const _Panel({required this.child});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.055),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.white.withValues(alpha: 0.10)),
      ),
      child: child,
    );
  }
}

class _StatusStrip extends StatelessWidget {
  final bool connected;
  final bool connecting;
  final bool fixOk;
  final bool rtkOk;
  final String? carrier;
  final String rtcmText;

  const _StatusStrip({
    required this.connected,
    required this.connecting,
    required this.fixOk,
    required this.rtkOk,
    required this.carrier,
    required this.rtcmText,
  });

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          child: _Pill(
            icon: connected ? Icons.wifi_rounded : Icons.wifi_off_rounded,
            text: connecting
                ? 'Подключение'
                : (connected ? 'Связь есть' : 'Нет связи'),
            color:
                connected ? const Color(0xFF38F6A7) : const Color(0xFFFF4D6D),
          ),
        ),
        const SizedBox(width: 8),
        Expanded(
          child: _Pill(
            icon: Icons.gps_fixed_rounded,
            text: fixOk ? 'Фикс есть' : 'Нет фикса',
            color: fixOk ? const Color(0xFF38F6A7) : const Color(0xFFFFD166),
          ),
        ),
        const SizedBox(width: 8),
        Expanded(
          child: _Pill(
            icon: Icons.satellite_alt_rounded,
            text: '${_GpsDebugScreenState._carrierLabel(carrier)} / $rtcmText',
            color: rtkOk ? const Color(0xFF38F6A7) : const Color(0xFFFFD166),
          ),
        ),
      ],
    );
  }
}

class _Pill extends StatelessWidget {
  final IconData icon;
  final String text;
  final Color color;

  const _Pill({required this.icon, required this.text, required this.color});

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 42,
      padding: const EdgeInsets.symmetric(horizontal: 10),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.13),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: color.withValues(alpha: 0.30)),
      ),
      child: Row(
        children: [
          Icon(icon, size: 18, color: color),
          const SizedBox(width: 7),
          Expanded(
            child: Text(
              text,
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: TextStyle(
                color: color,
                fontWeight: FontWeight.w900,
                fontSize: 12,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _Metric extends StatelessWidget {
  final String label;
  final String value;
  final bool? good;

  const _Metric({required this.label, required this.value, this.good});

  @override
  Widget build(BuildContext context) {
    final color = good == null
        ? Colors.white
        : (good! ? const Color(0xFF38F6A7) : const Color(0xFFFFD166));
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 3),
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: Colors.black.withValues(alpha: 0.20),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            label,
            style: TextStyle(
              color: Colors.white.withValues(alpha: 0.56),
              fontWeight: FontWeight.w800,
              fontSize: 11,
            ),
          ),
          const SizedBox(height: 5),
          Text(
            value,
            maxLines: 2,
            overflow: TextOverflow.ellipsis,
            style: TextStyle(
              color: color,
              fontWeight: FontWeight.w900,
              fontSize: 14,
              height: 1.12,
            ),
          ),
        ],
      ),
    );
  }
}

class _CoordLine extends StatelessWidget {
  final String label;
  final String value;

  const _CoordLine({required this.label, required this.value});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(
        children: [
          SizedBox(
            width: 76,
            child: Text(
              label,
              style: TextStyle(
                color: Colors.white.withValues(alpha: 0.56),
                fontWeight: FontWeight.w800,
              ),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: const TextStyle(
                fontFamily: 'monospace',
                fontWeight: FontWeight.w800,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _SavedPerimeterCard extends StatelessWidget {
  final GpsPerimeter perimeter;
  final bool selected;
  final Future<void> Function(GpsPerimeter perimeter) onOpen;
  final Future<void> Function(GpsPerimeter perimeter) onCopyJson;
  final Future<void> Function(GpsPerimeter perimeter) onDelete;

  const _SavedPerimeterCard({
    required this.perimeter,
    required this.selected,
    required this.onOpen,
    required this.onCopyJson,
    required this.onDelete,
  });

  @override
  Widget build(BuildContext context) {
    final accent = selected ? const Color(0xFFFFD166) : Colors.white;
    return Container(
      margin: const EdgeInsets.only(bottom: 10),
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: accent.withValues(alpha: selected ? 0.10 : 0.045),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(
          color: accent.withValues(alpha: selected ? 0.35 : 0.10),
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Row(
            children: [
              Icon(Icons.route_rounded, size: 18, color: accent),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  perimeter.name,
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(fontWeight: FontWeight.w900),
                ),
              ),
              const SizedBox(width: 8),
              Text(
                '${perimeter.points.length} точек',
                style: TextStyle(
                  color: Colors.white.withValues(alpha: 0.66),
                  fontWeight: FontWeight.w800,
                ),
              ),
            ],
          ),
          const SizedBox(height: 8),
          Wrap(
            spacing: 8,
            runSpacing: 8,
            children: [
              _Action(
                icon: Icons.folder_open_rounded,
                label: 'Открыть',
                color: const Color(0xFF38F6A7),
                onTap: () {
                  unawaited(onOpen(perimeter));
                },
              ),
              _Action(
                icon: Icons.copy_rounded,
                label: 'JSON',
                color: const Color(0xFF7AA2FF),
                onTap: () {
                  unawaited(onCopyJson(perimeter));
                },
              ),
              _Action(
                icon: Icons.delete_outline_rounded,
                label: 'Удалить',
                color: const Color(0xFFFF4D6D),
                onTap: () {
                  unawaited(onDelete(perimeter));
                },
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class _MapDebugInfo extends StatelessWidget {
  final int currentPoints;
  final int openedPoints;
  final int trackPoints;
  final GpsPerimeterPoint? currentPosition;

  const _MapDebugInfo({
    required this.currentPoints,
    required this.openedPoints,
    required this.trackPoints,
    required this.currentPosition,
  });

  @override
  Widget build(BuildContext context) {
    final position = currentPosition == null
        ? 'none'
        : '${currentPosition!.lat.toStringAsFixed(8)}, '
            '${currentPosition!.lon.toStringAsFixed(8)}';
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.055),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.white.withValues(alpha: 0.10)),
      ),
      child: DefaultTextStyle(
        style: TextStyle(
          color: Colors.white.withValues(alpha: 0.76),
          fontSize: 12,
          fontWeight: FontWeight.w800,
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('current points: $currentPoints'),
            Text('saved/opened perimeter points: $openedPoints'),
            Text('track points: $trackPoints'),
            Text('current position: $position'),
          ],
        ),
      ),
    );
  }
}

class _NavigationPanel extends StatelessWidget {
  final GpsPerimeterPoint? current;
  final GpsPerimeterPoint? origin;
  final GpsPerimeterPoint? target;
  final int? targetIndex;
  final String routeLabel;
  final List<GpsPerimeterPoint> routePoints;
  final NavigationResult result;
  final MotorDriveCommand motorPreview;
  final bool motorsEnabled;
  final bool routeRunning;
  final String? roverNavState;
  final int? roverNavWpIndex;
  final int? roverNavWpTotal;
  final double? roverNavDistToWp;
  final int forwardPercent;
  final int turnPercent;
  final double? rawHeadingDegrees;
  final double? headingDegrees;
  final String headingSource;
  final double headingOffsetDegrees;
  final bool invertYaw;
  final bool invertForward;
  final bool invertSteering;
  final double? movementBearingDegrees;
  final double? movementTargetErrorDegrees;
  final VoidCallback onToggleMotors;
  final VoidCallback? onStartRoute;
  final VoidCallback onPauseRoute;
  final VoidCallback? onCalibrateHeading;
  final VoidCallback? onResetImu;
  final ValueChanged<bool> onInvertYawChanged;
  final ValueChanged<bool> onInvertForwardChanged;
  final ValueChanged<bool> onInvertSteeringChanged;
  final VoidCallback? onSetCurrentTarget;
  final ValueChanged<int>? onSelectRoutePoint;
  final VoidCallback? onNextRoutePoint;
  final VoidCallback? onClearTarget;
  final ValueChanged<double> onForwardChanged;
  final ValueChanged<double> onTurnChanged;

  const _NavigationPanel({
    required this.current,
    required this.origin,
    required this.target,
    required this.targetIndex,
    required this.routeLabel,
    required this.routePoints,
    required this.result,
    required this.motorPreview,
    required this.motorsEnabled,
    required this.routeRunning,
    required this.roverNavState,
    required this.roverNavWpIndex,
    required this.roverNavWpTotal,
    required this.roverNavDistToWp,
    required this.forwardPercent,
    required this.turnPercent,
    required this.rawHeadingDegrees,
    required this.headingDegrees,
    required this.headingSource,
    required this.headingOffsetDegrees,
    required this.invertYaw,
    required this.invertForward,
    required this.invertSteering,
    required this.movementBearingDegrees,
    required this.movementTargetErrorDegrees,
    required this.onToggleMotors,
    required this.onStartRoute,
    required this.onPauseRoute,
    required this.onCalibrateHeading,
    required this.onResetImu,
    required this.onInvertYawChanged,
    required this.onInvertForwardChanged,
    required this.onInvertSteeringChanged,
    required this.onSetCurrentTarget,
    required this.onSelectRoutePoint,
    required this.onNextRoutePoint,
    required this.onClearTarget,
    required this.onForwardChanged,
    required this.onTurnChanged,
  });

  @override
  Widget build(BuildContext context) {
    final commandColor = _commandColor(result.command);
    final selectedIndex = targetIndex != null &&
            targetIndex! >= 0 &&
            targetIndex! < routePoints.length
        ? targetIndex
        : null;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Row(
          children: [
            const Expanded(
              child: Text(
                'Навигация к точке',
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.w900),
              ),
            ),
            _CommandBadge(command: result.command, color: commandColor),
          ],
        ),
        const SizedBox(height: 10),
        _RouteHeader(
          routeLabel: routeLabel,
          targetIndex: targetIndex,
          total: routePoints.length,
          routeRunning: routeRunning,
          motorsEnabled: motorsEnabled,
        ),
        if (roverNavState != null)
          _CoordLine(
            label: 'Rover NAV',
            value:
                '$roverNavState wp=${roverNavWpIndex ?? '-'}/${roverNavWpTotal ?? '-'} dist=${_meters(roverNavDistToWp)}',
          ),
        const SizedBox(height: 10),
        Row(
          children: [
            Expanded(
              child: _Metric(
                label: 'Дистанция',
                value: _meters(result.distanceMeters),
                good: result.command == NavigationCommand.arrived,
              ),
            ),
            Expanded(
              child: _Metric(
                label: 'Азимут',
                value: _degrees(result.bearingDegrees),
              ),
            ),
            Expanded(
              child: _Metric(
                label: 'Ошибка',
                value: _signedDegrees(result.headingErrorDegrees),
                good: result.headingErrorDegrees != null &&
                    result.headingErrorDegrees!.abs() <=
                        GpsNavigationController.turnThresholdDeg,
              ),
            ),
          ],
        ),
        const SizedBox(height: 10),
        Row(
          children: [
            Expanded(
              child: _Metric(
                label: 'Моторы',
                value: motorsEnabled ? 'ВКЛ' : 'ВЫКЛ',
                good: motorsEnabled,
              ),
            ),
            Expanded(
              child: _Metric(
                label: 'M-команда',
                value: motorPreview.protocol,
                good: motorsEnabled && !motorPreview.isStop,
              ),
            ),
          ],
        ),
        const SizedBox(height: 10),
        _CoordLine(label: 'Команда', value: result.command.wireName),
        _CoordLine(label: 'Причина', value: result.reason),
        _CoordLine(
          label: 'Курс',
          value: headingDegrees == null
              ? '-'
              : '${headingDegrees!.toStringAsFixed(1)} deg ($headingSource)',
        ),
        _CoordLine(
          label: 'Raw IMU',
          value: rawHeadingDegrees == null
              ? '-'
              : '${rawHeadingDegrees!.toStringAsFixed(1)} deg',
        ),
        _CoordLine(
          label: 'IMU offset',
          value:
              '${headingOffsetDegrees.toStringAsFixed(1)} deg inv=${invertYaw ? 'ON' : 'OFF'}',
        ),
        _CoordLine(
          label: 'GPS move',
          value: movementBearingDegrees == null
              ? '-'
              : '${movementBearingDegrees!.toStringAsFixed(1)} deg',
        ),
        _CoordLine(
          label: 'Move error',
          value: _signedDegrees(movementTargetErrorDegrees),
        ),
        _CoordLine(
          label: 'Текущая',
          value: _latLon(current),
        ),
        _CoordLine(
          label: 'Тек. X/Y',
          value: _xy(result.currentLocal),
        ),
        _CoordLine(
          label: 'Цель',
          value: _latLon(target),
        ),
        _CoordLine(
          label: 'Цель X/Y',
          value: _xy(result.targetLocal),
        ),
        _CoordLine(
          label: 'Origin',
          value: _latLon(origin),
        ),
        const SizedBox(height: 10),
        Wrap(
          spacing: 8,
          runSpacing: 8,
          crossAxisAlignment: WrapCrossAlignment.center,
          children: [
            _Action(
              icon: motorsEnabled
                  ? Icons.power_settings_new_rounded
                  : Icons.power_rounded,
              label: motorsEnabled ? 'Моторы выкл' : 'Моторы вкл',
              color: motorsEnabled
                  ? const Color(0xFFFF4D6D)
                  : const Color(0xFF38F6A7),
              onTap: onToggleMotors,
            ),
            _Action(
              icon: Icons.route_rounded,
              label: 'Старт маршрут',
              color: const Color(0xFF7AA2FF),
              onTap: onStartRoute,
            ),
            _Action(
              icon: Icons.stop_circle_rounded,
              label: 'Стоп маршрут',
              color: const Color(0xFFFFD166),
              onTap: onPauseRoute,
            ),
            _Action(
              icon: Icons.explore_rounded,
              label: 'Калибр IMU',
              color: const Color(0xFF38F6A7),
              onTap: onCalibrateHeading,
            ),
            _Action(
              icon: Icons.restart_alt_rounded,
              label: 'Reset IMU',
              color: const Color(0xFFFFD166),
              onTap: onResetImu,
            ),
            _Action(
              icon: Icons.my_location_rounded,
              label: 'Цель = я',
              color: const Color(0xFF7AA2FF),
              onTap: onSetCurrentTarget,
            ),
            _Action(
              icon: Icons.skip_next_rounded,
              label: 'Следующая',
              color: const Color(0xFF38F6A7),
              onTap: onNextRoutePoint,
            ),
            _Action(
              icon: Icons.close_rounded,
              label: 'Сброс',
              color: const Color(0xFFFFD166),
              onTap: onClearTarget,
            ),
            if (routePoints.isNotEmpty && onSelectRoutePoint != null)
              Container(
                height: 42,
                padding: const EdgeInsets.symmetric(horizontal: 10),
                decoration: BoxDecoration(
                  color: Colors.white.withValues(alpha: 0.07),
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(
                    color: Colors.white.withValues(alpha: 0.14),
                  ),
                ),
                child: DropdownButtonHideUnderline(
                  child: DropdownButton<int>(
                    value: selectedIndex,
                    dropdownColor: const Color(0xFF191D24),
                    hint: const Text('Точка периметра'),
                    style: const TextStyle(
                      color: Colors.white,
                      fontWeight: FontWeight.w900,
                    ),
                    iconEnabledColor: Colors.white,
                    items: [
                      for (var i = 0; i < routePoints.length; i++)
                        DropdownMenuItem<int>(
                          value: i,
                          child: Text('Точка ${i + 1}'),
                        ),
                    ],
                    onChanged: (value) {
                      if (value != null) onSelectRoutePoint!(value);
                    },
                  ),
                ),
              ),
          ],
        ),
        const SizedBox(height: 12),
        _MotorSlider(
          label: 'Скорость вперед',
          value: forwardPercent.toDouble(),
          min: 8,
          max: 35,
          onChanged: onForwardChanged,
        ),
        _MotorSlider(
          label: 'Скорость поворота',
          value: turnPercent.toDouble(),
          min: 8,
          max: 35,
          onChanged: onTurnChanged,
        ),
        const SizedBox(height: 8),
        _NavSwitch(
          label: 'Invert IMU yaw',
          value: invertYaw,
          onChanged: onInvertYawChanged,
        ),
        _NavSwitch(
          label: 'Invert forward',
          value: invertForward,
          onChanged: onInvertForwardChanged,
        ),
        _NavSwitch(
          label: 'Invert steering',
          value: invertSteering,
          onChanged: onInvertSteeringChanged,
        ),
        const SizedBox(height: 8),
        Text(
          'Курс берется с BNO085 только после калибровки offset; при уверенном движении offset плавно уточняется по RTK/GPS heading.',
          style: TextStyle(
            color: Colors.white.withValues(alpha: 0.58),
            fontSize: 12,
            fontWeight: FontWeight.w700,
          ),
        ),
      ],
    );
  }

  static Color _commandColor(NavigationCommand command) {
    switch (command) {
      case NavigationCommand.forward:
        return const Color(0xFF38F6A7);
      case NavigationCommand.arrived:
        return const Color(0xFF7AA2FF);
      case NavigationCommand.turnLeft:
      case NavigationCommand.turnRight:
        return const Color(0xFFFFD166);
      case NavigationCommand.stop:
        return const Color(0xFFFF4D6D);
    }
  }

  static String _latLon(GpsPerimeterPoint? point) {
    if (point == null) return '-';
    return '${point.lat.toStringAsFixed(8)}, ${point.lon.toStringAsFixed(8)}';
  }

  static String _xy(LocalPointMeters? point) {
    if (point == null) return '-';
    return 'X ${point.x.toStringAsFixed(2)} m, Y ${point.y.toStringAsFixed(2)} m';
  }

  static String _meters(double? value) {
    if (value == null) return '-';
    return '${value.toStringAsFixed(2)} m';
  }

  static String _degrees(double? value) {
    if (value == null) return '-';
    return '${value.toStringAsFixed(1)} deg';
  }

  static String _signedDegrees(double? value) {
    if (value == null) return '-';
    final sign = value > 0 ? '+' : '';
    return '$sign${value.toStringAsFixed(1)} deg';
  }
}

class _CommandBadge extends StatelessWidget {
  final NavigationCommand command;
  final Color color;

  const _CommandBadge({required this.command, required this.color});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.14),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: color.withValues(alpha: 0.34)),
      ),
      child: Text(
        command.wireName,
        style: TextStyle(
          color: color,
          fontWeight: FontWeight.w900,
          fontSize: 12,
        ),
      ),
    );
  }
}

class _RouteHeader extends StatelessWidget {
  final String routeLabel;
  final int? targetIndex;
  final int total;
  final bool routeRunning;
  final bool motorsEnabled;

  const _RouteHeader({
    required this.routeLabel,
    required this.targetIndex,
    required this.total,
    required this.routeRunning,
    required this.motorsEnabled,
  });

  @override
  Widget build(BuildContext context) {
    final progress = targetIndex == null || total == 0
        ? 0.0
        : ((targetIndex! + 1) / total).clamp(0.0, 1.0);
    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.06),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.white.withValues(alpha: 0.12)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Row(
            children: [
              Icon(
                motorsEnabled
                    ? Icons.play_circle_fill_rounded
                    : Icons.pause_circle_filled_rounded,
                color: motorsEnabled
                    ? const Color(0xFF38F6A7)
                    : const Color(0xFFFFD166),
                size: 20,
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  routeRunning
                      ? 'Едет к цели: $routeLabel'
                      : 'Цель: $routeLabel',
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                    fontWeight: FontWeight.w900,
                    fontSize: 14,
                  ),
                ),
              ),
              Text(
                total == 0 ? '0/0' : '${(targetIndex ?? -1) + 1}/$total',
                style: TextStyle(
                  color: Colors.white.withValues(alpha: 0.70),
                  fontWeight: FontWeight.w900,
                ),
              ),
            ],
          ),
          const SizedBox(height: 8),
          ClipRRect(
            borderRadius: BorderRadius.circular(8),
            child: LinearProgressIndicator(
              minHeight: 8,
              value: total == 0 ? 0 : progress,
              backgroundColor: Colors.black.withValues(alpha: 0.30),
              valueColor: AlwaysStoppedAnimation<Color>(
                motorsEnabled
                    ? const Color(0xFF38F6A7)
                    : const Color(0xFF7AA2FF),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _MotorSlider extends StatelessWidget {
  final String label;
  final double value;
  final double min;
  final double max;
  final ValueChanged<double> onChanged;

  const _MotorSlider({
    required this.label,
    required this.value,
    required this.min,
    required this.max,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        SizedBox(
          width: 118,
          child: Text(
            label,
            style: TextStyle(
              color: Colors.white.withValues(alpha: 0.62),
              fontWeight: FontWeight.w800,
              fontSize: 12,
            ),
          ),
        ),
        Expanded(
          child: Slider(
            value: value.clamp(min, max),
            min: min,
            max: max,
            divisions: (max - min).round(),
            label: '${value.round()}%',
            onChanged: onChanged,
          ),
        ),
        SizedBox(
          width: 42,
          child: Text(
            '${value.round()}%',
            textAlign: TextAlign.right,
            style: const TextStyle(
              fontWeight: FontWeight.w900,
              fontSize: 12,
            ),
          ),
        ),
      ],
    );
  }
}

class _NavSwitch extends StatelessWidget {
  final String label;
  final bool value;
  final ValueChanged<bool> onChanged;

  const _NavSwitch({
    required this.label,
    required this.value,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return SwitchListTile(
      contentPadding: EdgeInsets.zero,
      dense: true,
      title: Text(
        label,
        style: const TextStyle(fontWeight: FontWeight.w800, fontSize: 13),
      ),
      value: value,
      onChanged: onChanged,
    );
  }
}

class _Action extends StatelessWidget {
  final IconData icon;
  final String label;
  final VoidCallback? onTap;
  final Color color;

  const _Action({
    required this.icon,
    required this.label,
    this.onTap,
    this.color = Colors.white,
  });

  @override
  Widget build(BuildContext context) {
    final enabled = onTap != null;
    final c = enabled ? color : Colors.white.withValues(alpha: 0.25);
    return InkWell(
      borderRadius: BorderRadius.circular(8),
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
        decoration: BoxDecoration(
          color: c.withValues(alpha: enabled ? 0.12 : 0.05),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: c.withValues(alpha: enabled ? 0.30 : 0.10)),
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 18, color: c),
            const SizedBox(width: 7),
            Text(
              label,
              style: TextStyle(color: c, fontWeight: FontWeight.w900),
            ),
          ],
        ),
      ),
    );
  }
}

class _IconButton extends StatelessWidget {
  final IconData icon;
  final VoidCallback? onTap;

  const _IconButton({required this.icon, this.onTap});

  @override
  Widget build(BuildContext context) {
    return InkWell(
      borderRadius: BorderRadius.circular(8),
      onTap: onTap,
      child: Container(
        width: 42,
        height: 42,
        decoration: BoxDecoration(
          color: Colors.white.withValues(alpha: 0.07),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.white.withValues(alpha: 0.12)),
        ),
        child: Icon(icon, color: Colors.white),
      ),
    );
  }
}

class _Notice extends StatelessWidget {
  final String text;
  final VoidCallback onClose;

  const _Notice({required this.text, required this.onClose});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 10),
      child: Container(
        padding: const EdgeInsets.all(10),
        decoration: BoxDecoration(
          color: const Color(0xFF7AA2FF).withValues(alpha: 0.14),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(
              color: const Color(0xFF7AA2FF).withValues(alpha: 0.30)),
        ),
        child: Row(
          children: [
            Expanded(
              child: Text(
                text,
                style: const TextStyle(fontWeight: FontWeight.w800),
              ),
            ),
            IconButton(
              onPressed: onClose,
              icon: const Icon(Icons.close_rounded),
              iconSize: 18,
            ),
          ],
        ),
      ),
    );
  }
}

class _SiteMapPainter extends CustomPainter {
  final List<GpsPerimeterPoint> points;
  final GpsPerimeterPoint? current;
  final GpsPerimeterPoint? target;
  final int? targetIndex;

  const _SiteMapPainter({
    required this.points,
    required this.current,
    required this.target,
    required this.targetIndex,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final background = Paint()..color = const Color(0xFF2B2F36);
    canvas.drawRect(Offset.zero & size, background);
    _drawGrid(canvas, size);

    final display = List<GpsPerimeterPoint>.from(points);
    final currentPoint = current;
    if (display.isEmpty && currentPoint != null) {
      display.add(currentPoint);
    }

    if (display.isEmpty) {
      _drawCenterText(canvas, size, 'Нет точек');
      return;
    }

    final projectionSource = <GpsPerimeterPoint>[...display];
    final currentPointIndex =
        currentPoint == null ? null : projectionSource.length;
    if (currentPoint != null) projectionSource.add(currentPoint);
    final targetPoint = target;
    final targetPointIndex =
        targetPoint == null ? null : projectionSource.length;
    if (targetPoint != null) projectionSource.add(targetPoint);

    final projectedAll = _projectSimple(projectionSource, size);
    final projected = projectedAll.take(display.length).toList();

    if (projected.length >= 2) {
      _drawPath(
        canvas,
        projected,
        Colors.white,
        close: projected.length >= 3,
        width: 3.0,
      );
    }

    final outlinePaint = Paint()..color = Colors.black;
    final pointPaint = Paint()..color = Colors.white;
    for (var i = 0; i < projected.length; i++) {
      final p = projected[i];
      canvas.drawCircle(p, 10, outlinePaint);
      canvas.drawCircle(p, 8, pointPaint);
      _drawLabel(
        canvas,
        p + const Offset(11, -22),
        '${i + 1}',
        Colors.white,
        14,
      );
    }

    if (currentPoint != null) {
      final currentOffset = projectedAll[currentPointIndex!];
      canvas.drawCircle(currentOffset, 13, Paint()..color = Colors.red);
      canvas.drawCircle(currentOffset, 8, Paint()..color = Colors.white);
    }

    if (targetPoint != null) {
      final targetOffset = projectedAll[targetPointIndex!];
      canvas.drawCircle(
        targetOffset,
        16,
        Paint()..color = const Color(0xFF38F6A7),
      );
      canvas.drawCircle(targetOffset, 10, Paint()..color = Colors.black);
      canvas.drawCircle(
        targetOffset,
        7,
        Paint()..color = const Color(0xFF38F6A7),
      );
      _drawLabel(
        canvas,
        targetOffset + const Offset(14, 12),
        targetIndex == null ? 'TARGET' : 'TARGET ${targetIndex! + 1}',
        const Color(0xFF38F6A7),
        13,
      );
    }

    _drawLabel(
      canvas,
      const Offset(10, 10),
      'points: ${display.length}',
      Colors.white,
      12,
    );
  }

  List<Offset> _projectSimple(List<GpsPerimeterPoint> src, Size size) {
    double minLat = src.first.lat;
    double maxLat = src.first.lat;
    double minLon = src.first.lon;
    double maxLon = src.first.lon;

    for (final p in src) {
      minLat = math.min(minLat, p.lat);
      maxLat = math.max(maxLat, p.lat);
      minLon = math.min(minLon, p.lon);
      maxLon = math.max(maxLon, p.lon);
    }

    const minRange = 0.00001;
    if ((maxLat - minLat).abs() < minRange) {
      final center = (minLat + maxLat) / 2;
      minLat = center - minRange / 2;
      maxLat = center + minRange / 2;
    }
    if ((maxLon - minLon).abs() < minRange) {
      final center = (minLon + maxLon) / 2;
      minLon = center - minRange / 2;
      maxLon = center + minRange / 2;
    }

    final latPad = (maxLat - minLat) * 0.20;
    final lonPad = (maxLon - minLon) * 0.20;
    minLat -= latPad;
    maxLat += latPad;
    minLon -= lonPad;
    maxLon += lonPad;

    final drawW = math.max(1.0, size.width);
    final drawH = math.max(1.0, size.height);
    return src.map((p) {
      final x = ((p.lon - minLon) / (maxLon - minLon)) * drawW;
      final y = drawH - ((p.lat - minLat) / (maxLat - minLat)) * drawH;
      return Offset(x.clamp(12.0, drawW - 12.0), y.clamp(12.0, drawH - 12.0));
    }).toList();
  }

  void _drawGrid(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.white.withValues(alpha: 0.055)
      ..strokeWidth = 1;
    for (double x = 0; x <= size.width; x += 32) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }
    for (double y = 0; y <= size.height; y += 32) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }
  }

  void _drawPath(
    Canvas canvas,
    List<Offset> pts,
    Color color, {
    bool close = false,
    double width = 2,
  }) {
    if (pts.length < 2) return;
    final path = Path()..moveTo(pts.first.dx, pts.first.dy);
    for (final p in pts.skip(1)) {
      path.lineTo(p.dx, p.dy);
    }
    if (close) path.close();
    canvas.drawPath(
      path,
      Paint()
        ..color = color
        ..strokeWidth = width
        ..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round,
    );
  }

  void _drawCenterText(Canvas canvas, Size size, String text) {
    _drawLabel(
      canvas,
      size.center(Offset.zero),
      text,
      Colors.white.withValues(alpha: 0.45),
      14,
      centered: true,
    );
  }

  void _drawLabel(
    Canvas canvas,
    Offset offset,
    String text,
    Color color,
    double size, {
    bool centered = false,
  }) {
    final painter = TextPainter(
      text: TextSpan(
        text: text,
        style: TextStyle(
          color: color,
          fontSize: size,
          fontWeight: FontWeight.w800,
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    final pos = centered
        ? offset - Offset(painter.width / 2, painter.height / 2)
        : offset;
    painter.paint(canvas, pos);
  }

  @override
  bool shouldRepaint(covariant _SiteMapPainter oldDelegate) => true;
}

class _PerimeterPainter extends CustomPainter {
  final List<GpsPerimeterPoint> points;

  const _PerimeterPainter(this.points);

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(
      Offset.zero & size,
      Paint()..color = const Color(0xFF2B2F36),
    );

    final gridPaint = Paint()
      ..color = Colors.white.withValues(alpha: 0.10)
      ..strokeWidth = 1;
    for (double x = 0; x <= size.width; x += 32) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }
    for (double y = 0; y <= size.height; y += 32) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }

    if (points.isEmpty) {
      _drawCenterText(canvas, size, 'Нет точек');
      return;
    }

    final screen = _project(points, size);
    final linePaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 3.0
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    if (screen.length >= 2) {
      final path = Path()..moveTo(screen.first.dx, screen.first.dy);
      for (final p in screen.skip(1)) {
        path.lineTo(p.dx, p.dy);
      }
      if (screen.length >= 3) path.close();
      canvas.drawPath(path, linePaint);
    }

    final outlinePaint = Paint()..color = Colors.black;
    final pointPaint = Paint()..color = Colors.white;
    for (var i = 0; i < screen.length; i++) {
      final p = screen[i];
      canvas.drawCircle(p, 9.5, outlinePaint);
      canvas.drawCircle(p, 7.5, pointPaint);
      _drawLabel(canvas, p + const Offset(10, -21), '${i + 1}');
    }
  }

  List<Offset> _project(List<GpsPerimeterPoint> pts, Size size) {
    double minLat = pts.first.lat;
    double maxLat = pts.first.lat;
    double minLon = pts.first.lon;
    double maxLon = pts.first.lon;

    for (final p in pts) {
      minLat = math.min(minLat, p.lat);
      maxLat = math.max(maxLat, p.lat);
      minLon = math.min(minLon, p.lon);
      maxLon = math.max(maxLon, p.lon);
    }

    const minRange = 0.00001;
    if ((maxLat - minLat).abs() < minRange) {
      final center = (minLat + maxLat) / 2;
      minLat = center - minRange / 2;
      maxLat = center + minRange / 2;
    }
    if ((maxLon - minLon).abs() < minRange) {
      final center = (minLon + maxLon) / 2;
      minLon = center - minRange / 2;
      maxLon = center + minRange / 2;
    }

    final latPad = (maxLat - minLat) * 0.20;
    final lonPad = (maxLon - minLon) * 0.20;
    minLat -= latPad;
    maxLat += latPad;
    minLon -= lonPad;
    maxLon += lonPad;

    final drawW = math.max(1.0, size.width);
    final drawH = math.max(1.0, size.height);
    return pts.map((p) {
      final x = ((p.lon - minLon) / (maxLon - minLon)) * drawW;
      final y = drawH - ((p.lat - minLat) / (maxLat - minLat)) * drawH;
      return Offset(x.clamp(12.0, drawW - 12.0), y.clamp(12.0, drawH - 12.0));
    }).toList();
  }

  void _drawLabel(Canvas canvas, Offset offset, String text) {
    final painter = TextPainter(
      text: TextSpan(
        text: text,
        style: const TextStyle(
          color: Colors.white,
          fontSize: 13,
          fontWeight: FontWeight.w900,
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    painter.paint(canvas, offset);
  }

  void _drawCenterText(Canvas canvas, Size size, String text) {
    final span = TextSpan(
      text: text,
      style: TextStyle(
        color: Colors.white.withValues(alpha: 0.45),
        fontWeight: FontWeight.w900,
      ),
    );
    final painter = TextPainter(
      text: span,
      textDirection: TextDirection.ltr,
    )..layout();
    painter.paint(
      canvas,
      size.center(Offset.zero) - Offset(painter.width / 2, painter.height / 2),
    );
  }

  @override
  bool shouldRepaint(covariant _PerimeterPainter oldDelegate) {
    return true;
  }
}
