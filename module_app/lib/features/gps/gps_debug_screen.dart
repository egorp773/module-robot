import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:go_router/go_router.dart';

import '../../core/gps_display_math.dart';
import '../../core/gps_perimeter_storage.dart';
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
  static const int _maxRtcmAgeMs = 10000;

  final List<GpsPerimeterPoint> _trail = [];
  static const int _trailMaxPoints = 500;
  static const double _trailMinDistanceM = 0.15;

  // Navigation state
  GpsPerimeterPoint? _savedTarget;
  DateTime? _lastNavigationLogAt;
  bool _navActive = false;

  // UI state
  final TextEditingController _hostCtrl = TextEditingController();
  final FocusNode _hostFocus = FocusNode();
  String? _notice;
  Future<List<GpsPerimeter>>? _savedFuture;
  static const double _autoPointMinDistanceM = 0.20;
  final List<GpsPerimeterPoint> _points = [];
  final TextEditingController _nameCtrl = TextEditingController();
  bool _recording = false;
  Timer? _recordTimer;
  GpsPerimeter? _activeSaved;

  @override
  void initState() {
    super.initState();
    _savedFuture = GpsPerimeterStorage.list();
    _hostCtrl.text = WifiRobotHostNotifier.defaultHost;
    final stamp = DateTime.now();
    _nameCtrl.text = 'GPS ${stamp.year}-${_two(stamp.month)}-${_two(stamp.day)} ${_two(stamp.hour)}-${_two(stamp.minute)}';
  }

  @override
  void dispose() {
    _recordTimer?.cancel();
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

    // Check for nav state changes
    ref.listen<WifiConnectionState>(wifiConnectionProvider, (_, next) {
      _appendTrail(next);
      _maybeLogNavigation(next);
      // Sync _navActive with rover state
      if (next.navState == 'ARRIVED' && _navActive) {
        setState(() {
          _navActive = false;
          _notice = 'Робот прибыл к цели!';
        });
      }
    });

    if (!_hostFocus.hasFocus && _hostCtrl.text != robotHost) {
      _hostCtrl.text = robotHost;
    }

    // GPS data
    final fixOk = (wifi.gpsFixType ?? 0) >= 3;
    final rtkOk = wifi.gpsCarrier == 'fixed';
    final rtcmFresh = wifi.rtcmAgeMs != null && wifi.rtcmAgeMs! <= _maxRtcmAgeMs;
    final currentPoint = _currentPoint(wifi);
    final accM = wifi.gpsAccuracy == null ? null : wifi.gpsAccuracy! / 1000.0;

    // Navigation calculations
    final navResult = _calcNav(wifi);

    return Scaffold(
      backgroundColor: const Color(0xFF050608),
      body: SafeArea(
        child: Column(
          children: [
            // Header
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
                      'GPS Отладка',
                      style: TextStyle(fontSize: 18, fontWeight: FontWeight.w900),
                    ),
                  ),
                  _IconButton(
                    icon: wifi.isConnected ? Icons.link_off_rounded : Icons.wifi_rounded,
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
                  // Notice
                  if (_notice != null)
                    _Notice(text: _notice!, onClose: () => setState(() => _notice = null)),

                  // Status pills
                  _StatusStrip(
                    connected: wifi.isConnected,
                    connecting: wifi.isConnecting,
                    fixOk: fixOk,
                    rtkOk: rtkOk,
                    carrier: wifi.gpsCarrier,
                    rtcmFresh: rtcmFresh,
                  ),
                  const SizedBox(height: 10),

                  // Connection IP
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
                              border: OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
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

                  // GPS metrics
                  _Panel(
                    child: Column(
                      children: [
                        Row(
                          children: [
                            Expanded(child: _Metric(label: 'Фикс', value: _fixLabel(wifi.gpsFixType), good: fixOk)),
                            Expanded(child: _Metric(label: 'RTK', value: _carrierLabel(wifi.gpsCarrier), good: rtkOk)),
                            Expanded(child: _Metric(label: 'Спутн.', value: wifi.gpsSatellites?.toString() ?? '-', good: (wifi.gpsSatellites ?? 0) >= 12)),
                          ],
                        ),
                        const SizedBox(height: 8),
                        Row(
                          children: [
                            Expanded(child: _Metric(label: 'Точность', value: accM == null ? '-' : '${accM.toStringAsFixed(2)} м', good: accM != null && accM <= 0.03)),
                            Expanded(child: _Metric(label: 'RTCM age', value: wifi.rtcmAgeMs == null ? '-' : '${wifi.rtcmAgeMs}ms', good: rtcmFresh)),
                            Expanded(child: _Metric(label: 'IMU yaw', value: wifi.imuYaw == null ? '-' : '${wifi.imuYaw!.toStringAsFixed(1)}°', good: wifi.imuFresh == true)),
                          ],
                        ),
                        const SizedBox(height: 8),
                        _CoordLine(label: 'Lat', value: wifi.gpsLat?.toStringAsFixed(8) ?? '-'),
                        _CoordLine(label: 'Lon', value: wifi.gpsLon?.toStringAsFixed(8) ?? '-'),
                        _CoordLine(label: 'Курс', value: _headingValue(wifi)),
                      ],
                    ),
                  ),
                  const SizedBox(height: 10),

                  // Simple Navigation Panel
                  _Panel(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        const Text('Простая навигация', style: TextStyle(fontSize: 16, fontWeight: FontWeight.w900)),
                        const SizedBox(height: 8),
                        Text(
                          _savedTarget == null
                              ? 'Цель не установлена'
                              : 'Цель: ${_savedTarget!.lat.toStringAsFixed(8)}, ${_savedTarget!.lon.toStringAsFixed(8)}',
                          style: TextStyle(color: _savedTarget == null ? Colors.white.withValues(alpha: 0.5) : const Color(0xFF38F6A7), fontWeight: FontWeight.w800),
                        ),
                        const SizedBox(height: 8),
                        // Navigation info
                        Row(
                          children: [
                            Expanded(child: _Metric(label: 'Дистанция', value: navResult.distanceText, good: navResult.arrived)),
                            Expanded(child: _Metric(label: 'Азимут', value: navResult.bearingText)),
                            Expanded(child: _Metric(label: 'Ошибка курса', value: navResult.errorText, good: navResult.errorGood)),
                          ],
                        ),
                        const SizedBox(height: 4),
                        _CoordLine(label: 'Rover NAV', value: wifi.navState ?? '-'),
                        _CoordLine(label: 'Скорость', value: wifi.navDistToWp == null ? '-' : '${wifi.navDistToWp!.toStringAsFixed(2)} м'),
                        const SizedBox(height: 10),
                        // Buttons row 1: Save target / Go to target
                        Row(
                          children: [
                            Expanded(
                              child: _BigButton(
                                icon: Icons.bookmark_add_rounded,
                                label: 'Сохранить\nцель',
                                color: const Color(0xFF7AA2FF),
                                enabled: currentPoint != null,
                                onTap: () => _saveTarget(wifi),
                              ),
                            ),
                            const SizedBox(width: 8),
                            Expanded(
                              child: _BigButton(
                                icon: Icons.play_arrow_rounded,
                                label: 'Ехать\nк цели',
                                color: const Color(0xFF38F6A7),
                                enabled: _savedTarget != null && wifi.isConnected,
                                onTap: () => _goToTarget(wifi),
                              ),
                            ),
                            const SizedBox(width: 8),
                            Expanded(
                              child: _BigButton(
                                icon: Icons.stop_rounded,
                                label: 'Стоп',
                                color: const Color(0xFFFF4D6D),
                                enabled: _navActive,
                                onTap: _stopNav,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 8),
                        // Buttons row 2: Calibrate IMU / Clear target
                        Row(
                          children: [
                            Expanded(
                              child: _BigButton(
                                icon: Icons.explore_rounded,
                                label: 'Калибровать\nIMU',
                                color: const Color(0xFFFFD166),
                                enabled: wifi.imuFresh == true,
                                onTap: () => _calibrateImu(wifi),
                              ),
                            ),
                            const SizedBox(width: 8),
                            Expanded(
                              child: _BigButton(
                                icon: Icons.clear_rounded,
                                label: 'Очистить\nцель',
                                color: Colors.white54,
                                enabled: _savedTarget != null,
                                onTap: _clearTarget,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 8),
                        Text(
                          '1. Сохрани цель (当前位置). 2. Отнеси робота. 3. Нажми "Ехать". Робот сам вычисляет путь.',
                          style: TextStyle(color: Colors.white.withValues(alpha: 0.5), fontSize: 11),
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 10),

                  // Map with trail
                  _Panel(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        Row(
                          children: [
                            const Expanded(child: Text('Карта', style: TextStyle(fontSize: 16, fontWeight: FontWeight.w900))),
                            Text('${_trail.length} точек', style: TextStyle(color: Colors.white.withValues(alpha: 0.6), fontWeight: FontWeight.w800)),
                          ],
                        ),
                        const SizedBox(height: 10),
                        SizedBox(
                          height: 250,
                          child: ClipRRect(
                            borderRadius: BorderRadius.circular(8),
                            child: CustomPaint(
                              painter: _TrailPainter(
                                trail: _trail,
                                current: currentPoint,
                                target: _savedTarget,
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
                              label: 'Точка',
                              color: const Color(0xFF7AA2FF),
                              onTap: currentPoint == null ? null : () => _appendTrail(wifi, force: true),
                            ),
                            _Action(
                              icon: Icons.delete_sweep_rounded,
                              label: 'Стереть',
                              onTap: _trail.isEmpty ? null : () => setState(() => _trail.clear()),
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 10),

                  // Perimeter recording (simplified)
                  _Panel(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        Row(
                          children: [
                            const Expanded(child: Text('Периметр', style: TextStyle(fontSize: 16, fontWeight: FontWeight.w900))),
                            Text('${_points.length} точек', style: TextStyle(color: Colors.white.withValues(alpha: 0.6), fontWeight: FontWeight.w800)),
                          ],
                        ),
                        const SizedBox(height: 10),
                        SizedBox(
                          height: 180,
                          child: ClipRRect(
                            borderRadius: BorderRadius.circular(8),
                            child: CustomPaint(
                              painter: _PerimeterPainter(_points),
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
                              icon: _recording ? Icons.pause_rounded : Icons.fiber_manual_record_rounded,
                              label: _recording ? 'Пауза' : 'Запись',
                              color: _recording ? const Color(0xFFFFD166) : const Color(0xFFFF4D6D),
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
                              onTap: _points.isEmpty ? null : () => setState(() => _points.removeLast()),
                            ),
                            _Action(
                              icon: Icons.save_rounded,
                              label: 'Сохранить',
                              onTap: _save,
                            ),
                            _Action(
                              icon: Icons.delete_outline_rounded,
                              label: 'Очистить',
                              onTap: _points.isEmpty ? null : _clearPoints,
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 10),

                  // Saved perimeters
                  _Panel(
                    child: FutureBuilder<List<GpsPerimeter>>(
                      future: _savedFuture,
                      builder: (context, snapshot) {
                        final saved = snapshot.data ?? const [];
                        return Column(
                          crossAxisAlignment: CrossAxisAlignment.stretch,
                          children: [
                            const Text('Сохраненные', style: TextStyle(fontSize: 16, fontWeight: FontWeight.w900)),
                            const SizedBox(height: 8),
                            if (saved.isEmpty)
                              Text('Нет сохраненных периметров', style: TextStyle(color: Colors.white.withValues(alpha: 0.5)))
                            else
                              ...saved.map((p) => _SavedPerimeterCard(
                                perimeter: p,
                                selected: _activeSaved?.id == p.id,
                                onOpen: _openSaved,
                                onDelete: _deleteSaved,
                              )),
                          ],
                        );
                      },
                    ),
                  ),
                  const SizedBox(height: 10),

                  // Log
                  _Panel(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        const Text('Журнал', style: TextStyle(fontSize: 16, fontWeight: FontWeight.w900)),
                        const SizedBox(height: 8),
                        SizedBox(
                          height: 120,
                          child: ListView(
                            reverse: true,
                            children: wifi.rxLog.reversed.take(30).map((line) => Text(
                              line,
                              style: const TextStyle(fontFamily: 'monospace', fontSize: 10),
                            )).toList(),
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

  // Navigation calculations
  _NavResult _calcNav(WifiConnectionState wifi) {
    if (_savedTarget == null) {
      return const _NavResult(distanceText: '-', bearingText: '-', errorText: '-', arrived: false);
    }
    final current = _currentPoint(wifi);
    if (current == null) {
      return const _NavResult(distanceText: '-', bearingText: '-', errorText: '-', arrived: false);
    }

    final dist = GpsDisplayGeometry.distanceMeters(current.lat, current.lon, _savedTarget!.lat, _savedTarget!.lon);
    final bearing = GpsDisplayGeometry.bearingDegrees(current.lat, current.lon, _savedTarget!.lat, _savedTarget!.lon);

    double? error;
    final heading = _getHeading(wifi);
    if (heading != null) {
      error = GpsDisplayGeometry.headingErrorDegrees(heading, bearing);
    }

    return _NavResult(
      distanceText: '${dist.toStringAsFixed(2)} м',
      bearingText: '${bearing.toStringAsFixed(1)}°',
      bearing: bearing,
      errorText: error == null ? '-' : '${error >= 0 ? '+' : ''}${error.toStringAsFixed(1)}°',
      errorGood: error != null && error.abs() <= 20,
      arrived: dist < 0.3,
    );
  }

  void _saveTarget(WifiConnectionState wifi) {
    final point = _currentPoint(wifi);
    if (point == null) {
      setState(() => _notice = 'Нет GPS позиции');
      return;
    }
    setState(() {
      _savedTarget = point;
      _notice = 'Цель сохранена';
    });
  }

  void _goToTarget(WifiConnectionState wifi) {
    if (_savedTarget == null) {
      setState(() => _notice = 'Сначала сохрани цель');
      return;
    }
    if (!wifi.isConnected) {
      setState(() => _notice = 'Нет связи с роботом');
      return;
    }
    // Calibrate IMU using its own IMU yaw (robot ignores any parameters)
    ref.read(wifiConnectionProvider.notifier).sendRaw('CAL_IMU');
    // Send GO_TO command
    ref.read(wifiConnectionProvider.notifier).sendGoToTarget(_savedTarget!.lat, _savedTarget!.lon);
    setState(() {
      _navActive = true;
      _notice = 'Отправляю команду GO_TO...';
    });
  }

  void _stopNav() {
    ref.read(wifiConnectionProvider.notifier).sendNavStop();
    setState(() {
      _navActive = false;
      _notice = 'Стоп отправлен';
    });
  }

  void _calibrateImu(WifiConnectionState wifi) {
    // Send CAL_IMU_SELF - robot sets heading = current IMU yaw
    // Don't pass any bearing - robot uses its own IMU yaw
    ref.read(wifiConnectionProvider.notifier).sendRaw('CAL_IMU_SELF');
    setState(() => _notice = 'Калибровка отправлена...');
  }

  // Use IMU yaw if fresh, otherwise GPS heading
  double? _getHeading(WifiConnectionState wifi) {
    return wifi.imuFresh == true ? wifi.imuYaw : wifi.gpsHeading;
  }

  String _headingValue(WifiConnectionState wifi) {
    final h = _getHeading(wifi);
    return h == null ? '-' : '${h.toStringAsFixed(1)}°';
  }

  void _clearTarget() {
    ref.read(wifiConnectionProvider.notifier).sendNavStop();
    setState(() {
      _savedTarget = null;
      _navActive = false;
      _notice = 'Цель очищена';
    });
  }

  GpsPerimeterPoint? _currentPoint(WifiConnectionState wifi) {
    final lat = wifi.gpsLat;
    final lon = wifi.gpsLon;
    if (lat == null || lon == null) return null;
    // Invalid GPS: both coordinates near zero (uninitialized) or outside valid range
    if (lat.abs() < 0.000001 && lon.abs() < 0.000001) return null;
    if (lat < -90 || lat > 90 || lon < -180 || lon > 180) return null;
    return GpsPerimeterPoint(lat: lat, lon: lon, hAccM: wifi.gpsAccuracy == null ? null : wifi.gpsAccuracy! / 1000.0, at: DateTime.now());
  }

  void _appendTrail(WifiConnectionState wifi, {bool force = false}) {
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

  void _maybeLogNavigation(WifiConnectionState wifi) {
    final target = _savedTarget;
    if (target == null) return;
    final now = DateTime.now();
    final last = _lastNavigationLogAt;
    if (last != null && now.difference(last).inSeconds < 3) return;
    _lastNavigationLogAt = now;

    final nav = _calcNav(wifi);
    ref.read(wifiConnectionProvider.notifier).addLocalLog(
      'NAV: dist=${nav.distanceText} err=${nav.errorText} rover=${wifi.navState ?? "-"}',
    );
  }

  void _toggleRecording(WifiConnectionState wifi) {
    if (_recording) {
      _recordTimer?.cancel();
      _recordTimer = null;
      setState(() => _recording = false);
      return;
    }
    if (wifi.gpsCarrier != 'fixed') {
      setState(() => _notice = 'Нужен RTK FIXED для записи');
      return;
    }
    _addCurrentPoint(wifi, force: true);
    _recordTimer = Timer.periodic(const Duration(milliseconds: 700), (_) {
      _addCurrentPoint(ref.read(wifiConnectionProvider));
    });
    setState(() => _recording = true);
  }

  void _addCurrentPoint(WifiConnectionState wifi, {bool force = false}) {
    if (wifi.gpsCarrier != 'fixed') return;
    final lat = wifi.gpsLat;
    final lon = wifi.gpsLon;
    if (lat == null || lon == null) return;

    final point = GpsPerimeterPoint(lat: lat, lon: lon, hAccM: wifi.gpsAccuracy == null ? null : wifi.gpsAccuracy! / 1000.0, at: DateTime.now());

    if (!force && _points.isNotEmpty) {
      final last = _points.last;
      final dist = _distanceM(last.lat, last.lon, point.lat, point.lon);
      if (dist < _autoPointMinDistanceM) return;
    }

    setState(() {
      _points.add(point);
      _activeSaved = null;
    });
  }

  void _clearPoints() {
    _recordTimer?.cancel();
    _recordTimer = null;
    setState(() {
      _recording = false;
      _points.clear();
      _activeSaved = null;
    });
  }

  Future<void> _saveRobotHost(String value) async {
    await ref.read(wifiRobotHostProvider.notifier).setHost(value);
    if (mounted) {
      final saved = ref.read(wifiRobotHostProvider);
      _hostCtrl.text = saved;
      FocusScope.of(context).unfocus();
      setState(() => _notice = 'IP сохранен: $saved');
    }
  }

  Future<void> _save() async {
    if (_points.length < 3) {
      setState(() => _notice = 'Нужно минимум 3 точки');
      return;
    }
    try {
      final saved = await GpsPerimeterStorage.save(_nameCtrl.text, _points);
      if (!mounted) return;
      setState(() {
        _activeSaved = saved;
        _savedFuture = GpsPerimeterStorage.list();
        _notice = 'Сохранено: ${saved.points.length} точек';
      });
    } catch (e) {
      setState(() => _notice = 'Ошибка: $e');
    }
  }

  Future<void> _openSaved(GpsPerimeter p) async {
    try {
      final loaded = await GpsPerimeterStorage.load(p.id);
      if (!mounted || loaded == null) return;
      setState(() {
        _points..clear()..addAll(loaded.points);
        _activeSaved = loaded;
        _nameCtrl.text = loaded.name;
        _notice = 'Открыт: ${loaded.points.length} точек';
      });
    } catch (e) {
      setState(() => _notice = 'Ошибка: $e');
    }
  }

  Future<void> _deleteSaved(GpsPerimeter p) async {
    try {
      await GpsPerimeterStorage.delete(p.id);
      if (!mounted) return;
      setState(() {
        if (_activeSaved?.id == p.id) _activeSaved = null;
        _savedFuture = GpsPerimeterStorage.list();
      });
    } catch (e) {
      setState(() => _notice = 'Ошибка: $e');
    }
  }

  static String _fixLabel(int? fixType) {
    switch (fixType) {
      case 0: return 'нет';
      case 2: return '2D';
      case 3: return '3D';
      default: return fixType?.toString() ?? '-';
    }
  }

  static String _carrierLabel(String? c) {
    if (c == 'fixed') return 'FIXED';
    if (c == 'float') return 'FLOAT';
    return 'нет';
  }

  static double _distanceM(double lat1, double lon1, double lat2, double lon2) {
    const r = 6371000.0;
    final p1 = lat1 * math.pi / 180.0;
    final p2 = lat2 * math.pi / 180.0;
    final dp = (lat2 - lat1) * math.pi / 180.0;
    final dl = (lon2 - lon1) * math.pi / 180.0;
    final a = math.sin(dp / 2) * math.sin(dp / 2) + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) * math.sin(dl / 2);
    return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));
  }

  static String _two(int v) => v.toString().padLeft(2, '0');
}

class _NavResult {
  final String distanceText;
  final String bearingText;
  final double? bearing;
  final String errorText;
  final bool errorGood;
  final bool arrived;

  const _NavResult({
    required this.distanceText,
    required this.bearingText,
    this.bearing,
    required this.errorText,
    this.errorGood = false,
    required this.arrived,
  });
}

// Widgets

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
  final bool connected, connecting, fixOk, rtkOk, rtcmFresh;
  final String? carrier;

  const _StatusStrip({
    required this.connected, required this.connecting, required this.fixOk,
    required this.rtkOk, required this.carrier, required this.rtcmFresh,
  });

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(child: _Pill(icon: connected ? Icons.wifi_rounded : Icons.wifi_off_rounded, text: connected ? 'Связь' : 'Нет связи', color: connected ? const Color(0xFF38F6A7) : const Color(0xFFFF4D6D))),
        const SizedBox(width: 6),
        Expanded(child: _Pill(icon: Icons.gps_fixed_rounded, text: fixOk ? 'Фикс' : 'Нет', color: fixOk ? const Color(0xFF38F6A7) : const Color(0xFFFFD166))),
        const SizedBox(width: 6),
        Expanded(child: _Pill(icon: Icons.satellite_alt_rounded, text: carrier ?? 'нет', color: rtkOk ? const Color(0xFF38F6A7) : const Color(0xFFFFD166))),
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
      padding: const EdgeInsets.symmetric(horizontal: 8),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.13),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: color.withValues(alpha: 0.30)),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(icon, size: 16, color: color),
          const SizedBox(width: 5),
          Flexible(
            child: Text(text, maxLines: 1, overflow: TextOverflow.ellipsis, style: TextStyle(color: color, fontWeight: FontWeight.w900, fontSize: 12)),
          ),
        ],
      ),
    );
  }
}

class _Metric extends StatelessWidget {
  final String label, value;
  final bool? good;

  const _Metric({required this.label, required this.value, this.good});

  @override
  Widget build(BuildContext context) {
    final color = good == null ? Colors.white : (good! ? const Color(0xFF38F6A7) : const Color(0xFFFFD166));
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 2),
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.black.withValues(alpha: 0.20),
        borderRadius: BorderRadius.circular(6),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(label, style: TextStyle(color: Colors.white.withValues(alpha: 0.5), fontSize: 10, fontWeight: FontWeight.w800)),
          const SizedBox(height: 2),
          FittedBox(fit: BoxFit.scaleDown, child: Text(value, style: TextStyle(color: color, fontSize: 13, fontWeight: FontWeight.w900))),
        ],
      ),
    );
  }
}

class _CoordLine extends StatelessWidget {
  final String label, value;
  const _CoordLine({required this.label, required this.value});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          SizedBox(width: 60, child: Text(label, style: TextStyle(color: Colors.white.withValues(alpha: 0.5), fontSize: 11))),
          Expanded(child: Text(value, style: const TextStyle(fontFamily: 'monospace', fontWeight: FontWeight.w800, fontSize: 11))),
        ],
      ),
    );
  }
}

class _BigButton extends StatelessWidget {
  final IconData icon;
  final String label;
  final Color color;
  final bool enabled;
  final VoidCallback onTap;

  const _BigButton({required this.icon, required this.label, required this.color, required this.enabled, required this.onTap});

  @override
  Widget build(BuildContext context) {
    final c = enabled ? color : Colors.white.withValues(alpha: 0.25);
    return InkWell(
      borderRadius: BorderRadius.circular(10),
      onTap: enabled ? onTap : null,
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 12),
        decoration: BoxDecoration(
          color: c.withValues(alpha: enabled ? 0.15 : 0.05),
          borderRadius: BorderRadius.circular(10),
          border: Border.all(color: c.withValues(alpha: enabled ? 0.35 : 0.15)),
        ),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 24, color: c),
            const SizedBox(height: 4),
            Text(label, textAlign: TextAlign.center, style: TextStyle(color: c, fontSize: 11, fontWeight: FontWeight.w900)),
          ],
        ),
      ),
    );
  }
}

class _Action extends StatelessWidget {
  final IconData icon;
  final String label;
  final Color color;
  final VoidCallback? onTap;

  const _Action({required this.icon, required this.label, this.color = Colors.white, this.onTap});

  @override
  Widget build(BuildContext context) {
    final enabled = onTap != null;
    final c = enabled ? color : Colors.white.withValues(alpha: 0.25);
    return InkWell(
      borderRadius: BorderRadius.circular(8),
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        decoration: BoxDecoration(
          color: c.withValues(alpha: enabled ? 0.10 : 0.03),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: c.withValues(alpha: enabled ? 0.25 : 0.10)),
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 16, color: c),
            const SizedBox(width: 5),
            Text(label, style: TextStyle(color: c, fontSize: 12, fontWeight: FontWeight.w900)),
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
    return Container(
      margin: const EdgeInsets.only(bottom: 10),
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: const Color(0xFF7AA2FF).withValues(alpha: 0.14),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: const Color(0xFF7AA2FF).withValues(alpha: 0.30)),
      ),
      child: Row(
        children: [
          Expanded(child: Text(text, style: const TextStyle(fontWeight: FontWeight.w800, fontSize: 12))),
          IconButton(icon: const Icon(Icons.close_rounded, size: 18), onPressed: onClose),
        ],
      ),
    );
  }
}

class _SavedPerimeterCard extends StatelessWidget {
  final GpsPerimeter perimeter;
  final bool selected;
  final Future<void> Function(GpsPerimeter) onOpen;
  final Future<void> Function(GpsPerimeter) onDelete;

  const _SavedPerimeterCard({required this.perimeter, required this.selected, required this.onOpen, required this.onDelete});

  @override
  Widget build(BuildContext context) {
    final accent = selected ? const Color(0xFFFFD166) : Colors.white;
    return Container(
      margin: const EdgeInsets.only(bottom: 8),
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: accent.withValues(alpha: selected ? 0.10 : 0.045),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: accent.withValues(alpha: selected ? 0.30 : 0.10)),
      ),
      child: Row(
        children: [
          Icon(Icons.route_rounded, size: 18, color: accent),
          const SizedBox(width: 8),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(perimeter.name, maxLines: 1, overflow: TextOverflow.ellipsis, style: const TextStyle(fontWeight: FontWeight.w900, fontSize: 13)),
                Text('${perimeter.points.length} точек', style: TextStyle(color: Colors.white.withValues(alpha: 0.6), fontSize: 11)),
              ],
            ),
          ),
          _Action(icon: Icons.folder_open_rounded, label: 'Открыть', color: const Color(0xFF38F6A7), onTap: () => unawaited(onOpen(perimeter))),
          const SizedBox(width: 6),
          _Action(icon: Icons.delete_outline_rounded, label: 'Удалить', color: const Color(0xFFFF4D6D), onTap: () => unawaited(onDelete(perimeter))),
        ],
      ),
    );
  }
}

// Painters

class _TrailPainter extends CustomPainter {
  final List<GpsPerimeterPoint> trail;
  final GpsPerimeterPoint? current;
  final GpsPerimeterPoint? target;

  _TrailPainter({required this.trail, this.current, this.target});

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(Offset.zero & size, Paint()..color = const Color(0xFF1A1F24));

    // Grid
    final gridPaint = Paint()..color = Colors.white.withValues(alpha: 0.05)..strokeWidth = 1;
    for (double x = 0; x <= size.width; x += 32) canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    for (double y = 0; y <= size.height; y += 32) canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);

    if (trail.isEmpty && current == null && target == null) {
      _drawCenterText(canvas, size, 'Нет данных');
      return;
    }

    final all = <GpsPerimeterPoint>[];
    if (trail.isNotEmpty) all.addAll(trail);
    if (current != null) all.add(current!);
    if (target != null && !all.any((p) => p.lat == target!.lat && p.lon == target!.lon)) all.add(target!);

    if (all.isEmpty) return;

    final projected = _project(all, size);

    // Trail path
    if (trail.length >= 2) {
      final trailPaint = Paint()
        ..color = Colors.white.withValues(alpha: 0.4)
        ..strokeWidth = 2
        ..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round;
      final path = Path()..moveTo(projected[0].dx, projected[0].dy);
      for (int i = 1; i < trail.length; i++) path.lineTo(projected[i].dx, projected[i].dy);
      canvas.drawPath(path, trailPaint);
    }

    // Trail dots
    for (int i = 0; i < trail.length; i++) {
      final p = projected[i];
      canvas.drawCircle(p, 3, Paint()..color = Colors.white.withValues(alpha: 0.6));
    }

    // Current position (red)
    if (current != null) {
      final idx = trail.length;
      if (idx < projected.length) {
        canvas.drawCircle(projected[idx], 8, Paint()..color = const Color(0xFFFF4D6D));
        canvas.drawCircle(projected[idx], 5, Paint()..color = Colors.white);
      }
    }

    // Target (green)
    if (target != null) {
      final idx = all.indexWhere((p) => p.lat == target!.lat && p.lon == target!.lon);
      if (idx >= 0 && idx < projected.length) {
        canvas.drawCircle(projected[idx], 12, Paint()..color = const Color(0xFF38F6A7));
        canvas.drawCircle(projected[idx], 7, Paint()..color = Colors.black);
        canvas.drawCircle(projected[idx], 4, Paint()..color = const Color(0xFF38F6A7));
      }
    }

    _drawLabel(canvas, const Offset(8, 8), '${trail.length} pts', Colors.white, 11);
  }

  List<Offset> _project(List<GpsPerimeterPoint> src, Size size) {
    if (src.isEmpty) return [];

    double minLat = src.first.lat, maxLat = src.first.lat, minLon = src.first.lon, maxLon = src.first.lon;
    for (final p in src) {
      minLat = math.min(minLat, p.lat); maxLat = math.max(maxLat, p.lat);
      minLon = math.min(minLon, p.lon); maxLon = math.max(maxLon, p.lon);
    }

    const pad = 0.00001;
    if ((maxLat - minLat).abs() < pad) { minLat -= pad; maxLat += pad; }
    if ((maxLon - minLon).abs() < pad) { minLon -= pad; maxLon += pad; }

    final latPad = (maxLat - minLat) * 0.15;
    final lonPad = (maxLon - minLon) * 0.15;
    minLat -= latPad; maxLat += latPad;
    minLon -= lonPad; maxLon += lonPad;

    return src.map((p) {
      final x = ((p.lon - minLon) / (maxLon - minLon)) * size.width;
      final y = size.height - ((p.lat - minLat) / (maxLat - minLat)) * size.height;
      return Offset(x.clamp(8, size.width - 8), y.clamp(8, size.height - 8));
    }).toList();
  }

  void _drawCenterText(Canvas canvas, Size size, String text) {
    _drawLabel(canvas, size.center(Offset.zero), text, Colors.white.withValues(alpha: 0.4), 12, centered: true);
  }

  void _drawLabel(Canvas canvas, Offset offset, String text, Color color, double size, {bool centered = false}) {
    final painter = TextPainter(
      text: TextSpan(text: text, style: TextStyle(color: color, fontSize: size, fontWeight: FontWeight.w800)),
      textDirection: TextDirection.ltr,
    )..layout();
    final pos = centered ? offset - Offset(painter.width / 2, painter.height / 2) : offset;
    painter.paint(canvas, pos);
  }

  @override
  bool shouldRepaint(covariant _TrailPainter old) => true;
}

class _PerimeterPainter extends CustomPainter {
  final List<GpsPerimeterPoint> points;

  const _PerimeterPainter(this.points);

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(Offset.zero & size, Paint()..color = const Color(0xFF1A1F24));

    final gridPaint = Paint()..color = Colors.white.withValues(alpha: 0.08)..strokeWidth = 1;
    for (double x = 0; x <= size.width; x += 32) canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    for (double y = 0; y <= size.height; y += 32) canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);

    if (points.isEmpty) {
      _drawCenterText(canvas, size, 'Нет точек');
      return;
    }

    final screen = _project(points, size);

    // Path
    if (screen.length >= 2) {
      final path = Path()..moveTo(screen.first.dx, screen.first.dy);
      for (final p in screen.skip(1)) path.lineTo(p.dx, p.dy);
      if (screen.length >= 3) path.close();
      canvas.drawPath(path, Paint()..color = Colors.white..strokeWidth = 2..style = PaintingStyle.stroke..strokeCap = StrokeCap.round);
    }

    // Points
    for (int i = 0; i < screen.length; i++) {
      canvas.drawCircle(screen[i], 6, Paint()..color = Colors.white);
      _drawLabel(canvas, screen[i] + const Offset(8, -16), '${i + 1}', Colors.white, 11);
    }
  }

  List<Offset> _project(List<GpsPerimeterPoint> pts, Size size) {
    if (pts.isEmpty) return [];

    double minLat = pts.first.lat, maxLat = pts.first.lat, minLon = pts.first.lon, maxLon = pts.first.lon;
    for (final p in pts) {
      minLat = math.min(minLat, p.lat); maxLat = math.max(maxLat, p.lat);
      minLon = math.min(minLon, p.lon); maxLon = math.max(maxLon, p.lon);
    }

    const pad = 0.00001;
    if ((maxLat - minLat).abs() < pad) { minLat -= pad; maxLat += pad; }
    if ((maxLon - minLon).abs() < pad) { minLon -= pad; maxLon += pad; }

    final latPad = (maxLat - minLat) * 0.15;
    final lonPad = (maxLon - minLon) * 0.15;
    minLat -= latPad; maxLat += latPad;
    minLon -= lonPad; maxLon += lonPad;

    return pts.map((p) {
      final x = ((p.lon - minLon) / (maxLon - minLon)) * size.width;
      final y = size.height - ((p.lat - minLat) / (maxLat - minLat)) * size.height;
      return Offset(x.clamp(12, size.width - 12), y.clamp(12, size.height - 12));
    }).toList();
  }

  void _drawLabel(Canvas canvas, Offset offset, String text, Color color, double size) {
    final painter = TextPainter(
      text: TextSpan(text: text, style: TextStyle(color: color, fontSize: size, fontWeight: FontWeight.w900)),
      textDirection: TextDirection.ltr,
    )..layout();
    painter.paint(canvas, offset);
  }

  void _drawCenterText(Canvas canvas, Size size, String text) {
    final painter = TextPainter(
      text: TextSpan(text: text, style: TextStyle(color: Colors.white.withValues(alpha: 0.4), fontWeight: FontWeight.w900)),
      textDirection: TextDirection.ltr,
    )..layout();
    painter.paint(canvas, size.center(Offset.zero) - Offset(painter.width / 2, painter.height / 2));
  }

  @override
  bool shouldRepaint(covariant _PerimeterPainter old) => true;
}
