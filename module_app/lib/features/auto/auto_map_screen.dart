import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:go_router/go_router.dart';

import '../../core/wifi_connection.dart';
import '../../core/map_storage.dart';
import '../../core/cleaning_route_planner.dart';
import '../../core/gps_display_math.dart';
import '../manual/manual_control_screen.dart';
import '../home/home_screen.dart' show batteryPercentProvider;

class AutoMapScreen extends ConsumerStatefulWidget {
  final String mapId;
  const AutoMapScreen({super.key, required this.mapId});

  @override
  ConsumerState<AutoMapScreen> createState() => _AutoMapScreenState();
}

class _AutoMapScreenState extends ConsumerState<AutoMapScreen> {
  static const double _draftLineStepMeters = 0.35;
  static const int _maxRouteWaypoints = 254;
  static const int _maxForbiddenPolygons = 8;
  static const int _maxForbiddenPoints = 24;

  ManualMapState? _mapState;
  bool _isLoading = true;
  String? _error;
  List<Offset> _route = []; // Построенный маршрут
  double _routeDistanceM = 0.0;
  double _routeRunTimeS = 0.0;
  bool _routeSent = false;
  String? _routeWorkflowError;

  // Состояние для управления картой
  double _zoom = 1.0;
  Offset _pan = Offset.zero;

  @override
  void initState() {
    super.initState();
    _loadMap();
  }

  Future<void> _loadMap() async {
    // Проверяем подключение перед загрузкой карты
    final wifi = ref.read(wifiConnectionProvider);
    if (!wifi.isConnected && !wifi.isConnecting) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (!mounted) return;
        ref.read(noticeProvider.notifier).show(
              const NoticeState(
                title: 'Не подключено',
                message: 'Подключитесь к роботу для работы с картой.',
                kind: NoticeKind.warning,
              ),
            );
        // Возвращаемся назад
        Future.microtask(() {
          if (mounted) {
            if (context.canPop()) {
              context.pop();
            } else {
              context.go('/auto');
            }
          }
        });
      });
      return;
    }

    try {
      final map = await MapStorage.loadMap(widget.mapId);
      if (map == null) {
        setState(() {
          _error = 'Карта не найдена';
          _isLoading = false;
        });
        return;
      }

      setState(() {
        _mapState = map;
        _zoom = map.zoom;
        _pan = map.pan;
        _isLoading = false;
      });
    } catch (e) {
      setState(() {
        _error = 'Ошибка загрузки: $e';
        _isLoading = false;
      });
    }
  }

  Future<void> _toggleWifiConnection() async {
    final wifi = ref.read(wifiConnectionProvider);
    final ctrl = ref.read(wifiConnectionProvider.notifier);

    if (wifi.isConnected) {
      await ctrl.disconnect();
      return;
    }

    // Подключаемся к WebSocket
    await ctrl.connect();
  }

  Offset _currentRobotStart(ManualMapState map, WifiConnectionState wifi) {
    if (map.coordinateType == 'gps' &&
        map.refLat != null &&
        map.refLon != null &&
        wifi.gpsLat != null &&
        wifi.gpsLon != null) {
      final geometry = GpsDisplayGeometry(
        originLat: map.refLat!,
        originLon: map.refLon!,
      );
      final local = geometry.toLocal(wifi.gpsLat!, wifi.gpsLon!);
      return Offset(local.x, local.y);
    }
    return map.startPoint ?? map.robot;
  }

  ManualMapState _mapWithLiveRobot(
    ManualMapState map,
    WifiConnectionState wifi,
  ) {
    if (map.coordinateType != 'gps' ||
        map.refLat == null ||
        map.refLon == null ||
        wifi.gpsLat == null ||
        wifi.gpsLon == null) {
      return map;
    }
    final geometry = GpsDisplayGeometry(
      originLat: map.refLat!,
      originLon: map.refLon!,
    );
    final local = geometry.toLocal(wifi.gpsLat!, wifi.gpsLon!);
    return map.copyWith(robot: Offset(local.x, local.y));
  }

  void _buildRoute(BuildContext context, WidgetRef ref) {
    final map = _mapState;
    if (map == null) return;

    final currentStart =
        _currentRobotStart(map, ref.read(wifiConnectionProvider));
    final cleaningRoute = CleaningRoutePlanner.planRoute(
      map,
      lineStep: _draftLineStepMeters,
      borderPasses: 1,
      startOverride: currentStart,
      debugPrint: true,
    );

    if (cleaningRoute == null || cleaningRoute.path.isEmpty) {
      ref.read(noticeProvider.notifier).show(
            const NoticeState(
              title: 'Route error',
              message:
                  'Could not build a safe perimeter + snake route. Check zone, forbidden areas, and start distance.',
              kind: NoticeKind.danger,
            ),
          );
      return;
    }

    if (cleaningRoute.path.length > _maxRouteWaypoints) {
      ref.read(noticeProvider.notifier).show(
            NoticeState(
              title: 'Route is too long',
              message:
                  'Built ${cleaningRoute.path.length} waypoints, ESP32 limit is $_maxRouteWaypoints. Increase row step or split the zone.',
              kind: NoticeKind.danger,
            ),
          );
      return;
    }

    setState(() {
      _mapState = map.copyWith(robot: currentStart);
      _route = cleaningRoute.path;
      _routeDistanceM = cleaningRoute.totalDistance;
      _routeRunTimeS = _estimatedRunTimeSeconds(cleaningRoute.totalDistance);
      _routeSent = false;
      _routeWorkflowError = null;
    });

    ref.read(noticeProvider.notifier).show(
          NoticeState(
            title: 'Route built',
            message:
                'Built ${cleaningRoute.path.length} waypoints, ${cleaningRoute.totalDistance.toStringAsFixed(1)} m, about ${(_estimatedRunTimeSeconds(cleaningRoute.totalDistance) / 60).toStringAsFixed(1)} min.',
            kind: NoticeKind.success,
          ),
        );
  }

  void _sendRoute(BuildContext context, WidgetRef ref) {
    if (_route.isEmpty) {
      _showNotice(
        ref,
        title: 'Нет маршрута',
        message: 'Сначала постройте маршрут.',
        kind: NoticeKind.warning,
      );
      return;
    }

    final wifi = ref.read(wifiConnectionProvider);
    if (!wifi.isConnected) {
      _showNotice(
        ref,
        title: 'Не подключено',
        message: 'Подключитесь к роботу перед отправкой маршрута.',
        kind: NoticeKind.warning,
      );
      return;
    }

    final map = _mapState;
    if (map == null ||
        map.coordinateType != 'gps' ||
        map.refLat == null ||
        map.refLon == null) {
      const msg =
          'Route upload blocked: GPS origin/local-meter map is not ready.';
      setState(() => _routeWorkflowError = msg);
      _showNotice(
        ref,
        title: 'Маршрут не GPS',
        message: msg,
        kind: NoticeKind.danger,
      );
      return;
    }

    if (_route.length > _maxRouteWaypoints) {
      final msg =
          'Route has ${_route.length} waypoints, limit is $_maxRouteWaypoints.';
      setState(() => _routeWorkflowError = msg);
      _showNotice(
        ref,
        title: 'Route is too long',
        message: msg,
        kind: NoticeKind.danger,
      );
      return;
    }

    final forbiddenPolygons = map.forbiddens
        .map((poly) => poly.points
            .where((p) => p.dx.isFinite && p.dy.isFinite)
            .toList(growable: false))
        .where((points) => points.length >= 3)
        .toList(growable: false);

    if (forbiddenPolygons.length > _maxForbiddenPolygons ||
        forbiddenPolygons
            .any((points) => points.length > _maxForbiddenPoints)) {
      final msg =
          'Forbidden zones exceed firmware limits: max $_maxForbiddenPolygons zones, '
          '$_maxForbiddenPoints points each.';
      setState(() => _routeWorkflowError = msg);
      _showNotice(
        ref,
        title: 'Forbidden zones too complex',
        message: msg,
        kind: NoticeKind.danger,
      );
      return;
    }

    final routeCtrl = ref.read(wifiConnectionProvider.notifier);
    routeCtrl.sendForbiddenBegin(forbiddenPolygons.length);
    for (var polyIndex = 0; polyIndex < forbiddenPolygons.length; polyIndex++) {
      final points = forbiddenPolygons[polyIndex];
      for (var pointIndex = 0; pointIndex < points.length; pointIndex++) {
        final p = points[pointIndex];
        routeCtrl.sendForbiddenPoint(polyIndex, pointIndex, p.dx, p.dy);
      }
    }
    routeCtrl.sendForbiddenEnd();

    routeCtrl.sendRouteBegin(
      _route.length,
      originLat: map.refLat!,
      originLon: map.refLon!,
    );
    for (var i = 0; i < _route.length; i++) {
      final p = _route[i];
      routeCtrl.sendRoutePoint(i, p.dx, p.dy);
    }
    routeCtrl.sendRouteEnd();

    setState(() {
      _routeSent = true;
      _routeWorkflowError = null;
    });

    _showNotice(
      ref,
      title: 'Route sent',
      message: 'Robot received ${_route.length} exact waypoints from the app.',
      kind: NoticeKind.info,
    );
  }

  void _startNavigation(BuildContext context, WidgetRef ref) {
    final wifi = ref.read(wifiConnectionProvider);
    if (!wifi.isConnected) {
      _showNotice(
        ref,
        title: 'Не подключено',
        message: 'Подключитесь к роботу перед стартом.',
        kind: NoticeKind.warning,
      );
      return;
    }
    if (!_routeSent) {
      _showNotice(
        ref,
        title: 'Маршрут не отправлен',
        message: 'Сначала отправьте построенный маршрут.',
        kind: NoticeKind.warning,
      );
      return;
    }
    final carrier = wifi.gpsCarrier?.toLowerCase();
    final rtkReady = carrier == 'fixed' || carrier == 'float';
    final imuReady = wifi.imuFresh == true;
    final motorReady = wifi.motorFeedback == true;
    if (!rtkReady || !imuReady || !motorReady) {
      final msg = [
        if (!rtkReady) 'RTK is not float/fixed',
        if (!imuReady) 'IMU is not fresh',
        if (!motorReady) 'motor controller feedback is absent',
      ].join('; ');
      setState(() => _routeWorkflowError = msg);
      _showNotice(
        ref,
        title: 'Start blocked',
        message: msg,
        kind: NoticeKind.danger,
      );
      return;
    }
    ref.read(wifiConnectionProvider.notifier).sendNavStart();
    setState(() => _routeWorkflowError = null);
    _showNotice(
      ref,
      title: 'Автономка запущена',
      message: 'NAV_START sent to ESP32 rover autopilot.',
      kind: NoticeKind.info,
    );
  }

  void _pauseNavigation(WidgetRef ref) {
    ref.read(wifiConnectionProvider.notifier).sendNavPause();
  }

  void _stopNavigation(WidgetRef ref) {
    ref.read(wifiConnectionProvider.notifier).sendNavStop();
  }

  void _showNotice(
    WidgetRef ref, {
    required String title,
    required String message,
    required NoticeKind kind,
  }) {
    ref.read(noticeProvider.notifier).show(
          NoticeState(title: title, message: message, kind: kind),
        );
  }

  @override
  Widget build(BuildContext context) {
    final wifi = ref.watch(wifiConnectionProvider);
    final accent = wifi.isConnected ? Colors.white : const Color(0xFF6E6E6E);
    final notice = ref.watch(noticeProvider);
    final media = MediaQuery.of(context);
    final safeTop = media.padding.top;

    // Получаем батарею: если включена проверка Wi-Fi - используем только данные из WebSocket, иначе из настроек
    final pingCheckEnabled = ref.watch(wifiPingCheckProvider);
    final batteryFromSettings = ref.watch(batteryPercentProvider);
    final battery = pingCheckEnabled
        ? (wifi.batteryPercent ??
            batteryFromSettings) // При включенной проверке приоритет WebSocket, fallback на настройки
        : batteryFromSettings; // При выключенной проверке только настройки

    return Scaffold(
      body: LayoutBuilder(builder: (context, constraints) {
        final scaleH = constraints.maxHeight / 820.0;
        final scaleW = constraints.maxWidth / 390.0;
        final uiScale = math.min(scaleH, scaleW).clamp(0.70, 1.0);
        double u(double v) => v * uiScale;

        final pad = u(16).clamp(12.0, 16.0);
        final gap = u(10).clamp(8.0, 10.0);

        final topBarH = u(54).clamp(46.0, 54.0);
        final statusH = u(72).clamp(62.0, 72.0);

        final contentH = constraints.maxHeight - safeTop - media.padding.bottom;

        // Вычисляем высоту области кнопок (как в ручном режиме)
        final baseControls = (contentH * 0.33);
        final controlsMax = u(270).clamp(220.0, 270.0);
        final controlsMin = u(215).clamp(190.0, 215.0);
        final controlsH = baseControls.clamp(controlsMin, controlsMax);

        return Stack(
          children: [
            Positioned.fill(
              child: _PremiumStaticBackground(isConnected: wifi.isConnected),
            ),
            const Positioned.fill(child: _VignetteOverlay()),
            SafeArea(
              child: Padding(
                padding: EdgeInsets.fromLTRB(pad, pad, pad, pad),
                child: Column(
                  children: [
                    SizedBox(
                      height: topBarH,
                      child: Row(
                        children: [
                          _IconBtn(
                            icon: Icons.arrow_back_rounded,
                            onTap: () {
                              if (context.canPop()) {
                                context.pop();
                              } else {
                                context.go('/auto');
                              }
                            },
                          ),
                          SizedBox(width: gap),
                          Expanded(
                            child: Text(
                              _mapState?.mapName ?? 'Загрузка...',
                              style: TextStyle(
                                fontWeight: FontWeight.w900,
                                fontSize: u(18).clamp(16.0, 18.0),
                                color: Colors.white,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                    SizedBox(height: gap),
                    SizedBox(
                      height: statusH,
                      child: _StatusPanel(
                        uiScale: uiScale,
                        wifi: wifi,
                        batteryPercent: battery,
                        onToggle: _toggleWifiConnection,
                      ),
                    ),
                    SizedBox(height: gap),
                    // Карта (такого же размера как в ручном режиме)
                    Expanded(
                      child: LayoutBuilder(
                        builder: (context, constraints) {
                          return _isLoading
                              ? Center(
                                  child:
                                      CircularProgressIndicator(color: accent),
                                )
                              : _error != null
                                  ? Center(
                                      child: Text(
                                        _error!,
                                        style: TextStyle(
                                          color: Colors.red.withOpacity(0.8),
                                          fontWeight: FontWeight.w800,
                                        ),
                                      ),
                                    )
                                  : _mapState == null
                                      ? const Center(
                                          child: Text(
                                            'Карта не загружена',
                                            style: TextStyle(
                                              color: Colors.white,
                                              fontWeight: FontWeight.w800,
                                            ),
                                          ),
                                        )
                                      : _MapCardView(
                                          uiScale: uiScale,
                                          state: _mapWithLiveRobot(
                                              _mapState!, wifi),
                                          mapSize: constraints.biggest,
                                          route: _route,
                                          zoom: _zoom,
                                          pan: _pan,
                                          onPan: (delta) {
                                            setState(() {
                                              _pan = _pan + delta;
                                            });
                                          },
                                          onZoom: (z) {
                                            setState(() {
                                              _zoom = z.clamp(0.55, 48.0);
                                            });
                                          },
                                          onZoomIn: () {
                                            setState(() {
                                              _zoom = (_zoom * 1.12)
                                                  .clamp(0.55, 48.0);
                                            });
                                          },
                                          onZoomOut: () {
                                            setState(() {
                                              _zoom = (_zoom / 1.12)
                                                  .clamp(0.55, 48.0);
                                            });
                                          },
                                          onCenter: () {
                                            // Центрирование на роботе
                                            final center = constraints.biggest
                                                .center(Offset.zero);
                                            final baseCell = (18 * uiScale)
                                                .clamp(14.0, 20.0);
                                            final cell = baseCell * _zoom;
                                            // Текущая позиция робота на экране
                                            final robotScreenPos = center +
                                                _pan +
                                                Offset(
                                                  _mapState!.robot.dx * cell,
                                                  _mapState!.robot.dy * cell,
                                                );
                                            // Вычисляем нужный pan, чтобы робот был в центре
                                            final newPan = _pan -
                                                (robotScreenPos - center);
                                            setState(() {
                                              _pan = newPan;
                                            });
                                          },
                                        );
                        },
                      ),
                    ),
                    SizedBox(height: gap),
                    // Область кнопок снизу
                    SizedBox(
                      height: controlsH,
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          _ActionButton(
                            icon: Icons.route_rounded,
                            label: 'Build route',
                            onTap: () {
                              if (_mapState != null) {
                                _buildRoute(context, ref);
                              }
                            },
                            isPrimary: true,
                          ),
                          SizedBox(height: gap),
                          _AutoWorkflowPanel(
                            uiScale: uiScale,
                            wifi: wifi,
                            routePoints: _route.length,
                            routeDistanceM: _routeDistanceM,
                            routeRunTimeS: _routeRunTimeS,
                            mapSizeLabel: _mapSizeLabel(_mapState),
                            routeSent: _routeSent,
                            error: _routeWorkflowError,
                            onSendRoute: () => _sendRoute(context, ref),
                            onStart: () => _startNavigation(context, ref),
                            onPause: () => _pauseNavigation(ref),
                            onStop: () => _stopNavigation(ref),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ),
            // Уведомления поверх всего
            if (notice != null)
              Positioned(
                left: pad,
                right: pad,
                top: safeTop + u(110),
                child: IgnorePointer(
                  child: AnimatedSwitcher(
                    duration: const Duration(milliseconds: 220),
                    switchInCurve: Curves.easeOutCubic,
                    switchOutCurve: Curves.easeInCubic,
                    transitionBuilder: (child, anim) {
                      final slide = Tween<Offset>(
                        begin: const Offset(0, -0.18),
                        end: Offset.zero,
                      ).animate(
                        CurvedAnimation(
                            parent: anim, curve: Curves.easeOutCubic),
                      );
                      final fade = CurvedAnimation(
                          parent: anim, curve: Curves.easeOutCubic);
                      return FadeTransition(
                        opacity: fade,
                        child: SlideTransition(position: slide, child: child),
                      );
                    },
                    child: _NoticeBanner(
                      key: ValueKey(
                          '${notice.kind}-${notice.title}-${notice.message}'),
                      notice: notice,
                    ),
                  ),
                ),
              ),
          ],
        );
      }),
    );
  }
}

/// ============================================================================
/// Баннер уведомлений
/// ============================================================================
double _estimatedRunTimeSeconds(double distanceM) {
  return distanceM / 0.28 + 120.0;
}

String _mapSizeLabel(ManualMapState? map) {
  if (map == null || map.zones.isEmpty) return 'Map: -';
  var minX = double.infinity;
  var maxX = -double.infinity;
  var minY = double.infinity;
  var maxY = -double.infinity;
  for (final zone in map.zones) {
    for (final p in zone.points) {
      if (!p.dx.isFinite || !p.dy.isFinite) continue;
      minX = math.min(minX, p.dx);
      maxX = math.max(maxX, p.dx);
      minY = math.min(minY, p.dy);
      maxY = math.max(maxY, p.dy);
    }
  }
  if (!minX.isFinite || !maxX.isFinite || !minY.isFinite || !maxY.isFinite) {
    return 'Map: -';
  }
  final width = maxX - minX;
  final height = maxY - minY;
  return 'Map: ${width.toStringAsFixed(1)}x${height.toStringAsFixed(1)} m';
}

class _NoticeBanner extends StatelessWidget {
  final NoticeState notice;

  const _NoticeBanner({super.key, required this.notice});

  @override
  Widget build(BuildContext context) {
    const kGood = Color(0xFF38F6A7);
    const kBad = Color(0xFFFF4D6D);
    const kNeon = Color(0xFF3DE7FF);

    Color c;
    Color bg;
    switch (notice.kind) {
      case NoticeKind.success:
        c = kGood;
        bg = kGood.withOpacity(0.16);
        break;
      case NoticeKind.warning:
        c = const Color(0xFFFFD166);
        bg = const Color(0xFFFFD166).withOpacity(0.16);
        break;
      case NoticeKind.danger:
        c = kBad;
        bg = kBad.withOpacity(0.18);
        break;
      case NoticeKind.info:
        c = kNeon;
        bg = kNeon.withOpacity(0.14);
        break;
    }

    return ClipRRect(
      borderRadius: BorderRadius.circular(18),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 16, sigmaY: 16),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 18, vertical: 14),
          decoration: BoxDecoration(
            color: bg,
            borderRadius: BorderRadius.circular(18),
            border: Border.all(color: c.withOpacity(0.35)),
          ),
          child: Row(
            children: [
              Container(
                width: 4,
                height: 4,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: c,
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Text(
                      notice.title,
                      style: TextStyle(
                        fontWeight: FontWeight.w900,
                        fontSize: 15,
                        color: c,
                      ),
                    ),
                    const SizedBox(height: 4),
                    Text(
                      notice.message,
                      style: TextStyle(
                        fontWeight: FontWeight.w700,
                        fontSize: 12,
                        color: Colors.white.withOpacity(0.88),
                      ),
                      maxLines: 2,
                      overflow: TextOverflow.ellipsis,
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _IconBtn extends StatelessWidget {
  final IconData icon;
  final VoidCallback onTap;
  const _IconBtn({required this.icon, required this.onTap});

  @override
  Widget build(BuildContext context) {
    const accentWhite = Colors.white;

    return InkWell(
      borderRadius: BorderRadius.circular(14),
      onTap: onTap,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(14),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 14, sigmaY: 14),
          child: Container(
            width: 42,
            height: 42,
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.06),
              borderRadius: BorderRadius.circular(14),
              border: Border.all(color: Colors.white.withOpacity(0.10)),
            ),
            child: Icon(icon, color: accentWhite),
          ),
        ),
      ),
    );
  }
}

class _AutoWorkflowPanel extends StatelessWidget {
  final double uiScale;
  final WifiConnectionState wifi;
  final int routePoints;
  final double routeDistanceM;
  final double routeRunTimeS;
  final String mapSizeLabel;
  final bool routeSent;
  final String? error;
  final VoidCallback onSendRoute;
  final VoidCallback onStart;
  final VoidCallback onPause;
  final VoidCallback onStop;

  const _AutoWorkflowPanel({
    required this.uiScale,
    required this.wifi,
    required this.routePoints,
    required this.routeDistanceM,
    required this.routeRunTimeS,
    required this.mapSizeLabel,
    required this.routeSent,
    required this.error,
    required this.onSendRoute,
    required this.onStart,
    required this.onPause,
    required this.onStop,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    final gpsStatus = wifi.gpsFixType == null
        ? 'GPS: no data'
        : 'GPS: fix ${wifi.gpsFixType}, ${wifi.gpsAccuracy ?? 0} mm';
    final gpsCoords = (wifi.gpsLat == null || wifi.gpsLon == null)
        ? 'LL: -'
        : 'LL: ${wifi.gpsLat!.toStringAsFixed(7)}, '
            '${wifi.gpsLon!.toStringAsFixed(7)}';
    final navMode = wifi.navState ?? 'IDLE';
    final waypoint = wifi.navWpTotal == null
        ? 'WP: -'
        : 'WP: ${wifi.navWpIndex ?? 0}/${wifi.navWpTotal}';
    final routeStatus = routeSent ? 'sent' : 'not sent';
    final routeDistance = routePoints == 0
        ? 'Distance: -'
        : 'Distance: ${routeDistanceM.toStringAsFixed(1)} m';
    final routeTime = routePoints == 0
        ? 'Time: -'
        : 'Time: ${(routeRunTimeS / 60.0).toStringAsFixed(1)} min';
    final carrier = wifi.gpsCarrier ?? 'none';
    final rtkStatus = 'RTK: $carrier';
    final motorStatus =
        wifi.motorFeedback == true ? 'Motor: linked' : 'Motor: no feedback';
    final startReady = routeSent &&
        wifi.isConnected &&
        (wifi.gpsCarrier?.toLowerCase() == 'fixed' ||
            wifi.gpsCarrier?.toLowerCase() == 'float') &&
        wifi.imuFresh == true &&
        wifi.motorFeedback == true;

    return _GlassCard(
      borderColor: Colors.white.withOpacity(0.14),
      child: Padding(
        padding: EdgeInsets.all(u(10).clamp(8.0, 10.0)),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Wrap(
              spacing: u(8).clamp(6.0, 8.0),
              runSpacing: u(6).clamp(5.0, 6.0),
              children: [
                _StatusPill(label: 'Route: $routePoints pts, $routeStatus'),
                _StatusPill(label: mapSizeLabel),
                _StatusPill(label: routeDistance),
                _StatusPill(label: routeTime),
                _StatusPill(label: 'NAV: $navMode'),
                _StatusPill(label: waypoint),
                _StatusPill(label: gpsStatus),
                _StatusPill(label: rtkStatus),
                _StatusPill(label: motorStatus),
                _StatusPill(label: gpsCoords),
              ],
            ),
            if (error != null) ...[
              SizedBox(height: u(6).clamp(5.0, 6.0)),
              Text(
                error!,
                style: TextStyle(
                  color: const Color(0xFFFF6B7A),
                  fontSize: u(10.5).clamp(9.5, 10.5),
                  fontWeight: FontWeight.w800,
                ),
                maxLines: 2,
                overflow: TextOverflow.ellipsis,
                textAlign: TextAlign.center,
              ),
            ],
            SizedBox(height: u(10).clamp(8.0, 10.0)),
            Row(
              children: [
                Expanded(
                  child: _WorkflowButton(
                    icon: Icons.upload_rounded,
                    label: 'Send zone',
                    enabled: routePoints > 0 && wifi.isConnected,
                    onTap: onSendRoute,
                  ),
                ),
                SizedBox(width: u(8).clamp(6.0, 8.0)),
                Expanded(
                  child: _WorkflowButton(
                    icon: Icons.play_arrow_rounded,
                    label: 'Start',
                    enabled: startReady,
                    onTap: onStart,
                  ),
                ),
                SizedBox(width: u(8).clamp(6.0, 8.0)),
                Expanded(
                  child: _WorkflowButton(
                    icon: Icons.pause_rounded,
                    label: 'Pause',
                    enabled: wifi.isConnected,
                    onTap: onPause,
                  ),
                ),
                SizedBox(width: u(8).clamp(6.0, 8.0)),
                Expanded(
                  child: _WorkflowButton(
                    icon: Icons.stop_rounded,
                    label: 'Stop',
                    enabled: wifi.isConnected,
                    onTap: onStop,
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class _StatusPill extends StatelessWidget {
  final String label;

  const _StatusPill({required this.label});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 9, vertical: 5),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.06),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: Colors.white.withOpacity(0.12)),
      ),
      child: Text(
        label,
        style: TextStyle(
          color: Colors.white.withOpacity(0.82),
          fontSize: 10,
          fontWeight: FontWeight.w800,
        ),
      ),
    );
  }
}

class _WorkflowButton extends StatelessWidget {
  final IconData icon;
  final String label;
  final bool enabled;
  final VoidCallback onTap;

  const _WorkflowButton({
    required this.icon,
    required this.label,
    required this.enabled,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final color = enabled ? Colors.white : Colors.white.withOpacity(0.34);

    return InkWell(
      borderRadius: BorderRadius.circular(14),
      onTap: enabled ? onTap : null,
      child: Opacity(
        opacity: enabled ? 1.0 : 0.55,
        child: Container(
          height: 42,
          decoration: BoxDecoration(
            color: Colors.white.withOpacity(enabled ? 0.08 : 0.04),
            borderRadius: BorderRadius.circular(14),
            border: Border.all(color: color.withOpacity(0.20)),
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(icon, size: 17, color: color),
              const SizedBox(height: 2),
              Text(
                label,
                style: TextStyle(
                  color: color,
                  fontSize: 10,
                  fontWeight: FontWeight.w900,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

/// ============================================================================
/// Кнопка действия (Настройки / Построить маршрут)
/// ============================================================================
class _ActionButton extends StatelessWidget {
  final IconData icon;
  final String label;
  final VoidCallback onTap;
  final bool isPrimary;

  const _ActionButton({
    required this.icon,
    required this.label,
    required this.onTap,
    this.isPrimary = false,
  });

  @override
  Widget build(BuildContext context) {
    const accentWhite = Colors.white;

    return InkWell(
      borderRadius: BorderRadius.circular(18),
      onTap: onTap,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(18),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 16, sigmaY: 16),
          child: Container(
            padding: EdgeInsets.symmetric(
              horizontal: isPrimary ? 20 : 16,
              vertical: 16,
            ),
            decoration: BoxDecoration(
              gradient: isPrimary
                  ? LinearGradient(
                      begin: Alignment.topLeft,
                      end: Alignment.bottomRight,
                      colors: [
                        Colors.white.withOpacity(0.95),
                        Colors.white.withOpacity(0.85),
                      ],
                    )
                  : null,
              color: isPrimary ? null : Colors.white.withOpacity(0.06),
              borderRadius: BorderRadius.circular(18),
              border: Border.all(
                color: isPrimary
                    ? Colors.white.withOpacity(0.95)
                    : accentWhite.withOpacity(0.18),
                width: isPrimary ? 1.5 : 1,
              ),
            ),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(
                  icon,
                  color: isPrimary ? Colors.black : accentWhite,
                  size: isPrimary ? 20 : 18,
                ),
                if (isPrimary) ...[
                  const SizedBox(width: 10),
                  Text(
                    label,
                    style: TextStyle(
                      fontWeight: FontWeight.w900,
                      fontSize: 15,
                      color: Colors.black.withOpacity(0.92),
                    ),
                  ),
                ] else
                  SizedBox(
                    width: 42,
                    height: 42,
                    child: Center(
                      child: Icon(
                        icon,
                        color: accentWhite,
                        size: 20,
                      ),
                    ),
                  ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

/// ============================================================================
/// Background (такой же как на главном экране)
/// ============================================================================
class _PremiumStaticBackground extends StatelessWidget {
  final bool isConnected;
  const _PremiumStaticBackground({required this.isConnected});

  @override
  Widget build(BuildContext context) {
    // Черно-белый фон
    const bg0 = Color(0xFF000000);
    const bg1 = Color(0xFF1A1A1A);
    const bg2 = Color(0xFF2A2A2A);

    const tintWhite = Colors.white;
    const tintGray = Color(0xFF6E6E6E);
    final tint = isConnected ? tintWhite : tintGray;

    return Stack(
      children: [
        const Positioned.fill(
          child: DecoratedBox(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment(-0.9, -1.0),
                end: Alignment(1.0, 1.0),
                colors: [bg0, bg1, bg2],
                stops: [0.0, 0.55, 1.0],
              ),
            ),
          ),
        ),
        // Светлый градиент сверху
        Positioned.fill(
          child: Opacity(
            opacity: 0.18,
            child: DecoratedBox(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter,
                  colors: [
                    Colors.white.withOpacity(0.35),
                    Colors.transparent,
                  ],
                  stops: const [0.0, 0.30],
                ),
              ),
            ),
          ),
        ),
        const Positioned.fill(
          child: Opacity(
            opacity: 0.14,
            child: DecoratedBox(
              decoration: BoxDecoration(
                gradient: RadialGradient(
                  center: Alignment(0.0, -0.2),
                  radius: 1.15,
                  colors: [Colors.white, Colors.transparent],
                  stops: [0.0, 1.0],
                ),
              ),
            ),
          ),
        ),
        Positioned.fill(
          child: Opacity(
            opacity: 0.20,
            child: DecoratedBox(
              decoration: BoxDecoration(
                gradient: RadialGradient(
                  center: const Alignment(-0.55, -0.65),
                  radius: 1.10,
                  colors: [Colors.white.withOpacity(0.15), Colors.transparent],
                  stops: const [0.0, 1.0],
                ),
              ),
            ),
          ),
        ),
        Positioned.fill(
          child: Opacity(
            opacity: 0.18,
            child: DecoratedBox(
              decoration: BoxDecoration(
                gradient: RadialGradient(
                  center: const Alignment(0.65, 0.55),
                  radius: 1.25,
                  colors: [tint, Colors.transparent],
                  stops: const [0.0, 1.0],
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }
}

class _VignetteOverlay extends StatelessWidget {
  const _VignetteOverlay();

  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: BoxDecoration(
        gradient: RadialGradient(
          center: const Alignment(0.0, -0.1),
          radius: 1.15,
          colors: [Colors.transparent, Colors.black.withOpacity(0.58)],
          stops: const [0.55, 1.0],
        ),
      ),
    );
  }
}

/// ============================================================================
/// Карта для отображения
/// ============================================================================
class _MapCardView extends StatelessWidget {
  final double uiScale;
  final ManualMapState state;
  final Size mapSize;
  final List<Offset> route;
  final double zoom;
  final Offset pan;
  final ValueChanged<Offset> onPan;
  final ValueChanged<double> onZoom;

  final VoidCallback onZoomIn;
  final VoidCallback onZoomOut;
  final VoidCallback onCenter;

  const _MapCardView({
    required this.uiScale,
    required this.state,
    required this.mapSize,
    this.route = const [],
    required this.zoom,
    required this.pan,
    required this.onPan,
    required this.onZoom,
    required this.onZoomIn,
    required this.onZoomOut,
    required this.onCenter,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    final pad = u(12).clamp(10.0, 12.0);

    return ClipRRect(
      borderRadius: BorderRadius.circular(22),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 16, sigmaY: 16),
        child: Container(
          decoration: BoxDecoration(
            color: Colors.white.withOpacity(0.06),
            borderRadius: BorderRadius.circular(22),
            border: Border.all(color: Colors.white.withOpacity(0.16)),
          ),
          child: Padding(
            padding: EdgeInsets.all(pad),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(18),
              child: Stack(
                children: [
                  Positioned.fill(
                    child: _PanZoomSurface(
                      zoom: zoom,
                      onPan: onPan,
                      onZoom: onZoom,
                      child: CustomPaint(
                        size: mapSize,
                        painter: _GridPainter(
                          uiScale: uiScale,
                          s: state,
                          route: route,
                          zoom: zoom,
                          pan: pan,
                        ),
                      ),
                    ),
                  ),
                  Positioned(
                    right: u(10),
                    top: u(10),
                    child: Column(
                      children: [
                        _MiniGlassIcon(
                          uiScale: uiScale,
                          icon: Icons.add_rounded,
                          onTap: onZoomIn,
                        ),
                        SizedBox(height: u(8).clamp(6.0, 8.0)),
                        _MiniGlassIcon(
                          uiScale: uiScale,
                          icon: Icons.remove_rounded,
                          onTap: onZoomOut,
                        ),
                        SizedBox(height: u(8).clamp(6.0, 8.0)),
                        _MiniGlassIcon(
                          uiScale: uiScale,
                          icon: Icons.center_focus_strong_rounded,
                          onTap: onCenter,
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

/// ============================================================================
/// Отрисовка карты
/// ============================================================================
class _GridPainter extends CustomPainter {
  final double uiScale;
  final ManualMapState s;
  final List<Offset> route;
  final double zoom;
  final Offset pan;

  _GridPainter({
    required this.uiScale,
    required this.s,
    this.route = const [],
    required this.zoom,
    required this.pan,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = size.center(Offset.zero);

    // масштаб клетки
    final baseCell = (18 * uiScale).clamp(14.0, 20.0);
    final cell = baseCell * zoom;

    Offset w2s(Offset w) => center + pan + Offset(w.dx * cell, w.dy * cell);

    canvas.drawRect(
        Offset.zero & size, Paint()..color = Colors.white.withOpacity(0.03));

    final leftWorld = ((-center.dx - pan.dx) / cell) - 2;
    final rightWorld = (((size.width - center.dx) - pan.dx) / cell) + 2;
    final topWorld = ((-center.dy - pan.dy) / cell) - 2;
    final bottomWorld = (((size.height - center.dy) - pan.dy) / cell) + 2;

    final x0 = leftWorld.floor().clamp(-2000, 2000);
    final x1 = rightWorld.ceil().clamp(-2000, 2000);
    final y0 = topWorld.floor().clamp(-2000, 2000);
    final y1 = bottomWorld.ceil().clamp(-2000, 2000);

    final gPaint = Paint()
      ..color = Colors.white.withOpacity(0.08)
      ..strokeWidth = 1;

    for (int x = x0; x <= x1; x++) {
      canvas.drawLine(w2s(Offset(x.toDouble(), y0.toDouble())),
          w2s(Offset(x.toDouble(), y1.toDouble())), gPaint);
    }
    for (int y = y0; y <= y1; y++) {
      canvas.drawLine(w2s(Offset(x0.toDouble(), y.toDouble())),
          w2s(Offset(x1.toDouble(), y.toDouble())), gPaint);
    }

    const kGood = Color(0xFF38F6A7);
    const kBad = Color(0xFFFF4D6D);
    const kNeon = Color(0xFF3DE7FF);

    final zoneFill = Paint()..color = kGood.withOpacity(0.20);
    final zoneStroke = Paint()
      ..color = kGood.withOpacity(0.85)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    for (final z in s.zones) {
      final path = _polyPath(z.points, w2s);
      canvas.drawPath(path, zoneFill);
      canvas.drawPath(path, zoneStroke);
    }

    final forbFill = Paint()..color = kBad.withOpacity(0.22);
    final forbStroke = Paint()
      ..color = kBad.withOpacity(0.90)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    for (final f in s.forbiddens) {
      final path = _polyPath(f.points, w2s);
      canvas.drawPath(path, forbFill);
      canvas.drawPath(path, forbStroke);
    }

    for (final t in s.transitions) {
      _drawDashedPolyline(
        canvas,
        t.map(w2s).toList(growable: false),
        color: kNeon.withOpacity(0.9),
        stroke: 2,
      );
    }

    // Начальная точка — черный квадрат
    if (s.startPoint != null) {
      final sp = w2s(s.startPoint!);
      final squareSize = (12 * uiScale * zoom).clamp(8.0, 16.0);
      final squarePaint = Paint()
        ..color = Colors.black
        ..style = PaintingStyle.fill;
      canvas.drawRect(
        Rect.fromCenter(
          center: sp,
          width: squareSize,
          height: squareSize,
        ),
        squarePaint,
      );
      // Обводка квадрата
      final borderPaint = Paint()
        ..color = Colors.white.withOpacity(0.5)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.5;
      canvas.drawRect(
        Rect.fromCenter(
          center: sp,
          width: squareSize,
          height: squareSize,
        ),
        borderPaint,
      );
    }

    // Маршрут (если построен)
    if (route.isNotEmpty) {
      final routePaint = Paint()
        ..color = const Color(0xFFFFD700)
            .withOpacity(0.8) // Золотой цвет для маршрута
        ..strokeWidth = 2.5
        ..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round;

      final routePath = Path();
      final routePoints = route.map(w2s).toList();
      if (routePoints.isNotEmpty) {
        routePath.moveTo(routePoints.first.dx, routePoints.first.dy);
        for (int i = 1; i < routePoints.length; i++) {
          routePath.lineTo(routePoints[i].dx, routePoints[i].dy);
        }
      }
      canvas.drawPath(routePath, routePaint);
    }

    // робот — белый круг
    final rp = w2s(s.robot);
    final r = (6 * uiScale).clamp(5.0, 7.0);
    canvas.drawCircle(rp, r, Paint()..color = Colors.white.withOpacity(0.95));
  }

  Path _polyPath(List<Offset> worldPts, Offset Function(Offset) w2s) {
    final pts = worldPts.map(w2s).toList(growable: false);
    final p = Path()..moveTo(pts.first.dx, pts.first.dy);
    for (int i = 1; i < pts.length; i++) {
      p.lineTo(pts[i].dx, pts[i].dy);
    }
    p.close();
    return p;
  }

  void _drawDashedPolyline(Canvas canvas, List<Offset> pts,
      {required Color color, required double stroke}) {
    if (pts.length < 2) return;
    final paint = Paint()
      ..color = color
      ..strokeWidth = stroke
      ..style = PaintingStyle.stroke;

    const dash = 8.0;
    const gap = 6.0;

    for (int i = 0; i < pts.length - 1; i++) {
      final a = pts[i];
      final b = pts[i + 1];
      final d = (b - a).distance;
      if (d <= 0.001) continue;

      final dir = (b - a) / d;
      double t = 0;
      while (t < d) {
        final seg = math.min(dash, d - t);
        final p1 = a + dir * t;
        final p2 = a + dir * (t + seg);
        canvas.drawLine(p1, p2, paint);
        t += dash + gap;
      }
    }
  }

  @override
  bool shouldRepaint(covariant _GridPainter oldDelegate) {
    return oldDelegate.s != s ||
        oldDelegate.uiScale != uiScale ||
        oldDelegate.route != route ||
        oldDelegate.zoom != zoom ||
        oldDelegate.pan != pan;
  }
}

/// ============================================================================
/// Поверхность для панорамирования и зума
/// ============================================================================
class _PanZoomSurface extends StatefulWidget {
  final double zoom;
  final ValueChanged<Offset> onPan;
  final ValueChanged<double> onZoom;
  final Widget child;

  const _PanZoomSurface({
    required this.zoom,
    required this.onPan,
    required this.onZoom,
    required this.child,
  });

  @override
  State<_PanZoomSurface> createState() => _PanZoomSurfaceState();
}

class _PanZoomSurfaceState extends State<_PanZoomSurface> {
  double _startZoom = 1;
  Offset _lastFocal = Offset.zero;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onScaleStart: (d) {
        _startZoom = widget.zoom;
        _lastFocal = d.focalPoint;
      },
      onScaleUpdate: (d) {
        final nextZoom = (_startZoom * d.scale).clamp(0.55, 48.0);
        widget.onZoom(nextZoom);

        final delta = d.focalPoint - _lastFocal;
        _lastFocal = d.focalPoint;
        widget.onPan(delta);
      },
      child: widget.child,
    );
  }
}

/// ============================================================================
/// Мини-иконка для управления картой
/// ============================================================================
class _MiniGlassIcon extends StatelessWidget {
  final double uiScale;
  final IconData icon;
  final VoidCallback onTap;

  const _MiniGlassIcon({
    required this.uiScale,
    required this.icon,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    final s = u(44).clamp(38.0, 44.0);

    return InkWell(
      borderRadius: BorderRadius.circular(14),
      onTap: onTap,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(14),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 14, sigmaY: 14),
          child: Container(
            width: s,
            height: s,
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.06),
              borderRadius: BorderRadius.circular(14),
              border: Border.all(color: Colors.white.withOpacity(0.12)),
            ),
            child: Icon(icon, color: Colors.white.withOpacity(0.92)),
          ),
        ),
      ),
    );
  }
}

/// ============================================================================
/// Status Panel (унифицированная панель статуса)
/// ============================================================================
class _StatusPanel extends StatelessWidget {
  final double uiScale;
  final WifiConnectionState wifi;
  final int batteryPercent;
  final VoidCallback onToggle;

  const _StatusPanel({
    required this.uiScale,
    required this.wifi,
    required this.batteryPercent,
    required this.onToggle,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    const accentGray = Color(0xFF6E6E6E);
    final accent = wifi.isConnected ? Colors.white : accentGray;

    String statusText;
    Color statusColor;
    if (wifi.isConnecting) {
      statusText = 'Подключение…';
      statusColor = Colors.white;
    } else if (wifi.isConnected) {
      statusText = 'Подключено';
      statusColor = Colors.green;
    } else if (wifi.error != null) {
      statusText = wifi.error!;
      statusColor = Colors.red;
    } else {
      statusText = 'Не подключено';
      statusColor = Colors.red;
    }

    return Container(
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: statusColor.withOpacity(0.04),
            blurRadius: 8.0,
            spreadRadius: 0.2,
          ),
          BoxShadow(
            color: statusColor.withOpacity(0.03),
            blurRadius: 12.0,
            spreadRadius: 0.3,
          ),
        ],
      ),
      child: _GlassCard(
        borderColor: statusColor.withOpacity(0.15),
        child: Padding(
          padding: EdgeInsets.symmetric(
            horizontal: u(12).clamp(10.0, 12.0),
            vertical: u(10).clamp(8.0, 10.0),
          ),
          child: Row(
            children: [
              _BatteryChip(
                  uiScale: uiScale,
                  percent: batteryPercent,
                  isConnected: wifi.isConnected),
              SizedBox(width: u(10)),
              Expanded(
                child: Row(
                  children: [
                    Container(
                      decoration: BoxDecoration(
                        boxShadow: [
                          BoxShadow(
                            color: statusColor.withOpacity(0.06),
                            blurRadius: 4.0,
                            spreadRadius: 0.2,
                          ),
                        ],
                      ),
                      child: Icon(
                        wifi.isConnected
                            ? Icons.wifi_rounded
                            : Icons.wifi_off_rounded,
                        color: statusColor,
                        size: u(18).clamp(16.0, 18.0),
                      ),
                    ),
                    SizedBox(width: u(8)),
                    Expanded(
                      child: Text(
                        statusText,
                        style: TextStyle(
                          fontWeight: FontWeight.w900,
                          fontSize: u(11.5).clamp(10.5, 11.5),
                          color: statusColor,
                          shadows: [
                            Shadow(
                              color: statusColor.withOpacity(0.1),
                              blurRadius: 4.0,
                              offset: const Offset(0, 0),
                            ),
                            Shadow(
                              color: statusColor.withOpacity(0.06),
                              blurRadius: 6.0,
                              offset: const Offset(0, 0),
                            ),
                          ],
                        ),
                        maxLines: 2,
                        softWrap: true,
                        overflow: TextOverflow.visible,
                      ),
                    ),
                  ],
                ),
              ),
              SizedBox(width: u(10)),
              _ConnectBtn(
                uiScale: uiScale,
                accent: accent,
                busy: wifi.isConnecting,
                isConnected: wifi.isConnected,
                onTap: onToggle,
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _BatteryChip extends StatelessWidget {
  final double uiScale;
  final int percent;
  final bool isConnected;
  const _BatteryChip({
    required this.uiScale,
    required this.percent,
    required this.isConnected,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    final p = percent.clamp(0, 100);

    Color batteryColor;
    if (!isConnected) {
      batteryColor = const Color(0xFF6E6E6E);
    } else if (p <= 20) {
      batteryColor = const Color(0xFFCC6666);
    } else if (p <= 50) {
      batteryColor = const Color(0xFFCCAA66);
    } else {
      batteryColor = const Color(0xFF66CC66);
    }

    return Container(
      padding: EdgeInsets.symmetric(
        horizontal: u(10).clamp(8.0, 10.0),
        vertical: u(8).clamp(6.0, 8.0),
      ),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(16),
        color: Colors.white.withOpacity(0.05),
        border: Border.all(color: batteryColor.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Icon(Icons.battery_full_rounded,
              size: u(18).clamp(16.0, 18.0), color: batteryColor),
          if (isConnected) ...[
            SizedBox(width: u(6)),
            Text('$p%',
                style: TextStyle(
                    fontWeight: FontWeight.w900,
                    fontSize: u(12.5).clamp(11.0, 12.5),
                    color: batteryColor)),
          ],
        ],
      ),
    );
  }
}

class _ConnectBtn extends StatelessWidget {
  final double uiScale;
  final Color accent;
  final bool busy;
  final bool isConnected;
  final VoidCallback onTap;

  const _ConnectBtn({
    required this.uiScale,
    required this.accent,
    required this.busy,
    required this.isConnected,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    final label = isConnected ? 'Отключить' : 'Подключить';

    return InkWell(
      borderRadius: BorderRadius.circular(16),
      onTap: busy ? null : onTap,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(16),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 14, sigmaY: 14),
          child: Container(
            padding: EdgeInsets.symmetric(
              horizontal: u(12).clamp(10.0, 12.0),
              vertical: u(10).clamp(8.0, 10.0),
            ),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [
                  accent.withOpacity(0.26),
                  Colors.white.withOpacity(0.05)
                ],
              ),
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: accent.withOpacity(0.45)),
            ),
            child: busy
                ? SizedBox(
                    width: u(14).clamp(12.0, 14.0),
                    height: u(14).clamp(12.0, 14.0),
                    child: CircularProgressIndicator(
                      strokeWidth: 2,
                      valueColor: AlwaysStoppedAnimation(accent),
                    ),
                  )
                : Text(
                    label,
                    style: TextStyle(
                      fontWeight: FontWeight.w900,
                      fontSize: u(12.0).clamp(10.8, 12.0),
                    ),
                  ),
          ),
        ),
      ),
    );
  }
}

class _GlassCard extends StatelessWidget {
  final Widget child;
  final Color borderColor;

  const _GlassCard({required this.child, required this.borderColor});

  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(22),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 16, sigmaY: 16),
        child: Container(
          decoration: BoxDecoration(
            color: Colors.white.withOpacity(0.06),
            borderRadius: BorderRadius.circular(22),
            border: Border.all(color: borderColor),
          ),
          child: child,
        ),
      ),
    );
  }
}
