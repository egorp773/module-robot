import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:go_router/go_router.dart';

import '../../core/gps_perimeter_storage.dart';
import '../../core/wifi_connection.dart';

class GpsDebugScreen extends ConsumerStatefulWidget {
  const GpsDebugScreen({super.key});

  @override
  ConsumerState<GpsDebugScreen> createState() => _GpsDebugScreenState();
}

class _GpsDebugScreenState extends ConsumerState<GpsDebugScreen> {
  static const double _autoPointMinDistanceM = 0.50;

  final List<GpsPerimeterPoint> _points = [];
  final TextEditingController _nameCtrl = TextEditingController();
  Timer? _recordTimer;
  bool _recording = false;
  String? _notice;
  Future<List<GpsPerimeter>>? _savedFuture;

  @override
  void initState() {
    super.initState();
    _savedFuture = GpsPerimeterStorage.list();
    final stamp = DateTime.now();
    _nameCtrl.text =
        'GPS ${stamp.year}-${_two(stamp.month)}-${_two(stamp.day)} ${_two(stamp.hour)}-${_two(stamp.minute)}';
  }

  @override
  void dispose() {
    _recordTimer?.cancel();
    _nameCtrl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final wifi = ref.watch(wifiConnectionProvider);
    final ctrl = ref.read(wifiConnectionProvider.notifier);
    final fixOk = _fixOk(wifi);
    final ageText = _ageText(wifi);
    final accM = wifi.gpsAccuracy == null ? null : wifi.gpsAccuracy! / 1000.0;

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
                    carrier: wifi.gpsCarrier,
                    ageText: ageText,
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
                                good: wifi.gpsCarrier == 'fixed',
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
                                good: accM != null && accM <= 0.50,
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
                                label: 'RTCM',
                                value: wifi.rtcmBytes == null
                                    ? '-'
                                    : '${wifi.rtcmBytes} B',
                                good: (wifi.rtcmBytes ?? 0) > 0 &&
                                    (wifi.rtcmAgeMs ?? 999999) < 3000,
                              ),
                            ),
                            Expanded(
                              child: _Metric(
                                label: 'RTCM возраст',
                                value: wifi.rtcmAgeMs == null
                                    ? '-'
                                    : '${wifi.rtcmAgeMs} ms',
                                good: wifi.rtcmAgeMs != null &&
                                    wifi.rtcmAgeMs! < 3000,
                              ),
                            ),
                            const Expanded(child: SizedBox.shrink()),
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
                              child: Container(
                                decoration: BoxDecoration(
                                  color: const Color(0xFF0C1014),
                                  border: Border.all(
                                    color: Colors.white.withValues(alpha: 0.10),
                                  ),
                                ),
                              ),
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
                              onTap: _points.length < 3 ? null : _save,
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
                          'Автозапись добавляет точку каждые ${_autoPointMinDistanceM.toStringAsFixed(1)} м при валидном фиксе.',
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
                              ...saved.take(5).map(
                                    (p) => Padding(
                                      padding: const EdgeInsets.only(bottom: 8),
                                      child: Row(
                                        children: [
                                          const Icon(
                                            Icons.route_rounded,
                                            size: 18,
                                          ),
                                          const SizedBox(width: 8),
                                          Expanded(
                                            child: Text(
                                              p.name,
                                              maxLines: 1,
                                              overflow: TextOverflow.ellipsis,
                                              style: const TextStyle(
                                                fontWeight: FontWeight.w800,
                                              ),
                                            ),
                                          ),
                                          Text(
                                            '${p.points.length} точек',
                                            style: TextStyle(
                                              color: Colors.white
                                                  .withValues(alpha: 0.62),
                                              fontWeight: FontWeight.w800,
                                            ),
                                          ),
                                        ],
                                      ),
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

  void _toggleRecording(WifiConnectionState wifi) {
    if (_recording) {
      _recordTimer?.cancel();
      _recordTimer = null;
      setState(() => _recording = false);
      return;
    }

    if (!_fixOk(wifi)) {
      setState(() => _notice = 'Нет валидного GPS-фикса.');
      return;
    }

    _addCurrentPoint(wifi, force: true);
    _recordTimer = Timer.periodic(const Duration(milliseconds: 700), (_) {
      _addCurrentPoint(ref.read(wifiConnectionProvider));
    });
    setState(() => _recording = true);
  }

  void _addCurrentPoint(WifiConnectionState wifi, {bool force = false}) {
    final lat = wifi.gpsLat;
    final lon = wifi.gpsLon;
    if (!_fixOk(wifi) || lat == null || lon == null) {
      setState(() => _notice = 'Точка пропущена: GPS-фикс невалидный.');
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
      _notice = null;
    });
  }

  Future<void> _save() async {
    await GpsPerimeterStorage.save(_nameCtrl.text, _points);
    setState(() {
      _savedFuture = GpsPerimeterStorage.list();
      _notice = 'Сохранено точек периметра: ${_points.length}.';
    });
  }

  Future<void> _copyJson() async {
    await Clipboard.setData(
      ClipboardData(text: GpsPerimeterStorage.toExportJson(_points)),
    );
    setState(() => _notice = 'JSON периметра скопирован.');
  }

  void _clearPoints() {
    _recordTimer?.cancel();
    _recordTimer = null;
    setState(() {
      _recording = false;
      _points.clear();
      _notice = 'Текущий периметр очищен.';
    });
  }

  static bool _fixOk(WifiConnectionState wifi) {
    return (wifi.gpsFixType ?? 0) >= 3 &&
        wifi.gpsLat != null &&
        wifi.gpsLon != null;
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

  static String _ageText(WifiConnectionState wifi) {
    if (wifi.gpsAgeMs != null) return '${wifi.gpsAgeMs} ms';
    final receivedAt = wifi.gpsReceivedAt;
    if (receivedAt == null) return '-';
    final age = DateTime.now().difference(receivedAt);
    if (age.inSeconds < 2) return '${age.inMilliseconds} ms';
    return '${age.inSeconds} s';
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
  final String? carrier;
  final String ageText;

  const _StatusStrip({
    required this.connected,
    required this.connecting,
    required this.fixOk,
    required this.carrier,
    required this.ageText,
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
            text: '${_GpsDebugScreenState._carrierLabel(carrier)} / $ageText',
            color: carrier == 'fixed'
                ? const Color(0xFF38F6A7)
                : const Color(0xFF7AA2FF),
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
            maxLines: 1,
            overflow: TextOverflow.ellipsis,
            style: TextStyle(
              color: color,
              fontWeight: FontWeight.w900,
              fontSize: 15,
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

class _PerimeterPainter extends CustomPainter {
  final List<GpsPerimeterPoint> points;

  const _PerimeterPainter(this.points);

  @override
  void paint(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = Colors.white.withValues(alpha: 0.055)
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
      ..color = const Color(0xFF38F6A7)
      ..strokeWidth = 2.5
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round;

    if (screen.length >= 2) {
      final path = Path()..moveTo(screen.first.dx, screen.first.dy);
      for (final p in screen.skip(1)) {
        path.lineTo(p.dx, p.dy);
      }
      if (screen.length >= 3) path.close();
      canvas.drawPath(path, linePaint);
    }

    final pointPaint = Paint()..color = Colors.white;
    for (final p in screen) {
      canvas.drawCircle(p, 4.0, pointPaint);
    }
    canvas.drawCircle(
        screen.first, 6.0, Paint()..color = const Color(0xFF7AA2FF));
  }

  List<Offset> _project(List<GpsPerimeterPoint> pts, Size size) {
    final lat0 = pts.map((p) => p.lat).reduce((a, b) => a + b) / pts.length;
    final lon0 = pts.map((p) => p.lon).reduce((a, b) => a + b) / pts.length;
    const latScale = 111320.0;
    final lonScale = 111320.0 * math.cos(lat0 * math.pi / 180.0);

    final local = pts
        .map((p) =>
            Offset((p.lon - lon0) * lonScale, -(p.lat - lat0) * latScale))
        .toList();

    double minX = local.first.dx;
    double maxX = local.first.dx;
    double minY = local.first.dy;
    double maxY = local.first.dy;
    for (final p in local) {
      minX = math.min(minX, p.dx);
      maxX = math.max(maxX, p.dx);
      minY = math.min(minY, p.dy);
      maxY = math.max(maxY, p.dy);
    }

    final worldW = math.max(1.0, maxX - minX);
    final worldH = math.max(1.0, maxY - minY);
    final scale = math.min(
      (size.width - 34) / worldW,
      (size.height - 34) / worldH,
    );
    final center = size.center(Offset.zero);
    final worldCenter = Offset((minX + maxX) / 2, (minY + maxY) / 2);

    return local.map((p) => center + (p - worldCenter) * scale).toList();
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
