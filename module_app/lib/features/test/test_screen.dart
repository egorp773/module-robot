import 'dart:async';
import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:go_router/go_router.dart';

import '../../core/wifi_connection.dart';
import '../../core/gps_display_math.dart';

/// ============================================================
/// TestScreen v2 — калибровочный стенд для робота.
///
/// Главный принцип: одна кнопка = один конкретный тест.
/// Вверху — live state, всегда видно. Ниже — табы по группам тестов:
///   1) HEADING — калибровка heading
///   2) ODOMETRY — wheel_circum, kMeasToMps
///   3) STANLEY — control loop (heading/crosstrack)
///   4) DRIVE — drive N meters, perimeter, joystick
///   5) RTK — проверить hAcc, pvtAge, fix type
///   6) COMMANDS — все raw команды
///
/// "Last result" под каждой кнопкой теста — чтобы видеть результат
/// предыдущего прогона, не уходя с экрана.
/// ============================================================
class TestScreen extends ConsumerStatefulWidget {
  const TestScreen({super.key});

  @override
  ConsumerState<TestScreen> createState() => _TestScreenState();
}

class _TestScreenState extends ConsumerState<TestScreen> {
  int _tab = 0;

  // origin for drive commands
  double? _originLat;
  double? _originLon;

  // last-known initial state for odometry tests
  double? _x0, _y0, _h0;

  // test results cache (key → text)
  final Map<String, String> _results = {};
  String? _toastMsg;
  Timer? _toastTimer;
  bool _busy = false;

  // text controllers
  final _distCtrl = TextEditingController(text: "1.00");
  final _perimLCtrl = TextEditingController(text: "1.0");
  final _perimWCtrl = TextEditingController(text: "0.5");
  final _headingCtrl = TextEditingController(text: "0");
  final _rawCmdCtrl = TextEditingController();

  // map state
  final _mapScaleCtrl = TextEditingController(text: "50"); // px per meter
  bool _followRover = true;
  // planned route preview (in local meters, relative to origin)
  final List<_MapPoint> _plannedRoute = [];
  // robot trail (in local meters)
  final List<_MapPoint> _trail = [];
  // origin (set when we build first route)
  double? _mapOriginLat;
  double? _mapOriginLon;
  // last (x,y) computed from lat/lon (cached)
  double? _lastRobotX, _lastRobotY;

  // poll PING for live state — увеличенный период чтобы не забивать канал
  Timer? _pingTimer;
  static const _pingPeriod = Duration(milliseconds: 250);

  @override
  void initState() {
    super.initState();
    _pingTimer = Timer.periodic(_pingPeriod, (_) {
      final s = ref.read(wifiConnectionProvider);
      if (s.isConnected) {
        ref.read(wifiConnectionProvider.notifier).sendRaw("PING", log: false);
        // capture start state for odometry tests
        if (_x0 == null) {
          // Estimate (x,y) from current lat/lon relative to last known origin
          // We don't have it directly; use the GPS coords as a proxy.
        }
      }
    });
  }

  @override
  void dispose() {
    _pingTimer?.cancel();
    _distCtrl.dispose();
    _perimLCtrl.dispose();
    _perimWCtrl.dispose();
    _headingCtrl.dispose();
    _rawCmdCtrl.dispose();
    _toastTimer?.cancel();
    super.dispose();
  }

  // --- helpers ---

  void _toast(String s) {
    setState(() => _toastMsg = s);
    _toastTimer?.cancel();
    _toastTimer = Timer(const Duration(seconds: 3), () {
      if (mounted) setState(() => _toastMsg = null);
    });
  }

  void _setResult(String key, String value) {
    setState(() {
      _results[key] = value;
    });
  }

  void _captureOrigin() {
    final s = ref.read(wifiConnectionProvider);
    if (s.gpsLat != null && s.gpsLon != null) {
      _originLat = s.gpsLat;
      _originLon = s.gpsLon;
    }
  }

  Future<void> _sendCmd(String cmd, {String? resultKey}) async {
    final w = ref.read(wifiConnectionProvider.notifier);
    if (!ref.read(wifiConnectionProvider).isConnected) {
      _toast("Не подключено");
      return;
    }
    try {
      await w.sendRawFuture(cmd);
      if (resultKey != null) _setResult(resultKey, "→ $cmd");
      _toast("→ $cmd");
    } catch (e) {
      _toast("Ошибка: $e");
    }
  }

  // --- DRIVE ---

  Future<void> _driveTo({required double meters, required double compassDeg, String key = "drive"}) async {
    final w = ref.read(wifiConnectionProvider.notifier);
    final s = ref.read(wifiConnectionProvider);
    if (!s.isConnected) { _toast("Не подключено"); return; }
    _captureOrigin();
    if (_originLat == null) { _toast("Нет GPS"); return; }
    setState(() => _busy = true);
    try {
      final rad = compassDeg * math.pi / 180.0;
      final dx = meters * math.sin(rad);
      final dy = meters * math.cos(rad);
      await w.sendRouteBegin(2, originLat: _originLat!, originLon: _originLon!);
      await w.sendRoutePoint(0, 0, 0);
      await w.sendRoutePoint(1, dx, dy);
      await w.sendRouteEnd(2);
      await w.sendNavStart();
      _setResult(key, "еду ${meters.toStringAsFixed(2)}м на ${compassDeg.toStringAsFixed(0)}° — замерь рулеткой");
      _toast("Drive ${meters.toStringAsFixed(2)}м ${compassDeg.toStringAsFixed(0)}°");
    } catch (e) {
      _toast("Ошибка: $e");
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _driveToAndBack({required double meters, required double compassDeg, String key = "drive_back"}) async {
    final w = ref.read(wifiConnectionProvider.notifier);
    final s = ref.read(wifiConnectionProvider);
    if (!s.isConnected) { _toast("Не подключено"); return; }
    _captureOrigin();
    if (_originLat == null) { _toast("Нет GPS"); return; }
    setState(() => _busy = true);
    try {
      final rad = compassDeg * math.pi / 180.0;
      final dx = meters * math.sin(rad);
      final dy = meters * math.cos(rad);
      await w.sendRouteBegin(4, originLat: _originLat!, originLon: _originLon!);
      await w.sendRoutePoint(0, 0, 0);
      await w.sendRoutePoint(1, dx, dy);
      await w.sendRoutePoint(2, 0, 0);
      await w.sendRoutePoint(3, 0.001, 0.001);
      await w.sendRouteEnd(4);
      await w.sendNavStart();
      _setResult(key, "еду ${meters.toStringAsFixed(2)}м ${compassDeg.toStringAsFixed(0)}° + назад — замерь Δx Δy рулеткой");
      _toast("Drive ↩ ${meters}m ${compassDeg.toStringAsFixed(0)}°");
    } catch (e) {
      _toast("Ошибка: $e");
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _drivePerimeter({required double L, required double W, required bool andBack, String key = "perim"}) async {
    final w = ref.read(wifiConnectionProvider.notifier);
    final s = ref.read(wifiConnectionProvider);
    if (!s.isConnected) { _toast("Не подключено"); return; }
    _captureOrigin();
    if (_originLat == null) { _toast("Нет GPS"); return; }
    setState(() => _busy = true);
    try {
      final pts = <List<double>>[
        [0, 0], [L, 0], [L, W], [0, W], [0, 0],
      ];
      if (andBack) pts.add([0.001, 0.001]);
      final count = pts.length;
      await w.sendRouteBegin(count, originLat: _originLat!, originLon: _originLon!);
      for (var i = 0; i < count; i++) {
        await w.sendRoutePoint(i, pts[i][0], pts[i][1]);
      }
      await w.sendRouteEnd(count);
      await w.sendNavStart();
      _setResult(key, "perim ${L}×${W}м${andBack ? " + возврат" : ""} — замерь");
      _toast("Perimeter ${L}×${W}м${andBack ? " ↩" : ""}");
    } catch (e) {
      _toast("Ошибка: $e");
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _sendNav(String cmd) async {
    final w = ref.read(wifiConnectionProvider.notifier);
    if (!ref.read(wifiConnectionProvider).isConnected) { _toast("Не подключено"); return; }
    try {
      switch (cmd) {
        case "START": await w.sendNavStart(); break;
        case "PAUSE": await w.sendNavPause(); break;
        case "RESUME": await w.sendNavResume(); break;
        case "STOP": await w.sendNavStop(); break;
      }
      _toast("NAV_$cmd");
    } catch (e) {
      _toast("Ошибка: $e");
    }
  }

  void _sendMove(int left, int right) {
    final w = ref.read(wifiConnectionProvider.notifier);
    if (!ref.read(wifiConnectionProvider).isConnected) return;
    w.sendMove(left, right);
  }

  // --- BUILD ---

  @override
  Widget build(BuildContext context) {
    final s = ref.watch(wifiConnectionProvider);
    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Column(
          children: [
            _topBar(s),
            _liveStrip(s),
            _tabs(),
            Expanded(
              child: ListView(
                padding: const EdgeInsets.fromLTRB(10, 6, 10, 80),
                children: _tabContent(s),
              ),
            ),
            if (_toastMsg != null) _toastBar(_toastMsg!),
          ],
        ),
      ),
    );
  }

  Widget _topBar(WifiConnectionState s) {
    return Container(
      padding: const EdgeInsets.fromLTRB(4, 4, 12, 4),
      decoration: const BoxDecoration(border: Border(bottom: BorderSide(color: Colors.white12))),
      child: Row(
        children: [
          IconButton(
            icon: const Icon(Icons.arrow_back_ios_new, color: Colors.white, size: 18),
            onPressed: () => context.go('/'),
          ),
          const SizedBox(width: 4),
          const Text("Test Lab",
              style: TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.w900, letterSpacing: 0.5)),
          const Spacer(),
          _connPill(s),
        ],
      ),
    );
  }

  Widget _connPill(WifiConnectionState s) {
    final c = s.isConnected ? Colors.greenAccent : Colors.redAccent;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: c.withOpacity(0.10),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: c.withOpacity(0.5)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(width: 6, height: 6, decoration: BoxDecoration(color: c, shape: BoxShape.circle)),
          const SizedBox(width: 4),
          Text(s.isConnected ? "ON" : "OFF", style: TextStyle(color: c, fontSize: 10, fontWeight: FontWeight.w900)),
        ],
      ),
    );
  }

  Widget _liveStrip(WifiConnectionState s) {
    final heading = s.gpsHeading;
    final sol = s.gpsCarrier ?? "-";
    final hAcc = s.gpsAccuracy != null ? (s.gpsAccuracy! / 1000.0) : null;
    final sv = s.gpsSatellites;
    final bat = s.batteryPercent;
    final speed = s.gpsSpeedMps ?? 0;
    final wp = s.navWpIndex;
    final wpT = s.navWpTotal;
    final navState = s.navState ?? "—";
    return Container(
      padding: const EdgeInsets.fromLTRB(10, 6, 10, 6),
      decoration: const BoxDecoration(border: Border(bottom: BorderSide(color: Colors.white12))),
      child: Row(
        children: [
          _miniStat("head", heading != null ? "${heading.toStringAsFixed(0)}°" : "—"),
          _miniStat("sol", sol, color: sol == "fixed" ? Colors.greenAccent : sol == "float" ? Colors.amber : Colors.redAccent),
          _miniStat("hAcc", hAcc != null ? "${(hAcc * 1000).toStringAsFixed(0)}mm" : "—"),
          _miniStat("sv", sv?.toString() ?? "—"),
          _miniStat("bat", bat != null ? "$bat%" : "—"),
          _miniStat("v", "${speed.toStringAsFixed(2)}"),
          _miniStat("nav", navState, color: navState == "RUNNING" || navState == "APPROACHING" ? Colors.greenAccent : Colors.white70),
          if (wp != null && wpT != null) _miniStat("wp", "$wp/$wpT"),
        ],
      ),
    );
  }

  Widget _miniStat(String label, String value, {Color? color}) {
    return Padding(
      padding: const EdgeInsets.only(right: 10),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(label, style: const TextStyle(color: Colors.white38, fontSize: 8, fontWeight: FontWeight.w700, letterSpacing: 0.5)),
          Text(value, style: TextStyle(color: color ?? Colors.white, fontSize: 12, fontWeight: FontWeight.w900, fontFamily: "monospace")),
        ],
      ),
    );
  }

  Widget _tabs() {
    const labels = ["HEAD", "ODO", "STAN", "DRIVE", "RTK", "MAP", "RAW"];
    return Container(
      height: 40,
      decoration: const BoxDecoration(border: Border(bottom: BorderSide(color: Colors.white12))),
      child: ListView.builder(
        scrollDirection: Axis.horizontal,
        itemCount: labels.length,
        itemBuilder: (_, i) {
          final selected = _tab == i;
          return InkWell(
            onTap: () => setState(() => _tab = i),
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              decoration: BoxDecoration(
                border: Border(
                  bottom: BorderSide(color: selected ? Colors.white : Colors.transparent, width: 2),
                ),
              ),
              alignment: Alignment.center,
              child: Text(
                labels[i],
                style: TextStyle(
                  color: selected ? Colors.white : Colors.white54,
                  fontSize: 12,
                  fontWeight: FontWeight.w900,
                  letterSpacing: 0.5,
                ),
              ),
            ),
          );
        },
      ),
    );
  }

  List<Widget> _tabContent(WifiConnectionState s) {
    switch (_tab) {
      case 0: return _tabHeading(s);
      case 1: return _tabOdometry(s);
      case 2: return _tabStanley(s);
      case 3: return _tabDrive(s);
      case 4: return _tabRtk(s);
      case 5: return _tabMap(s);
      case 6: return _tabRaw(s);
      default: return [];
    }
  }

  // === TAB 0: HEADING ===
  List<Widget> _tabHeading(WifiConnectionState s) {
    return [
      _section("Seed heading (ручной ввод с компаса iPhone)"),
      _h2("Тест: задать heading равным текущему курсу по iPhone — затем проехать 1м и замерить боковое смещение Δx. Если Δx > 5см, значит heading seed неточный."),
      _btnRow([
        _testBtn("SET 0° N", () => _sendCmd("SET_HEADING,0", resultKey: "h_set_0")),
        _testBtn("SET 90° E", () => _sendCmd("SET_HEADING,90", resultKey: "h_set_90")),
        _testBtn("SET 180° S", () => _sendCmd("SET_HEADING,180", resultKey: "h_set_180")),
        _testBtn("SET 270° W", () => _sendCmd("SET_HEADING,270", resultKey: "h_set_270")),
      ]),
      _h2("Произвольный heading:"),
      Row(
        children: [
          Expanded(
            child: TextField(
              controller: _headingCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 14, fontFamily: "monospace"),
              decoration: _inputDeco(hint: "heading в градусах, например 124.5"),
            ),
          ),
          const SizedBox(width: 8),
          _testBtn("SET", () {
            final d = double.tryParse(_headingCtrl.text);
            if (d == null) { _toast("Введи число"); return; }
            _sendCmd("SET_HEADING,${d.toStringAsFixed(2)}", resultKey: "h_custom");
          }),
        ],
      ),
      const SizedBox(height: 8),
      _btnRow([
        _testBtn("= current heading", () {
          final h = s.gpsHeading;
          if (h == null) { _toast("Нет heading"); return; }
          _headingCtrl.text = h.toStringAsFixed(1);
          _sendCmd("SET_HEADING,${h.toStringAsFixed(2)}", resultKey: "h_current");
        }, accent: Colors.amber),
      ]),
      _resultBox("h_set_0"),
      _resultBox("h_set_90"),
      _resultBox("h_set_180"),
      _resultBox("h_set_270"),
      _resultBox("h_custom"),
      _resultBox("h_current"),

      const SizedBox(height: 16),
      _section("Heading probe: проехать 1м, замерить drift"),
      _h2("SET_HEADING,90 (E) → drive 1m east → рулеткой замерь Δx (север) и Δy (восток). Если heading правильный, Δy ≈ 1.0м, Δx ≈ 0. Введи результат:"),
      Row(
        children: [
          Expanded(
            child: TextField(
              controller: _distCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 14, fontFamily: "monospace"),
              decoration: _inputDeco(hint: "метры, например 1.00"),
            ),
          ),
          const SizedBox(width: 8),
          _testBtn("Drive 1m N (test heading)", () async {
            _sendCmd("SET_HEADING,0", resultKey: "h_probe_setup");
            await Future.delayed(const Duration(milliseconds: 300));
            final m = double.tryParse(_distCtrl.text) ?? 1.0;
            await _driveTo(meters: m, compassDeg: 0, key: "h_probe_n");
          }, accent: Colors.green),
        ],
      ),
      _resultBox("h_probe_n"),
      _h2("Замер: Δx (east), Δy (north), heading_err_deg = atan2(Δx, Δy)*180/π"),
      Row(
        children: [
          Expanded(
            child: TextField(
              controller: _perimLCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 13),
              decoration: _inputDeco(hint: "Δx (east), м"),
            ),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: TextField(
              controller: _perimWCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 13),
              decoration: _inputDeco(hint: "Δy (north), м"),
            ),
          ),
        ],
      ),
      _testBtn("Compute heading error", () {
        final dx = double.tryParse(_perimLCtrl.text);
        final dy = double.tryParse(_perimWCtrl.text);
        if (dx == null || dy == null || dy == 0) { _toast("Введи Δx и Δy"); return; }
        final errDeg = math.atan2(dx, dy) * 180 / math.pi;
        final hyp = math.sqrt(dx * dx + dy * dy);
        _setResult("h_err",
            "heading_err=${errDeg.toStringAsFixed(2)}°  travelled=${hyp.toStringAsFixed(3)}м  "
            "→ компенсация: SET_HEADING будет сдвигать на -${errDeg.toStringAsFixed(2)}°");
        _toast("heading_err=${errDeg.toStringAsFixed(2)}°");
      }, accent: Colors.amber),
      _resultBox("h_err"),
    ];
  }

  // === TAB 1: ODOMETRY ===
  List<Widget> _tabOdometry(WifiConnectionState s) {
    return [
      _section("Wheel circumference (ROVER_WHEEL_CIRCUM_M)"),
      _h2("Проедь 1м (или больше) по прямой с известным heading. Замерь рулеткой. Ratio = факт / заданное. kCircumNew = 0.6 * ratio."),
      Row(
        children: [
          Expanded(
            child: TextField(
              controller: _distCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 14, fontFamily: "monospace"),
              decoration: _inputDeco(hint: "дистанция, м (1.0 = 1м)"),
            ),
          ),
          const SizedBox(width: 8),
          _testBtn("Drive N", () async {
            final m = double.tryParse(_distCtrl.text) ?? 1.0;
            await _driveTo(meters: m, compassDeg: 0, key: "odo_drive");
          }, accent: Colors.green),
        ],
      ),
      _resultBox("odo_drive"),
      _h2("Замер: фактическая дистанция (рулеткой):"),
      Row(
        children: [
          Expanded(
            child: TextField(
              controller: _perimLCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 13),
              decoration: _inputDeco(hint: "факт. дистанция, м"),
            ),
          ),
          const SizedBox(width: 8),
          _testBtn("Compute circum", () {
            final want = double.tryParse(_distCtrl.text);
            final fact = double.tryParse(_perimLCtrl.text);
            if (want == null || fact == null || want == 0) { _toast("Введи заданную и факт"); return; }
            final ratio = fact / want;
            final newCircum = 0.6 * ratio;
            _setResult("odo_circum",
                "ratio=${ratio.toStringAsFixed(3)} → ROVER_WHEEL_CIRCUM_M = ${newCircum.toStringAsFixed(3)} м");
            _toast("ratio=${ratio.toStringAsFixed(3)} → new circum=${newCircum.toStringAsFixed(3)}");
          }, accent: Colors.amber),
        ],
      ),
      _resultBox("odo_circum"),

      const SizedBox(height: 16),
      _section("kMeasToMps (speed calibration)"),
      _h2("Это скорость. Проедь 1м замерив время, и сравни с тем, что показывает телеметрия speed_mps."),
      _h2("Сейчас в прошивке: kMeasToMps = 0.0017 (приблизительно). Нужно измерить реальную скорость hoverboard-фидбэка и подобрать коэффициент."),
      _h2("Телеметрия: смотри speed_mps в live strip / в TEL строке. Проедь 1м — замерь время t (сек). expected speed = 1/t м/с."),
      _h2("Если телеметрия показывает speed=0.25, а факт 0.18 (1м за 5.5сек), то ratio = 0.18/0.25 = 0.72, новый kMeasToMps = 0.0017 * 0.72."),
      _h2("Простой способ: drive 1м с замером времени + чтением телеметрии speed_mps."),
      _testBtn("Drive 1m N (замерь время)", () async {
        _sendCmd("SET_HEADING,0");
        await Future.delayed(const Duration(milliseconds: 200));
        await _driveTo(meters: 1.0, compassDeg: 0, key: "odo_speed");
      }, accent: Colors.green),
      _resultBox("odo_speed"),
      _h2("Введи: t_факт (время 1м, сек), speed_mps (что показала телеметрия):"),
      Row(
        children: [
          Expanded(
            child: TextField(
              controller: _perimLCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 13),
              decoration: _inputDeco(hint: "t_факт, сек (например 4.0)"),
            ),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: TextField(
              controller: _perimWCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 13),
              decoration: _inputDeco(hint: "speed_mps из телеметрии"),
            ),
          ),
        ],
      ),
      _testBtn("Compute kMeasToMps", () {
        final t = double.tryParse(_perimLCtrl.text);
        final sm = double.tryParse(_perimWCtrl.text);
        if (t == null || sm == null || t == 0 || sm == 0) { _toast("Введи t и speed_mps"); return; }
        final factSpeed = 1.0 / t;
        final ratio = factSpeed / sm;
        final newK = 0.0017 * ratio;
        _setResult("odo_kMeasToMps",
            "факт=${factSpeed.toStringAsFixed(3)} м/с, телеметрия=${sm.toStringAsFixed(3)} → "
            "ratio=${ratio.toStringAsFixed(3)} → kMeasToMps = ${newK.toStringAsFixed(5)}");
        _toast("kMeasToMps = ${newK.toStringAsFixed(5)}");
      }, accent: Colors.amber),
      _resultBox("odo_kMeasToMps"),
    ];
  }

  // === TAB 2: STANLEY ===
  List<Widget> _tabStanley(WifiConnectionState s) {
    return [
      _section("Heading error response"),
      _h2("SET_HEADING на 30/60/90° от фактического курса → drive 1м → замерь боковое смещение. Проверяет, насколько быстро Stanley выправляет heading."),
      _btnRow([
        _testBtn("+30° err drive 1m", () async {
          _sendCmd("SET_HEADING,30");
          await Future.delayed(const Duration(milliseconds: 200));
          await _driveTo(meters: 1.0, compassDeg: 0, key: "stan_30");
        }),
        _testBtn("+60° err drive 1m", () async {
          _sendCmd("SET_HEADING,60");
          await Future.delayed(const Duration(milliseconds: 200));
          await _driveTo(meters: 1.0, compassDeg: 0, key: "stan_60");
        }),
        _testBtn("+90° err drive 1m", () async {
          _sendCmd("SET_HEADING,90");
          await Future.delayed(const Duration(milliseconds: 200));
          await _driveTo(meters: 1.0, compassDeg: 0, key: "stan_90");
        }),
      ]),
      _resultBox("stan_30"),
      _resultBox("stan_60"),
      _resultBox("stan_90"),

      const SizedBox(height: 16),
      _section("Crosstrack recovery"),
      _h2("Сдвинуть ровер на 0.5м вбок от прямой, потом drive 2м по прямой. Проверяет crosstrack guard."),
      _h2("(вручную: подвинь ровер, потом жми drive 2m N)"),
      _btnRow([
        _testBtn("Drive 2m N", () async {
          _sendCmd("SET_HEADING,0");
          await Future.delayed(const Duration(milliseconds: 200));
          await _driveTo(meters: 2.0, compassDeg: 0, key: "ct_2m");
        }, accent: Colors.green),
      ]),
      _resultBox("ct_2m"),

      const SizedBox(height: 16),
      _section("NAV commands (для отладки)"),
      _btnRow([
        _testBtn("START", () => _sendNav("START"), accent: Colors.green),
        _testBtn("PAUSE", () => _sendNav("PAUSE"), accent: Colors.amber),
        _testBtn("RESUME", () => _sendNav("RESUME"), accent: Colors.amber),
        _testBtn("STOP", () => _sendNav("STOP"), accent: Colors.red),
      ]),

      const SizedBox(height: 16),
      _section("Live NAV state (смотри live strip + log)"),
      _h2("NAV state / wp / crossTrack / headingErr показываются в live strip сверху. Если что-то странное — смотри rxLog внизу экрана."),
    ];
  }

  // === TAB 3: DRIVE ===
  List<Widget> _tabDrive(WifiConnectionState s) {
    return [
      _section("Drive N meters in direction (one shot)"),
      _h2("Дистанция:"),
      TextField(
        controller: _distCtrl,
        keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
        style: const TextStyle(color: Colors.white, fontSize: 14, fontFamily: "monospace"),
        decoration: _inputDeco(hint: "метры (можно отрицательные чтобы ехать назад)"),
      ),
      const SizedBox(height: 8),
      _h2("Направление (тап = проехать, лонг-тап = проехать и вернуться):"),
      _compassPad(busy: _busy),
      const SizedBox(height: 4),
      const Text("compassDeg: 0=N, 90=E, 180=S, 270=W, 45=NE, и т.п.",
          style: TextStyle(color: Colors.white38, fontSize: 10)),

      const SizedBox(height: 16),
      _section("Drive туда-обратно (для проверки heading)"),
      _h2("Проехать N м, вернуться. Замерь стартовую и финальную позицию. Если heading driftнул, финальная точка будет смещена."),
      _btnRow([
        _testBtn("1м N ↩", () => _driveToAndBack(meters: 1, compassDeg: 0, key: "db_1n")),
        _testBtn("1м E ↩", () => _driveToAndBack(meters: 1, compassDeg: 90, key: "db_1e")),
        _testBtn("2м N ↩", () => _driveToAndBack(meters: 2, compassDeg: 0, key: "db_2n")),
        _testBtn("2м E ↩", () => _driveToAndBack(meters: 2, compassDeg: 90, key: "db_2e")),
        _testBtn("3м N ↩", () => _driveToAndBack(meters: 3, compassDeg: 0, key: "db_3n")),
        _testBtn("5м N ↩", () => _driveToAndBack(meters: 5, compassDeg: 0, key: "db_5n")),
      ]),
      _resultBox("db_1n"),
      _resultBox("db_1e"),
      _resultBox("db_2n"),
      _resultBox("db_2e"),
      _resultBox("db_3n"),
      _resultBox("db_5n"),

      const SizedBox(height: 16),
      _section("Perimeter (прямоугольник)"),
      Row(
        children: [
          Expanded(
            child: TextField(
              controller: _perimLCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 13),
              decoration: _inputDeco(hint: "L, м"),
            ),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: TextField(
              controller: _perimWCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 13),
              decoration: _inputDeco(hint: "W, м"),
            ),
          ),
        ],
      ),
      const SizedBox(height: 8),
      _btnRow([
        _testBtn("L×W", () {
          final l = double.tryParse(_perimLCtrl.text) ?? 1.0;
          final w = double.tryParse(_perimWCtrl.text) ?? 1.0;
          _drivePerimeter(L: l, W: w, andBack: false, key: "perim_lw");
        }),
        _testBtn("L×W ↩", () {
          final l = double.tryParse(_perimLCtrl.text) ?? 1.0;
          final w = double.tryParse(_perimWCtrl.text) ?? 1.0;
          _drivePerimeter(L: l, W: w, andBack: true, key: "perim_lw_back");
        }),
        _testBtn("1×1", () => _drivePerimeter(L: 1, W: 1, andBack: false, key: "perim_1x1")),
        _testBtn("1×1 ↩", () => _drivePerimeter(L: 1, W: 1, andBack: true, key: "perim_1x1_back")),
        _testBtn("2×2", () => _drivePerimeter(L: 2, W: 2, andBack: false, key: "perim_2x2")),
        _testBtn("2×2 ↩", () => _drivePerimeter(L: 2, W: 2, andBack: true, key: "perim_2x2_back")),
        _testBtn("3×1", () => _drivePerimeter(L: 3, W: 1, andBack: false, key: "perim_3x1")),
        _testBtn("3×1 ↩", () => _drivePerimeter(L: 3, W: 1, andBack: true, key: "perim_3x1_back")),
        _testBtn("5×0.5", () => _drivePerimeter(L: 5, W: 0.5, andBack: true, key: "perim_5x05")),
      ]),
      _resultBox("perim_1x1"),
      _resultBox("perim_1x1_back"),
      _resultBox("perim_2x2"),
      _resultBox("perim_2x2_back"),
      _resultBox("perim_3x1"),
      _resultBox("perim_3x1_back"),
      _resultBox("perim_5x05"),
      _resultBox("perim_lw"),
      _resultBox("perim_lw_back"),

      const SizedBox(height: 16),
      _section("Joystick (ручной drive, без логирования)"),
      _h2("Тяни стик вверх = ехать вперёд. Вниз = назад. Влево/вправо = разворот. Отпусти = стоп."),
      Center(child: _joystick()),
    ];
  }

  // === TAB 4: RTK ===
  List<Widget> _tabRtk(WifiConnectionState s) {
    return [
      _section("RTK live readouts (на покое)"),
      _h2("В покое должны быть: sol=fixed, hAcc ≤ 2см, pvtAge < 200ms, sv ≥ 10, pDOP < 4. Смотри live strip + TEL строку в логе ниже."),
      _kv("sol (carrier)", s.gpsCarrier ?? "—",
          color: s.gpsCarrier == "fixed" ? Colors.greenAccent : s.gpsCarrier == "float" ? Colors.amber : Colors.redAccent),
      _kv("sv (satellites)", s.gpsSatellites?.toString() ?? "—"),
      _kv("hAcc (горизонт. точность)", s.gpsAccuracy != null ? "${s.gpsAccuracy} mm" : "—"),
      _kv("vAcc (вертик. точность)", s.gpsVAccuracy != null ? "${s.gpsVAccuracy} mm" : "—"),
      _kv("pDop", s.gpsPDop != null ? s.gpsPDop!.toStringAsFixed(2) : "—"),
      _kv("pvtAge", s.gpsAgeMs != null ? "${s.gpsAgeMs} ms" : "—"),
      _kv("diff (diffcorr applied)", s.gpsDiff == true ? "1" : "0"),
      _kv("fixType (raw u-blox)", s.gpsFixType?.toString() ?? "—"),
      _kv("rtcmAge (trans)", s.rtcmTransportAgeMs != null ? "${s.rtcmTransportAgeMs} ms" : "—"),
      _kv("rtcmAge (F9P decoded)", s.rtcmF9pAgeMs != null ? "${s.rtcmF9pAgeMs} ms" : "—"),
      _kv("rtcm source", s.rtcmSource ?? "—"),
      _kv("rtcm last type", s.rtcmLastType?.toString() ?? "—"),
      _kv("rtcm messages / crcFail", "${s.rtcmF9pMessages ?? '—'} / ${s.rtcmCrcFail ?? '—'}"),

      const SizedBox(height: 16),
      _section("RTK commands"),
      _btnRow([
        _testBtn("PING", () => _sendCmd("PING", resultKey: "rtk_ping")),
        _testBtn("re-arm RTCM", () => _sendCmd("SET_HEADING,0", resultKey: "rtk_arm")), // no-op, just placeholder
      ]),
      _h2("Если RTCM_F9P_AGE не падает ниже 1000ms, значит F9P не декодирует RTCM — проверить TX-кабель rover → F9P."),

      const SizedBox(height: 16),
      _section("Покрытие (attachment)"),
      _btnRow([
        _testBtn("ATTACH ON", () => _sendCmd("ATTACHMENT_ON", resultKey: "attach_on")),
        _testBtn("ATTACH OFF", () => _sendCmd("ATTACHMENT_OFF", resultKey: "attach_off")),
        _testBtn("MOUNT ON", () => _sendCmd("MOUNT_ON", resultKey: "mount_on")),
        _testBtn("MOUNT OFF", () => _sendCmd("MOUNT_OFF", resultKey: "mount_off")),
      ]),
    ];
  }

  // === TAB 6: MAP ===
  List<Widget> _tabMap(WifiConnectionState s) {
    // capture origin on first build
    if (_mapOriginLat == null && s.gpsLat != null) {
      _mapOriginLat = s.gpsLat;
      _mapOriginLon = s.gpsLon;
    }
    // compute current (x,y) from latest lat/lon
    if (_mapOriginLat != null && _mapOriginLon != null && s.gpsLat != null && s.gpsLon != null) {
      final geo = GpsDisplayGeometry(originLat: _mapOriginLat!, originLon: _mapOriginLon!);
      final p = geo.toLocal(s.gpsLat!, s.gpsLon!);
      _lastRobotX = p.x;
      _lastRobotY = p.y;
      // append to trail (cap to 200 points)
      if (_trail.isEmpty ||
          (_trail.last.x - p.x).abs() > 0.05 ||
          (_trail.last.y - p.y).abs() > 0.05) {
        _trail.add(_MapPoint(p.x, p.y));
        if (_trail.length > 200) _trail.removeAt(0);
      }
    }

    return [
      _section("Live map (GPS → local meters)"),
      _h2("Синяя сетка = 1м. Зелёная точка = ровер. Стрелка = heading. Синие линии = planned route. Красные точки = пройденный трейл."),

      Row(
        children: [
          const Text("scale (px/m):",
              style: TextStyle(color: Colors.white70, fontSize: 11)),
          const SizedBox(width: 6),
          SizedBox(
            width: 60,
            child: TextField(
              controller: _mapScaleCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(color: Colors.white, fontSize: 12),
              decoration: _inputDeco(),
              onChanged: (_) => setState(() {}),
            ),
          ),
          const SizedBox(width: 12),
          _testBtn(_followRover ? "Following ON" : "Following OFF", () {
            setState(() => _followRover = !_followRover);
          }, accent: _followRover ? Colors.green : Colors.white54),
        ],
      ),
      const SizedBox(height: 6),

      // the map
      Container(
        height: 380,
        decoration: BoxDecoration(
          color: Colors.black.withOpacity(0.6),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.white24),
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(8),
          child: CustomPaint(
            painter: _MapFullPainter(
              robotX: _lastRobotX,
              robotY: _lastRobotY,
              headingDeg: s.gpsHeading,
              scalePxPerM: double.tryParse(_mapScaleCtrl.text) ?? 50.0,
              follow: _followRover,
              plannedRoute: _plannedRoute,
              trail: _trail,
            ),
            child: Container(),
          ),
        ),
      ),
      const SizedBox(height: 6),
      Text(
        "Ровер: x=${_lastRobotX?.toStringAsFixed(2) ?? '—'} y=${_lastRobotY?.toStringAsFixed(2) ?? '—'}  heading=${s.gpsHeading?.toStringAsFixed(1) ?? '—'}°  sol=${s.gpsCarrier ?? '—'}",
        style: const TextStyle(color: Colors.white54, fontSize: 11, fontFamily: "monospace"),
      ),

      const SizedBox(height: 16),
      _section("Build planned route (preview)"),
      _h2("Это только превью — на ровер не отправится. Используй DRIVE tab чтобы реально поехать."),

      Wrap(
        spacing: 6,
        runSpacing: 6,
        children: [
          _testBtn("Clear preview", () {
            setState(() {
              _plannedRoute.clear();
            });
            _toast("preview cleared");
          }, accent: Colors.redAccent),
          _testBtn("Rectangle L×W", () {
            final l = double.tryParse(_perimLCtrl.text) ?? 1.0;
            final w = double.tryParse(_perimWCtrl.text) ?? 1.0;
            setState(() {
              _plannedRoute
                ..clear()
                ..add(const _MapPoint(0, 0))
                ..add(_MapPoint(l, 0))
                ..add(_MapPoint(l, w))
                ..add(_MapPoint(0, w))
                ..add(const _MapPoint(0, 0));
            });
            _toast("rectangle ${l}×${w}м");
          }, accent: Colors.amber),
          _testBtn("Line N", () {
            final m = double.tryParse(_distCtrl.text) ?? 1.0;
            setState(() {
              _plannedRoute
                ..clear()
                ..add(const _MapPoint(0, 0))
                ..add(_MapPoint(0, m));
            });
            _toast("line ${m}м north");
          }, accent: Colors.amber),
          _testBtn("Line E", () {
            final m = double.tryParse(_distCtrl.text) ?? 1.0;
            setState(() {
              _plannedRoute
                ..clear()
                ..add(const _MapPoint(0, 0))
                ..add(_MapPoint(m, 0));
            });
            _toast("line ${m}м east");
          }, accent: Colors.amber),
          _testBtn("There and back", () {
            final m = double.tryParse(_distCtrl.text) ?? 1.0;
            setState(() {
              _plannedRoute
                ..clear()
                ..add(const _MapPoint(0, 0))
                ..add(_MapPoint(0, m))
                ..add(const _MapPoint(0, 0));
            });
            _toast("there and back ${m}м");
          }, accent: Colors.amber),
          _testBtn("Reset origin", () {
            setState(() {
              _mapOriginLat = s.gpsLat;
              _mapOriginLon = s.gpsLon;
              _trail.clear();
              _lastRobotX = 0;
              _lastRobotY = 0;
            });
            _toast("origin reset");
          }),
        ],
      ),

      const SizedBox(height: 16),
      _section("Clear trail"),
      _testBtn("Clear trail", () {
        setState(() {
          _trail.clear();
        });
        _toast("trail cleared");
      }, accent: Colors.redAccent),

      const SizedBox(height: 16),
      _section("Tips"),
      _h2("• Красные точки = трейл ровера (видно, реально ли он ехал по маршруту)."),
      _h2("• Синие линии + белые точки = planned route preview."),
      _h2("• Зелёная точка + стрелка = ровер + heading."),
      _h2("• Сетка = 1м клетка. 50px/m = 5м ширина."),
      _h2("• Following ON: камера следит за ровером."),
    ];
  }

  // === TAB 7: RAW ===
  List<Widget> _tabRaw(WifiConnectionState s) {
    return [
      _section("Произвольная команда"),
      _h2("Шли что угодно: SET_HEADING,X, ROUTE_BEGIN,3,55.7,37.6, NAV_START, и т.п."),
      TextField(
        controller: _rawCmdCtrl,
        style: const TextStyle(color: Colors.white, fontSize: 13, fontFamily: "monospace"),
        decoration: _inputDeco(hint: "команда"),
        onSubmitted: (v) {
          if (v.trim().isNotEmpty) _sendCmd(v.trim(), resultKey: "raw_last");
        },
      ),
      const SizedBox(height: 6),
      Row(
        children: [
          _testBtn("Send", () {
            final c = _rawCmdCtrl.text.trim();
            if (c.isEmpty) { _toast("Введи команду"); return; }
            _sendCmd(c, resultKey: "raw_last");
          }, accent: Colors.green),
          const SizedBox(width: 6),
          _testBtn("PING", () => _sendCmd("PING", resultKey: "raw_last")),
        ],
      ),
      _resultBox("raw_last"),

      const SizedBox(height: 16),
      _section("rxLog (входящие от ровера, свежие сверху)"),
      _h2("Если что-то странное — смотри сюда. TEL/NAV/GPSDBG/RTCM/IMU/MOTOR/OK/ERR — всё в одном потоке."),
      Container(
        height: 380,
        padding: const EdgeInsets.all(8),
        decoration: BoxDecoration(
          color: Colors.black.withOpacity(0.6),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.white12),
        ),
        child: ListView.builder(
          itemCount: s.rxLog.length,
          itemBuilder: (_, i) {
            final line = s.rxLog[s.rxLog.length - 1 - i];
            Color color = Colors.white60;
            if (line.startsWith("ERR,")) color = Colors.redAccent;
            if (line.startsWith("OK")) color = Colors.greenAccent;
            if (line.startsWith("NAV,")) color = Colors.amberAccent;
            if (line.startsWith("TEL,")) color = Colors.white;
            return Text(line,
                style: TextStyle(color: color, fontSize: 10, fontFamily: "monospace"));
          },
        ),
      ),

      const SizedBox(height: 16),
      _section("Battery / Motor live"),
      _kv("Battery", s.batteryPercent != null ? "${s.batteryPercent}%" : (s.motorBatteryRaw != null ? "${s.motorBatteryRaw! / 100.0}V" : "—")),
      _kv("Motor FB", s.motorFeedback == true ? "1" : "0"),
      _kv("Left PWM / Right PWM", "${s.motorLeft ?? '—'} / ${s.motorRight ?? '—'}"),
      _kv("speedL meas / speedR meas", "${s.motorSpeedLeft ?? '—'} / ${s.motorSpeedRight ?? '—'}"),
      _kv("IMU yaw / age / fresh", s.imuYaw != null
          ? "${s.imuYaw!.toStringAsFixed(1)}° / ${s.imuAgeMs ?? '—'}ms / ${s.imuFresh == true ? '1' : '0'}"
          : "—"),
    ];
  }

  // --- UI building blocks ---

  Widget _section(String title) {
    return Container(
      margin: const EdgeInsets.only(top: 8, bottom: 4),
      padding: const EdgeInsets.fromLTRB(10, 6, 10, 6),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.06),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.white24),
      ),
      child: Text(
        title,
        style: const TextStyle(color: Colors.white, fontSize: 13, fontWeight: FontWeight.w900, letterSpacing: 0.4),
      ),
    );
  }

  Widget _h2(String s) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Text(s, style: const TextStyle(color: Colors.white70, fontSize: 12, height: 1.3)),
    );
  }

  Widget _btnRow(List<Widget> children) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Wrap(spacing: 6, runSpacing: 6, children: children),
    );
  }

  Widget _testBtn(String label, VoidCallback onPressed, {Color? accent}) {
    return InkWell(
      borderRadius: BorderRadius.circular(8),
      onTap: _busy ? null : onPressed,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(8),
          color: (accent ?? Colors.white).withOpacity(0.10),
          border: Border.all(color: (accent ?? Colors.white).withOpacity(0.4)),
        ),
        child: Text(
          label,
          style: TextStyle(
            color: accent ?? Colors.white,
            fontSize: 11,
            fontWeight: FontWeight.w900,
            letterSpacing: 0.3,
          ),
        ),
      ),
    );
  }

  Widget _resultBox(String key) {
    final v = _results[key];
    if (v == null) return const SizedBox.shrink();
    return Container(
      margin: const EdgeInsets.only(top: 2),
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.amber.withOpacity(0.10),
        borderRadius: BorderRadius.circular(6),
        border: Border.all(color: Colors.amber.withOpacity(0.3)),
      ),
      child: Text(v, style: const TextStyle(color: Colors.amberAccent, fontSize: 11, fontFamily: "monospace")),
    );
  }

  Widget _compassPad({required bool busy}) {
    Widget btn(String label, double? deg) {
      if (deg == null) {
        return Center(child: Text(label, style: const TextStyle(color: Colors.white30, fontSize: 16)));
      }
      return GestureDetector(
        onTap: busy ? null : () async {
          final m = double.tryParse(_distCtrl.text) ?? 1.0;
          await _driveTo(meters: m, compassDeg: deg, key: "drive_$deg");
        },
        onLongPress: busy ? null : () async {
          final m = double.tryParse(_distCtrl.text) ?? 1.0;
          await _driveToAndBack(meters: m, compassDeg: deg, key: "drive_back_$deg");
        },
        child: Container(
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(8),
            color: Colors.white.withOpacity(0.06),
            border: Border.all(color: Colors.white24),
          ),
          alignment: Alignment.center,
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Text(label, style: const TextStyle(color: Colors.white, fontSize: 14, fontWeight: FontWeight.w900)),
              Text("${deg.toStringAsFixed(0)}°", style: const TextStyle(color: Colors.white54, fontSize: 9)),
            ],
          ),
        ),
      );
    }
    final dirs = [
      ["NW", 315.0], ["N", 0.0], ["NE", 45.0],
      ["W", 270.0], ["•", null], ["E", 90.0],
      ["SW", 225.0], ["S", 180.0], ["SE", 135.0],
    ];
    return Container(
      height: 200,
      child: GridView.count(
        crossAxisCount: 3,
        childAspectRatio: 1.2,
        mainAxisSpacing: 6,
        crossAxisSpacing: 6,
        physics: const NeverScrollableScrollPhysics(),
        children: dirs.map<Widget>((d) => btn(d[0] as String, d[1] as double?)).toList(),
      ),
    );
  }

  Widget _joystick() {
    return _Joystick(
      onChanged: (lx, ly) {
        // lx: -1 (left) .. +1 (right), ly: -1 (up/forward) .. +1 (down/back)
        final fwd = (ly * 60).round();
        final turn = (lx * 60).round();
        _sendMove((fwd + turn).clamp(-70, 70), (fwd - turn).clamp(-70, 70));
      },
      onReleased: () => _sendMove(0, 0),
    );
  }

  InputDecoration _inputDeco({String? hint}) {
    return InputDecoration(
      isDense: true,
      hintText: hint,
      hintStyle: const TextStyle(color: Colors.white24, fontSize: 12),
      contentPadding: const EdgeInsets.symmetric(horizontal: 8, vertical: 8),
      border: OutlineInputBorder(
        borderRadius: BorderRadius.circular(8),
        borderSide: const BorderSide(color: Colors.white24),
      ),
      enabledBorder: OutlineInputBorder(
        borderRadius: BorderRadius.circular(8),
        borderSide: const BorderSide(color: Colors.white24),
      ),
    );
  }

  Widget _kv(String k, String v, {Color? color}) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          SizedBox(
            width: 160,
            child: Text(k, style: const TextStyle(color: Colors.white54, fontSize: 11)),
          ),
          Expanded(
            child: Text(
              v,
              style: TextStyle(
                color: color ?? Colors.white,
                fontSize: 12,
                fontWeight: FontWeight.w700,
                fontFamily: "monospace",
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _toastBar(String msg) {
    return Container(
      width: double.infinity,
      color: Colors.black87,
      padding: const EdgeInsets.symmetric(vertical: 10, horizontal: 12),
      child: Text(msg, style: const TextStyle(color: Colors.white, fontSize: 12, fontWeight: FontWeight.w700), textAlign: TextAlign.center),
    );
  }
}

// =============================================================
// Joystick
// =============================================================
class _Joystick extends StatefulWidget {
  final void Function(double lx, double ly) onChanged;
  final VoidCallback onReleased;
  const _Joystick({required this.onChanged, required this.onReleased});

  @override
  State<_Joystick> createState() => _JoystickState();
}

class _JoystickState extends State<_Joystick> {
  Offset _knob = const Offset(100, 100);
  static const double _size = 200.0;
  static const double _knobR = 30.0;
  static const double _maxR = 80.0;

  void _update(Offset p) {
    final center = const Offset(_size / 2, _size / 2);
    final d = p - center;
    final dist = d.distance;
    Offset clamped = dist > _maxR
        ? center + Offset(d.dx * _maxR / dist, d.dy * _maxR / dist)
        : p;
    setState(() => _knob = clamped);
    widget.onChanged((clamped.dx - center.dx) / _maxR, (clamped.dy - center.dy) / _maxR);
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onPanStart: (d) => _update(d.localPosition),
      onPanUpdate: (d) => _update(d.localPosition),
      onPanEnd: (_) {
        setState(() => _knob = const Offset(_size / 2, _size / 2));
        widget.onReleased();
      },
      child: Container(
        width: _size,
        height: _size,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          color: Colors.white.withOpacity(0.05),
          border: Border.all(color: Colors.white24),
        ),
        child: Stack(
          children: [
            Center(child: Container(width: 4, height: 4, decoration: const BoxDecoration(color: Colors.white24, shape: BoxShape.circle))),
            Positioned(
              left: _knob.dx - _knobR,
              top: _knob.dy - _knobR,
              child: Container(
                width: _knobR * 2,
                height: _knobR * 2,
                decoration: BoxDecoration(shape: BoxShape.circle, color: Colors.white.withOpacity(0.30), border: Border.all(color: Colors.white)),
              ),
            ),
            const Positioned(top: 4, left: 0, right: 0, child: Center(child: Text("N", style: TextStyle(color: Colors.white30, fontSize: 10)))),
            const Positioned(bottom: 4, left: 0, right: 0, child: Center(child: Text("S", style: TextStyle(color: Colors.white30, fontSize: 10)))),
            const Positioned(top: 0, bottom: 0, left: 4, child: Center(child: Text("W", style: TextStyle(color: Colors.white30, fontSize: 10)))),
            const Positioned(top: 0, bottom: 0, right: 4, child: Center(child: Text("E", style: TextStyle(color: Colors.white30, fontSize: 10)))),
          ],
        ),
      ),
    );
  }
}

// =============================================================
// Map full painter — North-up grid, robot, heading arrow,
// planned route, trail
// =============================================================
class _MapPoint {
  final double x, y;
  const _MapPoint(this.x, this.y);
}

class _MapFullPainter extends CustomPainter {
  final double? robotX, robotY;
  final double? headingDeg;
  final double scalePxPerM;
  final bool follow;
  final List<_MapPoint> plannedRoute;
  final List<_MapPoint> trail;

  _MapFullPainter({
    required this.robotX,
    required this.robotY,
    required this.headingDeg,
    required this.scalePxPerM,
    required this.follow,
    required this.plannedRoute,
    required this.trail,
  });

  @override
  void paint(Canvas canvas, Size size) {
    // bg
    final bg = Paint()..color = const Color(0xFF050810);
    canvas.drawRect(Offset.zero & size, bg);

    // determine center: if follow and have robot, center on robot
    double cx = size.width / 2;
    double cy = size.height / 2;
    if (follow && robotX != null && robotY != null) {
      cx = size.width / 2 - robotX! * scalePxPerM;
      cy = size.height / 2 + robotY! * scalePxPerM; // y inverted (north up)
    }

    // grid 1m step
    final gridMain = Paint()..color = Colors.white.withOpacity(0.15)..strokeWidth = 1;
    final gridSub = Paint()..color = Colors.white.withOpacity(0.06)..strokeWidth = 0.5;
    // step in world units such that pixels are scalePxPerM * 0.5m (sub) and 1m (main)
    final halfW = size.width / 2;
    final halfH = size.height / 2;
    final xMin = (-cx) / scalePxPerM;
    final xMax = (size.width - cx) / scalePxPerM;
    final yMin = -(size.height - cy) / scalePxPerM;
    final yMax = -(-cy) / scalePxPerM; // y inverted

    // sub-grid 0.5m
    final subStep = 0.5;
    var sx = (xMin / subStep).floor() * subStep;
    while (sx <= xMax) {
      final px = cx + sx * scalePxPerM;
      canvas.drawLine(Offset(px, 0), Offset(px, size.height), gridSub);
      sx += subStep;
    }
    var sy = (yMin / subStep).floor() * subStep;
    while (sy <= yMax) {
      final py = cy - sy * scalePxPerM;
      canvas.drawLine(Offset(0, py), Offset(size.width, py), gridSub);
      sy += subStep;
    }

    // main 1m grid
    var gx = xMin.floor();
    while (gx <= xMax.ceil()) {
      final px = cx + gx * scalePxPerM;
      canvas.drawLine(Offset(px, 0), Offset(px, size.height), gridMain);
      // label
      if (gx != 0) {
        final tp = TextPainter(
          text: TextSpan(text: "${gx}m", style: const TextStyle(color: Colors.white38, fontSize: 9)),
          textDirection: TextDirection.ltr,
        )..layout();
        tp.paint(canvas, Offset(px + 2, 2));
      }
      gx++;
    }
    var gy = yMin.floor();
    while (gy <= yMax.ceil()) {
      final py = cy - gy * scalePxPerM;
      canvas.drawLine(Offset(0, py), Offset(size.width, py), gridMain);
      if (gy != 0) {
        final tp = TextPainter(
          text: TextSpan(text: "${gy}m", style: const TextStyle(color: Colors.white38, fontSize: 9)),
          textDirection: TextDirection.ltr,
        )..layout();
        tp.paint(canvas, Offset(2, py - 12));
      }
      gy++;
    }

    // axes (origin cross)
    final ax = Paint()
      ..color = Colors.white.withOpacity(0.4)
      ..strokeWidth = 1.5;
    if (cx >= 0 && cx <= size.width) canvas.drawLine(Offset(cx, 0), Offset(cx, size.height), ax);
    if (cy >= 0 && cy <= size.height) canvas.drawLine(Offset(0, cy), Offset(size.width, cy), ax);
    // origin marker
    if (cx >= 0 && cx <= size.width && cy >= 0 && cy <= size.height) {
      final om = Paint()..color = Colors.cyanAccent;
      canvas.drawCircle(Offset(cx, cy), 4, om);
      final otp = TextPainter(
        text: const TextSpan(text: "origin", style: TextStyle(color: Colors.cyanAccent, fontSize: 10, fontWeight: FontWeight.w900)),
        textDirection: TextDirection.ltr,
      )..layout();
      otp.paint(canvas, Offset(cx + 6, cy + 6));
    }

    // trail (red dots + line)
    if (trail.length > 1) {
      final trailLine = Paint()
        ..color = Colors.redAccent.withOpacity(0.6)
        ..strokeWidth = 1.5
        ..style = PaintingStyle.stroke;
      final path = Path();
      for (int i = 0; i < trail.length; i++) {
        final px = cx + trail[i].x * scalePxPerM;
        final py = cy - trail[i].y * scalePxPerM;
        if (i == 0) path.moveTo(px, py); else path.lineTo(px, py);
      }
      canvas.drawPath(path, trailLine);
      // dots
      final dot = Paint()..color = Colors.redAccent;
      for (final p in trail) {
        final px = cx + p.x * scalePxPerM;
        final py = cy - p.y * scalePxPerM;
        if (px >= -5 && px <= size.width + 5 && py >= -5 && py <= size.height + 5) {
          canvas.drawCircle(Offset(px, py), 2, dot);
        }
      }
    }

    // planned route (blue lines + white dots)
    if (plannedRoute.length > 1) {
      final planLine = Paint()
        ..color = Colors.lightBlueAccent.withOpacity(0.8)
        ..strokeWidth = 2
        ..style = PaintingStyle.stroke;
      final path = Path();
      for (int i = 0; i < plannedRoute.length; i++) {
        final px = cx + plannedRoute[i].x * scalePxPerM;
        final py = cy - plannedRoute[i].y * scalePxPerM;
        if (i == 0) path.moveTo(px, py); else path.lineTo(px, py);
      }
      canvas.drawPath(path, planLine);
      // waypoint dots
      for (int i = 0; i < plannedRoute.length; i++) {
        final p = plannedRoute[i];
        final px = cx + p.x * scalePxPerM;
        final py = cy - p.y * scalePxPerM;
        if (px >= -10 && px <= size.width + 10 && py >= -10 && py <= size.height + 10) {
          final isFirst = i == 0;
          final dot = Paint()..color = isFirst ? Colors.greenAccent : Colors.white;
          canvas.drawCircle(Offset(px, py), isFirst ? 6 : 4, dot);
          // index
          if (i > 0 && i < plannedRoute.length - 1) {
            final tp = TextPainter(
              text: TextSpan(text: "$i", style: const TextStyle(color: Colors.black, fontSize: 9, fontWeight: FontWeight.w900)),
              textDirection: TextDirection.ltr,
            )..layout();
            tp.paint(canvas, Offset(px - 3, py - 5));
          }
        }
      }
    }

    // robot at (x,y)
    if (robotX != null && robotY != null) {
      final px = cx + robotX! * scalePxPerM;
      final py = cy - robotY! * scalePxPerM;
      if (px >= -20 && px <= size.width + 20 && py >= -20 && py <= size.height + 20) {
        // body
        final rp = Paint()..color = Colors.greenAccent;
        canvas.drawCircle(Offset(px, py), 6, rp);
        // ring
        canvas.drawCircle(Offset(px, py), 9, Paint()..color = Colors.greenAccent.withOpacity(0.4)..style = PaintingStyle.stroke..strokeWidth = 1.5);
        // heading arrow
        if (headingDeg != null) {
          final rad = headingDeg! * math.pi / 180.0;
          final arrowLen = 18.0;
          final dx = math.sin(rad) * arrowLen;
          final dy = -math.cos(rad) * arrowLen;
          final ap = Paint()
            ..color = Colors.greenAccent
            ..strokeWidth = 2.5
            ..strokeCap = StrokeCap.round;
          canvas.drawLine(Offset(px, py), Offset(px + dx, py + dy), ap);
          // arrowhead
          final headLen = 5.0;
          final headAng = 0.5;
          final hx1 = px + dx - headLen * math.sin(rad - headAng);
          final hy1 = py + dy + headLen * math.cos(rad - headAng);
          final hx2 = px + dx - headLen * math.sin(rad + headAng);
          final hy2 = py + dy + headLen * math.cos(rad + headAng);
          canvas.drawLine(Offset(px + dx, py + dy), Offset(hx1, hy1), ap);
          canvas.drawLine(Offset(px + dx, py + dy), Offset(hx2, hy2), ap);
        }
      }
    }

    // compass rose top-right
    final compCx = size.width - 28.0;
    final compCy = 28.0;
    final ring = Paint()..color = Colors.white24..style = PaintingStyle.stroke..strokeWidth = 1;
    canvas.drawCircle(Offset(compCx, compCy), 18, ring);
    // north arrow
    final ntip = Paint()..color = Colors.redAccent;
    final nPath = Path()
      ..moveTo(compCx, compCy - 16)
      ..lineTo(compCx - 4, compCy + 4)
      ..lineTo(compCx + 4, compCy + 4)
      ..close();
    canvas.drawPath(nPath, ntip);
    final nTp = TextPainter(
      text: const TextSpan(text: "N", style: TextStyle(color: Colors.white, fontSize: 9, fontWeight: FontWeight.w900)),
      textDirection: TextDirection.ltr,
    )..layout();
    nTp.paint(canvas, Offset(compCx - 4, compCy - 30));

    // info
    final info = TextPainter(
      text: TextSpan(
        text: "scale=${scalePxPerM.toStringAsFixed(0)}px/m  follow=${follow ? "ON" : "OFF"}",
        style: const TextStyle(color: Colors.white38, fontSize: 9),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    info.paint(canvas, Offset(4, size.height - 14));
  }

  @override
  bool shouldRepaint(covariant _MapFullPainter old) =>
      old.robotX != robotX ||
      old.robotY != robotY ||
      old.headingDeg != headingDeg ||
      old.scalePxPerM != scalePxPerM ||
      old.follow != follow ||
      old.plannedRoute != plannedRoute ||
      old.trail != trail;
}
