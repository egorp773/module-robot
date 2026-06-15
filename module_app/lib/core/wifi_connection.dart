import 'dart:async';

import 'package:connectivity_plus/connectivity_plus.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

class WifiConnectionState {
  final bool isConnecting;
  final bool isConnected;
  final String? error;
  final List<String> rxLog;
  final int? batteryPercent; // Процент батареи из WebSocket (BAT_PCT)

  // GPS data
  final double? gpsLat;
  final double? gpsLon;
  final double? gpsHeading;
  final int? gpsFixType;
  final int? gpsAccuracy; // mm
  final double? gpsHeightM;
  final String? gpsCarrier;
  final bool? gpsDiff;
  final int? gpsSatellites;
  final int? gpsVAccuracy; // mm
  final double? gpsSpeedMps;
  final double? gpsPDop;
  final int? gpsAgeMs;
  final DateTime? gpsReceivedAt;
  final int? rtcmBytes;
  final int? rtcmAgeMs;
  final int? rtcmTransportAgeMs;
  final int? rtcmF9pAgeMs;
  final String? rtcmSource;
  final int? rtcmF9pMessages;
  final int? rtcmCrcFail;
  final int? rtcmLastType;

  // IMU data
  final double? imuYaw;
  final int? imuAgeMs;
  final bool? imuFresh;
  final DateTime? imuReceivedAt;

  // Navigation data
  final String? navState; // IDLE, RUNNING, PAUSED, DONE, ERROR
  final int? navWpIndex;
  final int? navWpTotal;
  final double? navDistToWp;
  final double?
      movementProgressRate; // скорость изменения расстояния до цели (м/с)
  final double? movementCrossTrack; // cross-track error (м)
  final String? movementStatus; // OK, STUCK, WRONG_DIR, APPROACHING, STARTING
  final int? motorLeft;
  final int? motorRight;
  final bool? motorFeedback;
  final int? motorSpeedLeft;
  final int? motorSpeedRight;
  final int? motorBatteryRaw;
  final int? motorBoardTempRaw;

  // Calibration data
  final String? calState; // IDLE, MEAS, VERIFY, OK, FAILED
  final double? calOffset; // IMU offset в градусах
  final double? calStdDev; // standard deviation
  final int? calQuality; // 0-100
  final bool? calVerified; // verified flag

  const WifiConnectionState({
    required this.isConnecting,
    required this.isConnected,
    required this.error,
    required this.rxLog,
    this.batteryPercent,
    this.gpsLat,
    this.gpsLon,
    this.gpsHeading,
    this.gpsFixType,
    this.gpsAccuracy,
    this.gpsHeightM,
    this.gpsCarrier,
    this.gpsDiff,
    this.gpsSatellites,
    this.gpsVAccuracy,
    this.gpsSpeedMps,
    this.gpsPDop,
    this.gpsAgeMs,
    this.gpsReceivedAt,
    this.rtcmBytes,
    this.rtcmAgeMs,
    this.rtcmTransportAgeMs,
    this.rtcmF9pAgeMs,
    this.rtcmSource,
    this.rtcmF9pMessages,
    this.rtcmCrcFail,
    this.rtcmLastType,
    this.imuYaw,
    this.imuAgeMs,
    this.imuFresh,
    this.imuReceivedAt,
    this.navState,
    this.navWpIndex,
    this.navWpTotal,
    this.navDistToWp,
    this.movementProgressRate,
    this.movementCrossTrack,
    this.movementStatus,
    this.motorLeft,
    this.motorRight,
    this.motorFeedback,
    this.motorSpeedLeft,
    this.motorSpeedRight,
    this.motorBatteryRaw,
    this.motorBoardTempRaw,
    this.calState,
    this.calOffset,
    this.calStdDev,
    this.calQuality,
    this.calVerified,
  });

  factory WifiConnectionState.initial() => const WifiConnectionState(
        isConnecting: false,
        isConnected: false,
        error: null,
        rxLog: <String>[],
        batteryPercent: null,
        gpsLat: null,
        gpsLon: null,
        gpsHeading: null,
        gpsFixType: null,
        gpsAccuracy: null,
        gpsHeightM: null,
        gpsCarrier: null,
        gpsDiff: null,
        gpsSatellites: null,
        gpsVAccuracy: null,
        gpsSpeedMps: null,
        gpsPDop: null,
        gpsAgeMs: null,
        gpsReceivedAt: null,
        rtcmBytes: null,
        rtcmAgeMs: null,
        rtcmTransportAgeMs: null,
        rtcmF9pAgeMs: null,
        rtcmSource: null,
        rtcmF9pMessages: null,
        rtcmCrcFail: null,
        rtcmLastType: null,
        imuYaw: null,
        imuAgeMs: null,
        imuFresh: null,
        imuReceivedAt: null,
        navState: null,
        navWpIndex: null,
        navWpTotal: null,
        navDistToWp: null,
        movementProgressRate: null,
        movementCrossTrack: null,
        movementStatus: null,
        motorLeft: null,
        motorRight: null,
        motorFeedback: null,
        motorSpeedLeft: null,
        motorSpeedRight: null,
        motorBatteryRaw: null,
        motorBoardTempRaw: null,
        calState: null,
        calOffset: null,
        calStdDev: null,
        calQuality: null,
        calVerified: null,
      );

  WifiConnectionState copyWith({
    bool? isConnecting,
    bool? isConnected,
    String? error,
    List<String>? rxLog,
    int? batteryPercent,
    double? gpsLat,
    double? gpsLon,
    double? gpsHeading,
    int? gpsFixType,
    int? gpsAccuracy,
    double? gpsHeightM,
    String? gpsCarrier,
    bool? gpsDiff,
    int? gpsSatellites,
    int? gpsVAccuracy,
    double? gpsSpeedMps,
    double? gpsPDop,
    int? gpsAgeMs,
    DateTime? gpsReceivedAt,
    int? rtcmBytes,
    int? rtcmAgeMs,
    int? rtcmTransportAgeMs,
    int? rtcmF9pAgeMs,
    String? rtcmSource,
    int? rtcmF9pMessages,
    int? rtcmCrcFail,
    int? rtcmLastType,
    double? imuYaw,
    int? imuAgeMs,
    bool? imuFresh,
    DateTime? imuReceivedAt,
    String? navState,
    int? navWpIndex,
    int? navWpTotal,
    double? navDistToWp,
    double? movementProgressRate,
    double? movementCrossTrack,
    String? movementStatus,
    int? motorLeft,
    int? motorRight,
    bool? motorFeedback,
    int? motorSpeedLeft,
    int? motorSpeedRight,
    int? motorBatteryRaw,
    int? motorBoardTempRaw,
    String? calState,
    double? calOffset,
    double? calStdDev,
    int? calQuality,
    bool? calVerified,
  }) {
    return WifiConnectionState(
      isConnecting: isConnecting ?? this.isConnecting,
      isConnected: isConnected ?? this.isConnected,
      error: error,
      rxLog: rxLog ?? this.rxLog,
      batteryPercent: batteryPercent ?? this.batteryPercent,
      gpsLat: gpsLat ?? this.gpsLat,
      gpsLon: gpsLon ?? this.gpsLon,
      gpsHeading: gpsHeading ?? this.gpsHeading,
      gpsFixType: gpsFixType ?? this.gpsFixType,
      gpsAccuracy: gpsAccuracy ?? this.gpsAccuracy,
      gpsHeightM: gpsHeightM ?? this.gpsHeightM,
      gpsCarrier: gpsCarrier ?? this.gpsCarrier,
      gpsDiff: gpsDiff ?? this.gpsDiff,
      gpsSatellites: gpsSatellites ?? this.gpsSatellites,
      gpsVAccuracy: gpsVAccuracy ?? this.gpsVAccuracy,
      gpsSpeedMps: gpsSpeedMps ?? this.gpsSpeedMps,
      gpsPDop: gpsPDop ?? this.gpsPDop,
      gpsAgeMs: gpsAgeMs ?? this.gpsAgeMs,
      gpsReceivedAt: gpsReceivedAt ?? this.gpsReceivedAt,
      rtcmBytes: rtcmBytes ?? this.rtcmBytes,
      rtcmAgeMs: rtcmAgeMs ?? this.rtcmAgeMs,
      rtcmTransportAgeMs: rtcmTransportAgeMs ?? this.rtcmTransportAgeMs,
      rtcmF9pAgeMs: rtcmF9pAgeMs ?? this.rtcmF9pAgeMs,
      rtcmSource: rtcmSource ?? this.rtcmSource,
      rtcmF9pMessages: rtcmF9pMessages ?? this.rtcmF9pMessages,
      rtcmCrcFail: rtcmCrcFail ?? this.rtcmCrcFail,
      rtcmLastType: rtcmLastType ?? this.rtcmLastType,
      imuYaw: imuYaw ?? this.imuYaw,
      imuAgeMs: imuAgeMs ?? this.imuAgeMs,
      imuFresh: imuFresh ?? this.imuFresh,
      imuReceivedAt: imuReceivedAt ?? this.imuReceivedAt,
      navState: navState ?? this.navState,
      navWpIndex: navWpIndex ?? this.navWpIndex,
      navWpTotal: navWpTotal ?? this.navWpTotal,
      navDistToWp: navDistToWp ?? this.navDistToWp,
      movementProgressRate: movementProgressRate ?? this.movementProgressRate,
      movementCrossTrack: movementCrossTrack ?? this.movementCrossTrack,
      movementStatus: movementStatus ?? this.movementStatus,
      motorLeft: motorLeft ?? this.motorLeft,
      motorRight: motorRight ?? this.motorRight,
      motorFeedback: motorFeedback ?? this.motorFeedback,
      motorSpeedLeft: motorSpeedLeft ?? this.motorSpeedLeft,
      motorSpeedRight: motorSpeedRight ?? this.motorSpeedRight,
      motorBatteryRaw: motorBatteryRaw ?? this.motorBatteryRaw,
      motorBoardTempRaw: motorBoardTempRaw ?? this.motorBoardTempRaw,
      calState: calState ?? this.calState,
      calOffset: calOffset ?? this.calOffset,
      calStdDev: calStdDev ?? this.calStdDev,
      calQuality: calQuality ?? this.calQuality,
      calVerified: calVerified ?? this.calVerified,
    );
  }

  WifiConnectionState clearTelemetry({
    bool? isConnecting,
    bool? isConnected,
    String? error,
    List<String>? rxLog,
  }) {
    return WifiConnectionState(
      isConnecting: isConnecting ?? this.isConnecting,
      isConnected: isConnected ?? this.isConnected,
      error: error,
      rxLog: rxLog ?? this.rxLog,
      batteryPercent: null,
      gpsLat: null,
      gpsLon: null,
      gpsHeading: null,
      gpsFixType: null,
      gpsAccuracy: null,
      gpsHeightM: null,
      gpsCarrier: null,
      gpsDiff: null,
      gpsSatellites: null,
      gpsVAccuracy: null,
      gpsSpeedMps: null,
      gpsPDop: null,
      gpsAgeMs: null,
      gpsReceivedAt: null,
      rtcmBytes: null,
      rtcmAgeMs: null,
      rtcmTransportAgeMs: null,
      rtcmF9pAgeMs: null,
      rtcmSource: null,
      rtcmF9pMessages: null,
      rtcmCrcFail: null,
      rtcmLastType: null,
      imuYaw: null,
      imuAgeMs: null,
      imuFresh: null,
      imuReceivedAt: null,
      navState: null,
      navWpIndex: null,
      navWpTotal: null,
      navDistToWp: null,
      movementProgressRate: null,
      movementCrossTrack: null,
      movementStatus: null,
      motorLeft: null,
      motorRight: null,
      motorFeedback: null,
      motorSpeedLeft: null,
      motorSpeedRight: null,
      motorBatteryRaw: null,
      motorBoardTempRaw: null,
      calState: null,
      calOffset: null,
      calStdDev: null,
      calQuality: null,
      calVerified: null,
    );
  }
}

/// ============================================================
/// Wi-Fi ping check setting
/// ============================================================
final wifiPingCheckProvider =
    StateNotifierProvider<WifiPingCheckNotifier, bool>(
  (ref) => WifiPingCheckNotifier(),
);

class WifiPingCheckNotifier extends StateNotifier<bool> {
  static const String _key = 'wifi_ping_check_enabled';
  bool _initialized = false;

  WifiPingCheckNotifier() : super(false) {
    _init();
  }

  Future<void> _init() async {
    if (_initialized) return;
    try {
      final prefs = await SharedPreferences.getInstance();
      final value = prefs.getBool(_key);
      debugPrint('DEBUG: Loading wifi ping check from prefs: $value');
      if (value != null) {
        state = value;
        debugPrint('DEBUG: Set wifi ping check state to: $value');
      } else {
        debugPrint('DEBUG: No saved value, using default: false');
      }
      _initialized = true;
    } catch (e) {
      debugPrint('DEBUG: Error loading wifi ping check: $e');
      _initialized = true;
      // Оставляем значение по умолчанию (false)
    }
  }

  // Публичный метод для инициализации (если нужно вызвать извне)
  Future<void> ensureInitialized() => _init();

  Future<void> setEnabled(bool enabled) async {
    debugPrint('DEBUG: setEnabled called with: $enabled');
    state = enabled;
    debugPrint('DEBUG: State updated to: $state');
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setBool(_key, enabled);
      // Проверяем, что значение сохранилось
      final saved = prefs.getBool(_key);
      debugPrint('DEBUG: Saved wifi ping check: $enabled, read back: $saved');
      if (saved != enabled) {
        debugPrint('DEBUG: WARNING! Saved value does not match!');
      }
    } catch (e) {
      debugPrint('DEBUG: Error saving wifi ping check: $e');
      // Игнорируем ошибки сохранения
    }
  }

  // Метод для получения актуального значения (с ожиданием инициализации)
  Future<bool> getValue() async {
    if (!_initialized) {
      await _init();
    }
    return state;
  }
}

final wifiConnectionProvider =
    StateNotifierProvider<WifiConnectionNotifier, WifiConnectionState>(
  (ref) => WifiConnectionNotifier(ref),
);

final wifiRobotHostProvider =
    StateNotifierProvider<WifiRobotHostNotifier, String>(
  (ref) => WifiRobotHostNotifier(),
);

class WifiRobotHostNotifier extends StateNotifier<String> {
  static const String defaultHost = '192.168.31.222';
  static const String _key = 'wifi_robot_host';
  bool _initialized = false;

  WifiRobotHostNotifier() : super(defaultHost) {
    _init();
  }

  Future<void> _init() async {
    if (_initialized) return;
    try {
      final prefs = await SharedPreferences.getInstance();
      final saved = prefs.getString(_key);
      if (saved != null && saved.trim().isNotEmpty) {
        state = saved.trim();
      }
    } catch (_) {
      // Keep default host.
    }
    _initialized = true;
  }

  Future<void> ensureInitialized() => _init();

  Future<void> setHost(String host) async {
    final clean = host.trim().isEmpty ? defaultHost : host.trim();
    state = clean;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_key, clean);
  }
}

class WifiConnectionNotifier extends StateNotifier<WifiConnectionState> {
  final Ref _ref;

  WifiConnectionNotifier(this._ref) : super(WifiConnectionState.initial()) {
    _initConnectivityListener();
  }

  String get _host {
    final host = _ref.read(wifiRobotHostProvider).trim();
    return host.isEmpty ? WifiRobotHostNotifier.defaultHost : host;
  }

  static const int _port = 81;

  WebSocketChannel? _channel;
  StreamSubscription? _sub;
  StreamSubscription<List<ConnectivityResult>>? _connectivitySubscription;
  Timer? _healthTimer;
  Timer? _reconnectTimer;
  DateTime? _lastRxAt;
  DateTime? _lastTelemetryLogAt;
  DateTime? _lastGpsDebugAt;
  bool _autoReconnectEnabled = false;
  int _reconnectAttempt = 0;

  Completer<void>? _pongWaiter;
  Completer<String>? _commandAckWaiter;
  bool Function(String)? _commandAckMatcher;
  Completer<String>? _routeAckWaiter;
  bool Function(String)? _routeAckMatcher;
  // WiFi на 11 dBm + TCP/IP стек ESP32 могут давать 2-4 с задержку. 2 с таймаут
  // приводил к ложным фейлам и карусели "press 15 раз". Поднимаем.
  static const Duration _commandAckTimeout = Duration(seconds: 5);
  // Per-waypoint ждём подольше — пакеты склеиваются/теряются на границах.
  static const Duration _routeAckTimeout = Duration(seconds: 8);

  Uri get _wsUri => Uri.parse("ws://$_host:$_port/ws");

  /// Инициализация слушателя изменений состояния сети
  void _initConnectivityListener() {
    _connectivitySubscription = Connectivity().onConnectivityChanged.listen(
      (List<ConnectivityResult> results) {
        _handleConnectivityChange(results);
      },
    );
  }

  /// Обработка изменений состояния сети
  void _handleConnectivityChange(List<ConnectivityResult> results) {
    final hasWifi = results.contains(ConnectivityResult.wifi);

    _log("→ Connectivity changed: $results (Wi-Fi: $hasWifi)");

    if (hasWifi && _autoReconnectEnabled && !state.isConnected) {
      _scheduleReconnect("Wi-Fi вернулся");
    }
  }

  void _log(String line) {
    final next = List<String>.from(state.rxLog);
    next.add(line);
    if (next.length > 200) next.removeRange(0, next.length - 200);
    state = state.copyWith(rxLog: next);
  }

  void addLocalLog(String line) {
    _log(line);
  }

  bool _isHighRateTelemetry(String msg) {
    return msg.startsWith("GPS,") ||
        msg.startsWith("GPSDBG,") ||
        msg.startsWith("RTCM,") ||
        msg.startsWith("IMU,") ||
        msg.startsWith("TEL,") ||
        msg == "OK M";
  }

  void _logIncoming(String msg) {
    if (_isHighRateTelemetry(msg)) return;
    _log("← $msg");
  }

  void _maybeLogTelemetrySummary() {
    final now = DateTime.now();
    final last = _lastTelemetryLogAt;
    if (last != null && now.difference(last) < const Duration(seconds: 1)) {
      return;
    }
    _lastTelemetryLogAt = now;

    final carrier = state.gpsCarrier ?? '-';
    final hAcc = state.gpsAccuracy == null ? '-' : '${state.gpsAccuracy}mm';
    final rtcmAge = state.rtcmAgeMs == null ? '-' : '${state.rtcmAgeMs}ms';
    final rtcmTransport = state.rtcmTransportAgeMs == null
        ? '-'
        : '${state.rtcmTransportAgeMs}ms';
    final rtcmF9p =
        state.rtcmF9pAgeMs == null ? '-' : '${state.rtcmF9pAgeMs}ms';
    final rtcmSource = state.rtcmSource ?? '-';
    final rtcmType = state.rtcmLastType?.toString() ?? '-';
    final imuYaw =
        state.imuYaw == null ? '-' : '${state.imuYaw!.toStringAsFixed(1)}deg';
    final imuAge = state.imuAgeMs == null ? '-' : '${state.imuAgeMs}ms';
    final imuFresh =
        state.imuFresh == null ? '-' : (state.imuFresh! ? '1' : '0');
    final sv = state.gpsSatellites?.toString() ?? '-';
    final fix = state.gpsFixType?.toString() ?? '-';
    final nav = state.navState ?? '-';
    final wp = state.navWpIndex == null || state.navWpTotal == null
        ? '-'
        : '${state.navWpIndex}/${state.navWpTotal}';
    final dist = state.navDistToWp == null
        ? '-'
        : '${state.navDistToWp!.toStringAsFixed(2)}m';
    _log(
      "← telemetry fix=$fix rtk=$carrier hAcc=$hAcc sv=$sv "
      "rtcm=$rtcmAge transport=$rtcmTransport f9p=$rtcmF9p "
      "src=$rtcmSource type=$rtcmType imu=$imuYaw/$imuAge fresh=$imuFresh "
      "nav=$nav wp=$wp dist=$dist",
    );
  }

  bool _hasFreshGpsDebug(DateTime now) {
    final last = _lastGpsDebugAt;
    return last != null && now.difference(last) < const Duration(seconds: 2);
  }

  void _startHealthTimer() {
    _healthTimer?.cancel();
    _healthTimer = Timer.periodic(const Duration(seconds: 2), (_) {
      if (!state.isConnected) {
        if (_autoReconnectEnabled && !state.isConnecting) {
          _scheduleReconnect("нет активного соединения");
        }
        return;
      }

      final lastRx = _lastRxAt;
      if (lastRx == null) return;
      final silence = DateTime.now().difference(lastRx);
      final heartbeatChannel = _channel;
      if (heartbeatChannel != null) {
        try {
          heartbeatChannel.sink.add("PING");
        } catch (e) {
          unawaited(_handleConnectionLost("РћС€РёР±РєР° PING: $e"));
          return;
        }
      }

      if (silence.inSeconds >= 8) {
        final ch = _channel;
        if (ch != null) {
          try {
            ch.sink.add("PING");
            _log("→ PING watchdog");
          } catch (e) {
            unawaited(_handleConnectionLost("Ошибка PING: $e"));
            return;
          }
        }
      }

      if (silence.inSeconds >= 20) {
        unawaited(
          _handleConnectionLost(
            "Нет данных от ровера ${silence.inSeconds} секунд",
          ),
        );
      }
    });
  }

  void _scheduleReconnect(String reason) {
    if (!_autoReconnectEnabled || state.isConnecting || state.isConnected) {
      return;
    }
    if (_reconnectTimer?.isActive ?? false) return;

    final delaySeconds = _reconnectAttempt <= 0
        ? 1
        : (_reconnectAttempt == 1 ? 2 : (_reconnectAttempt == 2 ? 3 : 5));
    _reconnectAttempt++;
    _log("↻ reconnect через ${delaySeconds}s: $reason");
    _reconnectTimer = Timer(Duration(seconds: delaySeconds), () {
      _reconnectTimer = null;
      if (!_autoReconnectEnabled || state.isConnected || state.isConnecting) {
        return;
      }
      unawaited(connect(skipPreflight: true));
    });
  }

  Future<void> _handleConnectionLost(String error) async {
    if (!state.isConnected && !state.isConnecting && _channel == null) {
      _scheduleReconnect(error);
      return;
    }

    _log("× connection lost: $error");
    await _closeSocketOnly();
    state = state.clearTelemetry(
      isConnecting: false,
      isConnected: false,
      error: error,
    );
    _scheduleReconnect(error);
  }

  void _failAckWaiters(String reason) {
    // Завершаем ожидающие completer'ы с ошибкой (не молча!), чтобы их `await` сразу
    // получил TimeoutException/StateError и вышел из retry-цикла.
    if (_commandAckWaiter != null && !_commandAckWaiter!.isCompleted) {
      _commandAckWaiter!.completeError(StateError(reason));
    }
    _commandAckWaiter = null;
    _commandAckMatcher = null;
    if (_routeAckWaiter != null && !_routeAckWaiter!.isCompleted) {
      _routeAckWaiter!.completeError(StateError(reason));
    }
    _routeAckWaiter = null;
    _routeAckMatcher = null;
  }

  Future<void> _closeSocketOnly() async {
    _failAckWaiters("connection closed");
    await _sub?.cancel();
    _sub = null;
    try {
      _channel?.sink.close();
    } catch (_) {}
    _channel = null;
  }

  Future<void> connect({bool skipPreflight = false}) async {
    if (state.isConnecting || state.isConnected) return;
    final _ = skipPreflight;

    _autoReconnectEnabled = true;
    _lastTelemetryLogAt = null;
    _lastGpsDebugAt = null;
    _reconnectTimer?.cancel();
    _reconnectTimer = null;
    state = state.clearTelemetry(isConnecting: true, error: null);
    _log("=== CONNECT START ===");
    await _ref.read(wifiRobotHostProvider.notifier).ensureInitialized();

    _log("WebSocket preflight disabled");

    try {
      _log("→ WS connect $_wsUri");
      WebSocketChannel ch;
      try {
        ch = WebSocketChannel.connect(_wsUri);
        _log("→ WebSocket channel created");
      } catch (e) {
        _log("× Failed to create WebSocket channel: $e");
        await _handleConnectFail("Не удалось создать WebSocket соединение: $e");
        return;
      }

      // На iOS ready может не работать, поэтому используем другой подход
      // Устанавливаем канал сразу и слушаем stream
      _channel = ch;
      _log("→ Waiting for connection and STATE,CONNECTED message...");

      // Создаем completer для отслеживания успешного подключения
      final connectionCompleter = Completer<bool>();
      bool connectionEstablished = false;

      _sub?.cancel();
      _sub = ch.stream.listen(
        (msg) {
          _lastRxAt = DateTime.now();
          final msgStr = msg.toString().trim();
          _logIncoming(msgStr);
          final upperMsg = msgStr.toUpperCase();

          if (msgStr.startsWith("TEL,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 19) {
                final lat = double.tryParse(parts[1]);
                final lon = double.tryParse(parts[2]);
                if (lat != null && lon != null) {
                  final now = DateTime.now();
                  _lastGpsDebugAt = now;
                  final imuYaw = double.tryParse(parts[16]);
                  state = state.copyWith(
                    gpsLat: lat,
                    gpsLon: lon,
                    gpsHeightM: double.tryParse(parts[3]),
                    gpsHeading: double.tryParse(parts[4]),
                    gpsFixType: int.tryParse(parts[5]),
                    gpsCarrier:
                        parts[6].trim().isEmpty ? null : parts[6].trim(),
                    gpsDiff: parts[7].trim() == '1',
                    gpsSatellites: int.tryParse(parts[8]),
                    gpsAccuracy: int.tryParse(parts[9]),
                    gpsVAccuracy: int.tryParse(parts[10]),
                    gpsSpeedMps: double.tryParse(parts[11]),
                    gpsPDop: double.tryParse(parts[12]),
                    gpsAgeMs: int.tryParse(parts[13]),
                    gpsReceivedAt: now,
                    rtcmBytes: int.tryParse(parts[14]),
                    rtcmAgeMs: int.tryParse(parts[15]),
                    rtcmTransportAgeMs:
                        parts.length > 19 ? int.tryParse(parts[19]) : null,
                    rtcmF9pAgeMs:
                        parts.length > 20 ? int.tryParse(parts[20]) : null,
                    rtcmSource: parts.length > 21 && parts[21].trim().isNotEmpty
                        ? parts[21].trim()
                        : null,
                    rtcmF9pMessages:
                        parts.length > 22 ? int.tryParse(parts[22]) : null,
                    rtcmCrcFail:
                        parts.length > 23 ? int.tryParse(parts[23]) : null,
                    rtcmLastType:
                        parts.length > 24 ? int.tryParse(parts[24]) : null,
                    imuYaw: imuYaw,
                    imuAgeMs: int.tryParse(parts[17]),
                    imuFresh: parts[18].trim() == '1',
                    imuReceivedAt: imuYaw == null ? null : now,
                  );
                  _maybeLogTelemetrySummary();
                }
              }
            } catch (e) {
              _log("× Failed to parse TEL: $e");
            }
            return;
          }

          // Парсинг BAT_PCT,<int>
          if (msgStr.startsWith("BAT_PCT,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 2) {
                final batteryValue = int.tryParse(parts[1]);
                if (batteryValue != null &&
                    batteryValue >= 0 &&
                    batteryValue <= 100) {
                  state = state.copyWith(batteryPercent: batteryValue);
                  _log("✓ Battery percent updated: $batteryValue%");
                }
              }
            } catch (e) {
              _log("× Failed to parse BAT_PCT: $e");
            }
          }

          // Парсинг GPS,<lat>,<lon>,<heading>,<fixType>,<hAcc>
          if (msgStr.startsWith("GPS,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 6) {
                final lat = double.tryParse(parts[1]);
                final lon = double.tryParse(parts[2]);
                final heading = double.tryParse(parts[3]);
                final fixType = int.tryParse(parts[4]);
                final hAcc = int.tryParse(parts[5]);

                final now = DateTime.now();
                if (lat != null && lon != null && !_hasFreshGpsDebug(now)) {
                  state = state.copyWith(
                    gpsLat: lat,
                    gpsLon: lon,
                    gpsHeading: heading,
                    gpsFixType: fixType,
                    gpsAccuracy: hAcc,
                    gpsReceivedAt: now,
                  );
                  _maybeLogTelemetrySummary();
                }
              }
            } catch (e) {
              _log("× Failed to parse GPS: $e");
            }
          }

          // Extended GPS debug:
          // GPSDBG,<lat>,<lon>,<heightM>,<heading>,<fixType>,<carrier>,<diff>,<numSV>,<hAccMm>,<vAccMm>,<speedMps>,<pDop>,<ageMs>
          if (msgStr.startsWith("GPSDBG,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 14) {
                final lat = double.tryParse(parts[1]);
                final lon = double.tryParse(parts[2]);
                final heightM = double.tryParse(parts[3]);
                final heading = double.tryParse(parts[4]);
                final fixType = int.tryParse(parts[5]);
                final carrier =
                    parts[6].trim().isEmpty ? null : parts[6].trim();
                final diff = parts[7].trim() == '1';
                final satellites = int.tryParse(parts[8]);
                final hAcc = int.tryParse(parts[9]);
                final vAcc = int.tryParse(parts[10]);
                final speedMps = double.tryParse(parts[11]);
                final pDop = double.tryParse(parts[12]);
                final ageMs = int.tryParse(parts[13]);

                if (lat != null && lon != null) {
                  final now = DateTime.now();
                  _lastGpsDebugAt = now;
                  state = state.copyWith(
                    gpsLat: lat,
                    gpsLon: lon,
                    gpsHeightM: heightM,
                    gpsHeading: heading,
                    gpsFixType: fixType,
                    gpsCarrier: carrier,
                    gpsDiff: diff,
                    gpsSatellites: satellites,
                    gpsAccuracy: hAcc,
                    gpsVAccuracy: vAcc,
                    gpsSpeedMps: speedMps,
                    gpsPDop: pDop,
                    gpsAgeMs: ageMs,
                    gpsReceivedAt: now,
                  );
                  _maybeLogTelemetrySummary();
                }
              }
            } catch (e) {
              _log("× Failed to parse GPSDBG: $e");
            }
          }

          // RTCM,<bytesTotal>,<ageMs>[,<transportAge>,<f9pAge>,<source>,<f9pMessages>,<crcFail>,<lastType>]
          if (msgStr.startsWith("RTCM,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 3) {
                final bytes = int.tryParse(parts[1]);
                final ageMs = int.tryParse(parts[2]);
                state = state.copyWith(
                  rtcmBytes: bytes,
                  rtcmAgeMs: ageMs,
                  rtcmTransportAgeMs:
                      parts.length > 3 ? int.tryParse(parts[3]) : null,
                  rtcmF9pAgeMs:
                      parts.length > 4 ? int.tryParse(parts[4]) : null,
                  rtcmSource: parts.length > 5 && parts[5].trim().isNotEmpty
                      ? parts[5].trim()
                      : null,
                  rtcmF9pMessages:
                      parts.length > 6 ? int.tryParse(parts[6]) : null,
                  rtcmCrcFail: parts.length > 7 ? int.tryParse(parts[7]) : null,
                  rtcmLastType:
                      parts.length > 8 ? int.tryParse(parts[8]) : null,
                );
                _maybeLogTelemetrySummary();
              }
            } catch (e) {
              _log("× Failed to parse RTCM: $e");
            }
          }

          // Парсинг IMU,<yaw>[,<ageMs>,<fresh>]
          if (msgStr.startsWith("IMU,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 2) {
                final yaw = double.tryParse(parts[1]);
                if (yaw != null) {
                  final ageMs =
                      parts.length >= 3 ? int.tryParse(parts[2]) : null;
                  final fresh =
                      parts.length >= 4 ? parts[3].trim() == '1' : true;
                  state = state.copyWith(
                    imuYaw: yaw,
                    imuAgeMs: ageMs,
                    imuFresh: fresh,
                    imuReceivedAt: DateTime.now(),
                  );
                }
              }
            } catch (e) {
              _log("× Failed to parse IMU: $e");
            }
          }

          // Парсинг NAV,<state>,<wpIdx>,<wpTotal>,<distToWp>[,<progressRate>,<crossTrack>,<movementStatus>]
          if (msgStr.startsWith("NAV,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 5) {
                final navState = parts[1];
                final wpIdx = int.tryParse(parts[2]);
                final wpTotal = int.tryParse(parts[3]);
                final distToWp = double.tryParse(parts[4]);

                state = state.copyWith(
                  navState: navState,
                  navWpIndex: wpIdx,
                  navWpTotal: wpTotal,
                  navDistToWp: distToWp,
                  movementProgressRate: null,
                  movementCrossTrack:
                      parts.length > 6 ? double.tryParse(parts[6]) : null,
                  movementStatus: parts.length > 9 && parts[9].trim().isNotEmpty
                      ? parts[9].trim()
                      : null,
                );
              }
            } catch (e) {
              _log("× Failed to parse NAV: $e");
            }
          }

          // Generic OK/ERR responses from rover (route acks, nav ack, etc.)
          if (msgStr.startsWith("OK")) {
            if (_commandAckWaiter != null &&
                !_commandAckWaiter!.isCompleted &&
                (_commandAckMatcher?.call(msgStr) ?? true)) {
              _commandAckWaiter!.complete(msgStr);
              _commandAckWaiter = null;
              _commandAckMatcher = null;
            }
            if (_routeAckWaiter != null &&
                !_routeAckWaiter!.isCompleted &&
                (_routeAckMatcher?.call(msgStr) ?? true)) {
              _routeAckWaiter!.complete(msgStr);
              _routeAckWaiter = null;
              _routeAckMatcher = null;
            }
          }
          if (msgStr.startsWith("ERR,")) {
            _log("× Rover ERR: $msgStr");
            if (_commandAckWaiter != null && !_commandAckWaiter!.isCompleted) {
              _commandAckWaiter!.completeError(msgStr);
              _commandAckWaiter = null;
              _commandAckMatcher = null;
            }
            if (_routeAckWaiter != null && !_routeAckWaiter!.isCompleted) {
              _routeAckWaiter!.completeError(msgStr);
              _routeAckWaiter = null;
              _routeAckMatcher = null;
            }
          }

          // Waypoint reached notification
          if (msgStr.startsWith("NAV_WP,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 2) {
                final wpIdx = int.tryParse(parts[1]);
                _log("✓ Waypoint $wpIdx reached");
              }
            } catch (e) {
              _log("× Failed to parse NAV_WP: $e");
            }
          }

          // Парсинг CAL,<state>,<offset>,<stdDev>,<quality>,<verified>
          if (msgStr.startsWith("CAL,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 6) {
                state = state.copyWith(
                  calState: parts[1].trim().isNotEmpty ? parts[1].trim() : null,
                  calOffset: double.tryParse(parts[2]),
                  calStdDev: double.tryParse(parts[3]),
                  calQuality: int.tryParse(parts[4]),
                  calVerified: parts[5].trim() == '1',
                );
              }
            } catch (e) {
              _log("× Failed to parse CAL: $e");
            }
          }

          // Если получили STATE,CONNECTED - соединение установлено
          if (msgStr.startsWith("MOTOR,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 3) {
                state = state.copyWith(
                  motorLeft: int.tryParse(parts[1]),
                  motorRight: int.tryParse(parts[2]),
                  motorFeedback:
                      parts.length >= 4 ? parts[3].trim() == '1' : null,
                  motorSpeedLeft:
                      parts.length >= 5 ? int.tryParse(parts[4]) : null,
                  motorSpeedRight:
                      parts.length >= 6 ? int.tryParse(parts[5]) : null,
                  motorBatteryRaw:
                      parts.length >= 7 ? int.tryParse(parts[6]) : null,
                  motorBoardTempRaw:
                      parts.length >= 8 ? int.tryParse(parts[7]) : null,
                );
              }
            } catch (e) {
              _log("Failed to parse MOTOR: $e");
            }
          }

          if (upperMsg.contains("CONNECTED") || upperMsg.contains("STATE")) {
            if (!connectionEstablished) {
              connectionEstablished = true;
              if (!connectionCompleter.isCompleted) {
                connectionCompleter.complete(true);
              }
              _log("✓ WS connected (received CONNECTED message)");
            }
          }

          // Обработка PONG (если робот его отправляет)
          if (upperMsg == "PONG" || upperMsg.startsWith("PONG")) {
            _log("✓ PONG received");
            _pongWaiter?.complete();
            _pongWaiter = null;
          }
        },
        onError: (e) {
          _log("× WS stream error: $e");
          if (!connectionCompleter.isCompleted) {
            _log("× Completing connectionCompleter with false due to error");
            connectionCompleter.complete(false);
          }
          if (connectionEstablished || state.isConnected) {
            unawaited(_handleConnectionLost("WebSocket ошибка: $e"));
          }
        },
        onDone: () {
          _log("× WS stream closed (onDone)");
          if (!connectionCompleter.isCompleted) {
            _log(
                "× Completing connectionCompleter with false due to stream closed");
            connectionCompleter.complete(false);
          }
          if (connectionEstablished || state.isConnected) {
            unawaited(_handleConnectionLost("WebSocket закрыт"));
          }
        },
        cancelOnError: false,
      );

      // Ждем либо первого сообщения, либо ошибки (максимум 10 секунд)
      try {
        _log("→ Waiting for STATE,CONNECTED message...");
        final connected = await connectionCompleter.future.timeout(
          Duration(seconds: skipPreflight ? 4 : 10),
          onTimeout: () {
            _log("× Connection timeout waiting for first message");
            return false;
          },
        );

        if (!connected) {
          _log("× Connection completer returned false");
          await _handleConnectFail(
              "Не удалось установить WebSocket соединение");
          return;
        }

        // Робот уже отправил STATE,CONNECTED, значит соединение установлено
        // Не требуем PONG, так как робот может не отвечать на PING текстовым сообщением
        _log("✓ Connection established, setting isConnected = true");
        state =
            state.copyWith(isConnecting: false, isConnected: true, error: null);
        _lastRxAt = DateTime.now();
        _reconnectAttempt = 0;
        _startHealthTimer();
        _log("=== CONNECT OK ===");

        // Опционально: отправляем PING для проверки (но не ждем ответа)
        _log("→ Sending PING (optional)");
        sendRaw("PING");
      } catch (e) {
        _log("=== CONNECT FAIL: $e ===");
        await _handleConnectFail("Не удалось подключиться по WebSocket: $e");
      }
    } catch (e) {
      _log("=== CONNECT FAIL: $e ===");
      await _handleConnectFail("Не удалось подключиться по WebSocket: $e");
    }
  }

  Future<void> _handleConnectFail(String error) async {
    await _closeSocketOnly();
    state = state.clearTelemetry(
      isConnecting: false,
      isConnected: false,
      error: error,
    );
    _scheduleReconnect(error);
  }

  Future<void> disconnect({String? error}) async {
    _autoReconnectEnabled = false;
    _reconnectTimer?.cancel();
    _reconnectTimer = null;
    _healthTimer?.cancel();
    _healthTimer = null;
    state = state.clearTelemetry(
      isConnecting: false,
      isConnected: false,
      error: error,
    );

    await _closeSocketOnly();
    _log("=== DISCONNECTED ===");
  }

  @override
  void dispose() {
    _autoReconnectEnabled = false;
    _healthTimer?.cancel();
    _reconnectTimer?.cancel();
    _connectivitySubscription?.cancel();
    _sub?.cancel();
    _channel?.sink.close();
    super.dispose();
  }

  void sendRaw(String text, {bool log = true}) {
    final ch = _channel;
    if (ch == null) {
      _log("× sendRaw failed: channel is null");
      return;
    }
    try {
      if (log) _log("→ $text");
      ch.sink.add(text);
    } catch (e) {
      _log("× sendRaw error: $e");
      unawaited(_handleConnectionLost("Ошибка отправки: $e"));
    }
  }

  /// left/right: -100..100
  void sendMove(int left, int right) {
    if (!state.isConnected) return;
    left = left.clamp(-100, 100);
    right = right.clamp(-100, 100);
    sendRaw("M,$left,$right", log: false);
  }

  void sendStop() {
    if (!state.isConnected) return;
    sendRaw("STOP", log: false);
  }

  /// Отправка команды для насадки (attachment)
  /// enabled: true = включить, false = выключить
  void sendAttachment(bool enabled) {
    if (!state.isConnected) return;
    sendRaw(enabled ? "ATTACHMENT_ON" : "ATTACHMENT_OFF");
  }

  /// Отправка команды для крепления (mount)
  /// enabled: true = включить, false = выключить
  void sendMount(bool enabled) {
    if (!state.isConnected) return;
    sendRaw(enabled ? "MOUNT_ON" : "MOUNT_OFF");
  }

  /// Route upload commands
  void sendAreaBegin(
    int count, {
    required double originLat,
    required double originLon,
    required double lineStepMeters,
  }) {
    if (!state.isConnected) return;
    sendRaw(
      "AREA_BEGIN,$count,${originLat.toStringAsFixed(8)},"
      "${originLon.toStringAsFixed(8)},${lineStepMeters.toStringAsFixed(3)}",
    );
  }

  void sendAreaPoint(int index, double xMeters, double yMeters) {
    if (!state.isConnected) return;
    sendRaw(
      "AREA_PT,$index,${xMeters.toStringAsFixed(3)},${yMeters.toStringAsFixed(3)}",
    );
  }

  void sendAreaEnd() {
    if (!state.isConnected) return;
    sendRaw("AREA_END");
  }

  Future<String> _sendAcked(
    String line,
    String label,
    bool Function(String ack) matcher,
  ) async {
    if (!state.isConnected) {
      throw StateError('not connected');
    }
    for (var attempt = 0; attempt < 2; attempt++) {
      final completer = Completer<String>();
      _commandAckWaiter = completer;
      _commandAckMatcher = matcher;
      sendRaw(line);
      try {
        final ack = await completer.future.timeout(_commandAckTimeout);
        _commandAckWaiter = null;
        _commandAckMatcher = null;
        return ack;
      } catch (e) {
        _commandAckWaiter = null;
        _commandAckMatcher = null;
        if (attempt == 0) {
          _log("× $label ack timeout, retry 1/1");
        } else {
          _log("× $label failed after 2 attempts");
          rethrow;
        }
      }
    }
    throw StateError('$label ack failed');
  }

  Future<void> _sendRouteAcked(String line, String label) async {
    await _sendAcked(line, label, (ack) {
      if (label == "ROUTE_BOUNDARY_BEGIN") {
        return ack == "OK,ROUTE_BOUNDARY_BEGIN";
      }
      if (label.startsWith("ROUTE_BOUNDARY_PT ")) {
        final index = label.substring("ROUTE_BOUNDARY_PT ".length);
        return ack == "OK,ROUTE_BOUNDARY_PT,$index";
      }
      if (label == "ROUTE_BOUNDARY_END") {
        return ack == "OK,ROUTE_BOUNDARY_END";
      }
      if (label == "FORBID_BEGIN") return ack == "OK,FORBID_BEGIN";
      if (label.startsWith("FORBID_PT ")) {
        final parts = label.substring("FORBID_PT ".length).split('/');
        return parts.length == 2 &&
            ack == "OK,FORBID_PT,${parts[0]},${parts[1]}";
      }
      if (label == "FORBID_END") return ack == "OK,FORBID_END";
      return false;
    });
  }

  Future<void> sendRouteBegin(
    int count, {
    required double originLat,
    required double originLon,
  }) async {
    if (!state.isConnected) {
      throw StateError('not connected');
    }
    for (var attempt = 0; attempt < 2; attempt++) {
      final completer = Completer<String>();
      _routeAckWaiter = completer;
      _routeAckMatcher = (ack) => ack == "OK,ROUTE_BEGIN";
      sendRaw(
        "ROUTE_BEGIN,$count,${originLat.toStringAsFixed(8)},"
        "${originLon.toStringAsFixed(8)}",
      );
      try {
        await completer.future.timeout(_routeAckTimeout);
        _routeAckWaiter = null;
        _routeAckMatcher = null;
        return;
      } catch (e) {
        _routeAckWaiter = null;
        _routeAckMatcher = null;
        if (attempt == 0) {
          _log("× ROUTE_BEGIN ack timeout, retry 1/1");
        } else {
          _log("× ROUTE_BEGIN failed after 2 attempts");
          rethrow;
        }
      }
    }
  }

  Future<void> sendRoutePoint(int index, double xMeters, double yMeters) async {
    if (!state.isConnected) {
      throw StateError('not connected');
    }
    for (var attempt = 0; attempt < 2; attempt++) {
      final completer = Completer<String>();
      _routeAckWaiter = completer;
      _routeAckMatcher = (ack) => ack == "OK,ROUTE_WP,$index";
      sendRaw(
        "ROUTE_WP,$index,${xMeters.toStringAsFixed(3)},${yMeters.toStringAsFixed(3)}",
      );
      try {
        await completer.future.timeout(_routeAckTimeout);
        _routeAckWaiter = null;
        _routeAckMatcher = null;
        return;
      } catch (e) {
        _routeAckWaiter = null;
        _routeAckMatcher = null;
        if (attempt == 0) {
          _log("× ROUTE_WP $index ack timeout, retry 1/1");
        } else {
          _log("× ROUTE_WP $index failed after 2 attempts");
          rethrow;
        }
      }
    }
  }

  Future<void> sendRouteEnd(int expectedCount) async {
    if (!state.isConnected) {
      throw StateError('not connected');
    }
    for (var attempt = 0; attempt < 2; attempt++) {
      final completer = Completer<String>();
      _routeAckWaiter = completer;
      _routeAckMatcher = (ack) => ack == "OK,ROUTE,$expectedCount";
      sendRaw("ROUTE_END");
      try {
        final ack = await completer.future.timeout(_routeAckTimeout);
        if (ack != "OK,ROUTE,$expectedCount") {
          throw StateError("unexpected ROUTE_END ack: $ack");
        }
        _routeAckWaiter = null;
        _routeAckMatcher = null;
        return;
      } catch (e) {
        _routeAckWaiter = null;
        _routeAckMatcher = null;
        if (attempt == 0) {
          _log("× ROUTE_END ack timeout, retry 1/1");
        } else {
          _log("× ROUTE_END failed after 2 attempts");
          rethrow;
        }
      }
    }
  }

  Future<void> sendRouteBoundaryBegin(int count) async {
    await _sendRouteAcked(
        "ROUTE_BOUNDARY_BEGIN,$count", "ROUTE_BOUNDARY_BEGIN");
  }

  Future<void> sendRouteBoundaryPoint(
    int index,
    double xMeters,
    double yMeters,
  ) async {
    await _sendRouteAcked(
      "ROUTE_BOUNDARY_PT,$index,${xMeters.toStringAsFixed(3)},${yMeters.toStringAsFixed(3)}",
      "ROUTE_BOUNDARY_PT $index",
    );
  }

  Future<void> sendRouteBoundaryEnd() async {
    await _sendRouteAcked("ROUTE_BOUNDARY_END", "ROUTE_BOUNDARY_END");
  }

  void sendForbiddenBegin(int polygonCount,
      [List<int> pointCounts = const []]) {
    if (!state.isConnected) return;
    final suffix = pointCounts.isEmpty
        ? ''
        : ',${pointCounts.map((v) => v.toString()).join(',')}';
    sendRaw("FORBID_BEGIN,$polygonCount$suffix");
  }

  void sendForbiddenPoint(
    int polygonIndex,
    int pointIndex,
    double xMeters,
    double yMeters,
  ) {
    if (!state.isConnected) return;
    sendRaw(
      "FORBID_PT,$polygonIndex,$pointIndex,"
      "${xMeters.toStringAsFixed(3)},${yMeters.toStringAsFixed(3)}",
    );
  }

  void sendForbiddenEnd() {
    if (!state.isConnected) return;
    sendRaw("FORBID_END");
  }

  Future<void> sendForbiddenBeginAck(
    int polygonCount,
    List<int> pointCounts,
  ) async {
    final suffix = pointCounts.isEmpty
        ? ''
        : ',${pointCounts.map((v) => v.toString()).join(',')}';
    await _sendRouteAcked("FORBID_BEGIN,$polygonCount$suffix", "FORBID_BEGIN");
  }

  Future<void> sendForbiddenPointAck(
    int polygonIndex,
    int pointIndex,
    double xMeters,
    double yMeters,
  ) async {
    await _sendRouteAcked(
      "FORBID_PT,$polygonIndex,$pointIndex,"
          "${xMeters.toStringAsFixed(3)},${yMeters.toStringAsFixed(3)}",
      "FORBID_PT $polygonIndex/$pointIndex",
    );
  }

  Future<void> sendForbiddenEndAck() async {
    await _sendRouteAcked("FORBID_END", "FORBID_END");
  }

  /// Navigation commands
  Future<void> sendNavStart() async {
    await _sendAcked(
      "NAV_START",
      "NAV_START",
      (ack) => ack == "OK,NAV_START",
    );
  }

  Future<void> sendNavPause() async {
    await _sendAcked(
      "NAV_PAUSE",
      "NAV_PAUSE",
      (ack) => ack == "OK,NAV_PAUSE",
    );
  }

  Future<void> sendNavResume() async {
    await _sendAcked(
      "NAV_RESUME",
      "NAV_RESUME",
      (ack) => ack == "OK,NAV_RESUME",
    );
  }

  Future<void> sendNavStop() async {
    await _sendAcked(
      "NAV_STOP",
      "NAV_STOP",
      (ack) => ack == "OK,NAV_STOP",
    );
  }

  /// Simple Go-To navigation: sends single target coordinates
  /// Robot computes local coords and navigates autonomously
  void sendGoToTarget(double lat, double lon) {
    _log("GO_TO disabled: use boundary route upload");
  }

  void clearLog() {
    state = state.copyWith(rxLog: []);
  }
}
