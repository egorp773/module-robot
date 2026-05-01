import 'dart:async';

import 'package:connectivity_plus/connectivity_plus.dart';
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

  // IMU data
  final double? imuYaw;

  // Navigation data
  final String? navState; // IDLE, RUNNING, PAUSED, DONE, ERROR
  final int? navWpIndex;
  final int? navWpTotal;
  final double? navDistToWp;

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
    this.imuYaw,
    this.navState,
    this.navWpIndex,
    this.navWpTotal,
    this.navDistToWp,
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
        imuYaw: null,
        navState: null,
        navWpIndex: null,
        navWpTotal: null,
        navDistToWp: null,
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
    double? imuYaw,
    String? navState,
    int? navWpIndex,
    int? navWpTotal,
    double? navDistToWp,
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
      imuYaw: imuYaw ?? this.imuYaw,
      navState: navState ?? this.navState,
      navWpIndex: navWpIndex ?? this.navWpIndex,
      navWpTotal: navWpTotal ?? this.navWpTotal,
      navDistToWp: navDistToWp ?? this.navDistToWp,
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

  WifiPingCheckNotifier() : super(true) {
    _init();
  }

  Future<void> _init() async {
    if (_initialized) return;
    try {
      final prefs = await SharedPreferences.getInstance();
      final value = prefs.getBool(_key);
      print('DEBUG: Loading wifi ping check from prefs: $value');
      if (value != null) {
        state = value;
        print('DEBUG: Set wifi ping check state to: $value');
      } else {
        print('DEBUG: No saved value, using default: true');
      }
      _initialized = true;
    } catch (e) {
      print('DEBUG: Error loading wifi ping check: $e');
      _initialized = true;
      // Оставляем значение по умолчанию (true)
    }
  }

  // Публичный метод для инициализации (если нужно вызвать извне)
  Future<void> ensureInitialized() => _init();

  Future<void> setEnabled(bool enabled) async {
    print('DEBUG: setEnabled called with: $enabled');
    state = enabled;
    print('DEBUG: State updated to: $state');
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setBool(_key, enabled);
      // Проверяем, что значение сохранилось
      final saved = prefs.getBool(_key);
      print('DEBUG: Saved wifi ping check: $enabled, read back: $saved');
      if (saved != enabled) {
        print('DEBUG: WARNING! Saved value does not match!');
      }
    } catch (e) {
      print('DEBUG: Error saving wifi ping check: $e');
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

  Completer<void>? _pongWaiter;

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

    // Если Wi-Fi отключился и приложение было подключено, отключаемся
    if (!hasWifi && state.isConnected) {
      _log("× Wi-Fi disconnected, disconnecting...");
      disconnect(error: "Wi-Fi отключен");
    }
  }

  void _log(String line) {
    final next = List<String>.from(state.rxLog);
    next.add(line);
    if (next.length > 200) next.removeRange(0, next.length - 200);
    state = state.copyWith(rxLog: next);
  }

  /// Минимальная проверка Wi-Fi: пробуем подключиться к WebSocket
  /// Если WebSocket подключается - значит Wi-Fi есть
  Future<bool> _testWebSocketConnection() async {
    WebSocketChannel? testChannel;
    StreamSubscription? testSub;
    try {
      _log("→ Testing WebSocket connection to $_wsUri");
      testChannel = WebSocketChannel.connect(_wsUri);

      final testCompleter = Completer<bool>();
      bool gotMessage = false;

      testSub = testChannel.stream.listen(
        (msg) {
          if (!gotMessage) {
            gotMessage = true;
            _log("← Test received: ${msg.toString().trim()}");
            if (!testCompleter.isCompleted) {
              testCompleter.complete(true);
            }
          }
        },
        onError: (e) {
          _log("× Test WebSocket error: $e");
          if (!testCompleter.isCompleted) {
            testCompleter.complete(false);
          }
        },
        onDone: () {
          _log("× Test WebSocket closed");
          if (!testCompleter.isCompleted) {
            testCompleter.complete(false);
          }
        },
        cancelOnError: false,
      );

      // Ждем либо первого сообщения, либо ошибки (максимум 3 секунды для теста)
      final success = await testCompleter.future.timeout(
        const Duration(seconds: 3),
        onTimeout: () {
          _log("× Test WebSocket timeout");
          return false;
        },
      );

      // Закрываем тестовое соединение
      await testSub.cancel();
      testSub = null;
      try {
        testChannel.sink.close();
      } catch (_) {}
      testChannel = null;

      if (success) {
        _log("✓ WebSocket test: connection successful");
      } else {
        _log("× WebSocket test: connection failed");
      }
      return success;
    } catch (e) {
      _log("× WebSocket test error: $e");
      try {
        await testSub?.cancel();
        testChannel?.sink.close();
      } catch (_) {}
      return false;
    }
  }

  Future<void> connect({bool skipPreflight = false}) async {
    if (state.isConnecting || state.isConnected) return;

    state = state.copyWith(isConnecting: true, error: null);
    _log("=== CONNECT START ===");
    await _ref.read(wifiRobotHostProvider.notifier).ensureInitialized();

    final bool pingCheckEnabled;
    if (skipPreflight) {
      pingCheckEnabled = false;
      _log("→ Fast connect: Wi-Fi preflight skipped");
    } else {
      // Проверяем настройку проверки Wi-Fi
      // Убеждаемся, что настройка загружена
      final notifier = _ref.read(wifiPingCheckProvider.notifier);
      await notifier.ensureInitialized();

      // Читаем актуальное значение после инициализации
      pingCheckEnabled = _ref.read(wifiPingCheckProvider);
      _log("→ Wi-Fi check setting: $pingCheckEnabled");
    }

    if (pingCheckEnabled) {
      // Проверка включена - выполняем минимальную проверку WebSocket
      _log("→ Wi-Fi check enabled, testing WebSocket connection...");
      final wsTestOk = await _testWebSocketConnection();
      if (!wsTestOk) {
        state = state.copyWith(
          isConnecting: false,
          isConnected: false,
          error:
              "Не вижу робота по Wi-Fi. Проверь что iPhone/Android подключён к сети Robot.",
        );
        _log("=== CONNECT FAIL: WebSocket test failed ===");
        return;
      }
      _log("✓ WebSocket test passed, Wi-Fi is available");
    } else {
      _log("→ Wi-Fi preflight disabled, connecting WebSocket directly...");
    }

    try {
      _log("→ WS connect $_wsUri");
      WebSocketChannel ch;
      try {
        ch = WebSocketChannel.connect(_wsUri);
        _log("→ WebSocket channel created");
      } catch (e) {
        _log("× Failed to create WebSocket channel: $e");
        await disconnect(error: "Не удалось создать WebSocket соединение: $e");
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
          final msgStr = msg.toString().trim();
          _log("← $msgStr");
          final upperMsg = msgStr.toUpperCase();

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

                if (lat != null && lon != null) {
                  state = state.copyWith(
                    gpsLat: lat,
                    gpsLon: lon,
                    gpsHeading: heading,
                    gpsFixType: fixType,
                    gpsAccuracy: hAcc,
                    gpsReceivedAt: DateTime.now(),
                  );
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
                    gpsReceivedAt: DateTime.now(),
                  );
                }
              }
            } catch (e) {
              _log("× Failed to parse GPSDBG: $e");
            }
          }

          // RTCM,<bytesTotal>,<ageMs>
          if (msgStr.startsWith("RTCM,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 3) {
                final bytes = int.tryParse(parts[1]);
                final ageMs = int.tryParse(parts[2]);
                state = state.copyWith(
                  rtcmBytes: bytes,
                  rtcmAgeMs: ageMs,
                );
              }
            } catch (e) {
              _log("× Failed to parse RTCM: $e");
            }
          }

          // Парсинг IMU,<yaw>
          if (msgStr.startsWith("IMU,")) {
            try {
              final parts = msgStr.split(",");
              if (parts.length >= 2) {
                final yaw = double.tryParse(parts[1]);
                if (yaw != null) {
                  state = state.copyWith(imuYaw: yaw);
                }
              }
            } catch (e) {
              _log("× Failed to parse IMU: $e");
            }
          }

          // Парсинг NAV,<state>,<wpIdx>,<wpTotal>,<distToWp>
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
                );
              }
            } catch (e) {
              _log("× Failed to parse NAV: $e");
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

          // Если получили STATE,CONNECTED - соединение установлено
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
        },
        onDone: () {
          _log("× WS stream closed (onDone)");
          if (!connectionCompleter.isCompleted) {
            _log(
                "× Completing connectionCompleter with false due to stream closed");
            connectionCompleter.complete(false);
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
          await disconnect(error: "Не удалось установить WebSocket соединение");
          return;
        }

        // Робот уже отправил STATE,CONNECTED, значит соединение установлено
        // Не требуем PONG, так как робот может не отвечать на PING текстовым сообщением
        _log("✓ Connection established, setting isConnected = true");
        state =
            state.copyWith(isConnecting: false, isConnected: true, error: null);
        _log("=== CONNECT OK ===");

        // Опционально: отправляем PING для проверки (но не ждем ответа)
        _log("→ Sending PING (optional)");
        sendRaw("PING");
      } catch (e) {
        _log("=== CONNECT FAIL: $e ===");
        await disconnect(error: "Не удалось подключиться по WebSocket: $e");
      }
    } catch (e) {
      _log("=== CONNECT FAIL: $e ===");
      await disconnect(error: "Не удалось подключиться по WebSocket: $e");
    }
  }

  Future<void> disconnect({String? error}) async {
    state = state.copyWith(
      isConnecting: false,
      isConnected: false,
      error: error,
      batteryPercent: null, // Сбрасываем данные батареи при отключении
    );

    _pongWaiter = null;

    await _sub?.cancel();
    _sub = null;

    try {
      _channel?.sink.close();
    } catch (_) {}

    _channel = null;
    _log("=== DISCONNECTED ===");
  }

  @override
  void dispose() {
    _connectivitySubscription?.cancel();
    _sub?.cancel();
    _channel?.sink.close();
    super.dispose();
  }

  void sendRaw(String text) {
    final ch = _channel;
    if (ch == null) {
      _log("× sendRaw failed: channel is null");
      return;
    }
    try {
      _log("→ $text");
      ch.sink.add(text);
    } catch (e) {
      _log("× sendRaw error: $e");
      disconnect(error: "Ошибка отправки: $e");
    }
  }

  /// left/right: -100..100
  void sendMove(int left, int right) {
    if (!state.isConnected) return;
    left = left.clamp(-100, 100);
    right = right.clamp(-100, 100);
    sendRaw("M,$left,$right");
  }

  void sendStop() {
    if (!state.isConnected) return;
    sendRaw("STOP");
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
  void sendRouteBegin(int count) {
    if (!state.isConnected) return;
    sendRaw("ROUTE_BEGIN,$count");
  }

  void sendRouteWaypoint(int index, double lat, double lon) {
    if (!state.isConnected) return;
    sendRaw(
        "ROUTE_WP,$index,${lat.toStringAsFixed(8)},${lon.toStringAsFixed(8)}");
  }

  void sendRouteEnd() {
    if (!state.isConnected) return;
    sendRaw("ROUTE_END");
  }

  /// Navigation commands
  void sendNavStart() {
    if (!state.isConnected) return;
    sendRaw("NAV_START");
  }

  void sendNavPause() {
    if (!state.isConnected) return;
    sendRaw("NAV_PAUSE");
  }

  void sendNavResume() {
    if (!state.isConnected) return;
    sendRaw("NAV_RESUME");
  }

  void sendNavStop() {
    if (!state.isConnected) return;
    sendRaw("NAV_STOP");
  }

  void clearLog() {
    state = state.copyWith(rxLog: []);
  }
}
