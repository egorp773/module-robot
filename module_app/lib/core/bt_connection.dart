import 'dart:async';
import 'dart:convert';
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:shared_preferences/shared_preferences.dart';

final btConnectionProvider =
    StateNotifierProvider<BtConnectionController, BtConnectionState>((ref) {
  return BtConnectionController(ref);
});

@immutable
class BtDeviceInfo {
  final BluetoothDevice device;
  final String id; // device.remoteId.str
  final String name;
  final int rssi;
  final bool connectable;

  const BtDeviceInfo({
    required this.device,
    required this.id,
    required this.name,
    required this.rssi,
    required this.connectable,
  });
}

@immutable
class BtConnectionState {
  final bool isConnected;
  final bool isBusy; // подключение/скан
  final bool isScanning;
  final bool isConnecting;

  final bool isSupported;
  final bool isBluetoothOn;

  final String deviceName;
  final String? deviceId;
  final int? deviceRssi;

  final List<BtDeviceInfo> devices; // результаты скана
  final List<String> logLines; // RX/TX терминал

  final String? error;

  const BtConnectionState({
    required this.isConnected,
    required this.isBusy,
    required this.isScanning,
    required this.isConnecting,
    required this.isSupported,
    required this.isBluetoothOn,
    required this.deviceName,
    required this.deviceId,
    required this.deviceRssi,
    required this.devices,
    required this.logLines,
    required this.error,
  });

  factory BtConnectionState.initial() => const BtConnectionState(
        isConnected: false,
        isBusy: false,
        isScanning: false,
        isConnecting: false,
        isSupported: true,
        isBluetoothOn: false,
        deviceName: 'ESP32 AutoBot',
        deviceId: null,
        deviceRssi: null,
        devices: [],
        logLines: [],
        error: null,
      );

  BtConnectionState copyWith({
    bool? isConnected,
    bool? isBusy,
    bool? isScanning,
    bool? isConnecting,
    bool? isSupported,
    bool? isBluetoothOn,
    String? deviceName,
    String? deviceId,
    int? deviceRssi,
    List<BtDeviceInfo>? devices,
    List<String>? logLines,
    String? error,
  }) {
    return BtConnectionState(
      isConnected: isConnected ?? this.isConnected,
      isBusy: isBusy ?? this.isBusy,
      isScanning: isScanning ?? this.isScanning,
      isConnecting: isConnecting ?? this.isConnecting,
      isSupported: isSupported ?? this.isSupported,
      isBluetoothOn: isBluetoothOn ?? this.isBluetoothOn,
      deviceName: deviceName ?? this.deviceName,
      deviceId: deviceId ?? this.deviceId,
      deviceRssi: deviceRssi ?? this.deviceRssi,
      devices: devices ?? this.devices,
      logLines: logLines ?? this.logLines,
      error: error,
    );
  }
}

class BtConnectionController extends StateNotifier<BtConnectionState> {
  final Ref ref;

  BtConnectionController(this.ref) : super(BtConnectionState.initial()) {
    _boot();
  }

  StreamSubscription? _adapterSub;
  StreamSubscription? _scanSub;
  StreamSubscription? _isScanningSub;
  StreamSubscription<BluetoothConnectionState>? _deviceConnSub;
  Timer? _connectionMonitorTimer;

  BluetoothDevice? _connectedDevice;

  static const String _prefsLastDeviceIdKey = 'last_bt_device_id';
  static const String _prefsLastDeviceNameKey = 'last_bt_device_name';

  BluetoothCharacteristic? _writeChar; // куда пишем (TX в сторону робота)
  BluetoothCharacteristic? _notifyChar; // откуда читаем (RX от робота)
  StreamSubscription<List<int>>? _notifySub;

  DateTime _lastDriveSend = DateTime.fromMillisecondsSinceEpoch(0);
  String _lastDriveSignature = '';
  int _seq = 0;

  int _nextSeq() {
    _seq = (_seq + 1) & 0xFFFF; // 0..65535
    return _seq;
  }

  static const _maxLog = 220;

  // Nordic UART Service (часто ставят на ESP32 BLE UART)
  static final Guid _nusService = Guid('6E400001-B5A3-F393-E0A9-E50E24DCCA9E');
  static final Guid _nusWrite =
      Guid('6E400002-B5A3-F393-E0A9-E50E24DCCA9E'); // WRITE
  static final Guid _nusNotify =
      Guid('6E400003-B5A3-F393-E0A9-E50E24DCCA9E'); // NOTIFY

  Future<void> _boot() async {
    // поддержка BLE
    final supported = await FlutterBluePlus.isSupported;
    state = state.copyWith(isSupported: supported);

    // состояние адаптера
    _adapterSub = FlutterBluePlus.adapterState.listen((s) {
      final wasOn = state.isBluetoothOn;
      final isOn = s == BluetoothAdapterState.on;
      state = state.copyWith(isBluetoothOn: isOn);

      // Если Bluetooth выключили, отключаемся
      if (wasOn && !isOn && state.isConnected) {
        _handleBluetoothTurnedOff();
      }

      // Если Bluetooth включили и есть сохранённое устройство, пытаемся подключиться
      if (!wasOn && isOn && !state.isConnected && !state.isConnecting) {
        // Небольшая задержка, чтобы система успела инициализировать Bluetooth
        Future.delayed(const Duration(milliseconds: 500), () {
          if (!state.isConnected && !state.isConnecting) {
            _tryAutoConnect();
          }
        });
      }
    });

    // Попытка авто-подключения при старте, если Bluetooth уже включен
    final currentState = await FlutterBluePlus.adapterState.first;
    if (currentState == BluetoothAdapterState.on) {
      state = state.copyWith(isBluetoothOn: true);
      // Небольшая задержка для инициализации
      Future.delayed(const Duration(milliseconds: 800), () {
        if (!state.isConnected && !state.isConnecting) {
          _tryAutoConnect();
        }
      });
    }

    // isScanning
    _isScanningSub = FlutterBluePlus.isScanning.listen((v) {
      state = state.copyWith(
        isScanning: v,
        isBusy: v || state.isConnecting,
      );
    });

    // результаты скана - собираем ВСЕ устройства без фильтров
    _scanSub = FlutterBluePlus.onScanResults.listen((results) {
      final map = <String, BtDeviceInfo>{};
      
      for (final r in results) {
        final dev = r.device;
        // Используем remoteId как уникальный ключ (НЕ MAC адрес)
        final id = dev.remoteId.str;

        // Имя: platformName приоритетнее (на iOS часто единственное доступное)
        // Если оба пустые - показываем "Без имени"
        final platform = dev.platformName.trim();
        final adv = dev.advName.trim();
        final name = platform.isNotEmpty
            ? platform
            : (adv.isNotEmpty ? adv : 'Без имени');

        // Добавляем ВСЕ устройства, даже без имени
        // Обновляем по remoteId (если устройство уже есть - обновляем RSSI)
        map[id] = BtDeviceInfo(
          device: dev,
          id: id,
          name: name,
          rssi: r.rssi,
          connectable: r.advertisementData.connectable,
        );
      }

      // Сортируем по RSSI (сильнее сигнал - выше в списке)
      final list = map.values.toList()
        ..sort((a, b) => b.rssi.compareTo(a.rssi));

      state = state.copyWith(devices: list);
    });
  }

  void _handleBluetoothTurnedOff() {
    _appendLog('… BLUETOOTH TURNED OFF');
    _handleDisconnection('Bluetooth выключен');
    state = state.copyWith(error: 'Bluetooth выключен');
  }

  void _handleDisconnection(String reason) {
    _appendLog('… DISCONNECTED ($reason)');
    _connectionMonitorTimer?.cancel();
    _connectionMonitorTimer = null;
    _connectedDevice = null;
    _writeChar = null;
    _notifyChar = null;
    _notifySub?.cancel();
    _notifySub = null;
    _deviceConnSub?.cancel();
    _deviceConnSub = null;
    state = state.copyWith(
      isConnected: false,
      isConnecting: false,
      isBusy: state.isScanning,
      deviceId: null,
      deviceRssi: null,
      error: reason == 'Bluetooth выключен' ? reason : null,
    );
    // При удалённом отключении НЕ очищаем сохранённое устройство,
    // чтобы можно было автоматически переподключиться
  }

  void _startConnectionMonitor() {
    _connectionMonitorTimer?.cancel();
    _connectionMonitorTimer =
        Timer.periodic(const Duration(seconds: 2), (timer) {
      if (!state.isConnected || _connectedDevice == null) {
        timer.cancel();
        return;
      }

      // Проверяем реальное состояние соединения
      _connectedDevice!.connectionState.first.then((s) {
        if (s == BluetoothConnectionState.disconnected && state.isConnected) {
          _handleDisconnection('потеря соединения');
          // Пытаемся переподключиться, если Bluetooth включен
          if (state.isBluetoothOn && !state.isConnecting) {
            Future.delayed(const Duration(seconds: 1), () {
              _tryAutoConnect();
            });
          }
        }
      }).catchError((e) {
        // Если не можем проверить состояние, считаем что отключено
        if (state.isConnected) {
          _handleDisconnection('ошибка проверки соединения');
        }
      });
    });
  }

  Future<void> _tryAutoConnect() async {
    if (state.isConnected || state.isConnecting) return;
    if (!state.isBluetoothOn) return;

    final prefs = await SharedPreferences.getInstance();
    final lastDeviceId = prefs.getString(_prefsLastDeviceIdKey);
    final lastDeviceName = prefs.getString(_prefsLastDeviceNameKey);

    if (lastDeviceId == null || lastDeviceName == null) {
      _appendLog('… AUTO-CONNECT: нет сохранённого устройства');
      return;
    }

    _appendLog('… AUTO-CONNECT к $lastDeviceName ($lastDeviceId)');

    // Сначала проверяем уже найденные устройства
    try {
      final found = state.devices.firstWhere((d) => d.id == lastDeviceId);
      _appendLog('… AUTO-CONNECT: устройство найдено в списке');
      await connectTo(found);
      return;
    } catch (_) {
      // Устройство не найдено в текущем списке
    }

    // Запускаем скан и пытаемся подключиться после нахождения
    _appendLog('… AUTO-CONNECT: запускаем скан...');
    await startScan(timeout: const Duration(seconds: 10));

    // Ждём появления устройства в результатах скана
    int attempts = 0;
    Timer.periodic(const Duration(milliseconds: 300), (timer) {
      attempts++;
      if (attempts > 33) {
        // ~10 секунд максимум
        timer.cancel();
        _appendLog('✗ AUTO-CONNECT: устройство не найдено за 10 сек');
        stopScan();
        return;
      }

      try {
        final found = state.devices.firstWhere((d) => d.id == lastDeviceId);
        timer.cancel();
        stopScan();
        _appendLog('… AUTO-CONNECT: устройство найдено, подключаемся...');
        connectTo(found);
      } catch (_) {
        // Ещё не найдено, продолжаем ждать
      }
    });
  }

  // --- permissions ---
  Future<bool> _ensurePermissions() async {
    if (kIsWeb) {
      state = state.copyWith(
          error:
              'Web не поддерживает BLE в этом проекте. Запускайте на телефоне.');
      return false;
    }

    // Android: нужно Scan/Connect, на старых — Location
    final perms = <Permission>[
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.locationWhenInUse,
    ];

    final res = await perms.request();
    final denied = res.entries.where((e) => !e.value.isGranted).toList();
    if (denied.isNotEmpty) {
      state = state.copyWith(
          error:
              'Нужны разрешения Bluetooth/Location для сканирования и подключения.');
      return false;
    }
    return true;
  }

  // --- scan ---
  Future<void> startScan(
      {Duration timeout = const Duration(seconds: 5)}) async {
    state = state.copyWith(error: null);

    if (!state.isSupported) {
      state = state.copyWith(
          error: 'Bluetooth LE не поддерживается на устройстве.');
      return;
    }

    final ok = await _ensurePermissions();
    if (!ok) return;

    if (!state.isBluetoothOn) {
      state = state.copyWith(error: 'Включите Bluetooth.');
      return;
    }

    // Останавливаем предыдущий скан, если он идет
    await stopScan();

    // очищаем список перед новым сканом
    state = state.copyWith(devices: []);

    try {
      // Сканируем ВСЕ устройства без фильтров по сервисам
      await FlutterBluePlus.startScan(
        timeout: timeout,
        // НЕ указываем withServices - сканируем все устройства
        // НЕ фильтруем по имени - на iOS имя часто пустое
      );
    } catch (e) {
      state = state.copyWith(error: 'Скан не запустился: $e');
    }
  }

  Future<void> stopScan() async {
    try {
      await FlutterBluePlus.stopScan();
    } catch (_) {}
  }

  Future<void> turnOnAdapterAndroid() async {
    // На Android flutter_blue_plus умеет turnOn (если поддерживается устройством/версией)
    try {
      await FlutterBluePlus.turnOn();
    } catch (_) {}
  }

  // --- connect ---
  Future<void> connectTo(BtDeviceInfo info) async {
    state = state.copyWith(error: null);

    final ok = await _ensurePermissions();
    if (!ok) return;

    if (!state.isBluetoothOn) {
      state = state.copyWith(error: 'Включите Bluetooth.');
      return;
    }

    // если уже подключены — сначала отключимся
    if (state.isConnected) {
      await disconnect();
    }

    await stopScan();

    state = state.copyWith(
      isConnecting: true,
      isBusy: true,
      deviceName: info.name,
      deviceId: info.id,
      deviceRssi: info.rssi,
    );

    try {
      _connectedDevice = info.device;

      await _connectedDevice!.connect(
        license: License.free,
        timeout: const Duration(seconds: 12),
      );

      // следим за состоянием соединения, чтобы отразить внезапный разрыв
      _deviceConnSub?.cancel();
      _deviceConnSub = _connectedDevice!.connectionState
          .listen((BluetoothConnectionState s) {
        if (s == BluetoothConnectionState.disconnected) {
          _handleDisconnection('удалённое отключение');
        } else if (s == BluetoothConnectionState.connected) {
          // Убеждаемся, что состояние синхронизировано
          if (!state.isConnected) {
            state = state.copyWith(isConnected: true);
          }
        }
      });
      _connectedDevice!.cancelWhenDisconnected(_deviceConnSub!);

      // Дополнительная проверка состояния соединения периодически
      _startConnectionMonitor();

      // services - проверяем наличие NUS ПОСЛЕ подключения
      await _connectedDevice!.discoverServices();
      final services = _connectedDevice!.servicesList;

      final pair = _pickUartChars(services);
      _writeChar = pair.$1;
      _notifyChar = pair.$2;

      // Проверяем наличие NUS сервиса и нужных характеристик
      bool hasNusService = false;
      bool hasNusWrite = false;
      bool hasNusNotify = false;
      
      for (final s in services) {
        if (s.uuid == _nusService) {
          hasNusService = true;
          for (final c in s.characteristics) {
            if (c.uuid == _nusWrite) {
              hasNusWrite = true;
            }
            if (c.uuid == _nusNotify) {
              hasNusNotify = true;
            }
          }
          break;
        }
      }

      // Если NUS сервис не найден или нет нужных характеристик - отключаемся
      if (!hasNusService || !hasNusWrite || !hasNusNotify || _writeChar == null || _notifyChar == null) {
        await _connectedDevice!.disconnect();
        _connectedDevice = null;
        _writeChar = null;
        _notifyChar = null;
        
        String errorMsg = 'Устройство подключилось, но NUS сервис не найден. ';
        if (hasNusService && (!hasNusWrite || !hasNusNotify)) {
          errorMsg = 'Устройство подключилось, но отсутствуют нужные характеристики NUS. ';
        }
        errorMsg += 'Убедитесь, что устройство поддерживает Nordic UART Service (UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E).';
        
        state = state.copyWith(
          isConnected: false,
          isConnecting: false,
          isBusy: state.isScanning,
          error: errorMsg,
        );
        _appendLog('✗ NUS SERVICE NOT FOUND');
        return;
      }

      if (_notifyChar != null) {
        await _notifyChar!.setNotifyValue(true);
        _notifySub?.cancel();
        _notifySub = _notifyChar!.onValueReceived.listen((data) {
          final txt = _bytesToText(data);
          if (txt.trim().isEmpty) return;
          _appendLog('← $txt');
        });
        _connectedDevice!.cancelWhenDisconnected(_notifySub!);
      }

      state = state.copyWith(
        isConnected: true,
        isConnecting: false,
        isBusy:
            state.isScanning, // busy только если продолжается скан (обычно нет)
      );

      // Сохраняем последнее подключённое устройство
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString(_prefsLastDeviceIdKey, info.id);
      await prefs.setString(_prefsLastDeviceNameKey, info.name);

      _appendLog('✓ CONNECT ${info.name} (${info.id})');
    } catch (e) {
      state = state.copyWith(
        isConnected: false,
        isConnecting: false,
        isBusy: state.isScanning,
        error: 'Ошибка подключения: $e',
      );
      _appendLog('✗ CONNECT FAIL: $e');

      // подчистим
      try {
        await _connectedDevice?.disconnect();
      } catch (_) {}
      _connectedDevice = null;
      _writeChar = null;
      _notifyChar = null;
      _notifySub?.cancel();
      _notifySub = null;
    }
  }

  Future<void> disconnect() async {
    state = state.copyWith(error: null);

    try {
      _appendLog('… DISCONNECT');
      _deviceConnSub?.cancel();
      _deviceConnSub = null;
      _notifySub?.cancel();
      _notifySub = null;

      if (_notifyChar != null) {
        try {
          await _notifyChar!.setNotifyValue(false);
        } catch (_) {}
      }

      await _connectedDevice?.disconnect();
    } catch (_) {}

    _connectedDevice = null;
    _writeChar = null;
    _notifyChar = null;

    state = state.copyWith(
      isConnected: false,
      isConnecting: false,
      isBusy: state.isScanning,
      deviceId: null,
      deviceRssi: null,
    );

    // Очищаем сохранённое устройство при ручном отключении
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove(_prefsLastDeviceIdKey);
    await prefs.remove(_prefsLastDeviceNameKey);
  }

  // --- write ---
  Future<void> sendRaw(String text) async {
    final t = text.trimRight();
    if (t.isEmpty) return;

    if (!state.isConnected || _connectedDevice == null || _writeChar == null) {
      _appendLog('✗ TX (нет соединения)');
      return;
    }

    try {
      final bytes = utf8.encode('$t\n');
      final withoutResp = _writeChar!.properties.writeWithoutResponse;

      await _writeChar!.write(bytes, withoutResponse: withoutResp);
      _appendLog('→ $t');
    } catch (e) {
      state = state.copyWith(error: 'Ошибка отправки: $e');
      _appendLog('✗ TX FAIL: $e');
    }
  }

  /// Джойстик: x/y в диапазоне -1..1, speedMultiplier для регулировки скорости
  /// Протокол: DRIVE <seq> <x> <y>  (x,y -100..100)
  /// Использует дифференциальное управление: x - поворот, y - движение вперед/назад
  Future<void> sendDrive(double x, double y,
      {double speedMultiplier = 1.0}) async {
    if (!state.isConnected || _connectedDevice == null || _writeChar == null) {
      return;
    }

    // throttle ~ 15-18 Hz (55-66ms между командами)
    final now = DateTime.now();
    final timeSinceLastSend = now.difference(_lastDriveSend).inMilliseconds;
    if (timeSinceLastSend < 55) {
      return; // Слишком рано, пропускаем
    }

    // deadzone - игнорируем очень маленькие значения
    double dz(double v) {
      if (v.abs() < 0.03) return 0.0;
      return v;
    }

    // exponential curve для более плавного управления
    double expo(double v) {
      if (v == 0.0) return 0.0;
      final s = v.sign;
      final a = v.abs();
      // Используем более мягкую кривую (1.5 вместо 1.7)
      return s * math.pow(a, 1.5).toDouble();
    }

    // Применяем deadzone и экспоненту
    final vx = expo(dz(x.clamp(-1.0, 1.0)));
    final vy = expo(dz(y.clamp(-1.0, 1.0)));

    // Применяем множитель скорости
    final vxScaled = vx * speedMultiplier;
    final vyScaled = vy * speedMultiplier;

    int toInt(double v) => (v * 100).round().clamp(-100, 100);

    final xi = toInt(vxScaled);
    final yi = toInt(vyScaled);

    // не слать одинаковые значения подряд
    final sig = '$xi:$yi';
    if (sig == _lastDriveSignature && timeSinceLastSend < 200) {
      return; // То же значение, и прошло меньше 200мс
    }
    _lastDriveSignature = sig;

    final seq = _nextSeq();
    final payload = 'DRIVE $seq $xi $yi';
    _lastDriveSend = now;

    try {
      final bytes = utf8.encode('$payload\n');
      final withoutResp = _writeChar!.properties.writeWithoutResponse;
      await _writeChar!.write(bytes, withoutResponse: withoutResp);
      // Не логируем каждую команду джойстика, чтобы не засорять лог
    } catch (e) {
      // При ошибке отправки обновляем состояние
      if (state.isConnected) {
        state = state.copyWith(error: 'Ошибка отправки команды: $e');
      }
      _appendLog('✗ DRIVE FAIL: $e');
    }
  }

  /// Команды стрелок: F (вперед), B (назад), R (вправо), L (влево), S (стоп)
  /// Отправляет только букву без номера последовательности
  Future<void> sendArrowCommand(String command) async {
    if (!state.isConnected || _connectedDevice == null || _writeChar == null) {
      return;
    }

    // Отправляем только букву (F, B, R, L, S)
    final payload = command;

    try {
      final bytes = utf8.encode('$payload\n');
      final withoutResp = _writeChar!.properties.writeWithoutResponse;
      await _writeChar!.write(bytes, withoutResponse: withoutResp);
      _appendLog('→ $payload');
    } catch (e) {
      if (state.isConnected) {
        state = state.copyWith(error: 'Ошибка отправки команды: $e');
      }
      _appendLog('✗ ARROW FAIL: $e');
    }
  }

  Future<void> sendStop() async {
    if (!state.isConnected || _connectedDevice == null || _writeChar == null) {
      return;
    }

    _lastDriveSignature = '';
    _lastDriveSend =
        DateTime.fromMillisecondsSinceEpoch(0); // Сбрасываем throttle

    // Отправляем только букву S для стоп
    const payload = 'S';

    try {
      final bytes = utf8.encode('$payload\n');
      final withoutResp = _writeChar!.properties.writeWithoutResponse;
      await _writeChar!.write(bytes, withoutResponse: withoutResp);
      _appendLog('→ $payload');
    } catch (e) {
      if (state.isConnected) {
        state = state.copyWith(error: 'Ошибка отправки STOP: $e');
      }
      _appendLog('✗ STOP FAIL: $e');
    }
  }

  void clearLog() => state = state.copyWith(logLines: []);

  void _appendLog(String line) {
    final next = [...state.logLines, line];
    final trimmed =
        next.length <= _maxLog ? next : next.sublist(next.length - _maxLog);
    state = state.copyWith(logLines: trimmed);
  }

  String _bytesToText(List<int> data) {
    try {
      return utf8.decode(data, allowMalformed: true);
    } catch (_) {
      return data.map((b) => b.toRadixString(16).padLeft(2, '0')).join(' ');
    }
  }

  /// Пытаемся найти UART:
  /// 1) NUS (6E40...) если есть
  /// 2) иначе: первый write + первый notify
  (BluetoothCharacteristic?, BluetoothCharacteristic?) _pickUartChars(
      List<BluetoothService> services) {
    BluetoothCharacteristic? w;
    BluetoothCharacteristic? n;

    // 1) NUS
    for (final s in services) {
      if (s.uuid == _nusService) {
        for (final c in s.characteristics) {
          if (c.uuid == _nusWrite &&
              (c.properties.write || c.properties.writeWithoutResponse)) {
            w = c;
          }
          if (c.uuid == _nusNotify &&
              (c.properties.notify || c.properties.indicate)) {
            n = c;
          }
        }
        if (w != null || n != null) return (w, n);
      }
    }

    // 2) fallback
    for (final s in services) {
      for (final c in s.characteristics) {
        if (w == null &&
            (c.properties.write || c.properties.writeWithoutResponse)) {
          w = c;
        }
        if (n == null && (c.properties.notify || c.properties.indicate)) {
          n = c;
        }
      }
    }
    return (w, n);
  }

  @override
  void dispose() {
    _adapterSub?.cancel();
    _scanSub?.cancel();
    _isScanningSub?.cancel();
    _notifySub?.cancel();
    _deviceConnSub?.cancel();
    _connectionMonitorTimer?.cancel();
    super.dispose();
  }
}
