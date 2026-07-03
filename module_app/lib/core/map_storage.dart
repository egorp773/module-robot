import 'dart:convert';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../features/manual/manual_control_screen.dart';

/// Сервис для сохранения и загрузки карт (в shared_preferences)
class MapStorage {
  static const String _mapsKey = 'saved_maps';

  /// Сохранить карту в shared_preferences
  static Future<void> saveMap(ManualMapState map) async {
    if (map.mapName == null || map.mapName!.trim().isEmpty) {
      throw Exception('Имя карты не может быть пустым');
    }

    final prefs = await SharedPreferences.getInstance();
    final mapsJson = prefs.getStringList(_mapsKey) ?? [];
    final mapName = map.mapName!.trim();

    // Если карта редактируется (есть mapId), заменяем существующую
    if (map.mapId != null) {
      // Удаляем старую карту с этим ID
      final updatedMaps = mapsJson.where((mapJsonStr) {
        try {
          final json = jsonDecode(mapJsonStr) as Map<String, dynamic>;
          return json['id'] != map.mapId;
        } catch (e) {
          return true;
        }
      }).toList();

      // Создаем обновленную карту с тем же ID
      final mapJson = {
        'id': map.mapId,
        'name': mapName,
        'coordinateType': map.coordinateType ?? 'cell',
        if (map.refLat != null) 'mapOriginLat': map.refLat,
        if (map.refLon != null) 'mapOriginLon': map.refLon,
        if (map.refLat != null) 'refLat': map.refLat,
        if (map.refLon != null) 'refLon': map.refLon,
        if (map.perimeter != null)
          'perimeter': map.perimeter!.map((p) => {'lat': p.$1, 'lon': p.$2}).toList(),
        'robot': {'x': map.robot.dx, 'y': map.robot.dy},
        'zones': map.zones
            .map((z) => z.points.map((p) => {'x': p.dx, 'y': p.dy}).toList())
            .toList(),
        'forbiddens': map.forbiddens
            .map((f) => f.points.map((p) => {'x': p.dx, 'y': p.dy}).toList())
            .toList(),
        'transitions': map.transitions
            .map((t) => t.map((p) => {'x': p.dx, 'y': p.dy}).toList())
            .toList(),
        if (map.startPoint != null)
          'startPoint': {'x': map.startPoint!.dx, 'y': map.startPoint!.dy},
        'savedAt': DateTime.now().toIso8601String(),
      };

      updatedMaps.add(jsonEncode(mapJson));
      await prefs.setStringList(_mapsKey, updatedMaps);
      return;
    }

    // Если создается новая карта, проверяем на дубликаты названий
    for (final mapJsonStr in mapsJson) {
      try {
        final json = jsonDecode(mapJsonStr) as Map<String, dynamic>;
        final existingName = (json['name'] as String?)?.trim();
        if (existingName != null &&
            existingName.toLowerCase() == mapName.toLowerCase()) {
          throw Exception('Карта с названием "$mapName" уже существует');
        }
      } catch (e) {
        if (e.toString().contains('уже существует')) {
          rethrow;
        }
        // Игнорируем ошибки парсинга других карт
      }
    }

    // Создаем новую карту
    final mapJson = {
      'id': DateTime.now().millisecondsSinceEpoch.toString(),
      'name': mapName,
      'coordinateType': map.coordinateType ?? 'cell',
      if (map.refLat != null) 'mapOriginLat': map.refLat,
      if (map.refLon != null) 'mapOriginLon': map.refLon,
      if (map.refLat != null) 'refLat': map.refLat,
      if (map.refLon != null) 'refLon': map.refLon,
      if (map.perimeter != null)
        'perimeter': map.perimeter!.map((p) => {'lat': p.$1, 'lon': p.$2}).toList(),
      'robot': {'x': map.robot.dx, 'y': map.robot.dy},
      'zones': map.zones
          .map((z) => z.points.map((p) => {'x': p.dx, 'y': p.dy}).toList())
          .toList(),
      'forbiddens': map.forbiddens
          .map((f) => f.points.map((p) => {'x': p.dx, 'y': p.dy}).toList())
          .toList(),
      'transitions': map.transitions
          .map((t) => t.map((p) => {'x': p.dx, 'y': p.dy}).toList())
          .toList(),
      if (map.startPoint != null)
        'startPoint': {'x': map.startPoint!.dx, 'y': map.startPoint!.dy},
      'savedAt': DateTime.now().toIso8601String(),
    };

    mapsJson.add(jsonEncode(mapJson));
    await prefs.setStringList(_mapsKey, mapsJson);

    // Проверяем, что карта сохранилась
    final saved = prefs.getStringList(_mapsKey) ?? [];
    debugPrint('DEBUG: Map saved. Total maps: ${saved.length}');
    debugPrint('DEBUG: Saved map name: ${mapJson['name']}');
  }

  /// Загрузить карту по ID
  static Future<ManualMapState?> loadMap(String mapId) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final mapsJson = prefs.getStringList(_mapsKey) ?? [];

      for (final mapJsonStr in mapsJson) {
        final json = jsonDecode(mapJsonStr) as Map<String, dynamic>;
        if (json['id'] == mapId) {
          return mapFromJson(json);
        }
      }

      return null;
    } catch (e) {
      debugPrint('Ошибка загрузки карты: $e');
      return null;
    }
  }

  /// Преобразовать JSON в ManualMapState
  static ManualMapState mapFromJson(Map<String, dynamic> json) {
    final robotJson = json['robot'] as Map<String, dynamic>;
    final robot = Offset(
      (robotJson['x'] as num).toDouble(),
      (robotJson['y'] as num).toDouble(),
    );

    final zones = (json['zones'] as List).map((z) {
      final points = (z as List).map((p) {
        final pointJson = p as Map<String, dynamic>;
        return Offset(
          (pointJson['x'] as num).toDouble(),
          (pointJson['y'] as num).toDouble(),
        );
      }).toList();
      return PolyShape(points);
    }).toList();

    final forbiddens = (json['forbiddens'] as List).map((f) {
      final points = (f as List).map((p) {
        final pointJson = p as Map<String, dynamic>;
        return Offset(
          (pointJson['x'] as num).toDouble(),
          (pointJson['y'] as num).toDouble(),
        );
      }).toList();
      return PolyShape(points);
    }).toList();

    final transitions = (json['transitions'] as List).map((t) {
      return (t as List).map((p) {
        final pointJson = p as Map<String, dynamic>;
        return Offset(
          (pointJson['x'] as num).toDouble(),
          (pointJson['y'] as num).toDouble(),
        );
      }).toList();
    }).toList();

    Offset? startPoint;
    if (json['startPoint'] != null) {
      final startPointJson = json['startPoint'] as Map<String, dynamic>;
      startPoint = Offset(
        (startPointJson['x'] as num).toDouble(),
        (startPointJson['y'] as num).toDouble(),
      );
    }

    // Parse GPS data
    final coordinateType = json['coordinateType'] as String? ?? 'cell';
    // mapOrigin* are the canonical persistent GPS anchor. ref* remain as a
    // backward-compatible alias for maps saved by older app versions.
    final refLat =
        ((json['mapOriginLat'] ?? json['refLat']) as num?)?.toDouble();
    final refLon =
        ((json['mapOriginLon'] ?? json['refLon']) as num?)?.toDouble();

    List<(double, double)>? perimeter;
    if (json['perimeter'] != null) {
      perimeter = (json['perimeter'] as List).map((p) {
        final pointJson = p as Map<String, dynamic>;
        return ((pointJson['lat'] as num).toDouble(), (pointJson['lon'] as num).toDouble());
      }).toList();
    }

    return ManualMapState(
      stage: ManualStage.idle,
      mapName: json['name'] as String?,
      kind: null,
      robot: robot,
      zoom: 1.0,
      pan: Offset.zero,
      zones: zones,
      forbiddens: forbiddens,
      transitions: transitions,
      stroke: const [],
      startPoint: startPoint,
      mapId: json['id'] as String?,
      coordinateType: coordinateType,
      refLat: refLat,
      refLon: refLon,
      perimeter: perimeter,
    );
  }

  /// Получить список всех сохраненных карт
  static Future<List<MapInfo>> listMaps() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final mapsJson = prefs.getStringList(_mapsKey) ?? [];

      debugPrint(
          'DEBUG: Loading maps. Found ${mapsJson.length} maps in storage');

      final maps = <MapInfo>[];
      for (final mapJsonStr in mapsJson) {
        try {
          final json = jsonDecode(mapJsonStr) as Map<String, dynamic>;
          final savedAt = json['savedAt'] as String?;

          maps.add(MapInfo(
            id: json['id'] as String,
            name: json['name'] as String? ?? 'Без названия',
            savedAt: savedAt != null ? DateTime.parse(savedAt) : null,
            mapData: json,
          ));
          debugPrint('DEBUG: Loaded map: ${json['name']} (id: ${json['id']})');
        } catch (e) {
          debugPrint('Ошибка чтения карты: $e');
        }
      }

      debugPrint('DEBUG: Total maps loaded: ${maps.length}');

      // Сортируем по дате сохранения (новые первыми)
      maps.sort((a, b) {
        if (a.savedAt == null && b.savedAt == null) return 0;
        if (a.savedAt == null) return 1;
        if (b.savedAt == null) return -1;
        return b.savedAt!.compareTo(a.savedAt!);
      });

      return maps;
    } catch (e) {
      debugPrint('Ошибка получения списка карт: $e');
      return [];
    }
  }

  /// Удалить карту по ID
  static Future<bool> deleteMap(String mapId) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final mapsJson = prefs.getStringList(_mapsKey) ?? [];

      final updatedMaps = mapsJson.where((mapJsonStr) {
        try {
          final json = jsonDecode(mapJsonStr) as Map<String, dynamic>;
          return json['id'] != mapId;
        } catch (e) {
          return true; // Оставляем некорректные записи
        }
      }).toList();

      await prefs.setStringList(_mapsKey, updatedMaps);
      return true;
    } catch (e) {
      debugPrint('Ошибка удаления карты: $e');
      return false;
    }
  }
}

/// Информация о сохраненной карте
class MapInfo {
  final String id;
  final String name;
  final DateTime? savedAt;
  final Map<String, dynamic> mapData; // Полные данные карты для превью

  MapInfo({
    required this.id,
    required this.name,
    this.savedAt,
    required this.mapData,
  });

  /// Загрузить полную карту
  Future<ManualMapState?> load() async {
    return MapStorage.mapFromJson(mapData);
  }
}
