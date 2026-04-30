import 'dart:convert';

import 'package:shared_preferences/shared_preferences.dart';

class GpsPerimeterPoint {
  final double lat;
  final double lon;
  final double? hAccM;
  final DateTime at;

  const GpsPerimeterPoint({
    required this.lat,
    required this.lon,
    required this.at,
    this.hAccM,
  });

  Map<String, dynamic> toJson() => {
        'lat': lat,
        'lon': lon,
        if (hAccM != null) 'hAccM': hAccM,
        'at': at.toIso8601String(),
      };

  factory GpsPerimeterPoint.fromJson(Map<String, dynamic> json) {
    return GpsPerimeterPoint(
      lat: (json['lat'] as num).toDouble(),
      lon: (json['lon'] as num).toDouble(),
      hAccM: (json['hAccM'] as num?)?.toDouble(),
      at: DateTime.tryParse(json['at'] as String? ?? '') ?? DateTime.now(),
    );
  }
}

class GpsPerimeter {
  final String id;
  final String name;
  final DateTime savedAt;
  final List<GpsPerimeterPoint> points;

  const GpsPerimeter({
    required this.id,
    required this.name,
    required this.savedAt,
    required this.points,
  });

  Map<String, dynamic> toJson() => {
        'id': id,
        'name': name,
        'savedAt': savedAt.toIso8601String(),
        'points': points.map((p) => p.toJson()).toList(),
      };

  factory GpsPerimeter.fromJson(Map<String, dynamic> json) {
    return GpsPerimeter(
      id: json['id'] as String,
      name: json['name'] as String? ?? 'GPS периметр',
      savedAt:
          DateTime.tryParse(json['savedAt'] as String? ?? '') ?? DateTime.now(),
      points: ((json['points'] as List?) ?? const [])
          .map((p) => GpsPerimeterPoint.fromJson(p as Map<String, dynamic>))
          .toList(),
    );
  }
}

class GpsPerimeterStorage {
  static const String _key = 'gps_perimeters';

  static Future<List<GpsPerimeter>> list() async {
    final prefs = await SharedPreferences.getInstance();
    final raw = prefs.getStringList(_key) ?? const [];
    final result = <GpsPerimeter>[];

    for (final item in raw) {
      try {
        result.add(GpsPerimeter.fromJson(jsonDecode(item)));
      } catch (_) {
        // Ignore corrupted local entries instead of blocking field tests.
      }
    }

    result.sort((a, b) => b.savedAt.compareTo(a.savedAt));
    return result;
  }

  static Future<void> save(String name, List<GpsPerimeterPoint> points) async {
    final prefs = await SharedPreferences.getInstance();
    final raw = prefs.getStringList(_key) ?? <String>[];
    final now = DateTime.now();
    final perimeter = GpsPerimeter(
      id: now.microsecondsSinceEpoch.toString(),
      name: name.trim().isEmpty ? 'GPS периметр' : name.trim(),
      savedAt: now,
      points: List<GpsPerimeterPoint>.from(points),
    );

    raw.add(jsonEncode(perimeter.toJson()));
    await prefs.setStringList(_key, raw);
  }

  static String toExportJson(List<GpsPerimeterPoint> points) {
    return const JsonEncoder.withIndent('  ').convert({
      'type': 'gps_perimeter',
      'savedAt': DateTime.now().toIso8601String(),
      'points': points.map((p) => p.toJson()).toList(),
    });
  }
}
