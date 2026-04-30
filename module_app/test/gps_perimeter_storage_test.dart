import 'dart:convert';

import 'package:flutter_test/flutter_test.dart';
import 'package:hello_flutter/core/gps_perimeter_storage.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  test('saves, lists, and exports GPS perimeter points', () async {
    SharedPreferences.setMockInitialValues({});

    final points = [
      GpsPerimeterPoint(
        lat: 55.12345678,
        lon: 37.12345678,
        hAccM: 0.42,
        at: DateTime.utc(2026, 4, 30, 12),
      ),
      GpsPerimeterPoint(
        lat: 55.12345700,
        lon: 37.12345700,
        hAccM: 0.40,
        at: DateTime.utc(2026, 4, 30, 12, 0, 1),
      ),
      GpsPerimeterPoint(
        lat: 55.12345800,
        lon: 37.12345800,
        hAccM: 0.39,
        at: DateTime.utc(2026, 4, 30, 12, 0, 2),
      ),
    ];

    await GpsPerimeterStorage.save('field test', points);
    final saved = await GpsPerimeterStorage.list();

    expect(saved, hasLength(1));
    expect(saved.first.name, 'field test');
    expect(saved.first.points, hasLength(3));
    expect(saved.first.points.first.lat, 55.12345678);

    final exported = jsonDecode(GpsPerimeterStorage.toExportJson(points))
        as Map<String, dynamic>;
    expect(exported['type'], 'gps_perimeter');
    expect(exported['points'], hasLength(3));
  });
}
