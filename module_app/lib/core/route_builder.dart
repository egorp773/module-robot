// lib/core/route_builder.dart

import 'dart:math' as math;
import 'dart:ui';
import 'package:flutter/foundation.dart';

import '../features/manual/manual_control_screen.dart';

enum RouteErrorCode {
  noStart,
  noTransitions,
  blueIntersectsForbidden,
  startToBlueBlocked,
  mowingFailed,
  zoneInvalid,
}

class _Poly {
  final List<Offset> points;
  final int id;
  const _Poly(this.points, {this.id = 0});
}

class _PickTransitionResult {
  final int transitionIndex;
  final int segIndex;
  final Offset snapPoint;
  final double distance;
  const _PickTransitionResult(this.transitionIndex, this.segIndex, this.snapPoint, this.distance);
}

class _Traversal {
  final List<Offset> pts;
  final List<double> cumulativeDistances;
  final List<int> originalIndices;
  const _Traversal(this.pts, this.cumulativeDistances, this.originalIndices);
}

class _BoundingBox {
  final double minX;
  final double maxX;
  final double minY;
  final double maxY;
  const _BoundingBox(this.minX, this.maxX, this.minY, this.maxY);
  
  double get width => maxX - minX;
  double get height => maxY - minY;
  Offset get center => Offset((minX + maxX) / 2, (minY + maxY) / 2);
}

class _LineInterval {
  final double start;
  final double end;
  const _LineInterval(this.start, this.end);
  
  double get length => end - start;
}

class _RouteLogger {
  final bool enabled;
  
  _RouteLogger(this.enabled);
  
  void log(String message) {
    if (enabled && kDebugMode) {
      print('[RouteBuilder] $message');
    }
  }
}

class RouteBuilder {
  static (List<Offset>, RouteErrorCode?) buildRoute(
    ManualMapState mapState, {
    double lineStepCells = 1.0,
    double cellSize = 1.0,
    bool debugPrint = false,
  }) {
    final logger = _RouteLogger(debugPrint);
    
    logger.log('=== НАЧАЛО ПОСТРОЕНИЯ МАРШРУТА ===');
    
    // 1. Инициализация данных
    final startPoint = mapState.startPoint ?? mapState.robot;
    logger.log('Стартовая точка: $startPoint');
    
    // Проверка стартовой точки
    if (startPoint.dx.isNaN || startPoint.dy.isNaN) {
      logger.log('ОШИБКА: Неверная стартовая точка');
      return (const [], RouteErrorCode.noStart);
    }
    
    // Получение данных карты
    final greenZones = mapState.zones.map((z) => _Poly(z.points, id: mapState.zones.indexOf(z))).toList();
    final forbiddenZones = mapState.forbiddens.map((f) => _Poly(f.points)).toList();
    final blueTransitions = mapState.transitions;
    
    logger.log('Зеленых зон: ${greenZones.length}');
    logger.log('Запретных зон: ${forbiddenZones.length}');
    logger.log('Синих линий: ${blueTransitions.length}');
    
    // 2. Проверка начальных условий
    if (_isPointInAnyPolygon(startPoint, forbiddenZones)) {
      logger.log('ОШИБКА: Стартовая точка находится в запретной зоне');
      return (const [], RouteErrorCode.noStart);
    }
    
    if (blueTransitions.isEmpty) {
      logger.log('ОШИБКА: Нет синих линий для следования');
      return (const [], RouteErrorCode.noTransitions);
    }
    
    // 3. Поиск ближайшей синей линии и точки привязки
    final transitionPick = _findNearestTransitionPoint(startPoint, blueTransitions);
    if (transitionPick == null) {
      logger.log('ОШИБКА: Не удалось найти ближайшую синюю линию');
      return (const [], RouteErrorCode.noTransitions);
    }
    
    logger.log('Выбрана синяя линия #${transitionPick.transitionIndex}, точка: ${transitionPick.snapPoint}');
    
    // 4. Проверка синей линии на пересечение с запретными зонами
    final selectedBlueLine = blueTransitions[transitionPick.transitionIndex];
    if (_doesPolylineIntersectForbidden(selectedBlueLine, forbiddenZones)) {
      logger.log('ОШИБКА: Синяя линия пересекает запретную зону');
      return (const [], RouteErrorCode.blueIntersectsForbidden);
    }
    
    // 5. Построение маршрута от старта до синей линии
    final route = <Offset>[startPoint];
    
    if ((startPoint - transitionPick.snapPoint).distance > cellSize * 0.01) {
      logger.log('Добавление сегмента: старт -> синяя линия');
      if (_doesSegmentIntersectForbidden(startPoint, transitionPick.snapPoint, forbiddenZones)) {
        logger.log('ОШИБКА: Путь от старта до синей линии пересекает запретную зону');
        return (const [], RouteErrorCode.startToBlueBlocked);
      }
      route.add(transitionPick.snapPoint);
    }
    
    // 6. Определение направления движения по синей линии
    final traversal = _buildTraversalAlongBlueLine(
      blueLine: selectedBlueLine,
      entryPoint: transitionPick.snapPoint,
      entrySegmentIndex: transitionPick.segIndex,
      forward: true,
    );
    
    logger.log('Траектория по синей линии: ${traversal.pts.length} точек');
    
    // 7. Поиск зеленых зон вдоль синей линии
    final zoneEntries = _findZoneEntriesAlongTraversal(traversal, greenZones);
    logger.log('Найдено входов в зеленые зоны: ${zoneEntries.length}');
    
    // 8. Основной цикл построения маршрута
    final visitedZones = <int>{};
    int currentTraversalIndex = 0;
    Offset currentPosition = route.last;
    
    while (currentTraversalIndex < traversal.pts.length - 1) {
      final nextPoint = traversal.pts[currentTraversalIndex + 1];
      final segmentStart = currentPosition;
      final segmentEnd = nextPoint;
      
      // Проверка на пересечение с запретной зоной
      if (_doesSegmentIntersectForbidden(segmentStart, segmentEnd, forbiddenZones)) {
        logger.log('ОШИБКА: Сегмент синей линии пересекает запретную зону');
        return (_simplifyRoute(route, minDistance: cellSize * 0.05), RouteErrorCode.blueIntersectsForbidden);
      }
      
      // Поиск входа в зеленую зону на этом сегменте
      final zoneEntry = _findZoneEntryOnSegment(
        segmentStart, 
        segmentEnd, 
        greenZones, 
        visitedZones,
      );
      
      if (zoneEntry != null) {
        final (zoneIndex, entryPoint) = zoneEntry;
        logger.log('ВХОД В ЗЕЛЕНУЮ ЗОНУ #$zoneIndex в точке $entryPoint');
        
        // Добавление точки входа
        if ((route.last - entryPoint).distance > cellSize * 0.01) {
          route.add(entryPoint);
        }
        currentPosition = entryPoint;
        
        // Построение маршрута уборки внутри зоны
        final zonePolygon = greenZones[zoneIndex].points;
        final mowingRoute = _buildCompleteMowingRoute(
          zone: zonePolygon,
          startPoint: entryPoint,
          lineStep: lineStepCells * cellSize,
          forbiddenZones: forbiddenZones,
          logger: logger,
        );
        
        if (mowingRoute.isEmpty) {
          logger.log('ОШИБКА: Не удалось построить маршрут уборки для зоны #$zoneIndex');
          return (_simplifyRoute(route, minDistance: cellSize * 0.05), RouteErrorCode.mowingFailed);
        }
        
        // Добавление маршрута уборки (пропускаем первую точку, так как это entryPoint)
        for (final point in mowingRoute.skip(1)) {
          if ((route.last - point).distance > cellSize * 0.01) {
            route.add(point);
          }
        }
        currentPosition = route.last;
        
        // Возврат на синюю линию
        final rejoinPoint = _findRejoinPointOnBlueLine(
          currentPosition: currentPosition,
          blueLine: selectedBlueLine,
          startSegmentIndex: transitionPick.segIndex,
          forward: true,
          forbiddenZones: forbiddenZones,
        );
        
        if (rejoinPoint != null) {
          if ((route.last - rejoinPoint).distance > cellSize * 0.01) {
            route.add(rejoinPoint);
          }
          currentPosition = rejoinPoint;
          // Находим ближайший индекс на синей линии
          currentTraversalIndex = _findClosestPointIndex(rejoinPoint, traversal.pts);
        }
        
        visitedZones.add(zoneIndex);
        logger.log('Зона #$zoneIndex обработана, маршрут уборки: ${mowingRoute.length} точек');
      } else {
        // Просто движемся по синей линии
        if ((route.last - segmentEnd).distance > cellSize * 0.01) {
          route.add(segmentEnd);
        }
        currentPosition = segmentEnd;
        currentTraversalIndex++;
      }
    }
    
    // 9. Финальная обработка маршрута
    final optimizedRoute = _optimizeRoute(route, minDistance: cellSize * 0.05);
    
    logger.log('=== ЗАВЕРШЕНИЕ ПОСТРОЕНИЯ МАРШРУТА ===');
    logger.log('Итоговый маршрут: ${optimizedRoute.length} точек');
    logger.log('Обработано зеленых зон: ${visitedZones.length}');
    
    return (optimizedRoute, null);
  }
  
  // ===========================================================================
  // ОСНОВНАЯ ФУНКЦИЯ ДЛЯ ПОСТРОЕНИЯ ЗМЕЙКИ
  // ===========================================================================
  
  static List<Offset> _buildCompleteMowingRoute({
    required List<Offset> zone,
    required Offset startPoint,
    required double lineStep,
    required List<_Poly> forbiddenZones,
    required _RouteLogger logger,
  }) {
    logger.log('Начинаем построение змейки для зоны с ${zone.length} вершинами');
    
    if (zone.isEmpty || zone.length < 3) {
      logger.log('ОШИБКА: Невалидная зона');
    return const [];
  }

    // 1. Получаем bounding box зоны
    final bbox = _calculateBoundingBox(zone);
    logger.log('Bounding box: ${bbox.minX}, ${bbox.minY} - ${bbox.maxX}, ${bbox.maxY}');
    
    // 2. Определяем ориентацию змейки
    final double width = bbox.width;
    final double height = bbox.height;
    final bool useHorizontal = height <= width; // Если высота меньше ширины, делаем горизонтальные линии
    logger.log('Ориентация змейки: ${useHorizontal ? "горизонтальная" : "вертикальная"}');
    
    // 3. Вычисляем количество линий
    final int numLines = useHorizontal 
        ? math.max(1, (height / lineStep).ceil())
        : math.max(1, (width / lineStep).ceil());
    logger.log('Количество линий: $numLines, шаг: $lineStep');
    
    // 4. Строим маршрут
    final List<Offset> route = <Offset>[startPoint];
    bool forwardDirection = true; // Направление движения
    
    if (useHorizontal) {
      // Горизонтальные линии
      for (int i = 0; i < numLines; i++) {
        final double y = bbox.minY + (i + 0.5) * lineStep;
        logger.log('Линия $i: y = $y');
        
        // Получаем интервалы этой линии внутри зоны
        final intervals = _getHorizontalIntervals(y, zone);
        
        if (intervals.isNotEmpty) {
          // Выбираем самый длинный интервал
          intervals.sort((a, b) => b.length.compareTo(a.length));
          final interval = intervals.first;
          
          // Определяем начальную и конечную точки для этой линии
          final double startX = forwardDirection ? interval.start : interval.end;
          final double endX = forwardDirection ? interval.end : interval.start;
          
          final Offset lineStart = Offset(startX, y);
          final Offset lineEnd = Offset(endX, y);
          
          // Добавляем переход к началу линии
          if (_isSegmentSafe(route.last, lineStart, forbiddenZones)) {
            if ((route.last - lineStart).distance > lineStep * 0.1) {
              route.add(lineStart);
            }
          }
          
          // Добавляем саму линию
          if (_isSegmentSafe(lineStart, lineEnd, forbiddenZones)) {
            route.add(lineEnd);
          }
          
          // Меняем направление для следующей линии
          forwardDirection = !forwardDirection;
        }
      }
    } else {
      // Вертикальные линии
      for (int i = 0; i < numLines; i++) {
        final double x = bbox.minX + (i + 0.5) * lineStep;
        logger.log('Линия $i: x = $x');
        
        // Получаем интервалы этой линии внутри зоны
        final intervals = _getVerticalIntervals(x, zone);
        
        if (intervals.isNotEmpty) {
          // Выбираем самый длинный интервал
          intervals.sort((a, b) => b.length.compareTo(a.length));
          final interval = intervals.first;
          
          // Определяем начальную и конечную точки для этой линии
          final double startY = forwardDirection ? interval.start : interval.end;
          final double endY = forwardDirection ? interval.end : interval.start;
          
          final Offset lineStart = Offset(x, startY);
          final Offset lineEnd = Offset(x, endY);
          
          // Добавляем переход к началу линии
          if (_isSegmentSafe(route.last, lineStart, forbiddenZones)) {
            if ((route.last - lineStart).distance > lineStep * 0.1) {
              route.add(lineStart);
            }
          }
          
          // Добавляем саму линию
          if (_isSegmentSafe(lineStart, lineEnd, forbiddenZones)) {
            route.add(lineEnd);
          }
          
          // Меняем направление для следующей линии
          forwardDirection = !forwardDirection;
        }
      }
    }
    
    // 5. Возвращаем построенный маршрут
    logger.log('Змейка построена: ${route.length} точек');
    return route;
  }
  
  // ===========================================================================
  // ГЕОМЕТРИЧЕСКИЕ ФУНКЦИИ
  // ===========================================================================
  
  static bool _isPointInPolygon(Offset point, List<Offset> polygon) {
    if (polygon.length < 3) return false;
    
    bool isInside = false;
    final double x = point.dx;
    final double y = point.dy;
    
    int j = polygon.length - 1;
    for (int i = 0; i < polygon.length; i++) {
      final xi = polygon[i].dx;
      final yi = polygon[i].dy;
      final xj = polygon[j].dx;
      final yj = polygon[j].dy;
      
      final bool intersect = ((yi > y) != (yj > y)) &&
          (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
      
      if (intersect) {
        isInside = !isInside;
      }
      j = i;
    }
    
    return isInside;
  }
  
  static bool _isPointInAnyPolygon(Offset point, List<_Poly> polygons) {
    for (final poly in polygons) {
      if (_isPointInPolygon(point, poly.points)) {
        return true;
      }
    }
    return false;
  }
  
  static bool _doesSegmentIntersectForbidden(Offset a, Offset b, List<_Poly> forbiddenZones) {
    // Простая проверка: если любая точка сегмента находится в запретной зоне
    const int samples = 10;
    for (int i = 0; i <= samples; i++) {
      final t = i / samples;
      final point = Offset(
        a.dx + (b.dx - a.dx) * t,
        a.dy + (b.dy - a.dy) * t,
      );
      if (_isPointInAnyPolygon(point, forbiddenZones)) {
        return true;
      }
    }
    return false;
  }
  
  static bool _isSegmentSafe(Offset a, Offset b, List<_Poly> forbiddenZones) {
    return !_doesSegmentIntersectForbidden(a, b, forbiddenZones);
  }
  
  static bool _doesPolylineIntersectForbidden(List<Offset> polyline, List<_Poly> forbiddenZones) {
    for (int i = 0; i < polyline.length - 1; i++) {
      if (_doesSegmentIntersectForbidden(polyline[i], polyline[i + 1], forbiddenZones)) {
        return true;
      }
    }
    return false;
  }
  
  static _BoundingBox _calculateBoundingBox(List<Offset> points) {
    double minX = double.infinity;
    double maxX = -double.infinity;
    double minY = double.infinity;
    double maxY = -double.infinity;
    
    for (final point in points) {
      minX = math.min(minX, point.dx);
      maxX = math.max(maxX, point.dx);
      minY = math.min(minY, point.dy);
      maxY = math.max(maxY, point.dy);
    }
    
    return _BoundingBox(minX, maxX, minY, maxY);
  }
  
  // ===========================================================================
  // ФУНКЦИИ ДЛЯ РАБОТЫ С ИНТЕРВАЛАМИ ЛИНИЙ
  // ===========================================================================
  
  static List<_LineInterval> _getHorizontalIntervals(double y, List<Offset> polygon) {
    final List<double> intersections = [];
    
    for (int i = 0; i < polygon.length; i++) {
      final j = (i + 1) % polygon.length;
      final Offset a = polygon[i];
      final Offset b = polygon[j];
      
      if ((a.dy <= y && b.dy >= y) || (a.dy >= y && b.dy <= y)) {
        if (a.dy == b.dy) continue; // Горизонтальная линия
        final double t = (y - a.dy) / (b.dy - a.dy);
        final double x = a.dx + t * (b.dx - a.dx);
        intersections.add(x);
      }
    }
    
    intersections.sort();
    final List<_LineInterval> intervals = [];
    
    for (int i = 0; i < intersections.length; i += 2) {
      if (i + 1 < intersections.length) {
        final double x1 = intersections[i];
        final double x2 = intersections[i + 1];
        // Проверяем, что средняя точка внутри полигона
        final double midX = (x1 + x2) / 2;
        if (_isPointInPolygon(Offset(midX, y), polygon)) {
          intervals.add(_LineInterval(x1, x2));
        }
      }
    }
    
    return intervals;
  }
  
  static List<_LineInterval> _getVerticalIntervals(double x, List<Offset> polygon) {
    final List<double> intersections = [];
    
    for (int i = 0; i < polygon.length; i++) {
      final j = (i + 1) % polygon.length;
      final Offset a = polygon[i];
      final Offset b = polygon[j];
      
      if ((a.dx <= x && b.dx >= x) || (a.dx >= x && b.dx <= x)) {
        if (a.dx == b.dx) continue; // Вертикальная линия
        final double t = (x - a.dx) / (b.dx - a.dx);
        final double y = a.dy + t * (b.dy - a.dy);
        intersections.add(y);
      }
    }
    
    intersections.sort();
    final List<_LineInterval> intervals = [];
    
    for (int i = 0; i < intersections.length; i += 2) {
      if (i + 1 < intersections.length) {
        final double y1 = intersections[i];
        final double y2 = intersections[i + 1];
        // Проверяем, что средняя точка внутри полигона
        final double midY = (y1 + y2) / 2;
        if (_isPointInPolygon(Offset(x, midY), polygon)) {
          intervals.add(_LineInterval(y1, y2));
        }
      }
    }
    
    return intervals;
  }
  
  // ===========================================================================
  // ФУНКЦИИ РАБОТЫ С СИНИМИ ЛИНИЯМИ
  // ===========================================================================
  
  static _PickTransitionResult? _findNearestTransitionPoint(
    Offset startPoint, 
    List<List<Offset>> transitions
  ) {
    if (transitions.isEmpty) return null;
    
    double minDistance = double.infinity;
    int bestTransitionIndex = -1;
    int bestSegmentIndex = -1;
    Offset bestSnapPoint = Offset.zero;
    
    for (int transitionIndex = 0; transitionIndex < transitions.length; transitionIndex++) {
      final transition = transitions[transitionIndex];
      
      if (transition.length < 2) continue;
      
      for (int segmentIndex = 0; segmentIndex < transition.length - 1; segmentIndex++) {
        final segmentStart = transition[segmentIndex];
        final segmentEnd = transition[segmentIndex + 1];
        
        final projection = _projectPointOntoSegment(startPoint, segmentStart, segmentEnd);
        final distance = _distanceBetweenPoints(startPoint, projection);
        
        if (distance < minDistance) {
          minDistance = distance;
          bestTransitionIndex = transitionIndex;
          bestSegmentIndex = segmentIndex;
          bestSnapPoint = projection;
        }
      }
    }
    
    if (bestTransitionIndex == -1) return null;
    
    return _PickTransitionResult(
      bestTransitionIndex,
      bestSegmentIndex,
      bestSnapPoint,
      minDistance,
    );
  }
  
  static _Traversal _buildTraversalAlongBlueLine({
    required List<Offset> blueLine,
    required Offset entryPoint,
    required int entrySegmentIndex,
    required bool forward,
  }) {
    final List<Offset> traversalPoints = [];
    final List<double> cumulativeDistances = [];
    final List<int> originalIndices = [];
    
    traversalPoints.add(entryPoint);
    cumulativeDistances.add(0.0);
    originalIndices.add(entrySegmentIndex);
    
    if (blueLine.length < 2) {
      return _Traversal(traversalPoints, cumulativeDistances, originalIndices);
    }
    
    if (forward) {
      for (int i = entrySegmentIndex + 1; i < blueLine.length; i++) {
        traversalPoints.add(blueLine[i]);
        final double segmentDistance = _distanceBetweenPoints(
          traversalPoints[traversalPoints.length - 2],
          traversalPoints[traversalPoints.length - 1],
        );
        cumulativeDistances.add(
          cumulativeDistances.last + segmentDistance,
        );
        originalIndices.add(i);
      }
    } else {
      for (int i = entrySegmentIndex; i >= 0; i--) {
        traversalPoints.add(blueLine[i]);
        final double segmentDistance = _distanceBetweenPoints(
          traversalPoints[traversalPoints.length - 2],
          traversalPoints[traversalPoints.length - 1],
        );
        cumulativeDistances.add(
          cumulativeDistances.last + segmentDistance,
        );
        originalIndices.add(i);
      }
    }
    
    return _Traversal(traversalPoints, cumulativeDistances, originalIndices);
  }
  
  static List<(int, Offset)> _findZoneEntriesAlongTraversal(
    _Traversal traversal,
    List<_Poly> greenZones,
  ) {
    final List<(int, Offset)> entries = [];
    
    if (traversal.pts.isEmpty || greenZones.isEmpty) {
      return entries;
    }
    
    // Проходим по всем точкам траектории
    for (int i = 0; i < traversal.pts.length; i++) {
      final point = traversal.pts[i];
      
      // Проверяем, в какой зеленой зоне находится точка
      for (int zoneIndex = 0; zoneIndex < greenZones.length; zoneIndex++) {
        final zone = greenZones[zoneIndex];
        
        if (_isPointInPolygon(point, zone.points)) {
          // Проверяем, не находимся ли мы уже в предыдущей точке этой зоны
          bool alreadyInZone = false;
          if (i > 0) {
            final previousPoint = traversal.pts[i - 1];
            if (_isPointInPolygon(previousPoint, zone.points)) {
              alreadyInZone = true;
            }
          }
          
          if (!alreadyInZone) {
            entries.add((zoneIndex, point));
          }
          break;
        }
      }
    }
    
    return entries;
  }
  
  static (int, Offset)? _findZoneEntryOnSegment(
    Offset segmentStart,
    Offset segmentEnd,
    List<_Poly> greenZones,
    Set<int> visitedZones,
  ) {
    // Проверяем, находится ли конечная точка сегмента в зеленой зоне
    for (int zoneIndex = 0; zoneIndex < greenZones.length; zoneIndex++) {
      if (visitedZones.contains(zoneIndex)) continue;
      
      final zone = greenZones[zoneIndex];
      final isEndInZone = _isPointInPolygon(segmentEnd, zone.points);
      
      if (isEndInZone) {
        // Проверяем, находится ли начало сегмента тоже в зоне
        final isStartInZone = _isPointInPolygon(segmentStart, zone.points);
        
        if (!isStartInZone) {
          // Находим точку пересечения сегмента с границей зоны
          const int samples = 10;
          for (int i = 1; i <= samples; i++) {
            final t = i / samples;
            final point = Offset(
              segmentStart.dx + (segmentEnd.dx - segmentStart.dx) * t,
              segmentStart.dy + (segmentEnd.dy - segmentStart.dy) * t,
            );
            if (_isPointInPolygon(point, zone.points)) {
              return (zoneIndex, point);
            }
          }
          return (zoneIndex, segmentEnd);
        }
      }
    }
    
    return null;
  }
  
  static Offset? _findRejoinPointOnBlueLine({
    required Offset currentPosition,
    required List<Offset> blueLine,
    required int startSegmentIndex,
    required bool forward,
    required List<_Poly> forbiddenZones,
  }) {
    // Ищем ближайшую точку на синей линии, которая безопасна для возврата
    double minDistance = double.infinity;
    Offset? bestPoint;
    
    for (int i = 0; i < blueLine.length; i++) {
      final point = blueLine[i];
      if (!_doesSegmentIntersectForbidden(currentPosition, point, forbiddenZones)) {
        final distance = _distanceBetweenPoints(currentPosition, point);
        if (distance < minDistance) {
          minDistance = distance;
          bestPoint = point;
        }
      }
    }
    
    return bestPoint;
  }
  
  static int _findClosestPointIndex(Offset point, List<Offset> points) {
    double minDistance = double.infinity;
    int bestIndex = 0;
    
    for (int i = 0; i < points.length; i++) {
      final distance = _distanceBetweenPoints(point, points[i]);
      if (distance < minDistance) {
        minDistance = distance;
        bestIndex = i;
      }
    }
    
    return bestIndex;
  }
  
  // ===========================================================================
  // УТИЛИТЫ
  // ===========================================================================
  
  static double _distanceBetweenPoints(Offset a, Offset b) {
    final double dx = b.dx - a.dx;
    final double dy = b.dy - a.dy;
    return math.sqrt(dx * dx + dy * dy);
  }
  
  static Offset _projectPointOntoSegment(Offset point, Offset segmentStart, Offset segmentEnd) {
    final double segmentLength = _distanceBetweenPoints(segmentStart, segmentEnd);
    if (segmentLength == 0) return segmentStart;
    
    final double dx = segmentEnd.dx - segmentStart.dx;
    final double dy = segmentEnd.dy - segmentStart.dy;
    
    final double t = ((point.dx - segmentStart.dx) * dx + (point.dy - segmentStart.dy) * dy) /
                    (segmentLength * segmentLength);
    
    final double clampedT = t.clamp(0.0, 1.0);
    
    return Offset(
      segmentStart.dx + clampedT * dx,
      segmentStart.dy + clampedT * dy,
    );
  }
  
  static List<Offset> _simplifyRoute(List<Offset> route, {required double minDistance}) {
    if (route.length <= 2) return List.from(route);
    
    final List<Offset> simplified = [route.first];
    final double minDistanceSquared = minDistance * minDistance;
    
    for (int i = 1; i < route.length; i++) {
      final Offset lastPoint = simplified.last;
      final Offset currentPoint = route[i];
      
      final double dx = currentPoint.dx - lastPoint.dx;
      final double dy = currentPoint.dy - lastPoint.dy;
      final double distanceSquared = dx * dx + dy * dy;
      
      if (distanceSquared >= minDistanceSquared) {
        simplified.add(currentPoint);
      }
    }
    
    if (simplified.last != route.last) {
      simplified.add(route.last);
    }
    
    return simplified;
  }
  
  static List<Offset> _optimizeRoute(List<Offset> route, {required double minDistance}) {
    return _simplifyRoute(route, minDistance: minDistance);
  }
}

// ===========================================================================
// УПРОЩЕННЫЙ ИНТЕРФЕЙС ДЛЯ ИСПОЛЬЗОВАНИЯ
// ===========================================================================

Future<List<Offset>> buildRobotRoute(
  ManualMapState mapState, {
  double lineStep = 1.0,
  double cellSize = 1.0,
}) async {
  try {
    final (route, error) = RouteBuilder.buildRoute(
      mapState,
      lineStepCells: lineStep,
      cellSize: cellSize,
      debugPrint: kDebugMode,
    );
    
    if (error != null) {
      print('Ошибка построения маршрута: $error');
      return const [];
    }
    
    return route;
  } catch (e) {
    print('Исключение при построении маршрута: $e');
    return const [];
  }
}