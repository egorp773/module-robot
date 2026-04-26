// lib/core/cleaning_route_planner.dart
// Реальный маршрут змейкой внутри зелёной зоны с учётом запреток.

import 'dart:math' as math;
import 'dart:ui';
import 'package:flutter/foundation.dart';

import '../features/manual/manual_control_screen.dart';

enum RobotCommandType { move, cleanOn, cleanOff }

class RobotCommand {
  final RobotCommandType type;
  final Offset? target;
  const RobotCommand(this.type, {this.target});

  @override
  String toString() {
    switch (type) {
      case RobotCommandType.move:
        return 'MOVE(${target!.dx.toStringAsFixed(2)}, ${target!.dy.toStringAsFixed(2)})';
      case RobotCommandType.cleanOn:
        return 'CLEAN_ON';
      case RobotCommandType.cleanOff:
        return 'CLEAN_OFF';
    }
  }
}

class CleaningRoute {
  final List<RobotCommand> commands;
  final List<Offset> path;
  final double totalDistance;
  final int cleaningSegments;

  const CleaningRoute({
    required this.commands,
    required this.path,
    required this.totalDistance,
    required this.cleaningSegments,
  });
}

class CleaningRoutePlanner {
  static CleaningRoute? planRoute(
    ManualMapState mapState, {
    double lineStep = 44.0,
    int borderPasses = 1,
    Offset? startOverride,
    bool debugPrint = false,
  }) {
    final logger = _Logger(debugPrint);
    logger.log('=== CLEAN ROUTE ===');

    if (mapState.zones.isEmpty) return null;

    final start = startOverride ?? mapState.startPoint;
    if (start == null) {
      logger.log('ОШИБКА: Нет стартовой точки (чёрный квадрат)');
      return null;
    }
    final commands = <RobotCommand>[];
    final path = <Offset>[];
    double totalDistance = 0.0;
    int cleaningSegments = 0;
    Offset? current;

    void moveTo(Offset p) {
      if (current == null) {
        path.add(p);
        current = p;
        return;
      }
      if (_dist(current!, p) < 0.001) return;
      commands.add(const RobotCommand(RobotCommandType.cleanOff));
      commands.add(RobotCommand(RobotCommandType.move, target: p));
      path.add(p);
      totalDistance += _dist(current!, p);
      current = p;
    }

    for (int zi = 0; zi < mapState.zones.length; zi++) {
      final zone = mapState.zones[zi];
      final forbiddens = _findInternalForbiddens(zone, mapState.forbiddens);
      if (zone.points.length < 3) continue;

      current ??= (zi == 0 ? start : zone.points.first);

      // 1) Периметр зоны (1–2 прохода)
      for (final poly in _buildBorderPasses(zone.points, lineStep, borderPasses)) {
        final polyPath = _close(poly);
        moveTo(polyPath.first);
        commands.add(const RobotCommand(RobotCommandType.cleanOn));
        for (int i = 1; i < polyPath.length; i++) {
          final p = polyPath[i];
          commands.add(RobotCommand(RobotCommandType.move, target: p));
          path.add(p);
          totalDistance += _dist(current!, p);
          current = p;
        }
        cleaningSegments++;
      }

      // 2) Периметры запреток (по одному проходу)
      for (final f in forbiddens) {
        if (f.points.length < 3) continue;
        final fp = _close(f.points);
        moveTo(fp.first);
        commands.add(const RobotCommand(RobotCommandType.cleanOn));
        for (int i = 1; i < fp.length; i++) {
          final p = fp[i];
          commands.add(RobotCommand(RobotCommandType.move, target: p));
          path.add(p);
          totalDistance += _dist(current!, p);
          current = p;
        }
        cleaningSegments++;
      }

      // 3) Основная змейка внутри зоны
      final snakeSegments = _buildSnakeSegments(zone.points, forbiddens, lineStep);
      for (final seg in snakeSegments) {
        moveTo(seg.start);
        commands.add(const RobotCommand(RobotCommandType.cleanOn));
        commands.add(RobotCommand(RobotCommandType.move, target: seg.end));
        path.add(seg.end);
        totalDistance += _dist(seg.start, seg.end);
        current = seg.end;
        cleaningSegments++;
      }
    }

    if (commands.isEmpty) return null;

    return CleaningRoute(
      commands: commands,
      path: path,
      totalDistance: totalDistance,
      cleaningSegments: cleaningSegments,
    );
  }

  static List<_Segment> _buildSnakeSegments(
    List<Offset> zone,
    List<PolyShape> forbiddens,
    double step,
  ) {
    final bbox = _bbox(zone);
    final horizontal = bbox.width >= bbox.height;
    final segments = <_Segment>[];
    int lineIndex = 0;

    if (horizontal) {
      double y = bbox.minY + step * 0.5;
      while (y <= bbox.maxY - step * 0.5) {
        var allowed = _scanIntervalsHorizontal(zone, y);
        for (final f in forbiddens) {
          final blocked = _scanIntervalsHorizontal(f.points, y);
          allowed = _subtractIntervals(allowed, blocked);
        }
        if (allowed.isNotEmpty) {
          final reverse = lineIndex.isOdd;
          final ordered = reverse ? allowed.reversed.toList() : allowed;
          for (final i in ordered) {
            final a = Offset(i.start, y);
            final b = Offset(i.end, y);
            segments.add(reverse ? _Segment(b, a) : _Segment(a, b));
          }
          lineIndex++;
        }
        y += step;
      }
    } else {
      double x = bbox.minX + step * 0.5;
      while (x <= bbox.maxX - step * 0.5) {
        var allowed = _scanIntervalsVertical(zone, x);
        for (final f in forbiddens) {
          final blocked = _scanIntervalsVertical(f.points, x);
          allowed = _subtractIntervals(allowed, blocked);
        }
        if (allowed.isNotEmpty) {
          final reverse = lineIndex.isOdd;
          final ordered = reverse ? allowed.reversed.toList() : allowed;
          for (final i in ordered) {
            final a = Offset(x, i.start);
            final b = Offset(x, i.end);
            segments.add(reverse ? _Segment(b, a) : _Segment(a, b));
          }
          lineIndex++;
        }
        x += step;
      }
    }

    return segments;
  }

  static List<_Interval> _scanIntervalsHorizontal(List<Offset> poly, double y) {
    final xs = <double>[];
    for (int i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % poly.length];
      if ((a.dy <= y && b.dy > y) || (b.dy <= y && a.dy > y)) {
        final t = (y - a.dy) / (b.dy - a.dy);
        final x = a.dx + (b.dx - a.dx) * t;
        xs.add(x);
      }
    }
    xs.sort();
    final out = <_Interval>[];
    for (int i = 0; i + 1 < xs.length; i += 2) {
      if ((xs[i + 1] - xs[i]).abs() > 0.001) {
        out.add(_Interval(xs[i], xs[i + 1]));
      }
    }
    return out;
  }

  static List<_Interval> _scanIntervalsVertical(List<Offset> poly, double x) {
    final ys = <double>[];
    for (int i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % poly.length];
      if ((a.dx <= x && b.dx > x) || (b.dx <= x && a.dx > x)) {
        final t = (x - a.dx) / (b.dx - a.dx);
        final y = a.dy + (b.dy - a.dy) * t;
        ys.add(y);
      }
    }
    ys.sort();
    final out = <_Interval>[];
    for (int i = 0; i + 1 < ys.length; i += 2) {
      if ((ys[i + 1] - ys[i]).abs() > 0.001) {
        out.add(_Interval(ys[i], ys[i + 1]));
      }
    }
    return out;
  }

  static List<_Interval> _subtractIntervals(
    List<_Interval> base,
    List<_Interval> cut,
  ) {
    if (base.isEmpty || cut.isEmpty) return base;
    final out = <_Interval>[];
    final cuts = List<_Interval>.from(cut)..sort((a, b) => a.start.compareTo(b.start));
    for (final b in base) {
      var curStart = b.start;
      for (final c in cuts) {
        if (c.end <= curStart) continue;
        if (c.start >= b.end) break;
        if (c.start > curStart) {
          out.add(_Interval(curStart, math.min(c.start, b.end)));
        }
        curStart = math.max(curStart, c.end);
        if (curStart >= b.end) break;
      }
      if (curStart < b.end) out.add(_Interval(curStart, b.end));
    }
    return out;
  }

  static List<Offset> _close(List<Offset> pts) {
    if (pts.isEmpty) return const [];
    if (pts.first == pts.last) return pts;
    return [...pts, pts.first];
  }

  static List<List<Offset>> _buildBorderPasses(
    List<Offset> poly,
    double step,
    int count,
  ) {
    final passes = <List<Offset>>[];
    if (poly.length < 3) return passes;
    passes.add(poly);
    if (count <= 1) return passes;

    // Грубый внутренний проход: сдвигаем к центроиду
    final c = _centroid(poly);
    final inner = poly
        .map((p) => Offset(
              p.dx + (c.dx - p.dx) * (step / 100).clamp(0.02, 0.15),
              p.dy + (c.dy - p.dy) * (step / 100).clamp(0.02, 0.15),
            ))
        .toList();
    passes.add(inner);
    return passes;
  }

  static Offset _centroid(List<Offset> poly) {
    double x = 0, y = 0;
    for (final p in poly) {
      x += p.dx;
      y += p.dy;
    }
    return Offset(x / poly.length, y / poly.length);
  }

  static List<PolyShape> _findInternalForbiddens(
    PolyShape zone,
    List<PolyShape> all,
  ) {
    return all.where((f) => f.points.any((p) => _isPointInPolygon(p, zone.points))).toList();
  }

  static bool _isPointInPolygon(Offset p, List<Offset> poly) {
    if (poly.length < 3) return false;
    bool inside = false;
    int j = poly.length - 1;
    for (int i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[j];
      final intersects = ((a.dy > p.dy) != (b.dy > p.dy)) &&
          (p.dx < (b.dx - a.dx) * (p.dy - a.dy) / (b.dy - a.dy) + a.dx);
      if (intersects) inside = !inside;
      j = i;
    }
    return inside;
  }

  static _BBox _bbox(List<Offset> pts) {
    double minX = pts.first.dx, maxX = pts.first.dx;
    double minY = pts.first.dy, maxY = pts.first.dy;
    for (final p in pts) {
      minX = math.min(minX, p.dx);
      maxX = math.max(maxX, p.dx);
      minY = math.min(minY, p.dy);
      maxY = math.max(maxY, p.dy);
    }
    return _BBox(minX, maxX, minY, maxY);
  }

  static double _dist(Offset a, Offset b) {
    final dx = b.dx - a.dx;
    final dy = b.dy - a.dy;
    return math.sqrt(dx * dx + dy * dy);
  }
}

class _Segment {
  final Offset start;
  final Offset end;
  const _Segment(this.start, this.end);
}

class _Interval {
  final double start;
  final double end;
  const _Interval(this.start, this.end);
}

class _BBox {
  final double minX, maxX, minY, maxY;
  const _BBox(this.minX, this.maxX, this.minY, this.maxY);
  double get width => maxX - minX;
  double get height => maxY - minY;
}

class _Logger {
  final bool enabled;
  _Logger(this.enabled);
  void log(String message) {
    if (enabled && kDebugMode) {
      print('[CleaningRoutePlanner] $message');
    }
  }
}
