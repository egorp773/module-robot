// lib/core/cleaning_route_planner.dart

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
        final p = target!;
        return 'MOVE(${p.dx.toStringAsFixed(2)}, ${p.dy.toStringAsFixed(2)})';
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
  static const double defaultLineStepMeters = 0.35;
  static const double defaultRobotWidthMeters = 0.50;
  static const double defaultBoundaryMarginMeters = 0.25;
  // Forbidden contours are recorded by driving around the obstacle, so the
  // stored line is already a safe centerline contour. Keep a 5 cm buffer so the
  // robot may pass close to the contour without treating it as robot radius.
  static const double defaultForbiddenMarginMeters = 0.05;
  static const double defaultMaxStartDistanceMeters = 5.0;
  static const double _eps = 1e-6;

  static CleaningRoute? planRoute(
    ManualMapState mapState, {
    double lineStep = defaultLineStepMeters,
    int borderPasses = 1,
    Offset? startOverride,
    double robotWidthMeters = defaultRobotWidthMeters,
    double? boundaryMarginMeters,
    double? forbiddenMarginMeters,
    double maxStartDistanceMeters = defaultMaxStartDistanceMeters,
    bool includeRecordedPerimeterPass = true,
    bool debugPrint = false,
  }) {
    final logger = _Logger(debugPrint);
    logger.log('=== CLEAN ROUTE ===');

    if (mapState.zones.isEmpty) return null;
    if (!lineStep.isFinite || lineStep <= 0) return null;
    if (!robotWidthMeters.isFinite || robotWidthMeters <= 0) return null;

    final start = startOverride ?? mapState.startPoint ?? mapState.robot;
    if (!_isFinitePoint(start)) return null;

    final boundaryMargin = boundaryMarginMeters ??
        math.max(robotWidthMeters * 0.5, defaultBoundaryMarginMeters);
    final forbiddenMargin =
        forbiddenMarginMeters ?? defaultForbiddenMarginMeters;
    final step = lineStep.clamp(0.15, 2.0).toDouble();

    final commands = <RobotCommand>[];
    final path = <Offset>[start];
    var current = start;
    var cleaningSegments = 0;
    var reachedAnyZone = false;

    void addPoint(Offset p) {
      if (!_isFinitePoint(p)) return;
      if (_dist(current, p) < 0.03) return;
      commands.add(RobotCommand(RobotCommandType.move, target: p));
      path.add(p);
      current = p;
    }

    bool addTravelPath(List<Offset> pts) {
      if (pts.isEmpty) return false;
      commands.add(const RobotCommand(RobotCommandType.cleanOff));
      for (final p in pts.skip(1)) {
        addPoint(p);
      }
      return true;
    }

    bool addSafeCleaningPath(
      List<Offset> pts,
      List<Offset> outer,
      List<List<Offset>> blocked,
    ) {
      if (pts.length < 2) return false;
      commands.add(const RobotCommand(RobotCommandType.cleanOn));
      var added = false;
      for (final p in pts.skip(1)) {
        if (_isSegmentSafe(current, p, outer, blocked)) {
          addPoint(p);
          added = true;
          continue;
        }

        final detour = _safePathBetween(current, p, outer, blocked);
        if (detour == null) return false;
        addTravelPath(detour);
        commands.add(const RobotCommand(RobotCommandType.cleanOn));
        added = true;
      }
      return added;
    }

    for (final zone in mapState.zones) {
      final rawZone = _normalizePolygon(zone.points);
      if (rawZone.length < 3) continue;

      var safeOuter = _safeOffsetPolygon(rawZone, boundaryMargin);
      if (safeOuter.length < 3 || _polygonArea(safeOuter).abs() < 0.2) {
        safeOuter = _safeOffsetPolygon(rawZone, math.min(boundaryMargin, 0.05));
        if (safeOuter.length < 3 || _polygonArea(safeOuter).abs() < 0.2) {
          logger.log('inset collapsed, using recorded perimeter as safe outer');
          safeOuter = rawZone;
        }
      }

      final zoneForbiddens = _findInternalForbiddens(
              rawZone, mapState.forbiddens, margin: forbiddenMargin)
          .map((f) =>
              _safeOffsetPolygon(_normalizePolygon(f.points), -forbiddenMargin))
          .where((f) => f.length >= 3 && _polygonArea(f).abs() >= 0.05)
          .toList();
      logger.log('expanded forbiddens: ${zoneForbiddens.length}');

      final rawEntry = _closestPointOnPolygon(current, rawZone);
      final entry = _closestPointOnPolygon(rawEntry.point, safeOuter);
      final startDistance = _dist(start, rawEntry.point);
      if (!reachedAnyZone &&
          startDistance > maxStartDistanceMeters &&
          !_isPointInPolygon(start, rawZone)) {
        logger.log(
            'start too far from zone: ${startDistance.toStringAsFixed(2)}m');
        continue;
      }

      if (!reachedAnyZone && !_isPointInPolygon(start, rawZone)) {
        if (!_isSegmentFreeOfBlocked(start, rawEntry.point, zoneForbiddens)) {
          logger.log(
              'outside approach touches a forbidden polygon, entering via boundary anyway');
        }
        addTravelPath([current, rawEntry.point]);
      } else if (_isPointInPolygon(current, rawZone)) {
        final toEntry =
            _safePathBetween(current, rawEntry.point, rawZone, zoneForbiddens);
        if (toEntry == null) {
          logger.log('zone entry is blocked from inside, skipping zone');
          continue;
        }
        addTravelPath(toEntry);
      } else {
        final toEntry = _transitionPathBetween(
          current,
          rawEntry.point,
          mapState.transitions,
          zoneForbiddens,
        );
        if (toEntry == null) {
          logger
              .log('zone is not reachable from current position or transition');
          continue;
        }
        addTravelPath(toEntry);
      }

      reachedAnyZone = true;

      if (includeRecordedPerimeterPass) {
        final recordedPerimeter =
            _perimeterPathFromProjection(rawZone, rawEntry);
        if (recordedPerimeter.length >= 2 &&
            addSafeCleaningPath(recordedPerimeter, rawZone, zoneForbiddens)) {
          cleaningSegments++;
        }
      }

      final toInset =
          _safePathBetween(current, entry.point, rawZone, zoneForbiddens);
      if (toInset != null) {
        addTravelPath(toInset);
      } else {
        logger.log('inset entry unreachable, continuing from perimeter');
      }

      final extraPasses = math.max(0, borderPasses - 1);
      for (var i = 0; i < extraPasses; i++) {
        final pass = _safeOffsetPolygon(safeOuter, step * (i + 1));
        if (pass.length < 3 || _polygonArea(pass).abs() < 0.2) continue;
        final passEntry = _closestPointOnPolygon(current, pass);
        final toPass = _safePathBetween(
            current, passEntry.point, safeOuter, zoneForbiddens);
        if (toPass == null) continue;
        addTravelPath(toPass);
        final passPath = _perimeterPathFromProjection(pass, passEntry);
        if (addSafeCleaningPath(passPath, safeOuter, zoneForbiddens)) {
          cleaningSegments++;
        }
      }

      final primaryHorizontal =
          _bbox(safeOuter).width >= _bbox(safeOuter).height;
      final snakeOuter = rawZone;
      var snakeSegments = _buildSnakeSegments(
        snakeOuter,
        zoneForbiddens,
        step,
        edgeInset: boundaryMargin,
        horizontalOverride: primaryHorizontal,
      );
      if (snakeSegments.isEmpty && boundaryMargin > 0.05) {
        snakeSegments = _buildSnakeSegments(
          snakeOuter,
          zoneForbiddens,
          step,
          edgeInset: 0.05,
          horizontalOverride: primaryHorizontal,
        );
      }
      if (snakeSegments.isEmpty) {
        snakeSegments = _buildSnakeSegments(
          snakeOuter,
          zoneForbiddens,
          step,
          edgeInset: boundaryMargin,
          horizontalOverride: !primaryHorizontal,
        );
      }
      if (snakeSegments.isEmpty && boundaryMargin > 0.05) {
        snakeSegments = _buildSnakeSegments(
          snakeOuter,
          zoneForbiddens,
          step,
          edgeInset: 0.05,
          horizontalOverride: !primaryHorizontal,
        );
      }
      snakeSegments = _orderSnakeSegments(
        snakeSegments,
        current,
        snakeOuter,
        zoneForbiddens,
      );
      logger.log('snake segments: ${snakeSegments.length}');
      var skippedSnakeSegments = 0;
      for (final seg in snakeSegments) {
        final reachableSeg = _reachableSegment(
          current,
          seg,
          snakeOuter,
          zoneForbiddens,
        );
        if (reachableSeg == null ||
            !_isSegmentSafe(
              reachableSeg.start,
              reachableSeg.end,
              snakeOuter,
              zoneForbiddens,
            )) {
          skippedSnakeSegments++;
          continue;
        }

        final connector = _safePathBetween(
          current,
          reachableSeg.start,
          snakeOuter,
          zoneForbiddens,
        );
        if (connector == null) {
          skippedSnakeSegments++;
          continue;
        }
        addTravelPath(connector);

        commands.add(const RobotCommand(RobotCommandType.cleanOn));
        addPoint(reachableSeg.end);
        cleaningSegments++;
      }
      if (skippedSnakeSegments > 0 &&
          _shouldBuildCrossFallback(
            skippedSnakeSegments,
            snakeSegments,
            snakeOuter,
          )) {
        logger.log('skipped unsafe/unreachable snake segments: '
            '$skippedSnakeSegments');
        var crossSegments = _buildSnakeSegments(
          snakeOuter,
          zoneForbiddens,
          step,
          edgeInset: boundaryMargin,
          horizontalOverride: !primaryHorizontal,
        );
        crossSegments = _orderSnakeSegments(
          crossSegments,
          current,
          snakeOuter,
          zoneForbiddens,
        );
        var skippedCrossSegments = 0;
        logger.log('cross snake segments: ${crossSegments.length}');
        for (final seg in crossSegments) {
          final reachableSeg = _reachableSegment(
            current,
            seg,
            snakeOuter,
            zoneForbiddens,
          );
          if (reachableSeg == null ||
              !_isSegmentSafe(
                reachableSeg.start,
                reachableSeg.end,
                snakeOuter,
                zoneForbiddens,
              )) {
            skippedCrossSegments++;
            continue;
          }

          final connector = _safePathBetween(
            current,
            reachableSeg.start,
            snakeOuter,
            zoneForbiddens,
          );
          if (connector == null) {
            skippedCrossSegments++;
            continue;
          }
          addTravelPath(connector);

          commands.add(const RobotCommand(RobotCommandType.cleanOn));
          addPoint(reachableSeg.end);
          cleaningSegments++;
        }
        if (skippedCrossSegments > 0) {
          logger.log('skipped unsafe/unreachable cross segments: '
              '$skippedCrossSegments');
        }
      }
    }

    if (!reachedAnyZone || path.length < 2) return null;
    final simplified = _simplifyPath(path, minDistance: 0.01);
    final simplifiedDistance = _pathDistance(simplified);
    logger.log(
      'route points: ${simplified.length}, distance: '
      '${simplifiedDistance.toStringAsFixed(2)}m, cleaning segments: $cleaningSegments',
    );

    return CleaningRoute(
      commands: commands,
      path: simplified,
      totalDistance: simplifiedDistance,
      cleaningSegments: cleaningSegments,
    );
  }

  static List<_Segment> _buildSnakeSegments(
      List<Offset> outer, List<List<Offset>> blocked, double step,
      {double edgeInset = 0.04, bool? horizontalOverride}) {
    final bbox = _bbox(outer);
    final horizontal = horizontalOverride ?? bbox.width >= bbox.height;
    final segments = <_Segment>[];
    var lineIndex = 0;
    final inset = math.max(0.04, edgeInset);

    if (horizontal) {
      for (var y = bbox.minY + inset;
          y <= bbox.maxY - inset + _eps;
          y += step) {
        var allowed = _scanIntervalsHorizontal(outer, y);
        for (final f in blocked) {
          allowed = _subtractIntervals(allowed, _scanIntervalsHorizontal(f, y));
        }
        allowed = _shrinkIntervals(allowed, inset)
            .where((i) => i.length >= step * 0.65)
            .toList();
        if (allowed.isEmpty) continue;
        final reverse = lineIndex.isOdd;
        final indexed = <({int index, _Interval interval})>[
          for (var idx = 0; idx < allowed.length; idx++)
            (index: idx, interval: allowed[idx]),
        ];
        final ordered = reverse ? indexed.reversed.toList() : indexed;
        for (final item in ordered) {
          final i = item.interval;
          final a = Offset(i.start, y);
          final b = Offset(i.end, y);
          segments.add(
            reverse
                ? _Segment(b, a,
                    laneIndex: lineIndex, intervalIndex: item.index)
                : _Segment(a, b,
                    laneIndex: lineIndex, intervalIndex: item.index),
          );
        }
        lineIndex++;
      }
    } else {
      for (var x = bbox.minX + inset;
          x <= bbox.maxX - inset + _eps;
          x += step) {
        var allowed = _scanIntervalsVertical(outer, x);
        for (final f in blocked) {
          allowed = _subtractIntervals(allowed, _scanIntervalsVertical(f, x));
        }
        allowed = _shrinkIntervals(allowed, inset)
            .where((i) => i.length >= step * 0.65)
            .toList();
        if (allowed.isEmpty) continue;
        final reverse = lineIndex.isOdd;
        final indexed = <({int index, _Interval interval})>[
          for (var idx = 0; idx < allowed.length; idx++)
            (index: idx, interval: allowed[idx]),
        ];
        final ordered = reverse ? indexed.reversed.toList() : indexed;
        for (final item in ordered) {
          final i = item.interval;
          final a = Offset(x, i.start);
          final b = Offset(x, i.end);
          segments.add(
            reverse
                ? _Segment(b, a,
                    laneIndex: lineIndex, intervalIndex: item.index)
                : _Segment(a, b,
                    laneIndex: lineIndex, intervalIndex: item.index),
          );
        }
        lineIndex++;
      }
    }

    return segments;
  }

  static bool _shouldBuildCrossFallback(
    int skippedSegments,
    List<_Segment> primarySegments,
    List<Offset> outer,
  ) {
    if (primarySegments.isEmpty) return false;
    final skippedRatio = skippedSegments / primarySegments.length;
    final area = _polygonArea(outer).abs();

    // On large mostly-rectangular maps, a few skipped split intervals around
    // red islands do not justify adding a full perpendicular pass. That creates
    // a doubled, chaotic-looking route and can exceed the ESP32 waypoint limit.
    if (area >= 160.0 && skippedRatio < 0.30) return false;

    return true;
  }

  static List<_Segment> _orderSnakeSegments(
    List<_Segment> segments,
    Offset start,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    if (segments.length <= 1) return segments;

    final maxInterval = segments.map((s) => s.intervalIndex).reduce(math.max);
    if (maxInterval > 0 && _segmentsBoundsArea(segments) > 120.0) {
      return _orderSplitSnakeGroups(segments, start, outer, blocked);
    }

    final forwardCost = _snakeStartCost(start, segments, outer, blocked);
    final reversed =
        segments.reversed.map((seg) => seg.reversed()).toList(growable: false);
    final reverseCost = _snakeStartCost(start, reversed, outer, blocked);

    return reverseCost < forwardCost ? reversed : segments;
  }

  static List<_Segment> _orderSplitSnakeGroups(
    List<_Segment> segments,
    Offset start,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    final maxInterval = segments.map((s) => s.intervalIndex).reduce(math.max);
    final groups = <int, List<_Segment>>{
      for (var group = 0; group <= maxInterval; group++)
        group: segments
            .where((s) => s.intervalIndex == group)
            .toList(growable: false),
    }..removeWhere((_, value) => value.isEmpty);

    final out = <_Segment>[];
    var current = start;
    final remaining = groups.keys.toSet();

    while (remaining.isNotEmpty) {
      int? bestGroup;
      List<_Segment>? bestSegments;
      var bestCost = double.infinity;

      for (final group in remaining) {
        final groupSegments = groups[group]!;
        final forward = _orientSegmentRun(
          groupSegments,
          current,
          outer,
          blocked,
        );
        final reversed = _orientSegmentRun(
          groupSegments.reversed.toList(growable: false),
          current,
          outer,
          blocked,
        );
        final forwardCost = _snakeStartCost(current, forward, outer, blocked);
        final reverseCost = _snakeStartCost(current, reversed, outer, blocked);
        final picked = reverseCost < forwardCost ? reversed : forward;
        final cost = math.min(forwardCost, reverseCost);
        if (cost < bestCost) {
          bestCost = cost;
          bestGroup = group;
          bestSegments = picked;
        }
      }

      if (bestGroup == null || bestSegments == null) break;
      out.addAll(bestSegments);
      current = bestSegments.last.end;
      remaining.remove(bestGroup);
    }

    return out;
  }

  static List<_Segment> _orientSegmentRun(
    List<_Segment> segments,
    Offset start,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    final out = <_Segment>[];
    var current = start;
    for (final seg in segments) {
      final startCost = _snakeEndpointCost(current, seg.start, outer, blocked);
      final endCost = _snakeEndpointCost(current, seg.end, outer, blocked);
      final picked = endCost < startCost ? seg.reversed() : seg;
      out.add(picked);
      current = picked.end;
    }
    return out;
  }

  static double _segmentsBoundsArea(List<_Segment> segments) {
    var minX = math.min(segments.first.start.dx, segments.first.end.dx);
    var maxX = math.max(segments.first.start.dx, segments.first.end.dx);
    var minY = math.min(segments.first.start.dy, segments.first.end.dy);
    var maxY = math.max(segments.first.start.dy, segments.first.end.dy);
    for (final s in segments.skip(1)) {
      minX = math.min(minX, math.min(s.start.dx, s.end.dx));
      maxX = math.max(maxX, math.max(s.start.dx, s.end.dx));
      minY = math.min(minY, math.min(s.start.dy, s.end.dy));
      maxY = math.max(maxY, math.max(s.start.dy, s.end.dy));
    }
    return math.max(0.0, maxX - minX) * math.max(0.0, maxY - minY);
  }

  static double _snakeStartCost(
    Offset start,
    List<_Segment> segments,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    final first = segments.first.start;
    return _snakeEndpointCost(start, first, outer, blocked);
  }

  static double _snakeEndpointCost(
    Offset start,
    Offset first,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    if (_isSegmentSafe(start, first, outer, blocked)) {
      return _dist(start, first);
    }
    final path = _safePathBetween(start, first, outer, blocked);
    return path == null ? double.infinity : _pathDistance(path);
  }

  static _Segment? _reachableSegment(
    Offset current,
    _Segment segment,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    final startCost =
        _snakeEndpointCost(current, segment.start, outer, blocked);
    final endCost = _snakeEndpointCost(current, segment.end, outer, blocked);
    if (!startCost.isFinite && !endCost.isFinite) return null;
    return endCost < startCost ? segment.reversed() : segment;
  }

  static List<Offset>? _transitionPathBetween(
    Offset start,
    Offset goal,
    List<List<Offset>> transitions,
    List<List<Offset>> blocked,
  ) {
    if (_dist(start, goal) < 0.03) return [start];
    if (transitions.isEmpty) return null;

    List<Offset>? bestPath;
    var bestDistance = double.infinity;

    for (final transition in transitions) {
      final clean = _normalizePolyline(transition);
      if (clean.length < 2) continue;
      if (!_isPolylineFreeOfBlocked(clean, blocked)) continue;

      final startProjection = _closestPointOnPolyline(start, clean);
      final goalProjection = _closestPointOnPolyline(goal, clean);
      if (startProjection == null || goalProjection == null) continue;
      if (!_isSegmentFreeOfBlocked(start, startProjection.point, blocked)) {
        continue;
      }
      if (!_isSegmentFreeOfBlocked(goalProjection.point, goal, blocked)) {
        continue;
      }

      final forward = _polylineSubPath(
        clean,
        startProjection,
        goalProjection,
      );
      final backward = _polylineSubPath(
        clean,
        goalProjection,
        startProjection,
      ).reversed.toList();

      for (final middle in [forward, backward]) {
        if (middle.length < 2) continue;
        final path = _simplifyPath(
          [start, ...middle, goal],
          minDistance: 0.03,
        );
        final distance = _pathDistance(path);
        if (distance < bestDistance) {
          bestDistance = distance;
          bestPath = path;
        }
      }
    }

    return bestPath;
  }

  static List<Offset>? _safePathBetween(
    Offset start,
    Offset goal,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    if (_dist(start, goal) < 0.03) return [start];
    if (_isSegmentSafe(start, goal, outer, blocked)) return [start, goal];

    final nodes = <Offset>[start, goal];
    nodes.addAll(outer);
    for (final f in blocked) {
      nodes.addAll(f);
    }

    final n = nodes.length;
    final dist = List<double>.filled(n, double.infinity);
    final prev = List<int>.filled(n, -1);
    final used = List<bool>.filled(n, false);
    dist[0] = 0;

    for (var iter = 0; iter < n; iter++) {
      var u = -1;
      var best = double.infinity;
      for (var i = 0; i < n; i++) {
        if (!used[i] && dist[i] < best) {
          best = dist[i];
          u = i;
        }
      }
      if (u < 0 || u == 1) break;
      used[u] = true;

      for (var v = 0; v < n; v++) {
        if (used[v] || v == u) continue;
        if (!_isSegmentSafe(nodes[u], nodes[v], outer, blocked)) continue;
        final nd = dist[u] + _dist(nodes[u], nodes[v]);
        if (nd < dist[v]) {
          dist[v] = nd;
          prev[v] = u;
        }
      }
    }

    if (!dist[1].isFinite) {
      return _gridPathBetween(start, goal, outer, blocked);
    }
    final out = <Offset>[];
    var cur = 1;
    while (cur >= 0) {
      out.add(nodes[cur]);
      cur = prev[cur];
    }
    return _simplifyPath(out.reversed.toList(), minDistance: 0.04);
  }

  static List<Offset>? _gridPathBetween(
    Offset start,
    Offset goal,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    final bbox = _bbox([...outer, start, goal]);
    const resolution = 0.25;
    final minX = bbox.minX - resolution;
    final minY = bbox.minY - resolution;
    final nx = ((bbox.width + resolution * 2) / resolution).ceil() + 1;
    final ny = ((bbox.height + resolution * 2) / resolution).ceil() + 1;
    if (nx <= 0 || ny <= 0 || nx * ny > 6000) return null;

    Offset pointFor(int index) {
      final x = index % nx;
      final y = index ~/ nx;
      return Offset(minX + x * resolution, minY + y * resolution);
    }

    bool pointSafe(Offset p) {
      if (!_isPointInPolygon(p, outer)) return false;
      for (final f in blocked) {
        if (_isPointInPolygon(p, f, includeBoundary: false)) return false;
      }
      return true;
    }

    int? nearestIndex(Offset p) {
      final cx = ((p.dx - minX) / resolution).round().clamp(0, nx - 1);
      final cy = ((p.dy - minY) / resolution).round().clamp(0, ny - 1);
      int? bestIndex;
      var bestDistance = double.infinity;
      for (var r = 0; r <= 8; r++) {
        for (var y = math.max(0, cy - r); y <= math.min(ny - 1, cy + r); y++) {
          for (var x = math.max(0, cx - r);
              x <= math.min(nx - 1, cx + r);
              x++) {
            if ((x - cx).abs() != r && (y - cy).abs() != r) continue;
            final index = y * nx + x;
            final q = pointFor(index);
            if (!pointSafe(q)) continue;
            final d = _dist(p, q);
            if (d < bestDistance && _isSegmentSafe(p, q, outer, blocked)) {
              bestDistance = d;
              bestIndex = index;
            }
          }
        }
        if (bestIndex != null) return bestIndex;
      }
      return null;
    }

    final startIndex = nearestIndex(start);
    final goalIndex = nearestIndex(goal);
    if (startIndex == null || goalIndex == null) return null;

    final n = nx * ny;
    final cost = List<double>.filled(n, double.infinity);
    final prev = List<int>.filled(n, -1);
    final closed = List<bool>.filled(n, false);
    final safe = List<bool?>.filled(n, null);
    cost[startIndex] = 0;

    bool isSafeIndex(int index) {
      final cached = safe[index];
      if (cached != null) return cached;
      final value = pointSafe(pointFor(index));
      safe[index] = value;
      return value;
    }

    const dirs = <(int, int)>[
      (-1, -1),
      (0, -1),
      (1, -1),
      (-1, 0),
      (1, 0),
      (-1, 1),
      (0, 1),
      (1, 1),
    ];

    for (var iter = 0; iter < n; iter++) {
      var u = -1;
      var best = double.infinity;
      for (var i = 0; i < n; i++) {
        if (closed[i] || !cost[i].isFinite) continue;
        final score = cost[i] + _dist(pointFor(i), goal);
        if (score < best) {
          best = score;
          u = i;
        }
      }
      if (u < 0 || u == goalIndex) break;
      closed[u] = true;

      final ux = u % nx;
      final uy = u ~/ nx;
      final up = pointFor(u);
      for (final (dx, dy) in dirs) {
        final vx = ux + dx;
        final vy = uy + dy;
        if (vx < 0 || vx >= nx || vy < 0 || vy >= ny) continue;
        final v = vy * nx + vx;
        if (closed[v] || !isSafeIndex(v)) continue;
        final vp = pointFor(v);
        if (!_isSegmentSafe(up, vp, outer, blocked)) continue;
        final nd = cost[u] + _dist(up, vp);
        if (nd < cost[v]) {
          cost[v] = nd;
          prev[v] = u;
        }
      }
    }

    if (!cost[goalIndex].isFinite) return null;
    final middle = <Offset>[];
    var cur = goalIndex;
    while (cur >= 0) {
      middle.add(pointFor(cur));
      if (cur == startIndex) break;
      cur = prev[cur];
    }
    if (middle.isEmpty || middle.last != pointFor(startIndex)) return null;
    final path = <Offset>[start, ...middle.reversed, goal];
    return _simplifyPath(path, minDistance: 0.04);
  }

  static bool _isSegmentSafe(
    Offset a,
    Offset b,
    List<Offset> outer,
    List<List<Offset>> blocked,
  ) {
    if (!_isPointInPolygon(a, outer) || !_isPointInPolygon(b, outer)) {
      return false;
    }
    if (!_isSegmentFreeOfBlocked(a, b, blocked)) return false;
    const samples = 16;
    for (var i = 1; i < samples; i++) {
      final t = i / samples;
      final p = Offset(a.dx + (b.dx - a.dx) * t, a.dy + (b.dy - a.dy) * t);
      if (!_isPointInPolygon(p, outer)) return false;
      for (final f in blocked) {
        if (_isPointInPolygon(p, f, includeBoundary: false)) return false;
      }
    }
    return true;
  }

  static bool _isSegmentFreeOfBlocked(
    Offset a,
    Offset b,
    List<List<Offset>> blocked,
  ) {
    for (final f in blocked) {
      if (_isPointInPolygon(a, f, includeBoundary: false) ||
          _isPointInPolygon(b, f, includeBoundary: false)) {
        return false;
      }
      for (var i = 0; i < f.length; i++) {
        final c = f[i];
        final d = f[(i + 1) % f.length];
        if (_segmentsIntersect(a, b, c, d)) return false;
      }
    }
    return true;
  }

  static _Projection _closestPointOnPolygon(Offset p, List<Offset> poly) {
    var bestPoint = poly.first;
    var bestIndex = 0;
    var bestDistance = double.infinity;
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % poly.length];
      final q = _projectPointToSegment(p, a, b);
      final d = _dist(p, q);
      if (d < bestDistance) {
        bestDistance = d;
        bestPoint = q;
        bestIndex = i;
      }
    }
    return _Projection(bestPoint, bestIndex);
  }

  static List<Offset> _perimeterPathFromProjection(
    List<Offset> poly,
    _Projection projection,
  ) {
    final out = <Offset>[projection.point];
    final n = poly.length;
    var idx = (projection.edgeIndex + 1) % n;
    for (var c = 0; c < n; c++) {
      out.add(poly[idx]);
      idx = (idx + 1) % n;
    }
    out.add(projection.point);
    return _simplifyPath(out, minDistance: 0.03);
  }

  static List<Offset> _offsetPolygon(List<Offset> input, double distance) {
    var poly = _normalizePolygon(input);
    if (poly.length < 3) return const [];
    if (_polygonArea(poly) < 0) poly = poly.reversed.toList();

    final n = poly.length;
    final lines = <_Line>[];
    for (var i = 0; i < n; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % n];
      final d = b - a;
      final len = d.distance;
      if (len < _eps) continue;
      final dir = d / len;
      final normal = Offset(-dir.dy, dir.dx);
      lines.add(_Line(a + normal * distance, dir));
    }
    if (lines.length < 3) return const [];

    final out = <Offset>[];
    for (var i = 0; i < lines.length; i++) {
      final prev = lines[(i - 1 + lines.length) % lines.length];
      final cur = lines[i];
      final p = _lineIntersection(prev, cur);
      if (p == null || !_isFinitePoint(p)) {
        final source = poly[i % poly.length];
        out.add(source);
      } else {
        out.add(p);
      }
    }
    return _normalizePolygon(out);
  }

  static List<Offset> _safeOffsetPolygon(List<Offset> input, double distance) {
    final offset = _offsetPolygon(input, distance);
    if (_isUsablePolygon(offset)) return offset;

    final fallback = _radialOffsetPolygon(input, distance);
    if (_isUsablePolygon(fallback)) return fallback;
    return const [];
  }

  static bool _isUsablePolygon(List<Offset> poly) {
    if (poly.length < 3) return false;
    if (_polygonArea(poly).abs() < 0.05) return false;
    if (_polygonSelfIntersects(poly)) return false;
    return poly.every(_isFinitePoint);
  }

  static List<Offset> _radialOffsetPolygon(
      List<Offset> input, double distance) {
    final poly = _normalizePolygon(input);
    if (poly.length < 3) return const [];
    final center = _centroid(poly);
    final out = <Offset>[];
    for (final p in poly) {
      final fromCenter = p - center;
      final len = fromCenter.distance;
      if (len < _eps) continue;
      if (distance >= 0) {
        final inward = math.min(distance, len * 0.65);
        out.add(p - fromCenter * (inward / len));
      } else {
        final outward = -distance;
        out.add(p + fromCenter * (outward / len));
      }
    }
    return _normalizePolygon(out);
  }

  static Offset _centroid(List<Offset> poly) {
    var x = 0.0;
    var y = 0.0;
    for (final p in poly) {
      x += p.dx;
      y += p.dy;
    }
    return Offset(x / poly.length, y / poly.length);
  }

  static bool _polygonSelfIntersects(List<Offset> poly) {
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % poly.length];
      for (var j = i + 1; j < poly.length; j++) {
        if ((i - j).abs() <= 1) continue;
        if (i == 0 && j == poly.length - 1) continue;
        final c = poly[j];
        final d = poly[(j + 1) % poly.length];
        if (_segmentsIntersectOrTouch(a, b, c, d)) return true;
      }
    }
    return false;
  }

  static Offset? _lineIntersection(_Line a, _Line b) {
    final cross = _cross(a.dir, b.dir);
    if (cross.abs() < _eps) return null;
    final delta = b.point - a.point;
    final t = _cross(delta, b.dir) / cross;
    return a.point + a.dir * t;
  }

  static List<_Interval> _scanIntervalsHorizontal(List<Offset> poly, double y) {
    final xs = <double>[];
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % poly.length];
      if ((a.dy <= y && b.dy > y) || (b.dy <= y && a.dy > y)) {
        final t = (y - a.dy) / (b.dy - a.dy);
        xs.add(a.dx + (b.dx - a.dx) * t);
      }
    }
    xs.sort();
    final out = <_Interval>[];
    for (var i = 0; i + 1 < xs.length; i += 2) {
      if (xs[i + 1] - xs[i] > 0.05) out.add(_Interval(xs[i], xs[i + 1]));
    }
    return out;
  }

  static List<_Interval> _scanIntervalsVertical(List<Offset> poly, double x) {
    final ys = <double>[];
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % poly.length];
      if ((a.dx <= x && b.dx > x) || (b.dx <= x && a.dx > x)) {
        final t = (x - a.dx) / (b.dx - a.dx);
        ys.add(a.dy + (b.dy - a.dy) * t);
      }
    }
    ys.sort();
    final out = <_Interval>[];
    for (var i = 0; i + 1 < ys.length; i += 2) {
      if (ys[i + 1] - ys[i] > 0.05) out.add(_Interval(ys[i], ys[i + 1]));
    }
    return out;
  }

  static List<_Interval> _subtractIntervals(
    List<_Interval> base,
    List<_Interval> cut,
  ) {
    if (base.isEmpty || cut.isEmpty) return base;
    final out = <_Interval>[];
    final cuts = List<_Interval>.from(cut)
      ..sort((a, b) => a.start.compareTo(b.start));
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

  static List<_Interval> _shrinkIntervals(
    List<_Interval> intervals,
    double margin,
  ) {
    if (margin <= 0) return intervals;
    return intervals
        .map((i) => _Interval(i.start + margin, i.end - margin))
        .where((i) => i.length > 0)
        .toList();
  }

  static List<PolyShape> _findInternalForbiddens(
      List<Offset> zone, List<PolyShape> all,
      {double margin = 0.0}) {
    return all.where((f) {
      final forbidden = _normalizePolygon(f.points);
      if (forbidden.length < 3) return false;
      if (_polygonsOverlap(zone, forbidden)) return true;
      return margin > 0 && _polygonDistance(zone, forbidden) <= margin;
    }).toList();
  }

  static bool _polygonsOverlap(List<Offset> a, List<Offset> b) {
    final pa = _normalizePolygon(a);
    final pb = _normalizePolygon(b);
    if (pa.length < 3 || pb.length < 3) return false;
    if (pb.any((p) => _isPointInPolygon(p, pa))) return true;
    if (pa.any((p) => _isPointInPolygon(p, pb))) return true;
    for (var i = 0; i < pa.length; i++) {
      final a1 = pa[i];
      final a2 = pa[(i + 1) % pa.length];
      for (var j = 0; j < pb.length; j++) {
        final b1 = pb[j];
        final b2 = pb[(j + 1) % pb.length];
        if (_segmentsIntersectOrTouch(a1, a2, b1, b2)) return true;
      }
    }
    return false;
  }

  static double _polygonDistance(List<Offset> a, List<Offset> b) {
    var best = double.infinity;
    for (var i = 0; i < a.length; i++) {
      final a1 = a[i];
      final a2 = a[(i + 1) % a.length];
      for (var j = 0; j < b.length; j++) {
        final b1 = b[j];
        final b2 = b[(j + 1) % b.length];
        if (_segmentsIntersectOrTouch(a1, a2, b1, b2)) return 0.0;
        best = math.min(best, _segmentDistance(a1, a2, b1, b2));
      }
    }
    return best;
  }

  static List<Offset> _normalizePolygon(List<Offset> pts) {
    final out = <Offset>[];
    for (final p in pts) {
      if (!_isFinitePoint(p)) continue;
      if (out.isEmpty || _dist(out.last, p) > 0.001) out.add(p);
    }
    if (out.length > 1 && _dist(out.first, out.last) < 0.001) {
      out.removeLast();
    }
    return out;
  }

  static List<Offset> _normalizePolyline(List<Offset> pts) {
    final out = <Offset>[];
    for (final p in pts) {
      if (!_isFinitePoint(p)) continue;
      if (out.isEmpty || _dist(out.last, p) > 0.001) out.add(p);
    }
    return out;
  }

  static _PolylineProjection? _closestPointOnPolyline(
    Offset p,
    List<Offset> line,
  ) {
    if (line.length < 2) return null;
    var bestPoint = line.first;
    var bestSegmentIndex = 0;
    var bestDistance = double.infinity;
    var bestAlong = 0.0;
    var along = 0.0;

    for (var i = 0; i < line.length - 1; i++) {
      final a = line[i];
      final b = line[i + 1];
      final q = _projectPointToSegment(p, a, b);
      final d = _dist(p, q);
      if (d < bestDistance) {
        bestDistance = d;
        bestPoint = q;
        bestSegmentIndex = i;
        bestAlong = along + _dist(a, q);
      }
      along += _dist(a, b);
    }

    return _PolylineProjection(
      bestPoint,
      bestSegmentIndex,
      bestAlong,
      bestDistance,
    );
  }

  static List<Offset> _polylineSubPath(
    List<Offset> line,
    _PolylineProjection from,
    _PolylineProjection to,
  ) {
    if (from.along <= to.along) {
      final out = <Offset>[from.point];
      for (var i = from.segmentIndex + 1; i <= to.segmentIndex; i++) {
        out.add(line[i]);
      }
      out.add(to.point);
      return _simplifyPath(out, minDistance: 0.03);
    }

    final out = <Offset>[from.point];
    for (var i = from.segmentIndex; i > to.segmentIndex; i--) {
      out.add(line[i]);
    }
    out.add(to.point);
    return _simplifyPath(out, minDistance: 0.03);
  }

  static bool _isPolylineFreeOfBlocked(
    List<Offset> line,
    List<List<Offset>> blocked,
  ) {
    for (var i = 0; i < line.length - 1; i++) {
      if (!_isSegmentFreeOfBlocked(line[i], line[i + 1], blocked)) {
        return false;
      }
    }
    return true;
  }

  static List<Offset> _simplifyPath(List<Offset> pts,
      {required double minDistance}) {
    if (pts.isEmpty) return const [];
    final out = <Offset>[pts.first];
    for (final p in pts.skip(1)) {
      if (_dist(out.last, p) >= minDistance) out.add(p);
    }
    if (out.length == 1 && pts.length > 1) out.add(pts.last);
    return out;
  }

  static double _pathDistance(List<Offset> pts) {
    var total = 0.0;
    for (var i = 1; i < pts.length; i++) {
      total += _dist(pts[i - 1], pts[i]);
    }
    return total;
  }

  static bool _isPointInPolygon(
    Offset p,
    List<Offset> poly, {
    bool includeBoundary = true,
  }) {
    if (poly.length < 3) return false;
    final boundaryDistance = _distanceToPolygon(p, poly);
    if (boundaryDistance < 0.01) return includeBoundary;
    var inside = false;
    var j = poly.length - 1;
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[j];
      final intersects = ((a.dy > p.dy) != (b.dy > p.dy)) &&
          (p.dx < (b.dx - a.dx) * (p.dy - a.dy) / (b.dy - a.dy) + a.dx);
      if (intersects) inside = !inside;
      j = i;
    }
    return inside;
  }

  static double _distanceToPolygon(Offset p, List<Offset> poly) {
    var best = double.infinity;
    for (var i = 0; i < poly.length; i++) {
      best = math.min(
          best, _distanceToSegment(p, poly[i], poly[(i + 1) % poly.length]));
    }
    return best;
  }

  static Offset _projectPointToSegment(Offset p, Offset a, Offset b) {
    final ab = b - a;
    final len2 = ab.dx * ab.dx + ab.dy * ab.dy;
    if (len2 <= _eps) return a;
    final t = (((p.dx - a.dx) * ab.dx + (p.dy - a.dy) * ab.dy) / len2)
        .clamp(0.0, 1.0);
    return Offset(a.dx + ab.dx * t, a.dy + ab.dy * t);
  }

  static double _distanceToSegment(Offset p, Offset a, Offset b) {
    return _dist(p, _projectPointToSegment(p, a, b));
  }

  static double _segmentDistance(
    Offset a,
    Offset b,
    Offset c,
    Offset d,
  ) {
    if (_segmentsIntersectOrTouch(a, b, c, d)) return 0.0;
    return math.min(
      math.min(_distanceToSegment(a, c, d), _distanceToSegment(b, c, d)),
      math.min(_distanceToSegment(c, a, b), _distanceToSegment(d, a, b)),
    );
  }

  static bool _segmentsIntersect(Offset a, Offset b, Offset c, Offset d) {
    final o1 = _orientation(a, b, c);
    final o2 = _orientation(a, b, d);
    final o3 = _orientation(c, d, a);
    final o4 = _orientation(c, d, b);
    if (o1 * o2 < 0 && o3 * o4 < 0) return true;
    return false;
  }

  static bool _segmentsIntersectOrTouch(
    Offset a,
    Offset b,
    Offset c,
    Offset d,
  ) {
    final o1 = _orientation(a, b, c);
    final o2 = _orientation(a, b, d);
    final o3 = _orientation(c, d, a);
    final o4 = _orientation(c, d, b);
    if (o1 * o2 < 0 && o3 * o4 < 0) return true;
    if (o1.abs() < _eps && _pointOnSegment(c, a, b)) return true;
    if (o2.abs() < _eps && _pointOnSegment(d, a, b)) return true;
    if (o3.abs() < _eps && _pointOnSegment(a, c, d)) return true;
    if (o4.abs() < _eps && _pointOnSegment(b, c, d)) return true;
    return false;
  }

  static bool _pointOnSegment(Offset p, Offset a, Offset b) {
    return p.dx >= math.min(a.dx, b.dx) - _eps &&
        p.dx <= math.max(a.dx, b.dx) + _eps &&
        p.dy >= math.min(a.dy, b.dy) - _eps &&
        p.dy <= math.max(a.dy, b.dy) + _eps;
  }

  static double _orientation(Offset a, Offset b, Offset c) {
    return _cross(b - a, c - a);
  }

  static double _polygonArea(List<Offset> poly) {
    var sum = 0.0;
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % poly.length];
      sum += a.dx * b.dy - b.dx * a.dy;
    }
    return sum * 0.5;
  }

  static _BBox _bbox(List<Offset> pts) {
    var minX = pts.first.dx;
    var maxX = pts.first.dx;
    var minY = pts.first.dy;
    var maxY = pts.first.dy;
    for (final p in pts.skip(1)) {
      minX = math.min(minX, p.dx);
      maxX = math.max(maxX, p.dx);
      minY = math.min(minY, p.dy);
      maxY = math.max(maxY, p.dy);
    }
    return _BBox(minX, maxX, minY, maxY);
  }

  static bool _isFinitePoint(Offset p) => p.dx.isFinite && p.dy.isFinite;

  static double _dist(Offset a, Offset b) => (a - b).distance;

  static double _cross(Offset a, Offset b) => a.dx * b.dy - a.dy * b.dx;
}

class _Segment {
  final Offset start;
  final Offset end;
  final int laneIndex;
  final int intervalIndex;

  const _Segment(
    this.start,
    this.end, {
    this.laneIndex = 0,
    this.intervalIndex = 0,
  });

  _Segment reversed() => _Segment(
        end,
        start,
        laneIndex: laneIndex,
        intervalIndex: intervalIndex,
      );
}

class _Interval {
  final double start;
  final double end;
  const _Interval(this.start, this.end);
  double get length => end - start;
}

class _BBox {
  final double minX;
  final double maxX;
  final double minY;
  final double maxY;
  const _BBox(this.minX, this.maxX, this.minY, this.maxY);
  double get width => maxX - minX;
  double get height => maxY - minY;
}

class _Projection {
  final Offset point;
  final int edgeIndex;
  const _Projection(this.point, this.edgeIndex);
}

class _PolylineProjection {
  final Offset point;
  final int segmentIndex;
  final double along;
  final double distance;
  const _PolylineProjection(
    this.point,
    this.segmentIndex,
    this.along,
    this.distance,
  );
}

class _Line {
  final Offset point;
  final Offset dir;
  const _Line(this.point, this.dir);
}

class _Logger {
  final bool enabled;
  _Logger(this.enabled);

  void log(String message) {
    if (enabled && kDebugMode) {
      debugPrint('[CleaningRoutePlanner] $message');
    }
  }
}
