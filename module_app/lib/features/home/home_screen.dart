import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:go_router/go_router.dart';

import 'package:hello_flutter/core/wifi_connection.dart';

/// ============================================================
/// Core picker
/// ============================================================
final controlCoreProvider = StateProvider<String>((ref) => 'Модуль Для Снега');

/// ============================================================
/// Battery mock (потом заменим на реальную телеметрию)
/// ============================================================
final batteryPercentProvider = StateProvider<int>((ref) => 46);

class HomeScreen extends ConsumerStatefulWidget {
  const HomeScreen({super.key});

  @override
  ConsumerState<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends ConsumerState<HomeScreen>
    with TickerProviderStateMixin {
  late final AnimationController _flareCtrl;
  late final AnimationController _robotCtrl;

  late final Animation<double> _robotScale;
  late final Animation<double> _robotGlowBoost;

  @override
  void initState() {
    super.initState();

    _flareCtrl = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 900),
    );

    // Синхронизируем начальное состояние с подключением
    WidgetsBinding.instance.addPostFrameCallback((_) {
      final wifi = ref.read(wifiConnectionProvider);
      if (wifi.isConnected) {
        _flareCtrl.value = 1.0;
      }
    });

    _robotCtrl = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 520),
    );

    _robotScale = TweenSequence<double>([
      TweenSequenceItem(
        tween: Tween(begin: 1.0, end: 1.03)
            .chain(CurveTween(curve: Curves.easeOutCubic)),
        weight: 55,
      ),
      TweenSequenceItem(
        tween: Tween(begin: 1.03, end: 1.0)
            .chain(CurveTween(curve: Curves.easeInCubic)),
        weight: 45,
      ),
    ]).animate(_robotCtrl);

    _robotGlowBoost = TweenSequence<double>([
      TweenSequenceItem(
        tween: Tween(begin: 0.0, end: 1.0)
            .chain(CurveTween(curve: Curves.easeOutCubic)),
        weight: 50,
      ),
      TweenSequenceItem(
        tween: Tween(begin: 1.0, end: 0.0)
            .chain(CurveTween(curve: Curves.easeInCubic)),
        weight: 50,
      ),
    ]).animate(_robotCtrl);
  }

  @override
  void dispose() {
    _flareCtrl.dispose();
    _robotCtrl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final wifi = ref.watch(wifiConnectionProvider);
    final core = ref.watch(controlCoreProvider);
    // Получаем батарею: если включена проверка Wi-Fi - используем только данные из WebSocket, иначе из настроек
    final pingCheckEnabled = ref.watch(wifiPingCheckProvider);
    final batteryFromSettings = ref.watch(batteryPercentProvider);
    final battery = pingCheckEnabled
        ? (wifi.batteryPercent ??
            batteryFromSettings) // При включенной проверке приоритет WebSocket, fallback на настройки
        : batteryFromSettings; // При выключенной проверке только настройки

    ref.listen<WifiConnectionState>(wifiConnectionProvider, (prev, next) {
      if (prev == null) return;
      if (prev.isConnected != next.isConnected && !next.isConnecting) {
        if (next.isConnected) {
          // При подключении - плавно до максимума и остаемся там
          _flareCtrl.animateTo(1.0,
              duration: const Duration(milliseconds: 600),
              curve: Curves.easeOutCubic);
          _robotCtrl.forward(from: 0);
        } else {
          // При отключении - плавно до нуля с анимацией робота
          _flareCtrl.animateTo(0.0,
              duration: const Duration(milliseconds: 500),
              curve: Curves.easeInCubic);
          _robotCtrl.forward(from: 0);
        }
      }
    });

    // Черно-белая цветовая схема
    const accentWhite = Colors.white;
    const accentGray = Color(0xFF9E9E9E);

    return Scaffold(
      body: LayoutBuilder(
        builder: (context, constraints) {
          final scaleH = constraints.maxHeight / 820.0;
          final scaleW = constraints.maxWidth / 390.0;
          final uiScale = math.min(scaleH, scaleW).clamp(0.78, 1.0);

          double s(double v) => v * uiScale;

          final padH = s(18).clamp(12.0, 18.0);
          final padTop = s(14).clamp(10.0, 14.0);

          final gapS = s(10).clamp(6.0, 10.0);
          final gapM = s(14).clamp(9.0, 14.0);
          final gapL = s(16).clamp(10.0, 16.0);

          final topIconSize = s(44).clamp(36.0, 44.0);
          final topIconGlyph = s(20).clamp(17.0, 20.0);

          final brandSize = s(34).clamp(28.0, 34.0);

          return Stack(
            children: [
              Positioned.fill(
                child: _PremiumStaticBackground(isConnected: wifi.isConnected),
              ),
              Positioned.fill(
                child: IgnorePointer(
                  child: AnimatedBuilder(
                    animation: _flareCtrl,
                    builder: (_, __) {
                      // Используем позицию контроллера напрямую для плавного перехода
                      // Когда подключено - контроллер на 1.0, когда отключено - на 0.0
                      final intensity = _flareCtrl.value;
                      return _ConnectionFlareOverlay(
                        isConnected: wifi.isConnected,
                        intensity: intensity,
                      );
                    },
                  ),
                ),
              ),
              const Positioned.fill(child: _VignetteOverlay()),
              SafeArea(
                child: Padding(
                  padding: EdgeInsets.fromLTRB(padH, padTop, padH, padTop),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      // TOP BAR
                      Row(
                        children: [
                          _BrandMark(size: brandSize),
                          SizedBox(width: s(10)),
                          Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(
                                'AutoBot',
                                style: TextStyle(
                                  fontSize: s(20).clamp(16.5, 20.0),
                                  fontWeight: FontWeight.w900,
                                  letterSpacing: 0.35,
                                ),
                              ),
                              SizedBox(height: s(2).clamp(1.0, 2.0)),
                              Row(
                                children: [
                                  Text(
                                    'Premium Control',
                                    style: TextStyle(
                                      fontSize: s(12).clamp(10.0, 12.0),
                                      color: Colors.white.withOpacity(0.68),
                                      letterSpacing: 0.2,
                                    ),
                                  ),
                                  SizedBox(width: s(8).clamp(6.0, 8.0)),
                                  _WifiStatusIndicator(
                                    isConnected: wifi.isConnected,
                                    isBusy: wifi.isConnecting,
                                    deviceName: null,
                                    size: s(14).clamp(12.0, 14.0),
                                  ),
                                ],
                              ),
                            ],
                          ),
                          const Spacer(),
                          _IconGlassButton(
                            size: topIconSize,
                            glyphSize: topIconGlyph,
                            icon: Icons.map_rounded,
                            tooltip: 'Карты',
                            iconColor: accentWhite,
                            onTap: () => context.go('/maps'),
                          ),
                          SizedBox(width: s(10)),
                          _IconGlassButton(
                            size: topIconSize,
                            glyphSize: topIconGlyph,
                            icon: Icons.gps_fixed_rounded,
                            tooltip: 'GPS Отладка',
                            iconColor: accentWhite,
                            onTap: () => context.go('/gps'),
                          ),
                          SizedBox(width: s(10)),
                          _IconGlassButton(
                            size: topIconSize,
                            glyphSize: topIconGlyph,
                            icon: Icons.science_rounded,
                            tooltip: 'Test Lab',
                            iconColor: accentWhite,
                            onTap: () => context.go('/test'),
                          ),
                          SizedBox(width: s(10)),
                          _IconGlassButton(
                            size: topIconSize,
                            glyphSize: topIconGlyph,
                            icon: Icons.tune_rounded,
                            tooltip: 'Настройки',
                            iconColor: accentWhite,
                            onTap: () {
                              showModalBottomSheet(
                                context: context,
                                backgroundColor: Colors.transparent,
                                barrierColor: Colors.black.withOpacity(0.55),
                                builder: (_) => const _QuickSheet(),
                              );
                            },
                          ),
                        ],
                      ),

                      SizedBox(height: gapM),

                      // STATUS CARD (такой же, как на втором экране)
                      _StatusPanel(
                        uiScale: uiScale,
                        wifi: wifi,
                        batteryPercent: battery,
                        onToggle: () async {
                          final ctrl =
                              ref.read(wifiConnectionProvider.notifier);

                          if (wifi.isConnected) {
                            await ctrl.disconnect();
                            return;
                          }

                          await ctrl.connect();
                        },
                      ),

                      SizedBox(height: gapL),

                      Expanded(
                        child: Center(
                          child: FittedBox(
                            fit: BoxFit.contain,
                            child: SizedBox(
                              width: 360,
                              height: 380,
                              child: RepaintBoundary(
                                child: AnimatedBuilder(
                                  animation: _robotCtrl,
                                  builder: (_, __) {
                                    return Transform.scale(
                                      scale: _robotScale.value,
                                      child: _RobotStable(
                                        neon: accentWhite,
                                        boost: _robotGlowBoost.value,
                                        boostColor: wifi.isConnected
                                            ? accentWhite
                                            : accentGray,
                                        core: core,
                                      ),
                                    );
                                  },
                                ),
                              ),
                            ),
                          ),
                        ),
                      ),

                      SizedBox(height: gapS),

                      Row(
                        children: [
                          Expanded(
                            child: _BigActionCard(
                              uiScale: uiScale,
                              title: 'Ручное Управление',
                              subtitle: 'Джойстик + запись маршрута',
                              icon: Icons.sports_esports_rounded,
                              border: accentWhite.withOpacity(0.22),
                              glow: accentWhite.withOpacity(0.12),
                              onTap: () => _navigateToManual(context),
                            ),
                          ),
                          SizedBox(width: s(12)),
                          Expanded(
                            child: _BigActionCard(
                              uiScale: uiScale,
                              title: 'Автоматический Режим',
                              subtitle:
                                  'Уборка территории в автоматическом режиме',
                              icon: Icons.route_rounded,
                              border: accentWhite.withOpacity(0.22),
                              glow: accentWhite.withOpacity(0.12),
                              onTap: () => _navigateToAuto(context),
                            ),
                          ),
                        ],
                      ),

                      SizedBox(height: gapS),

                      Center(
                        child: _CapsuleGlassButton(
                          uiScale: uiScale,
                          text: core,
                          border: accentWhite.withOpacity(0.22),
                          onTap: () => _showCorePicker(context),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          );
        },
      ),
    );
  }

  void _showCorePicker(BuildContext context) {
    showModalBottomSheet(
      context: context,
      backgroundColor: Colors.transparent,
      barrierColor: Colors.black.withOpacity(0.55),
      builder: (_) {
        final selected = ref.read(controlCoreProvider);
        return _CorePickerSheet(
          selected: selected,
          onSelect: (value) {
            ref.read(controlCoreProvider.notifier).state = value;
            Navigator.pop(context);
          },
        );
      },
    );
  }

  void _navigateToManual(BuildContext context) {
    final wifi = ref.read(wifiConnectionProvider);
    // Если подключено по Wi-Fi и включена проверка - используем ТОЛЬКО реальные данные из WebSocket
    // Настройки не учитываются когда подключено по Wi-Fi
    final pingCheckEnabled = ref.read(wifiPingCheckProvider);
    final batteryFromSettings = ref.read(batteryPercentProvider);

    int? battery;
    if (wifi.isConnected && pingCheckEnabled && wifi.batteryPercent != null) {
      // Подключено по Wi-Fi - используем ТОЛЬКО реальное значение из WebSocket
      battery = wifi.batteryPercent;
    } else {
      // Не подключено или проверка выключена - используем настройки
      battery = batteryFromSettings;
    }

    // Показываем предупреждение только если робот подключен и заряд <= 40%
    // Используем то же значение, что и для отображения
    if (wifi.isConnected && battery != null && battery <= 40) {
      _showLowBatteryWarning(
        context,
        onContinue: () {
          Navigator.pop(context);
          context.go('/manual');
        },
      );
    } else {
      context.go('/manual');
    }
  }

  void _navigateToAuto(BuildContext context) {
    final wifi = ref.read(wifiConnectionProvider);
    // Если подключено по Wi-Fi и включена проверка - используем ТОЛЬКО реальные данные из WebSocket
    // Настройки не учитываются когда подключено по Wi-Fi
    final pingCheckEnabled = ref.read(wifiPingCheckProvider);
    final batteryFromSettings = ref.read(batteryPercentProvider);

    int? battery;
    if (wifi.isConnected && pingCheckEnabled && wifi.batteryPercent != null) {
      // Подключено по Wi-Fi - используем ТОЛЬКО реальное значение из WebSocket
      battery = wifi.batteryPercent;
    } else {
      // Не подключено или проверка выключена - используем настройки
      battery = batteryFromSettings;
    }

    // Показываем предупреждение только если робот подключен и заряд <= 40%
    // Используем то же значение, что и для отображения
    if (wifi.isConnected && battery != null && battery <= 40) {
      _showLowBatteryWarning(
        context,
        onContinue: () {
          Navigator.pop(context);
          context.go('/auto');
        },
      );
    } else {
      context.go('/auto');
    }
  }

  void _showLowBatteryWarning(BuildContext context,
      {required VoidCallback onContinue}) {
    showDialog(
      context: context,
      barrierColor: Colors.black.withOpacity(0.7),
      builder: (context) => _LowBatteryDialog(
        batteryPercent: ref.read(batteryPercentProvider),
        onContinue: onContinue,
        onCancel: () => Navigator.pop(context),
      ),
    );
  }
}

/// ============================================================
/// AdaptiveText — без троеточий, подбирает размер шрифта
/// ============================================================
class AdaptiveText extends StatelessWidget {
  final String text;
  final TextStyle style;
  final int maxLines;
  final double minFontSize;
  final double maxFontSize;
  final TextAlign align;

  const AdaptiveText(
    this.text, {
    super.key,
    required this.style,
    required this.maxLines,
    required this.minFontSize,
    required this.maxFontSize,
    this.align = TextAlign.start,
  });

  @override
  Widget build(BuildContext context) {
    final scaler = MediaQuery.textScalerOf(context);

    return LayoutBuilder(
      builder: (context, constraints) {
        final width =
            constraints.maxWidth.isFinite ? constraints.maxWidth : 300.0;

        double lo = minFontSize;
        double hi = maxFontSize;
        double best = minFontSize;

        for (int i = 0; i < 14; i++) {
          final mid = (lo + hi) / 2;

          final tp = TextPainter(
            text: TextSpan(text: text, style: style.copyWith(fontSize: mid)),
            textDirection: TextDirection.ltr,
            maxLines: maxLines,
            textAlign: align,
            textScaler: scaler,
          )..layout(maxWidth: width);

          final fits = !tp.didExceedMaxLines;
          if (fits) {
            best = mid;
            lo = mid;
          } else {
            hi = mid;
          }
        }

        return Text(
          text,
          textAlign: align,
          maxLines: maxLines,
          softWrap: true,
          overflow: TextOverflow.visible,
          style: style.copyWith(fontSize: best),
          textHeightBehavior: const TextHeightBehavior(
            applyHeightToFirstAscent: false,
            applyHeightToLastDescent: false,
          ),
        );
      },
    );
  }
}

/// ============================================================
/// Action card
/// ============================================================
class _BigActionCard extends StatelessWidget {
  final double uiScale;
  final String title;
  final String subtitle;
  final IconData icon;
  final Color border;
  final Color glow;
  final VoidCallback onTap;

  const _BigActionCard({
    required this.uiScale,
    required this.title,
    required this.subtitle,
    required this.icon,
    required this.border,
    required this.glow,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    double s(double v) => v * uiScale;
    final pad = s(14).clamp(9.0, 14.0);
    final minH = s(118).clamp(92.0, 118.0);

    return InkWell(
      borderRadius: BorderRadius.circular(20),
      onTap: onTap,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(20),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 14, sigmaY: 14),
          child: ConstrainedBox(
            constraints: BoxConstraints(minHeight: minH),
            child: Container(
              padding: EdgeInsets.all(pad),
              decoration: BoxDecoration(
                color: Colors.white.withOpacity(0.06),
                borderRadius: BorderRadius.circular(20),
                border: Border.all(color: border),
                boxShadow: [
                  BoxShadow(color: glow, blurRadius: 18, spreadRadius: 1)
                ],
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  _NeonIconBadge(icon: icon, size: s(44).clamp(36.0, 44.0)),
                  SizedBox(height: s(10).clamp(6.0, 10.0)),
                  AdaptiveText(
                    title,
                    maxLines: 2,
                    minFontSize: s(12.0).clamp(10.5, 12.0),
                    maxFontSize: s(14.5).clamp(12.0, 14.5),
                    style: const TextStyle(
                      fontWeight: FontWeight.w900,
                      letterSpacing: 0.1,
                      height: 1.10,
                    ),
                  ),
                  SizedBox(height: s(6).clamp(4.0, 6.0)),
                  AdaptiveText(
                    subtitle,
                    maxLines: 2,
                    minFontSize: s(10.5).clamp(9.5, 10.5),
                    maxFontSize: s(12.0).clamp(10.5, 12.0),
                    style: TextStyle(
                      fontWeight: FontWeight.w700,
                      height: 1.15,
                      color: Colors.white.withOpacity(0.72),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

/// ============================================================
/// Background
/// ============================================================
class _PremiumStaticBackground extends StatelessWidget {
  final bool isConnected;
  const _PremiumStaticBackground({required this.isConnected});

  @override
  Widget build(BuildContext context) {
    // Черно-белый фон
    const bg0 = Color(0xFF000000);
    const bg1 = Color(0xFF1A1A1A);
    const bg2 = Color(0xFF2A2A2A);

    const tintWhite = Colors.white;
    const tintGray = Color(0xFF6E6E6E);
    final tint = isConnected ? tintWhite : tintGray;

    return Stack(
      children: [
        const Positioned.fill(
          child: DecoratedBox(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment(-0.9, -1.0),
                end: Alignment(1.0, 1.0),
                colors: [bg0, bg1, bg2],
                stops: [0.0, 0.55, 1.0],
              ),
            ),
          ),
        ),
        // Светлый градиент сверху
        Positioned.fill(
          child: Opacity(
            opacity: 0.18,
            child: DecoratedBox(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter,
                  colors: [
                    Colors.white.withOpacity(0.35),
                    Colors.transparent,
                  ],
                  stops: const [0.0, 0.30],
                ),
              ),
            ),
          ),
        ),
        const Positioned.fill(
          child: Opacity(
            opacity: 0.14,
            child: DecoratedBox(
              decoration: BoxDecoration(
                gradient: RadialGradient(
                  center: Alignment(0.0, -0.2),
                  radius: 1.15,
                  colors: [Colors.white, Colors.transparent],
                  stops: [0.0, 1.0],
                ),
              ),
            ),
          ),
        ),
        Positioned.fill(
          child: Opacity(
            opacity: 0.20,
            child: DecoratedBox(
              decoration: BoxDecoration(
                gradient: RadialGradient(
                  center: const Alignment(-0.55, -0.65),
                  radius: 1.10,
                  colors: [Colors.white.withOpacity(0.15), Colors.transparent],
                  stops: const [0.0, 1.0],
                ),
              ),
            ),
          ),
        ),
        Positioned.fill(
          child: Opacity(
            opacity: 0.18,
            child: DecoratedBox(
              decoration: BoxDecoration(
                gradient: RadialGradient(
                  center: const Alignment(0.65, 0.55),
                  radius: 1.25,
                  colors: [tint, Colors.transparent],
                  stops: const [0.0, 1.0],
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }
}

class _ConnectionFlareOverlay extends StatelessWidget {
  final bool isConnected;
  final double intensity;

  const _ConnectionFlareOverlay({
    required this.isConnected,
    required this.intensity,
  });

  @override
  Widget build(BuildContext context) {
    if (intensity <= 0.001) return const SizedBox.shrink();

    const white = Colors.white;
    const gray = Color(0xFF6E6E6E);
    final c = isConnected ? white : gray;

    final op = (0.60 * intensity).clamp(0.0, 0.60);

    return Opacity(
      opacity: op,
      child: DecoratedBox(
        decoration: BoxDecoration(
          gradient: RadialGradient(
            center: const Alignment(0.62, 0.30),
            radius: 1.25,
            colors: [c, Colors.transparent],
            stops: const [0.0, 1.0],
          ),
        ),
      ),
    );
  }
}

class _VignetteOverlay extends StatelessWidget {
  const _VignetteOverlay();

  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: BoxDecoration(
        gradient: RadialGradient(
          center: const Alignment(0.0, -0.1),
          radius: 1.15,
          colors: [Colors.transparent, Colors.black.withOpacity(0.58)],
          stops: const [0.55, 1.0],
        ),
      ),
    );
  }
}

/// ============================================================
/// Glass card
/// ============================================================
class _GlassCard extends StatelessWidget {
  final Widget child;
  final Color borderColor;

  const _GlassCard({
    required this.child,
    required this.borderColor,
  });

  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(22),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 16, sigmaY: 16),
        child: Container(
          decoration: BoxDecoration(
            color: Colors.white.withOpacity(0.06),
            borderRadius: BorderRadius.circular(22),
            border: Border.all(color: borderColor),
          ),
          child: child,
        ),
      ),
    );
  }
}

/// ============================================================
/// Robot hero
/// ============================================================
class _RobotStable extends StatelessWidget {
  final Color neon;
  final double boost;
  final Color boostColor;
  final String core;

  const _RobotStable({
    required this.neon,
    required this.boost,
    required this.boostColor,
    required this.core,
  });

  @override
  Widget build(BuildContext context) {
    const double glowSize = 320;
    const double robotHeight = 330;

    final extra = (0.22 * boost).clamp(0.0, 0.22);

    return SizedBox(
      width: 360,
      height: 380,
      child: Stack(
        alignment: Alignment.center,
        children: [
          // Glow эффект (фон)
          SizedBox(
            width: glowSize,
            height: glowSize,
            child: DecoratedBox(
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: neon.withOpacity(0.20 + extra),
                    blurRadius: 60 + (18 * boost),
                    spreadRadius: 10,
                  ),
                  BoxShadow(
                    color: boostColor.withOpacity(0.13 * boost),
                    blurRadius: 90,
                    spreadRadius: 14,
                  ),
                ],
              ),
            ),
          ),

          // Робот с плавной анимацией перехода
          SizedBox(
            height: robotHeight,
            child: AnimatedSwitcher(
              duration: const Duration(milliseconds: 600),
              transitionBuilder: (Widget child, Animation<double> animation) {
                return FadeTransition(
                  opacity: animation,
                  child: ScaleTransition(
                    scale: Tween<double>(begin: 0.9, end: 1.0).animate(
                      CurvedAnimation(
                        parent: animation,
                        curve: Curves.easeOutCubic,
                      ),
                    ),
                    child: child,
                  ),
                );
              },
              child: Image.asset(
                core == 'Теннисный Робот'
                    ? 'assets/images/tennisbot.png'
                    : 'assets/images/robot.png',
                key: ValueKey<String>(core),
                fit: BoxFit.contain,
                filterQuality: FilterQuality.high,
                gaplessPlayback: true,
                errorBuilder: (_, __, ___) => Icon(
                  Icons.image_not_supported_outlined,
                  size: 46,
                  color: Colors.white.withOpacity(0.55),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

/// ============================================================
/// Top UI pieces
/// ============================================================
class _BrandMark extends StatelessWidget {
  final double size;
  const _BrandMark({required this.size});

  @override
  Widget build(BuildContext context) {
    return Container(
      width: size,
      height: size,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(12),
        gradient: const LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [Colors.white, Color(0xFF9E9E9E)],
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.white.withOpacity(0.20),
            blurRadius: 14,
            spreadRadius: 1,
          ),
        ],
      ),
      child: Center(
        child: Container(
          width: size * 0.47,
          height: size * 0.47,
          decoration: BoxDecoration(
            color: Colors.black.withOpacity(0.35),
            borderRadius: BorderRadius.circular(6),
          ),
          child: Center(
            child: Icon(Icons.ac_unit_rounded, size: size * 0.35),
          ),
        ),
      ),
    );
  }
}

class _IconGlassButton extends StatelessWidget {
  final double size;
  final double glyphSize;
  final IconData icon;
  final String tooltip;
  final Color iconColor;
  final VoidCallback onTap;

  const _IconGlassButton({
    required this.size,
    required this.glyphSize,
    required this.icon,
    required this.tooltip,
    required this.iconColor,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return Tooltip(
      message: tooltip,
      child: InkWell(
        borderRadius: BorderRadius.circular(14),
        onTap: onTap,
        child: ClipRRect(
          borderRadius: BorderRadius.circular(14),
          child: BackdropFilter(
            filter: ImageFilter.blur(sigmaX: 14, sigmaY: 14),
            child: Container(
              width: size,
              height: size,
              decoration: BoxDecoration(
                color: Colors.white.withOpacity(0.06),
                borderRadius: BorderRadius.circular(14),
                border: Border.all(color: Colors.white.withOpacity(0.10)),
              ),
              child: Icon(icon, size: glyphSize, color: iconColor),
            ),
          ),
        ),
      ),
    );
  }
}

class _NeonIconBadge extends StatelessWidget {
  final IconData icon;
  final double size;
  const _NeonIconBadge({required this.icon, required this.size});

  @override
  Widget build(BuildContext context) {
    return Container(
      width: size,
      height: size,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(16),
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.white.withOpacity(0.25),
            Colors.white.withOpacity(0.08),
          ],
        ),
        border: Border.all(color: Colors.white.withOpacity(0.20)),
        boxShadow: [
          BoxShadow(
            color: Colors.white.withOpacity(0.10),
            blurRadius: 16,
            spreadRadius: 1,
          ),
        ],
      ),
      child:
          Icon(icon, size: size * 0.50, color: Colors.white.withOpacity(0.92)),
    );
  }
}

/// ============================================================
/// Core selector
/// ============================================================
class _CapsuleGlassButton extends StatelessWidget {
  final double uiScale;
  final String text;
  final Color border;
  final VoidCallback onTap;

  const _CapsuleGlassButton({
    required this.uiScale,
    required this.text,
    required this.border,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    double s(double v) => v * uiScale;

    return InkWell(
      borderRadius: BorderRadius.circular(999),
      onTap: onTap,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(999),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 16, sigmaY: 16),
          child: Container(
            padding: EdgeInsets.symmetric(
              horizontal: s(14).clamp(10.0, 14.0),
              vertical: s(10).clamp(8.0, 10.0),
            ),
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.06),
              borderRadius: BorderRadius.circular(999),
              border: Border.all(color: border),
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Text(
                  text,
                  style: TextStyle(
                    fontSize: s(12).clamp(10.0, 12.0),
                    letterSpacing: 0.35,
                    color: Colors.white.withOpacity(0.86),
                    fontWeight: FontWeight.w900,
                  ),
                ),
                SizedBox(width: s(8).clamp(6.0, 8.0)),
                Icon(Icons.expand_more_rounded,
                    size: s(18).clamp(16.0, 18.0),
                    color: Colors.white.withOpacity(0.70)),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

/// ============================================================
/// Sheets
/// ============================================================
class _CorePickerSheet extends StatelessWidget {
  final String selected;
  final ValueChanged<String> onSelect;

  const _CorePickerSheet({required this.selected, required this.onSelect});

  @override
  Widget build(BuildContext context) {
    const accent = Colors.white;

    return Padding(
      padding: const EdgeInsets.fromLTRB(14, 0, 14, 14),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(26),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 18, sigmaY: 18),
          child: Container(
            padding: const EdgeInsets.all(14),
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.06),
              borderRadius: BorderRadius.circular(26),
              border: Border.all(color: accent.withOpacity(0.22)),
            ),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Container(
                  width: 48,
                  height: 5,
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(999),
                    color: Colors.white.withOpacity(0.18),
                  ),
                ),
                const SizedBox(height: 14),
                Row(
                  children: [
                    const Icon(Icons.layers_rounded, color: accent),
                    const SizedBox(width: 10),
                    const Expanded(
                      child: Text(
                        'Выбор Модуля',
                        style: TextStyle(fontWeight: FontWeight.w900),
                      ),
                    ),
                    IconButton(
                      onPressed: () => Navigator.pop(context),
                      icon: const Icon(Icons.close_rounded),
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                _CoreOption(
                  title: 'Модуль Для Снега',
                  selected: selected == 'Модуль Для Снега',
                  onTap: () => onSelect('Модуль Для Снега'),
                ),
                const SizedBox(height: 10),
                _CoreOption(
                  title: 'Теннисный Робот',
                  selected: selected == 'Теннисный Робот',
                  onTap: () => onSelect('Теннисный Робот'),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class _CoreOption extends StatelessWidget {
  final String title;
  final bool selected;
  final VoidCallback onTap;

  const _CoreOption({
    required this.title,
    required this.selected,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    const accent = Colors.white;

    return InkWell(
      borderRadius: BorderRadius.circular(18),
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(18),
          color: Colors.white.withOpacity(0.05),
          border: Border.all(
            color:
                selected ? accent.withOpacity(0.35) : accent.withOpacity(0.14),
          ),
          boxShadow: selected
              ? [
                  BoxShadow(
                    color: accent.withOpacity(0.14),
                    blurRadius: 18,
                    spreadRadius: 1,
                  )
                ]
              : null,
        ),
        child: Row(
          children: [
            Container(
              width: 42,
              height: 42,
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(16),
                color: accent.withOpacity(selected ? 0.14 : 0.10),
                border: Border.all(
                    color: accent.withOpacity(selected ? 0.28 : 0.18)),
              ),
              child: Icon(
                selected ? Icons.check_circle_rounded : Icons.circle_outlined,
                color: accent,
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: Text(
                title,
                style: const TextStyle(fontWeight: FontWeight.w900),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _BatterySetting extends StatelessWidget {
  final int value;
  final ValueChanged<int> onChanged;

  const _BatterySetting({
    required this.value,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    const accent = Colors.white;

    // Определяем цвет в зависимости от уровня заряда
    Color batteryColor;
    if (value <= 20) {
      batteryColor = const Color(0xFFCC6666);
    } else if (value <= 50) {
      batteryColor = const Color(0xFFCCAA66);
    } else {
      batteryColor = const Color(0xFF66CC66);
    }

    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(18),
        color: Colors.white.withOpacity(0.05),
        border: Border.all(color: accent.withOpacity(0.18)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.battery_full_rounded, color: batteryColor, size: 20),
              const SizedBox(width: 8),
              const Expanded(
                child: Text(
                  'Заряд батареи',
                  style: TextStyle(
                    fontWeight: FontWeight.w900,
                    fontSize: 14,
                  ),
                ),
              ),
              Text(
                '$value%',
                style: TextStyle(
                  fontWeight: FontWeight.w900,
                  fontSize: 14,
                  color: batteryColor,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Slider(
            value: value.toDouble(),
            min: 0,
            max: 100,
            divisions: 100,
            activeColor: batteryColor,
            inactiveColor: Colors.white.withOpacity(0.2),
            onChanged: (newValue) {
              onChanged(newValue.round());
            },
          ),
          const SizedBox(height: 4),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                '0%',
                style: TextStyle(
                  fontWeight: FontWeight.w700,
                  fontSize: 11,
                  color: Colors.white.withOpacity(0.5),
                ),
              ),
              Text(
                '100%',
                style: TextStyle(
                  fontWeight: FontWeight.w700,
                  fontSize: 11,
                  color: Colors.white.withOpacity(0.5),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class _SettingSwitch extends StatelessWidget {
  final String title;
  final String subtitle;
  final bool value;
  final ValueChanged<bool> onChanged;

  const _SettingSwitch({
    required this.title,
    required this.subtitle,
    required this.value,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    const accent = Colors.white;

    return InkWell(
      borderRadius: BorderRadius.circular(18),
      onTap: () => onChanged(!value),
      child: Container(
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(18),
          color: Colors.white.withOpacity(0.05),
          border: Border.all(color: accent.withOpacity(0.18)),
        ),
        child: Row(
          children: [
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    title,
                    style: const TextStyle(
                      fontWeight: FontWeight.w900,
                      fontSize: 14,
                    ),
                  ),
                  const SizedBox(height: 4),
                  Text(
                    subtitle,
                    style: TextStyle(
                      fontWeight: FontWeight.w700,
                      fontSize: 12,
                      color: Colors.white.withOpacity(0.70),
                      height: 1.15,
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(width: 12),
            Switch(
              value: value,
              onChanged: onChanged,
              activeThumbColor: accent,
            ),
          ],
        ),
      ),
    );
  }
}

class _QuickSheet extends StatelessWidget {
  const _QuickSheet();

  @override
  Widget build(BuildContext context) {
    const accent = Colors.white;

    return Padding(
      padding: const EdgeInsets.fromLTRB(14, 0, 14, 14),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(26),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 18, sigmaY: 18),
          child: Container(
            padding: const EdgeInsets.all(14),
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.06),
              borderRadius: BorderRadius.circular(26),
              border: Border.all(color: accent.withOpacity(0.22)),
            ),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Container(
                  width: 48,
                  height: 5,
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(999),
                    color: Colors.white.withOpacity(0.18),
                  ),
                ),
                const SizedBox(height: 14),
                Row(
                  children: [
                    const Icon(Icons.tune_rounded, color: accent),
                    const SizedBox(width: 10),
                    const Expanded(
                      child: Text(
                        'Настройки (пока базовые)',
                        style: TextStyle(fontWeight: FontWeight.w900),
                      ),
                    ),
                    IconButton(
                      onPressed: () => Navigator.pop(context),
                      icon: const Icon(Icons.close_rounded),
                    ),
                  ],
                ),
                const SizedBox(height: 12),
                Consumer(
                  builder: (context, ref, _) {
                    final pingCheckEnabled = ref.watch(wifiPingCheckProvider);
                    return _SettingSwitch(
                      title: 'Проверка Wi-Fi перед подключением',
                      subtitle: pingCheckEnabled
                          ? 'Проверяет доступность робота через /ping'
                          : 'Пропускает проверку (для тестирования)',
                      value: pingCheckEnabled,
                      onChanged: (value) {
                        ref
                            .read(wifiPingCheckProvider.notifier)
                            .setEnabled(value);
                      },
                    );
                  },
                ),
                const SizedBox(height: 12),
                Consumer(
                  builder: (context, ref, _) {
                    final batteryPercent = ref.watch(batteryPercentProvider);
                    return _BatterySetting(
                      value: batteryPercent,
                      onChanged: (value) {
                        ref.read(batteryPercentProvider.notifier).state = value;
                      },
                    );
                  },
                ),
                const SizedBox(height: 12),
                SizedBox(
                  width: double.infinity,
                  child: _WhiteGlassButton(
                    text: 'Закрыть',
                    onPressed: () => Navigator.pop(context),
                    isSecondary: false,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

/// ============================================================
/// Low battery warning dialog
/// ============================================================
class _LowBatteryDialog extends StatelessWidget {
  final int batteryPercent;
  final VoidCallback onContinue;
  final VoidCallback onCancel;

  const _LowBatteryDialog({
    required this.batteryPercent,
    required this.onContinue,
    required this.onCancel,
  });

  @override
  Widget build(BuildContext context) {
    const accent = Colors.white;
    final batteryColor = batteryPercent <= 20
        ? const Color(0xFFCC6666)
        : const Color(0xFFCCAA66);

    return Dialog(
      backgroundColor: Colors.transparent,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(26),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 18, sigmaY: 18),
          child: Container(
            padding: const EdgeInsets.all(20),
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.06),
              borderRadius: BorderRadius.circular(26),
              border: Border.all(color: batteryColor.withOpacity(0.4)),
            ),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Container(
                  padding: const EdgeInsets.all(16),
                  decoration: BoxDecoration(
                    color: batteryColor.withOpacity(0.15),
                    shape: BoxShape.circle,
                  ),
                  child: Icon(
                    Icons.battery_alert_rounded,
                    color: batteryColor,
                    size: 48,
                  ),
                ),
                const SizedBox(height: 20),
                const Text(
                  'Низкий заряд батареи',
                  style: TextStyle(
                    fontWeight: FontWeight.w900,
                    fontSize: 20,
                  ),
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 12),
                Text(
                  'Заряд батареи: $batteryPercent%\n\nРекомендуется зарядить робота перед началом работы.',
                  style: TextStyle(
                    fontWeight: FontWeight.w700,
                    fontSize: 14,
                    color: Colors.white.withOpacity(0.80),
                    height: 1.4,
                  ),
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 24),
                Row(
                  children: [
                    Flexible(
                      flex: 1,
                      child: _WhiteGlassButton(
                        text: 'Отмена',
                        onPressed: onCancel,
                        isSecondary: true,
                        isSmall: true,
                      ),
                    ),
                    const SizedBox(width: 10),
                    Flexible(
                      flex: 1,
                      child: _WhiteGlassButton(
                        text: 'Продолжить',
                        onPressed: onContinue,
                        isSecondary: false,
                        isSmall: true,
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

/// ============================================================
/// Beautiful contrast glass button (bright white with dark text)
/// ============================================================
class _WhiteGlassButton extends StatelessWidget {
  final String text;
  final VoidCallback? onPressed;
  final bool isSecondary;
  final bool isSmall;

  const _WhiteGlassButton({
    required this.text,
    this.onPressed,
    this.isSecondary = false,
    this.isSmall = false,
  });

  @override
  Widget build(BuildContext context) {
    return InkWell(
      borderRadius: BorderRadius.circular(16),
      onTap: onPressed,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(16),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 14, sigmaY: 14),
          child: Container(
            padding: EdgeInsets.symmetric(
              horizontal: isSmall ? 12 : 20,
              vertical: isSmall ? 10 : 14,
            ),
            decoration: BoxDecoration(
              gradient: isSecondary
                  ? null
                  : LinearGradient(
                      begin: Alignment.topLeft,
                      end: Alignment.bottomRight,
                      colors: [
                        Colors.white.withOpacity(0.95),
                        Colors.white.withOpacity(0.85),
                      ],
                    ),
              color: isSecondary
                  ? Colors.white.withOpacity(0.25)
                  : Colors.white.withOpacity(0.90),
              borderRadius: BorderRadius.circular(16),
              border: Border.all(
                color: Colors.white.withOpacity(isSecondary ? 0.50 : 0.95),
                width: 1.5,
              ),
              boxShadow: [
                BoxShadow(
                  color: Colors.white.withOpacity(0.35),
                  blurRadius: 20,
                  spreadRadius: 0,
                ),
                BoxShadow(
                  color: Colors.black.withOpacity(0.15),
                  blurRadius: 8,
                  spreadRadius: 0,
                ),
              ],
            ),
            child: Center(
              child: Text(
                text,
                style: TextStyle(
                  fontWeight: FontWeight.w900,
                  fontSize: isSmall ? 13 : 15,
                  color: Colors.black.withOpacity(0.92),
                  letterSpacing: 0.3,
                ),
                textAlign: TextAlign.center,
                overflow: TextOverflow.visible,
                softWrap: true,
              ),
            ),
          ),
        ),
      ),
    );
  }
}

/// ============================================================
/// Status Panel (такой же, как на втором экране)
/// ============================================================
class _StatusPanel extends StatelessWidget {
  final double uiScale;
  final WifiConnectionState wifi;
  final int batteryPercent;
  final VoidCallback onToggle;

  const _StatusPanel({
    required this.uiScale,
    required this.wifi,
    required this.batteryPercent,
    required this.onToggle,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    // Черно-белая цветовая схема
    const accentWhite = Colors.white;
    const accentGray = Color(0xFF6E6E6E);
    final accent = wifi.isConnected ? accentWhite : accentGray;

    String statusText;
    Color statusColor;
    if (wifi.isConnecting) {
      statusText = 'Подключение…';
      statusColor = Colors.white;
    } else if (wifi.isConnected) {
      statusText = 'Подключено';
      statusColor = Colors.green; // Зеленый для "Подключено"
    } else if (wifi.error != null) {
      statusText = wifi.error!;
      statusColor = Colors.red; // Красный для ошибки
    } else {
      statusText = 'Не подключено';
      statusColor = Colors.red; // Красный для "Не подключено"
    }

    return Container(
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          // Едва заметное неоновое свечение вокруг всей панели
          BoxShadow(
            color: statusColor.withOpacity(0.04),
            blurRadius: 8.0,
            spreadRadius: 0.2,
          ),
          BoxShadow(
            color: statusColor.withOpacity(0.03),
            blurRadius: 12.0,
            spreadRadius: 0.3,
          ),
        ],
      ),
      child: _GlassCard(
        borderColor: statusColor.withOpacity(0.15),
        child: Padding(
          padding: EdgeInsets.symmetric(
            horizontal: u(12).clamp(10.0, 12.0),
            vertical: u(10).clamp(8.0, 10.0),
          ),
          child: Row(
            children: [
              _BatteryChip(
                  uiScale: uiScale,
                  percent: batteryPercent,
                  isConnected: wifi.isConnected),
              SizedBox(width: u(10)),
              Expanded(
                child: Row(
                  children: [
                    Container(
                      decoration: BoxDecoration(
                        boxShadow: [
                          BoxShadow(
                            color: statusColor.withOpacity(0.06),
                            blurRadius: 4.0,
                            spreadRadius: 0.2,
                          ),
                        ],
                      ),
                      child: Icon(
                        wifi.isConnected
                            ? Icons.wifi_rounded
                            : Icons.wifi_off_rounded,
                        color: statusColor,
                        size: u(18).clamp(16.0, 18.0),
                      ),
                    ),
                    SizedBox(width: u(8)),
                    Expanded(
                      child: Text(
                        statusText,
                        style: TextStyle(
                          fontWeight: FontWeight.w900,
                          fontSize: u(11.5).clamp(10.5, 11.5),
                          color: statusColor,
                          shadows: [
                            // Едва заметное неоновое свечение
                            Shadow(
                              color: statusColor.withOpacity(0.1),
                              blurRadius: 4.0,
                              offset: const Offset(0, 0),
                            ),
                            Shadow(
                              color: statusColor.withOpacity(0.06),
                              blurRadius: 6.0,
                              offset: const Offset(0, 0),
                            ),
                          ],
                        ),
                        maxLines: 2,
                        softWrap: true,
                        overflow: TextOverflow.visible,
                      ),
                    ),
                  ],
                ),
              ),
              SizedBox(width: u(10)),
              _ConnectBtn(
                uiScale: uiScale,
                accent: accent,
                busy: wifi.isConnecting,
                isConnected: wifi.isConnected,
                onTap: onToggle,
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _BatteryChip extends StatelessWidget {
  final double uiScale;
  final int percent;
  final bool isConnected;
  const _BatteryChip({
    required this.uiScale,
    required this.percent,
    required this.isConnected,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    final p = percent.clamp(0, 100);

    // Определяем цвет в зависимости от уровня заряда и подключения
    Color batteryColor;
    if (!isConnected) {
      // Серый цвет, когда робот не подключен
      batteryColor = const Color(0xFF6E6E6E);
    } else if (p <= 20) {
      // Красный для низкого заряда
      batteryColor = const Color(0xFFCC6666);
    } else if (p <= 50) {
      // Желтый для среднего заряда
      batteryColor = const Color(0xFFCCAA66);
    } else {
      // Зеленый для высокого заряда
      batteryColor = const Color(0xFF66CC66);
    }

    return Container(
      padding: EdgeInsets.symmetric(
        horizontal: u(10).clamp(8.0, 10.0),
        vertical: u(8).clamp(6.0, 8.0),
      ),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(16),
        color: Colors.white.withOpacity(0.05),
        border: Border.all(color: batteryColor.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Icon(Icons.battery_full_rounded,
              size: u(18).clamp(16.0, 18.0), color: batteryColor),
          if (isConnected) ...[
            SizedBox(width: u(6)),
            Text('$p%',
                style: TextStyle(
                    fontWeight: FontWeight.w900,
                    fontSize: u(12.5).clamp(11.0, 12.5),
                    color: batteryColor)),
          ],
        ],
      ),
    );
  }
}

class _ConnectBtn extends StatelessWidget {
  final double uiScale;
  final Color accent;
  final bool busy;
  final bool isConnected;
  final VoidCallback onTap;

  const _ConnectBtn({
    required this.uiScale,
    required this.accent,
    required this.busy,
    required this.isConnected,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    double u(double v) => v * uiScale;
    final label = isConnected ? 'Отключить' : 'Подключить';

    return InkWell(
      borderRadius: BorderRadius.circular(16),
      onTap: busy ? null : onTap,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(16),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 14, sigmaY: 14),
          child: Container(
            padding: EdgeInsets.symmetric(
              horizontal: u(12).clamp(10.0, 12.0),
              vertical: u(10).clamp(8.0, 10.0),
            ),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [
                  accent.withOpacity(0.26),
                  Colors.white.withOpacity(0.05)
                ],
              ),
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: accent.withOpacity(0.45)),
            ),
            child: busy
                ? SizedBox(
                    width: u(14).clamp(12.0, 14.0),
                    height: u(14).clamp(12.0, 14.0),
                    child: CircularProgressIndicator(
                      strokeWidth: 2,
                      valueColor: AlwaysStoppedAnimation(accent),
                    ),
                  )
                : Text(
                    label,
                    style: TextStyle(
                      fontWeight: FontWeight.w900,
                      fontSize: u(12.0).clamp(10.8, 12.0),
                    ),
                  ),
          ),
        ),
      ),
    );
  }
}

/// ============================================================
/// Wi-Fi Status Indicator
/// ============================================================
class _WifiStatusIndicator extends StatelessWidget {
  final bool isConnected;
  final bool isBusy;
  final String? deviceName;
  final double size;

  const _WifiStatusIndicator({
    required this.isConnected,
    required this.isBusy,
    this.deviceName,
    required this.size,
  });

  @override
  Widget build(BuildContext context) {
    const goodWhite = Colors.white;
    const badGray = Color(0xFF6E6E6E);
    const busyWhite = Colors.white;

    final color = isConnected ? goodWhite : (isBusy ? busyWhite : badGray);
    final icon = isConnected
        ? Icons.wifi_rounded
        : (isBusy ? Icons.wifi_find_rounded : Icons.wifi_off_rounded);

    return Tooltip(
      message: isConnected
          ? (deviceName != null ? 'Подключено: $deviceName' : 'Подключено')
          : (isBusy ? 'Подключение...' : 'Не подключено'),
      child: Container(
        padding: EdgeInsets.symmetric(
          horizontal: size * 0.4,
          vertical: size * 0.2,
        ),
        decoration: BoxDecoration(
          color: color.withOpacity(0.15),
          borderRadius: BorderRadius.circular(size * 0.5),
          border: Border.all(
            color: color.withOpacity(0.4),
            width: 1.0,
          ),
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              icon,
              size: size,
              color: color,
            ),
            if (isConnected && deviceName != null) ...[
              SizedBox(width: size * 0.3),
              SizedBox(
                width: size * 4,
                child: Text(
                  deviceName!,
                  style: TextStyle(
                    fontSize: size * 0.75,
                    color: color,
                    fontWeight: FontWeight.w700,
                  ),
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }
}
