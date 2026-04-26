import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:device_preview/device_preview.dart';
import 'package:go_router/go_router.dart';

import 'features/home/home_screen.dart';
import 'features/manual/manual_control_screen.dart';
import 'features/maps/maps_screen.dart';
import 'features/auto/auto_screen.dart';
import 'features/auto/auto_map_screen.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();

  runApp(
    ProviderScope(
      child: kReleaseMode
          ? const _RootApp()
          : DevicePreview(
              enabled: !kReleaseMode,
              builder: (_) => const _RootApp(),
            ),
    ),
  );
}

class _RootApp extends ConsumerWidget {
  const _RootApp();

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final router = GoRouter(
      initialLocation: '/',
      routes: [
        GoRoute(
          path: '/',
          builder: (_, __) => const HomeScreen(),
        ),
        GoRoute(
          path: '/manual',
          builder: (_, __) => const ManualControlScreen(),
        ),
        GoRoute(
          path: '/maps',
          builder: (_, __) => const MapsScreen(),
        ),
        GoRoute(
          path: '/auto',
          builder: (_, __) => const AutoScreen(),
        ),
        GoRoute(
          path: '/auto/map/:mapId',
          builder: (context, state) {
            final mapId = state.pathParameters['mapId']!;
            return AutoMapScreen(mapId: mapId);
          },
        ),
      ],
    );

    return MaterialApp.router(
      debugShowCheckedModeBanner: false,
      routerConfig: router,
      themeMode: ThemeMode.dark,
      theme: ThemeData.dark(useMaterial3: true),
    );
  }
}
