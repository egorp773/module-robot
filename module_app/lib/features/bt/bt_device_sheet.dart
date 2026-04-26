import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../core/bt_connection.dart';

class BtDevicePickerSheet extends ConsumerWidget {
  const BtDevicePickerSheet({super.key, this.autoScan = true});

  final bool autoScan;

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final bt = ref.watch(btConnectionProvider);
    final ctrl = ref.read(btConnectionProvider.notifier);

    // авто-скан при открытии
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (autoScan && !bt.isScanning && bt.devices.isEmpty && !bt.isConnected) {
        ctrl.startScan();
      }
    });

    return ClipRRect(
      borderRadius: const BorderRadius.vertical(top: Radius.circular(28)),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 18, sigmaY: 18),
        child: Container(
          padding: const EdgeInsets.fromLTRB(16, 14, 16, 18),
          decoration: BoxDecoration(
            color: Colors.black.withOpacity(0.55),
            borderRadius: const BorderRadius.vertical(top: Radius.circular(28)),
            border: Border.all(color: Colors.white.withOpacity(0.10)),
          ),
          child: SafeArea(
            top: false,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                _Header(
                  title: 'Bluetooth устройства',
                  subtitle: _subtitle(bt),
                  onClose: () => Navigator.pop(context),
                ),
                const SizedBox(height: 10),
                if (bt.error != null) ...[
                  _ErrorPill(text: bt.error!),
                  const SizedBox(height: 10),
                ],
                Row(
                  children: [
                    Expanded(
                      child: _ActionBtn(
                        icon: bt.isScanning ? Icons.stop : Icons.radar,
                        label: bt.isScanning ? 'Стоп' : 'Сканировать',
                        onTap: bt.isScanning ? ctrl.stopScan : ctrl.startScan,
                        loading: bt.isScanning,
                      ),
                    ),
                    const SizedBox(width: 10),
                    Expanded(
                      child: _ActionBtn(
                        icon: bt.isConnected ? Icons.link_off : Icons.link,
                        label: bt.isConnected ? 'Отключить' : 'Подключить',
                        onTap: bt.isConnected ? ctrl.disconnect : null,
                        loading: bt.isConnecting,
                        enabled: bt.isConnected,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 12),
                ConstrainedBox(
                  constraints: BoxConstraints(
                    maxHeight: MediaQuery.of(context).size.height * 0.5,
                  ),
                  child: bt.devices.isEmpty
                      ? _EmptyState(isScanning: bt.isScanning)
                      : ListView.separated(
                          physics: const BouncingScrollPhysics(),
                          padding: EdgeInsets.only(
                              bottom: MediaQuery.of(context).padding.bottom),
                          itemCount: bt.devices.length,
                          separatorBuilder: (_, __) =>
                              const SizedBox(height: 10),
                          itemBuilder: (_, i) {
                            final d = bt.devices[i];
                            final isCurrent =
                                bt.isConnected && bt.deviceId == d.id;

                            return _DeviceCard(
                              device: d,
                              isCurrent: isCurrent,
                              busy: bt.isBusy,
                              onConnect: () async {
                                await ctrl.connectTo(d);
                                if (ref
                                    .read(btConnectionProvider)
                                    .isConnected) {
                                  if (context.mounted) Navigator.pop(context);
                                }
                              },
                            );
                          },
                        ),
                ),
                const SizedBox(height: 8),
              ],
            ),
          ),
        ),
      ),
    );
  }

  String _subtitle(BtConnectionState bt) {
    final a = bt.isBluetoothOn ? 'BT ВКЛ' : 'BT ВЫКЛ';
    if (!bt.isSupported) return 'BLE не поддерживается';
    if (bt.isConnected) return 'Подключено: ${bt.deviceName} • $a';
    if (bt.isScanning) return 'Сканирование… • $a';
    if (bt.isConnecting) return 'Подключение… • $a';
    return 'Готово • $a';
  }
}

class _Header extends StatelessWidget {
  const _Header({
    required this.title,
    required this.subtitle,
    required this.onClose,
  });

  final String title;
  final String subtitle;
  final VoidCallback onClose;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Container(
          width: 42,
          height: 42,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(14),
            color: Colors.white.withOpacity(0.08),
            border: Border.all(color: Colors.white.withOpacity(0.10)),
          ),
          child: const Icon(Icons.bluetooth, color: Colors.white),
        ),
        const SizedBox(width: 12),
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(title,
                  style: const TextStyle(
                      fontWeight: FontWeight.w700, fontSize: 16)),
              const SizedBox(height: 2),
              Text(subtitle,
                  style: TextStyle(
                      color: Colors.white.withOpacity(0.65), fontSize: 12)),
            ],
          ),
        ),
        IconButton(
          onPressed: onClose,
          icon: const Icon(Icons.close),
        ),
      ],
    );
  }
}

class _ActionBtn extends StatelessWidget {
  const _ActionBtn({
    required this.icon,
    required this.label,
    required this.onTap,
    this.loading = false,
    this.enabled = true,
  });

  final IconData icon;
  final String label;
  final VoidCallback? onTap;
  final bool loading;
  final bool enabled;

  @override
  Widget build(BuildContext context) {
    final canTap = enabled && onTap != null && !loading;

    return GestureDetector(
      onTap: canTap ? onTap : null,
      child: Container(
        height: 48,
        alignment: Alignment.center,
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(16),
          color: Colors.white.withOpacity(canTap ? 0.10 : 0.06),
          border: Border.all(color: Colors.white.withOpacity(0.10)),
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            if (loading)
              const SizedBox(
                width: 18,
                height: 18,
                child: CircularProgressIndicator(strokeWidth: 2),
              )
            else
              Icon(icon, size: 18, color: Colors.white),
            const SizedBox(width: 10),
            Text(label,
                style: TextStyle(
                  fontWeight: FontWeight.w700,
                  color: Colors.white.withOpacity(canTap ? 1 : 0.6),
                )),
          ],
        ),
      ),
    );
  }
}

class _DeviceCard extends StatelessWidget {
  const _DeviceCard({
    required this.device,
    required this.onConnect,
    required this.isCurrent,
    required this.busy,
  });

  final BtDeviceInfo device;
  final VoidCallback onConnect;
  final bool isCurrent;
  final bool busy;

  @override
  Widget build(BuildContext context) {
    final title = device.name;
    final subtitle = '${device.id} • RSSI ${device.rssi}';

    return Container(
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(18),
        color: Colors.white.withOpacity(0.08),
        border: Border.all(color: Colors.white.withOpacity(0.10)),
      ),
      child: Row(
        children: [
          _SignalBadge(rssi: device.rssi, connectable: device.connectable),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(title,
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                    style: const TextStyle(fontWeight: FontWeight.w700)),
                const SizedBox(height: 4),
                Text(subtitle,
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                    style: TextStyle(
                        color: Colors.white.withOpacity(0.65), fontSize: 12)),
              ],
            ),
          ),
          const SizedBox(width: 10),
          GestureDetector(
            onTap: (!busy && !isCurrent) ? onConnect : null,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(14),
                color: Colors.white
                    .withOpacity((!busy && !isCurrent) ? 0.12 : 0.06),
                border: Border.all(color: Colors.white.withOpacity(0.10)),
              ),
              child: Text(
                isCurrent ? 'Подключено' : 'Подключить',
                style: TextStyle(
                  fontWeight: FontWeight.w800,
                  color: Colors.white.withOpacity(isCurrent ? 0.75 : 1),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _SignalBadge extends StatelessWidget {
  const _SignalBadge({required this.rssi, required this.connectable});

  final int rssi;
  final bool connectable;

  @override
  Widget build(BuildContext context) {
    IconData icon;
    if (rssi >= -55) {
      icon = Icons.network_wifi_3_bar;
    } else if (rssi >= -70)
      icon = Icons.network_wifi_2_bar;
    else
      icon = Icons.network_wifi_1_bar;

    return Container(
      width: 42,
      height: 42,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(14),
        color: Colors.white.withOpacity(0.10),
        border: Border.all(color: Colors.white.withOpacity(0.10)),
      ),
      child: Stack(
        alignment: Alignment.center,
        children: [
          Icon(icon, color: Colors.white),
          Positioned(
            bottom: 6,
            right: 6,
            child: Container(
              width: 8,
              height: 8,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                color: connectable ? Colors.greenAccent : Colors.orangeAccent,
              ),
            ),
          )
        ],
      ),
    );
  }
}

class _EmptyState extends StatelessWidget {
  const _EmptyState({required this.isScanning});
  final bool isScanning;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 24),
        child: Text(
          isScanning ? 'Ищу устройства…' : 'Нажми «Сканировать»',
          style: TextStyle(color: Colors.white.withOpacity(0.7)),
        ),
      ),
    );
  }
}

class _ErrorPill extends StatelessWidget {
  const _ErrorPill({required this.text});
  final String text;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(14),
        color: Colors.redAccent.withOpacity(0.18),
        border: Border.all(color: Colors.redAccent.withOpacity(0.35)),
      ),
      child: Text(text, style: const TextStyle(fontWeight: FontWeight.w600)),
    );
  }
}
