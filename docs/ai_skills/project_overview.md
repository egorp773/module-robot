# Project Overview Skill

Use this when a task needs broad project context.

## What this project is

MVP modular robotic snow-cleaner / outdoor cleaner.
It uses ESP32 firmware and a Flutter app.

The immediate value is manual control and attachment on/off.
Future value is GPS-based route cleaning with local-meter waypoints.

## Repository roles

- `firmware/`: active modular ESP32 firmware.
- `module_app/`: active Flutter app.
- `sound/`: legacy monolithic reference only.
- root markdown files: source-of-truth project memory.
- `docs/ai_skills/`: small task-specific Codex instructions.

## Current status categories

Confirmed working:

- manual control over Wi-Fi/WebSocket,
- attachment on/off.

Implemented but unverified:

- modular firmware shape,
- auto UI pieces,
- route planner,
- GPS projection,
- map storage.

Known broken:

- firmware build conflicts,
- snake route quality,
- wrong old `lineStep 44.0`,
- incomplete auto workflow.

Planned:

- GPS,
- local x/y,
- perimeter recording,
- waypoint upload,
- autonomy,
- RTK.

## Do not assume

- Do not assume docs are stale if they say a feature is unverified.
- Do not assume code presence means hardware success.
- Do not assume autonomous claims are acceptable without tests.

