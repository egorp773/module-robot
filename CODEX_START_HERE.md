# Codex Start Here

Read this file before any project work.

## Immediate truth

This is an MVP modular robot snow-cleaner / outdoor cleaner project.
The current useful working behavior is manual control from the Flutter app over Wi-Fi/WebSocket plus attachment on/off.

The robot is not a confirmed autonomous system yet.
GPS is not physically tested.
RTK is not field-tested.
Snake route generation is not reliable.

## Read order

1. `MEMORY.md`
2. `IMPLEMENTATION_STATUS.md`
3. `DECISIONS.md`
4. `SAFETY.md`
5. `ROADMAP.md`
6. `CODEX_TASKS.md`
7. Relevant file in `docs/ai_skills/`

## Active code areas

- `firmware/`: active modular ESP32 firmware.
- `module_app/`: active Flutter app.
- `sound/`: legacy monolithic firmware reference only.

Do not edit code in `firmware/`, `module_app/`, or `sound/` unless the user explicitly asks for a code task.

## Current development order

1. Fix firmware build.
2. Stabilize manual mode.
3. Verify failsafe.
4. Fix auto UI.
5. Fix route planner in local meters.
6. Connect GPS.
7. Display lat/lon.
8. Build local x/y from origin.
9. Record GPS perimeter.
10. Send waypoints.
11. Test autonomy without attachment.
12. Test autonomy with attachment.
13. RTK after ordinary GPS.

## Do not assume

- No git repository is present.
- GPS does not work until tested.
- RTK does not work until tested.
- Auto route following is not ready.
- `sound/` is not current architecture.
- No PASS result without a real `TEST_LOG.md` entry.

