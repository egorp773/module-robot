# Demo And Grant Skill

Use this when writing demo text, grant text, pitch text, README-style summaries, or progress reports.

## Honest positioning

Describe the project as:

- an MVP modular robotic snow-cleaner / outdoor cleaner,
- currently manually controlled from a Flutter app over Wi-Fi/WebSocket,
- capable of attachment on/off control,
- moving toward GPS-assisted route cleaning.

Do not describe it as a finished autonomous snow-removal robot.

## Safe wording

Use:

- "MVP",
- "prototype",
- "manual control is working",
- "autonomous route workflow is under development",
- "GPS integration is planned/not yet field-tested",
- "RTK is a future precision upgrade".

Avoid:

- "fully autonomous",
- "RTK-accurate" unless tested,
- "GPS navigation works" before hardware tests,
- "field-ready" before safety tests.

## Demo scope

Good demo today:

- app connects to ESP32,
- manual movement,
- stop,
- attachment on/off,
- telemetry/status if available.

Future demo:

- GPS lat/lon,
- local x/y,
- route build/send,
- dry autonomous route without attachment,
- attachment route only after safety proof.

## Do not assume

- Do not overclaim autonomy.
- Do not overclaim GPS/RTK.
- Do not hide known safety gates.
- Do not call simulated mapping real perimeter recording.

