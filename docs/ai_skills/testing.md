# Testing Skill

Use this whenever a real build, hardware run, app run, GPS check, or safety test is performed.

## Core rule

No real run means no PASS.

Use:

- PASS only when the test actually ran and matched expected behavior.
- FAIL when it ran and did not match.
- PARTIAL when some expected behavior worked.
- BLOCKED when setup prevented the test.
- NOT RUN when it was not attempted.
- OBSERVED for user-provided state or documentation-only observations.

## Where to record

Record real tests in `../../TEST_LOG.md`.
If a test changes feature status, update `../../IMPLEMENTATION_STATUS.md`.
If a test confirms or changes safety behavior, update `../../SAFETY.md`.

## Minimum details

Each test entry should include:

- date/time,
- area,
- code state,
- hardware,
- conditions,
- steps,
- expected result,
- actual result,
- result label,
- issues found,
- next action.

## High-priority tests

- Firmware build.
- Manual forward/back/left/right.
- Manual stop while moving.
- Attachment on/off.
- App disconnect while moving.
- Heartbeat or command timeout.
- Invalid command behavior.
- Startup safe state.
- GPS raw UART.
- GPS fix/no-fix display.
- Local x/y projection.
- GPS perimeter recording.
- Waypoint upload.
- Dry autonomous run without attachment.
- Autonomous run with attachment only after dry run passes.
- RTK field test only after ordinary GPS.

## Do not assume

- Do not convert "user says it worked before" into a PASS unless the entry says it is an observation.
- Do not mark GPS PASS without hardware.
- Do not mark autonomy PASS from a simulated map.
- Do not mark failsafe PASS from code inspection alone.

