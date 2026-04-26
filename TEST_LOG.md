# Test Log

This file records real tests only.

Rule: if a test was not actually run, do not write PASS.
Use NOT RUN, BLOCKED, FAIL, PARTIAL, or OBSERVED instead.

## How to write results

- Include date and time.
- Include firmware/app version or file state if known.
- Include hardware setup.
- Include what was done.
- Include expected and actual result.
- Include whether the result changes `IMPLEMENTATION_STATUS.md`.

## Test entry template

```text
Date/time:
Test:
Area: firmware / app / hardware / safety / route / GPS / RTK
Code state:
Hardware:
Conditions:
Steps:
Expected result:
Actual result:
Result: PASS / FAIL / PARTIAL / BLOCKED / NOT RUN / OBSERVED
Issues found:
Status update needed:
Next action:
```

## Current baseline notes

### 2026-04-26 - Documentation initialization

Area: documentation

Steps: created source-of-truth project documentation files.

Actual result: documentation structure exists and has been expanded with known project state.

Result: OBSERVED

Status update needed: no feature status should be upgraded from this entry.

### Manual control MVP - user-provided current state

Area: app/firmware

Actual result: user reports manual control from Flutter over Wi-Fi/WebSocket works, and attachment on/off works.

Result: OBSERVED

Issues found: exact protocol and safety behavior still need confirmation from code and real test entries.

Status update needed: `IMPLEMENTATION_STATUS.md` lists this as confirmed current MVP based on user-provided state, but detailed PASS tests still need to be recorded.

## Required future tests

- Firmware build test.
- Manual forward/back/left/right/stop test.
- Attachment on/off test.
- Stop while moving test.
- WebSocket disconnect failsafe test.
- Heartbeat or command timeout test.
- Invalid command safety test.
- GPS raw UART test.
- GPS fix/no-fix app display test.
- Local x/y projection test.
- GPS perimeter recording test.
- Route upload test.
- Autonomous dry run without attachment.
- Autonomous run with attachment after dry run passes.
- RTK field test after ordinary GPS works.

