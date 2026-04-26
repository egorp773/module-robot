# Manual Mode Skill

Use this for manual driving, stop, connection, and attachment control.

## Current truth

Manual mode is the MVP baseline.
The user reports manual control from Flutter over Wi-Fi/WebSocket works.
The user reports attachment on/off works.

Detailed test logs are still needed.

## Manual mode must include

- connect to robot,
- show connection state,
- drive forward/back/left/right or equivalent tank control,
- stop immediately,
- control attachment on/off,
- handle disconnect,
- avoid infinite stale movement.

## Firmware expectations

- Motor targets owned in one place.
- Stop sets both targets to zero.
- Command timestamp is updated correctly.
- Failsafe checks command age.
- Attachment output defaults off.

Known firmware issues:

- duplicate `g_targetLeft/g_targetRight`,
- incomplete `g_lastCmdMs`,
- incomplete smooth stop/failsafe functions.

## App expectations

- Stop visible.
- Disconnected state visible.
- Manual controls disabled or clearly unsafe when disconnected.
- Attachment state visible.

## Tests to log

- manual drive directions,
- stop while moving,
- attachment on/off,
- disconnect while moving,
- stale command timeout.

## Do not assume

- Do not assume manual mode is safe just because it moves.
- Do not assume attachment off state after reset until tested.
- Do not assume motor direction is correct until physical test.

