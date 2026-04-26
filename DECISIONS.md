# Decisions

This file records project decisions that future Codex sessions must respect.
If a decision changes, add a new entry. Do not silently rewrite history.

## Decision format

```text
Date:
Decision:
Reason:
Consequences:
Status:
```

## 2026-04-26 - Use English documentation for Codex memory

Decision: project memory and task documentation should be written in English.

Reason: future Codex sessions read technical English more reliably and avoid encoding problems from mixed Cyrillic files.

Consequences: user notes can still be in Russian, but source-of-truth docs should remain English unless there is a strong reason to change.

Status: active.

## 2026-04-26 - `firmware/` is the active ESP32 architecture

Decision: `firmware/` is the new modular firmware and should be treated as the current target.

Reason: the project is moving away from a monolithic sketch toward modular firmware files for motors, WebSocket, navigation, safety, and telemetry.

Consequences: future work should fix and extend `firmware/`, not rebuild the active project inside `sound/`.

Status: active.

## 2026-04-26 - `sound/` is legacy reference only

Decision: `sound/` is not the current architecture.

Reason: it is an older monolithic implementation useful for reference only.

Consequences: use it only to understand old motor control, WebSocket, sound, relay, or attachment behavior. Do not copy its structure back into the modular firmware.

Status: active.

## 2026-04-26 - Manual mode is the MVP baseline

Decision: the MVP baseline is manual control over Wi-Fi/WebSocket plus attachment on/off.

Reason: this is the only workflow currently considered honestly working.

Consequences: manual mode and failsafe must be stabilized before route following, GPS autonomy, or RTK work.

Status: active.

## 2026-04-26 - Autonomy is gated by safety and real tests

Decision: autonomous movement must not be treated as ready until stop, connection loss, heartbeat/failsafe, and manual recovery are verified.

Reason: this is a moving outdoor robot with a working attachment; incorrect autonomy can be physically unsafe.

Consequences: autonomous tests start without the attachment. Attachment tests happen only after autonomous movement is controlled and stoppable.

Status: active.

## 2026-04-26 - Route planning must use local meters

Decision: future route planning should use a local metric coordinate system.

Reason: cleaning line spacing is a physical distance. The old `lineStep 44.0` value is not a valid meter-based spacing.

Consequences: target line spacing should be around `0.40` to `0.45` m unless hardware tests prove otherwise. `cleaning_route_planner.dart` must be corrected.

Status: active.

## 2026-04-26 - GPS before RTK

Decision: ordinary GPS must be connected, parsed, displayed, and used for local x/y before RTK work.

Reason: RTK cannot be validated before the base GPS workflow works.

Consequences: do not build RTK UX or navigation assumptions as if field accuracy is already available.

Status: active.

## Do not assume

- Do not assume a decision is obsolete because code appears to differ.
- Do not assume legacy `sound/` behavior overrides these decisions.
- Do not assume a feature is confirmed without `TEST_LOG.md`.

