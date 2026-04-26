# Memory Maintenance Skill

Use this when updating project documentation after code changes, tests, or new decisions.

## Main rule

Documentation must stay more honest than the code.
If code exists but was not tested, document it as unverified.

## Update map

Update `../../MEMORY.md` when:

- project direction changes,
- active architecture changes,
- major status changes.

Update `../../IMPLEMENTATION_STATUS.md` when:

- a feature becomes confirmed,
- a feature is found broken,
- an unverified area changes.

Update `../../DECISIONS.md` when:

- architecture changes,
- route/GPS/safety policy changes,
- legacy/current boundaries change.

Update `../../PROTOCOL.md` when:

- WebSocket command names change,
- telemetry shape changes,
- route upload protocol changes.

Update `../../SAFETY.md` when:

- stop/failsafe behavior changes,
- timeout values are known,
- autonomy gates change.

Update `../../HARDWARE.md` when:

- pins are known,
- sensors are connected,
- power details are known,
- relay/driver behavior is confirmed.

Update `../../TEST_LOG.md` when:

- a real test or build is run.

## Status language

Use exact categories:

- confirmed working,
- implemented but unverified,
- known broken,
- planned.

## Do not assume

- Do not write PASS without real test.
- Do not delete known issues just because they are inconvenient.
- Do not move planned work into confirmed status.
- Do not let `sound/` become current architecture by accident.

