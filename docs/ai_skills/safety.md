# Safety Skill

Use this for any stop, failsafe, motor, attachment, NAV, GPS, or autonomy work.

## Non-negotiable rule

Uncertain state means stop motion and keep attachment off.

## Safety gates

Before autonomy:

- firmware builds,
- manual mode works,
- stop is tested,
- disconnect/timeout failsafe is tested,
- invalid command behavior is safe,
- auto UI has visible Stop.

Before autonomy with attachment:

- dry autonomy without attachment passes,
- stop during autonomy passes,
- pause/stop behavior passes,
- disconnect during autonomy passes,
- attachment on/off manual test passes.

Before RTK:

- ordinary GPS works,
- lat/lon is displayed,
- local x/y origin works,
- perimeter recording works.

## Firmware requirements

- Startup motors off.
- Attachment off on startup.
- Stop command always accepted.
- Command timeout stops motion.
- WebSocket disconnect stops motion.
- Heartbeat timeout stops motion if heartbeat is implemented.
- Invalid JSON/unknown commands do not move motors.
- NAV start rejected if no valid route.

## App requirements

- Stop visible in manual and auto contexts.
- Connection state visible.
- GPS fix/no-fix visible.
- NAV mode visible.
- Errors visible.
- Route sent state visible.
- Simulated/conditional map state not labeled as real GPS.

## Do not assume

- Do not assume code inspection proves safety.
- Do not assume route generation is safe.
- Do not assume GPS fix exists.
- Do not assume operator can stop the robot if UI hides Stop.
- Do not assume attachment can be enabled during early tests.

