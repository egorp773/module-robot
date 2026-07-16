# module_robot_bringup

`manual_bringup.launch.py` is the normal entry point. It starts Safety,
diagnostics, and (by default) the ESP32 bridge. It never calls ARM and does not
start localization or Nav2. Description remains off while transforms are
`TODO_MEASURE`; opt in with `start_description:=true` only when the supplied TF
arguments are intentional. `start_gateway` is also false by default.
The systemd split should pass `start_bridge:=false` because the bridge has its
own earlier service.

`autonomous_bringup.launch.py` is intentionally difficult to start: the caller
must explicitly enable autonomy, attest that yaw and localization were verified,
attest that manual control and watchdog STOP were physically verified, and
provide all finite measured dimensions. Every `TODO*` value aborts launch.
Even after that preflight, bringup sends neither ARM nor a FollowPath goal; the
live Safety node remains authoritative.
