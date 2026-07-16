# module_robot_tools

`readiness_monitor` publishes read-only candidate diagnostics. Its booleans are
for humans and dashboards only; Safety does not consume them as authorization.

`publish_zero` is the only command helper and is structurally incapable of
creating non-zero movement. It publishes a zero `TwistStamped` on
`/cmd_vel_manual` for one second by default. It does not ARM or DISARM.
