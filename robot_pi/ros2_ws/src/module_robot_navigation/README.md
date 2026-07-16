# module_robot_navigation

This package deliberately starts only the Nav2 `FollowPath` action server and a
velocity smoother. It does not start a global planner: the gateway must convert
the validated Flutter GPS route into a `nav_msgs/Path` in `map` and send that
exact path to Nav2's `FollowPath` action.

Command ownership is:

```text
FollowPath/RPP -> /cmd_vel_nav_raw -> velocity_smoother -> /cmd_vel_nav
               -> module_robot_safety -> /cmd_vel_safe -> ESP32 bridge
```

`follow_path.launch.py` defaults `autostart` to `false`. The autonomous bringup
package may set it to true only after explicit, fail-closed preflight. The
rotation shim file is a review template and is not loaded. There are no obstacle
layers and collision checking is disabled until a measured footprint and real
observation source exist; this is not obstacle avoidance.
