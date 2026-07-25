# MyArm M750 MoveIt config

This package targets ROS 2 Foxy and intentionally uses hand-written launch and
configuration files instead of the newer `MoveItConfigsBuilder` API.

- `plan_only.launch.py` starts `move_group` with trajectory execution disabled.
- `mock_execution.launch.py` starts the mock SDK driver and routes execution to
  `/myarm_m750/follow_joint_trajectory`. There is no second hardware path.
- `model_variant:=lightweight` is the default because the supplied detailed
  visual meshes exceed the release size/triangle budget.

The collision boxes and velocity/acceleration values are provisional. Do not
use them as certified hardware safety limits. Planning against the physical
robot remains blocked until collision geometry and limits are reviewed.

## Foxy runtime evidence and known cancel limitation

Local/headless evidence covers successful plan-only, collision rejection and
mock execution routed through `/myarm_m750/follow_joint_trajectory`. Direct
cancel of that driver action is also covered by the ROS runtime gate.

MoveIt-level cancellation is **not** closed on the supported Foxy stack. In the
current `move_group`, canceling `/execute_trajectory` or `/move_action` during
the mock controller run is only serviced after the trajectory has already
terminated (`CancelGoal` reports `GOAL_TERMINATED`), so the driver never
receives the cancel. Phase 6 therefore remains partial; do not advertise
MoveIt cancel or hardware execution until this behavior is fixed or a supported
executor/controller architecture is validated.
