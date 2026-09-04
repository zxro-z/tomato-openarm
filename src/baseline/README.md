# Piper Tomato One Arm MoveIt

This package uses `piper_tomato_one_arm/urdf/one_piper_tomato.urdf` as the robot model and configures MoveIt for the `right_arm` planning group in the tomato-cell scene.

## Dry Run

Build and source the workspace, then start MoveIt without real robot execution:

```bash
ros2 launch piper_tomato_one_arm_moveit_config demo.launch.py
```

In RViz, test the `right_arm` planning group. Use `handover_staging` as a conservative named state before tuning a final pose for the real fixture.

## Real Robot

Start MoveIt with the real Piper CAN driver and trajectory bridge:

```bash
ros2 launch piper_tomato_one_arm_moveit_config real_robot.launch.py \
  can_port:=can0 \
  auto_enable:=false \
  command_speed_percent:=10.0
```

The launch starts:

- `piper_single_ctrl` for the real CAN driver.
- `single_arm_moveit_bridge`, which exposes MoveIt `FollowJointTrajectory` and publishes Piper joint commands.
- MoveIt `move_group` with trajectory execution enabled.
- RViz, unless `use_rviz:=false`.

Controller action exposed by the bridge:

- `/right_arm_controller/follow_joint_trajectory`

## Safety Notes

- Keep `auto_enable:=false` until CAN feedback, joint signs, and the RViz robot state match the physical robot.
- Start with `command_speed_percent:=5.0` to `10.0` for first real tests.
- Do not execute a plan unless RViz shows a valid, collision-free trajectory.
- The bridge separates driver feedback on `/joint_states_piper` from MoveIt state output on `/joint_states`.
