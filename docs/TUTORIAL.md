# Tutorial — from a raster path to a joint path with `compute_joint_path`

This tutorial runs the whole chain on the fake Sharework cell:

```
test mesh ──▶ raster_planner_node ──PoseArray──▶ compute_joint_path ──JointTrajectory──▶ Rviz
```

Time required: about 15 minutes.

---

## 0. Build

```bash
cd ~/projects/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to surface_path_planning surface_raster_planner
source install/setup.bash
```

`CNR_PARAM_ROOT_DIRECTORY` must be set (see the README). Remember to run
`source ~/projects/ros2_ws/install/setup.bash` in every new terminal.

---

## 1. Start-up

**Terminal 1**, the fake cell with move_group and Rviz:

```bash
ros2 launch sharework_cembre_bringup sharework_cembre_bringup.launch.py fake_ur:=true fake_gripper:=true \
  robotiq_use_socket_communication:=true enable_zed_camera:=false enable_dual_realsense:=false \
  launch_rviz:=true robotiq_activate_gripper_by_default:=1 enable_wrist_camera:=false
```

Wait for `You can start planning now!`.

**Terminal 2**, the ik_solver node and the action server:

```bash
ros2 launch surface_path_planning compute_joint_path.launch.py
```

You should see:

```
[ur_ik_solver.ik_solver_node]: IK Solver node: READY
[compute_joint_path]: compute_joint_path action server ready (group 'ur_robot', 6 joints)
```

**Terminal 3**, the raster planner:

```bash
ros2 launch surface_raster_planner raster_planner.launch.py
```

---

## 2. The tool frame

The raster planner builds poses with **z = outward surface normal** (pointing away from the
surface) and x = travel direction. `planning_frame` is the robot frame that is placed on these
poses, so its z axis must also point away from the surface.

On the Sharework cell, the z axis of `open_tip` points *out of the tool*, i.e. into the surface:
using it directly asks the robot to point the tool upwards, and no IK solution is found
(the server aborts with `no IK solution for any pose: check planning_frame orientation ...`).

For this tutorial, add a frame rotated by 180° around x (**Terminal 4**):

```bash
ros2 run tf2_ros static_transform_publisher --frame-id open_tip --child-frame-id open_tip_flipped --roll 3.14159265
```

In a real application this frame belongs to the robot description (URDF) of the tool.

---

## 3. A test surface within reach

The tool of the fake robot starts at about (-0.03, -0.11, 1.22) in `world`. Create a 40 × 25 cm
patch with a hole, 17 cm below it, and load it in the raster planner (**Terminal 5**):

```bash
ros2 run surface_raster_planner make_test_mesh.py --output /tmp/patch.ply --hole-radius 0.03 \
  --x -0.03 --y -0.11 --z 1.05
ros2 service call /raster_planner_node/load_mesh_from_file surface_raster_planner_msgs/srv/LoadMesh \
  "{filename: /tmp/patch.ply, frame_id: world}"
```

> Note: the patch is not added to the planning scene, so the robot can cross it in free motions.
> Collisions are checked only against the cell.

---

## 4. Compute the joint path

The example client calls `compute_trajectory` on the raster planner and sends the result to
`compute_joint_path`:

```bash
ros2 run surface_path_planning send_raster_to_joint_path.py --planning-frame open_tip_flipped \
  --row-step 0.04 --deviation 0 0 0.05
```

Expected output (numbers may change slightly):

```
raster: 556 poses, jumps [258]
...
path computed, 68 pose(s) skipped: 516 points, 488 poses reached, skipped [313, 314, ...]
```

and in the server terminal:

```
IK computed in 3.29 s: 490/556 poses and 494/556 leave/approach poses have at least one solution
compute_joint_path: 516 points, 68 poses skipped
```

What happened:

- `jump_indices = [258]`: the hole splits one row of the raster.
- Each pose has several IK solutions: the configuration file asks for 5 rotations around the tool z
  axis (`task_redundant` in `config/compute_joint_path_ik.yaml`), and each one has up to 8 UR solutions.
- 66 poses have no IK at all (out of reach in that orientation) and are skipped, together with a
  few poses dropped because the robot could not leave the surface from them.
- At the end of each row the x axis of the raster flips by 180°: the wrist cannot rotate by
  that much within `max_joint_step`, so the row change becomes an *implicit jump*: the robot
  moves 5 cm away from the surface, reorients and comes back.

---

## 5. Look at it in Rviz

The server publishes every computed path on `/compute_joint_path/display_path`
(`moveit_msgs/DisplayTrajectory`, latched). In Rviz:

1. *Add* → *moveit_ros_visualization* → **Trajectory**, and set *Trajectory Topic* to
   `/compute_joint_path/display_path`. Enable *Loop Animation* and, to see all points at once,
   *Show Trail*.
2. *Add* → **PoseArray** on `/raster_planner_node/poses`, to compare with the raster.

The animation uses a fake timing of `display_time_step` seconds (default 1 s) between consecutive
points: it is only a preview, the real timing is computed by the time-parametrization node. For
a faster preview set, for example, `ros2 param set /compute_joint_path display_time_step 0.05`
before sending the goal (or use *State Display Time* in the Rviz display).

What to check:

- the tool follows the surface on every segment;
- between segments the tool leaves the surface along its z axis, moves in free space and comes
  back from above;
- no point of the surface is reached with a large reconfiguration of the arm.

---

## 6. Using the action from your code

Goal fields (see `surface_path_planning_msgs/action/ComputeJointPath.action`):

```python
goal = ComputeJointPath.Goal()
goal.poses = raster_result.poses                 # geometry_msgs/PoseArray
goal.jump_indices = raster_result.jump_indices
goal.planning_frame = 'open_tip_flipped'
goal.deviation_distance.z = 0.05                 # in the tool frame
```

The result contains `trajectory` (positions only), `pose_indices` (which pose each point
reaches, -1 for auxiliary points) and `skipped_indices`. Use `pose_indices` to apply the
process speed only between two consecutive surface points.

---

## 7. Tuning and common problems

| Symptom | Cause / fix |
|---|---|
| `no IK solution for any pose` | `planning_frame` axes do not match the poses (see section 2), the poses are out of reach, or `poses.header.frame_id` is unknown to TF. |
| Many poses skipped in one area | Out of reach or in collision with the cell. Widen `task_redundant` (more orientations) or move the part. |
| Too many implicit jumps | Increase `max_joint_step`, or use `keep_tool_orientation: true` in the raster planner to avoid the 180° flip at every row (only for symmetric tools it is safe to also widen the yaw range). |
| `unable to leave the surface from pose N` | The leave configurations are not reachable: change `deviation_distance` or increase `max_backtracking_poses`. |
| Slow computation | Fewer task-redundant perturbations, smaller `multigoal_planning_time` or `max_multigoal_goals`. |
| `another goal is running` | Only one goal at a time is accepted: wait or cancel the previous one. |
