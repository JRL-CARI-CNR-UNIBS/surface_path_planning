# Surface Path Planning

Trajectory planning for robotic surface operations.

## Overview

Many industrial applications, such as polishing, additive manufacturing (3D printing), surface inspection, painting, and cleaning, require a robot to visit a large number of poses distributed over a surface.

In these tasks, the end-effector often has **operational redundancy**, meaning that multiple robot configurations can achieve the same surface pose. Exploiting this redundancy allows the trajectory planner to optimize the robot motion according to application-specific criteria.

The goal of **Surface Path Planning** is to generate trajectories that minimize execution cost, such as:

- execution time
- energy consumption
- joint motion
- trajectory smoothness

while satisfying the task constraints imposed by the surface process.

---

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy

Install the required system dependencies:

```bash
sudo apt update

sudo apt -y install libeigen3-dev
sudo apt -y install \
    libboost-all-dev \
    libyaml-cpp-dev \
    libpoco-dev \
    liblog4cxx-dev \
    libgtest-dev

sudo apt install -y libpoco-dev
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-moveit
```

---

## Installation

Clone this repository inside the `src` folder of your ROS 2 workspace together with all required dependencies.

### Required repositories

```bash
cd ~/ros2_ws/src

git clone -b modern_cmake https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader.git

git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git

git clone -b ros2 https://github.com/JRL-CARI-CNR-UNIBS/graph_display.git

git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/ik_solver.git

git clone -b master https://github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs.git

git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/moveit_collision_checker.git

git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/ur_ik_solver.git

# surface_path_planning_msgs (action definitions) and, for the full pipeline, surface_raster_planner
# with surface_raster_planner_msgs must also be in the workspace.

git clone -b jazzy https://github.com/JRL-CARI-CNR-UNIBS/sharework_cell.git
```

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## Applications

This package is intended for robotic applications involving dense surface trajectories, including:

- Polishing
- Grinding
- Painting
- Sanding
- Surface inspection
- Additive manufacturing (3D printing)
- Cleaning

---

## compute_joint_path and compute_joint_trajectory action servers

`compute_joint_path_node` offers two actions, called one after the other:

1. `compute_joint_path` converts the tool `PoseArray` produced by
   [`surface_raster_planner`](../surface_raster_planner) into a joint-space path, without time
   parametrization, also split into surface / free-motion paths;
2. `compute_joint_trajectory` retimes that set of paths (one time parametrization request per path)
   and appends them in one trajectory ready to be executed.

```
surface_raster_planner ──PoseArray + jump_indices──▶ compute_joint_path ──paths──▶ compute_joint_trajectory ──RobotTrajectory──▶ execution
                                                      │                              │
                                       parallel_multigoal_planner |        apply_time_parametrization (TOTG),
                                       prm_multigoal_planner (free motions)   one request per path
```

A step-by-step tutorial is in [`docs/TUTORIAL.md`](docs/TUTORIAL.md).

### Interface

Action `compute_joint_path` (`surface_path_planning_msgs/action/ComputeJointPath`):

| Goal field | Description |
|---|---|
| `poses` | Tool poses, in `poses.header.frame_id` (must be known to TF). |
| `jump_indices` | Indices where a new segment starts (same convention as the raster planner). |
| `planning_frame` | Robot frame that must reach the poses (the TCP, e.g. `open_tip`). Its axes must match the pose axes: the raster planner uses z = outward surface normal. |
| `deviation_distance` | Leave/approach offset in the **tool frame** of each pose. With the raster planner, `[0, 0, 0.05]` means 5 cm away from the surface. |

| Result field | Description |
|---|---|
| `success`, `message` | Outcome. |
| `trajectory` | `trajectory_msgs/JointTrajectory` with positions only (`time_from_start` = 0). |
| `pose_indices` | For each trajectory point, the reached pose index, or -1 for auxiliary points (free motions, leave/approach points). |
| `skipped_indices` | Poses skipped because no feasible configuration was found. |
| `paths`, `path_types` | The same path, preceded by the robot configuration and split at the first and at the last pose of every continuous run of surface poses (i.e. at every jump). `SURFACE` paths and `FREE_MOTION` paths (leave + transfer + approach) alternate; each one starts at the end of the previous one. Input of `compute_joint_trajectory`. |

The path is also published for Rviz on `/compute_joint_path/display_path`
(`moveit_msgs/DisplayTrajectory`, latched), starting from the robot configuration and with a fake
time step of `display_time_step` seconds between points: add a MoveIt **Trajectory** display on
that topic to animate it.

Action `compute_joint_trajectory` (`surface_path_planning_msgs/action/ComputeJointTrajectory`):

| Goal field | Description |
|---|---|
| `paths`, `path_types` | Consecutive paths (usually `paths`, `path_types` of the `compute_joint_path` result). Only positions are used; joints are matched by name. |
| `surface_velocity_scaling`, `surface_acceleration_scaling` | Scaling factors of the `SURFACE` paths, in (0, 1] (other values mean 1). |
| `free_motion_velocity_scaling`, `free_motion_acceleration_scaling` | Scaling factors of the `FREE_MOTION` paths. |

| Result field | Description |
|---|---|
| `success`, `message` | Outcome. |
| `trajectory` | `moveit_msgs/RobotTrajectory` with positions, velocities, accelerations and `time_from_start`, ready for `/execute_trajectory`. Its points are resampled by TOTG. |

Each path is sent as a separate request to `time_parametrization_action` (all requests at once) and
the results are appended, so the robot stops at the end of every path. The goal is aborted if a
path does not start at the end of the previous one or if a request fails. The trajectory is
published on the same Rviz topic. Feedback reports the path being retimed.

Feedback reports the pose being processed and a status string. Goals can be canceled. Each action
accepts one goal at a time; the two actions can run at the same time.

### Algorithm

1. The robot configuration is read from `/joint_states`, the planning scene from move_group
   (`/get_planning_scene`), and `planning_frame` is set as IK tool with the ik_solver `change_tool` service.
2. One call to `get_task_redundant_ik_array` solves every pose and every pose moved by
   `deviation_distance`. All the solutions are kept (task redundancy, see the `task_redundant`
   parameters of ik_solver).
3. The path is built greedily, with point-to-point joint motions only:
   - **Approach** (start and after each jump): the closest `max_multigoal_goals` leave/approach
     configurations of the pose (by joint distance) are sent together, as alternative goals, to the
     `parallel_multigoal_planner` action (mirrt*), which plans a free motion to one of them; the robot
     then moves down to the closest compatible IK of the pose.
   - **Along a segment**: the robot moves to the closest IK of the next pose whose joint motion is
     below `max_joint_step` and collision free.
   - **Jump**: at each `jump_indices` entry, or when the next pose cannot be reached along the
     segment (*implicit jump*), the robot moves from the last feasible pose to its
     leave configuration (pose + `deviation_distance`) and approaches the next pose.
     If no leave configuration is directly reachable, a free motion to one of them is planned with
     `parallel_multigoal_planner`; if that fails too, the last poses of the segment are dropped
     (and reported as skipped) until the robot can leave.
   - A pose that cannot be approached is skipped. The path ends with a leave motion.

All the messages of this package use the ROS 2 logger; graph_core and ik_solver write to the child
loggers `<node>.graph_core` and `<node>.ik_solver`. The ik_solver parameters are node parameters
under `ur_ik_solver.` (`config/ik_params.yaml`).

Collision checking uses the move_group planning scene (`moveit_collision_checker`); the free
motions are checked by `collision_checker_server` (`/collision_check`), which reads the same scene. The surface
itself is **not** a collision object unless it is added to the planning scene.

### Launch

```bash
ros2 launch surface_path_planning compute_joint_path.launch.py
# free motions with the persistent Lazy PRM* planner:
ros2 launch surface_path_planning compute_joint_path.launch.py multigoal_planner:=prm_multigoal_planner
```

It starts the `ur_ik_solver` node (`start_ik_solver:=false` if it is already running),
`collision_checker_server` and the planner chosen with `multigoal_planner` (`start_multigoal_planner:=false` if they
are already running), the `compute_joint_path` node (both actions) and the `apply_time_parametrization` server
(package `time_parametrization`, MoveIt TOTG, configured in `config/apply_time_parametrization.yaml`).
Robot (or fake hardware) and move_group must already be running.

`scripts/send_raster_to_joint_path.py` runs the full chain: raster path → `compute_joint_path` →
`compute_joint_trajectory` → execution with MoveIt (`/execute_trajectory`). ROS parameters of the script:
`scaling_factor` (velocity and acceleration scaling of the surface paths, default 0.1),
`free_motion_scaling_factor` (default: `scaling_factor`), `execute` (default `true`; `false` stops
after the time parametrization).

| Launch argument | Default | Description |
|---|---|---|
| `params` | `config/compute_joint_path.yaml` | ROS parameters of the server |
| `start_time_parametrization` | `true` | start the `apply_time_parametrization` server (`false` if it is already running, e.g. started by `picking_example`) |
| `time_parametrization_params` | `config/apply_time_parametrization.yaml` | ROS parameters of the time parametrization server |
| `joint_limits` | `''` | MoveIt `joint_limits.yaml` of the robot, used by the time parametrization (empty: URDF limits, accelerations = 5 × max velocity) |
| `multigoal_planner` | `parallel_multigoal_planner` | free-motion planner started and used by the server: `parallel_multigoal_planner` or `prm_multigoal_planner` |
| `prm_params` | `prm_multigoal_planner/config/prm_multigoal_planner.yaml` | ROS 2 parameters of `prm_multigoal_planner` |
| `mirrt_star_params` | `config/mirrt_star.yaml` | ROS 2 parameters of the mirrt* solver of `parallel_multigoal_planner` (prefix `mirrt_star.`) |
| `ik_config` | `config/compute_joint_path_ik.yaml` | ik_solver configuration (including `task_redundant`) |
| `start_ik_solver` | `true` | start the ik_solver node |
| `start_multigoal_planner` | `true` | start `collision_checker_server` and the selected multigoal planner |
| `ik_solver_ns` | `ur_ik_solver` | ik_solver namespace used by `parallel_multigoal_planner` (no leading `/`) |
| `collision_checker_params` | `collision_checker_server/config/collision_checker_params.yaml` | `group_name`, `padding`, `scaling` of the free-motion collision checker |

### Parameters (`config/compute_joint_path.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `group_name` | `manipulator` | MoveIt planning group (`ur_robot` in the config file) |
| `ik_solver_namespace` | `/ur_ik_solver` | namespace of the ik_solver node |
| `joint_states_topic` | `/joint_states` | |
| `planning_scene_service` | `/get_planning_scene` | |
| `service_timeout` | 10.0 | [s] change_tool and planning scene services |
| `ik_timeout` | 120.0 | [s] IK service |
| `ik_parallel` | true | solve the IK poses in parallel (`parallelize` of the request; overrides `parallel_ik_mode` of the ik_solver node) |
| `ik_threads` | 0 | IK worker threads when `ik_parallel` is true; 0 uses all the threads of the ik_solver node (CPU cores of its build machine minus 2) |
| `joint_state_timeout` | 5.0 | [s] |
| `max_joint_step` | 0.35 | [rad] max joint motion (infinity norm) between consecutive surface points |
| `max_deviation_joint_step` | 0.8 | [rad] max joint motion between a surface point and its leave/approach point |
| `multigoal_planner` | `parallel_multigoal_planner` | free-motion planner: `parallel_multigoal_planner` or `prm_multigoal_planner` (read at every goal) |
| `parallel_multigoal_action` | `/parallel_multigoal_plan` | action of `parallel_multigoal_planner` |
| `prm_multigoal_action` | `/prm_multigoal_plan` | action of `prm_multigoal_planner` |
| `multigoal_planning_time` | 5.0 | [s] `allowed_planning_time` of one free-motion request |
| `collision_check_service` | `/collision_check` | collision checker of `parallel_multigoal_planner`; when a request fails it is called (if running) with `detailed = true` and the colliding links are logged |
| `max_multigoal_goals` | 20 | candidate configurations sent in one free-motion request |
| `max_backtracking_poses` | 20 | poses that can be dropped when the surface cannot be left |
| `time_parametrization_action` | `/apply_time_parametrization` | `time_parametrization_msgs/ApplyTimeParametrization` action used by `compute_joint_trajectory` |
| `time_parametrization_timeout` | 30.0 | [s] for all the paths of one `compute_joint_trajectory` goal |
| `display_topic` | `~/display_path` | Rviz topic (`moveit_msgs/DisplayTrajectory`) |
| `display_time_step` | 1.0 | [s] time between points in the displayed path (visualization only) |
| `checker_resolution` | 0.02 | [rad] discretization of the straight joint motions |
| `checker_threads` | 4 | collision checking threads |

---

## Simple tests

Terminal 1:
```bash
ros2 launch sharework_cembre_bringup sharework_cembre_bringup.launch.py fake_ur:=true fake_gripper:=true robotiq_use_socket_communication:=true enable_zed_camera:=false enable_dual_realsense:=false launch_rviz:=true robotiq_activate_gripper_by_default:=1 enable_wrist_camera:=true
```

Terminal 2 (`simple_test_1`, RRT between two joint configurations):
```bash
ros2 launch surface_path_planning test_solver.launch.py
```

or (`simple_test_2`, RRT towards the IK solutions of a Cartesian goal):
```bash
ros2 launch surface_path_planning test_solver2.launch.py
```

---

## Authors

- Manuel Beschi (manuel.beschi@unibs.it)
- Elisa Zappa

## License

Apache License 2.0, see the [`LICENSE`](LICENSE) file.

## API documentation

The API reference of the classes, functions and scripts is in [docs/API.md](docs/API.md) (generated from the
Doxygen comments).

The code is documented with Doxygen. From the package folder run `doxygen Doxyfile` and open
`doxygen/html/index.html`.
