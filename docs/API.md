# surface_path_planning API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions and the scripts. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`include/surface_path_planning/compute_joint_path_server.hpp`](#includesurface_path_planningcompute_joint_path_serverhpp)
  - [`surface_path_planning::ComputeJointPathServer`](#class-surface_path_planningcomputejointpathserver)
- [`include/surface_path_planning/joint_path_builder.hpp`](#includesurface_path_planningjoint_path_builderhpp)
  - [`surface_path_planning::JointPathBuilderOptions`](#struct-surface_path_planningjointpathbuilderoptions)
  - [`surface_path_planning::JointPathBuilderResult`](#struct-surface_path_planningjointpathbuilderresult)
  - [`surface_path_planning::JointPathBuilder`](#class-surface_path_planningjointpathbuilder)
- [`include/surface_path_planning/robot_description_node.hpp`](#includesurface_path_planningrobot_description_nodehpp)
  - [`surface_path_planning::RobotDescriptionNode`](#class-surface_path_planningrobotdescriptionnode)
- [`include/surface_path_planning/surface_path_planning.hpp`](#includesurface_path_planningsurface_path_planninghpp)
  - [`surface_path_planning::SurfacePlannerNode`](#class-surface_path_planningsurfaceplannernode)
- [`include/surface_path_planning/utils.hpp`](#includesurface_path_planningutilshpp)
  - [`surface_path_planning::utils::PlannerComponentsFactory`](#class-surface_path_planningutilsplannercomponentsfactory)
- [Scripts](#scripts)

## include/surface_path_planning/compute_joint_path_server.hpp

### class `surface_path_planning::ComputeJointPathServer`

Action server "compute_joint_path" (surface_path_planning_msgs/action/ComputeJointPath).

For each goal:

1. reads the robot configuration from the joint states topic;
2. downloads the current planning scene from move_group (used for collision checking);
3. sets goal.planning_frame as IK tool frame (ik_solver change_tool service);
4. solves with one call of get_task_redundant_ik_array the IK of every pose and of every pose moved by goal.deviation_distance (in the tool frame);
5. builds the joint path with `JointPathBuilder`; free motions are planned by the parallel_multigoal_planner action;
6. publishes the path for Rviz (moveit_msgs/DisplayTrajectory, fixed time step between points).

Only one goal at a time is accepted.

#### Public types

##### `Action`

```cpp
using surface_path_planning::ComputeJointPathServer::Action = surface_path_planning_msgs::action::ComputeJointPath
```

##### `GoalHandle`

```cpp
using surface_path_planning::ComputeJointPathServer::GoalHandle = rclcpp_action::ServerGoalHandle<Action>
```

#### Public functions

##### `ComputeJointPathServer`

```cpp
ComputeJointPathServer(const rclcpp::Node::SharedPtr &node)
```

Declare the parameters, create clients, subscriptions and the action server.

**Parameters:**

- `node`: Node; it must be spun by a multi-threaded executor.

##### `init`

```cpp
bool init()
```

Loads the robot model. Call it before adding the node to an executor (the model loader spins the node).

## include/surface_path_planning/joint_path_builder.hpp

### struct `surface_path_planning::JointPathBuilderOptions`

Options of `JointPathBuilder`.

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `max_joint_step` | `double` | [rad] Max joint displacement (infinity norm) between two consecutive points of the same segment. |
| `max_deviation_joint_step` | `double` | [rad] Max joint displacement (infinity norm) between a surface point and its leave/approach point. |
| `max_backtracking_poses` | `unsigned int` | Max number of surface poses dropped (going backwards along the segment) when the surface cannot be left. |
| `max_multigoal_goals` | `unsigned int` | Max number of candidate goal configurations sent in one multigoal planning request. |

### struct `surface_path_planning::JointPathBuilderResult`

Output of `JointPathBuilder`.

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `success` | `bool` | True if a path was built (some poses may be skipped). |
| `message` | `std::string` | Description of the result or of the error. |
| `waypoints` | `std::vector< Eigen::VectorXd >` | Joint configurations, in order, starting from the robot current configuration (excluded). |
| `pose_indices` | `std::vector< int >` | Same length as waypoints: index of the reached pose, -1 for auxiliary points. |
| `skipped_indices` | `std::vector< unsigned int >` | Poses skipped because no feasible configuration was found. |

### class `surface_path_planning::JointPathBuilder`

Builds a joint-space path through a sequence of surface poses.

Inputs are, for every pose i, the IK solutions of the pose itself (process[i]) and of the pose moved by the deviation offset (deviation[i]). The path is built greedily:

- From free space (robot start or after leaving the surface), the robot approaches pose i: the closest approach configurations are all sent to the multigoal planner in one request, and it chooses which one to reach. Then the robot moves point-to-point from the approach configuration to the closest compatible IK of pose i.
- Inside a segment, the robot moves to the closest IK of the next pose that is within max_joint_step and whose straight joint-space connection is collision free.
- At an explicit jump, or when the next pose cannot be reached inside the segment (implicit jump), the robot leaves the surface moving to the deviation configuration of the last feasible pose, then it approaches the next pose as above. If the deviation configuration of that pose is not reachable, the last poses of the segment are dropped (and reported as skipped) until the robot can leave the surface.
- A pose that cannot be approached is skipped. The path ends leaving the surface.

Only point-to-point joint motions are produced: leave and approach motions are not Cartesian lines.

#### Public types

##### `ProgressCallback`

```cpp
using surface_path_planning::JointPathBuilder::ProgressCallback = std::function<bool(size_t, const std::string&)>
```

Called before each pose: (pose index, status). Returning false cancels the computation.

##### `MultigoalPlanner`

```cpp
using surface_path_planning::JointPathBuilder::MultigoalPlanner = std::function<bool(const Eigen::VectorXd& q_from, const std::vector<Eigen::VectorXd>& goals, std::vector<Eigen::VectorXd>& path, size_t& goal_index)>
```

Plans a free motion from q_from to any one of goals.

On success, path excludes q_from and ends exactly at goals[goal_index].

#### Public functions

##### `JointPathBuilder`

```cpp
JointPathBuilder(const graph::core::CollisionCheckerPtr &checker, const Eigen::VectorXd &lower_bound, const Eigen::VectorXd &upper_bound, const rclcpp::Logger &logger, const MultigoalPlanner &planner, const JointPathBuilderOptions &options)
```

**Parameters:**

- `logger`: ROS 2 logger used for all the messages of this class
- `planner`: planner of the free motions (approach and leave)

##### `build`

```cpp
JointPathBuilderResult build(const Eigen::VectorXd &q_start, const std::vector< Candidates > &process, const std::vector< Candidates > &deviation, const std::vector< bool > &segment_start, const ProgressCallback &progress=nullptr)
```

**Parameters:**

- `q_start`: robot current configuration
- `process`: IK solutions of each pose
- `deviation`: IK solutions of each pose moved by the deviation offset
- `segment_start`: segment_start[i] is true if poses[i] starts a new segment (explicit jump)
- `progress`: optional progress/cancel callback

### Free functions and types

#### `Candidates`

```cpp
using surface_path_planning::Candidates = std::vector<Eigen::VectorXd>
```

All the IK solutions of one pose (each one ordered as the planning group joints).

## include/surface_path_planning/robot_description_node.hpp

### class `surface_path_planning::RobotDescriptionNode`

Inherits from `rclcpp::Node`.

Helper node designed to latch onto the "/robot_description" topic.

Useful to block startup sequences until the URDF is fully published and available.

#### Public functions

##### `RobotDescriptionNode`

```cpp
RobotDescriptionNode()
```

Subscribe to /robot_description (transient local).

##### `isRobotDescriptionReceived`

```cpp
bool isRobotDescriptionReceived() const
```

True once the URDF has been received.

##### `getRobotDescription`

```cpp
std::string getRobotDescription() const
```

The received URDF (empty until received).

## include/surface_path_planning/surface_path_planning.hpp

Test node that plans with graph_core RRT* between configurations (used by simple_test_1/2).

### class `surface_path_planning::SurfacePlannerNode`

Inherits from `rclcpp::Node`.

Test node: sets up planning scene and graph_core plugins from parameters and plans a joint-space path with RRT*.

Used by the simple_test executables.

#### Public functions

##### `SurfacePlannerNode`

```cpp
SurfacePlannerNode(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
```

Constructor.

**Parameters:**

- `options`: Node options.

##### `executePlanning`

```cpp
bool executePlanning(const std::string &robot_desc_string)
```

Orchestrates setup, initializes plugins, maps environment parameters, and executes the core RRT planning pipeline loop.

##### `executePlanningCartesian`

```cpp
bool executePlanningCartesian(const std::string &robot_desc_string)
```

Same as `executePlanning()`, with the goal given as a Cartesian pose solved with the IK solver.

## include/surface_path_planning/utils.hpp

### class `surface_path_planning::utils::PlannerComponentsFactory`

Factory class responsible for dynamically loading and configuring graph_core plugins using cnr_class_loader.

#### Public functions

##### `PlannerComponentsFactory`

```cpp
PlannerComponentsFactory(cnr_class_loader::MultiLibraryClassLoader &loader, const rclcpp::Node::SharedPtr &node, const cnr_logger::TraceLoggerPtr &logger)
```

Construct a new Planner Components Factory.

**Parameters:**

- `loader`: Reference to the multi-library class loader (must outlive the plugins).
- `node`: Shared pointer to the calling ROS 2 node for logging and parameter access.
- `logger`: Pointer to the cnr_logger trace instance.

##### `createCollisionChecker`

```cpp
graph::core::CollisionCheckerPtr createCollisionChecker(const std::string &param_ns, const planning_scene::PlanningScenePtr &scene)
```

Dynamically instantiates and initializes the collision checker plugin.

##### `createSampler`

```cpp
graph::core::SamplerPtr createSampler(const std::string &param_ns, const Eigen::VectorXd &start_conf, const Eigen::VectorXd &goal_conf, const Eigen::VectorXd &lb, const Eigen::VectorXd &ub)
```

Dynamically instantiates and initializes the sampler plugin.

##### `createMetrics`

```cpp
graph::core::MetricsPtr createMetrics(const std::string &param_ns)
```

Dynamically instantiates and initializes the metrics (distance function) plugin.

### Free functions and types

#### `permutationName`

```cpp
bool permutationName(const std::vector< std::string > &order_names, std::vector< std::string > &names, std::vector< double > &position, std::vector< double > &velocity, std::vector< double > &effort, std::stringstream *report=nullptr)
```

Reorders joint arrays (names, position, velocity, effort) in-place to match a target ordering specified by order_names.

**Returns:** True if permutation succeeded, false otherwise (with optional diagnostic report).

## Scripts

### `scripts/send_raster_to_joint_path.py`

```
Example client of the compute_joint_path action.

1. Calls the compute_trajectory service of surface_raster_planner (a mesh must already be loaded).
2. Sends the resulting PoseArray and jump_indices to the compute_joint_path action.
3. Sends the joint path to the compute_joint_trajectory action (time_parametrization package, TOTG)
   to compute the timed trajectory.
4. Executes the trajectory with MoveIt (execute_trajectory action of move_group).
The compute_joint_path server publishes the path for Rviz on /compute_joint_path/display_path.

ROS parameters:
  scaling_factor (double, default 0.1): velocity and acceleration scaling factor, in (0, 1]
  group_name (string, default ur_robot): MoveIt planning group
  execute (bool, default true): false to stop after the time parametrization

Example:
  ros2 run surface_path_planning send_raster_to_joint_path.py --planning-frame open_tip \
      --row-step 0.02 --deviation 0 0 0.05 --ros-args -p scaling_factor:=0.2
```
