<img width="5848" height="719" alt="fbot_manipulator" src="https://github.com/user-attachments/assets/c73b64f3-05bf-47c7-bb67-3e38c7f4161a" />

## Overview
This is a group of ROS2 packages responsible for manipulation features of [FBOT@Work](https://fbotwork.vercel.app/) industrial robot (MICKY) in RoboCup@Work league.

## Architecture

```
fbot_manipulator/
├── 📁 include/                           #.hpp descriptiive files used by the package 
└── 📁 src/
    ├──  📁 mtc/                          # Mtc tasks
    |         ├── mtc_pick_and_place_task
    |         ├── mtc_shared_logic_task
    |         └── other tasks
    ├──  manipulator_interface_node/      # Interface node for manipulators
    └──  manipulation_task_server/        # Server to which the actions are requested
```

## Prerequisites
 - ROS2 Humble
 - Python 3.10+
 - Ubuntu 22.04
 - ROS dependencies are listed in `package.xml`.
## Installation
### 1. Clone Repository

```bash
cd ~/work_ws/src
git clone https://github.com/FBOTWork/fbot_manipulator.git
```
### 2. Install Dependencies
```bash
cd ~/work_ws
sudo rosdep init  # Skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
### 3. Build

```bash
cd ~/work_ws
colcon build --packages-select fbot_manipulator
source install/setup.bash
```

## Usage

### Example - Calling an action

We define the task thats going to be executed using a number code in the action, you can see more bellow:

### task type guide:
 
| ID | Task | 
| --- | --- | 
| 0 | Pick | 
| 1 | Place |
| 2 | Pick and place |
| 3 | Load cargo |
| 4 | Unload cargo |

#### 1. Launch the Interbotix MoveIt bringup (starts move_group + xs_sdk hardware interface)
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200 hardware_type:=actual
```
###### to use with only a simulated arm -> `hardware_type:=fake`
#### 2. Launch fbot_manipulator nodes (in a new terminal)
```bash
ros2 launch fbot_manipulator manipulator_interface_wx200.launch.py
```
#### 3. Call the action (in a new terminal)
```bash
ros2 action send_goal /fbot_manipulator/manipulation_task   fbot_manipulator_msgs/action/ManipulationTask   "{
    task_type: 4,
    target_id: 'target',
    cargo_index: 0,
    pick_offset: {x: 0.0, y: 0.0, z: 0.0},
    object_ids: ['target', 'obstaculo_1', 'obstaculo_2'],
    object_poses: [
      {position: {x: 0.3, y: 0.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 0.3, y: 0.15, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 0.3, y: -0.15, z: 0.0}, orientation: {w: 1.0}}
    ],
    object_sizes: [
      {x: 0.04, y: 0.04, z: 0.04},
      {x: 0.05, y: 0.05, z: 0.15},
      {x: 0.04, y: 0.08, z: 0.08}
    ],
    place_pose: {position: {x: 0.1, y: 0.2, z: 0.0}, orientation: {w: 1.0}},
    place_pose_names: ['pose_entrega_1']
  }"   --feedback
```
The number of items in the arrays can be modified as desired, the onlye requirement is that all arrays have the same lenght.

## GPU-accelerated planning (CUDA / NVIDIA Isaac ROS cuMotion)

By default the MTC task server plans on CPU through MoveIt's OMPL pipeline. It can instead
delegate planning to [NVIDIA Isaac ROS cuMotion](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion),
a CUDA-accelerated planner (built on cuRobo) exposed to MoveIt as a planning-pipeline plugin.
MTC itself has no GPU code -- `PipelinePlanner` just asks MoveIt's planning pipeline for a
trajectory, and this swaps which pipeline answers that call.

### What's already wired up (this branch)
 - `mtc.planning_pipeline` config value (default `"ompl"`), read by `MtcTask::setupSolvers()` and
   passed to `PipelinePlanner` instead of a hardcoded pipeline id.
 - A `planner_backend` arg on `manipulator_interface.launch.py` (`ompl` default, or `cumotion`),
   which also takes `cumotion_xrdf_file`, `cumotion_urdf_file` and `cumotion_tool_frame`.

### What's still required before `planner_backend:=cumotion` actually works
 1. Install NVIDIA Isaac ROS cuMotion (`isaac_ros_cumotion` + `isaac_ros_cumotion_moveit`) --
    this is a separate, GPU-dependent install (NVIDIA driver + CUDA, typically via the Isaac ROS
    docker dev environment); it is **not** a `fbot_manipulator` dependency and isn't installed by
    `rosdep install` here.
 2. Generate an XRDF robot description for the xarm6 + xarm_gripper combo (cuMotion's own robot
    format, built with `isaac_ros_cumotion_robot_description` from the URDF) and pass its path as
    `cumotion_xrdf_file`. None exists in this repo yet -- this is the main remaining piece of work.
 3. Launch with the new args, e.g.:
    ```bash
    ros2 launch fbot_manipulator manipulator_interface.launch.py \
      planner_backend:=cumotion \
      cumotion_xrdf_file:=/path/to/xarm6_gripper.xrdf \
      cumotion_urdf_file:=/path/to/xarm6_gripper.urdf \
      cumotion_tool_frame:=link_tcp
    ```

This has not been installed or run on real hardware yet -- treat it as plumbing ready for step 1/2
above, not as a validated feature.





## Development

### Creating a New Feature

1. Switch to the `release` branch (`git checkout release`)
2. Update local branch with `git fetch` then `git pull`
3. Create a feature branch (`git checkout -b feature/feature-name`)
   - Create feature file in `fbot_manipulator/src/mtc` if its an mtc task.
   - Create feature file in `fbot_manipulator/launch` if its a launch file.
4. Implement the feature
6. Update `package.xml` and/or `CMakeLists.txt` 
7. Test and verify that the feature is fully functional
8.  Commit changes (`git commit -m 'Add feature-name'`)
9.  Push the branch (`git push`)
10. Open a Pull Request from `feature/feature-name` to `release` and add a reviewer
11. After review and validation, merge the Pull Request into `release`
12. Once `release` is tested and stable, merge it into `master`

### Fixing a Feature

1. Switch to the `release` branch (`git checkout release`)
2. Update local branch with `git fetch` then `git pull`
3. Create a fix branch (`git checkout -b fix/broken-feature`)
4. Fix a feature
5. Commit changes (`git commit -m 'Fix amazing feature'`)
6. Push to the branch (`git push`)
7. Open a Pull Request from `fix/feature-name` to `release` and add a reviewer
8. After review and validation, merge the Pull Request into `release`
9. Once `release` is tested and stable, merge it into `master`

---
