<img width="5848" height="719" alt="fbot_manipulator" src="https://github.com/user-attachments/assets/c73b64f3-05bf-47c7-bb67-3e38c7f4161a" />

## Overview
This is a group of ROS2 packages responsible for manipulation features of [FBOT@Work](https://fbotwork.vercel.app/) industrial robot (MICKY) in RoboCup@Work league.

## Architecture

```
fbot_manipulator/
├── 📁 include/                           #.hpp descriptiive files used by the package 
└── 📁 src/
    ├──  📁 mtc/                          # Mtc tasks
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

```bash
# 1. Launch the Interbotix MoveIt bringup (starts move_group + xs_sdk hardware interface)
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200 hardware_type:=actual

# 2. Launch fbot_manipulator nodes (in a new terminal)
ros2 launch fbot_manipulator manipulator_interface_wx200.launch.py

# 3. call the action (in a new terminal)
ros2 action send_goal /fbot_manipulator/manipulation_task   fbot_manipulator_msgs/action/ManipulationTask   "{
    task_type: 4,
    target_id: 'cup',
    cargo_index: 0,
    pick_offset: {x: 0.0, y: 0.0, z: 0.0},
    object_ids: ['cup', 'obstaculo_1', 'obstaculo_2'],
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
    place_pose_name: ''
  }"   --feedback
```
The number of items in the arrays can be modified as desired, the onlye requirement is that all arrays have the same lenght.





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
