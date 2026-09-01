<img width="5848" height="719" alt="fbot_manipulator" src="https://github.com/user-attachments/assets/c73b64f3-05bf-47c7-bb67-3e38c7f4161a" />

## Overview
This is a group of ROS2 packages responsible for manipulation features of [FBOT@Work](https://fbotwork.vercel.app/) industrial robot (MICKY) in RoboCup@Work league.

## Architecture

```
fbot_manipulator/
├── 📁 manipulator_interface_node/      # Descrição do conteúdo da pasta
└── 📁 manipulation_task_server/        # Descrição do conteúdo da pasta
```

## Prerequisites

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
pip install -r src/micky_vision/requirements.txt
```
### 3. Build

```bash
cd ~/work_ws
colcon build --packages-select fbot_manipulator
source install/setup.bash
```

## Usage

### Example - Calling a service

```bash
# Move to home pose
ros2 service call /fbot_manipulator/move_to_named_target \
  fbot_manipulator_msgs/srv/MoveToNamedTarget "{target_name: 'home'}"

# Open gripper
ros2 service call /fbot_manipulator/set_gripper_position \
  fbot_manipulator_msgs/srv/MoveGripper "{position: 0.0}"
```

### Example -

```bash
# --- xArm6 (simulation) ---
# 1. Launch MoveIt with MTC support
ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py add_gripper:=true add_mtc:=true

# 2. Launch fbot_manipulator nodes (in a new terminal)
ros2 launch fbot_manipulator manipulator_interface.launch.py arm_type:=xarm6
```

```bash
# --- WidowX 200 / wx200 (real hardware) ---
# 1. Launch the Interbotix MoveIt bringup (starts move_group + xs_sdk hardware interface)
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200 hardware_type:=actual

# 2. Launch fbot_manipulator nodes (in a new terminal)
ros2 launch fbot_manipulator manipulator_interface_wx200.launch.py
```
## Config Parameters

| Parameter | Default | Description |
|-----------|:-------:|:-----------:|
| | | |
| | | |
| | | |
| | | |
| | | |
| | | |
| | | |
| | | |
| | | |

## Development

### Creating a New Feature

1. Switch to the `release` branch (`git checkout release`)
2. Update local branch with `git fetch` then `git pull`
3. Create a feature branch (`git checkout -b feature/feature-name`)
4. Create feature directory in `/`
5. Implement the feature
6. Update `__init__.py` imports
7. Add launch file in `launch/`
8. Add feature node to `setup.py`
9. Test and verify that the feature is fully functional
10. Commit changes (`git commit -m 'Add feature-name'`)
11. Push the branch (`git push`)
12. Open a Pull Request from `feature/feature-name` to `release` and add a reviewer
13. After review and validation, merge the Pull Request into `release`
14. Once `release` is tested and stable, merge it into `master`

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