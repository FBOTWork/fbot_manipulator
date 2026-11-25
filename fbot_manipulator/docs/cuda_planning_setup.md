# GPU planning setup (NVIDIA Isaac ROS cuMotion), bare metal

Full install guide for `planner_backend:=cumotion` on `manipulator_interface_wx200.launch.py`,
without Docker, on this machine's actual setup: Pop!_OS 22.04, ROS 2 Humble, NVIDIA RTX 4050
(Ada Lovelace -- above the Ampere-or-newer requirement).

Read this fully before starting. Steps 3 (apt package availability) and 5 (XRDF) are the two
places this is known to go wrong or require judgment calls -- they are not one-liners you can
run blindly.

Commands below source `setup.zsh`, not `setup.bash` -- this machine's shell is zsh. Everything
else (apt, find, dpkg, ros2 CLI) is shell-agnostic.

## 0. Rebuild fbot_manipulator on this branch

The code already has the `mtc.planning_pipeline` switch (branch `cuda_planning`), but the
compiled binary in `install/` predates it. Nothing below works until this is rebuilt:

```bash
cd ~/arm_test_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select fbot_manipulator
source install/setup.zsh
```

## 1. Confirm the GPU is reachable (should already pass)

```bash
nvidia-smi
```

Expect to see the RTX 4050 listed. If this fails, stop -- nothing past this point will work,
and the fix is a driver issue unrelated to cuMotion.

No Docker / NVIDIA Container Toolkit step is needed for this path: cuMotion's action server
runs as a normal process on the host and uses the driver directly, the same way `nvidia-smi`
already does above.

## 2. Add the Isaac ROS apt repository (release-3, the Humble-targeted line)

```bash
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
```

```bash
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
```

`apt-key` is deprecated on newer Ubuntu/apt but still works on 22.04; ignore the deprecation
warning if one appears.

## 3. Check the package actually exists before installing (known to be flaky)

NVIDIA's own forums report this exact package disappearing from the release-3 apt feed at
times. Check first instead of jumping straight to `apt-get install`:

```bash
apt-cache search isaac-ros-cumotion
apt-cache policy ros-humble-isaac-ros-cumotion
```

- **Nothing listed / "Unable to locate package":** the repo is broken or the package was
  pulled. Don't fight it here -- see "If step 3 fails" below.
- **Something listed:** proceed.

```bash
sudo apt-get install -y ros-humble-isaac-ros-cumotion
```

Then find out whether the MoveIt plugin (`isaac_ros_cumotion_moveit`) shipped inside this
package or needs installing separately:

```bash
apt-cache search isaac-ros-cumotion-moveit
dpkg -L ros-humble-isaac-ros-cumotion | grep -i moveit
```

If `apt-cache search` shows a separate `ros-humble-isaac-ros-cumotion-moveit` (or similarly
named) package, install that too. The launch files in this branch load
`isaac_ros_cumotion_moveit/config/isaac_ros_cumotion_planning.yaml` -- confirm that file exists
somewhere under `/opt/ros/humble/share/isaac_ros_cumotion_moveit/` once installed:

```bash
find /opt/ros/humble/share -maxdepth 1 -iname "*cumotion*"
```

## 4. If the cuMotion action server won't start

cuMotion's planner backend (cuRobo) needs a CUDA-enabled PyTorch and a few other Python
packages. If `ros2 launch isaac_ros_cumotion ...` (or the combined launch in step 6) fails with
a Python import error rather than a ROS/pluginlib error, that's what's missing -- check the
actual traceback and install only what it names; don't pre-install things speculatively.

## 5. Generate the XRDF for the wx200 + gripper

There is no CLI command that converts a URDF straight into an XRDF. NVIDIA documents two ways:

- **Isaac Sim's Robot Description Editor** (GUI): load the wx200 URDF, build collision spheres
  per link, export via *Export to File > Export to cuMotion XRDF*. Isaac Sim itself is a large,
  separate install -- not covered here.
- **Manual, from a template** (what this guide assumes): copy a shipped example XRDF (e.g.
  `ur10e.xrdf`) and hand-edit the joint names, limits, and collision-sphere geometry to match
  the wx200 + gripper.

```bash
find /opt/ros/humble/share/isaac_ros_cumotion_robot_description -iname "*.xrdf" 2>/dev/null
```

Copy the closest match (a small 6-DOF arm, e.g. UR10e) to a working file, then edit:
- Joint/link names -> the wx200's (see `interbotix_xsarm_descriptions`'s URDF/xacro for the
  exact names: `waist`, `shoulder`, `elbow`, `wrist_angle`, `wrist_rotate`, `ee_gripper_link`,
  etc., depending on the exact model).
- Joint limits -> from the wx200 URDF / `interbotix_xsarm_moveit`'s joint_limits yaml.
- Collision spheres -> approximate the wx200's link geometry. Get this wrong and MTC either
  reports false collisions (never finds a plan) or lets the arm clip through itself/the table.

This step needs the actual wx200 URDF/xacro output open side by side -- bring it when you get
here and build the XRDF together rather than guessing values blind.

## 6. Launch with the GPU pipeline

```bash
source /opt/ros/humble/setup.zsh
source ~/arm_test_ws/install/setup.zsh

ros2 launch fbot_manipulator manipulator_interface_wx200.launch.py \
  planner_backend:=cumotion \
  cumotion_xrdf_file:=/path/to/wx200_gripper.xrdf \
  cumotion_urdf_file:=/path/to/wx200_gripper.urdf \
  cumotion_tool_frame:=ee_gripper_link
```

(Run the Interbotix MoveIt bringup in another terminal first, as usual -- see the main
[README.md](../../README.md) for that command.)

## 7. Confirm the GPU is actually doing the planning

In another terminal, while sending a manipulation goal:

```bash
nvidia-smi dmon -s u
```

- `GPU-Util` should spike during planning (idle is usually single digits on this machine).
- `manipulation_task_server`'s log should print `Planning pipeline: 'isaac_ros_cumotion'`
  (not `'ompl'`).
- `ros2 node list` should include the cuMotion action server node.
- Planning time (visible in the action feedback / server logs) should drop to low
  single-digit milliseconds, versus OMPL's tens-to-hundreds of ms.

If any of these don't hold -- GPU-Util stays flat, or the log still says `'ompl'` -- the
pipeline isn't actually switching, and the fix is in the launch/parameter wiring, not in cuMotion
itself.

## If step 3 fails (package not found / repo broken)

The apt route is NVIDIA's lighter-weight path but the less-supported one -- their primary,
better-maintained path is the Docker dev container (`isaac_ros_common`, `release-3.2` branch,
built from source). It avoids the apt flakiness entirely but requires:
- The NVIDIA Container Toolkit (`docker run --gpus all ...` working), and
- Cloning `fbot_manipulator` into the same colcon workspace as `isaac_ros_cumotion` inside the
  container, since the MoveIt plugin loads in-process with `manipulation_task_server` and both
  need to be in the same workspace overlay.

Ask for that version of this guide if step 3 dead-ends.
