# GPU planning setup (NVIDIA Isaac ROS cuMotion), bare metal

Full install guide for `planner_backend:=cumotion` on `manipulator_interface_wx200.launch.py`,
without Docker, on this machine's actual setup: Pop!_OS 22.04, ROS 2 Humble, NVIDIA RTX 4050
(Ada Lovelace -- above the Ampere-or-newer requirement).

Read this fully before starting. Step 3 (apt package availability) is where this is known to go
wrong -- not a one-liner you can run blindly. Step 5's XRDF is already generated in this branch
(not a placeholder), but its collision geometry is an approximation worth validating in RViz
before trusting it on hardware.

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

## 5. XRDF for the wx200 + gripper -- done

There is no CLI command that converts a URDF straight into an XRDF (the two documented ways are
Isaac Sim's Robot Description Editor GUI, or hand-authoring from a template -- not a one-liner).
Both files already exist in this branch, generated from the real wx200 geometry (link bounding
boxes parsed directly from the `.stl` meshes, joint limits copied from
`interbotix_xsarm_moveit`'s `wx200_joint_limits.yaml`), not guessed:

- [`config/interbotix_arm/wx200_gripper.xrdf`](../config/interbotix_arm/wx200_gripper.xrdf)
- [`config/interbotix_arm/wx200_gripper.urdf`](../config/interbotix_arm/wx200_gripper.urdf)
  (the exact URDF `manipulator_interface_wx200.launch.py` builds internally, with
  `robot_name:=wx200` -- that's why every link is `wx200/<link>`)

Read the comment block at the top of the `.xrdf` -- it lists exactly which values are measured
(collision geometry, acceleration limits) versus placeholders that still need validating
(`default_joint_positions`, `cspace.jerk_limits`). Collision spheres are a cautious
approximation: watch for false-positive self-collisions (never finds a plan) or, worse, spheres
too small (arm clips through itself/the table) once you see real plans in RViz.

## 6. Launch with the GPU pipeline

```bash
source /opt/ros/humble/setup.zsh
source ~/arm_test_ws/install/setup.zsh

ros2 launch fbot_manipulator manipulator_interface_wx200.launch.py \
  planner_backend:=cumotion \
  cumotion_xrdf_file:=$(ros2 pkg prefix fbot_manipulator)/share/fbot_manipulator/config/interbotix_arm/wx200_gripper.xrdf \
  cumotion_urdf_file:=$(ros2 pkg prefix fbot_manipulator)/share/fbot_manipulator/config/interbotix_arm/wx200_gripper.urdf \
  cumotion_tool_frame:=wx200/ee_gripper_link
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
