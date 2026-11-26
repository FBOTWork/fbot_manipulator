# GPU planning setup (NVIDIA Isaac ROS cuMotion), bare metal

Full install guide for `planner_backend:=cumotion` on `manipulator_interface_wx200.launch.py`,
without Docker, on this machine's actual setup: Pop!_OS 22.04, ROS 2 Humble, NVIDIA RTX 4050
(Ada Lovelace), driver 580.x (CUDA 13 capable).

This has been run end-to-end on this machine. Every step below reflects what was actually
needed, not a plan -- including the two source patches in step 7, which are not optional.

Commands below source `setup.zsh`, not `setup.bash` -- this machine's shell is zsh. Everything
else (apt, find, dpkg, pip, ros2 CLI) is shell-agnostic.

## 0. Rebuild fbot_manipulator on this branch

```bash
cd ~/arm_test_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select fbot_manipulator
source install/setup.zsh
```

(`--symlink-install` matters here -- this workspace's other packages were built that way, and
mixing build modes across packages causes `ament_cmake_python` symlink conflicts on a later
full-workspace build. If you hit `failed to create symbolic link '...' because existing path
cannot be removed: Is a directory` on some package, `rm -rf build/<pkg> install/<pkg>` for the
affected packages and rebuild with `--symlink-install`.)

## 1. Confirm the GPU is reachable (should already pass)

```bash
nvidia-smi
```

No Docker / NVIDIA Container Toolkit needed for this path -- everything below runs as normal
host processes using the driver directly.

## 2. Isaac ROS apt repository + packages

```bash
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
```

Two separate packages are needed -- `ros-humble-isaac-ros-cumotion` does **not** include the
MoveIt plugin:

```bash
sudo apt-get install -y ros-humble-isaac-ros-cumotion ros-humble-isaac-ros-cumotion-moveit
```

If `apt-get update`/`install` 404s or can't find the package: NVIDIA's own forums report this
exact repo going flaky. See "If step 2 fails" at the bottom before troubleshooting further.

## 3. A modern CUDA Toolkit (the Ubuntu-packaged one is too old)

Ubuntu 22.04's own `nvidia-cuda-toolkit` ships `nvcc` 11.5 (from 2021) -- too old to compile
against a current PyTorch (needs `-std=c++20`, which 11.5 rejects outright). Install a current
one from NVIDIA's own repo, side by side, without touching the display driver:

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
```

This needs ~10GB free (3.3GB download + ~6.5GB installed) -- check `df -h /` first; if short,
`sudo apt-get clean` and `pip cache purge` are safe, reversible ways to free space.

```bash
sudo apt-get install -y cuda-toolkit-13-0
```

`cuda-toolkit-13-0` only (not the `cuda` meta-package) -- it skips the driver component, so it
can't conflict with the one already working. Then point the shell at it:

```bash
export PATH=/usr/local/cuda-13.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH
```

(Put these two lines wherever you normally export PATH for this workspace -- every terminal
that launches `cumotion_planner` needs them.)

## 4. Python dependencies (pip, user site-packages)

None of these are declared as rosdep/apt dependencies of the packages above -- they only show
up as `ModuleNotFoundError` / `ImportError` the first time `cumotion_planner` actually runs:

```bash
pip install --user warp-lang ninja trimesh nvidia-cuda-runtime-cu12
```

- `warp-lang`: NVIDIA's GPU kernel library, what cuRobo's math runs on.
- `ninja`: needed the first time cuRobo JIT-compiles its CUDA extensions against your installed
  PyTorch (the apt-shipped precompiled ones are ABI-incompatible with a recent PyTorch -- JIT
  recompiling against your actual PyTorch headers is the real fix, not a fallback).
- `trimesh`: mesh handling, used by cuRobo's collision geometry loader.
- `nvidia-cuda-runtime-cu12`: provides `libcudart.so.12` in Python's own site-packages, needed
  because this install didn't otherwise have a CUDA 12 runtime anywhere (only 11.5 and 13.x).
  Add its lib dir too:
  ```bash
  export LD_LIBRARY_PATH=$HOME/.local/lib/python3.10/site-packages/nvidia/cuda_runtime/lib:$LD_LIBRARY_PATH
  ```

If `cumotion_planner` still dies with a Python traceback after this, read the actual
`ModuleNotFoundError`/`ImportError` line and install only what it names -- don't pre-install
speculatively.

## 5. XRDF for the wx200 + gripper -- done, validated against two real poses

There is no CLI command that converts a URDF straight into an XRDF. Both files already exist in
this branch, generated from the real wx200 geometry (mesh bounding boxes parsed from the `.stl`
files, joint limits from `interbotix_xsarm_moveit`'s `wx200_joint_limits.yaml`):

- [`config/interbotix_arm/wx200_gripper.xrdf`](../config/interbotix_arm/wx200_gripper.xrdf)
- [`config/interbotix_arm/wx200_gripper.urdf`](../config/interbotix_arm/wx200_gripper.urdf)

The collision-sphere ignore list was validated with forward-kinematics math (not RViz, not
guessed) against two *real* robot poses: the zero joint state, and the factory **Sleep** pose
(SRDF group_state: `shoulder=-1.88, elbow=1.55, wrist_angle=0.8`) -- the second one matters
because it's the robot's actual resting pose and folds the forearm/wrist/gripper back close to
the shoulder, which the original sphere sizing flagged as a false self-collision. Read the
comment block at the top of the `.xrdf` for what's still a placeholder
(`default_joint_positions`, and `jerk_limits`, bumped to 500 after 50 caused
`TRAJOPT_FAIL` on an otherwise-feasible move).

Still worth a visual RViz sanity check across more poses before trusting this on real hardware
-- the validation above covers two specific configurations, not the whole joint-limit range.

## 6. Launch the standalone cuMotion planner node

This is the piece that's easy to miss: the MoveIt plugin that loads inside
`manipulation_task_server` (`isaac_ros_cumotion_moveit/CumotionPlanner`) is only a thin
forwarder -- the actual GPU planning happens in a **separate node**
(`cumotion_planner`, package `isaac_ros_cumotion`, executable `cumotion_goal_set_planner_node`)
that it calls over a ROS 2 action. Nothing works until this is running too, in its own
terminal:

```bash
ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.urdf_path:=$(ros2 pkg prefix fbot_manipulator)/share/fbot_manipulator/config/interbotix_arm/wx200_gripper.urdf \
  cumotion_planner.yml_file_path:=$(ros2 pkg prefix fbot_manipulator)/share/fbot_manipulator/config/interbotix_arm/wx200_gripper.xrdf \
  cumotion_planner.tool_frame:=wx200/ee_gripper_link \
  cumotion_planner.read_esdf_world:=false \
  cumotion_planner.joint_states_topic:=/wx200/joint_states
```

First launch JIT-compiles several CUDA extensions (`kinematics_fused_cu`, `geom_cu`,
`tensor_step_cu`, `lbfgs_step_cu`, `line_search_cu`) -- takes a minute or two; later launches
use the cache and start in a few seconds. Wait for `cuMotion is ready for planning queries!`
before moving on.

**This node needs step 7's patches to actually succeed on this robot** (not just start) --
without them it reliably reports `INVALID_START_STATE_*_COLLISION` (fixed by the XRDF
work above), then `IK_FAIL`/`FINETUNE_TRAJOPT_FAIL` (fixed by patch 2 below) once collision was
no longer the blocker.

## 7. Two source patches (apply once per install/upgrade of these apt packages)

Both patch system files owned by `root`, outside this repo -- `sudo` is required, and a
reinstall or version bump of the underlying apt package reverts them. Scripts are kept here so
they're not lost:

**7a. cuRobo's `helper_math.h` -- C++20 `lerp` conflict.** cuRobo's own `lerp(float,float,float)`
collides with `std::lerp` (added in C++20's `<cmath>`) once nvcc actually supports
`-std=c++20` (which it only does from the CUDA 13 toolkit in step 3 -- this is why this didn't
show up until after that install). `std::lerp` computes the exact same thing, so disabling
cuRobo's own overload under C++20 is a semantics-preserving fix:

```bash
sudo python3 fbot_manipulator/docs/patch_curobo_helper_math_lerp.py
```

**7b. `isaac_ros_cumotion`'s `cumotion_planner.py` -- re-deriving IK it doesn't need.** MoveIt
Task Constructor resolves IK itself before calling the planning pipeline, so the request this
node receives is usually already a known-feasible joint-space target (that's why OMPL succeeds
on the exact same goal immediately). This node discarded that and converted it to a Cartesian
pose, asking cuRobo's full IK+graph+trajopt pipeline to re-solve it from scratch -- which failed
(`FINETUNE_TRAJOPT_FAIL`) on the wx200's 5 DOF (little null-space margin to re-optimize in) even
though a valid joint solution was already known. The patch makes it call `plan_single_js`
(joint-space to joint-space, no IK) when the request is already a joint target:

```bash
sudo python3 fbot_manipulator/docs/patch_cumotion_planner_js_target.py
```

Re-launch the node from step 6 after applying these (Ctrl+C it first -- it only reads these
files at startup).

## 8. Launch fbot_manipulator with the GPU pipeline

In a third terminal (alongside the Interbotix MoveIt bringup and the node from step 6):

```bash
source /opt/ros/humble/setup.zsh
source ~/arm_test_ws/install/setup.zsh

ros2 launch fbot_manipulator manipulator_interface_wx200.launch.py \
  planner_backend:=cumotion \
  cumotion_xrdf_file:=$(ros2 pkg prefix fbot_manipulator)/share/fbot_manipulator/config/interbotix_arm/wx200_gripper.xrdf \
  cumotion_urdf_file:=$(ros2 pkg prefix fbot_manipulator)/share/fbot_manipulator/config/interbotix_arm/wx200_gripper.urdf \
  cumotion_tool_frame:=wx200/ee_gripper_link
```

(Run `ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200
hardware_type:=fake` first, as usual -- see the main [README.md](../../README.md).)

## 9. Confirm the GPU is actually doing the planning

While sending a manipulation goal:

- `manipulation_task_server`'s log should print `Planning pipeline: 'isaac_ros_cumotion'`.
- `cumotion_planner`'s own log (step 6's terminal) is the source of truth for whether planning
  actually succeeded -- `manipulation_task_server` has shown `Success` with `No trajectory`
  even when `cumotion_planner` logged a real failure, so don't trust "Success" on its own without
  checking that terminal too.
- `nvidia-smi dmon -s u` should show `GPU-Util` spike above its idle baseline (single digits on
  this machine) during planning.
- Planning time, once it succeeds, should be noticeably faster than OMPL's (OMPL on this task
  plans in well under a second already, for context -- the wx200 is a simple, low-DOF arm, so
  don't expect a dramatic difference the way you would on a cluttered scene with a 6-7 DOF arm).

## If step 2 fails (package not found / repo broken)

The apt route is NVIDIA's lighter-weight path but the less-supported one -- their primary,
better-maintained path is the Docker dev container (`isaac_ros_common`, `release-3.2` branch,
built from source). It avoids the apt flakiness entirely but requires the NVIDIA Container
Toolkit and cloning `fbot_manipulator` into the same colcon workspace as `isaac_ros_cumotion`
inside the container (the MoveIt plugin loads in-process with `manipulation_task_server`, so
both need to be in the same workspace overlay). Ask for that version of this guide if step 2
dead-ends.
