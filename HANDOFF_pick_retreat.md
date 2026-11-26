# Handoff: pick from the front — retreat to approach + side walls

Branch `fix/pick_retreat_lateral_collision` (from `release`) in `src/fbot_manipulator`, commit `98fcac7`.
Not a deliverable of the repo: delete this file before merging.

## Problem
After grasping (pick from the front), going home the arm often planned a sideways motion (OMPL) and hit
what is beside the robot.

## What changed (all in `fbot_manipulator/src/mtc/mtc_shared_logic.cpp` unless noted)
1. `addPickStages`, front only (`approach_from_front`): new stage **"retreat to approach"** after the lift:
   cartesian, horizontal, away from the object (direction `-(cos yaw, sin yaw, 0)`, yaw = atan2 of the object
   in the arm frame), `approach_min..approach_max`, keeping the lift height. Then home / place as before.
   Top-down needs nothing: the lift already is the approach in reverse. Applies to every task that uses
   `addPickStages` (PICK, PICK_AND_PLACE(_SHAKE), LOAD_CARGO, UNLOAD_CARGO(_SHAKE)).
2. `addPickStages`, front only: new stage **"lower to grasp height"** before the approach: the approach pose
   is `mtc.front_approach_height` above the grasp (tag); the arm lowers exactly that behind the object,
   then goes in horizontally. Param in `MtcConfig` / `mtc_task.cpp`; `config/interbotix_arm/mtc_config.yaml`
   sets 0.035 (0 = old behavior).
3. `setupWorkspace`: collision boxes `robot_side_left` / `robot_side_right`, 0.30 long (same as
   `workspace_table`), 0.02 thick, 0.50 tall (center z 0.20), inner face 7 cm out from the base's sides
   (|y| = 0.15 + 0.07).

## Tested (MoveIt fake WX200 + RViz + fbot_manipulator, no Gazebo, micky_dev container)
- PICK front, cubes at (0.25-0.35, ±0.20, 0.10-0.15): all plan and execute; arm beside the base
  (x < 0.05) stays within |y| ≤ 0.13, inside the walls (0.22).
- Front pick needs the cube at z ≳ 0.10 in the arm frame (lower: no IK with the gripper horizontal, also
  before these changes). Low poses with the higher approach were NOT verified (test interrupted).
- LOAD_CARGO front -> slot 0: 3/3 succeeded (|y| ≤ 0.20 on the way to the cargo). Slots 1-3 not run.
- In fake hardware the grasp check reports "gripper closed empty" after execution (fingers close fully);
  this is expected there.

## Next: Gazebo arena, picking tags from the shelf with micky_behavior
- Build: `./scripts/build.sh --packages-select fbot_manipulator` (fbot_manipulator_msgs failed to build in
  the container with a setuptools `canonicalize_version` error; unchanged, so not needed).
- `ros2 launch micky_simulation sim_micky_full.launch.py`, `ros2 launch micky_behavior sim_bringup.launch.py
  rviz:=moveit`, then the shelf catch (approach_from_front is the fallback in `call_mtc.py` /
  `retrieve_object.py`; shelf catches in `micky_behavior/nodes/config/final_params.yaml`).
- Check in RViz: the side walls, the retreat after the lift, the 3.5 cm higher approach; the shelf above
  the cube limits the lift (lift_min 0.01) and the higher approach (3.5 cm must fit under the shelf above).

## Test helpers used (in the container under /tmp, not committed)
- `up.sh`: `xsarm_moveit.launch.py robot_model:=wx200 hardware_type:=fake use_moveit_rviz:=true`, then
  `manipulator_interface_wx200.launch.py robot_model:=wx200` (ROS_DOMAIN_ID=61).
- `reset.py`: arm to Sleep + gripper open through the fake controllers (the fake starts at 0, out of bounds).
- `pick.py x y z [front]` (`SLOT=n` -> LOAD_CARGO): sends the goal to `/fbot_manipulator/manipulation_task`
  and samples the links' TF to measure the lateral excursion.
