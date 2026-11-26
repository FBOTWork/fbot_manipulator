"""Patch isaac_ros_cumotion's cumotion_planner.py (release-3.2, isaac-ros-cumotion==3.0.0) to
plan joint-space to joint-space (plan_single_js) when the MoveGroup request already carries a
resolved joint target, instead of converting it to a Cartesian pose and re-deriving IK from
scratch via plan_single.

Why: MoveIt Task Constructor resolves IK itself before calling the planning pipeline, so by the
time this node sees the request it's often already a known-feasible joint-space goal (that's
why OMPL succeeds on it immediately). This node discarded that and asked cuRobo's full
IK+graph+trajopt pipeline to re-solve it, which was failing (MotionGenStatus.FINETUNE_TRAJOPT_FAIL)
on the wx200 (5 DOF, little null-space to re-optimize) even though a valid joint solution was
already known. See fbot_manipulator/docs/cuda_planning_setup.md.

Re-run this after any reinstall/upgrade of ros-humble-isaac-ros-cumotion -- it's a system file,
not tracked by this repo, so package updates will revert it.

    sudo python3 fbot_manipulator/docs/patch_cumotion_planner_js_target.py
"""

path = "/opt/ros/humble/lib/python3.10/site-packages/isaac_ros_cumotion/cumotion_planner.py"
with open(path) as f:
    content = f.read()

old1 = ("        if len(plan_req.goal_constraints[0].joint_constraints) > 0:\n"
        "            self.get_logger().info('Calculating goal pose from Joint target')\n")
new1 = ("        is_joint_target = len(plan_req.goal_constraints[0].joint_constraints) > 0\n"
        "        if is_joint_target:\n"
        "            self.get_logger().info('Calculating goal pose from Joint target')\n")
assert content.count(old1) == 1, f"old1 matches: {content.count(old1)}"
content = content.replace(old1, new1)

old2 = """        self.motion_gen.reset(reset_seed=False)
        motion_gen_result = self.motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(max_attempts=self.__max_attempts, enable_graph_attempt=1,
                                time_dilation_factor=time_dilation_factor),
        )
"""
new2 = """        self.motion_gen.reset(reset_seed=False)
        if is_joint_target:
            # Goal already came in as a resolved joint-space target (e.g. from MoveIt Task
            # Constructor, which does its own IK before calling the pipeline) -- plan
            # joint-space to joint-space directly instead of converting to a Cartesian pose
            # and re-deriving IK from scratch, which is needlessly harder for this planner on
            # low-DOF arms and was causing FINETUNE_TRAJOPT_FAIL even though a valid joint
            # solution was already known.
            motion_gen_result = self.motion_gen.plan_single_js(
                start_state,
                goal_state,
                MotionGenPlanConfig(max_attempts=self.__max_attempts, enable_graph_attempt=1,
                                    time_dilation_factor=time_dilation_factor),
            )
        else:
            motion_gen_result = self.motion_gen.plan_single(
                start_state,
                goal_pose,
                MotionGenPlanConfig(max_attempts=self.__max_attempts, enable_graph_attempt=1,
                                    time_dilation_factor=time_dilation_factor),
            )
"""
assert content.count(old2) == 1, f"old2 matches: {content.count(old2)}"
content = content.replace(old2, new2)

with open(path, "w") as f:
    f.write(content)
print("patched OK")
