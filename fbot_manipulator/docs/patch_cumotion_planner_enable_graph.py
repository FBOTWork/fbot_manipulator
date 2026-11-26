"""Patch isaac_ros_cumotion's cumotion_planner.py (release-3.2, isaac-ros-cumotion==3.0.0) to
pass enable_graph=True to MotionGenPlanConfig on the plan_single_js call (added by
patch_cumotion_planner_js_target.py).

Why: MotionGenPlanConfig's own default is enable_graph=False -- enable_graph_attempt (which the
node does set) only controls how many *attempts* use the graph planner, but does nothing if
graph planning itself is off. Without it, plan_single_js only does local trajectory refinement
from a straight-line seed between start and goal, with no path search -- it reliably failed
(MotionGenStatus.TRAJOPT_FAIL) going from the wx200's folded "Sleep" start pose to a pick target,
because the direct interpolation between the two passes through a self-collision region partway
through, and there is no fallback to find a different path around it.

Apply this AFTER patch_cumotion_planner_js_target.py (it edits the code that patch adds).

Re-run after any reinstall/upgrade of ros-humble-isaac-ros-cumotion -- system file, not tracked
by this repo. See fbot_manipulator/docs/cuda_planning_setup.md.

    sudo python3 fbot_manipulator/docs/patch_cumotion_planner_enable_graph.py
"""

path = "/opt/ros/humble/lib/python3.10/site-packages/isaac_ros_cumotion/cumotion_planner.py"
with open(path) as f:
    content = f.read()

old = """            motion_gen_result = self.motion_gen.plan_single_js(
                start_state,
                goal_state,
                MotionGenPlanConfig(max_attempts=self.__max_attempts, enable_graph_attempt=1,
                                    time_dilation_factor=time_dilation_factor),
            )
"""
new = """            motion_gen_result = self.motion_gen.plan_single_js(
                start_state,
                goal_state,
                MotionGenPlanConfig(max_attempts=self.__max_attempts, enable_graph_attempt=1,
                                    enable_graph=True,
                                    time_dilation_factor=time_dilation_factor),
            )
"""
assert content.count(old) == 1, f"match count: {content.count(old)} -- run patch_cumotion_planner_js_target.py first"
content = content.replace(old, new)

with open(path, "w") as f:
    f.write(content)
print("patched OK")
