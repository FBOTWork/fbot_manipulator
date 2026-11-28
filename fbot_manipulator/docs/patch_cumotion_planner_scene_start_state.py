"""Patch isaac_ros_cumotion's cumotion_planner.py (release-3.2, isaac-ros-cumotion==3.0.0) to use
the planning_scene_diff's robot_state as a fallback start state, before falling back to reading
the live /joint_states topic.

Why: MoveIt Task Constructor chains stages by computing each stage's start state from the
*previous* stage's end state (not yet executed -- MTC evaluates whole candidate solutions before
committing to execute any of them). That intermediate state is MoveIt's standard carrier for
this: the planning_scene_diff passed alongside the MotionPlanRequest (same as OMPL reads it from
the in-process PlanningScene object). plan_req.start_state itself is routinely empty by MoveIt
convention (OMPL ignores it too, in favor of the scene) -- that part isn't a bug. But
cumotion_planner.py, being an out-of-process GPU service reached over a ROS action, never looks
at the scene's robot_state at all: when start_state is empty it jumps straight to subscribing to
the live /joint_states topic, which only reflects the robot's *actual, already-executed* state --
not MTC's calculated intermediate one. Every stage past the first therefore planned from the
same stale starting pose instead of chaining, which is why execution visibly "jumped" between
stage trajectories instead of continuing smoothly.

Re-run after any reinstall/upgrade of ros-humble-isaac-ros-cumotion -- system file, not tracked
by this repo. See fbot_manipulator/docs/cuda_planning_setup.md.

    sudo python3 fbot_manipulator/docs/patch_cumotion_planner_scene_start_state.py
"""

path = "/opt/ros/humble/lib/python3.10/site-packages/isaac_ros_cumotion/cumotion_planner.py"
with open(path) as f:
    content = f.read()

old = """        world_objects = scene.world.collision_objects
        world_update_status = self.update_world_objects(world_objects)
        result = MoveGroup.Result()
"""
new = """        world_objects = scene.world.collision_objects
        world_update_status = self.update_world_objects(world_objects)
        result = MoveGroup.Result()

        # MoveIt Task Constructor chains stages via the planning_scene_diff's robot_state (same
        # place OMPL, running in-process, reads the current state from) -- use it as a fallback
        # before resorting to the live /joint_states topic, which only reflects already-executed
        # motion, not MTC's calculated-but-not-yet-executed intermediate state between stages.
        if (
            len(plan_req.start_state.joint_state.position) == 0
            and len(scene.robot_state.joint_state.position) > 0
        ):
            plan_req.start_state = scene.robot_state
"""
assert content.count(old) == 1, f"match count: {content.count(old)}"
content = content.replace(old, new)

with open(path, "w") as f:
    f.write(content)
print("patched OK")
