"""Patch isaac_ros_cumotion's cumotion_planner.py (release-3.2, isaac-ros-cumotion==3.0.0) to
replace NaN joint velocities from the real /joint_states topic with 0.0 before using them as
the planning start state.

Why: this robot's joint_states publisher (fake controller, hardware_type:=fake) always reports
velocity as NaN (never measures/estimates it). cumotion_planner.py's js_callback stores that raw,
and when a PlanRequest has no explicit start state (the normal case here -- MTC leaves it
empty and expects "read the current state"), those NaN values get fed straight into the
trajectory optimizer's start-state velocity, which can silently corrupt a gradient-based solve
(TRAJOPT_FAIL) regardless of graph/seed settings -- consistent with every attempt failing the
same way in this setup, independent of target, buffer, or enable_graph fixes. Treating an
unknown/NaN velocity as "assume stationary" (0.0) is the standard, safe default. See
fbot_manipulator/docs/cuda_planning_setup.md.

Re-run after any reinstall/upgrade of ros-humble-isaac-ros-cumotion -- system file, not tracked
by this repo.

    sudo python3 fbot_manipulator/docs/patch_cumotion_planner_nan_velocity.py
"""

path = "/opt/ros/humble/lib/python3.10/site-packages/isaac_ros_cumotion/cumotion_planner.py"
with open(path) as f:
    content = f.read()

old = """    def js_callback(self, msg):
        self.__js_buffer = {
            'joint_names': msg.name,
            'position': msg.position,
            'velocity': msg.velocity,
        }
"""
new = """    def js_callback(self, msg):
        # Some joint_states publishers (e.g. MoveIt's fake controller) never estimate velocity
        # and always report NaN -- treat unknown velocity as stationary (0.0) rather than
        # feeding NaN into the trajectory optimizer's start state.
        velocity = [0.0 if v != v else v for v in msg.velocity]
        self.__js_buffer = {
            'joint_names': msg.name,
            'position': msg.position,
            'velocity': velocity,
        }
"""
assert content.count(old) == 1, f"match count: {content.count(old)}"
content = content.replace(old, new)

with open(path, "w") as f:
    f.write(content)
print("patched OK")
