"""Patch curobo's helper_math.h (ros-humble-curobo-core, release-3.2) to stop defining a
float lerp(float,float,float) overload under C++20 -- it conflicts with std::lerp (added in
C++20's <cmath>) and breaks the JIT compile of curobo's CUDA extensions (kinematics_fused_cu
etc.) against a C++20-built PyTorch. std::lerp computes the exact same thing
(a + t*(b-a)), so disabling this overload under C++20 is a semantics-preserving fix, not a
behavior change. See fbot_manipulator/docs/cuda_planning_setup.md.

Re-run this after any reinstall/upgrade of ros-humble-curobo-core -- it's a system file, not
tracked by this repo, so package updates will revert it.

    sudo python3 fbot_manipulator/docs/patch_curobo_helper_math_lerp.py
"""

path = "/opt/ros/humble/lib/python3.10/site-packages/curobo/curobolib/cpp/helper_math.h"
with open(path) as f:
    content = f.read()

old = """inline __device__ __host__ float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}
"""
new = """#if __cplusplus < 202002L
inline __device__ __host__ float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}
#endif
"""
assert content.count(old) == 1, f"match count: {content.count(old)}"
content = content.replace(old, new)

with open(path, "w") as f:
    f.write(content)
print("patched OK")
