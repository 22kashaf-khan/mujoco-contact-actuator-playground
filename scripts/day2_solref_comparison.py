"""
day2_solref_comparison.py

Day 2 goal: compare how different solref values (contact stiffness/damping)
affect foot-ground contact behavior, by dropping the same bent leg pose
with three different floor solref settings and plotting foot height over time.
"""

import mujoco
import numpy as np
import matplotlib.pyplot as plt

MODEL_PATH = "../models/simple_leg_freefall.xml"


def run_with_solref(model_path, solref_value, n_steps=1000):
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    model.geom_solref[floor_id] = solref_value

    hip_addr = model.joint("hip_joint").qposadr[0]
    knee_addr = model.joint("knee_joint").qposadr[0]
    data.qpos[hip_addr] = np.deg2rad(30)
    data.qpos[knee_addr] = np.deg2rad(30)
    mujoco.mj_forward(model, data)

    foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot")
    heights = np.zeros(n_steps)
    for i in range(n_steps):
        mujoco.mj_step(model, data)
        heights[i] = data.xpos[foot_id][2]

    return heights


def main():
    settings = {
        "default (0.02, 1)": [0.02, 1],
        "stiff (0.005, 1)": [0.005, 1],
        "soft (0.1, 1)": [0.1, 1],
    }

    n_steps = 1000
    t = np.arange(n_steps) * 0.002

    plt.figure(figsize=(9, 5))
    for label, solref in settings.items():
        heights = run_with_solref(MODEL_PATH, solref, n_steps)
        plt.plot(t, heights, label=label)
        print(f"{label}: min height = {heights.min():.4f} m, final height = {heights[-1]:.4f} m")

    plt.xlabel("Time (s)")
    plt.ylabel("Foot height (m)")
    plt.title("Day 2: foot height vs time for different solref (contact stiffness) values")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig("../results/day2_solref_comparison.png", dpi=120)
    print("Saved plot to results/day2_solref_comparison.png")


if __name__ == "__main__":
    main()