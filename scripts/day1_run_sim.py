"""
run_sim.py

Day 1 goal: load simple_leg_freefall.xml, step the simulation, and confirm
the leg actually falls under gravity (no actuators/tuned contacts yet,
so it's expected to flop around, not stand).
"""

import mujoco
import numpy as np

MODEL_PATH = "../models/simple_leg_freefall.xml"


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    print(f"Model loaded OK. Bodies: {model.nbody}, Joints: {model.njnt}, DOFs: {model.nv}")

    foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot")

    n_steps = 1000
    foot_heights = np.zeros(n_steps)

    for i in range(n_steps):
        mujoco.mj_step(model, data)
        foot_heights[i] = data.xpos[foot_body_id][2]

    print(f"Foot height at t=0.0s: {foot_heights[0]:.3f} m")
    print(f"Foot height at t=1.0s: {foot_heights[500]:.3f} m")
    print(f"Foot height at t=2.0s: {foot_heights[-1]:.3f} m")

    if foot_heights[-1] < foot_heights[0]:
        print("PASS: foot height decreased -- the leg is falling under gravity as expected.")
    else:
        print("UNEXPECTED: foot did not fall -- check joint/body definitions.")

    np.save("../results/day1_foot_heights.npy", foot_heights)
    print("Saved height trace to results/day1_foot_heights.npy")


if __name__ == "__main__":
    main()