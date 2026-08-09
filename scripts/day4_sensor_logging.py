"""
day4_sensor_logging.py

Day 4 goal: log touch, hip position, and knee velocity sensors during a
run where the hip actuator swings to a target angle and the knee actuator
drives toward its limit. Plot all four signals together.
"""

import mujoco
import numpy as np
import matplotlib.pyplot as plt

MODEL_PATH = "../models/simple_leg.xml"


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    hip_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip_pos_actuator")
    knee_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee_vel_actuator")
    data.ctrl[hip_act] = 30
    data.ctrl[knee_act] = -30

    touch_addr = model.sensor("foot_touch_sensor").adr[0]
    hip_pos_addr = model.sensor("hip_pos_sensor").adr[0]
    knee_vel_addr = model.sensor("knee_vel_sensor").adr[0]

    n_steps = 1000
    touch_log = np.zeros(n_steps)
    hip_pos_log = np.zeros(n_steps)
    knee_vel_log = np.zeros(n_steps)

    foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot")
    foot_height_log = np.zeros(n_steps)

    for i in range(n_steps):
        mujoco.mj_step(model, data)
        touch_log[i] = data.sensordata[touch_addr]
        hip_pos_log[i] = np.rad2deg(data.sensordata[hip_pos_addr])
        knee_vel_log[i] = np.rad2deg(data.sensordata[knee_vel_addr])
        foot_height_log[i] = data.xpos[foot_id][2]

    t = np.arange(n_steps) * 0.002

    fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True)

    axes[0].plot(t, foot_height_log, color="tab:green")
    axes[0].set_ylabel("Foot height (m)")
    axes[0].set_title("Day 4: sensor logging during hip swing + knee drive")
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, touch_log, color="tab:red")
    axes[1].set_ylabel("Touch sensor")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t, hip_pos_log, color="tab:blue")
    axes[2].axhline(30, color="gray", linestyle="--", linewidth=1, label="target 30 deg")
    axes[2].set_ylabel("Hip pos (deg)")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)

    axes[3].plot(t, knee_vel_log, color="tab:orange")
    axes[3].set_ylabel("Knee vel (deg/s)")
    axes[3].set_xlabel("Time (s)")
    axes[3].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("../results/day4_sensor_logging.png", dpi=120)
    print("Saved plot to results/day4_sensor_logging.png")
    print(f"Touch sensor max value: {touch_log.max():.4f}")
    print(f"Hip settled near: {hip_pos_log[-1]:.1f} deg")
    print(f"Knee velocity at end: {knee_vel_log[-1]:.1f} deg/s")


if __name__ == "__main__":
    main()