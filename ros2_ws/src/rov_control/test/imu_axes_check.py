#!/usr/bin/env python3
"""Sprawdzenie mapowania osi IMU na pomiarze z pojazdu (CSV: t_s, acc_x/y/z czujnika, gyro_x/y/z_dps; srednie co 1 s).
Rysuje roll/pitch z akcelerometru dla starego i obecnego mapowania (config/attitude.yaml).
  python3 ros2_ws/src/rov_control/test/imu_axes_check.py docs/stabilizacja/imu_osie/pomiar_2026-11-05.csv docs/stabilizacja/imu_osie
Sekwencja ruchow przy pomiarze: plasko -> dziob w gore -> plasko -> prawa burta w dol -> plasko."""
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from rov_control.attitude_controller import axis_map, load_imu_rotation  # noqa: E402

csv, out = sys.argv[1], sys.argv[2]
d = np.genfromtxt(csv, delimiter=",", names=True)
acc = np.vstack([d["acc_x"], d["acc_y"], d["acc_z"]]).T
gyr = np.vstack([d["gyro_x_dps"], d["gyro_y_dps"], d["gyro_z_dps"]]).T
cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "attitude.yaml")
maps = [("STARE: imu_axes x,z,-y (bez trimu)", axis_map("x,z,-y")), ("OBECNE: config/attitude.yaml", load_imu_rotation(cfg))]


def rp(R):
    f = (R @ acc.T).T
    f /= np.linalg.norm(f, axis=1, keepdims=True)
    return np.degrees(np.arctan2(f[:, 1], f[:, 2])), np.degrees(np.arcsin(np.clip(f[:, 0], -1, 1)))


fig, axs = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
axs[0].plot(d["t_s"], acc, lw=1.5); axs[0].legend(["acc_x", "acc_y", "acc_z"], fontsize=8)
axs[0].set_ylabel("akcelerometr czujnika [m/s2]"); axs[0].set_title("Surowy pomiar (osie czujnika)")
for ax, (name, R) in zip(axs[1:], maps):
    roll, pitch = rp(R)
    ax.plot(d["t_s"], roll, "tab:red", lw=2, label="roll (+ prawa burta w dol)")
    ax.plot(d["t_s"], pitch, "tab:blue", lw=2, label="pitch (+ dziob w gore)")
    ax.set_ylabel("kat [deg]"); ax.set_title(name); ax.legend(fontsize=8); ax.set_ylim(-70, 70)
for ax in axs:
    ax.grid(alpha=0.3)
    for t0, t1, lab in ((29, 41, "DZIOB W GORE"), (45, 53, "PRAWA W DOL")):
        ax.axvspan(t0, t1, color="orange", alpha=0.15)
        if ax is axs[0]:
            ax.text((t0 + t1) / 2, ax.get_ylim()[1] * 0.9, lab, ha="center", fontsize=9)
axs[-1].set_xlabel("t [s]")
fig.suptitle("Mapowanie osi IMU: dobre = plasko 0/0, 'dziob w gore' tylko pitch +, 'prawa w dol' tylko roll +", fontsize=11)
fig.tight_layout(); os.makedirs(out, exist_ok=True); fig.savefig(os.path.join(out, "osie_imu.png"), dpi=85)
for name, R in maps:
    roll, pitch = rp(R)
    sel = lambda a, b: (d["t_s"] >= a) & (d["t_s"] <= b)  # noqa: E731
    print(f"{name}\n  plasko roll {roll[sel(2, 27)].mean():+.1f} pitch {pitch[sel(2, 27)].mean():+.1f} | "
          f"dziob w gore roll {roll[sel(35, 38)].mean():+.1f} pitch {pitch[sel(35, 38)].mean():+.1f} | "
          f"prawa w dol roll {roll[sel(47, 51)].mean():+.1f} pitch {pitch[sel(47, 51)].mean():+.1f}")
    for lab, a, b in (("zyro dziob w gore", 31, 32), ("zyro prawa w dol", 45, 45)):
        w = (R @ gyr[sel(a, b)].mean(0))
        print(f"  {lab}: w_FLU = {np.round(w, 1)}")
