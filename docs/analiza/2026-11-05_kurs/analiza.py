#!/usr/bin/env python3
"""Analiza testu trzymania kursu 2026-11-05 (bag 20261105_094555, eksport rov_bag export -> CSV).
  python3 docs/analiza/2026-11-05_kurs/analiza.py ~/rov_logs/kurs_csv docs/analiza/2026-11-05_kurs [tabela HH:MM:SS HH:MM:SS]"""
import sys
import datetime as dt
import numpy as np
import pandas as pd

D = sys.argv[1]
T = lambda s: dt.datetime.strptime(f"2026-11-05 {s}", "%Y-%m-%d %H:%M:%S.%f" if "." in s else "%Y-%m-%d %H:%M:%S").timestamp()  # noqa: E731  czas lokalny
L = lambda t: dt.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-4]  # noqa: E731


def load():
    c = pd.read_csv(f"{D}/rov__debug__ctrl.csv", low_memory=False).sort_values("t")
    m = pd.read_csv(f"{D}/rov__debug__mixer.csv", low_memory=False).sort_values("t")
    e = pd.read_csv(f"{D}/rov__esc__telemetry.csv", low_memory=False).sort_values("t")
    c = c.rename(columns={"rpy_2": "yaw", "sp.yaw_deg": "sp", "pid.yaw_0": "err", "pid.yaw_1": "P", "pid.yaw_2": "I", "pid.yaw_3": "D",
                          "pid.yaw_4": "u", "pid.yaw_5": "usat", "susp.yaw": "susp", "gyro_dps_2": "wz", "pilot_5": "pil_yaw", "out_5": "out_yaw"})
    m["thr_max"] = m[[f"thr_{i}" for i in range(8)]].abs().max(axis=1)
    e["rpm_max"] = e[[f"rpm_{i}" for i in range(8)]].max(axis=1)
    x = pd.merge_asof(c, m[["t", "armed", "src", "gain", "sat", "tau_5", "thr_max"] + [f"thr_{i}" for i in range(8)]], on="t", direction="nearest")
    x = pd.merge_asof(x, e[["t", "rpm_max"] + [f"rpm_{i}" for i in range(8)]], on="t", direction="nearest")
    return x


def table(x, a, b, step=4):
    s = x[(x.t >= T(a)) & (x.t <= T(b))].iloc[::step]
    print(f"{'czas':11s} arm {'src':8s} gain |   yaw     sp    err      P      D      u sus |  pil   out |    wz | tauN  thr  rpm")
    f = lambda v, w=6, p=1: f"{v:{w}.{p}f}" if pd.notna(v) else " " * (w - 1) + "-"  # noqa: E731
    for r in s.itertuples():
        print(f"{L(r.t):11s} {'A' if r.armed else '-':3s} {str(r.src):8s} {r.gain:4.2f} | {f(r.yaw)} {f(r.sp)} {f(r.err)} {f(r.P,6,3)} {f(r.D,6,3)} {f(r.u,6,3)} "
              f"{'Z' if str(r.susp) == 'True' else ' ':3s} | {f(r.pil_yaw,5,2)} {f(r.out_yaw,5,2)} | {f(r.wz)} | {f(r.tau_5,5,1)} {r.thr_max:4.2f} {r.rpm_max:5.0f}")


if __name__ == "__main__":
    x = load()
    if len(sys.argv) > 4 and sys.argv[3] == "tabela":
        table(x, sys.argv[4], sys.argv[5], int(sys.argv[6]) if len(sys.argv) > 6 else 4)


def plot(x, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    wins = [("10:07:36", "10:07:55", "Pierwsze ARM z trzymaniem kursu (gain 0.15)"), ("10:08:03", "10:10:14", "Gain 0.05, 2 minuty")]
    fig, axs = plt.subplots(4, 2, figsize=(18, 13), sharex="col")
    for col, (a, b, title) in enumerate(wins):
        S_ = x[(x.t >= T(a)) & (x.t <= T(b))]
        s = type("N", (), {"__getattr__": lambda self, k, S_=S_: S_[k].to_numpy(dtype=float)})()   # numpy dla matplotlib 3.5
        t = s.t - s.t[0]
        arm = s.armed.astype(bool)
        for ax in axs[:, col]:
            ax.fill_between(t, 0, 1, where=arm, transform=ax.get_xaxis_transform(), color="green", alpha=0.08, step="post")
            ax.grid(alpha=0.3)
        axs[0, col].plot(t, s.yaw, "k", lw=2, label="kurs (yaw) [deg], + = w lewo"); axs[0, col].plot(t, s.sp, "b--", label="zadany")
        axs[0, col].set_title(f"{title}  ({a}-{b}, zielone = ARM)", fontsize=10); axs[0, col].legend(fontsize=8)
        axs[1, col].plot(t, s.out_yaw, "r", lw=2, label="komenda yaw z regulatora (+ = skrec w lewo)")
        axs[1, col].plot(t, s.pil_yaw, "gray", label="galka pilota yaw"); axs[1, col].legend(fontsize=8); axs[1, col].set_ylim(-0.7, 0.7)
        axs[2, col].plot(t, -s.tau_5, "r", lw=2, label="moment z miksera -N [Nm] (+ = ma skrecac w LEWO)")
        axs[2, col].legend(fontsize=8, loc="upper left")
        ax2 = axs[2, col].twinx(); ax2.plot(t, s.wz, "k", lw=1.5, label="zyro wz [deg/s] (+ = obraca sie w LEWO)"); ax2.legend(fontsize=8, loc="upper right")
        axs[3, col].plot(t, s.rpm_max, "purple", label="max RPM pednikow"); axs[3, col].plot(t, s.thr_max * 4000, "orange", label="max |gaz| x4000")
        axs[3, col].legend(fontsize=8); axs[3, col].set_xlabel("t [s]")
    fig.suptitle("Trzymanie kursu 2026-11-05: moment 'w lewo' (czerwony) daje obrot w PRAWO (czarny) i odwrotnie -> dodatnie sprzezenie, ROV sie rozkreca",
                 fontsize=12)
    fig.tight_layout(); fig.savefig(f"{out}/kurs.png", dpi=80)
    print(f"{out}/kurs.png")


if __name__ == "__main__" and len(sys.argv) > 2 and (len(sys.argv) < 4 or sys.argv[3] != "tabela"):
    plot(load(), sys.argv[2])
